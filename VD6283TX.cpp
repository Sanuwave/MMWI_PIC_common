#include "VD6283TX.h"
#include <cmath>
#include <iostream>
#include <cstring>
#include <cassert>

// =============================================================================
// Register map  (DS13735 Rev 3, Table 10 / Section 4.3)
// =============================================================================
namespace Reg {
    static constexpr uint8_t DEVICE_ID         = 0x00;
    static constexpr uint8_t REVISION_ID       = 0x01;
    static constexpr uint8_t INTERRUPT_CTRL    = 0x02;
    static constexpr uint8_t ALS_CTRL          = 0x03;
    static constexpr uint8_t ALS_PERIOD        = 0x04;

    static constexpr uint8_t ALS_CH1_DATA_H   = 0x06;
    static constexpr uint8_t ALS_CH2_DATA_H   = 0x0A;
    static constexpr uint8_t ALS_CH3_DATA_H   = 0x0E;
    static constexpr uint8_t ALS_CH4_DATA_H   = 0x12;
    static constexpr uint8_t ALS_CH5_DATA_H   = 0x16;
    static constexpr uint8_t ALS_CH6_DATA_H   = 0x1A;

    static constexpr uint8_t ALS_EXPOSURE_M   = 0x1D;
    static constexpr uint8_t ALS_EXPOSURE_L   = 0x1E;

    static constexpr uint8_t ALS_GAIN_CH1     = 0x25;
    static constexpr uint8_t ALS_GAIN_CH2     = 0x26;
    static constexpr uint8_t ALS_GAIN_CH3     = 0x27;
    static constexpr uint8_t ALS_GAIN_CH4     = 0x28;
    static constexpr uint8_t ALS_GAIN_CH5     = 0x29;
    static constexpr uint8_t ALS_GAIN_CH6     = 0x2A;

    static constexpr uint8_t CHANNEL6_ENABLE  = 0x2D;
    static constexpr uint8_t ALS_CHANNEL_ENABLE = 0x2E;

    static constexpr uint8_t AC_MODE_CTRL     = 0x31;
    static constexpr uint8_t PEDESTAL_VALUE   = 0x32;

    static constexpr uint8_t SDA_DRV_CFG     = 0x3C;
    static constexpr uint8_t GPIO1_DRV_CFG   = 0x41;
}

// =============================================================================
// Register bit fields
// =============================================================================
namespace Bits {
    static constexpr uint8_t INTR_ST             = (1u << 1);
    static constexpr uint8_t CLR_INTR            = (1u << 0);

    static constexpr uint8_t ALS_EN              = (1u << 0);
    static constexpr uint8_t ALS_CONT_SHIFT      = 1u;

    static constexpr uint8_t CH6_EN              = (1u << 0);

    static constexpr uint8_t PDM_CLK_SEL_BIT     = (1u << 5);
    static constexpr uint8_t AC_OUT_SEL_BIT      = (1u << 4);
    static constexpr uint8_t AC_CH_SEL_SHIFT     = 1u;
    static constexpr uint8_t AC_FREQEXT_EN_BIT   = (1u << 0);

    static constexpr uint8_t SDA_LOAD_BIT        = (1u << 3);
    static constexpr uint8_t SDA_DRV_DEFAULT     = SDA_LOAD_BIT | 0x01u;
}

static constexpr uint8_t  PEDESTAL_RECOMMENDED = 3u;
static constexpr uint16_t EXTIME_DEFAULT_REG   = 0x31;

// =============================================================================
// Singleton interface
// =============================================================================
VD6283TX& VD6283TX::getInstance()
{
    static VD6283TX instance;
    return instance;
}

// =============================================================================
// Private default constructor
// =============================================================================
VD6283TX::VD6283TX()
    : m_addr(DEFAULT_I2C_ADDR)
    , m_extimeReg(EXTIME_DEFAULT_REG)
    , m_initialized(false)
{
}

// =============================================================================
// Lifecycle
// =============================================================================
bool VD6283TX::init(uint8_t addr)
{
    m_addr = addr;

    uint8_t id = 0;
    if (!getDeviceId(id)) {
        std::cerr << "VD6283TX: failed to read DEVICE_ID\n";
        return false;
    }
    if (id != DEVICE_ID_VALUE) {
        std::cerr << "VD6283TX: unexpected DEVICE_ID 0x"
                  << std::hex << static_cast<int>(id) << std::dec << '\n';
        return false;
    }

    if (!writeReg(Reg::PEDESTAL_VALUE, PEDESTAL_RECOMMENDED))                       return false;
    if (!writeReg(Reg::SDA_DRV_CFG,    Bits::SDA_DRV_DEFAULT))                      return false;
    if (!writeReg(Reg::GPIO1_DRV_CFG,  static_cast<uint8_t>(Gpio1Cfg::OPEN_DRAIN))) return false;
    if (!writeReg(Reg::ALS_CTRL,       0x00))                                       return false;
    if (!clearInterrupt())                                                           return false;

    m_extimeReg = EXTIME_DEFAULT_REG;
    if (!writeReg(Reg::ALS_EXPOSURE_M, static_cast<uint8_t>((m_extimeReg >> 8) & 0x03))) return false;
    if (!writeReg(Reg::ALS_EXPOSURE_L, static_cast<uint8_t>( m_extimeReg       & 0xFF))) return false;

    m_initialized = true;
    std::cout << "VD6283TX: init OK (DEVICE_ID=0x"
              << std::hex << static_cast<int>(id) << std::dec << ")\n";
    return true;
}

void VD6283TX::deinit()
{
    if (!m_initialized) return;

    stopALS();
    stopFlicker();

    writeReg(Reg::ALS_CHANNEL_ENABLE, 0x00);
    writeReg(Reg::CHANNEL6_ENABLE,    0x00);

    m_initialized = false;
}

// =============================================================================
// Device identification
// =============================================================================
bool VD6283TX::getDeviceId(uint8_t& id) const   { return readReg(Reg::DEVICE_ID,   id); }
bool VD6283TX::getRevisionId(uint8_t& rev) const { return readReg(Reg::REVISION_ID, rev); }

// =============================================================================
// Channel and gain configuration
// =============================================================================
bool VD6283TX::enableChannels(uint8_t mask)
{
    if (!writeReg(Reg::ALS_CHANNEL_ENABLE, mask & 0x1Fu))                           return false;
    if (!writeReg(Reg::CHANNEL6_ENABLE, (mask & CH_CLEAR) ? Bits::CH6_EN : 0x00u)) return false;
    return true;
}

bool VD6283TX::setGain(Channel ch, Gain gain)
{
    return writeReg(gainRegForChannel(ch), static_cast<uint8_t>(gain) & 0x0Fu);
}

bool VD6283TX::setGainAll(Gain gain)
{
    const uint8_t val = static_cast<uint8_t>(gain) & 0x0Fu;
    const uint8_t gainRegs[] = {
        Reg::ALS_GAIN_CH1, Reg::ALS_GAIN_CH2, Reg::ALS_GAIN_CH3,
        Reg::ALS_GAIN_CH4, Reg::ALS_GAIN_CH5, Reg::ALS_GAIN_CH6
    };
    for (auto reg : gainRegs) {
        if (!writeReg(reg, val)) return false;
    }
    return true;
}

// =============================================================================
// Exposure and timing
// =============================================================================
bool VD6283TX::setExposureMs(float ms)
{
    if (ms < EXTIME_STEP_MS) ms = EXTIME_STEP_MS;

    uint16_t reg = static_cast<uint16_t>(std::roundf(ms / EXTIME_STEP_MS) - 1.0f);
    if (reg > EXTIME_REG_MAX) reg = EXTIME_REG_MAX;

    if (!writeReg(Reg::ALS_EXPOSURE_M, static_cast<uint8_t>((reg >> 8) & 0x03u))) return false;
    if (!writeReg(Reg::ALS_EXPOSURE_L, static_cast<uint8_t>( reg       & 0xFFu))) return false;

    m_extimeReg = reg;
    return true;
}

float VD6283TX::getExposureMs() const
{
    return static_cast<float>(m_extimeReg + 1) * EXTIME_STEP_MS;
}

bool VD6283TX::setIntermeasurementPeriod(uint8_t steps)
{
    return writeReg(Reg::ALS_PERIOD, steps);
}

// =============================================================================
// ALS operation
// =============================================================================
bool VD6283TX::startALS(AlsMode mode)
{
    uint8_t val = static_cast<uint8_t>(
        (static_cast<uint8_t>(mode) << Bits::ALS_CONT_SHIFT) | Bits::ALS_EN);
    return writeReg(Reg::ALS_CTRL, val);
}

bool VD6283TX::stopALS()
{
    return writeReg(Reg::ALS_CTRL, 0x00);
}

// =============================================================================
// Data readout
// =============================================================================
bool VD6283TX::isDataReady() const
{
    uint8_t ctrl = 0;
    if (!readReg(Reg::INTERRUPT_CTRL, ctrl)) return false;
    return (ctrl & Bits::INTR_ST) != 0;
}

bool VD6283TX::clearInterrupt()
{
    if (!writeReg(Reg::INTERRUPT_CTRL, Bits::CLR_INTR)) return false;
    if (!writeReg(Reg::INTERRUPT_CTRL, 0x00))           return false;
    return true;
}

bool VD6283TX::readAllChannels(ChannelData& data)
{
    static constexpr size_t  BUF_LEN   = 23u;
    static constexpr uint8_t START_REG = Reg::ALS_CH1_DATA_H;

    uint8_t buf[BUF_LEN];
    if (!readRegs(START_REG, buf, BUF_LEN)) return false;

    auto assemble = [&](size_t off) -> uint32_t {
        return (static_cast<uint32_t>(buf[off])     << 16)
             | (static_cast<uint32_t>(buf[off + 1]) <<  8)
             |  static_cast<uint32_t>(buf[off + 2]);
    };

    data.red     = assemble(0);
    data.visible = assemble(4);
    data.blue    = assemble(8);
    data.green   = assemble(12);
    data.ir      = assemble(16);
    data.clear   = assemble(20);
    return true;
}

bool VD6283TX::readChannel(Channel ch, uint32_t& counts)
{
    uint8_t buf[3];
    if (!readRegs(dataRegForChannel(ch), buf, 3)) return false;
    counts = (static_cast<uint32_t>(buf[0]) << 16)
           | (static_cast<uint32_t>(buf[1]) <<  8)
           |  static_cast<uint32_t>(buf[2]);
    return true;
}

// =============================================================================
// Saturation
// =============================================================================
uint32_t VD6283TX::getSaturationLimit() const
{
    if (m_extimeReg <= EXTIME_SAT_BOUNDARY)
        return static_cast<uint32_t>(SAT_SHORT_PER_STEP) * (m_extimeReg + 1u);
    return static_cast<uint32_t>(SAT_LONG);
}

bool VD6283TX::isSaturated(uint16_t count16) const
{
    return static_cast<uint32_t>(count16) >= getSaturationLimit();
}

// =============================================================================
// Flicker detection
// =============================================================================
bool VD6283TX::startFlicker(Channel ch, FlickerMode mode)
{
    if (!writeReg(Reg::PEDESTAL_VALUE, PEDESTAL_RECOMMENDED)) return false;

    uint8_t acCtrl = Bits::AC_FREQEXT_EN_BIT
                   | static_cast<uint8_t>(flickerChSel(ch) << Bits::AC_CH_SEL_SHIFT);

    switch (mode) {
        case FlickerMode::INTERNAL_GPIO1: break;
        case FlickerMode::INTERNAL_GPIO2: acCtrl |= Bits::AC_OUT_SEL_BIT;  break;
        case FlickerMode::EXTERNAL_CLK:   acCtrl |= Bits::PDM_CLK_SEL_BIT; break;
    }

    if (!writeReg(Reg::AC_MODE_CTRL, acCtrl)) return false;

    if (mode != FlickerMode::INTERNAL_GPIO2) {
        if (!setGpio1Config(Gpio1Cfg::PUSH_PULL)) return false;
    }
    return true;
}

bool VD6283TX::stopFlicker()
{
    uint8_t cur = 0;
    if (!readReg(Reg::AC_MODE_CTRL, cur)) return false;
    cur &= static_cast<uint8_t>(~Bits::AC_FREQEXT_EN_BIT);
    return writeReg(Reg::AC_MODE_CTRL, cur);
}

bool VD6283TX::setFlickerPedestalValue(uint8_t val)
{
    return writeReg(Reg::PEDESTAL_VALUE, val & 0x07u);
}

// =============================================================================
// GPIO / driver configuration
// =============================================================================
bool VD6283TX::setGpio1Config(Gpio1Cfg cfg)
{
    return writeReg(Reg::GPIO1_DRV_CFG, static_cast<uint8_t>(cfg) & 0x03u);
}

// =============================================================================
// Private helpers
// =============================================================================
bool VD6283TX::writeReg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return I2cMgr::getInstance().write(m_addr, buf, sizeof(buf));
}

bool VD6283TX::readReg(uint8_t reg, uint8_t& data) const
{
    return I2cMgr::getInstance().writeRead(m_addr, &reg, 1, &data, 1);
}

bool VD6283TX::readRegs(uint8_t startReg, uint8_t* buf, size_t len) const
{
    return I2cMgr::getInstance().writeRead(m_addr, &startReg, 1, buf, len);
}

uint8_t VD6283TX::gainRegForChannel(Channel ch)
{
    switch (ch) {
        case Channel::RED:     return Reg::ALS_GAIN_CH1;
        case Channel::VISIBLE: return Reg::ALS_GAIN_CH2;
        case Channel::BLUE:    return Reg::ALS_GAIN_CH3;
        case Channel::GREEN:   return Reg::ALS_GAIN_CH4;
        case Channel::IR:      return Reg::ALS_GAIN_CH5;
        case Channel::CLEAR:   return Reg::ALS_GAIN_CH6;
        default:               return Reg::ALS_GAIN_CH1;
    }
}

uint8_t VD6283TX::dataRegForChannel(Channel ch)
{
    switch (ch) {
        case Channel::RED:     return Reg::ALS_CH1_DATA_H;
        case Channel::VISIBLE: return Reg::ALS_CH2_DATA_H;
        case Channel::BLUE:    return Reg::ALS_CH3_DATA_H;
        case Channel::GREEN:   return Reg::ALS_CH4_DATA_H;
        case Channel::IR:      return Reg::ALS_CH5_DATA_H;
        case Channel::CLEAR:   return Reg::ALS_CH6_DATA_H;
        default:               return Reg::ALS_CH1_DATA_H;
    }
}

uint8_t VD6283TX::flickerChSel(Channel ch)
{
    switch (ch) {
        case Channel::CLEAR:   return 0x01;
        case Channel::RED:     return 0x02;
        case Channel::VISIBLE: return 0x03;
        case Channel::BLUE:    return 0x04;
        case Channel::GREEN:   return 0x05;
        case Channel::IR:      return 0x06;
        default:               return 0x01;
    }
}