#include "AS7331.h"
#include "I2cMgr.h"

#include <iostream>
#include <cmath>
#include <unistd.h>   // usleep

// Sensitivity table definition (nW·s/cm² per count) – matches header declaration
constexpr float AS7331::SENSITIVITY_UV[3];

// -----------------------------------------------------------------------------
// Singleton
// -----------------------------------------------------------------------------

AS7331& AS7331::getInstance() {
    static AS7331 instance;
    return instance;
}

AS7331::AS7331()
    : m_addr(DEFAULT_I2C_ADDR)
    , m_gain(Gain::GAIN_64)
    , m_intTime(IntTime::TIME_64MS)
    , m_measMode(MeasMode::CMD)
    , m_initialised(false) {
}

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------

bool AS7331::init(uint8_t i2cAddr, Gain gain, IntTime intTime, MeasMode mode) {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_addr     = i2cAddr;
    m_gain     = gain;
    m_intTime  = intTime;
    m_measMode = mode;

    I2cMgr& i2c = I2cMgr::getInstance();
    if (!i2c.isOpen()) {
        if (!i2c.open()) {
            std::cerr << "[AS7331] Failed to open I2C bus" << std::endl;
            return false;
        }
    }

    // Soft-reset: write OSR_SW_RES (0x0A = SW_RES|DOS_CONF) per datasheet Figure 46.
    // Using 0x08 (SW_RES alone) leaves DOS=000b which is a NOP and the reset won't take.
    if (!writeReg(REG_OSR, OSR_SW_RES)) {
        std::cerr << "[AS7331] Software reset failed" << std::endl;
        return false;
    }

    // After reset the OSR should read 0x02 (DOS=010b CONF, PD=0, SS=0, SW_RES reads 0).
    // We mask off the PD bit since board power state may vary.
    uint8_t osr = 0;
    if (!readReg(REG_OSR, osr)) {
        std::cerr << "[AS7331] Failed to read OSR reg" << std::endl;
        return false;
    }
    if ((osr & ~OSR_PD) != DEFAULT_OSR_REG) {
        std::cerr << "[AS7331] Unexpected OSR reg: 0x"
                  << std::hex << static_cast<int>(osr) << std::dec << std::endl;
        return false;
    }

    if (!enterConfState())  return false;
    if (!applyConfig())     return false;

    m_initialised = true;
    std::cout << "[AS7331] Initialized at 0x"
              << std::hex << static_cast<int>(m_addr) << std::dec << std::endl;
    return true;
}

void AS7331::deinit() {
    std::lock_guard<std::mutex> lock(m_mutex);
    // Power down the device (PD=1, DOS=CONF → 0x42)
    writeReg(REG_OSR, OSR_PD | OSR_DOS_CONF);
    m_initialised = false;
}

bool AS7331::isInitialized() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialised;
}

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

bool AS7331::setGain(Gain gain) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gain = gain;
    return m_initialised ? applyConfig() : true;
}

bool AS7331::setIntegrationTime(IntTime t) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_intTime = t;
    return m_initialised ? applyConfig() : true;
}

bool AS7331::setMeasurementMode(MeasMode mode) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_measMode = mode;
    return m_initialised ? applyConfig() : true;
}

bool AS7331::softwareReset() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!writeReg(REG_OSR, OSR_SW_RES)) return false;
    usleep(1000);
    m_initialised = false;
    return true;
}

// -----------------------------------------------------------------------------
// Measurement
// -----------------------------------------------------------------------------

AS7331::UvData AS7331::measure() {
    std::lock_guard<std::mutex> lock(m_mutex);
    UvData result{};

    if (!m_initialised) {
        std::cerr << "[AS7331] measure() called before init()" << std::endl;
        return result;
    }

    // Must be in CONF state before triggering
    if (!enterConfState())  return result;
    if (!applyConfig())     return result;

    // Switch to MEAS state and set SS=1 to start conversion (0x83 per datasheet Fig 46)
    uint8_t osr = OSR_SS | OSR_DOS_MEAS;
    if (!writeReg(REG_OSR, osr)) return result;

    // Calculate worst-case conversion time (ms) and add 20% margin
    uint32_t intMs   = (1u << static_cast<uint8_t>(m_intTime));
    uint32_t timeout = intMs + (intMs / 5) + 10u;

    if (!waitForDataReady(timeout)) {
        std::cerr << "[AS7331] Measurement timeout" << std::endl;
        return result;
    }

    return readResults();
}

AS7331::UvData AS7331::readResults() {
    UvData result{};

    uint16_t temp = 0, uva = 0, uvb = 0, uvc = 0;

    if (!readWord(REG_TEMP_LSB, temp)) return result;  // addr 0x01 = TEMP
    if (!readWord(REG_UVA_LSB,  uva))  return result;  // addr 0x02 = MRES1
    if (!readWord(REG_UVB_LSB,  uvb))  return result;  // addr 0x03 = MRES2
    if (!readWord(REG_UVC_LSB,  uvc))  return result;  // addr 0x04 = MRES3

    // Temperature conversion: T(°C) = raw × 0.05 − 66.9  (AS7331 datasheet §7.4)
    result.tempC = static_cast<float>(temp) * 0.05f - 66.9f;
    result.uva   = uva;
    result.uvb   = uvb;
    result.uvc   = uvc;
    result.valid = true;

    return result;
}

// -----------------------------------------------------------------------------
// Irradiance per channel conversion and channel sensitiveity constants
// -----------------------------------------------------------------------------

float AS7331::rawToIrradiance(uint16_t raw, uint8_t channel) {
    if (channel > 2) return 0.0f;

    // Gain register value 0x00 = 2048x, 0x0B = 1x.
    // Actual gain factor = 2048 >> reg_value
    float gainFactor = static_cast<float>(2048u >> static_cast<uint8_t>(m_gain));
    float timeSec    = static_cast<float>(1u << static_cast<uint8_t>(m_intTime)) / 1000.0f;
    return (static_cast<float>(raw) * SENSITIVITY_UV[channel]) / (gainFactor * timeSec);
}

// -----------------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------------

bool AS7331::writeReg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return I2cMgr::getInstance().write(m_addr, buf, sizeof(buf));
}

bool AS7331::readReg(uint8_t reg, uint8_t& val) {
    return I2cMgr::getInstance().writeRead(m_addr, &reg, 1, &val, 1);
}

bool AS7331::readWord(uint8_t lsbReg, uint16_t& val) {
    uint8_t buf[2] = {};
    if (!I2cMgr::getInstance().writeRead(m_addr, &lsbReg, 1, buf, 2)) return false;
    val = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    return true;
}

bool AS7331::enterConfState() {
    return writeReg(REG_OSR, OSR_DOS_CONF);
}

bool AS7331::enterMeasState() {
    return writeReg(REG_OSR, OSR_DOS_MEAS);
}

bool AS7331::applyConfig() {
    // CREG1: GAIN[7:4] | TIME[3:0]
    uint8_t creg1 = (static_cast<uint8_t>(m_gain) << 4) |
                    (static_cast<uint8_t>(m_intTime) & 0x0F);
    if (!writeReg(REG_CREG1, creg1)) return false;

    // CREG2: EN_TM=1 (bit6, enables temperature measurement), EN_DIV=0, DIV=0
    if (!writeReg(REG_CREG2, 0x40)) return false;

    // CREG3: MMODE[7:6] | RDYOD[3] – MMODE shift is 6 per datasheet Figure 50
    uint8_t creg3 = (static_cast<uint8_t>(m_measMode) << CREG3_MMODE_SHIFT)
                  | CREG3_RDYOD;
    if (!writeReg(REG_CREG3, creg3)) return false;

    return true;
}

bool AS7331::waitForDataReady(uint32_t timeoutMs) {
    // Poll STATUS register bit 3 (NDATA) – set when new data is available.
    // Note: reading STATUS resets NDATA, so a single poll per iteration is correct.
    constexpr uint32_t POLL_INTERVAL_US = 1000u; // 1 ms
    uint32_t elapsed = 0;

    while (elapsed < timeoutMs) {
        uint16_t statusOsr = 0;
        if (!readWord(REG_STATUS, statusOsr)) return false;
        if ((statusOsr >> 8) & STATUS_NDATA) return true;
        usleep(POLL_INTERVAL_US);
        elapsed++;
    }
    return false;
}