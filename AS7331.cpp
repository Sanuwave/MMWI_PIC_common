#include "AS7331.h"
#include "I2cMgr.h"

#include <iostream>
#include <cmath>
#include <unistd.h>   // usleep

// -----------------------------------------------------------------------------
// FSR lookup tables (µW/cm²) per channel A/B/C, indexed by GAIN register value
// (0x00 = 2048x highest gain/lowest FSR ... 0x0B = 1x lowest gain/highest FSR)
// Specified at CREG1:TIME = 0x0A (1024 ms), CREG3:CCLK = 00b (1 MHz)
// Source: AS7331 datasheet Table Figure 48
// -----------------------------------------------------------------------------
static constexpr float FSR_A[12] = {
      9.75f,   19.50f,   39.00f,   78.00f,
    156.00f,  312.00f,  624.00f, 1248.00f,
   2496.00f, 4992.00f, 9984.00f, 19968.00f
};
static constexpr float FSR_B[12] = {
     12.75f,   25.50f,   51.00f,  102.00f,
    204.00f,  408.00f,  816.00f, 1632.00f,
   3264.00f, 6528.00f, 13056.00f, 26112.00f
};
static constexpr float FSR_C[12] = {
      6.13f,   12.25f,   24.50f,   49.00f,
     98.00f,  196.00f,  392.00f,  784.00f,
   1568.00f, 3136.00f, 6272.00f, 12544.00f
};

// Reference integration time at which FSR values are defined (ms @ 1 MHz)
static constexpr float T_REF_MS = 1024.0f;

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
    if (!writeReg(REG_OSR, OSR_SW_RES)) {
        std::cerr << "[AS7331] Software reset failed" << std::endl;
        return false;
    }

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

    if (!enterConfState())  return result;
    if (!applyConfig())     return result;

    // Switch to MEAS state and trigger (SS=1, DOS=MEAS → 0x83, per datasheet Fig 46)
    if (!writeReg(REG_OSR, OSR_SS | OSR_DOS_MEAS)) return result;

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

    if (!readWord(REG_TEMP_LSB, temp)) return result;
    if (!readWord(REG_UVA_LSB,  uva))  return result;
    if (!readWord(REG_UVB_LSB,  uvb))  return result;
    if (!readWord(REG_UVC_LSB,  uvc))  return result;

    // Temperature: T(°C) = raw × 0.05 − 66.9  (AS7331 datasheet §7.4)
    result.tempC = static_cast<float>(temp) * 0.05f - 66.9f;
    result.uva   = uva;
    result.uvb   = uvb;
    result.uvc   = uvc;
    result.valid = true;

    return result;
}

// -----------------------------------------------------------------------------
// Irradiance conversion  (µW/cm²)
//
// Formula derived from AS7331 datasheet Figure 48:
//
//   FSR values are specified at TIME = 0x0A (1024 ms) and CCLK = 00b (1 MHz).
//   For any other integration time the measurement range scales inversely:
//
//     Ee = (raw / 65535) × FSR[gain][ch] × (T_REF_MS / tconv_ms)
//
//   Where tconv_ms = 2^TIME_index ms  (at 1 MHz; CCLK divides this further
//   but we fix CCLK = 00b in applyConfig, so no additional scale needed).
//
//   EN_DIV is not used (applyConfig sets EN_DIV=0), so div_factor = 1.
// -----------------------------------------------------------------------------

float AS7331::rawToIrradiance(uint16_t raw, uint8_t channel) const {
    if (channel > 2) return 0.0f;

    uint8_t gainIdx = static_cast<uint8_t>(m_gain);
    if (gainIdx > 11) gainIdx = 11;

    // Select FSR for this channel
    float fsr = 0.0f;
    switch (channel) {
        case 0:  fsr = FSR_A[gainIdx]; break;
        case 1:  fsr = FSR_B[gainIdx]; break;
        default: fsr = FSR_C[gainIdx]; break;
    }

    // Integration time in ms: tconv = 2^TIME_index
    // TIME index 15 wraps back to 1 ms (same as index 0) per datasheet Table Figure 48
    uint8_t tIdx  = static_cast<uint8_t>(m_intTime);
    uint8_t tExp  = (tIdx == 15u) ? 0u : tIdx;
    float tconvMs = static_cast<float>(1u << tExp);   // ms at 1 MHz (CCLK=00b)

    // Ee (µW/cm²) = (raw / 65535) × FSR × (T_REF / tconv)
    return (static_cast<float>(raw) / 65535.0f) * fsr * (T_REF_MS / tconvMs);
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

    // CREG2: EN_TM=1 (bit6), EN_DIV=0, DIV=0
    // EN_DIV disabled – rawToIrradiance assumes no hardware divider
    if (!writeReg(REG_CREG2, 0x40)) return false;

    // CREG3: MMODE[7:6] | RDYOD[3] – CCLK fixed at 00b (1 MHz)
    uint8_t creg3 = (static_cast<uint8_t>(m_measMode) << CREG3_MMODE_SHIFT)
                  | CREG3_RDYOD;
    if (!writeReg(REG_CREG3, creg3)) return false;

    return true;
}

bool AS7331::waitForDataReady(uint32_t timeoutMs) {
    constexpr uint32_t POLL_INTERVAL_US = 1000u;
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