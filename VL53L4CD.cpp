#include "VL53L4CD.h"
#include "VL53L4CD_platform.h"
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Default sensor configuration (registers 0x2D–0x87)
// ---------------------------------------------------------------------------

const uint8_t VL53L4CD::DEFAULT_CONFIGURATION[] = {
#ifdef VL53L4CD_I2C_FAST_MODE_PLUS
    0x12, /* 0x2D : Fast Mode Plus (1 MHz I2C) */
#else
    0x00, /* 0x2D */
#endif
    0x00, /* 0x2E */ 0x00, /* 0x2F */ 0x11, /* 0x30 */ 0x02, /* 0x31 */
    0x00, /* 0x32 */ 0x02, /* 0x33 */ 0x08, /* 0x34 */ 0x00, /* 0x35 */
    0x08, /* 0x36 */ 0x10, /* 0x37 */ 0x01, /* 0x38 */ 0x01, /* 0x39 */
    0x00, /* 0x3A */ 0x00, /* 0x3B */ 0x00, /* 0x3C */ 0x00, /* 0x3D */
    0xFF, /* 0x3E */ 0x00, /* 0x3F */ 0x0F, /* 0x40 */ 0x00, /* 0x41 */
    0x00, /* 0x42 */ 0x00, /* 0x43 */ 0x00, /* 0x44 */ 0x00, /* 0x45 */
    0x20, /* 0x46 */ 0x0B, /* 0x47 */ 0x00, /* 0x48 */ 0x00, /* 0x49 */
    0x02, /* 0x4A */ 0x14, /* 0x4B */ 0x21, /* 0x4C */ 0x00, /* 0x4D */
    0x00, /* 0x4E */ 0x05, /* 0x4F */ 0x00, /* 0x50 */ 0x00, /* 0x51 */
    0x00, /* 0x52 */ 0x00, /* 0x53 */ 0xC8, /* 0x54 */ 0x00, /* 0x55 */
    0x00, /* 0x56 */ 0x38, /* 0x57 */ 0xFF, /* 0x58 */ 0x01, /* 0x59 */
    0x00, /* 0x5A */ 0x08, /* 0x5B */ 0x00, /* 0x5C */ 0x00, /* 0x5D */
    0x01, /* 0x5E */ 0xCC, /* 0x5F */ 0x07, /* 0x60 */ 0x01, /* 0x61 */
    0xF1, /* 0x62 */ 0x05, /* 0x63 */ 0x00, /* 0x64 : Sigma thresh MSB */
    0xA0, /* 0x65 : Sigma thresh LSB */ 0x00, /* 0x66 : Signal thresh MSB */
    0x80, /* 0x67 : Signal thresh LSB */ 0x08, /* 0x68 */ 0x38, /* 0x69 */
    0x00, /* 0x6A */ 0x00, /* 0x6B */ 0x00, /* 0x6C : Intermeas MSB */
    0x00, /* 0x6D */ 0x0F, /* 0x6E */ 0x89, /* 0x6F : Intermeas LSB */
    0x00, /* 0x70 */ 0x00, /* 0x71 */ 0x00, /* 0x72 : Dist thresh high MSB */
    0x00, /* 0x73 */ 0x00, /* 0x74 : Dist thresh low MSB */ 0x00, /* 0x75 */
    0x00, /* 0x76 */ 0x01, /* 0x77 */ 0x07, /* 0x78 */ 0x05, /* 0x79 */
    0x06, /* 0x7A */ 0x06, /* 0x7B */ 0x00, /* 0x7C */ 0x00, /* 0x7D */
    0x02, /* 0x7E */ 0xC7, /* 0x7F */ 0xFF, /* 0x80 */ 0x9B, /* 0x81 */
    0x00, /* 0x82 */ 0x00, /* 0x83 */ 0x00, /* 0x84 */ 0x01, /* 0x85 */
    0x00, /* 0x86 : clear interrupt */ 0x00  /* 0x87 : start ranging */
};

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

VL53L4CD::VL53L4CD(uint8_t i2c_address) : addr_(i2c_address) {}

// ---------------------------------------------------------------------------
// Platform I/O helpers
// ---------------------------------------------------------------------------

static std::string fmtReg(uint16_t reg) {
    char buf[7];
    snprintf(buf, sizeof(buf), "0x%04X", reg);
    return buf;
}

void VL53L4CD::wrByte(uint16_t reg, uint8_t val) const {
    if (VL53L4CD_WrByte(static_cast<Dev_t>(addr_), reg, val) != 0)
        throw std::runtime_error("VL53L4CD: WrByte failed at " + fmtReg(reg));
}

void VL53L4CD::wrWord(uint16_t reg, uint16_t val) const {
    if (VL53L4CD_WrWord(static_cast<Dev_t>(addr_), reg, val) != 0)
        throw std::runtime_error("VL53L4CD: WrWord failed at " + fmtReg(reg));
}

void VL53L4CD::wrDWord(uint16_t reg, uint32_t val) const {
    if (VL53L4CD_WrDWord(static_cast<Dev_t>(addr_), reg, val) != 0)
        throw std::runtime_error("VL53L4CD: WrDWord failed at " + fmtReg(reg));
}

uint8_t VL53L4CD::rdByte(uint16_t reg) const {
    uint8_t val = 0;
    if (VL53L4CD_RdByte(static_cast<Dev_t>(addr_), reg, &val) != 0)
        throw std::runtime_error("VL53L4CD: RdByte failed at " + fmtReg(reg));
    return val;
}

uint16_t VL53L4CD::rdWord(uint16_t reg) const {
    uint16_t val = 0;
    if (VL53L4CD_RdWord(static_cast<Dev_t>(addr_), reg, &val) != 0)
        throw std::runtime_error("VL53L4CD: RdWord failed at " + fmtReg(reg));
    return val;
}

uint32_t VL53L4CD::rdDWord(uint16_t reg) const {
    uint32_t val = 0;
    if (VL53L4CD_RdDWord(static_cast<Dev_t>(addr_), reg, &val) != 0)
        throw std::runtime_error("VL53L4CD: RdDWord failed at " + fmtReg(reg));
    return val;
}

void VL53L4CD::waitMs(uint32_t ms) const {
    usleep(ms * 1000u);
}

void VL53L4CD::waitForDataReady(uint32_t timeout_ms) const {
    for (uint32_t i = 0; i < timeout_ms; ++i) {
        if (isDataReady()) return;
        waitMs(1);
    }
    throw std::runtime_error("VL53L4CD: timeout waiting for data ready");
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void VL53L4CD::sensorInit() {
    // Wait for firmware boot (status == 0x03)
    uint32_t i = 0;
    while (rdByte(REG_FIRMWARE_STATUS) != 0x03) {
        if (++i > 1000)
            throw std::runtime_error("VL53L4CD: boot timeout");
        waitMs(1);
    }

    // Load default configuration
    for (uint8_t addr = 0x2D; addr <= 0x87; ++addr)
        wrByte(addr, DEFAULT_CONFIGURATION[addr - 0x2D]);

    // VHV init measurement
    wrByte(REG_SYSTEM_START, 0x40);
    waitForDataReady(1000);
    clearInterrupt();
    stopRanging();

    // Post-VHV tweaks (from ST reference)
    wrByte(REG_VHV_CONFIG_TIMEOUT, 0x09);
    wrByte(0x000B, 0x00);
    wrWord(0x0024, 0x0500);

    setRangeTiming(50, 0);
}

// ---------------------------------------------------------------------------
// Ranging
// ---------------------------------------------------------------------------

void VL53L4CD::startRanging() {
    // Continuous mode if inter-measurement == 0, autonomous otherwise
    wrByte(REG_SYSTEM_START, rdDWord(REG_INTERMEASUREMENT) == 0u ? 0x21 : 0x40);
}

void VL53L4CD::stopRanging() {
    wrByte(REG_SYSTEM_START, 0x80);
}

bool VL53L4CD::isDataReady() const {
    uint8_t mux  = rdByte(REG_GPIO_HV_MUX_CTRL);
    uint8_t pol  = ((mux & 0x10u) >> 4) == 1u ? 0u : 1u;
    uint8_t stat = rdByte(REG_GPIO_TIO_HV_STATUS);
    return (stat & 0x01u) == pol;
}

void VL53L4CD::clearInterrupt() {
    wrByte(REG_INTERRUPT_CLEAR, 0x01);
}

VL53L4CD::Results VL53L4CD::getResult() const {
    static constexpr uint8_t STATUS_RTN[24] = {
        255, 255, 255,  5,  2,  4,  1,  7,  3,
          0, 255, 255,  9, 13, 255, 255, 255, 255, 10,  6,
        255, 255,  11, 12
    };

    Results r{};

    uint8_t raw_status = rdByte(REG_RESULT_RANGE_STATUS) & 0x1Fu;
    r.range_status = (raw_status < 24) ? STATUS_RTN[raw_status] : raw_status;

    uint16_t raw_spads  = rdWord(REG_RESULT_SPAD_NB);
    r.number_of_spad    = raw_spads / 256u;
    r.signal_rate_kcps  = static_cast<uint32_t>(rdWord(REG_RESULT_SIGNAL_RATE))   * 8u;
    r.ambient_rate_kcps = static_cast<uint32_t>(rdWord(REG_RESULT_AMBIENT_RATE))  * 8u;
    r.sigma_mm          = rdWord(REG_RESULT_SIGMA) / 4u;
    r.distance_mm       = rdWord(REG_RESULT_DISTANCE);

    r.signal_per_spad_kcps  = r.signal_rate_kcps  * 256u / raw_spads;
    r.ambient_per_spad_kcps = r.ambient_rate_kcps * 256u / raw_spads;

    return r;
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

void VL53L4CD::setRangeTiming(uint32_t timing_budget_ms, uint32_t inter_measurement_ms) {
    if (timing_budget_ms < 10u || timing_budget_ms > 200u)
        throw std::runtime_error("VL53L4CD: timing budget must be 10–200 ms");

    uint16_t osc_freq = rdWord(REG_OSC_FREQUENCY);
    if (osc_freq == 0)
        throw std::runtime_error("VL53L4CD: invalid oscillator frequency");

    uint32_t budget_us      = timing_budget_ms * 1000u;
    uint32_t macro_period   = (2304u * (0x40000000u / osc_freq)) >> 6;

    if (inter_measurement_ms == 0u) {
        wrDWord(REG_INTERMEASUREMENT, 0);
        budget_us -= 2500u;
    } else if (inter_measurement_ms > timing_budget_ms) {
        uint16_t clock_pll = rdWord(REG_OSC_CALIBRATE_VAL) & 0x3FFu;
        float    factor    = 1.055f * static_cast<float>(inter_measurement_ms)
                                    * static_cast<float>(clock_pll);
        wrDWord(REG_INTERMEASUREMENT, static_cast<uint32_t>(factor));
        budget_us -= 4300u;
        budget_us /= 2u;
    } else {
        throw std::runtime_error("VL53L4CD: inter_measurement_ms must be 0 or > timing_budget_ms");
    }

    // budget_us is shifted once and shared by both register calculations
    budget_us <<= 12;

    auto calc_reg = [&](uint32_t period_factor) -> uint16_t {
        uint32_t tmp     = macro_period * period_factor;
        uint32_t ls_byte = ((budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1u;
        uint16_t ms_byte = 0;
        while (ls_byte & 0xFFFFFF00u) { ls_byte >>= 1; ++ms_byte; }
        return static_cast<uint16_t>((ms_byte << 8) | (ls_byte & 0xFFu));
    };

    wrWord(REG_RANGE_CONFIG_A, calc_reg(16));
    wrWord(REG_RANGE_CONFIG_B, calc_reg(12));
}

void VL53L4CD::getRangeTiming(uint32_t& timing_budget_ms, uint32_t& inter_measurement_ms) const {
    uint32_t tmp       = rdDWord(REG_INTERMEASUREMENT);
    uint16_t clock_pll = rdWord(REG_OSC_CALIBRATE_VAL) & 0x3FFu;
    inter_measurement_ms = static_cast<uint32_t>(
        tmp / static_cast<uint32_t>(1.065f * static_cast<float>(clock_pll)));

    uint16_t osc_freq  = rdWord(REG_OSC_FREQUENCY);
    uint16_t cfg_high  = rdWord(REG_RANGE_CONFIG_A);

    uint32_t macro_period = (2304u * (0x40000000u / osc_freq)) >> 6;
    uint32_t ls_byte      = static_cast<uint32_t>(cfg_high & 0x00FFu) << 4;
    uint32_t ms_byte      = 0x04u - ((cfg_high & 0xFF00u) >> 8) - 1u + 1u;

    macro_period     *= 16u;
    timing_budget_ms  = (((ls_byte + 1u) * (macro_period >> 6))
                         - ((macro_period >> 6) >> 1)) >> 12;

    if (ms_byte < 12u)
        timing_budget_ms >>= ms_byte;

    if (tmp == 0u) {
        timing_budget_ms += 2500u;
    } else {
        timing_budget_ms  = timing_budget_ms * 2u + 4300u;
    }
    timing_budget_ms /= 1000u;
}

// ---------------------------------------------------------------------------
// Correction settings
// ---------------------------------------------------------------------------

void VL53L4CD::setOffset(int16_t offset_mm) {
    wrWord(REG_RANGE_OFFSET_MM, static_cast<uint16_t>(offset_mm * 4));
    wrWord(REG_INNER_OFFSET_MM, 0);
    wrWord(REG_OUTER_OFFSET_MM, 0);
}

int16_t VL53L4CD::getOffset() const {
    uint16_t raw    = rdWord(REG_RANGE_OFFSET_MM);
    raw             = (raw << 3) >> 5;   // sign-extend from 11-bit field
    int16_t  offset = static_cast<int16_t>(raw);
    if (offset > 1024) offset -= 2048;
    return offset;
}

void VL53L4CD::setXtalk(uint16_t xtalk_kcps) {
    wrWord(REG_XTALK_X_GRADIENT, 0);
    wrWord(REG_XTALK_Y_GRADIENT, 0);
    wrWord(REG_XTALK_PLANE_OFFSET, static_cast<uint16_t>(xtalk_kcps << 9));
}

uint16_t VL53L4CD::getXtalk() const {
    return static_cast<uint16_t>(
        std::round(static_cast<float>(rdWord(REG_XTALK_PLANE_OFFSET)) / 512.0f));
}

void VL53L4CD::setSignalThreshold(uint16_t signal_kcps) {
    wrWord(REG_MIN_COUNT_RATE, signal_kcps >> 3);
}

uint16_t VL53L4CD::getSignalThreshold() const {
    return rdWord(REG_MIN_COUNT_RATE) << 3;
}

void VL53L4CD::setSigmaThreshold(uint16_t sigma_mm) {
    if (sigma_mm > (0xFFFFu >> 2))
        throw std::runtime_error("VL53L4CD: sigma threshold value too large");
    wrWord(REG_SIGMA_THRESH, static_cast<uint16_t>(sigma_mm << 2));
}

uint16_t VL53L4CD::getSigmaThreshold() const {
    return rdWord(REG_SIGMA_THRESH) >> 2;
}

void VL53L4CD::setDetectionThresholds(uint16_t low_mm, uint16_t high_mm, uint8_t window) {
    wrByte(REG_SYSTEM_INTERRUPT, window);
    wrWord(REG_THRESH_HIGH, high_mm);
    wrWord(REG_THRESH_LOW,  low_mm);
}

void VL53L4CD::getDetectionThresholds(uint16_t& low_mm, uint16_t& high_mm, uint8_t& window) const {
    high_mm = rdWord(REG_THRESH_HIGH);
    low_mm  = rdWord(REG_THRESH_LOW);
    window  = rdByte(REG_SYSTEM_INTERRUPT) & 0x07u;
}

// ---------------------------------------------------------------------------
// Temperature compensation
// ---------------------------------------------------------------------------

void VL53L4CD::startTemperatureUpdate() {
    wrByte(REG_VHV_CONFIG_TIMEOUT, 0x81);
    wrByte(0x000B, 0x92);
    startRanging();
    waitForDataReady(1000);
    clearInterrupt();
    stopRanging();
    wrByte(REG_VHV_CONFIG_TIMEOUT, 0x09);
    wrByte(0x000B, 0x00);
}

// ---------------------------------------------------------------------------
// Factory calibration
// ---------------------------------------------------------------------------

int16_t VL53L4CD::calibrateOffset(int16_t target_mm, int16_t nb_samples) {
    if (nb_samples < 5 || nb_samples > 255 || target_mm < 10 || target_mm > 1000)
        throw std::runtime_error("VL53L4CD: calibrateOffset invalid arguments");

    wrWord(REG_RANGE_OFFSET_MM, 0);
    wrWord(REG_INNER_OFFSET_MM, 0);
    wrWord(REG_OUTER_OFFSET_MM, 0);

    // Warm-up: 10 samples discarded
    startRanging();
    for (int i = 0; i < 10; ++i) {
        waitForDataReady(5000);
        getResult();
        clearInterrupt();
    }
    stopRanging();

    // Averaging pass
    int32_t avg = 0;
    startRanging();
    for (int i = 0; i < nb_samples; ++i) {
        waitForDataReady(5000);
        avg += getResult().distance_mm;
        clearInterrupt();
    }
    stopRanging();

    int16_t offset = static_cast<int16_t>(target_mm - static_cast<int16_t>(avg / nb_samples));
    wrWord(REG_RANGE_OFFSET_MM, static_cast<uint16_t>(offset * 4));
    return offset;
}

uint16_t VL53L4CD::calibrateXtalk(int16_t target_mm, int16_t nb_samples) {
    if (nb_samples < 5 || nb_samples > 255 || target_mm < 10 || target_mm > 5000)
        throw std::runtime_error("VL53L4CD: calibrateXtalk invalid arguments");

    wrWord(REG_XTALK_PLANE_OFFSET, 0); // disable xtalk compensation

    // Warm-up: 10 samples discarded
    startRanging();
    for (int i = 0; i < 10; ++i) {
        waitForDataReady(5000);
        getResult();
        clearInterrupt();
    }
    stopRanging();

    // Averaging pass — skip first sample and invalid measurements
    float avg_dist = 0, avg_spad = 0, avg_signal = 0, count = 0;
    startRanging();
    for (int i = 0; i < nb_samples; ++i) {
        waitForDataReady(5000);
        Results r = getResult();
        clearInterrupt();
        if (r.range_status == 0 && i > 0) {
            avg_dist   += static_cast<float>(r.distance_mm);
            avg_spad   += static_cast<float>(r.number_of_spad);
            avg_signal += static_cast<float>(r.signal_rate_kcps);
            ++count;
        }
    }
    stopRanging();

    if (count == 0.0f)
        throw std::runtime_error("VL53L4CD: xtalk calibration failed — no valid samples");

    avg_dist   /= count;
    avg_spad   /= count;
    avg_signal /= count;

    float tmp_xtalk = (1.0f - (avg_dist / static_cast<float>(target_mm)))
                    * (avg_signal / avg_spad);

    if (tmp_xtalk > 127.0f)
        throw std::runtime_error("VL53L4CD: xtalk calibration failed — value exceeds 127 kcps");

    wrWord(REG_XTALK_PLANE_OFFSET, static_cast<uint16_t>(tmp_xtalk * 512.0f));
    return static_cast<uint16_t>(std::round(tmp_xtalk));
}

// ---------------------------------------------------------------------------
// Misc
// ---------------------------------------------------------------------------

void VL53L4CD::setI2CAddress(uint8_t new_address) {
    wrByte(REG_I2C_SLAVE_ADDRESS, new_address >> 1);
    addr_ = new_address;
}

uint16_t VL53L4CD::getSensorId() const {
    return rdWord(REG_MODEL_ID);
}

VL53L4CD::Version VL53L4CD::getSWVersion() {
    return { VER_MAJOR, VER_MINOR, VER_BUILD, VER_REVISION };
}