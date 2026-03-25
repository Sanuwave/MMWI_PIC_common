#ifndef VL53L4CD_H
#define VL53L4CD_H

#include <cstdint>
#include <string>

/**
 * @brief C++ driver for the ST VL53L4CD Time-of-Flight sensor.
 *
 * Replaces VL53L4CD_api.h/.c and VL53L4CD_calibration.h/.c.
 * All I/O goes through platform.cpp -> I2cMgr.
 * All errors are reported via std::runtime_error.
 */
class VL53L4CD {
public:
    // -----------------------------------------------------------------------
    // Types
    // -----------------------------------------------------------------------

    struct Version {
        uint8_t  major;
        uint8_t  minor;
        uint8_t  build;
        uint32_t revision;
    };

    struct Results {
        uint8_t  range_status;           ///< 0 = valid measurement
        uint16_t distance_mm;
        uint32_t ambient_rate_kcps;
        uint32_t ambient_per_spad_kcps;
        uint32_t signal_rate_kcps;
        uint32_t signal_per_spad_kcps;
        uint16_t number_of_spad;
        uint16_t sigma_mm;
    };

    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    explicit VL53L4CD(uint8_t i2c_address);

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    /**
     * @brief Boot-wait, load default config, run VHV, set 50 ms timing.
     * @throws std::runtime_error on boot timeout or I2C failure.
     */
    void sensorInit();

    // -----------------------------------------------------------------------
    // Ranging
    // -----------------------------------------------------------------------

    void    startRanging();
    void    stopRanging();
    bool    isDataReady() const;
    void    clearInterrupt();
    Results getResult() const;

    // -----------------------------------------------------------------------
    // Timing
    // -----------------------------------------------------------------------

    /**
     * @param timing_budget_ms    10–200 ms.
     * @param inter_measurement_ms 0 = continuous; else must be > timing_budget_ms.
     */
    void setRangeTiming(uint32_t timing_budget_ms, uint32_t inter_measurement_ms = 0);
    void getRangeTiming(uint32_t& timing_budget_ms, uint32_t& inter_measurement_ms) const;

    // -----------------------------------------------------------------------
    // Correction settings
    // -----------------------------------------------------------------------

    void    setOffset(int16_t offset_mm);   ///< −1024 to +1023 mm
    int16_t getOffset() const;

    void     setXtalk(uint16_t xtalk_kcps); ///< 0–128 kcps
    uint16_t getXtalk() const;

    void     setSignalThreshold(uint16_t signal_kcps);
    uint16_t getSignalThreshold() const;

    void     setSigmaThreshold(uint16_t sigma_mm);
    uint16_t getSigmaThreshold() const;

    /**
     * @param window 0=below low, 1=above high, 2=outside, 3=inside.
     */
    void setDetectionThresholds(uint16_t low_mm, uint16_t high_mm, uint8_t window);
    void getDetectionThresholds(uint16_t& low_mm, uint16_t& high_mm, uint8_t& window) const;

    // -----------------------------------------------------------------------
    // Temperature compensation
    // -----------------------------------------------------------------------

    /** Call when temperature has shifted by more than 8 °C. Sensor must be stopped. */
    void startTemperatureUpdate();

    // -----------------------------------------------------------------------
    // Factory calibration
    // -----------------------------------------------------------------------

    /**
     * @param target_mm  True distance (10–1000 mm, recommend 100 mm).
     * @param nb_samples 5–255, recommend ≥10.
     * @return Offset applied to device (mm).
     */
    int16_t calibrateOffset(int16_t target_mm = 100, int16_t nb_samples = 20);

    /**
     * @param target_mm  True distance (10–5000 mm).
     * @param nb_samples 5–255, recommend ≥10.
     * @return Xtalk value applied to device (kcps).
     */
    uint16_t calibrateXtalk(int16_t target_mm, int16_t nb_samples = 20);

    // -----------------------------------------------------------------------
    // Misc
    // -----------------------------------------------------------------------

    void     setI2CAddress(uint8_t new_address);
    uint16_t getSensorId() const;            ///< Should return 0xEBAA

    uint8_t i2cAddress() const { return addr_; }

    static Version getSWVersion();

private:
    uint8_t addr_;

    // Platform I/O — throw std::runtime_error on any failure
    void     wrByte (uint16_t reg, uint8_t  val) const;
    void     wrWord (uint16_t reg, uint16_t val) const;
    void     wrDWord(uint16_t reg, uint32_t val) const;
    uint8_t  rdByte (uint16_t reg) const;
    uint16_t rdWord (uint16_t reg) const;
    uint32_t rdDWord(uint16_t reg) const;
    void     waitMs (uint32_t ms)  const;

    /** Poll isDataReady() for up to timeout_ms × 1 ms ticks; throw on timeout. */
    void waitForDataReady(uint32_t timeout_ms = 1000) const;

    // Register map
    static constexpr uint16_t REG_I2C_SLAVE_ADDRESS  = 0x0001;
    static constexpr uint16_t REG_VHV_CONFIG_TIMEOUT = 0x0008;
    static constexpr uint16_t REG_XTALK_PLANE_OFFSET = 0x0016;
    static constexpr uint16_t REG_XTALK_X_GRADIENT   = 0x0018;
    static constexpr uint16_t REG_XTALK_Y_GRADIENT   = 0x001A;
    static constexpr uint16_t REG_RANGE_OFFSET_MM    = 0x001E;
    static constexpr uint16_t REG_INNER_OFFSET_MM    = 0x0020;
    static constexpr uint16_t REG_OUTER_OFFSET_MM    = 0x0022;
    static constexpr uint16_t REG_GPIO_HV_MUX_CTRL   = 0x0030;
    static constexpr uint16_t REG_GPIO_TIO_HV_STATUS = 0x0031;
    static constexpr uint16_t REG_SYSTEM_INTERRUPT   = 0x0046;
    static constexpr uint16_t REG_RANGE_CONFIG_A     = 0x005E;
    static constexpr uint16_t REG_RANGE_CONFIG_B     = 0x0061;
    static constexpr uint16_t REG_SIGMA_THRESH       = 0x0064;
    static constexpr uint16_t REG_MIN_COUNT_RATE     = 0x0066;
    static constexpr uint16_t REG_INTERMEASUREMENT   = 0x006C;
    static constexpr uint16_t REG_THRESH_HIGH        = 0x0072;
    static constexpr uint16_t REG_THRESH_LOW         = 0x0074;
    static constexpr uint16_t REG_INTERRUPT_CLEAR    = 0x0086;
    static constexpr uint16_t REG_SYSTEM_START       = 0x0087;
    static constexpr uint16_t REG_RESULT_RANGE_STATUS= 0x0089;
    static constexpr uint16_t REG_RESULT_SPAD_NB     = 0x008C;
    static constexpr uint16_t REG_RESULT_SIGNAL_RATE = 0x008E;
    static constexpr uint16_t REG_RESULT_AMBIENT_RATE= 0x0090;
    static constexpr uint16_t REG_RESULT_SIGMA       = 0x0092;
    static constexpr uint16_t REG_RESULT_DISTANCE    = 0x0096;
    static constexpr uint16_t REG_OSC_CALIBRATE_VAL  = 0x00DE;
    static constexpr uint16_t REG_FIRMWARE_STATUS    = 0x00E5;
    static constexpr uint16_t REG_MODEL_ID           = 0x010F;
    static constexpr uint16_t REG_OSC_FREQUENCY      = 0x0006;

    static constexpr uint8_t  VER_MAJOR    = 2;
    static constexpr uint8_t  VER_MINOR    = 2;
    static constexpr uint8_t  VER_BUILD    = 3;
    static constexpr uint32_t VER_REVISION = 0;

    static const uint8_t DEFAULT_CONFIGURATION[];
};

#endif // VL53L4CD_H