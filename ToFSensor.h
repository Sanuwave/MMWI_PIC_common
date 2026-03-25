#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <cstdint>
#include <memory>
#include "VL53L4CD.h"

/**
 * @brief Singleton wrapper for VL53L4CD.
 *
 * Owns a VL53L4CD instance and exposes an identical interface.
 * I2cMgr must be open before calling init().
 *
 * Usage:
 *   I2cMgr::getInstance().open("/dev/i2c-1");
 *   ToFSensor::getInstance().init();
 *   ToFSensor::getInstance().startRanging();
 */
class ToFSensor {
public:
    using Results = VL53L4CD::Results;

    // -----------------------------------------------------------------------
    // Singleton access
    // -----------------------------------------------------------------------

    /** @brief Returns the singleton instance. */
    static ToFSensor& getInstance();

    ToFSensor(const ToFSensor&)            = delete;
    ToFSensor& operator=(const ToFSensor&) = delete;
    ToFSensor(ToFSensor&&)                 = delete;
    ToFSensor& operator=(ToFSensor&&)      = delete;

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    /**
     * @brief Initialise the sensor. Safe to call again to re-initialise.
     * @param i2c_address  7-bit I2C address (default 0x29).
     * @param timing_ms    Timing budget in ms (10–200, default 50).
     * @throws std::runtime_error if sensor ID is unexpected or init fails.
     */
    void init(uint8_t i2c_address = 0x29, uint32_t timing_ms = 50);

    /**
     * @brief Returns true if init() has completed successfully.
     */
    bool isInit() const;

    // -----------------------------------------------------------------------
    // Ranging
    // -----------------------------------------------------------------------

    /**
     * @brief Start a ranging session.
     * Runs in continuous mode if inter-measurement is 0, autonomous otherwise.
     */
    void startRanging();

    /** @brief Stop an active ranging session. */
    void stopRanging();

    /**
     * @brief Check whether a new measurement is available.
     * @return true if data is ready to be read.
     */
    bool isDataReady();

    /**
     * @brief Clear the data-ready interrupt.
     * Must be called after each getResult() to arm the interrupt for the next measurement.
     */
    void clearInterrupt();

    /**
     * @brief Retrieve the latest measurement from the sensor.
     * @return Results struct. Check range_status == 0 for a valid measurement.
     * @throws std::runtime_error on I2C failure.
     */
    Results getResult();

    // -----------------------------------------------------------------------
    // Timing
    // -----------------------------------------------------------------------

    /**
     * @brief Set the ranging timing budget and inter-measurement period.
     * @param budget_ms   Timing budget in ms (10–200).
     * @param inter_ms    Inter-measurement period in ms.
     *                    0 = continuous mode; otherwise must be > budget_ms.
     * @throws std::runtime_error if arguments are out of range.
     */
    void setRangeTiming(uint32_t budget_ms, uint32_t inter_ms = 0);

    /**
     * @brief Get the current timing budget and inter-measurement period.
     * @param budget_ms   Populated with the current timing budget in ms.
     * @param inter_ms    Populated with the current inter-measurement period in ms.
     */
    void getRangeTiming(uint32_t& budget_ms, uint32_t& inter_ms) const;

    // -----------------------------------------------------------------------
    // Correction settings
    // -----------------------------------------------------------------------

    /**
     * @brief Set the range offset correction.
     * @param mm  Offset in millimetres (−1024 to +1023).
     */
    void setOffset(int16_t mm);

    /**
     * @brief Get the current range offset correction.
     * @return Offset in millimetres.
     */
    int16_t getOffset() const;

    /**
     * @brief Set the crosstalk (cover-glass) correction.
     * @param kcps  Xtalk value in kcps (0–128). 0 = no correction.
     */
    void setXtalk(uint16_t kcps);

    /**
     * @brief Get the current crosstalk correction value.
     * @return Xtalk value in kcps.
     */
    uint16_t getXtalk() const;

    /**
     * @brief Set the minimum signal threshold.
     * Results below this level will have range_status == 2.
     * @param kcps  Signal threshold in kcps (0 = disabled, default 1024).
     */
    void setSignalThreshold(uint16_t kcps);

    /**
     * @brief Get the current minimum signal threshold.
     * @return Signal threshold in kcps.
     */
    uint16_t getSignalThreshold() const;

    /**
     * @brief Set the sigma (measurement std deviation) threshold.
     * Results exceeding this value will have range_status == 1.
     * @param mm  Sigma threshold in mm (0 = disabled, default 15).
     * @throws std::runtime_error if value exceeds the 14-bit maximum.
     */
    void setSigmaThreshold(uint16_t mm);

    /**
     * @brief Get the current sigma threshold.
     * @return Sigma threshold in mm.
     */
    uint16_t getSigmaThreshold() const;

    /**
     * @brief Configure distance thresholds for GPIO1 interrupt generation.
     * @param low     Low distance threshold in mm.
     * @param high    High distance threshold in mm.
     * @param window  Trigger condition: 0 = below low, 1 = above high,
     *                2 = outside window, 3 = inside window.
     */
    void setDetectionThresholds(uint16_t low, uint16_t high, uint8_t window);

    /**
     * @brief Get the current distance detection thresholds.
     * @param low     Populated with the low threshold in mm.
     * @param high    Populated with the high threshold in mm.
     * @param window  Populated with the current trigger condition.
     */
    void getDetectionThresholds(uint16_t& low, uint16_t& high, uint8_t& window) const;

    // -----------------------------------------------------------------------
    // Temperature compensation
    // -----------------------------------------------------------------------

    /**
     * @brief Perform a VHV temperature recalibration.
     * Call when ambient temperature has shifted by more than 8 °C.
     * The sensor must not be ranging when this is called.
     * @throws std::runtime_error on timeout or I2C failure.
     */
    void updateTemperatureCalibration();

    // -----------------------------------------------------------------------
    // Factory calibration
    // -----------------------------------------------------------------------

    /**
     * @brief Run an offset calibration routine and program the result into the device.
     * The sensor must not be ranging when this is called.
     * @param target_mm   True distance to the calibration target (10–1000 mm, recommend 100 mm).
     * @param nb_samples  Number of averaging samples (5–255, recommend ≥10).
     * @return Measured offset in mm as programmed into the device.
     * @throws std::runtime_error if arguments are out of range or I2C fails.
     */
    int16_t calibrateOffset(int16_t target_mm = 100, int16_t nb_samples = 20);

    /**
     * @brief Run a crosstalk calibration routine and program the result into the device.
     * Used when a protective cover-glass is fitted over the sensor.
     * The sensor must not be ranging when this is called.
     * @param target_mm   True distance to the calibration target (10–5000 mm).
     * @param nb_samples  Number of averaging samples (5–255, recommend ≥10).
     * @return Measured xtalk value in kcps as programmed into the device.
     * @throws std::runtime_error if no valid samples are obtained or value exceeds 127 kcps.
     */
    uint16_t calibrateXtalk(int16_t target_mm = 200, int16_t nb_samples = 10);

    // -----------------------------------------------------------------------
    // Misc
    // -----------------------------------------------------------------------

    /**
     * @brief Reassign the sensor's I2C address.
     * Useful when multiple sensors share the same bus.
     * @param addr  New 7-bit I2C address.
     */
    void setI2CAddress(uint8_t addr);

    /**
     * @brief Read the sensor model ID.
     * @return 0xEBAA if the sensor is responding correctly.
     */
    uint16_t getSensorId() const;

    /**
     * @brief Get the sensor's current 7-bit I2C address.
     * @return I2C address byte.
     */
    uint8_t i2cAddress() const;

private:
    ToFSensor()  = default;
    ~ToFSensor() = default;

    std::unique_ptr<VL53L4CD> sensor_;
};

#endif // TOF_SENSOR_H