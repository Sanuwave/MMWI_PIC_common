#ifndef VD6283TX_H
#define VD6283TX_H

// =============================================================================
// VD6283TX — Hybrid filter multispectral ALS + flicker sensor driver
// ST Microelectronics DS13735 Rev 3
//
// Channels:  CH1=Red  CH2=Visible  CH3=Blue  CH4=Green  CH5=IR  CH6=Clear
// I²C addr:  0x20 (7-bit default)
// ADC:       24-bit per channel (H/M/L bytes), 16-bit mode supported
//
// Singleton: use VD6283TX::getInstance() from anywhere.
//            Call init(addr) once before any other method.
// =============================================================================

#include <cstdint>
#include <array>
#include "I2cMgr.h"

class VD6283TX {
public:
    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------
    static constexpr uint8_t  DEFAULT_I2C_ADDR = 0x20;
    static constexpr uint8_t  DEVICE_ID_VALUE  = 0x70;
    static constexpr uint8_t  NUM_CHANNELS     = 6;

    static constexpr float    EXTIME_STEP_MS      = 1.6f;
    static constexpr uint16_t EXTIME_REG_MAX       = 0x3FF;
    static constexpr uint16_t EXTIME_SAT_BOUNDARY  = 0x047;
    static constexpr uint16_t SAT_SHORT_PER_STEP   = 910;
    static constexpr uint16_t SAT_LONG             = 65535;

    // -------------------------------------------------------------------------
    // Channel bitmask flags
    // -------------------------------------------------------------------------
    static constexpr uint8_t CH_RED     = (1u << 0);
    static constexpr uint8_t CH_VISIBLE = (1u << 1);
    static constexpr uint8_t CH_BLUE    = (1u << 2);
    static constexpr uint8_t CH_GREEN   = (1u << 3);
    static constexpr uint8_t CH_IR      = (1u << 4);
    static constexpr uint8_t CH_CLEAR   = (1u << 5);
    static constexpr uint8_t CH_ALL     = 0x3Fu;

    // -------------------------------------------------------------------------
    // Enumerations
    // -------------------------------------------------------------------------
    enum class Channel : uint8_t {
        RED     = 1,
        VISIBLE = 2,
        BLUE    = 3,
        GREEN   = 4,
        IR      = 5,
        CLEAR   = 6
    };

    enum class Gain : uint8_t {
        X66_6 = 0x01, X50   = 0x02, X33   = 0x03, X25   = 0x04,
        X16   = 0x05, X10   = 0x06, X7_1  = 0x07, X5    = 0x08,
        X3_33 = 0x09, X2_5  = 0x0A, X1_67 = 0x0B, X1_25 = 0x0C,
        X1    = 0x0D, X0_83 = 0x0E, X0_71 = 0x0F
    };

    enum class AlsMode : uint8_t {
        SINGLE_SHOT = 0x00,
        CONTINUOUS  = 0x03
    };

    enum class FlickerMode : uint8_t {
        INTERNAL_GPIO1 = 0,
        INTERNAL_GPIO2 = 1,
        EXTERNAL_CLK   = 2
    };

    enum class Gpio1Cfg : uint8_t {
        OPEN_DRAIN = 0x00,
        PUSH_PULL  = 0x01,
        ANALOG     = 0x02
    };

    // -------------------------------------------------------------------------
    // Data structures
    // -------------------------------------------------------------------------
    struct ChannelData {
        uint32_t red;
        uint32_t visible;
        uint32_t blue;
        uint32_t green;
        uint32_t ir;
        uint32_t clear;
    };

    // -------------------------------------------------------------------------
    // Singleton interface
    // -------------------------------------------------------------------------

    /**
     * @brief Retrieve the singleton instance.
     * The instance is constructed on first call (Meyers singleton).
     * Construction is thread-safe in C++11 and later.
     */
    static VD6283TX& getInstance();

    // Non-copyable, non-movable
    VD6283TX(const VD6283TX&)            = delete;
    VD6283TX& operator=(const VD6283TX&) = delete;
    VD6283TX(VD6283TX&&)                 = delete;
    VD6283TX& operator=(VD6283TX&&)      = delete;

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    /**
     * @brief Initialise the sensor.
     * Must be called once before any other method.
     * Retrieves the I²C bus via I2cMgr::getInstance().
     * Verifies DEVICE_ID and applies recommended post-power-up register
     * defaults (pedestal = 3, SDA driver, GPIO1 open-drain).
     * @param addr 7-bit I²C slave address (default 0x20).
     * @return true on success.
     */
    bool init(uint8_t addr = DEFAULT_I2C_ADDR);

    /**
     * @brief Stop all operations and place sensor in idle state.
     * Should be called before power-down.
     */
    void deinit();

    /// @return true if init() has completed successfully.
    bool isInitialized() const { return m_initialized; }

    // -------------------------------------------------------------------------
    // Device identification
    // -------------------------------------------------------------------------
    bool getDeviceId(uint8_t& id) const;
    bool getRevisionId(uint8_t& rev) const;

    // -------------------------------------------------------------------------
    // Channel and gain configuration
    // -------------------------------------------------------------------------
    bool enableChannels(uint8_t mask);
    bool setGain(Channel ch, Gain gain);
    bool setGainAll(Gain gain);

    // -------------------------------------------------------------------------
    // Exposure and timing
    // -------------------------------------------------------------------------
    bool  setExposureMs(float ms);
    float getExposureMs() const;
    bool  setIntermeasurementPeriod(uint8_t steps);

    // -------------------------------------------------------------------------
    // ALS operation
    // -------------------------------------------------------------------------
    bool startALS(AlsMode mode = AlsMode::SINGLE_SHOT);
    bool stopALS();

    // -------------------------------------------------------------------------
    // Data readout
    // -------------------------------------------------------------------------
    bool isDataReady() const;
    bool clearInterrupt();
    bool readAllChannels(ChannelData& data);
    bool readChannel(Channel ch, uint32_t& counts);

    // -------------------------------------------------------------------------
    // Saturation
    // -------------------------------------------------------------------------
    uint32_t getSaturationLimit() const;
    bool     isSaturated(uint16_t count16) const;

    // -------------------------------------------------------------------------
    // Flicker detection
    // -------------------------------------------------------------------------
    bool startFlicker(Channel ch, FlickerMode mode = FlickerMode::INTERNAL_GPIO1);
    bool stopFlicker();
    bool setFlickerPedestalValue(uint8_t val);

    // -------------------------------------------------------------------------
    // GPIO / driver configuration
    // -------------------------------------------------------------------------
    bool setGpio1Config(Gpio1Cfg cfg);

private:
    // Private constructor/destructor — only getInstance() may instantiate
    VD6283TX();
    ~VD6283TX() = default;

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------
    bool writeReg(uint8_t reg, uint8_t data);
    bool readReg(uint8_t reg, uint8_t& data) const;
    bool readRegs(uint8_t startReg, uint8_t* buf, size_t len) const;

    static uint8_t gainRegForChannel(Channel ch);
    static uint8_t dataRegForChannel(Channel ch);
    static uint8_t flickerChSel(Channel ch);

    // -------------------------------------------------------------------------
    // Instance state
    // -------------------------------------------------------------------------
    uint8_t  m_addr;
    uint16_t m_extimeReg;
    bool     m_initialized;
};

#endif // VD6283TX_H