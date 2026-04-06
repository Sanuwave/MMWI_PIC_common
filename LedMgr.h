#ifndef LED_MGR_H
#define LED_MGR_H

#include <memory>
#include <mutex>
#include <cstdint>
#include <array>
#include "Gpio.h"
#include "LM3643.h"

// Forward declarations
class I2cMgr;
class LM3643;
class PCA9545A;

class LedMgr {
public:
    enum class LedId : uint8_t {
        LED0 = 0,
        LED1,
        LED2,
        LED3,
        LED_GROUP_0 = LED3,
        LED4,
        LED5,
        LED6,
        LED7,
        LED_GROUP_1= LED7,
        LED8,
        LED9,
        LED10,
        LED11,
        LED_GROUP_2 = LED11,
        LED12,
        LED13,
        LED14,
        LED15,
        LED_GROUP_3 = LED15,
        LED16,
        LED17,
        LED18,
        LED19,
        LED_GROUP_4 = LED19,
        LED20,
        LED21,
        LED22,
        LED23,
        LED_GROUP_5 = LED23,
        LED24,
        LED25,
        LED26,
        LED27,
        LED_GROUP_6 = LED27,
        LED28,
        LED29,
        LED30,
        LED31,
        LED_GROUP_7 = LED31,
        NUM_LED_IDS
    };

    enum class LedHwStrobeLineId : uint8_t {
        STROBE_A_NS = 0,
        STROBE_B_NS,
        STROBE_A_WE,
        STROBE_B_WE,
        NUM_HW_STROBE_LINE_IDS = STROBE_B_WE // TODO - remove this when GPIO7 is replaced as camera SPI needs this
    };

    enum class LedHwTorchLineId : uint8_t {
        TORCH_A_NS = 0,
        TORCH_B_NS,
        TORCH_A_WE,
        TORCH_B_WE,
        NUM_HW_TORCH_LINE_IDS
    };
    
    enum class LedHwStrobeMode : uint8_t {
        STROBE_MODE_LEVEL = 0,   // Flash pulse width determined by duration of GPIO line held high
        STROBE_MODE_EDGE = 1,    // Flash pulse width determined by physical strobe duration (GPIO line held high for at least pulse timeout duration)   
        NUM_HW_STROBE_MODE_IDS
    };
    // Singleton access
    static LedMgr& getInstance();
    
    // Delete copy/move constructors and assignment operators
    LedMgr(const LedMgr&) = delete;
    LedMgr& operator=(const LedMgr&) = delete;
    LedMgr(LedMgr&&) = delete;
    LedMgr& operator=(LedMgr&&) = delete;
    
    // Initialization
    bool initialize();
    bool isInitialized() const;
    
    // Individual LED torch control (S/W trigger)
    bool setTorchMode(LedId ledId);
    
    // Individual LED flash control (S/W trigger)
    bool setFlashMode(LedId ledId);
    bool setFlashTimeout(LedId ledId, uint16_t timeout_ms);

    bool setFlashTorchBrightness(LedId ledId, uint8_t brightness);

    // LED control
    bool turnOff(LedId ledId);
    bool resetLeds();

    // LED state
    bool isLedEnabled(LedId ledId);
    
    // H/W Trigger !!
    bool setStrobeEnable(LedId ledId, LedHwStrobeMode strobeMode = LedHwStrobeMode::STROBE_MODE_EDGE);
    bool setTorchEnable(LedId ledId);

    // Pulse control
    // For edge mode, pulse width is determined by physical strobe duration. 
    // For level mode, pulse width is determined by duration of GPIO line held high.
    bool enableHwStrobeEdgeMode(LedHwStrobeLineId line);
    bool enableHwStrobeLevelMode(LedHwStrobeLineId line, uint32_t pulseWidthUs);
    void enableAllStrobeLines();
    void disableAllStrobeLines();
    bool enableHwTorch(LedHwTorchLineId line);
    bool disableHwTorch(LedHwTorchLineId line);
    bool setTxMask(bool enable);

    std::string getLastError() const { return m_lastError; }
    // Rick here's a suggested group of calls for the UVBF test:
    // 1) For each led that is enabled (isLedEnabled()), turnOff() the LED
    // 2) When user select an LED (or all), call setStrobeEnable() for that LED.
    // 3) At the appropriate time, call enableHwStrobeLevelMode() with 
    //    the appropriate timeout for the first 3 strobe lines (note the 4th is not functional)
    // 4) Repeat as necessary
    // 5) On exiting turnOff() any enabled LEDs to ensure they are off when the test finishes
    
private:
    LedMgr();
    ~LedMgr();
       
    LM3643*     m_ledDriver0;    
    LM3643*     m_ledDriverM0;
    LM3643*     m_ledDriver1;    
    LM3643*     m_ledDriverM1;
    LM3643*     m_ledDriver2;    
    LM3643*     m_ledDriverM2;
    LM3643*     m_ledDriver3;    
    LM3643*     m_ledDriverM3;
    LM3643*     m_ledDriver4;    
    LM3643*     m_ledDriverM4;
    LM3643*     m_ledDriver5;    
    LM3643*     m_ledDriverM5;
    LM3643*     m_ledDriver6;    
    LM3643*     m_ledDriverM6;
    LM3643*     m_ledDriver7;    
    LM3643*     m_ledDriverM7;
    LM3643*     m_currentLedDriver;
    PCA9545A*   m_i2cSwitchDriver;
    PCA9545A*   m_i2cSwitchDriverAlt;
    PCA9545A*   m_currentI2cSwitchDriver;
    I2cMgr*     m_i2cMgr;
    
    mutable std::mutex m_mutex;
    bool m_initialized;
    LM3643::LedChannel m_ledChannel; 
    std::string m_lastError;
    struct  LedChannelInfo {
        const bool          isDefaultLedDriver;
        const bool          isDefaultI2cSwitchDriver;
        const uint8_t       switchChannel;
        LM3643::LedChannel  ledChannel;
        bool                isEnabled;
    };

    std::array<LedChannelInfo, 32> m_ledChannelInfoTable;
    std::array<uint8_t, 4> m_strobeLineInfoTable;
    std::array<uint8_t, 4> m_torchLineInfoTable;


    static constexpr uint32_t STROBE_PULSE_TIMEOUT_US = 2000;

    bool setDriversAndChannel(LedId ledId);
    bool resetLedStates();
    LM3643* getLedDriver(LedId ledId);
};

#endif // LED_MGR_H