
#ifndef LM3643_H
#define LM3643_H

#include <cstdint>

class I2cMgr;

class LM3643 {
public:
    enum class LedChannel {
        LED1,
        LED2,
        BOTH
    };
    
    enum class LedMode {
        STANDBY,
        TORCH,
        FLASH
    };

    enum class LedFlashTimeoutId : uint8_t {
        TIMEOUT_10MS = 0,
        TIMEOUT_20MS,
        TIMEOUT_30MS,
        TIMEOUT_40MS,
        TIMEOUT_50MS,
        TIMEOUT_60MS,
        TIMEOUT_70MS,
        TIMEOUT_80MS,
        TIMEOUT_90MS,
        TIMEOUT_100MS,
        TIMEOUT_150MS,
        TIMEOUT_200MS,
        TIMEOUT_250MS,
        TIMEOUT_300MS,
        TIMEOUT_350MS,
        TIMEOUT_400MS,
        NUM_FLASH_TIMEOUT_IDS
    };

    static constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x63;
    static constexpr uint8_t ALT_I2C_ADDRESS = 0x67;
    
    explicit LM3643(I2cMgr* i2cMgr, uint8_t address = DEFAULT_I2C_ADDRESS);
    ~LM3643();
    
    // Initialization
    bool init();
    bool initDevice();
    
    // Per-LED torch control
    bool enableTorch(LedChannel channel);
    bool setTorchBrightness(LedChannel channel, uint8_t brightness);
    
    // Per-LED flash control
    bool enableFlash(LedChannel channel);
    bool setFlashBrightness(LedChannel channel, uint8_t brightness);
    bool setFlashTimeout(LedFlashTimeoutId timeout);
    
    // LED control
    bool disable(LedChannel channel = LedChannel::BOTH);
    bool isEnabled(LedChannel channel) const;
    LedMode getMode(LedChannel channel) const;
    
    // Advanced features
    bool setTxMask(bool enable); // TX event input mask
    bool setStrobeEnable(LedChannel channel); // Hardware strobe
    bool setTorchEnable(LedChannel channel); // Hardware torch
    
private:
    I2cMgr* m_i2cMgr;
    uint8_t m_address;
    
    // Current state tracking
    uint8_t m_enableReg;
    uint8_t m_led1TorchBrightness;
    uint8_t m_led2TorchBrightness;
    uint8_t m_led1FlashBrightness;
    uint8_t m_led2FlashBrightness;
    
    // LM3643 Register addresses (from datasheet)
    static constexpr uint8_t REG_ENABLE = 0x01;
    static constexpr uint8_t REG_IVFM = 0x02;
    static constexpr uint8_t REG_LED1_FLASH_BRIGHTNESS = 0x03;
    static constexpr uint8_t REG_LED2_FLASH_BRIGHTNESS = 0x04;
    static constexpr uint8_t REG_LED1_TORCH_BRIGHTNESS = 0x05;
    static constexpr uint8_t REG_LED2_TORCH_BRIGHTNESS = 0x06;
    static constexpr uint8_t REG_BOOST_CONFIG = 0x07;
    static constexpr uint8_t REG_TIMING_CONFIG = 0x08;
    static constexpr uint8_t REG_TEMP = 0x09;
    static constexpr uint8_t REG_FLAGS1 = 0x0A;
    static constexpr uint8_t REG_FLAGS2 = 0x0B;
    static constexpr uint8_t REG_DEVICE_ID = 0x0C;
    
    // Enable register (0x01) bit masks
    static constexpr uint8_t ENABLE_MODE_MASK = 0x03 << 2;
    static constexpr uint8_t ENABLE_MODE_STANDBY = 0x00 << 2;
    static constexpr uint8_t ENABLE_MODE_IR = 0x01 << 2;
    static constexpr uint8_t ENABLE_MODE_TORCH = 0x02 << 2;
    static constexpr uint8_t ENABLE_MODE_FLASH = 0x03 << 2;
    
    static constexpr uint8_t ENABLE_EDGE    = 0x01 << 6;
    static constexpr uint8_t ENABLE_STROBE  = 0x01 << 5;
    static constexpr uint8_t ENABLE_TORCH   = 0x01 << 4;
    static constexpr uint8_t ENABLE_TX_MASK = 0x01 << 7;
    static constexpr uint8_t ENABLE_LED2_EN = 0x01 << 1;
    static constexpr uint8_t ENABLE_LED1_EN = 0x01 << 0;
    
    // Brightness masks (7 bits)
    static constexpr uint8_t MAX_BRIGHTNESS = 0x7F;
    
    // Timing configuration register (0x08) for flash duration/timeout
    static constexpr uint8_t TIMING_DURATION_MASK = 0x1F;
    static constexpr uint8_t TIMING_TIMEOUT_MASK = 0xE0;
    
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    bool updateEnableRegister();
};

#endif // LM3643_H