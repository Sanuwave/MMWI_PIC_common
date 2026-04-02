#include "LM3643.h"
#include "I2cMgr.h"
#include <iostream>

LM3643::LM3643(I2cMgr* i2cMgr, uint8_t address)
    : m_i2cMgr(i2cMgr)
    , m_address(address)
    , m_enableReg(ENABLE_MODE_STANDBY)
    , m_led1TorchBrightness(0)
    , m_led2TorchBrightness(0)
    , m_led1FlashBrightness(0)
    , m_led2FlashBrightness(0) {
}

LM3643::~LM3643() {
    disable(LedChannel::BOTH);
}

bool LM3643::init() {
    if (!m_i2cMgr) {
        std::cerr << "LM3643: I2C manager not initialized" << std::endl;
        return false;
    }
    else {
        return true;
    }
}

bool LM3643::initDevice() {
    
    // Read device ID to verify communication
    uint8_t deviceId;
    if (!readRegister(REG_DEVICE_ID, deviceId)) {
        return false;
    }    
    
    // Reset to standby mode with both LEDs disabled
    m_enableReg = ENABLE_MODE_STANDBY;
    if (!writeRegister(REG_ENABLE, m_enableReg)) {
        return false;
    }

    // Set default flash timeout
    if (!setFlashTimeout(LedFlashTimeoutId::TIMEOUT_150MS)) {
        return false;
    }

    // Set midlevel brightness for both torch and flash as default
    if (!setTorchBrightness(LedChannel::BOTH, MAX_BRIGHTNESS / 2) ||
        !setFlashBrightness(LedChannel::BOTH, MAX_BRIGHTNESS / 2)) {
        return false;
    }
    
    return true;
}

bool LM3643::enableTorch(LedChannel channel) {
    // Clear mode bits and set torch mode
    m_enableReg &= ~ENABLE_MODE_MASK;    
    m_enableReg |= ENABLE_MODE_TORCH;
    
    // Enable the appropriate LED(s)
    switch (channel) {
        case LedChannel::LED1:
            m_enableReg |= ENABLE_LED1_EN;
            break;
        case LedChannel::LED2:
            m_enableReg |= ENABLE_LED2_EN;
            break;
        case LedChannel::BOTH:
            m_enableReg |= (ENABLE_LED1_EN | ENABLE_LED2_EN);
            break;
    }
    return updateEnableRegister();
}

bool LM3643::setTorchBrightness(LedChannel channel, uint8_t brightness) {
    uint8_t level = brightness > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : brightness;
    
    bool success = true;
    
    switch (channel) {
        case LedChannel::LED1:
            m_led1TorchBrightness = level;
            success = writeRegister(REG_LED1_TORCH_BRIGHTNESS, level);
            break;
        case LedChannel::LED2:
            m_led2TorchBrightness = level;
            success = writeRegister(REG_LED2_TORCH_BRIGHTNESS, level);
            break;
        case LedChannel::BOTH:
            m_led1TorchBrightness = level;
            m_led2TorchBrightness = level;
            success = writeRegister(REG_LED1_TORCH_BRIGHTNESS, level) &&
                     writeRegister(REG_LED2_TORCH_BRIGHTNESS, level);
            break;
    }
    
    return success;
}

bool LM3643::enableFlash(LedChannel channel) {
    // Clear mode bits and set flash mode
    m_enableReg &= ~ENABLE_MODE_MASK;
    m_enableReg |= ENABLE_MODE_FLASH;
    
    // Enable the appropriate LED(s)
    switch (channel) {
        case LedChannel::LED1:
            m_enableReg |= ENABLE_LED1_EN;
            break;
        case LedChannel::LED2:
            m_enableReg |= ENABLE_LED2_EN;
            break;
        case LedChannel::BOTH:
            m_enableReg |= (ENABLE_LED1_EN | ENABLE_LED2_EN);
            break;
    }
    
    return updateEnableRegister();
}

bool LM3643::setFlashBrightness(LedChannel channel, uint8_t brightness) {
    uint8_t level = brightness > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : brightness;
    
    bool success = true;
    
    switch (channel) {
        case LedChannel::LED1:
            m_led1FlashBrightness = level;
            success = writeRegister(REG_LED1_FLASH_BRIGHTNESS, level);
            break;
        case LedChannel::LED2:
            m_led2FlashBrightness = level;
            success = writeRegister(REG_LED2_FLASH_BRIGHTNESS, level);
            break;
        case LedChannel::BOTH:
            m_led1FlashBrightness = level;
            m_led2FlashBrightness = level;
            success = writeRegister(REG_LED1_FLASH_BRIGHTNESS, level) &&
                     writeRegister(REG_LED2_FLASH_BRIGHTNESS, level);
            break;
    }
    
    return success;
}

bool LM3643::setFlashTimeout(LedFlashTimeoutId timeout) {
    uint8_t timing;
    if (!readRegister(REG_TIMING_CONFIG, timing)) {
        return false;
    }
    
    if (LedFlashTimeoutId::NUM_FLASH_TIMEOUT_IDS <= timeout) {
        // Cap off at max
        timeout = LedFlashTimeoutId::TIMEOUT_400MS;
    }
    
    return writeRegister(REG_TIMING_CONFIG, static_cast<uint8_t>(timeout));
}

bool LM3643::disable(LedChannel channel) {
    // If disabling this channel means no channels are left to disable,
    // set to both so strobe/torch bits are cleared too.
    if ((channel ==  LedChannel::LED1 &&  !(m_enableReg & ENABLE_LED2_EN)) || 
        (channel ==  LedChannel::LED2 &&  !(m_enableReg & ENABLE_LED1_EN))) {
        channel = LedChannel::BOTH;
    }
    switch (channel) {
        case LedChannel::LED1:
            m_enableReg &= ~ENABLE_LED1_EN;
            break;
        case LedChannel::LED2:
            m_enableReg &= ~ENABLE_LED2_EN;
            break;
        case LedChannel::BOTH:
            m_enableReg &= ~(ENABLE_LED1_EN | ENABLE_LED2_EN);
            // If both LEDs disabled, go to standby mode and clear 
            // torch and strobe bits
            if (!(m_enableReg & (ENABLE_LED1_EN | ENABLE_LED2_EN))) {
                m_enableReg &= ~ENABLE_MODE_MASK;
                m_enableReg |= ENABLE_MODE_STANDBY;
                m_enableReg &= ~(ENABLE_TORCH | ENABLE_STROBE);
            }
            break;
    }
    
    return updateEnableRegister();
}

bool LM3643::isEnabled(LedChannel channel) const {
    switch (channel) {
        case LedChannel::LED1:
            return (m_enableReg & ENABLE_LED1_EN) != 0;
        case LedChannel::LED2:
            return (m_enableReg & ENABLE_LED2_EN) != 0;
        case LedChannel::BOTH:
            return (m_enableReg & (ENABLE_LED1_EN | ENABLE_LED2_EN)) == 
                   (ENABLE_LED1_EN | ENABLE_LED2_EN);
    }
    return false;
}

LM3643::LedMode LM3643::getMode(LedChannel channel) const {
    if (!isEnabled(channel)) {
        return LedMode::STANDBY;
    }
    
    uint8_t mode = m_enableReg & ENABLE_MODE_MASK;
    switch (mode) {
        case ENABLE_MODE_TORCH:
            return LedMode::TORCH;
        case ENABLE_MODE_FLASH:
            return LedMode::FLASH;
        default:
            return LedMode::STANDBY;
    }
}

bool LM3643::setTxMask(bool enable) {
    if (enable) {
        m_enableReg |= ENABLE_TX_MASK;
    } else {
        m_enableReg &= ~ENABLE_TX_MASK;
    }
    return updateEnableRegister();
}

bool LM3643::setStrobeEnable(LedChannel channel, bool isEdgeMode) {
    m_enableReg |= ENABLE_STROBE;
    if (isEdgeMode) {
        m_enableReg |= ENABLE_EDGE;
    } else {
        m_enableReg &= ~ENABLE_EDGE;
    }
    // Enable the appropriate LED(s)
    switch (channel) {
        case LedChannel::LED1:
            m_enableReg |= ENABLE_LED1_EN;
            break;
        case LedChannel::LED2:
            m_enableReg |= ENABLE_LED2_EN;
            break;
        case LedChannel::BOTH:
            m_enableReg |= (ENABLE_LED1_EN | ENABLE_LED2_EN);
            break;
    }
    
    return updateEnableRegister();
}

bool  LM3643::setTorchEnable(LedChannel channel) {
    m_enableReg |= ENABLE_TORCH;
    // Enable the appropriate LED(s)
    switch (channel) {
        case LedChannel::LED1:
            m_enableReg |= ENABLE_LED1_EN;
            break;
        case LedChannel::LED2:
            m_enableReg |= ENABLE_LED2_EN;
            break;
        case LedChannel::BOTH:
            m_enableReg |= (ENABLE_LED1_EN | ENABLE_LED2_EN);
            break;
    }

    return updateEnableRegister();
}

bool LM3643::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return m_i2cMgr->write(m_address, data, 2);
}

bool LM3643::readRegister(uint8_t reg, uint8_t& value) {
    return m_i2cMgr->writeRead(m_address, &reg, 1, &value, 1);
}

bool LM3643::updateEnableRegister() {
    return writeRegister(REG_ENABLE, m_enableReg);
}