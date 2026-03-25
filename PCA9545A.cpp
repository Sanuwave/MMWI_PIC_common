#include "PCA9545A.h"
#include "I2cMgr.h" 
#include <iostream>
#include <QDebug>

PCA9545A::PCA9545A(I2cMgr* i2cMgr, uint8_t address)
    : m_i2cMgr(i2cMgr)
    , m_address(address)
    , m_currentChannels(0x00)
{
}

PCA9545A::~PCA9545A() {
    // Disable all channels on destruction
    disableAllChannels();
}

bool PCA9545A::init() {
    // Verify device is responsive by reading control register
    uint8_t controlReg;
    if (!readControl(controlReg)) {
        return false;
    }
    else {
        std::cout << "PCA9545A I2C Switch initialized" << std::endl;
    
    }

    // Disable all channels on initialization
    return disableAllChannels();
}

bool PCA9545A::selectChannel(uint8_t channel) {
    uint8_t channelBit = 1 << static_cast<uint8_t>(channel);    
    return setChannelMask(channelBit);
}

bool PCA9545A::setChannelMask(uint8_t channelMask) {
    // Only bits 0-3 are valid for channel selection
    channelMask &= 0x0F;
    
    if (!writeControl(channelMask)) {
        return false;
    }

    m_currentChannels = channelMask;
    return true;
}

bool PCA9545A::disableAllChannels() {
    return setChannelMask(0x00);
}

bool PCA9545A::getChannelMask(uint8_t& channelMask) {
    uint8_t controlReg;
    if (!readControl(controlReg)) {
        return false;
    }

    // Extract channel bits (bits 0-3)
    channelMask = controlReg & 0x0F;
    m_currentChannels = channelMask;
    return true;
}

bool PCA9545A::isChannelEnabled(uint8_t channel, bool& isEnabled) {
    uint8_t channelMask = 1 << (channel);
    if (!getChannelMask(channelMask)) {
        return false;
    }

    uint8_t channelBit = 1 << (channel);
    isEnabled = (channelMask & channelBit) != 0;
    return true;
}

bool PCA9545A::writeControl(uint8_t value) {
    // Write single byte to control register
    return m_i2cMgr->write(m_address, &value, 1);
}

bool PCA9545A::readControl(uint8_t& value) {
    // Read single byte from control register
    return m_i2cMgr->read(m_address, &value, 1);
}