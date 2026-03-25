#include "I2cMgr.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <cstring>

I2cMgr::I2cMgr() 
    : m_fd(-1)
    , m_currentAddress(0xFF)
    , m_devicePath("")
    , m_resetLine(Gpio::GPIO::GPIO_I2C_RESET, Gpio::Direction::OUTPUT) {
}

I2cMgr::~I2cMgr() {
    close();
}

I2cMgr& I2cMgr::getInstance() {
    static I2cMgr instance;
    return instance;
}

bool I2cMgr::open(const std::string& device) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Close existing connection if open
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
     
    // Open I2C device
    m_fd = ::open(device.c_str(), O_RDWR);
    if (m_fd < 0) {
        std::cerr << "Failed to open I2C device " << device 
                  << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    m_devicePath = device;
    m_currentAddress = 0xFF; // Reset current address

    // Reset the bus and all attached
    resetI2cBus();
    
    std::cout << "I2C device " << device << " opened successfully" << std::endl;
    return true;
}

void I2cMgr::close() {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
        m_currentAddress = 0xFF;
        std::cout << "I2C device closed" << std::endl;
    }
}

bool I2cMgr::isOpen() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_fd >= 0;
}

bool I2cMgr::setSlaveAddress(uint8_t address) {
    // Only set if different from current address to reduce ioctl calls
    if (m_currentAddress == address) {
        return true;
    }
    
    if (ioctl(m_fd, I2C_SLAVE, address) < 0) {
        std::cerr << "Failed to set I2C slave address 0x" 
                  << std::hex << static_cast<int>(address) << std::dec
                  << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    m_currentAddress = address;
    return true;
}
void I2cMgr::resetI2cBus(void) {
    m_resetLine.set_low();
    usleep(50000);
    m_resetLine.set_high();
    usleep(50000);
}

bool I2cMgr::write(uint8_t address, const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_fd < 0) {
        std::cerr << "I2C device not open" << std::endl;
        return false;
    }
    
    if (!data || len == 0) {
        std::cerr << "Invalid write parameters" << std::endl;
        return false;
    }
    
    // Set slave address
    if (!setSlaveAddress(address)) {
        return false;
    }
    
    // Write data
    ssize_t result = ::write(m_fd, data, len);
    if (result != static_cast<ssize_t>(len)) {
        std::cerr << "I2C write failed to address 0x" 
                  << std::hex << static_cast<int>(address) << std::dec
                  << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    return true;
}

bool I2cMgr::read(uint8_t address, uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_fd < 0) {
        std::cerr << "I2C device not open" << std::endl;
        return false;
    }
    
    if (!data || len == 0) {
        std::cerr << "Invalid read parameters" << std::endl;
        return false;
    }
    
    // Set slave address
    if (!setSlaveAddress(address)) {
        return false;
    }
    
    // Read data
    ssize_t result = ::read(m_fd, data, len);
    if (result != static_cast<ssize_t>(len)) {
        std::cerr << "I2C read failed from address 0x" 
                  << std::hex << static_cast<int>(address) << std::dec
                  << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    return true;
}

bool I2cMgr::writeRead(uint8_t address, const uint8_t* writeData, size_t writeLen,
                       uint8_t* readData, size_t readLen) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_fd < 0) {
        std::cerr << "I2C device not open" << std::endl;
        return false;
    }
    
    if (!writeData || writeLen == 0 || !readData || readLen == 0) {
        std::cerr << "Invalid writeRead parameters" << std::endl;
        return false;
    }
    
    // Set slave address
    if (!setSlaveAddress(address)) {
        return false;
    }
    
    // Write data
    ssize_t result = ::write(m_fd, writeData, writeLen);
    if (result != static_cast<ssize_t>(writeLen)) {
        std::cerr << "I2C write phase of writeRead failed to address 0x" 
                  << std::hex << static_cast<int>(address) << std::dec
                  << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    // Read data
    result = ::read(m_fd, readData, readLen);
    if (result != static_cast<ssize_t>(readLen)) {
        std::cerr << "I2C read phase of writeRead failed from address 0x" 
                  << std::hex << static_cast<int>(address) << std::dec
                  << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    return true;
}