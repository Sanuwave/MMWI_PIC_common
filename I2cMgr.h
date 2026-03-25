#ifndef I2C_MGR_H
#define I2C_MGR_H

#include <cstdint>
#include <string>
#include <mutex>
#include "Gpio.h"

class I2cMgr {
public:
    static I2cMgr& getInstance();
    
    I2cMgr(const I2cMgr&) = delete;
    I2cMgr& operator=(const I2cMgr&) = delete;
    I2cMgr(I2cMgr&&) = delete;
    I2cMgr& operator=(I2cMgr&&) = delete;
    
    bool open(const std::string& device = "/dev/i2c-1");
    void close();
    bool isOpen() const;
    
    bool write(uint8_t address, const uint8_t* data, size_t len);
    bool read(uint8_t address, uint8_t* data, size_t len);
    bool writeRead(uint8_t address, const uint8_t* writeData, size_t writeLen,
                   uint8_t* readData, size_t readLen);
    
private:
    I2cMgr();
    ~I2cMgr();
    
    bool setSlaveAddress(uint8_t address);
    void resetI2cBus(void);
    
    int m_fd;
    uint8_t m_currentAddress;
    Gpio::GPIO m_resetLine;
    std::string m_devicePath;
    mutable std::mutex m_mutex;    
};

#endif // I2C_MGR_H