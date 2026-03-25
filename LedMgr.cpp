#include "LedMgr.h"
#include "I2cMgr.h"
#include "PCA9545A.h"
#include <iostream>
#include <string>
#include <QDebug>
#include <unistd.h>
#include <stdio.h>

LedMgr::LedMgr() 
    : m_ledDriver0(nullptr)
    , m_ledDriverM0(nullptr)
    , m_ledDriver1(nullptr)
    , m_ledDriverM1(nullptr)
    , m_ledDriver2(nullptr)
    , m_ledDriverM2(nullptr)
    , m_ledDriver3(nullptr)
    , m_ledDriverM3(nullptr)
    , m_ledDriver4(nullptr)
    , m_ledDriverM4(nullptr)
    , m_ledDriver5(nullptr)
    , m_ledDriverM5(nullptr)
    , m_ledDriver6(nullptr)
    , m_ledDriverM6(nullptr)
    , m_ledDriver7(nullptr)
    , m_ledDriverM7(nullptr)
    , m_currentLedDriver(nullptr)
    , m_i2cSwitchDriver(nullptr)
    , m_i2cSwitchDriverAlt(nullptr)
    , m_currentI2cSwitchDriver(nullptr)
    , m_i2cMgr(nullptr)
    , m_initialized(false) 
    , m_ledChannel(LM3643::LedChannel::BOTH) 
    , m_ledChannelInfoTable
    {{
        // North boards
        {false, true, 1, LM3643::LedChannel::LED1, false},     // LED0
        {false, true, 1, LM3643::LedChannel::LED2, false},     // LED1

        {true, true, 1, LM3643::LedChannel::LED1, false},    // LED2
        {true, true, 1, LM3643::LedChannel::LED2, false},    // LED3

        {true, true, 0, LM3643::LedChannel::LED2, false},     // LED4
        {true, true, 0, LM3643::LedChannel::LED1, false},     // LED5

        {false, true, 0, LM3643::LedChannel::LED2, false},    // LED6
        {false, true, 0, LM3643::LedChannel::LED1, false},    // LED7

        ////////////////////////////////////////////////////////

        // East boards
        {true, false, 2, LM3643::LedChannel::LED2, false},  // LED8
        {true, false, 2, LM3643::LedChannel::LED1, false},  // LED9

        {false, false, 2, LM3643::LedChannel::LED2, false}, // LED10
        {false, false, 2, LM3643::LedChannel::LED1, false}, // LED11

        {false, false, 3, LM3643::LedChannel::LED1, false}, // LED12
        {false, false, 3, LM3643::LedChannel::LED2, false}, // LED13

        {true, false, 3, LM3643::LedChannel::LED1, false}, // LED14
        {true, false, 3, LM3643::LedChannel::LED2, false}, // LED15

        ////////////////////////////////////////////////////////

        // South boards
        {false, true, 2, LM3643::LedChannel::LED1, false}, // LED16
        {false, true, 2, LM3643::LedChannel::LED2, false}, // LED17

        {true, true, 2, LM3643::LedChannel::LED1, false}, // LED18
        {true, true, 2, LM3643::LedChannel::LED2, false}, // LED19

        {true, true, 3, LM3643::LedChannel::LED2, false}, // LED20
        {true, true, 3, LM3643::LedChannel::LED1, false}, // LED21

        {false, true, 3, LM3643::LedChannel::LED2, false}, // LED22
        {false, true, 3, LM3643::LedChannel::LED1, false}, // LED23

        ////////////////////////////////////////////////////////

        // West boards
        {true, false, 1, LM3643::LedChannel::LED2, false}, // LED24
        {true, false, 1, LM3643::LedChannel::LED1, false}, // LED25

        {false, false, 1, LM3643::LedChannel::LED2, false}, // LED26
        {false, false, 1, LM3643::LedChannel::LED1, false}, // LED27

        {false, false, 0, LM3643::LedChannel::LED1, false}, // LED28
        {false, false, 0, LM3643::LedChannel::LED2, false}, // LED29

        {true, false, 0, LM3643::LedChannel::LED1, false}, // LED30
        {true, false, 0, LM3643::LedChannel::LED2, false}, // LED31
    }} 
    , m_strobeLineInfoTable {
        Gpio::GPIO::GPIO_STROBE_A_NS,
        Gpio::GPIO::GPIO_STROBE_B_NS,
        Gpio::GPIO::GPIO_STROBE_A_WE,
        Gpio::GPIO::GPIO_STROBE_B_WE
    } 
    
    , m_torchLineInfoTable {
        Gpio::GPIO::GPIO_TORCH_A_NS,
        Gpio::GPIO::GPIO_TORCH_B_NS,
        Gpio::GPIO::GPIO_TORCH_A_WE,
        Gpio::GPIO::GPIO_TORCH_B_WE,
    }  {
}

LedMgr::~LedMgr() {
    if (m_initialized) {
        // todo turnAllOff();        
    }
}

LedMgr& LedMgr::getInstance() {
    static LedMgr instance;
    return instance;
}

bool LedMgr::initialize() {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_initialized) {
        return true;
    }
    
    try {        
        // Get I2C manager singleton
        m_i2cMgr = &I2cMgr::getInstance();
        
        // Ensure I2C is open
        if (!m_i2cMgr->open() ||!m_i2cMgr->isOpen()) {
            std::cerr << "LedMgr: I2C device could not open" << std::endl;
            return false;
        }

        // Create 2 PCA9545A driver instances
        m_i2cSwitchDriver = new PCA9545A(m_i2cMgr, PCA9545A::DEFAULT_ADDRESS);
        m_i2cSwitchDriverAlt = new PCA9545A(m_i2cMgr, PCA9545A::ALTERNATE_ADDRESS);
        
        // Initialize the i2c switch drivers
        if (!m_i2cSwitchDriver->init()) {
            std::cerr << "LedMgr: Failed to initialize default PCA9545A driver" << std::endl;            
            return false;
        }
        if (!m_i2cSwitchDriverAlt->init()) {
            std::cerr << "LedMgr: Failed to initialize alt PCA9545A driver" << std::endl;            
            return false;
        }
        
        // Create LM3643 driver instances
        m_ledDriver0 = new LM3643(m_i2cMgr, LM3643::DEFAULT_I2C_ADDRESS);
        m_ledDriverM0 = new LM3643(m_i2cMgr, LM3643::ALT_I2C_ADDRESS);
        m_ledDriver1 = new LM3643(m_i2cMgr, LM3643::DEFAULT_I2C_ADDRESS);
        m_ledDriverM1 = new LM3643(m_i2cMgr, LM3643::ALT_I2C_ADDRESS);
        m_ledDriver2 = new LM3643(m_i2cMgr, LM3643::DEFAULT_I2C_ADDRESS);
        m_ledDriverM2 = new LM3643(m_i2cMgr, LM3643::ALT_I2C_ADDRESS);
        m_ledDriver3 = new LM3643(m_i2cMgr, LM3643::DEFAULT_I2C_ADDRESS);
        m_ledDriverM3 = new LM3643(m_i2cMgr, LM3643::ALT_I2C_ADDRESS);
        m_ledDriver4 = new LM3643(m_i2cMgr, LM3643::DEFAULT_I2C_ADDRESS);
        m_ledDriverM4 = new LM3643(m_i2cMgr, LM3643::ALT_I2C_ADDRESS);
        m_ledDriver5 = new LM3643(m_i2cMgr, LM3643::DEFAULT_I2C_ADDRESS);
        m_ledDriverM5 = new LM3643(m_i2cMgr, LM3643::ALT_I2C_ADDRESS);
        m_ledDriver6 = new LM3643(m_i2cMgr, LM3643::DEFAULT_I2C_ADDRESS);
        m_ledDriverM6 = new LM3643(m_i2cMgr, LM3643::ALT_I2C_ADDRESS);
        m_ledDriver7 = new LM3643(m_i2cMgr, LM3643::DEFAULT_I2C_ADDRESS);
        m_ledDriverM7 = new LM3643(m_i2cMgr, LM3643::ALT_I2C_ADDRESS);
        
        // Initialize the driverS
        if (!m_ledDriver0->init()) { return false; }
        if (!m_ledDriverM0->init()) { return false; }
        if (!m_ledDriver1->init()) { return false; }
        if (!m_ledDriverM1->init()) { return false; }
        if (!m_ledDriver2->init()) { return false; }
        if (!m_ledDriverM2->init()) { return false; }
        if (!m_ledDriver3->init()) { return false; }
        if (!m_ledDriverM3->init()) { return false; }
        if (!m_ledDriver4->init()) { return false; }
        if (!m_ledDriverM4->init()) { return false; }
        if (!m_ledDriver5->init()) { return false; }
        if (!m_ledDriverM5->init()) { return false; }
        if (!m_ledDriver6->init()) { return false; }
        if (!m_ledDriverM6->init()) { return false; }
        if (!m_ledDriver7->init()) { return false; }
        if (!m_ledDriverM7->init()) { return false; }

        // Reset all LEDs
        if (!resetLedStates()) {
            std::cerr << "LedMgr: Failed to reset LEDs" << std::endl;
            return false;
        }
        
        m_initialized = true;        
        
        std::cout << "LedMgr: Initialized successfully" << std::endl;
        return true;;
        
    } catch (const std::exception& e) {
        std::cerr << "LedMgr: Exception during initialization: " << e.what() << std::endl;        
        return false;
    }
}

bool LedMgr::isInitialized() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialized;
}

bool LedMgr::setTorchMode(LedId ledId) {
    std::lock_guard<std::mutex> lock(m_mutex);    
    
    if (!m_initialized) {
        std::cerr << "LedMgr: Not initialized" << std::endl;
        return false;
    }

    if (!setDriversAndChannel(ledId)) {
        std::cerr << "LedMgr: Couldn't set Driver/Channel" << std::endl;
        return false;
    }
    
    // Enable torch mode
    if (!m_currentLedDriver->enableTorch(m_ledChannel)) {
        std::cerr << "LedMgr: Failed to enable torch" << std::endl;
        return false;
    }
    
    return true;
}

bool LedMgr::setFlashMode(LedId ledId) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized) {
        std::cerr << "LedMgr: Not initialized" << std::endl;
        return false;
    }

    if (!setDriversAndChannel(ledId)) {
        std::cerr << "LedMgr: Couldn't set Driver/Channel" << std::endl;
        return false;
    }
    
    // Enable flash mode
    if (!m_currentLedDriver->enableFlash(m_ledChannel)) {
        std::cerr << "LedMgr: Failed to enable flash" << std::endl;
        return false;
    }
    
    return true;
}

bool LedMgr::setFlashTimeout(LedId ledId, uint16_t timeout_ms) {
    std::lock_guard<std::mutex> lock(m_mutex);

    LM3643::LedFlashTimeoutId timeout = LM3643::LedFlashTimeoutId::NUM_FLASH_TIMEOUT_IDS;
    
    if (!m_initialized) {
        return false;
    }

    if (!setDriversAndChannel(ledId)) {
        std::cerr << "LedMgr: Couldn't set Driver/Channel" << std::endl;
        return false;
    }

    // Determine flash timeout value based on ms value
    if (10 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_10MS;
    }
    else if (20 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_20MS;
    }
    else if (30 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_30MS;
    }
    else if (40 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_40MS;
    }
    else if (50 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_50MS;
    }
    else if (60 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_60MS;
    }
    else if (70 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_70MS;
    }
    else if (80 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_80MS;
    }
    else if (90 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_90MS;
    }
    else if (100 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_100MS;
    }
    else if (150 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_150MS;
    }
    else if (200 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_200MS;
    }
    else if (250 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_250MS;
    }
    else if (300 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_300MS;
    }
    else if (350 >= timeout_ms) {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_350MS;
    }
    else {
        timeout = LM3643::LedFlashTimeoutId::TIMEOUT_400MS;
    }
    
    return m_currentLedDriver->setFlashTimeout(timeout);
}

bool LedMgr::setFlashTorchBrightness(LedId ledId, uint8_t brightness) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized) {
        return false;
    }

    if (!setDriversAndChannel(ledId)) {
        std::cerr << "LedMgr: Couldn't set Driver/Channel" << std::endl;
        return false;
    }

    // Set flash brightness
    if (!m_currentLedDriver->setFlashBrightness(m_ledChannel, brightness)) {
        std::cerr << "LedMgr: Failed to set flash brightness" << std::endl;
        return false;
    }

    // Set torch brightness
    if (!m_currentLedDriver->setTorchBrightness(m_ledChannel, brightness)) {
        std::cerr << "LedMgr: Failed to set flash brightness" << std::endl;
        return false;
    }
    return true;    
}

bool LedMgr::enableHwStrobe(LedHwStrobeLineId line)
{
    if (!m_initialized || line >= LedHwStrobeLineId::NUM_HW_STROBE_LINE_IDS) {
        return false;
    }
    
    Gpio::GPIO gpioLine(m_strobeLineInfoTable[static_cast<uint8_t>(line)], 
                        Gpio::Direction::OUTPUT);
    gpioLine.set_high();
    usleep(LedMgr::STROBE_PULSE_TIMEOUT_US);
    gpioLine.set_low();

    return true; 
}

bool LedMgr::enableHwTorch(LedHwTorchLineId line)
{
    if (!m_initialized || line >= LedHwTorchLineId::NUM_HW_TORCH_LINE_IDS) {
        return false;
    }
    
    Gpio::GPIO gpioLine(m_torchLineInfoTable[static_cast<uint8_t>(line)], 
                        Gpio::Direction::OUTPUT);
    gpioLine.set_high();    

    return true; 
}

bool LedMgr::disableHwTorch(LedHwTorchLineId line)
{
    if (!m_initialized || line >= LedHwTorchLineId::NUM_HW_TORCH_LINE_IDS) {
        return false;
    }
    
    Gpio::GPIO gpioLine(m_torchLineInfoTable[static_cast<uint8_t>(line)], 
                        Gpio::Direction::OUTPUT);   
                        
    gpioLine.set_low();                        

    return true; 
}

bool LedMgr::turnOff(LedId ledId) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized) {
        return false;
    }

    if (!setDriversAndChannel(ledId)) {
        std::cerr << "LedMgr: Couldn't set Driver/Channel" << std::endl;
        return false;
    }
    
    if (!m_currentLedDriver->disable(m_ledChannel)) {
        return false;
    }
    
    return true;
}

bool LedMgr::resetLeds() {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized) {
        return false;
    }

    return resetLedStates();
}   

bool LedMgr::isLedEnabled(LedId ledId) {
    return m_ledChannelInfoTable[static_cast<uint8_t>(ledId)].isEnabled;
}

bool LedMgr::setStrobeEnable(LedId ledId) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized) {
        return false;
    }

    if (!setDriversAndChannel(ledId)) {
        std::cerr << "LedMgr: Couldn't set Driver/Channel" << std::endl;
        return false;
    }
    
    return m_currentLedDriver->setStrobeEnable(m_ledChannel);
}

bool LedMgr::setTorchEnable(LedId ledId) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized) {
        return false;
    }

    if (!setDriversAndChannel(ledId)) {
        std::cerr << "LedMgr: Couldn't set Driver/Channel" << std::endl;
        return false;
    }
    
    return m_currentLedDriver->setTorchEnable(m_ledChannel);
}

bool LedMgr::setTxMask(bool enable) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized) {
        return false;
    }
    
    return m_currentLedDriver->setTxMask(enable);
}

bool LedMgr::setDriversAndChannel(LedId ledId)
{
    uint8_t tableIndex = static_cast<uint8_t>(ledId);
    if (m_ledChannelInfoTable[tableIndex].isEnabled) {  
        m_currentI2cSwitchDriver->disableAllChannels();
        if (m_ledChannelInfoTable[tableIndex].isDefaultI2cSwitchDriver) {
            m_currentI2cSwitchDriver = m_i2cSwitchDriver;
        } 
        else {
            m_currentI2cSwitchDriver = m_i2cSwitchDriverAlt;
        }

        //Select switch channel
        if (!m_currentI2cSwitchDriver->selectChannel(m_ledChannelInfoTable[tableIndex].switchChannel)) {
                std::cerr << "LedMgr: Failed to select I2C Switch" << std::endl;                    
                return false;
        }
               
        // Delay to allow physical switch
        usleep(10000);

        // Set LED Driver
        m_currentLedDriver = getLedDriver(ledId);

        m_ledChannel = m_ledChannelInfoTable[tableIndex].ledChannel;        
    }
    else {
        std::cerr << "LedMgr: LED not enabled" << std::endl;
    }

    return true;;
}
bool LedMgr::resetLedStates() {
    // Loop through GPIO strobe/torch lines and set inactive (low)
    for (uint8_t i = 0; i < static_cast<uint8_t>(LedHwStrobeLineId::NUM_HW_STROBE_LINE_IDS) -1; ++i) {
        Gpio::GPIO gpio(m_strobeLineInfoTable[i], Gpio::Direction::OUTPUT);
        gpio.set_low();
    }

    for (uint8_t i = 0; i < static_cast<uint8_t>(LedHwTorchLineId::NUM_HW_TORCH_LINE_IDS); ++i) {
        Gpio::GPIO gpio(m_torchLineInfoTable[i], Gpio::Direction::OUTPUT);
        gpio.set_low();
    }
    
    // There are 8 driver boards, 4 with default address and 4 with alt address.
    // Cycle through the boards, via the 2 i2c switch drivers, and initial the devices.
    // 2 Switches.  Disable both switches prior.
    m_i2cSwitchDriver->disableAllChannels();
    m_i2cSwitchDriverAlt->disableAllChannels();
    m_currentI2cSwitchDriver = m_i2cSwitchDriver;
    for (uint8_t i = 0; i < 32; i+=2) {
        // Loop through the LedChannelInfoTable, 2 leds at a time (since 2 LEDs per
        // driver board) and attempt to initialize.
        m_currentI2cSwitchDriver->disableAllChannels();
        if (m_ledChannelInfoTable[i].isDefaultI2cSwitchDriver) {
            m_currentI2cSwitchDriver = m_i2cSwitchDriver;
        } 
        else {
            m_currentI2cSwitchDriver = m_i2cSwitchDriverAlt;
        }

        //Select switch channel
        if (!m_currentI2cSwitchDriver->selectChannel(m_ledChannelInfoTable[i].switchChannel)) {
                std::cerr << "LedMgr: Failed to select I2C Switch" << std::endl;
                return false;
        }
        usleep(10 * 1000);            
        // Verify channel is enabled:
        bool isEnabled = false;
        m_currentI2cSwitchDriver->isChannelEnabled(m_ledChannelInfoTable[i].switchChannel, isEnabled);
        if (!isEnabled) {
            qDebug() << "LedMgr: Failed to verify i2c switch to channel " << m_ledChannelInfoTable[i].switchChannel;
            return false;
        }

        // Delay to allow physical switch
        usleep(10 * 1000);

        // Try to initial the LED Driver board per this switch, channel, and driver
        m_currentLedDriver = getLedDriver(static_cast<LedId>(i));
        if (m_currentLedDriver->initDevice()) {
            m_ledChannelInfoTable[i].isEnabled = true;
            m_ledChannelInfoTable[i+1].isEnabled = true;
            std::cout << "LedMgr: LM3643 device (re)initialized" << std::endl;
        }            
    }

    return true;
}

LM3643* LedMgr::getLedDriver(LedId ledId)
{
    if (LedId::LED_GROUP_0 >= ledId) {
        if (m_ledChannelInfoTable[static_cast<uint8_t>(ledId)].isDefaultLedDriver) {return m_ledDriver0; }
        else {return m_ledDriverM0;}
    }
    if (LedId::LED_GROUP_1 >= ledId) {
        if (m_ledChannelInfoTable[static_cast<uint8_t>(ledId)].isDefaultLedDriver) {return m_ledDriver1; }
        else {return m_ledDriverM1;}
    }
    if (LedId::LED_GROUP_2 >= ledId) {
        if (m_ledChannelInfoTable[static_cast<uint8_t>(ledId)].isDefaultLedDriver) {return m_ledDriver2; }
        else {return m_ledDriverM2;}
    }
    if (LedId::LED_GROUP_3 >= ledId) {
        if (m_ledChannelInfoTable[static_cast<uint8_t>(ledId)].isDefaultLedDriver) {return m_ledDriver3; }
        else {return m_ledDriverM3;}
    }
    if (LedId::LED_GROUP_4 >= ledId) {
        if (m_ledChannelInfoTable[static_cast<uint8_t>(ledId)].isDefaultLedDriver) {return m_ledDriver4; }
        else {return m_ledDriverM4;}
    }
    if (LedId::LED_GROUP_5 >= ledId) {
        if (m_ledChannelInfoTable[static_cast<uint8_t>(ledId)].isDefaultLedDriver) {return m_ledDriver5; }
        else {return m_ledDriverM5;}
    }
    if (LedId::LED_GROUP_6 >= ledId) {
        if (m_ledChannelInfoTable[static_cast<uint8_t>(ledId)].isDefaultLedDriver) {return m_ledDriver6; }
        else {return m_ledDriverM6;}
    }
    if (m_ledChannelInfoTable[static_cast<uint8_t>(ledId)].isDefaultLedDriver) {return m_ledDriver7; }
    else {return m_ledDriverM7;}
}