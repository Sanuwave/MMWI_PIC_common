#ifndef PCA9545A_H
#define PCA9545A_H
/**
 * Driver for TI PCA9545A 4-channel I2C switch
 * 
 * The PCA9545A is a 4-channel I2C multiplexer that allows multiple I2C devices
 * with the same address to coexist on the same bus by switching between channels.
 */

#include <cstdint>

class I2cMgr;

class PCA9545A {
public:
    // Default I2C addresses for PCA9545A (A0, A1, A2 all low)
    static constexpr uint8_t DEFAULT_ADDRESS   = 0x70;
    static constexpr uint8_t ALTERNATE_ADDRESS = 0x71;

    /**
     * Constructor
     * @param i2cMgr Reference to I2cMgr singleton instance
     * @param address I2C address of the PCA9545A (default 0x70)
     */
    explicit PCA9545A(I2cMgr* i2cMgr, uint8_t address = DEFAULT_ADDRESS);

    /**
     * Destructor - disables all channels
     */
    ~PCA9545A();

    // Prevent copying
    PCA9545A(const PCA9545A&) = delete;
    PCA9545A& operator=(const PCA9545A&) = delete;

    /**
     * Initialize the device
     * @return true if successful, false otherwise
     */
    bool init();

    /**
     * Enable a specific channel (disables all other channels)
     * @param channel Channel to enable (0-3)
     * @return true if successful, false otherwise
     */
    bool selectChannel(uint8_t channel);

    /**
     * Enable multiple channels simultaneously
     * @param channelMask Bit mask of channels to enable (bit 0-3)
     * @return true if successful, false otherwise
     */
    bool setChannelMask(uint8_t channelMask);

    /**
     * Disable all channels
     * @return true if successful, false otherwise
     */
    bool disableAllChannels();

    /**
     * Get the currently enabled channels
     * @param channelMask Output parameter for current channel mask
     * @return true if successful, false otherwise
     */
    bool getChannelMask(uint8_t& channelMask);

    /**
     * Check if a specific channel is enabled
     * @param channel Channel to check
     * @param isEnabled Output parameter indicating if channel is enabled
     * @return true if read successful, false otherwise
     */
    bool isChannelEnabled(uint8_t channel, bool& isEnabled);



private:
    I2cMgr* m_i2cMgr;           // Reference to I2C manager singleton
    uint8_t m_address;          // I2C address of the device
    uint8_t m_currentChannels;  // Cache of currently selected channels

    /**
     * Write control register
     * @param value Value to write
     * @return true if successful, false otherwise
     */
    bool writeControl(uint8_t value);

    /**
     * Read control register
     * @param value Output parameter for read value
     * @return true if successful, false otherwise
     */
    bool readControl(uint8_t& value);
};

#endif // PCA9545A_H