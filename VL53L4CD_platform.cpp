/**
 * @file VL53L4CD_platform.c
 * @brief VL53L4CD platform layer implementation using I2cMgr.
 *
 * The VL53L4CD uses 16-bit register addresses transmitted big-endian.
 * All multi-byte values are also big-endian on the wire.
 * Dev_t is the 8-bit I2C address stored as uint16_t.
 */

#include "VL53L4CD_platform.h"
#include "I2cMgr.h"
#include <unistd.h>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline void encode16(uint8_t* buf, uint16_t v) {
    buf[0] = static_cast<uint8_t>(v >> 8);
    buf[1] = static_cast<uint8_t>(v & 0xFF);
}

static inline void encode32(uint8_t* buf, uint32_t v) {
    buf[0] = static_cast<uint8_t>(v >> 24);
    buf[1] = static_cast<uint8_t>((v >> 16) & 0xFF);
    buf[2] = static_cast<uint8_t>((v >>  8) & 0xFF);
    buf[3] = static_cast<uint8_t>(v & 0xFF);
}

static inline uint16_t decode16(const uint8_t* buf) {
    return (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
}

static inline uint32_t decode32(const uint8_t* buf) {
    return (static_cast<uint32_t>(buf[0]) << 24)
         | (static_cast<uint32_t>(buf[1]) << 16)
         | (static_cast<uint32_t>(buf[2]) <<  8)
         |  static_cast<uint32_t>(buf[3]);
}

// All transactions: write 2-byte register address, then read or write payload.
static bool reg_write(Dev_t dev, uint16_t reg, const uint8_t* payload, size_t len) {
    uint8_t buf[2 + len];
    encode16(buf, reg);
    memcpy(buf + 2, payload, len);
    return I2cMgr::getInstance().write(static_cast<uint8_t>(dev), buf, sizeof(buf));
}

static bool reg_read(Dev_t dev, uint16_t reg, uint8_t* payload, size_t len) {
    uint8_t addr[2];
    encode16(addr, reg);
    return I2cMgr::getInstance().writeRead(
        static_cast<uint8_t>(dev), addr, sizeof(addr), payload, len);
}

// ---------------------------------------------------------------------------
// Platform API implementation
// ---------------------------------------------------------------------------

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t reg, uint8_t value) {
    return reg_write(dev, reg, &value, 1) ? 0 : 255;
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t reg, uint16_t value) {
    uint8_t buf[2];
    encode16(buf, value);
    return reg_write(dev, reg, buf, sizeof(buf)) ? 0 : 255;
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t reg, uint32_t value) {
    uint8_t buf[4];
    encode32(buf, value);
    return reg_write(dev, reg, buf, sizeof(buf)) ? 0 : 255;
}

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t reg, uint8_t* value) {
    return reg_read(dev, reg, value, 1) ? 0 : 255;
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t reg, uint16_t* value) {
    uint8_t buf[2];
    if (!reg_read(dev, reg, buf, sizeof(buf))) return 255;
    *value = decode16(buf);
    return 0;
}

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t reg, uint32_t* value) {
    uint8_t buf[4];
    if (!reg_read(dev, reg, buf, sizeof(buf))) return 255;
    *value = decode32(buf);
    return 0;
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t ms) {
    (void)dev;
    usleep(ms * 1000);
    return 0;
}