#include <stdexcept>
#include <cstdio>
#include <iostream>
#include "ToFSensor.h"

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------

ToFSensor& ToFSensor::getInstance() {
    static ToFSensor instance;
    return instance;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void ToFSensor::init(uint8_t i2c_address, uint32_t timing_ms) {
    sensor_.reset();

    auto s = std::make_unique<VL53L4CD>(i2c_address);

    uint16_t id = s->getSensorId();
    if (id != 0xEBAA) {
        char buf[48];
        snprintf(buf, sizeof(buf), "VL53L4CD: unexpected sensor ID 0x%04X", id);
        throw std::runtime_error(buf);
    }

    s->sensorInit();
    s->setRangeTiming(timing_ms, 0);
    sensor_ = std::move(s);

    std::cout << "[VL53L4CD] Initialized at 0x"
              << std::hex << static_cast<int>(i2c_address) << std::dec << std::endl;
}

bool ToFSensor::isInit() const { return sensor_ != nullptr; }

// ---------------------------------------------------------------------------
// Ranging
// ---------------------------------------------------------------------------

void    ToFSensor::startRanging()   { sensor_->startRanging(); }
void    ToFSensor::stopRanging()    { sensor_->stopRanging(); }
bool    ToFSensor::isDataReady()    { return sensor_->isDataReady(); }
void    ToFSensor::clearInterrupt() { sensor_->clearInterrupt(); }

ToFSensor::Results ToFSensor::getResult() { return sensor_->getResult(); }

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

void ToFSensor::setRangeTiming(uint32_t budget_ms, uint32_t inter_ms) {
    sensor_->setRangeTiming(budget_ms, inter_ms);
}

void ToFSensor::getRangeTiming(uint32_t& budget_ms, uint32_t& inter_ms) const {
    sensor_->getRangeTiming(budget_ms, inter_ms);
}

// ---------------------------------------------------------------------------
// Correction settings
// ---------------------------------------------------------------------------

void     ToFSensor::setOffset(int16_t mm)             { sensor_->setOffset(mm); }
int16_t  ToFSensor::getOffset() const                 { return sensor_->getOffset(); }
void     ToFSensor::setXtalk(uint16_t kcps)           { sensor_->setXtalk(kcps); }
uint16_t ToFSensor::getXtalk() const                  { return sensor_->getXtalk(); }
void     ToFSensor::setSignalThreshold(uint16_t kcps) { sensor_->setSignalThreshold(kcps); }
uint16_t ToFSensor::getSignalThreshold() const        { return sensor_->getSignalThreshold(); }
void     ToFSensor::setSigmaThreshold(uint16_t mm)    { sensor_->setSigmaThreshold(mm); }
uint16_t ToFSensor::getSigmaThreshold() const         { return sensor_->getSigmaThreshold(); }

void ToFSensor::setDetectionThresholds(uint16_t low, uint16_t high, uint8_t window) {
    sensor_->setDetectionThresholds(low, high, window);
}

void ToFSensor::getDetectionThresholds(uint16_t& low, uint16_t& high, uint8_t& window) const {
    sensor_->getDetectionThresholds(low, high, window);
}

// ---------------------------------------------------------------------------
// Temperature compensation
// ---------------------------------------------------------------------------

void ToFSensor::updateTemperatureCalibration() { sensor_->startTemperatureUpdate(); }

// ---------------------------------------------------------------------------
// Factory calibration
// ---------------------------------------------------------------------------

int16_t ToFSensor::calibrateOffset(int16_t target_mm, int16_t nb_samples) {
    return sensor_->calibrateOffset(target_mm, nb_samples);
}

uint16_t ToFSensor::calibrateXtalk(int16_t target_mm, int16_t nb_samples) {
    return sensor_->calibrateXtalk(target_mm, nb_samples);
}

// ---------------------------------------------------------------------------
// Misc
// ---------------------------------------------------------------------------

void     ToFSensor::setI2CAddress(uint8_t addr) { sensor_->setI2CAddress(addr); }
uint16_t ToFSensor::getSensorId() const         { return sensor_->getSensorId(); }
uint8_t  ToFSensor::i2cAddress() const          { return sensor_->i2cAddress(); }