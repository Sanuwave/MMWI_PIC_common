#ifndef AS7331_H
#define AS7331_H

#include <cstdint>
#include <mutex>

// AS7331 Spectral UV Sensor Driver
// Channels: UVA (320–400nm), UVB (280–320nm), UVC (200–280nm)

class AS7331 {
public:
    // -------------------------------------------------------------------------
    // Public types
    // -------------------------------------------------------------------------

    // ADC gain: register value 0x00 = 2048x (max), 0x0B = 1x (min)
    // Per datasheet CREG1 Figure 48: value 0000b → GAIN=2048x, 1011b → GAIN=1x
    enum class Gain : uint8_t {
        GAIN_2048  = 0x00,   // 2048x  (max sensitivity)
        GAIN_1024  = 0x01,   // 1024x
        GAIN_512   = 0x02,   //  512x
        GAIN_256   = 0x03,   //  256x
        GAIN_128   = 0x04,   //  128x
        GAIN_64    = 0x05,   //   64x
        GAIN_32    = 0x06,   //   32x
        GAIN_16    = 0x07,   //   16x
        GAIN_8     = 0x08,   //    8x
        GAIN_4     = 0x09,   //    4x
        GAIN_2     = 0x0A,   //    2x
        GAIN_1     = 0x0B    //    1x  (min sensitivity)
    };

    // Integration time: register value n → 2^n ms  (0→1ms … 14→16384ms)
    enum class IntTime : uint8_t {
        TIME_1MS    = 0x00,
        TIME_2MS    = 0x01,
        TIME_4MS    = 0x02,
        TIME_8MS    = 0x03,
        TIME_16MS   = 0x04,
        TIME_32MS   = 0x05,
        TIME_64MS   = 0x06,
        TIME_128MS  = 0x07,
        TIME_256MS  = 0x08,
        TIME_512MS  = 0x09,
        TIME_1024MS = 0x0A,
        TIME_2048MS = 0x0B,
        TIME_4096MS = 0x0C,
        TIME_8192MS = 0x0D
    };

    // Measurement mode – written into CREG3[7:6]
    enum class MeasMode : uint8_t {
        CONT = 0x00,   // Continuous
        CMD  = 0x01,   // Command (single-shot, recommended for low power)
        SYNS = 0x02,   // Synchronised via SYN pin (rising edge start)
        SYND = 0x03    // Synchronised via SYN pin (falling edge start+stop)
    };

    struct UvData {
        uint16_t uva;    // Raw UVA counts  (MRES1)
        uint16_t uvb;    // Raw UVB counts  (MRES2)
        uint16_t uvc;    // Raw UVC counts  (MRES3)
        float    tempC;  // Die temperature in °C
        bool     valid;  // True when all fields are populated
    };

    // -------------------------------------------------------------------------
    // Singleton interface
    // -------------------------------------------------------------------------
    static AS7331& getInstance();

    AS7331(const AS7331&)            = delete;
    AS7331& operator=(const AS7331&) = delete;
    AS7331(AS7331&&)                 = delete;
    AS7331& operator=(AS7331&&)      = delete;

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    // Initialise: opens I2cMgr (if not already open), resets the device,
    // applies the supplied configuration, and switches to CONF state.
    // Returns false on any I2C or configuration error.
    bool init(uint8_t i2cAddr  = DEFAULT_I2C_ADDR,
              Gain    gain     = Gain::GAIN_64,
              IntTime intTime  = IntTime::TIME_64MS,
              MeasMode mode    = MeasMode::CMD);

    void deinit();
    bool isInitialized() const;

    // -------------------------------------------------------------------------
    // Configuration (call before or after init – re-applies to hardware)
    // -------------------------------------------------------------------------
    bool setGain(Gain gain);
    bool setIntegrationTime(IntTime t);
    bool setMeasurementMode(MeasMode mode);

    // -------------------------------------------------------------------------
    // Measurement
    // -------------------------------------------------------------------------

    // Trigger a single CMD-mode measurement and block until data is ready.
    // Returns populated UvData; valid==false on error.
    UvData measure();

    // For CONT mode: just read the latest results without triggering.
    UvData readResults();

    // -------------------------------------------------------------------------
    // Utility
    // -------------------------------------------------------------------------

    // Convert raw counts to irradiance (µW/cm²).
    // Requires knowledge of the current gain and integration time.
    float rawToIrradiance(uint16_t raw, uint8_t channel);

    bool softwareReset();

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------
    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x74;
    static constexpr uint8_t DEFAULT_OSR_REG  = 0x02; // Post-reset: DOS=010b, PD=0, SS=0

private:
    AS7331();
    ~AS7331() = default;

    // -------------------------------------------------------------------------
    // Register map
    // -------------------------------------------------------------------------

    // -- Configuration-state registers (DOS = CONF) --
    static constexpr uint8_t REG_OSR    = 0x00; // Operational State Register
    static constexpr uint8_t REG_AGEN   = 0x02; // Device ID / API generation
    static constexpr uint8_t REG_CREG1  = 0x06; // Gain + integration time
    static constexpr uint8_t REG_CREG2  = 0x07; // En/disable channels, divider
    static constexpr uint8_t REG_CREG3  = 0x08; // Meas. mode, standby, RDYOD
    static constexpr uint8_t REG_BREAK  = 0x09; // Break time between cycles
    static constexpr uint8_t REG_EDGES  = 0x0A; // Number of SYN edges (SYND)
    static constexpr uint8_t REG_OPTREG = 0x0B; // Optional output register

    // -- Measurement-state registers (DOS = MEAS) --
    // Per datasheet Figure 54: addr 0=OSR/STATUS, 1=TEMP, 2=MRES1(UVA),
    //                          3=MRES2(UVB), 4=MRES3(UVC)
    static constexpr uint8_t REG_STATUS    = 0x00; // STATUS (second byte at addr 0)
    static constexpr uint8_t REG_TEMP_LSB  = 0x01; // TEMP result (16-bit, LSB first)
    static constexpr uint8_t REG_UVA_LSB   = 0x02; // MRES1 – UVA result LSB
    static constexpr uint8_t REG_UVB_LSB   = 0x03; // MRES2 – UVB result LSB
    static constexpr uint8_t REG_UVC_LSB   = 0x04; // MRES3 – UVC result LSB

    // OSR bit fields
    static constexpr uint8_t OSR_DOS_CONF  = 0x02; // bits[2:0] → CONF state (010b)
    static constexpr uint8_t OSR_DOS_MEAS  = 0x03; // bits[2:0] → MEAS state (011b)
    static constexpr uint8_t OSR_SS        = (1u << 7); // Start/Stop
    static constexpr uint8_t OSR_PD        = (1u << 6); // Power-down
    // SW_RES must be written with a valid DOS to avoid NOP; use 0x0A (SW_RES|DOS_CONF)
    static constexpr uint8_t OSR_SW_RES    = 0x0A;      // bit3=SW_RES, bits[1:0]=010b (CONF)

    // STATUS register bit fields (read at addr 0x00 in MEAS state, second byte)
    static constexpr uint8_t STATUS_NDATA    = (1u << 3); // New data ready
    static constexpr uint8_t STATUS_NOTREADY = (1u << 2); // Inverted READY pin

    // CREG3 bit fields
    static constexpr uint8_t CREG3_MMODE_SHIFT = 6;        // MMODE at bits [7:6]
    static constexpr uint8_t CREG3_RDYOD       = (1u << 3);
    static constexpr uint8_t CREG3_SB          = (1u << 4); // Standby

    // Datasheet sensitivity constants (nW·s/cm² per count) – Table 8
    // Index 0 = UVA, 1 = UVB, 2 = UVC
    static constexpr float SENSITIVITY_UV[3] = { 0.000896f, 0.000950f, 0.000544f };

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------
    bool writeReg(uint8_t reg, uint8_t val);
    bool readReg(uint8_t reg, uint8_t& val);
    bool readWord(uint8_t lsbReg, uint16_t& val);

    bool enterConfState();
    bool enterMeasState();
    bool waitForDataReady(uint32_t timeoutMs);
    bool applyConfig();

    uint8_t  m_addr;
    Gain     m_gain;
    IntTime  m_intTime;
    MeasMode m_measMode;
    bool     m_initialised;

    mutable std::mutex m_mutex;
};

#endif // AS7331_H