#ifndef GPIO_H
#define GPIO_H

#include <cstdint>
#include <string>

namespace Gpio {

enum class Direction {
    INPUT,
    OUTPUT
};

enum class Pull {
    NONE,
    UP,
    DOWN
};

enum class Edge {
    NONE,
    RISING,
    FALLING,
    BOTH
};

enum class Value {
    LOW = 0,
    HIGH = 1
};

class GPIO {
private:
    int chip_fd;
    int line_fd;
    unsigned int pin;
    Direction dir;
    std::string chip_path;
    bool is_open;

public:
    // Resolves the chip by label at construction time so the correct
    // /dev/gpiochipN is used regardless of kernel enumeration order.
    // Defaults to "pinctrl-rp1" which is the Pi 5 main 40-pin GPIO bank.
    GPIO(unsigned int pin_number, Direction direction = Direction::INPUT,
         const std::string& chip_label = "pinctrl-rp1");

    ~GPIO();

    // Delete copy constructor and assignment
    GPIO(const GPIO&) = delete;
    GPIO& operator=(const GPIO&) = delete;

    // Move constructor and assignment
    GPIO(GPIO&& other) noexcept;
    GPIO& operator=(GPIO&& other) noexcept;

    void open();
    void close();

    void write(Value value);
    Value read();

    void set_high();
    void set_low();
    void toggle();

    bool is_high();
    bool is_low();

    unsigned int get_pin() const;
    Direction get_direction() const;

    // Scans /dev/gpiochip0..31 and returns the path of the chip whose
    // kernel label matches the given string.  Throws std::runtime_error
    // if no matching chip is found.
    static std::string chipPathByLabel(const std::string& label);

    static constexpr uint8_t GPIO_I2C_RESET = 18;

    static constexpr uint8_t GPIO_TORCH_A_NS = 4;
    static constexpr uint8_t GPIO_STROBE_A_NS = 17;

    static constexpr uint8_t GPIO_TORCH_B_NS = 5;
    static constexpr uint8_t GPIO_STROBE_B_NS = 6;

    static constexpr uint8_t GPIO_TORCH_A_WE = 23;
    static constexpr uint8_t GPIO_STROBE_A_WE = 24;

    static constexpr uint8_t GPIO_TORCH_B_WE = 25;
    static constexpr uint8_t GPIO_STROBE_B_WE = 7; // todo - replace with different GPIO when camera SPI CS is moved to a different pin
};

} // namespace Gpio

#endif // GPIO_H
