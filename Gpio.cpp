#include "Gpio.h"
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <cstring>
#include <stdexcept>

namespace Gpio {

// static
std::string GPIO::chipPathByLabel(const std::string& label)
{
    // Scan /dev/gpiochip0 through /dev/gpiochip31 and return the path
    // whose GPIO_GET_CHIPINFO_IOCTL label matches.
    for (int i = 0; i < 32; ++i) {
        std::string path = "/dev/gpiochip" + std::to_string(i);
        int fd = ::open(path.c_str(), O_RDONLY);
        if (fd < 0) continue;

        struct gpiochip_info info;
        memset(&info, 0, sizeof(info));
        int rc = ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info);
        ::close(fd);

        if (rc == 0 && label == info.label)
            return path;
    }
    throw std::runtime_error("GPIO chip with label '" + label + "' not found");
}

GPIO::GPIO(unsigned int pin_number, Direction direction, const std::string& chip_label)
    : chip_fd(-1), line_fd(-1), pin(pin_number), dir(direction),
      chip_path(chipPathByLabel(chip_label)), is_open(false)
{
    open();
}

GPIO::~GPIO() {
    close();
}

GPIO::GPIO(GPIO&& other) noexcept
    : chip_fd(other.chip_fd), line_fd(other.line_fd),
      pin(other.pin), dir(other.dir),
      chip_path(std::move(other.chip_path)), is_open(other.is_open)
{
    other.chip_fd = -1;
    other.line_fd = -1;
    other.is_open = false;
}

GPIO& GPIO::operator=(GPIO&& other) noexcept {
    if (this != &other) {
        close();
        chip_fd = other.chip_fd;
        line_fd = other.line_fd;
        pin = other.pin;
        dir = other.dir;
        chip_path = std::move(other.chip_path);
        is_open = other.is_open;
        other.chip_fd = -1;
        other.line_fd = -1;
        other.is_open = false;
    }
    return *this;
}

void GPIO::open() {
    if (is_open) return;

    chip_fd = ::open(chip_path.c_str(), O_RDONLY);
    if (chip_fd < 0) {
        throw std::runtime_error("Failed to open GPIO chip: " + 
                               std::string(strerror(errno)));
    }

    struct gpiohandle_request req;
    memset(&req, 0, sizeof(req));
    
    req.lineoffsets[0] = pin;
    req.lines = 1;
    
    if (dir == Direction::INPUT) {
        req.flags = GPIOHANDLE_REQUEST_INPUT;
    } else {
        req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    }
    
    strncpy(req.consumer_label, "rpi5_gpio", sizeof(req.consumer_label) - 1);

    if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        ::close(chip_fd);
        throw std::runtime_error("Failed to get line handle for GPIO " + 
                               std::to_string(pin) + ": " + strerror(errno));
    }

    line_fd = req.fd;
    is_open = true;
}

void GPIO::close() {
    if (line_fd >= 0) {
        ::close(line_fd);
        line_fd = -1;
    }
    if (chip_fd >= 0) {
        ::close(chip_fd);
        chip_fd = -1;
    }
    is_open = false;
}

void GPIO::write(Value value) {
    if (!is_open || dir != Direction::OUTPUT) {
        throw std::runtime_error("GPIO not configured for output");
    }

    struct gpiohandle_data data;
    memset(&data, 0, sizeof(data));
    data.values[0] = static_cast<uint8_t>(value);

    if (ioctl(line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0) {
        throw std::runtime_error("Failed to set GPIO value: " + 
                               std::string(strerror(errno)));
    }
}

Value GPIO::read() {
    if (!is_open) {
        throw std::runtime_error("GPIO not open");
    }

    struct gpiohandle_data data;
    memset(&data, 0, sizeof(data));

    if (ioctl(line_fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data) < 0) {
        throw std::runtime_error("Failed to read GPIO value: " + 
                               std::string(strerror(errno)));
    }

    return data.values[0] ? Value::HIGH : Value::LOW;
}

void GPIO::set_high() { 
    write(Value::HIGH); 
}

void GPIO::set_low() { 
    write(Value::LOW); 
}

void GPIO::toggle() { 
    write(read() == Value::HIGH ? Value::LOW : Value::HIGH); 
}

bool GPIO::is_high() { 
    return read() == Value::HIGH; 
}

bool GPIO::is_low() { 
    return read() == Value::LOW; 
}

unsigned int GPIO::get_pin() const { 
    return pin; 
}

Direction GPIO::get_direction() const { 
    return dir; 
}

} // namespace Gpio