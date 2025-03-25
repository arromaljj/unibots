#include <gpiod.h>
#include <iostream>
#include <string.h>
#include <errno.h>

int main() {
    std::cout << "Testing libgpiod functionality..." << std::endl;
    
    // Try to open the chip with full path
    struct gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip4");
    if (!chip) {
        std::cerr << "Failed to open GPIO chip: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
        return 1;
    }
    
    std::cout << "Successfully opened gpiochip4" << std::endl;
    
    // Get a line
    struct gpiod_line *line = gpiod_chip_get_line(chip, 0);
    if (!line) {
        std::cerr << "Failed to get GPIO line: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }
    
    std::cout << "Successfully got GPIO line" << std::endl;
    
    // Request the line for output
    if (gpiod_line_request_output(line, "test", 0) < 0) {
        std::cerr << "Failed to request GPIO line: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return 1;
    }
    
    std::cout << "Successfully requested GPIO line" << std::endl;
    
    // Cleanup
    gpiod_line_release(line);
    gpiod_chip_close(chip);
    
    std::cout << "Test completed successfully" << std::endl;
    return 0;
} 