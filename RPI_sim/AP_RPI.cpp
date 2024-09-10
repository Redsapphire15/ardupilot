// #include "AP_RPI.h"

// extern const AP_HAL::HAL& hal;

// AP_RPI::AP_RPI()
// {
// }

// void AP_RPI::init()
// {
//     // Get the UART driver instance for RPI communication
//     _uart = hal.serial(AP_SERIALMANAGER_RPI);

//     // Configure and begin the UART communication
//     _uart->begin(115200); // Set the desired baud rate
// }

// void AP_RPI::update()
// {
//     // Receive data from the RPI
//     uint16_t len = 0;=
//     uint8_t data[256]; // Adjust the buffer size as needed

//     if (receive_data(data, len)) {
//         // Process the received data
//         if (len > 0) {
//             // Display the received message
//             hal.console->printf("Received from RPI (%u bytes): ", len);
//             for (uint16_t i = 0; i < len; i++) {
//                 hal.console->printf("%02X ", data[i]);
//             }
//             hal.console->printf("\n");
//         }
//     }
// }

// void AP_RPI::send_data(const uint8_t *data, uint16_t len)
// {
//     // Send data to the RPI
//     _uart->write(data, len);
// }

// bool AP_RPI::receive_data(uint8_t *data, uint16_t &len)
// {
//     // Read data from the UART buffer
//     len = _uart->available();
//     if (len > 0) {
//         len = _uart->read(data, len);
//         return true;
//     }
//     return false;
// }

//NEW CODE
// AP_RPI.cpp
#include "AP_RPI.h"

extern const AP_HAL::HAL& hal;

AP_RPI::AP_RPI()
{
}

void AP_RPI::init()
{
    const auto &serial_manager = AP::serialmanager();
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_RPI, 0); // Find the first instance of the RPi protocol

    if (_port) {
        _port->begin(115200); // Set the desired baud rate
    }
}

void AP_RPI::update()
{
    hal.console->printf("update function executed\n");
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"update function executed\n");
    if (!_port) {
        return;
    }

    // Receive data from the RPI
    uint16_t len = 0;
    uint8_t data[256]; // Adjust the buffer size as needed

    if (receive_data(data, len)) {
        // Process the received data
        if (len > 0) {
            // Display the received message
            hal.console->printf("Received from RPI (%u bytes): ", len);
            for (uint16_t i = 0; i < len; i++) {
                hal.console->printf("%02X ", data[i]);
            }
            hal.console->printf("\n");
        }
    }
}

void AP_RPI::send_data(const uint8_t *data, uint16_t len)
{
    if (_port) {
        // Send data to the RPI
        _port->write(data, len);
    }
}

bool AP_RPI::receive_data(uint8_t *data, uint16_t &len)
{
    if (!_port) {
        return false;
    }

    // Read data from the UART buffer
    len = _port->available();
    if (len > 0) {
        len = _port->read(data, len);
        return true;
    }
    return false;
}

