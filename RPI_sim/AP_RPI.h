// #pragma once

// #include <AP_HAL/AP_HAL.h>

// class AP_RPI {
// public:
//     AP_RPI();

//     /* Do not allow copies */
//     AP_RPI(const AP_RPI &other) = delete;
//     AP_RPI &operator=(const AP_RPI&) = delete;

//     // Initialize the RPI communication
//     void init();

//     // Update the RPI communication
//     void update();

//     // Send data to the RPI
//     void send_data(const uint8_t *data, uint16_t len);

//     // Receive data from the RPI
//     bool receive_data(uint8_t *data, uint16_t &len);

// private:
//     AP_HAL::UARTDriver *_uart = nullptr;
// };

//NEW CODE
// AP_RPI.h
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

class AP_RPI {
public:
    AP_RPI();

    /* Do not allow copies */
    AP_RPI(const AP_RPI &other) = delete;
    AP_RPI &operator=(const AP_RPI&) = delete;

    // Initialize the RPI communication
    void init();

    // Update the RPI communication
    void update();

    // Send data to the RPI
    void send_data(const uint8_t *data, uint16_t len);

private:
    AP_HAL::UARTDriver *_port = nullptr;
    bool receive_data(uint8_t *data, uint16_t &len);
};
