#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_CANManager/AP_CANSensor.h>
//#include <AP_CANManager/AP_CANManager.h>
//#include <AP_CANManager/AP_CANDriver.h>

// Declare the HAL instance for Linux
extern const AP_HAL::HAL &hal;

// Global pointer to the CAN Manager instance
AP_CANManager* can_manager = nullptr;

//AP_CAN::Protocol proto = None;

void setup() {
    // Initialize the CAN Manager
    can_manager = AP_CANManager::get_singleton();
    if (!can_manager) {
        hal.console->printf("CAN Manager is not allocated.\n");
        return;
    }
    
    can_manager->init();
    hal.console->printf("CAN Manager initialized successfully.\n");

    // Assuming we register a driver with NONE protocol
    AP_CANDriver* can_driver = new AP_CANSensor::AP_CANSensor();
    if (can_manager->register_driver(AP_CAN::Protocol::None, can_driver)) {
        hal.console->printf("CAN driver registered successfully.\n");
    } else {
        hal.console->printf("Failed to register CAN driver.\n");
    }

    // Optionally, register a dummy sensor for 11-bit frames
    CANSensor* sensor = new CANSensor();
    uint8_t driver_index;
    if (can_manager->register_11bit_driver(AP_CAN::Protocol::None, sensor, driver_index)) {
        hal.console->printf("11-bit CAN sensor registered successfully at index %d.\n", driver_index);
    } else {
        hal.console->printf("Failed to register 11-bit CAN sensor.\n");
    }

    // Log initial status
    can_manager->log_text(AP_CANManager::LogLevel::LOG_INFO, "Setup", "CAN Manager setup completed.");
}

void loop() {
    // Log retrieval example
    ExpandingString log_str;
    can_manager->log_retrieve(log_str);
    hal.console->printf("CAN Manager Log: %s\n", log_str.c_str());

    // Display the number of CAN drivers and their types
    uint8_t num_drivers = can_manager->get_num_drivers();
    hal.console->printf("Number of CAN drivers: %d\n", num_drivers);

    for (uint8_t i = 0; i < num_drivers; i++) {
        AP_CANDriver* driver = can_manager->get_driver(i);
        if (driver != nullptr) {
            AP_CAN::Protocol driver_type = can_manager->get_driver_type(i);
            hal.console->printf("Driver %d: Type = %d\n", i, static_cast<int>(driver_type));
        }
    }

    // Prepare a CAN frame for transmission
    AP_HAL::CANFrame frame;
    frame.id = 0x123;  // Example CAN ID
    frame.dlc = 8;     // Data length code (number of bytes)
    frame.data[0] = 0x01;
    frame.data[1] = 0x02;
    frame.data[2] = 0x03;
    frame.data[3] = 0x04;
    frame.data[4] = 0x05;
    frame.data[5] = 0x06;
    frame.data[6] = 0x07;
    frame.data[7] = 0x08;

    // Send the CAN frame via the first CAN interface (can0)
    if (can_manager->get_driver(0) != nullptr) {
        int16_t send_status = can_manager->get_driver(0)->send(frame, 0, AP_HAL::CANIface::CanIOFlags());

        if (send_status > 0) {
            hal.console->printf("CAN frame sent successfully.\n");
        } else if (send_status == 0) {
            hal.console->printf("CAN Tx buffer is full.\n");
        } else {
            hal.console->printf("Failed to send CAN frame.\n");
        }
    }

    // Attempt to receive a CAN frame
    AP_HAL::CANFrame received_frame;
    uint64_t timestamp;
    AP_HAL::CANIface::CanIOFlags flags;

    if (can_manager->get_driver(0) != nullptr) {
        int16_t recv_status = can_manager->get_driver(0)->receive(received_frame, timestamp, flags);

        if (recv_status > 0) {
            hal.console->printf("Received CAN frame: ID=0x%03X, Data=", received_frame.id);
            for (int i = 0; i < received_frame.dlc; i++) {
                hal.console->printf("%02X ", received_frame.data[i]);
            }
            hal.console->printf("\n");
        } else if (recv_status == 0) {
            hal.console->printf("No CAN frame available.\n");
        } else {
            hal.console->printf("Failed to receive CAN frame.\n");
        }
    }

    // Add a delay to avoid flooding the CAN bus
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();