
//#include <AP_Common/AP_Common.h>
//#include <AP_HAL/AP_HAL.h>
//#include <AP_HAL_Linux/HAL_Linux_Class.h>
//#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Linux/CANSocketIface.h>
#include <AP_HAL/CANIface.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>  // Include the CANManager header



const AP_HAL::HAL& hal = AP_HAL::get_HAL();
// //Linux::CANIface& canIF;
// std::unique_ptr<Linux::CANIface> can_iface;



// void setup();
// void loop();


// void setup(){
//     can_iface = std::make_unique<Linux::CANIface>(0);

// canIF.init();

// }

// void loop(){}

// Declare the HAL instance for Linux
//extern const AP_HAL::HAL &hal;

// Create an instance of the CAN interface
std::unique_ptr<Linux::CANIface> can_iface(new Linux::CANIface(0));
AP_CANManager canManager;

void setup() {


    //canManager.init();
    hal.console->printf("Starting CAN interface test\n");

    // Initialize the CAN interface (assuming bitrate is 500,000 bps and Normal mode)
    uint32_t bitrate = 500000;
    AP_HAL::CANIface::OperatingMode mode = AP_HAL::CANIface::OperatingMode::NormalMode;

    // Assuming the CAN interface index is 0 (can0)
    //can_iface = new Linux::CANIface(0);//std::unique_ptr<Linux::CANIface>(0);

    if (can_iface->init(bitrate, mode)) {
        hal.console->printf("CAN interface initialized successfully\n");
    } else {
        hal.console->printf("Failed to initialize CAN interface\n");
        return;
    }

    // Optionally configure filters if needed
    // AP_HAL::CANIface::CanFilterConfig filter_configs[1];
    // configure filters as necessary
    // can_iface->configureFilters(filter_configs, 1);
}

void loop() {
    // Prepare a CAN frame to send
    AP_HAL::CANFrame frame;
    frame.id = 0x123;   // Example CAN ID
    frame.dlc = 8;      // Data length code (number of bytes)
    frame.data[0] = 0x01;
    frame.data[1] = 0x02;
    frame.data[2] = 0x03;
    frame.data[3] = 0x04;
    frame.data[4] = 0x05;
    frame.data[5] = 0x06;
    frame.data[6] = 0x07;
    frame.data[7] = 0x08;

    // Send the CAN frame
    int16_t send_status = can_iface->send(frame, 0, AP_HAL::CANIface::CanIOFlags());

    if (send_status > 0) {
        hal.console->printf("CAN frame sent successfully\n");
    } else if (send_status == 0) {
        hal.console->printf("CAN Tx buffer is full\n");
    } else {
        hal.console->printf("Failed to send CAN frame\n");
    }

    // Attempt to receive a CAN frame
    AP_HAL::CANFrame received_frame;
    uint64_t timestamp;
    AP_HAL::CANIface::CanIOFlags flags;

    int16_t recv_status = can_iface->receive(received_frame, timestamp, flags);

    if (recv_status > 0) {
        hal.console->printf("Received CAN frame: ID=0x%03X, Data=", received_frame.id);
        for (int i = 0; i < received_frame.dlc; i++) {
            hal.console->printf("%02X ", received_frame.data[i]);
        }
        hal.console->printf("\n");
    } else if (recv_status == 0) {
        hal.console->printf("No CAN frame available\n");
    } else {
        hal.console->printf("Failed to receive CAN frame\n");
    }

    // Add a delay to avoid flooding the CAN bus
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();


// int main(int argc, char **argv) {
//     hal.console->printf("Starting CAN interface test\n");

//     setup();  // Call the setup function once

//     while (true) {
//         loop();  // Repeatedly call the loop function
//     }

//     return 0;
// } libraries/AP_HAL_Linux/examples/CanBus-socketCan/CanTest.cpp
