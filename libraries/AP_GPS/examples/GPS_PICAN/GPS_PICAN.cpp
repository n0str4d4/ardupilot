#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Linux/UARTDriver.h>
#include <AP_GPS_NMEA.h>

//#include <AP_HAL/utility.h>
//#include <AP_Common/AP_Common.h>
//#include <AP_SerialManager/AP_SerialManager.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();                     //Declare "hal" reference variable. This variable is pointing to Ap_HAL::HAL class's object. Here, AP_HAL is library and HAL is a class in that library. This reference variable can be used to get access to hardware specific functions.                     

const AP_HAL::HAL& hal_linux = AP_HAL_Linux::get_HAL();

class GPSModuleTest {
public:
    void setup();
    void loop();

private:
    Linux::UARTDriver* gps_uart;
};

void GPSModuleTest::setup() {
    // Initialize SerialManager to get the GPS UART port
    //gps_uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_GPS);
    //gps_uart = hal.serial(0).set_de;
    gps_uart_test = ha

    gps_uart = (Linux::UARTDriver*) hal.serial(3);
    gps_uart->set_device_path("/dev/ttyS0");
	// if not relying on Serial Manager to find the serial interface that supports/talk GPS protocol
    // gps_uart = hal.serial(0); or hal->serial(0);

    if (gps_uart != nullptr) {
        // Set baud rate to 9600 and initialize the GPS UART
        gps_uart->begin(9600);
        // having the flow control, Character size being setup on system startup from the linux shell
        // we might not need to do this here, but still, try the DISABLED/NONE flow control 
        gps_uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        hal.console->printf("GPS UART initialized at 9600 baud\n");
    } else {
        hal.console->printf("Failed to initialize GPS UART\n");
    }
}

void GPSModuleTest::loop() {
    if (gps_uart != nullptr && gps_uart->available()) {
        char gps_data[100];  // Buffer to hold GPS data
        uint8_t i = 0;

        // Read from GPS UART until a full sentence is captured
        while (gps_uart->available() && i < sizeof(gps_data) - 1) {
            gps_data[i++] = gps_uart->read();
        }
        gps_data[i] = '\0';  // Null-terminate the string

        // Print the raw NMEA sentence
        hal.console->printf("Received GPS Data: %s\n", gps_data);
    }
}

// Instantiate and use the GPSModuleTest in ArduPilot's architecture
GPSModuleTest gps_module;

void setup() {
    gps_module.setup();
}

void loop() {
    gps_module.loop();
    hal.scheduler->delay(100); // Small delay to avoid overwhelming the serial buffer
}

// Register above functions in HAL board level
AP_HAL_MAIN();