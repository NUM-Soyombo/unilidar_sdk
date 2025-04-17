/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_lidar_sdk.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>

using namespace unitree_lidar_sdk;

// Function to initialize serial port for STM32 communication
int initSerialPort(const char* portName, int baudRate) {
    int serialPort = open(portName, O_RDWR);
    
    if (serialPort < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }
    
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    
    if (tcgetattr(serialPort, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        close(serialPort);
        return -1;
    }
    
    // Set Baud Rate
    speed_t baud;
    switch (baudRate) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        case 500000: baud = B500000; break;
        case 921600: baud = B921600; break;
        case 1000000: baud = B1000000; break;
        default: baud = B115200; break;
    }
    
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    
    // 8N1 Mode (8 bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    
    tty.c_cflag &= ~CRTSCTS;           // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;     // Enable receiver, ignore modem control lines
    
    tty.c_lflag &= ~ICANON;            // Non-canonical mode
    tty.c_lflag &= ~ECHO;              // Disable echo
    tty.c_lflag &= ~ECHOE;             // Disable erasure
    tty.c_lflag &= ~ECHONL;            // Disable new-line echo
    tty.c_lflag &= ~ISIG;              // Disable interpretation of INTR, QUIT and SUSP
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special handling of received bytes
    
    tty.c_oflag &= ~OPOST;             // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR;             // Prevent conversion of newline to carriage return/line feed
    
    tty.c_cc[VTIME] = 10;              // Wait for up to 1s (10 deciseconds)
    tty.c_cc[VMIN] = 0;                // Non-blocking read
    
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        close(serialPort);
        return -1;
    }
    
    return serialPort;
}
// Corrected function to get yaw from quaternion
float getYawFromQuaternion(const float quaternion[4]) {
  // Extract yaw (rotation around z-axis) from quaternion
  // Formula: atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
  float x = quaternion[0];
  float y = quaternion[1];
  float z = quaternion[2];
  float w = quaternion[3];
  
  return atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}
// Function to send IMU data to STM32
void sendIMUDataToSTM32(int serialPort, const IMUUnitree& imuData) {
    if (serialPort < 0) return;
    
    // Create a simple packet structure for IMU data
    // Header (2 bytes) + Quaternion (4 floats) + Timestamp (1 float) + Checksum (1 byte)
    // Total: 2 + 16 + 4 + 1 = 23 bytes
    uint8_t packet[23];
    
    // Header
    packet[0] = 0xAA;
    packet[1] = 0x55;

    // Calculate yaw from quaternion
    float yaw = getYawFromQuaternion(imuData.quaternion);
    // Quaternion data (x, y, z, w)
    memcpy(&packet[2], &imuData.quaternion[0], 4);
    memcpy(&packet[6], &imuData.quaternion[1], 4);
    memcpy(&packet[10], &imuData.quaternion[2], 4);
    memcpy(&packet[14], &imuData.quaternion[3], 4);
    
    // Timestamp
    memcpy(&packet[18], &imuData.stamp, 4);
    
    // Simple checksum (sum of all bytes)
    uint8_t checksum = 0;
    for (int i = 0; i < 22; i++) {
        checksum += packet[i];
    }
    packet[22] = checksum;
    
    // Send the packet
    ssize_t bytes_written = write(serialPort, packet, sizeof(packet));
    if (bytes_written < 0) {
        printf("Error writing to serial port: %s\n", strerror(errno));
    } else if (bytes_written < (ssize_t)sizeof(packet)) {
        printf("Warning: Only wrote %zd of %zu bytes\n", bytes_written, sizeof(packet));
    }
}

int main() {
    // Initialize STM32 serial port
    int stm32SerialPort = initSerialPort("/dev/ttyAMA0", 115200); // Use appropriate port for STM32
    
    if (stm32SerialPort < 0) {
        printf("Failed to initialize STM32 serial port. Continuing without STM32 communication.\n");
    } else {
        printf("STM32 serial port initialized successfully.\n");
    }

    // Initialize Lidar Object
    UnitreeLidarReader* lreader = createUnitreeLidarReader();
    int cloud_scan_num = 18;
    std::string port_name = "/dev/ttyUSB0";

    if (lreader->initialize(cloud_scan_num, port_name)) {
        printf("Unilidar initialization failed! Exit here!\n");
        if (stm32SerialPort >= 0) close(stm32SerialPort);
        exit(-1);
    } else {
        printf("Unilidar initialization succeed!\n");
    }

    printf("\n");

    // Print Lidar Version
    while(true) {
        if (lreader->runParse() == VERSION) {
            printf("lidar firmware version = %s\n", lreader->getVersionOfFirmware().c_str());
            break;
        }
        usleep(500);
    }
    printf("lidar sdk version = %s\n\n", lreader->getVersionOfSDK().c_str());
    sleep(2);

    printf("\n");
    sleep(2);

    // Set LED
    printf("Turn on all the LED lights ...\n");
    uint8_t led_table[45];
    for (int i=0; i < 45; i++) {
        led_table[i] = 0xFF;
    }
    lreader->setLEDDisplayMode(led_table);
    sleep(2);

    printf("Turn off all the LED lights ...\n");
    for (int i=0; i < 45; i++) {
        led_table[i] = 0x00;
    }
    lreader->setLEDDisplayMode(led_table);
    sleep(2);

    printf("Set LED mode to: FORWARD_SLOW ...\n");
    lreader->setLEDDisplayMode(FORWARD_SLOW);
    sleep(2);

    printf("Set LED mode to: REVERSE_SLOW ...\n");
    lreader->setLEDDisplayMode(REVERSE_SLOW);
    sleep(2);

    printf("Set LED mode to: SIXSTAGE_BREATHING ...\n");
    lreader->setLEDDisplayMode(SIXSTAGE_BREATHING);

    printf("\n");
    sleep(2);

    // Parse PointCloud and IMU data
    MessageType result;
    while (true) {
        result = lreader->runParse(); // You need to call this function at least 1500Hz

        switch (result) {
        case NONE:
            break;
        
        case IMU:
            printf("An IMU msg is parsed!\n");
            printf("\tstamp = %f, id = %d\n", lreader->getIMU().stamp, lreader->getIMU().id);
            printf("\tquaternion (x, y, z, w) = [%.4f, %.4f, %.4f, %.4f]\n", 
                    lreader->getIMU().quaternion[0], lreader->getIMU().quaternion[1], 
                    lreader->getIMU().quaternion[2], lreader->getIMU().quaternion[3]);
            printf("\ttimedelay (us) = %d\n\n", lreader->getTimeDelay());
            
            // Send IMU data to STM32
            if (stm32SerialPort >= 0) {
                sendIMUDataToSTM32(stm32SerialPort, lreader->getIMU());
                printf("\tIMU data sent to STM32\n");
            }
            break;
        
        default:
            break;
        }
    }
    
    // Cleanup (this part will never be reached in the current implementation)
    if (stm32SerialPort >= 0) {
        close(stm32SerialPort);
    }
    
    return 0;
}