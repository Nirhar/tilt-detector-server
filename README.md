# Tilt-Detector ESP32 Server

This Repo contains an HTTP Server implemented in ESP32 Micrcocontroller that serves the client the roll and the pitch of the MCU plane.

## HTTP API Reference

Place a request at 
    /hello
This gives a response from the MCU "Hello from esp32!"

    /get_angles
This gives the Roll and the Pitch of the MCU plane in a String format which can be parsed as a JSON on the client side. 

## Building the Code
    
Prerequisites:

    ESP-IDF Build System

To build the code and flash it to the MCU activate the ESP-IDF Build System and type

    idf.py -p <PORT> flash

where <PORT> is the port where the MCU is connected, typically of the form /dev/ttyUSB[X]

## How Does it work?

If we have an ESP32 connected to a MPU6050 6-DOF Accelerometer via the I2C protocol, one can obtain the Roll and the Pitch of the MCU plane.

The code to obtain Roll and Pitch has been borrowed from https://github.com/imxieyi/esp32-i2c-mpu6050/tree/master/main . Essentially this code uses a kalman filter on top of the raw sensor data to obtain better state estimates of the roll and pitch. We obtain Roll and Pitch in degrees

The obtained roll and pitch are delivered to a client when /get_angles GET request is made

