# DIY 1kW MPPT Solar Charge Controller Firmware

This document provides a comprehensive guide to the firmware for the FUGU-MPPT, a 1-kilowatt Maximum Power Point Tracking (MPPT) solar charge controller. It is intended for electronics enthusiasts, students, and DIYers who want to understand, build, and customize their own high-efficiency solar charge controller.

## Project Overview

An MPPT solar charge controller is a smart DC-to-DC converter that optimizes the power harvested from a solar panel. It adjusts its operating point (voltage and current) to match the panel's maximum power output, which varies with sunlight and temperature. This results in 10-30% more power harvested compared to simpler PWM controllers.

This project is a complete hardware and software solution for controlling the charging of a battery bank from a solar array, ensuring efficient and safe operation.

### Key Features

*   **MPPT Charging:** Implements the "Perturb and Observe" algorithm to maximize solar harvest.
*   **Multi-Chemistry Support:** Configurable for different battery voltages and charging parameters.
*   **Comprehensive Protections:** Includes safeguards against over-voltage, under-voltage, over-current, and over-temperature.
*   **LCD Interface:** An interactive 16x2 LCD screen for real-time monitoring and configuration.
*   **Telemetry:** Outputs detailed performance data over the serial USB port.
*   **Wireless Monitoring:** (Optional) Supports Wi-Fi for remote telemetry.

### Original Project

This project is based on the outstanding open-source work by AngeloCasi.
*   **Original Firmware:** [AngeloCasi/FUGU-ARDUINO-MPPT-FIRMWARE](https://github.com/AngeloCasi/FUGU-ARDUINO-MPPT-FIRMWARE)
*   **Instructables Guide:** [1kW Arduino MPPT Solar Charge Controller](https://www.instructables.com/DIY-1kW-MPPT-Solar-Charge-Controller/)

---

## Hardware Requirements

This firmware is not standalone; it is designed to run on a specific hardware platform. **You must build the circuit for this code to be useful.**

### Schematic

The primary reference for the hardware build is the KiCad schematic file located in the project's `1-schematic-files/` directory. This file details all component connections and values.

### Key Components

The controller's functionality relies on several key electronic components. The datasheets for these parts are available in the `7-datasheets/` directory and are essential reading for understanding the hardware.

| Component         | Role in Project
| :---              | :---
| **ESP32-WROOM**   | **Main Microcontroller:** The "brain" of the controller. It runs the MPPT algorithm, reads sensors, drives the LCD, and manages all logic.
| **ADS1115**       | **High-Precision ADC:** A 16-bit Analog-to-Digital converter that measures solar voltage, battery voltage, and current with high accuracy via an I2C interface.
| **IR2104**        | **MOSFET Driver:** Takes the low-power PWM signal from the ESP32 and provides the high-current drive needed to switch the main power MOSFETs quickly and efficiently.
| **CSD19505**      | **Power MOSFETs:** High-current switches that form the heart of the buck converter, controlling the flow of power from the solar panel to the battery.
| **XL7005A**       | **Buck Regulator:** An auxiliary power supply that efficiently steps down the high input voltage to power the ESP32 and other low-voltage components.
| **16x2 I2C LCD**  | **Display:** A standard 16-character, 2-line display for the user interface. |

---

## Getting Started

Follow these steps to compile the firmware and upload it to your ESP32 board.

### Prerequisites

*   [Arduino IDE](https://www.arduino.cc/en/software) software needs to be installed and the current user added to the `dialout` group.
*   The charge controller hardware must be assembled.

```
sudo apt install arduino -y
sudo usermod -a -G dialout $USER
```

### Install Drivers

Your computer needs to communicate with the ESP32 module. Most ESP32 development boards use a USB-to-UART bridge chip like the **CP2102** or **CH340**. Modern Linux and Windows versions often include these drivers, but if your board is not recognized, you will need to install them manually. You can use `lsusb` to check if the boards were recognized by your system:

```
lsusb
```

In Linux, communication with (external) devices is made possible by device files in the `dev` directory. To figure out which device is associated with which board, unplug the board and plug it in again, then look at `dmesg` to find the file for the device.

```
dmesg | tail
``` 

### Configure Arduino IDE

You must teach the Arduino IDE how to work with ESP32 boards.

1.  Go to **File » Preferences**.
2.  In the "Additional Boards Manager URLs" field, paste the following URL:
    ```
    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    ```
3.  Go to **Tools » Board » Boards Manager**.
4.  Search for `esp32` and install the package by **Espressif Systems**.
5.  Select the correct board: **Tools » Board » ESP32 Arduino » ESP32 Dev Module**.
6.  Then select the board's device name found in the **Drivers** section above by going to ** Tools » Port**.

### Install Libraries

This firmware requires two external libraries.

1.  Go to **Sketch » Include Library » Manage Libraries**.
2.  Search for and install `Adafruit ADS1X15`.
3.  The `LiquidCrystal_I2C` library is not in the default manager. Download the ZIP file from [this repository](https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library) by going to **Code** and then clicking **Download ZIP**.
4.  In the Arduino IDE, go to **Sketch » Include Library » Add .ZIP Library...** and select the downloaded file.

### Upload Firmware

1.  Connect the ESP32 board to your computer via USB.
2.  Go to **Tools » Port** and select the correct serial port for your device.
3.  Click the **Upload** button (the right-arrow icon).

If uploading to the board works then the Arduino environment was set up correctly, however don't go using this board is your solar setup yet as the firmware still needs to be modified for your setup.

---

## Firmware files

The firmware is split into multiple `.ino` files, which the Arduino IDE treats as tabs and combines into a single program. This keeps the code organized by function.

| `ARDUINO_MPPT_FIRMWARE.ino`   | This is the main file. It contains the `setup()` function for initialization and the `loop()` function, which is the repeating heart of the program.
| `2_Read_Sensors.ino`          | Handles all sensor measurements. It communicates with the **ADS1115 ADC** over I2C to get precise readings of panel voltage, battery voltage, and current.
| `3_Device_Protection.ino`     | Acts as the safety supervisor. It constantly checks sensor values against the limits defined in the configuration.
| `4_Charging_Algorithm.ino`    | This is the core of the MPPT logic. It find the maximum power point and also manages the battery charging states.
| `5_System_Processes.ino`      | Manages various background tasks, such as checking for user input from the buttons and updating timers.
| `6_Onboard_Telemetry.ino`     | Gathers all the data and prepares it for output.
| `7_Wireless_Telemetry.ino`    | (If enabled) Manages the Wi-Fi connection and sends telemetry data to a remote server.
| `8_LCD_Menu.ino`              | Controls the 16x2 LCD, displaying real-time data and allowing the user to navigate menus to view statistics and change settings.

---

## Main Loop

The firmware's behavior can be customized by changing variables in the **USER PARAMETER** section of the main `ARDUINO_MPPT_FIRMWARE.ino` file. The parameter are ordered by type (`bool`, `int` and `float`), and the following section quickly goes into the use of each parameter.

### Feature Flags (On/Off)

These flags enable or disable certain features of the controller.

| `MPPT_Mode`                   | Enable MPPT algorithm, when disabled charger uses CC-CV algorithm 
| `output_Mode`                 | 0 = PSU MODE, 1 = Charger Mode  
| `disableFlashAutoLoad`        | Forces the MPPT to not use flash saved settings, enabling this "1" defaults to programmed firmware settings.
| `enablePPWM`                  | Enables Predictive PWM, this accelerates regulation speed (only applicable for battery charging application)
| `enableWiFi`                  | Enable WiFi Connection
| `enableFan`                   | Enable Cooling Fan
| `enableBluetooth`             | Enable Bluetooth Connection
| `enableLCD`                   | Enable LCD display
| `enableLCDBacklight`          | Enable LCD display's backlight
| `overrideFan`                 | Fan always on
| `enableDynamicCooling`        | Enable for PWM cooling control 

Most of these settings are self-explanatory, but the two mode settings do with some extra information.

#### MPPT Mode

The `MPPT_Mode` configuration enables (1) or disables (0) the MPPT algorithm, and it should be enabled when connecting to a solar panel (or a wind generator), but not when connecting to a regular power supply. This is because a solar panel is a *current source*, and the amount of power it provides is highly dependant on the load you connect to it. Its I-V curve has a very distinct "knee" where the maximum power is produced.

   * If you don't draw enough current: The voltage is high, but the power (P = V * I) is low. (This is the Open Circuit Voltage, `Voc`).
   * If you try to draw too much current: The voltage collapses, and the power is again low. (This is the Short Circuit Current, `Isc`).
   * The Sweet Spot: Somewhere in the middle is the Maximum Power Point (MPP), a specific voltage (Vmp) and current (Imp) that gives you the most power.

The job of an MPPT algorithm is to actively "hunt" for this sweet spot, which moves around depending on sunlight, temperature, and panel age. It's a dynamic hill-climbing algorithm, constantly searching for the peak of the power curve. A normal power supply (like a benchtop supply or a wall adapter) on the other hand is designed to be a constant-voltage source. Its goal is the exact opposite of a solar panel's behavior.

   * It will provide a fixed, stable voltage (e.g., 24V) no matter what the load is, right up to its maximum current limit.
   * Its ideal I-V curve is essentially a vertical line at its set voltage.

Because the voltage is constant, the power delivered is simply Power = (Set Voltage) * (Current Drawn). To get the maximum power out of it, you simply need to draw the maximum current it can safely provide. There is no need to use an MPPT algorithm in this case, and `MPPT_Mode` should be set to `0`.

#### Output Mode

### Core Charging Parameters

These are the most common settings to adjust for your specific battery bank.

*   `voltageBatteryMax`: The target charging voltage (Absorption voltage).
*   `voltageBatteryMin`: The low-voltage cutoff point.
*   `currentCharging`: The maximum charging current (Bulk current).

### Technical Settings

*   `pwmFrequency`: The switching frequency of the buck converter (e.g., 39000 Hz).
*   `temperatureFan`: The temperature (°C) at which the fan turns on.
*   `temperatureMax`: The temperature (°C) at which the controller shuts down for safety.

---

## Read Sensors

### ADC gains

The `ADC_SetGain()` function configures the programmable gain of the external ADS1115/ADS1015 ADC. The gain determines the measurement range and precision. A smaller voltage range (e.g., +/- 2.048V) provides higher resolution for measuring small signals. In this design the gain for all measurements from the ADC is set to `GAIN_TWO`, which gives a measuring range of +/- 2.048V for all the signals measured by the ADC. 

  * The voltage divider for the **input voltage** has a 200kOhm resistor at the top and a 5.1kOhm resistor at the bottom. This means that the maximum input voltage of 80V would be divided as (5.1k) / (200k + 5.1k) * 80V = 1.989V, which is below the maximum of 2.048 that can be measured by the ADC.
  * The voltage divider for the **output voltage** has a 23.5k resitor at the top (two 47k resistors in parallel) and a 1k resitor at the bottom. For the maximum output voltage of 50V, the ADC would be measuring (1k) / (23.5k + 1k) * 50V = 2.041V, which is also below the 2.048V maximum.

The `ADC_BitReso` variable holds the voltage increment per single ADC bit, which in case of using the ADS1115 at 2x gain is an impressive 0.0625mV/bit. In other words, the smallest voltage increment the ADC can measure is 0.0625mV.  

### Temperature

The temperature of the heatsink is calculated using a simple averaging technique. It keeps adding the current temperature measurement (`analogRead(TempSensor)`) for a given number of loops, and then divides the sum by the total number of loops to get the average. 

```
TS = TS/sampleStoreTS;
```

The resistance of the termistor is then calculated based on the average reading from the analog input pin, and the formula for calculating the resistance is derived as follows: 

1. The voltage divider circuit

The calculation uses the principles of a voltage divider to solve for the unknown resistance of the thermistor. The fundamental voltage divider equation for the circuit is:

  `V_out = V_in * (R_fixed / (R_ntc + R_fixed))`

```
      3.3V (V_in)
        |
       ---
       | | R_ntc (variable)
       | |
       ---
        |
        o-----> To ESP32 ADC pin (V_out)
        |
       ---
       | | R_fixed (10kΩ)
       | |
       ---
        |
       GND (0V)
```

2. The ADC measurement

The ESP32's analogRead() function doesn't give you `V_out` directly. It gives you a number from 0 to 4095 (for its 12-bit ADC). Let's call this ADC reading `TS` (as it is in the code). The highest value (4096) corresponds to the highest voltage the analog input of the ESP32 can read (3.3V). This means the actual input voltage can be calculated as `V_out = TS / 4096 * 3.3V`. As the 3.3V measuring range of the analog input is the same as the 3.3V input voltage of the voltage divider, our formula becomes:

  `V_out = TS / 4095 * 3.3V = TS / 4096 * V_in`

Divide both sides by `V_in`:

  `V_out / V_in = TS / 4095` (eq. 1)

3. Deriving the formula

  Step 1: Start with the voltage divider equation.

  `V_out = V_in * (R_fixed / (R_ntc + R_fixed))` 

  Step 2: Isolate the resistance ratio by dividing each side by `V_in`.

  `V_out / V_in = R_fixed / (R_ntc + R_fixed)` 

  Step 3: Substitute the ADC reading from equation 1.

  `TS / 4095 = R_fixed / (R_ntc + R_fixed)`

  Step 4: Solve for `R_ntc`.

  Flip both sides to make it easier to isolate `R_ntc`:

  `4095 / TS = (R_ntc + R_fixed) / R_fixed`

  Then carry out the division on the right side:

  `4095 / TS = (R_ntc / R_fixed) + (R_fixed / R_fixed)`

  `4095 / TS = (R_ntc / R_fixed) + 1`

  Bring the 1 to the other side:

  `4095 / TS - 1 = (R_ntc / R_fixed)`

  And finally multiply both sides by `R_fixed`:

  `R_ntc = R_fixed * ( (4095 / TS) - 1 )`

This formula calculates the resistance of the termistor, and it is clear that the `ntcResistance` in the code is in fact the resistance of the fixed risistor in the voltage divider.

```
TSlog = log(ntcResistance*(4095.00/TS-1.00));
```

The logarithm of the resistance is subsequently calculated, as it is used in the **Steinhart-Hart equation**, which is a standard formla for converting a thermistor's resistance into a temperature reading in degrees Kelvin. The -273.15 at the end converts it to degrees Celsius.

```
temperature = (1.0/(1.009249522e-03+2.378405444e-04*TSlog+2.019202697e-07*TSlog*TSlog*TSlog))-273.15;
```

### Voltages

### Current

---

## Device Protection

---

## Charging Algorithm

---

## System Processes

---

## Onboard Tememetry

When connected to a computer, the controller sends a stream of data over the USB serial port. You can view this using the Arduino IDE's **Serial Monitor** (**Tools » Serial Monitor**). Set the baud rate to **500000**, like it was configured in the firmware. You should now see acronyms and values scrolling over the screen.

### Error & Fault Flags

The following flags indicate that an error has occured (1 = Active, 0 = Inactive).

| Flag      | Meaning                   | Explanation
| :---      | :---                      | :--- 
| `ERR`     | General Error             | A summary flag. If this is 1, at least one of the other error flags is active.
| `FLV`     | Fatally Low Voltage       | The system voltage is too low to continue operation.
| `BNC`     | Battery Not Connected     | No battery is detected on the output (in charger mode).
| `IUV`     | Input Undervoltage        | The input voltage (from solar) is too low.
| `IOC`     | Input Overcurrent         | The input current is exceeding the absolute safe limit.
| `OOV`     | Output Overvoltage        | The output voltage is exceeding the absolute safe limit.
| `OOC`     | Output Overcurrent        | The output current is exceeding the absolute safe limit.
| `OTE`     | Overtemperature           | The system temperature has exceeded the shutdown threshold.
| `REC`     | Recovery Mode             | The system is in a special recovery state, usually after an input undervoltage event.

### System Status

These flags show the current operating mode and status of various components.

| Flag      | Meaning                   | Explanation
| :---      | :---                      | :--- 
| `MPPTA`   | MPPT Algorithm            | 1 if the MPPT algorithm is active, 0 if using the simpler CC-CV algorithm.
| `CM`      | Charger Mode              | 1 for Battery Charger Mode, 0 for Programmable Power Supply (PSU) Mode.
| `BYP`     | Bypass MOSFET             | 1 if the input backflow protection MOSFET is enabled.
| `EN`      | Buck Enabled              | 1 if the main buck converter is active and delivering power.
| `FAN`     | Fan Status                | 1 if the cooling fan is currently on.
| `WiFi`    | WiFi Status               | 1 if the WiFi is enabled.

### Live Measurements

This is the core telemetry data, showing live values.

| Flag      | Meaning                           | Explanation
| :---      | :---                              | :--- 
| `PI`      | Power Input                       | Power being generated by the solar panels, in Watts (W).
| `PWM`     | PWM Duty Cycle                    | The current duty cycle value being sent to the buck converter.
| `PPWM`    | Predictive PWM                    | The minimum duty cycle calculated by the predictive algorithm to maintain output voltage.
| `VI`      | Voltage Input                     | The voltage from the solar panels, in Volts (V).
| `VO`      | Voltage Output                    | The voltage of the battery, in Volts (V).
| `CI`      | Current Input                     | The current from the solar panels, in Amps (A).
| `CO`      | Current Output                    | The current going to the battery, in Amps (A).
| `Wh`      | Watt-hours                        | The total energy harvested since the last reset, in Watt-hours.
| `Temp`    | Temperature                       | The internal temperature of the unit, in Celsius (°C).
| `CSMPV`   | Current Sensor Midpoint Voltage   | The auto-calibrated zero-current voltage of the ACS712 sensor.
| `CSV`     | Current Sensor Voltage            | The raw voltage reading from the current sensor.
| `VO%Dev`  | Voltage Output Deviation          | The battery voltage as a percentage of the maximum configured charging voltage.
| `SOC`     | State of Charge                   | The estimated battery percentage (%).
| `T`       | Time                              | The total time the unit has been running, in seconds.
| `LoopT`   | Loop Time                         | The time it takes for the main loop() to execute once, in milliseconds (ms). This is a measure of how fast the controller is running.

---

## Wireless Telemetry

---

## LCD Menu

---

## Credits

*   **Original Author:** AngeloCasi
*   **Documentation:** Gemini
