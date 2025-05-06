# NIBP-monitor-oscillometric-using-STM32

A project developed for the Embedded Systems Design course (EE3003) at Ho Chi Minh City University of Technology (BKU). This device measures human blood pressure using an oscillometric method with an MPS20N0040D-D pressure sensor and displays the results on an LCD screen.

## Purpose & Main Features

*   **Measure Blood Pressure:** Determines Systolic (SYS) and Diastolic (DIA) blood pressure values.
*   **Oscillometric Method:** Utilizes cuff pressure oscillations to calculate blood pressure.
*   **LCD Display:** Shows measurement status, pressure values (mmHg), and warnings.
*   **User Modes:**
    *   **Normal Mode (START1):** Inflates cuff to a standard pressure (~150-160 mmHg) suitable for typical users.
    *   **High BP Mode (START2):** Inflates cuff to a higher pressure (~180-200 mmHg) for users suspected of having high blood pressure.
*   **Abnormal BP Alert:** Uses a Red LED to indicate if the measured blood pressure is outside the normal range (High or Low).
*   **Emergency Stop:** A dedicated STOP button to immediately halt the measurement process and deflate the cuff.
*   **Automatic Power Off:** (Planned/Mentioned in requirements) System shuts down after a period of inactivity (e.g., 3 minutes) to save power.

## Hardware Used

*   **Microcontroller:** STM32F103C8T6 (Blue Pill board)
*   **Pressure Sensor:** MPS20N0040D-D (Wheatstone bridge type)
*   **Instrumentation Amplifier:** AD620AN (To amplify the differential signal from the pressure sensor)
*   **Operational Amplifier:** LM741 (Used in the band-pass filter circuit)
*   **Display:** LCD 1602 (16 characters x 2 lines) with PCF8574 I2C backpack module
*   **Air Pump:** 370 Air Pump Motor
*   **Air Valve:** Solenoid Valve (Normally Open type, used for controlled deflation)
*   **Buttons:** Tactile Push Buttons (x3 - START1, START2, STOP)
*   **Indicator:** LED (Red)
*   **Power Regulation:** LM7805 (+5V), LM7905 (-5V)
*   **Cuff:** Standard inflatable blood pressure cuff

## How to Compile Code

*   **IDE:** STM32CubeIDE (Version used during development is not specified, but any recent version should work).
*   **Toolchain:** ARM GCC (Typically included with STM32CubeIDE).
*   **Libraries:** STM32Cube HAL (Hardware Abstraction Layer) library.
*   **Steps:**
    1.  Open STM32CubeIDE.
    2.  Import the project (File -> Import -> General -> Existing Projects into Workspace).
    3.  Select the project root directory.
    4.  Build the project (Project -> Build All or Ctrl+B).

## How to Flash Code

You will need a programmer/debugger compatible with STM32 microcontrollers.

*   **Method 1: ST-Link (Recommended)**
    1.  Connect an ST-Link V2 (or V3) programmer to the SWD pins (SWDIO, SWCLK, GND, 3.3V) on the Blue Pill board.
    2.  Connect the ST-Link to your computer via USB.
    3.  In STM32CubeIDE, click "Run" or "Debug" (or use STM32CubeProgrammer software). The IDE should automatically detect the ST-Link and flash the compiled code (`.elf` or `.hex` file) to the microcontroller.
*   **Method 2: USB DFU (Device Firmware Update)**
    1.  Set the BOOT0 jumper to 1 (High) and BOOT1 to 0 (Low) on the Blue Pill.
    2.  Connect the Blue Pill to your computer via its USB port.
    3.  Use STM32CubeProgrammer software, select "USB" connection, and flash the firmware (usually a `.bin` file).
    4.  Set the BOOT0 jumper back to 0 after flashing.
*   **Method 3: UART Bootloader**
    1.  Set the BOOT0 jumper to 1 and BOOT1 to 0.
    2.  Connect a USB-to-Serial (TTL) adapter to the PA9 (TX) and PA10 (RX) pins of the Blue Pill.
    3.  Use STM32CubeProgrammer (select UART) or the `stm32flash` utility to flash the firmware.
    4.  Set BOOT0 back to 0.

## How to Use

1.  **Power Up:** Connect a suitable power source (e.g., 9V-12V DC to the input connector for the onboard regulators, or 5V via USB if the circuit design supports it - *check the final PCB power input*).
2.  **Apply Cuff:** Wrap the inflatable cuff around the upper arm, ensuring it's snug but not too tight. Position it according to standard blood pressure measurement guidelines (usually aligning an artery mark on the cuff with the brachial artery).
3.  **Start Measurement:**
    *   Press the **START1** button (often designated by color or label for "Normal") for a standard measurement.
    *   Press the **START2** button (often designated for "High BP") if you suspect higher blood pressure, which inflates the cuff further.
4.  **Measurement Process:** The pump will inflate the cuff. Once the target pressure is reached, the valve will slowly release the air. The device measures pressure oscillations during deflation. Remain still and quiet during this process.
5.  **View Results:** The Systolic (SYS) and Diastolic (DIA) pressure values will be displayed on the LCD screen in mmHg.
6.  **Alerts:** If the Red LED turns on, it indicates the reading is outside the normal range.
7.  **Stop:** Press the **STOP** button at any time to cancel the measurement and deflate the cuff immediately.

## Visuals

*   **Block Diagram:** (See report PDF, Figure 3.1.1)
*   **Schematic:** (See report PDF, Figure 4.1.1)
*   **PCB Layout:** (See report PDF, Figure 4.2.1)
*   **Photo of Device:** (See report PDF, Page 31 or 32)
*   **Operation Flowchart:** (See report PDF, Page 34)
*   ![Report](Docs/Project_Report/Final_Report.pdf)
