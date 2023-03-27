
% **Industrial robot arm hardware documentation**

***

This document describes the hardware aspect (mechanical construction, power distribution and control circuitry) of the industrial robot arm (nicknamed "Karcsi") made by the INDACT project of LEGO Kör.

# Mechanical construction

> TODO complete this section with mCAD models and diagrams

The robot arm is constructed from extruded aluminium bars, machined stainless steel plates, aluminium spacer rods, and held together with screws. A vertical profile and a paralel ball screw are mounted on a rotary base, forming the **Z axis** of the arm's movement. A carriage riding on this profile is connected to another, horizontal profile, which forms the **R axis**. The end-effector, currently a 3D-printed mechanical hand with 5 fingers, is attached to one end of the R axis profile.

The base enables rotary movement (**Phi axis**), realised by a stepper motor and belt-drive. The Z axis movement is realised by a stepper motor, mounted at the upper end of the Z axis profile, driving the ball screw. The R axis movement is realised by a stepper motor mounted on the carriage, driving a studded belt fastened to the R axis profile, similarly to how cheap laser-engravers move. The movement of the fingers is realised by 3 servos inside the palm section of the hand assembly.

We plan to extend the arm's movement capabilities by introducing up to 3 more degrees of freedom through a wrist assembly that would enable tilting (Alpha, Beta axes) and rotation (Theta axis) of the end-effector.

# Power distribution

The arm is powered from the European power grid (**single-phase 230V 50Hz AC**) through the Electronics Box. This cabinet is equipped with a RCD, circuit breaker, mains switch, and 3 separate, galvanically isolated switch-mode AC-to-DC power supplies:

* **PSU1** supplies +24V DC to the stepper motor drivers, is sized to continuously withstand the high currents the motors draw, and wired directly to the motor drivers in the Electronics Box.
* **PSU2** supplies +24V DC to the control electronics, for long distance signal wiring, powering a cooling fan, and optical isolation. This 24V power line is routed through the Main Control Board and auxiliary control boards. The auxiliary control boards create their own 5V power with switch-mode 24VDC-to-5VDC Buck converters.
* **PSU3** supplies +5V DC to the control electronics, to power digital control and measurement circuitry. This 5V power line is routed through the Main Control Board. The Main Control Board creates a 3.3V supply rail for logic-level circuitry with a 5V-to-3V3 Low-Dropout linear regulator (LDO).

The **Emergency Stop** (ESTOP) button breaks the PSU1 24V circuit, but not 5V, so the ESTOP event can be logged by the control logic.

The diagram below shows how power is distributed.

![Power distribution diagram](./power.png)

# Control and data flow

## The main microcontroller

The arm is mainly controlled and programmed through **ROS** (the Robot Operating System) running on a PC connected via Ethernet. Direct control is possible from a remote connected over Wifi, or a wired remote (for debugging and tesing purposes).

An **STM32F7**46ZG microcontroller running **micro-ROS** firmware (located on a **Nucleo-F746** development board, connected to the Main Control Board via pin headers) performs low-level control and telemetry data collection:

* Communicates with the **PC over Ethernet**
* Controls the **motor drivers** for the Phi, R, Z (and optionally Theta) axes
* Controls the current, hand-like  **end-effector's servos**
* Measures and reports the movement speed and position of the arm, through **limit switches, position sensing gates and motor encoders**
* Communicates with the Wifi remote through an **ESP-01 Wifi module**
* Handles **Emergency Stop** events, the **Electronics Box being opened** during operation, or a limit switch being triggered, and stops the arm according to the given rules
* Regulates the temperature of the Electronics Box through a **fan and a temperature sensor**
* Communicates with any future smart end-effector over **MODBUS** (TIA/EIA-485)
* Handles the **wired remote**
* Displays status and error messages on a 2x16 **character lCD**
* Provides additional expansion capability through **SPI and GPIO** ports

The specific microcontroller and development board were chosen due to ROS and Ethernet support, and part availability considerations.

Detailed information (including datasheet) about the microcontroller can be found [here](https://www.st.com/en/microcontrollers-microprocessors/stm32f746zg.html#documentation "STM32F746 documentation"). Documentation for the Nucleo-F746 development board can be found [here](https://www.st.com/en/evaluation-tools/nucleo-f746zg.html#documentation "Nucleo documentation").

## Peripherals

* The STM32F7 microcontroller can only read 4 separate encoders, so for complete and precise control in 6 degrees of freedom, an additional Cortex-M0-based, smaller microcontroller may be soldered directly to the Main Control Board, to extend the timer capabilities of the Nucleo, and thus control the planned **Alpha, Beta and Theta axes**, and report back their position and speed. This microcontroller communicates with the Nucleo over SPI. The exact model of this microcontroller is TBD.
* The Wifi remote connects over Wifi to an ESP-01 Wifi module, connected to the Nucleo over a full-duplex serial (UART) connection (and 4 additional GPIO control signals).
* The wired remote features one pushbutton for each direction of each axis and servo, up to 18 in total, pulling a corresponding GPIO input to ground.
* All axes feature limit switches to prevent movement outside the safe operating movement range. These are all connected to correspoding GPIO pins.
* Some axes feature additional position sensing gates to aid the robot in determining its position. These are all connected to corresponding GPIO pins.
* The Nucleo development board features built-in Ethernet and USB-OTG connectivity, 3 programmable LED outputs, 2 pushbutton inputs, and JTAG and ST-LINK programming interfaces.
* Each axis requires a motor driver (located in the Electronics Box), which are controlled with Step, Direction and Enable signals for each. Direction and Enable signals are simple GPIO outputs, but Step signals need to be generated with timers, preferably a separate timer for each axis. The steper motors connect directly to the motor drivers.
* The 3 servos of the end-effector are controlled by PWM signals, one each. These can be from the same timer.
* All axes feature encoders to accurately measure the rotational speed and angle of the motors. These require 2 channels of an encoder-compatible timer each.
* The Electronics Box cooling fan is PWM-controlled through a MOSFET, and thus requires one channel of any one timer.
* The character LCD module is connected to the Nucleo over a 11-pin paralel GPIO interface, and its backlight LED is PWM-controlled through a MOSFET (also reuiring one channel of any one timer).
* The Electronics Box internal temperature is measured with a BME-280 temperature sensor, connected to the Nucleo over I2C.
* The ESTOP button, and the switch detecting the Electronics Box being open, both pull one GPIO pin each to ground.
* For intercompatibility with future smart end-effectors, a MODBUS (TIA/EIA-485, full-duplex differential UART) link is provided from the Nucleo.

For better signal integrity, electrical safety, reliability and ESD-protection, all low-current signals travelling long distances (encoders, position sensing gates, limit switches, end-effector servos) are optically isolated on both ends, and shifted to 24V levels.
The MODBUS link does not require optical isolation, as it uses a shielded, impedance-controlled, duplex differential twisted pair trunk cable terminated on both ends, operating at 5V line levels, and all devices on it connect via line driver buffers. This works well in the industry for distances over kilometers long, and rather noisy environments. More info can be found [here](https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf "MODBUS over Serial documentation")

The diagram below shows how control signals and peripherals are connected.

![Control signals diagram](./control.png)

Current microcontroller connection list (Nucleo-only):

| Peripheral | Type of connection | Optical isolation | Number of connections |
| :--------: | :----------------: | :---------------: | :------------------------ |
| Limit switch | digital input | yes | 6 (R, Z, Phi axes, 2 each) |
| Position sensing gate | digital input | yes | 4 (R, Z axes, 2 each) |
| Encoder | 32-bit timer digital input | yes | 8 (R, Z, Phi, Theta axes, 2 wires each) |
| Temperature sensor | I2C | no | 2 (SDA, SCL) |
| ESTOP button | digital input | no | 1 |
| Door switch | digital input | no | 1 |
| Wired remote buttons | digital input | no | 6...18 (all axes and servos, 2 each) |
| Cooling fan | PWM output | no | 1 |
| Stepper motor driver | digital and PWM output | no | 12 (R, Z, Phi, Theta axes, 3 wires each) |
| Servo | PWM output | yes | 3 (3 servos, 1 wire each) |
| Display | paralel digital input-output and PWM putput | no | 11+1 (paralel IO + backlight) |
| Wifi module | UART and digital output | no | 4 (Rx, Tx, CH-PD, Reset) |
| MODBUS link | UART | yes | 2 (Rx, Tx) |
| SPI expansion | SPI | no | 4 (SCL, MISO, MOSI, NSS) |
| Nucleo Ethernet | built-in | no | built-in |
| Nucleo Reset and other buttons | built-in | no | built-in |
| Nucleo USB-OTG | built-in | no | built-in |
| Nucleo LEDs | built-in | no | built-in |

`TODO update when aux. microcontroller implemented`

## Nucleo-only pinout and IO configuration

The table below contains pinout information for the STM32F746 Nucleo board.

| Pin | Function | Notes |
| --: | :------- | :---- |
| PA0 | TIM2_CH1 | End-effector Servo 1 PWM control |
| PA1 | RMII_Ref_CLK | Ethernet (built-in) |
| PA2 | RMII_MDIO | Ethernet (built-in) |
| PA3 | TIM2_CH4 | End-effector Servo 2 PWM control |
| PA4 | SPI3_NSS | Expansion SPI Slave-Select |
| PA5 | GPIO_IN | Phi axis Pushbutton for Counterclockwise movement |
| PA6 | TIM3_CH1 | Z axis Encoder A |
| PA7 | RMII_CRS | Ethernet (built-in) |
| PA8 | USB_SOF | USB (built-in) |
| PA9 | USB_VBUS | USB (built-in) |
| PA10 | USB_ID | USB (built-in) |
| PA11 | USB_DM | USB (built-in) |
| PA12 | UDB_DP | USB (built-in) |
| PA13 | SWD_TMS | Serial Wire Debug programming (built-in) |
| PA14 | SWD_TCK | Serial Wire Debug programming (built-in) |
| PA15 | GPIO_IN | Phi axis Pushbutton for Clockwise movement |
| PB0 | LED1 | Green LED (built-in) |
| PB1 | GPIO_IN | Z axis Pushbutton for Up movement |
| PB2 | SPI3_MOSI | Expansion SPI MOSI (DO) |
| PB3 | SW0 | programming switch (built-in) |
| PB4 | GPIO_OUT | Phi axis stepper motor driver Direction |
| PB5 | TIM3_CH2 | Z axis Encoder B |
| PB6 | GPIO_OUT | Phi axis stepper motor driver Enable |
| PB7 | LED2 | Blue LED (built-in) |
| PB8 | TIM10_CH1 | Phi axis stepper motor driver Step PWM control |
| PB9 | TIM11_CH1 | Theta axis stepper motor driver Step PWM control |
| PB10 | TIM2_CH3 | End-effector Servo 3 PWM control |
| PB11 | GPIO_IN | Z axis Pushbutton for Down movement |
| PB12 | GPIO_IN | R axis Pushbutton for Outward movement |
| PB13 | RMII_TXD1 | Ethernet (built-in) |
| PB14 | LED3 | Red LED (built-in) |
| PB15 | GPIO_IN | R axis Pushbutton for Inward movement |
| PC0 |  |  |
| PC1 | RMII_MDC | Ethernet (built-in) |
| PC2 | GPIO_IN | Theta axis Pushbutton for Counterclockwise movement |
| PC3 | GPIO_IN | Theta axis Pushbutton for Clockwise movement |
| PC4 | RMII_RXD0 | Ethernet (built-in) |
| PC5 | RMII_RXD1 | Ethernet (built-in) |
| PC6 | TIM8_CH1 | Theta axis Encoder A |
| PC7 | TIM8_CH2 | Theta axis Encoder B |
| PC8 | GPIO_IN | End-effector Servo 1 Pushbutton for curl finger movement |
| PC9 | GPIO_IN | End-effector Servo 1 Pushbutton for straighten finger movement |
| PC10 | SPI3_CLK | Expansion SPI clock |
| PC11 | SPI3_MISO | Expansion SPI MISO (DI) |
| PC12 | UART5_TX | MODBUS transmit |
| PC13 | USER_BTN | Pushbutton (built-in) |
| PC14 | RCC_OSC32_IN | 32 MHz oscillator (built-in) |
| PC15 | RCC_OSC32_OUT | 32 MHz oscillator (built-in) |
| PD0 | GPIO_IN | Phi axis Limit switch 1 |
| PD1 | GPIO_IN | Phi axis Limit switch 2 |
| PD2 | UART5_RX | MODBUS receive |
| PD3 | GPIO_IN | Z axis limit switch 1 |
| PD4 | GPIO_IN | Z axis limit switch 2 |
| PD5 | GPIO_IN | R axis Limit switch 1 |
| PD6 | GPIO_IN | R axis Limit switch 2 |
| PD7 |  |  |
| PD8 | STLK_RX | ST-link (built-in) |
| PD9 | STLK_TX | ST-link (built-in) |
| PD10 | GPIO_IN | End-effector Servo 2 Pushbutton for curl finger movement |
| PD11 | GPIO_IN | End-effector Servo 2 Pushbutton for straighten finger movement |
| PD12 | TIM4_CH1 | R axis Encoder A |
| PD13 | TIM4_CH2 | R axis Encoder B |
| PD14 | GPIO_IN | End-effector Servo 3 Pushbutton for curl finger movement |
| PD15 | GPIO_IN | End-effector Servo 3 Pushbutton for straighten finger movement |
| PE0 | UART8_RX | Wifi module serial receive |
| PE1 | UART8_TX | Wifi module serial transmit |
| PE2 | GPIO_OUT | Wifi module CH_PD |
| PE3 | GPIO_OUT | Wifi module Reset |
| PE4 |  |  |
| PE5 | TIM9_CH1 | Z axis stepper motor driver Step PWM control |
| PE6 | TIM9_CH2 | R axis stepper motor driver Step PWM control |
| PE7 | GPIO_OUT | Z axis stepper motor driver Direction |
| PE8 | GPIO_OUT | Z axis stepper motor driver Enable |
| PE9 | TIM1_CH1 | Phi axis Encoder A |
| PE10 | GPIO_OUT | R axis stepper motor driver Direction |
| PE11 | TIM1_CH2 | Phi axis Encoder B |
| PE12 | GPIO_OUT | R axis stepper motor driver Enable |
| PE13 | TIM1_CH3 | Cooling fan PWM control |
| PE14 | TIM1_CH4 | LCD backlight PWM control |
| PE15 |  |  |
| PF0 | I2C2_SDA | Temperature sensor data |
| PF1 | I2C2_SCL | Temperature sensor clock |
| PF2 |  |  |
| PF3 |  |  |
| PF4 |  |  |
| PF5 | GPIO_OUT | LCD Data (H) / Command (L) |
| PF6 | GPIO_OUT | LCD Read-Write (MCU->LCD H, LCD->MCU L) |
| PF7 | GPIO_OUT | LCD Enable |
| PF8 | GPIO_OUT | LCD paralel data 0 |
| PF9 | GPIO_OUT | LCD paralel data 1 |
| PF10 | GPIO_OUT | LCD paralel data 2 |
| PF11 | GPIO_OUT | LCD paralel data 3 |
| PF12 | GPIO_OUT | LCD paralel data 4 |
| PF13 | GPIO_OUT | LCD paralel data 5 |
| PF14 | GPIO_OUT | LCD paralel data 6 |
| PF15 | GPIO_OUT | LCD paralel data 7 |
| PG0 | GPIO_EXTI0 | ESTOP interrupt input |
| PG1 | GPIO_IN | Door switch input |
| PG2 | GPIO_IN | Z axis Position sensing gate 1 |
| PG3 | GPIO_IN | Z axis Position sensing gate 2 |
| PG4 | GPIO_IN | R axis Position sensing gate 1 |
| PG5 | GPIO_IN | R axis Position sensing gate 2 |
| PG6 | GPIO_OUT |  |
| PG7 | USB_OverCurrent | USB (built-in) |
| PG8 |  |  |
| PG9 |  |  |
| PG10 |  |  |
| PG11 | RMII_TX_EN | Ethernet (built-in) |
| PG12 |  |  |
| PG13 | RMII_TXD0 | Ethernet (built-in) |
| PG14 | GPIO_OUT | Theta axis stepper motor driver Direction |
| PG15 | GPIO_OUT | Theta axis stepper motor driver Enable |
| PH0 | RCC_OSC_IN | Bypass oscillator (built-in) |
| PH1 | RCC_OSC_OUT | Bypass oscillator (built-in) |
| PDR | Power-down Reset | (built-in) |
| BOOT0 | Boot selector | (built-in) |
| NRST | System reset | (built-in) |

* The MODBUS link uses UART5 with the following parameters: Baudrate 19200, 8 data bits, even parity, 1 stop bit.
* The Wifi module uses GPIO pins PE2, PE3 and UART8 with the following parameters: `TODO fill ESP-01 UART parameters`
* The 16x2 character LCD module uses GPIO pins PF5-PF15 for its paralel interface, and its backlight is PWM controlled using TIMER1, CH4.
* The temperature sensor uses I2C2.
* The cooling fan is PWM controlled using TIMER1, CH3.
* The Phi, Z, R, Theta axis encoders use timers TIMER1, TIMER3, TIMER4 and TIMER8, respectively, in Encoder mode.
* The end-effector servos are PWM controlled using TIMER2.
* The Z and R axis position sensing gates use GPIO pins PG2-PG5 as inputs, with internal pull-up.
* The Emergency Stop (ESTOP) button uses GPIO pin PG0 as input, with internal pull-up, and interrupt EXTI0 on falling edge.
* The Electronics Box door switch uses GPIO pin PG1 as input, with internal pull-up.
* The Phi, Z, R axis limit switches use GPIO pins PD0-PD1, PD3-PD4, PD5-PD6, respectively, as inputs, with internal pull-up.
* The Phi axis stepper motor driver uses TIMER10, CH1 for its Step (PWM) signal, and GPIO pins PB4, PB6, respectively, for its Direction and Enable signals. The Z axis stepper motor driver uses TIMER9, CH1 for its Step (PWM) signal, and GPIO pins PE7, PE8 for its Direction and Enable signals. The R axis stepper motor driver uses TIMER9, CH2 for its Step (PWM) signal, and GPIO pins PE10, PE12 for its Direction and Enable signals. The planned Theta (wrist rotation) axis stepper motor driver uses TIMER11, CH1 for its Step (PWM) signal, and GPIO pins PG14 and PG15 for its Direction and Enable signals.
* The expansion SPI interface is SPI3, in full-duplex Master mode, with hardware NSS output.
* The wired remote (one pushbutton for each direction of each axis) uses GPIO pins PA5 (Phi CCW), PA15 (Phi CW), PB1 (Z up), PB11 (Z down), PB12 (R out), PB15 (R in), PC2 (Theta CCW), PC3 (Theta CW), PC8 (Servo 1 curl), PC9 (Servo 1 straighten), PD10 (Servo 2 curl), PD11 (Servo 2 straighten), PD14 (Servo 3 curl), PD15 (Servo 3 straighten), as inputs, with internal pull-up.
* The rest of the GPIO pins (PC0, PD7, PE4, PE15, PF2, PF3, PF4, PG8, PG9, PG12) are routed to connectors on the Main Control Board, for the possibility of future expansion.

## Nucleo and Cortex-M0 pinouts and IO configuration
`TODO fill this section`

# Electrical Design

The electronics operating the arm are categorised by location as follows:

1. The Electronics Box
    1. Mains voltage circuitry
        * Mains power connector
        * RCD
        * 6A circuit breaker
        * Mains power switch
        * Protective Earth connection
        * Line and Neutral distribution blocks
    2. Galvanically isolated power supplies
        * PSU1: 24V DC, wired directly to motor drivers
        * PSU2: 24V DC, supplied to Main Control Board and further distributed to Auxiliary Control Boards
        * PSU3: 5V DC, supplied to the Main Control Board
    3. Motor drivers
        * Phi axis: stepper
        * Z axis: stepper
        * R axis: stepper

            > All stepper drivers have optically isolated Step (PUL+), Direction (DIR+), Enable (ENA+) inputs referencing PSU3 common point (PUL-, DIR-, ENA-); Stepper motor A+, A-, B+, B- connections; and 24V power connections (directly wired to PSU1).

        * Space for planned Theta, Alpha, Beta axes motor drivers (BLDC or stepper)
    3. The Main Control Board
        * Nucleo-F746 development board {Nucleo-morpho pin header}
        * auxiliary Cortex-M0 microcontroller and related circuitry (capacitors, crystal oscillators, JTAG programming, Boot configuration, etc.) {optionally soldered on when Alpha, Beta and Theta axes are implemented}
        * 5V to 3.3V Low-Dropout (LDO) linear regulator
        * Filtering and decoupling ceramic capacitors, bulk decoupling electrolytic capacitors
        * ESD protection
        * MODBUS differential line buffer (SN75LBC179AD) and termination with pull-up and pull-down resistors and grounded shielding, Rx and Tx routed as impedance-controlled diff. pairs {DB-9 serial connector}
        * ESP-01 Wifi module (3.3V power, UART, GPIO2 connected through a blue LED and series resistor to ground, GPIO0 connected through an 1k resistor to 3.3V supply) {2x4 pin header}
        * BME-280 temperature sensor module (3.3V power, I2C) {1x4 pin header}
        * 2x16 character LCD module (3.3V power, 11-wire paralel interface, contrast setting with trimmer potentiometer between 3.3V and ground, backlight LED PWM-controlled through N-channel MOSFET and series resistor) {1x16 pin header}
        * N-channel MOSFET to PWM-control cooling fan, {2-pin fan header}
        * LTV847 quad optocouplers for slower signals, and SFH6345 digital optocouplers for PWM and fast signals
        * Screw terminals for power input (24V, COM_PSU2, 5V, COM_PSU3), ESTOP (COM_PSU3, GPIO), Electronics Box door switch (COM_PSU3, GPIO), motor driver control signals (6x3 + COM_PSU3)
        * DB-25 connector to wired remote (18 GPIOs, 7x COM_PSU3)
        * Connection to Aux Control Board 1 (3 encoders, 4 limit switches, 2 position sensing gates, 24V, COM_PSU2) and Aux. Control Board 2 (3 encoders, 8 limit switches, 2 position sensing gates, 3 servos, 24V, COM_PSU2)
    4. Other miscellania
        * Cooling fan(s)
        * Door switch
        * ESTOP button
        * the wired remote itself on an 1m cable
2. Mounted on (or near) Z axis
    1. Aux. Control Board 1: mounted on the back of the Z axis profile approx. in the middle
        * 24V to 5V Buck converter and 5V to 3.3V LDO, LED indicator
        * decoupling capacitors
        * Optocouplers
        * connectors to Phi, R and Z axis encoders, Phi and Z axis limit switches, Z axis position sensing gates
        * connector to Main Cntrol Board
    2. Z axis stepper motor + Z axis encoder
    3. Phi axis stepper motor + Phi axis encoder
    4. R axis stepper motor + R axis encoder
    5. Z axis limit switches and position gates
    6. Phi axis limit switches
3. Mounted on (or near) R axis
    1. Aux. Control Board 2: mounted on the fron of the R axis profile towards the end-effector
        * 24V to 5V Buck converter and 5V to 3.3V LDO, LED indicator
        * decoupling capacitors
        * Optocouplers
        * connectors to end-effector servos, Alpha, Beta and Theta axis encoders, R, Alpha, Beta and Theta axis limit switches, R axis position gates
        * MODBUS termination (without pull-up and pull-down resistors and shield connection), line driver, DB-9 connector, output to end-effector
        * connector to Main Control Board
    2. R axis limit switches and position gates
    3. Alpha, Beta and Theta axis motors (planned), with encoders and limit switches
    4. The end-effector

The Main Control Board is designed with Altium Designer. The Auxiliary Control Boards are designed with KiCAD as one board, manufactured together, and separated later along the perforated edge. All boards are assembled in-house by project members.

Connector pinouts are marked on boards, and most connectors used can only be connected in the correct orientation.

## Main Control Board schematic

`TODO add images`

## Main Control Board PCB layout

## Auxiliary Control Boards schematic

## Auxiliary Control Boards PCB layout
