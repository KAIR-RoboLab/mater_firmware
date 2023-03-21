# Mater firmware

## Changing ROS_DOMAIN_ID

In [platform.ini](./platformio.ini) change line:
``` ini
build_flags =
    '-D ROS_DOMAIN_ID=my_cool_domain_id'
```

## Pinout

Pinout is based on official [documentation](https://docs.google.com/document/d/1DJASwxgbattM37V4AIlJVR4pxukq0up25LppA8-z_AY/edit).

### Groove connectors

|    Name    | GPIO |   PWM  | SPI  | I2C  | UART | Analog |
|:----------:|:----:|:------:|:----:|:----:|:----:|:------:|
| GROOVE_1_A |  0   | PWM0-A | SDI0 | SDA0 | TX0  |    -   |
| GROOVE_1_B |  1   | PWM0-B | CSn0 | SCL0 | RX0  |    -   |
| GROOVE_2_A |  2   | PWM1-A | SCK0 | SDA1 |  -   |    -   |
| GROOVE_2_B |  3   | PWM1-B | SDO0 | SCL1 |  -   |    -   |
| GROOVE_3_A |  4   | PWM2-A | SDI0 | SDA0 | TX1  |    -   |
| GROOVE_3_B |  5   | PWM2-B | CSn0 | SCL0 | RX1  |    -   |
| GROOVE_4_A |  16  | PWM0-A | SDI0 | SDA0 | TX0  |    -   |
| GROOVE_4_B |  17  | PWM0-B | CSn0 | SCL0 | RX0  |    -   |
| GROOVE_5_A |  6   | PWM3-A | SCK0 | SDA1 |  -   |    -   |
| GROOVE_5_B |  26  | PWM5-A |   -  | SDA1 |  -   |  ADC0  |
| GROOVE_6_A |  26  | PWM5-A |   -  | SDA1 |  -   |  ADC0  |
| GROOVE_6_B |  27  | PWM5-B |   -  | SCL1 |  -   |  ADC1  |
| GROOVE_7_A |  7   | PWM3-B | SDO0 | SCL1 |  -   |    -   |
| GROOVE_7_B |  28  | PWM6-A |   -  |  -   |  -   |  ADC2  |

### Other connectors

|     Name     | GPIO | Function          |
|:------------:|:----:|-------------------|
| SERVO_0      |  12  | Servo on pin GP12 |
| SERVO_1      |  13  | Servo on pin GP13 |
| SERVO_2      |  14  | Servo on pin GP14 |
| SERVO_3      |  15  | Servo on pin GP15 |
| MOTOR_1_A    |  8   | Motor 1 pin A     |
| MOTOR_1_B    |  9   | Motor 1 pin B     |
| MOTOR_2_A    |  10  | Motor 2 pin A     |
| MOTOR_2_B    |  11  | Motor 2 pin B     |
| BUTTON_GP_20 |  20  | Button on pin 20  |
| BUTTON_GP_21 |  21  | Button on pin 21  |
| BUZZER       |  11  | Buzzer pin        |
| RGB_DIODE    |  22  | WS2812B pin       |