#ifndef __MAKER_PI_PINOUT__
#define __MAKER_PI_PINOUT__

namespace maker_pi {
    // servo motors
    static const unsigned int SERVO_0 = 12;
    static const unsigned int SERVO_1 = 13;
    static const unsigned int SERVO_2 = 14;
    static const unsigned int SERVO_3 = 15;

    // dc motors
    static const unsigned int MOTOR_1_A = 8;
    static const unsigned int MOTOR_1_B = 9;
    static const unsigned int MOTOR_2_A = 10;
    static const unsigned int MOTOR_2_B = 11;

    // buttons
    static const unsigned int BUTTON_GP_20 = 20;
    static const unsigned int BUTTON_GP_21 = 21;

    // misc
    static const unsigned int BUZZER = 11;
    static const unsigned int RGB_DIODE = 22;
    static const unsigned int BATTER_ADC = 29;

    // groove
    static const unsigned int GROOVE_1_A = 0;
    static const unsigned int GROOVE_1_B = 1;
    static const unsigned int GROOVE_2_A = 2;
    static const unsigned int GROOVE_2_B = 3;
    static const unsigned int GROOVE_3_A = 4;
    static const unsigned int GROOVE_3_B = 5;
    static const unsigned int GROOVE_4_A = 16;
    static const unsigned int GROOVE_4_B = 17;
    static const unsigned int GROOVE_5_A = 6;
    static const unsigned int GROOVE_5_B = 26;
    static const unsigned int GROOVE_6_A = 26;
    static const unsigned int GROOVE_6_B = 27;
    static const unsigned int GROOVE_7_A = 7;
    static const unsigned int GROOVE_7_B = 28;
  
    // constant for PWM
    static const int MAX_PWM = 65535;
} // namespace maker_pi    

#endif //__MAKER_PI_PINOUT__