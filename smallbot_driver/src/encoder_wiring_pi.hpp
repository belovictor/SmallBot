#ifndef ENCODER_WIRING_PI_HPP_
#define ENCODER_WIRING_PI_HPP_

#include <ros/ros.h>
#include <wiringPi.h>

// BCM pinout is used here
#define ENCODER_1_PIN_A 27
#define ENCODER_1_PIN_B 4
#define ENCODER_2_PIN_A 23
#define ENCODER_2_PIN_B 18
#define ENCODER_3_PIN_A 10
#define ENCODER_3_PIN_B 22
#define ENCODER_4_PIN_A 25
#define ENCODER_4_PIN_B 24
#define ENCODER_5_PIN_A 11
#define ENCODER_5_PIN_B 9
#define ENCODER_6_PIN_A 7
#define ENCODER_6_PIN_B 8

#define PULSES_PER_REVOLUTION 748 // 374 pulses per evolution * 2 encoders

namespace EncoderWiringPiISR {

    volatile long encoderPosition1;
    volatile long encoderPosition2;
    volatile long encoderPosition3;
    volatile long encoderPosition4;
    volatile long encoderPosition5;
    volatile long encoderPosition6;
    volatile uint8_t encoderState1;
    volatile uint8_t encoderState2;
    volatile uint8_t encoderState3;
    volatile uint8_t encoderState4;
    volatile uint8_t encoderState5;
    volatile uint8_t encoderState6;

    void encoderISR(const int pinA, const int pinB, volatile long &encoderPosition, volatile uint8_t &encoderState) {
        uint8_t valA = digitalRead(pinA);
        uint8_t valB = digitalRead(pinB);
        uint8_t s = encoderState & 3;
        if (valA) s |= 4;
        if (valB) s |= 8; 
        encoderState = (s >> 2);
        if (s == 1 || s == 7 || s == 8 || s == 14)
            encoderPosition++;
        else if (s == 2 || s == 4 || s == 11 || s == 13)
            encoderPosition--;
        else if (s == 3 || s == 12)
            encoderPosition += 2;
        else if (s == 6 || s == 9)
            encoderPosition -= 2;
    }

    void encoderISR1(void) {
        encoderISR(ENCODER_1_PIN_A, ENCODER_1_PIN_B, encoderPosition1, encoderState1);
    }

    void encoderISR2(void) {
        encoderISR(ENCODER_2_PIN_A, ENCODER_2_PIN_B, encoderPosition2, encoderState2);
    }
    
    void encoderISR3(void) {
        encoderISR(ENCODER_3_PIN_A, ENCODER_3_PIN_B, encoderPosition3, encoderState3);
    }

    void encoderISR4(void) {
        encoderISR(ENCODER_4_PIN_A, ENCODER_4_PIN_B, encoderPosition4, encoderState4);
    }

    void encoderISR5(void) {
        encoderISR(ENCODER_5_PIN_A, ENCODER_5_PIN_B, encoderPosition5, encoderState5);
    }

    void encoderISR6(void) {
        encoderISR(ENCODER_6_PIN_A, ENCODER_6_PIN_B, encoderPosition6, encoderState6);
    }
}

class EncoderWiringPi {
public:
    EncoderWiringPi(const int &pinA, const int &pinB, void (*isrFunction)(void), volatile long* encoderPosition);
    double getAngle();
private:
    int _pinA;
    int _pinB;
    volatile long* _encoderPosition;
    double _initial_angle;
    double ticks2Angle(long position);
};

EncoderWiringPi::EncoderWiringPi(const int &pinA, const int &pinB, void (*isrFunction)(void), volatile long* encoderPosition) {
    _encoderPosition = encoderPosition;

    if (wiringPiSetupSys() < 0) {
        ROS_ERROR("Encoder wiringPi error: GPIO setup error");
        throw std::runtime_error("");
    }
    ROS_INFO("Encoder wiringPi: GPIO setup");

    _pinA = pinA;
    _pinB = pinB;
    pinMode(_pinA, INPUT);
    pinMode(_pinB, INPUT);
    pullUpDnControl(_pinA, PUD_UP);
    pullUpDnControl(_pinB, PUD_UP);

    if (wiringPiISR(_pinA, INT_EDGE_BOTH, isrFunction) < 0) {
        ROS_ERROR("Encoder wiringPi error: ISR pinA error");
        throw std::runtime_error("");
    }

    if (wiringPiISR(_pinB, INT_EDGE_BOTH, isrFunction) < 0) {
        ROS_ERROR("Encoder wiringPi error: ISR pinB error");
        throw std::runtime_error("");
    }

    _initial_angle = ticks2Angle(*_encoderPosition);
    ROS_INFO("Encoder wiringPi: ISR setup");
}

double EncoderWiringPi::getAngle() {
    double current_angle = ticks2Angle(*_encoderPosition);
    return current_angle - _initial_angle;
}

double EncoderWiringPi::ticks2Angle(long position) {
	return position * ((double)2 * M_PI / PULSES_PER_REVOLUTION / 2);
}

#endif // ENCODER_WIRING_PI_HPP_