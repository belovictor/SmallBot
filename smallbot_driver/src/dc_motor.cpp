#include "dc_motor.hpp"

DCMOTOR::DCMOTOR(int bus, int address, bool rA, bool rB) {
    pca9685 = new PCA9685(bus, address);
    pca9685->setPWMFreq(50);
    reverseA = rA;
    reverseB = rB;
}

DCMOTOR::~DCMOTOR() {
    delete pca9685;
}

void DCMOTOR::motorRun(int motor, int direction, int speed) {
    if (speed > 100) {
        speed = 100;
    }
    if (speed < 0) {
        speed = 0;
    }

    if (motor == 0) {
        pca9685->setPWM(PWMA, 4095/100*speed);
        if ((direction == 0 && reverseA == false) || (direction == 1 && reverseA == true)) {
            pca9685->setLevel(AIN1, 0);
            pca9685->setLevel(AIN2, 1);
        } else {
            pca9685->setLevel(AIN1, 1);
            pca9685->setLevel(AIN2, 0);
        }
    } else if (motor == 1) {
        pca9685->setPWM(PWMB, 4095/100*speed);
        if ((direction == 0 && reverseB == false) || (direction == 1 && reverseB == true)) {
            pca9685->setLevel(BIN1, 0);
            pca9685->setLevel(BIN2, 1);
        } else {
            pca9685->setLevel(BIN1, 1);
            pca9685->setLevel(BIN2, 0);
        }
    }
}

void DCMOTOR::motorStop(int motor) {
    if (motor == 0) {
        pca9685->setPWM(PWMA, 0);
    } else if (motor == 1) {
        pca9685->setPWM(PWMB, 0);
    }
}
