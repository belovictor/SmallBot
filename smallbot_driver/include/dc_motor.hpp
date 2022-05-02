#ifndef DC_MOTOR_HPP_
#define DC_MOTOR_HPP_

#include <PCA9685.h>

#define MOTOR_DIRECTION_FORWARD 0
#define MOTOR_DIRECTION_BACKWARD 1
#define PWMA 0
#define AIN1 1
#define AIN2 2
#define PWMB 5
#define BIN1 3
#define BIN2 4

class DCMOTOR {
    public:
        DCMOTOR(int, int, bool, bool);
        virtual ~DCMOTOR();

        void motorRun(int, int, int);
        void motorStop(int);
    private:
        PCA9685 *pca9685;
        bool reverseA, reverseB;
};

#endif
