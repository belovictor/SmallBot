#include "encoder_jetsongpio.hpp"
#include <boost/chrono.hpp>
#include <std_msgs/Float64.h>

#undef PI
#ifdef PI
#define EncoderGpio EncoderWiringPi
#define EncoderGpioISR EncoderWiringPiISR
#else
#define EncoderGpio EncoderJetsonGpio
#define EncoderGpioISR EncoderJetsonGpioISR
#endif

typedef boost::chrono::steady_clock time_source;

class Encoders6WD {
    public:
        Encoders6WD(double update_rate);
    private:
        ros::NodeHandle node;

        ros::Publisher front_left_wheel_angle_pub;
        ros::Publisher front_right_wheel_angle_pub;
        ros::Publisher middle_left_wheel_angle_pub;
        ros::Publisher middle_right_wheel_angle_pub;
        ros::Publisher rear_left_wheel_angle_pub;
        ros::Publisher rear_right_wheel_angle_pub;

        ros::Publisher front_left_wheel_velocity_pub;
        ros::Publisher front_right_wheel_velocity_pub;
        ros::Publisher middle_left_wheel_velocity_pub;
        ros::Publisher middle_right_wheel_velocity_pub;
        ros::Publisher rear_left_wheel_velocity_pub;
        ros::Publisher rear_right_wheel_velocity_pub;

        ros::Timer encoders_timer;

        std_msgs::Float64 front_left_wheel_angle_msg;
        std_msgs::Float64 front_right_wheel_angle_msg;
        std_msgs::Float64 middle_left_wheel_angle_msg;
        std_msgs::Float64 middle_right_wheel_angle_msg;
        std_msgs::Float64 rear_left_wheel_angle_msg;
        std_msgs::Float64 rear_right_wheel_angle_msg;

        std_msgs::Float64 front_left_wheel_velocity_msg;
        std_msgs::Float64 front_right_wheel_velocity_msg;
        std_msgs::Float64 middle_left_wheel_velocity_msg;
        std_msgs::Float64 middle_right_wheel_velocity_msg;
        std_msgs::Float64 rear_left_wheel_velocity_msg;
        std_msgs::Float64 rear_right_wheel_velocity_msg;

        EncoderGpio encoder_front_left;
        EncoderGpio encoder_front_right;
        EncoderGpio encoder_middle_left;
        EncoderGpio encoder_middle_right;
        EncoderGpio encoder_rear_left;
        EncoderGpio encoder_rear_right;

        double front_left_wheel_angle;
        double front_right_wheel_angle;
        double middle_left_wheel_angle;
        double middle_right_wheel_angle;
        double rear_left_wheel_angle;
        double rear_right_wheel_angle;

        double front_left_wheel_velocity;
        double front_right_wheel_velocity;
        double middle_left_wheel_velocity;
        double middle_right_wheel_velocity;
        double rear_left_wheel_velocity;
        double rear_right_wheel_velocity;

        double front_left_wheel_position;
        double front_right_wheel_position;
        double middle_left_wheel_position;
        double middle_right_wheel_position;
        double rear_left_wheel_position;
        double rear_right_wheel_position;

        time_source::time_point last_time;

        void encodersCallback(const ros::TimerEvent& event);
};

Encoders6WD::Encoders6WD(double update_rate):
    encoder_front_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderGpioISR::encoderISR1, &EncoderGpioISR::encoderPosition1),
    encoder_front_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderGpioISR::encoderISR2, &EncoderGpioISR::encoderPosition2),
    encoder_middle_left(ENCODER_3_PIN_A, ENCODER_3_PIN_B, &EncoderGpioISR::encoderISR3, &EncoderGpioISR::encoderPosition3),
    encoder_middle_right(ENCODER_4_PIN_A, ENCODER_4_PIN_B, &EncoderGpioISR::encoderISR4, &EncoderGpioISR::encoderPosition4),
    encoder_rear_left(ENCODER_5_PIN_A, ENCODER_5_PIN_B, &EncoderGpioISR::encoderISR5, &EncoderGpioISR::encoderPosition5),
    encoder_rear_right(ENCODER_6_PIN_A, ENCODER_6_PIN_B, &EncoderGpioISR::encoderISR6, &EncoderGpioISR::encoderPosition6)
    {
    ROS_INFO("Encoder: initializing angle publishers");
    front_left_wheel_angle_pub = node.advertise<std_msgs::Float64>("/smallbot/front_left_wheel/angle", 1);
    front_right_wheel_angle_pub = node.advertise<std_msgs::Float64>("/smallbot/front_right_wheel/angle", 1);
    middle_left_wheel_angle_pub = node.advertise<std_msgs::Float64>("/smallbot/middle_left_wheel/angle", 1);
    middle_right_wheel_angle_pub = node.advertise<std_msgs::Float64>("/smallbot/middle_right_wheel/angle", 1);
    rear_left_wheel_angle_pub = node.advertise<std_msgs::Float64>("/smallbot/rear_left_wheel/angle", 1);
    rear_right_wheel_angle_pub = node.advertise<std_msgs::Float64>("/smallbot/rear_right_wheel/angle", 1);
    ROS_INFO("Encoder: initializing velocity publishers");
    front_left_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/smallbot/front_left_wheel/current_velocity", 1);
    front_right_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/smallbot/front_right_wheel/current_velocity", 1);
    middle_left_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/smallbot/middle_left_wheel/current_velocity", 1);
    middle_right_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/smallbot/middle_right_wheel/current_velocity", 1);
    rear_left_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/smallbot/rear_left_wheel/current_velocity", 1);
    rear_right_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/smallbot/rear_right_wheel/current_velocity", 1);
    ROS_INFO("Encoder: initializing publishing timer");
    encoders_timer = node.createTimer(ros::Duration(update_rate), &Encoders6WD::encodersCallback, this);
    ROS_INFO("Encoder: initialization complete");
}

void Encoders6WD::encodersCallback(const ros::TimerEvent& event) {
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    front_left_wheel_angle = -1 * encoder_front_left.getAngle();
    front_right_wheel_angle = 1 * encoder_front_right.getAngle();
    middle_left_wheel_angle = -1 * encoder_middle_left.getAngle();
    middle_right_wheel_angle = 1 * encoder_middle_right.getAngle();
    rear_left_wheel_angle = -1 * encoder_rear_left.getAngle();
    rear_right_wheel_angle = 1 * encoder_rear_right.getAngle();

    front_left_wheel_angle_msg.data = front_left_wheel_angle;
    front_right_wheel_angle_msg.data = front_right_wheel_angle;
    middle_left_wheel_angle_msg.data = middle_left_wheel_angle;
    middle_right_wheel_angle_msg.data = middle_right_wheel_angle;
    rear_left_wheel_angle_msg.data = rear_left_wheel_angle;
    rear_right_wheel_angle_msg.data = rear_right_wheel_angle;

    front_left_wheel_angle_pub.publish(front_left_wheel_angle_msg);
    front_right_wheel_angle_pub.publish(front_right_wheel_angle_msg);
    middle_left_wheel_angle_pub.publish(middle_left_wheel_angle_msg);
    middle_right_wheel_angle_pub.publish(middle_right_wheel_angle_msg);
    rear_left_wheel_angle_pub.publish(rear_left_wheel_angle_msg);
    rear_right_wheel_angle_pub.publish(rear_right_wheel_angle_msg);

    double delta_front_left_wheel = front_left_wheel_angle - front_left_wheel_position;
    double delta_front_right_wheel = front_right_wheel_angle - front_right_wheel_position;
    double delta_middle_left_wheel = middle_left_wheel_angle - middle_left_wheel_position;
    double delta_middle_right_wheel = middle_right_wheel_angle - middle_right_wheel_position;
    double delta_rear_left_wheel = rear_left_wheel_angle - rear_left_wheel_position;
    double delta_rear_right_wheel = rear_right_wheel_angle - rear_right_wheel_position;

    front_left_wheel_position += delta_front_left_wheel;
    front_right_wheel_position += delta_front_right_wheel;
    middle_left_wheel_position += delta_middle_left_wheel;
    middle_right_wheel_position += delta_middle_right_wheel;
    rear_left_wheel_position += delta_rear_left_wheel;
    rear_right_wheel_position += delta_rear_right_wheel;

    front_left_wheel_velocity = delta_front_left_wheel / elapsed.toSec();
    front_right_wheel_velocity = delta_front_right_wheel / elapsed.toSec();
    middle_left_wheel_velocity = delta_middle_left_wheel / elapsed.toSec();
    middle_right_wheel_velocity = delta_middle_right_wheel / elapsed.toSec();
    rear_left_wheel_velocity = delta_rear_left_wheel / elapsed.toSec();
    rear_right_wheel_velocity = delta_rear_right_wheel / elapsed.toSec();

    front_left_wheel_velocity_msg.data = front_left_wheel_velocity;
    front_right_wheel_velocity_msg.data = front_right_wheel_velocity;
    middle_left_wheel_velocity_msg.data = middle_left_wheel_velocity;
    middle_right_wheel_velocity_msg.data = middle_right_wheel_velocity;
    rear_left_wheel_velocity_msg.data = rear_left_wheel_velocity;
    rear_right_wheel_velocity_msg.data = rear_right_wheel_velocity;

    front_left_wheel_velocity_pub.publish(front_left_wheel_velocity_msg);
    front_right_wheel_velocity_pub.publish(front_right_wheel_velocity_msg);
    middle_left_wheel_velocity_pub.publish(middle_left_wheel_velocity_msg);
    middle_right_wheel_velocity_pub.publish(middle_right_wheel_velocity_msg);
    rear_left_wheel_velocity_pub.publish(rear_left_wheel_velocity_msg);
    rear_right_wheel_velocity_pub.publish(rear_right_wheel_velocity_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "encoders");
    Encoders6WD encoders_6wd(0.01);
    ros::spin();
    #ifndef PI
    GPIO::cleanup();
    #endif
	return 0;
}
