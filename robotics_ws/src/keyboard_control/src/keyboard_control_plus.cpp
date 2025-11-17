#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <string.h>

#define KEYCODE_Right 0x43
#define KEYCODE_Left 0x44
#define KEYCODE_Up 0x41
#define KEYCODE_Down 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_P 0x70
#define KEYCODE_SPACE 0x20

class keyboard : public rclcpp::Node
{
public:
    keyboard() : Node("keyboard_control_plus")
    {
        // Read parameters and set default values
        this->declare_parameter<float>("linear_v_inc", 0.5);
        this->get_parameter("linear_v_inc", linear_v_inc);
        printf("[KeyboardControlPlus] linear_v_inc: %.2f\n", linear_v_inc);

        this->declare_parameter<float>("angular_v_inc", 0.5);
        this->get_parameter("angular_v_inc", angular_v_inc);
        printf("[KeyboardControlPlus] angular_v_inc: %.2f\n", angular_v_inc);

        this->declare_parameter<std::string>("publish_topic", "cmd_vel");
        this->get_parameter("publish_topic", publish_topic);
        printf("[KeyboardControlPlus] publish_topic: %s\n", publish_topic.c_str());

        // Set publisher topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(publish_topic, 1);

        // init vars
        exit_prog = false;
        key = 0;
        paused = true;
        linear_v = 0.0;
        angular_v = 0.0;
    }

    /* Non-blocking get char function!
     * Linux (POSIX) implementation of _kbhit().
     * Morgan McGuire, morgan@cs.brown.edu
     */
    int _kbhit()
    {
        static const int STDIN = 0;
        static bool initialized = false;

        if (!initialized)
        {
            // Use termios to turn off line buffering
            termios term;
            tcgetattr(STDIN, &term);
            term.c_lflag &= ~ICANON;
            tcsetattr(STDIN, TCSANOW, &term);
            setbuf(stdin, NULL);
            initialized = true;
        }

        int bytesWaiting;
        ioctl(STDIN, FIONREAD, &bytesWaiting);
        return bytesWaiting;
    }

    void keyLoop()
    {
        printf("Reading from keyboard\n");
        printf("---------------------------\n");
        printf("Use arrow keys to move the robot.\n");
        printf("Press the space bar to stop the robot.\n");
        printf("Press p to pause the program\n");
        printf("Press q to stop the program\n");

        // Endless loop
        while (rclcpp::ok() && !exit_prog)
        {
            // similar to getchat, but non-blocking
            if (_kbhit())
            {
                key = getchar(); /* consume the character */
                // printf("readed value %u\n",key);
                switch (key)
                {
                case KEYCODE_Up:
                {
                    paused = false;
                    linear_v += linear_v_inc;
                    break;
                }
                case KEYCODE_Down:
                {
                    paused = false;
                    linear_v -= linear_v_inc;
                    break;
                }
                case KEYCODE_Right:
                {
                    paused = false;
                    angular_v -= angular_v_inc;
                    break;
                }
                case KEYCODE_Left:
                {
                    paused = false;
                    angular_v += angular_v_inc;
                    break;
                }
                case KEYCODE_SPACE:
                {
                    paused = false;
                    linear_v = 0.0;
                    angular_v = 0.0;
                    break;
                }
                case KEYCODE_P:
                {
                    paused = true;
                    break;
                }
                case KEYCODE_Q:
                {
                    linear_v = 0.0;
                    angular_v = 0.0;
                    exit_prog = true;
                    system("stty cooked");
                }
                }
            }

            // publish current cmd_vel
            if (!paused)
            {
                geometry_msgs::msg::Twist twist;
                twist.angular.z = angular_v;
                twist.linear.x = linear_v;
                publisher_->publish(twist);
            }

            // sleep a while
            fflush(stdout);
            // rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        return;
    }

private:
    // vars
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double linear_v, angular_v, linear_v_inc, angular_v_inc;
    std::string publish_topic;
    int key;
    bool paused, exit_prog;
};

// MAIN
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // create object
    std::shared_ptr<keyboard> myNode = std::make_shared<keyboard>();

    // Call de keyloop method
    myNode->keyLoop();
    rclcpp::shutdown();
    return 0;
}
