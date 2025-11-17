#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20

class keyboard : public rclcpp::Node
{
public:
    keyboard() : Node("keyboard_control")
    {
        // Read parameters and set default values
        this->declare_parameter<float>("linear_v", 2.4);
        this->get_parameter("linear_v", linear_v);

        this->declare_parameter<float>("angular_v", 2.4);
        this->get_parameter("angular_v", angular_v);

        this->declare_parameter<std::string>("publish_topic", "/youBot/cmd_vel");
        this->get_parameter("publish_topic", publish_topic);

        RCLCPP_INFO(this->get_logger(), "[keyboard_control] v=%.2f, w=%.2f, topic=%s", linear_v, angular_v, publish_topic.c_str());

        // Set publisher topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(publish_topic, 1);

        // configure interruption
        // signal(SIGINT,this->quit);
    }

    void keyLoop()
    {
        double linear_, angular_;
        char c;
        bool dirty = false;

        // get the console in raw mode
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);

        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
        puts("Reading from keyboard");
        puts("---------------------------");
        puts("Use arrow keys to move the robot.");
        puts("Press the space bar to stop the robot.");
        puts("Press q to stop the program");

        // Endless loop
        while (rclcpp::ok())
        {
            // get the next event from the keyboard
            if (read(kfd, &c, 1) < 0)
            {
                perror("read():");
                exit(-1);
            }

            linear_ = angular_ = 0;
            // RCLCPP_DEBUG(this->get_logger(),"value: 0x%02X\n", c);
            switch (c)
            {
            case KEYCODE_L:
                RCLCPP_DEBUG(this->get_logger(), "LEFT");
                angular_ = angular_v;
                linear_ = 0.0;
                dirty = true;
                break;
            case KEYCODE_R:
                RCLCPP_DEBUG(this->get_logger(), "RIGHT");
                angular_ = -angular_v;
                linear_ = 0;
                dirty = true;
                break;
            case KEYCODE_U:
                RCLCPP_DEBUG(this->get_logger(), "UP");
                linear_ = linear_v;
                angular_ = 0;
                dirty = true;
                break;
            case KEYCODE_D:
                RCLCPP_DEBUG(this->get_logger(), "DOWN");
                linear_ = -linear_v;
                angular_ = 0;
                dirty = true;
                break;
            case KEYCODE_SPACE:
                RCLCPP_DEBUG(this->get_logger(), "STOP");
                linear_ = 0;
                angular_ = 0;
                dirty = true;
                break;
            case KEYCODE_Q:
                RCLCPP_DEBUG(this->get_logger(), "QUIT");
                RCLCPP_INFO(this->get_logger(), "You quit the Keyboard_Control successfully");
                return;
                break;
            }
            geometry_msgs::msg::Twist twist;
            twist.angular.z = angular_;
            twist.linear.x = linear_;
            if (dirty == true)
            {
                publisher_->publish(twist);
                dirty = false;
            }
        } // endless-for

        return;
    }

    void quit()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
        exit(0);
    }

private:
    // vars
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double linear_v, angular_v;
    std::string publish_topic;
    int kfd = 0;
    struct termios cooked, raw;
};

// MAIN
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // create object
    std::shared_ptr<keyboard> myNode = std::make_shared<keyboard>();

    // Call de keyloop method
    myNode->keyLoop();
    myNode->quit();
    // rclcpp::spin(std::make_shared<keyboard>());
    rclcpp::shutdown();
    return 0;
}
