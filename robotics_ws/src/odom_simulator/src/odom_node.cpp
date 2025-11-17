#include "Utils.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using tf2::Vector3;

class OdomSim : public rclcpp::Node
{
public:
    OdomSim();
    void odomCallback(const Odometry::SharedPtr msg);
    void initialPoseCallback(const PoseWithCovarianceStamped::SharedPtr msg);

private:
    void UpdateState(float deltaTime);

private:
    rclcpp::Subscription<Odometry>::SharedPtr odomSub;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr initialPoseSub;
    rclcpp::Publisher<Odometry>::SharedPtr pub;

    tf2_ros::TransformBroadcaster tf_broadcaster;
    std::string tf_frame;

    struct State
    {
        tf2::Transform currentTransform;
        Vector3 linearSpeed;
        float angularSpeed;
        rclcpp::Time lastUpdate;
    };
    State state;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomSim>();
    rclcpp::spin(node);
    return 0;
}

// Implementation

inline OdomSim::OdomSim()
    : Node("odom_sim"),
      tf_broadcaster(*this)
{
    std::string topic_in = get_parameter_or<std::string>("topic_in", "/base_pose_ground_truth");
    odomSub = create_subscription<Odometry>(topic_in, 1, std::bind(&OdomSim::odomCallback, this, std::placeholders::_1));
    std::string topic_out = get_parameter_or<std::string>("topic_out", "/noisy_odom");
    pub = create_publisher<Odometry>(topic_out, 1);

    initialPoseSub = create_subscription<PoseWithCovarianceStamped>("initialpose", 1, std::bind(&OdomSim::initialPoseCallback, this, std::placeholders::_1));

    tf_frame = get_parameter_or<std::string>("tf_frame", "noisy_odom");

    state.lastUpdate = rclcpp::Time(0, 0, RCL_ROS_TIME);
    state.currentTransform.setIdentity();
}

inline void OdomSim::initialPoseCallback(const PoseWithCovarianceStamped::SharedPtr msg)
{
    tf2::Vector3 position;
    tf2::fromMsg(msg->pose.pose.position, position);
    state.currentTransform.setOrigin(position);

    tf2::Quaternion rotation;
    tf2::fromMsg(msg->pose.pose.orientation, rotation);
    state.currentTransform.setRotation(rotation);
}

inline void OdomSim::odomCallback(const Odometry::SharedPtr msg)
{
    rclcpp::Time stamp = msg->header.stamp;
    float deltaTime = (stamp - state.lastUpdate).seconds();
    state.lastUpdate = stamp;

    // odom msg from mvsim has velocity in world space, move it to local space
    tf2::Vector3 worldSpaceVelocity;
    tf2::fromMsg(msg->twist.twist.linear, worldSpaceVelocity);
    tf2::Quaternion robotRotation;
    tf2::fromMsg(msg->pose.pose.orientation, robotRotation);
    tf2::Vector3 localVelocity = tf2::quatRotate(robotRotation.inverse(), worldSpaceVelocity);

    // simulate a delay in the velocities
    // {
        // float acc = 1.f;
        // state.linearSpeed.setX(Math::MoveTowards(state.linearSpeed.x(), localVelocity.x(), acc * deltaTime));
        // state.linearSpeed.setY(Math::MoveTowards(state.linearSpeed.y(), localVelocity.y(), acc * deltaTime));
        // state.linearSpeed.setZ(Math::MoveTowards(state.linearSpeed.z(), localVelocity.z(), acc * deltaTime));
        // 
        // float angleAcc = 1.f;
        // state.angularSpeed = Math::MoveTowards(state.angularSpeed, msg->twist.twist.angular.z, angleAcc * deltaTime);
    // }

    // interestingly, we don't actually need to alter the odometry, because the ground truth topic uses the cmd_vel value as the velocities
    // so, even though the odometry pose is perfect, the odometry *velocity* is unreliable
    // if we just integrate that over time, we have super realistic unreliable odometry
    {
        state.linearSpeed.setX(localVelocity.x());
        state.linearSpeed.setY(localVelocity.y());
        state.linearSpeed.setZ(localVelocity.z());
    }

    state.angularSpeed = msg->twist.twist.angular.z;

    if (deltaTime < 0.5) // avoid weird results if the time step is unusually large
        UpdateState(deltaTime);

    // publish
    // -------------

    // odom msg
    Odometry noisyOdom;
    noisyOdom.header.frame_id = "map";
    noisyOdom.header.stamp = msg->header.stamp;
    noisyOdom.pose.pose.position.x = state.currentTransform.getOrigin().x();
    noisyOdom.pose.pose.position.y = state.currentTransform.getOrigin().y();
    noisyOdom.pose.pose.position.z = state.currentTransform.getOrigin().z();
    noisyOdom.pose.pose.orientation = tf2::toMsg(state.currentTransform.getRotation());

    noisyOdom.twist.twist.linear = tf2::toMsg(state.linearSpeed);
    noisyOdom.twist.twist.angular.z = state.angularSpeed;

    pub->publish(noisyOdom);

    // noisyOdom -> base_link TF
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = tf_frame;
    tf.child_frame_id = "base_link";
    tf.transform.translation = tf2::toMsg(state.currentTransform.getOrigin());
    tf.transform.rotation = tf2::toMsg(state.currentTransform.getRotation());
    tf_broadcaster.sendTransform(tf);
}

inline void OdomSim::UpdateState(float deltaTime)
{
    // calculate and apply the movement
    float angularSpeed = state.angularSpeed;
    float deltaAngle = angularSpeed * deltaTime;

    // get the translation
    tf2::Vector3 translation;
    {
        tf2::Vector3 linearMovementVelocity = state.linearSpeed;

        if (std::abs(deltaAngle) > 0)
        {
            float curvatureRadius = linearMovementVelocity.x() / angularSpeed;
            translation = tf2::Vector3(curvatureRadius * std::sin(deltaAngle), curvatureRadius * (1 - std::cos(deltaAngle)), 0);
        }
        else
            translation = linearMovementVelocity * deltaTime;
    }

    tf2::Quaternion rotation({0, 0, 1}, deltaAngle);
    tf2::Transform movement(rotation, translation);

    state.currentTransform.mult(state.currentTransform, movement);
}
