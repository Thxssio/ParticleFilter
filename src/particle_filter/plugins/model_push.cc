#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>

namespace gazebo {
class ModelPush : public ModelPlugin {
private:
  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rosSub;
  double xSpeed = 0, ySpeed = 0, thetaSpeed = 0;
  physics::ModelPtr model;
  rclcpp::Time lastUpdateTime;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    this->model = _parent;
    this->rosNode = rclcpp::Node::make_shared("gazebo_client");

    this->rosSub = this->rosNode->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&ModelPush::OnRosMsg, this, std::placeholders::_1));
    
    this->lastUpdateTime = this->rosNode->now();
  }

  void OnUpdate() {
    if ((this->rosNode->now() - this->lastUpdateTime).seconds() > 0.15) {
      this->xSpeed = 0.0;
      this->ySpeed = 0.0;
      this->thetaSpeed = 0.0;
    }

    float world_angle = this->model->WorldPose().Rot().Yaw();
    float x_setpoint = xSpeed * std::cos(world_angle) +
                       ySpeed * std::cos(world_angle + PI / 2.0);
    float y_setpoint = xSpeed * std::sin(world_angle) +
                       ySpeed * std::sin(world_angle + PI / 2.0);

    this->model->SetLinearVel(
        ignition::math::Vector3d(x_setpoint, y_setpoint, this->model->RelativeLinearVel().Z()));
    this->model->SetAngularVel(ignition::math::Vector3d(0, 0, thetaSpeed));
  }

  void OnRosMsg(const geometry_msgs::msg::Twist::SharedPtr _msg) {
    this->xSpeed = _msg->linear.x;
    this->ySpeed = _msg->linear.y;
    this->thetaSpeed = _msg->angular.z;
    this->lastUpdateTime = this->rosNode->now();
  }
};

GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // namespace gazebo
