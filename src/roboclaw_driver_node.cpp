//
// Created by lukasz on 14.10.16.
//

#include "roboclaw_driver_node.h"

u_char roboclaw_address = 0x80;
long encoder1, encoder2;
Roboclaw *roboclaw;
ros::Rate *freq;
int update_frequency = 50;
ros::Publisher odometry_publisher;
ros::Subscriber velocity_subscriber;
float robot_width = 0.3; // meters
float wheel_radius = 0.05; // meters
int32_t ticks_per_rev = 0; // 33000 - antrobot
double rad_per_tick;
double wheel_dst = 0;
double rev_per_meter = 0;
uint32_t pulses_per_meter = 0;
double left_spd; // desired left wheel speed [m/s]
double right_spd; // desired right wheel speed [m/s]
double accel_limit; // desired wheel acceleration [m/s^2]
double w1_angle[] = {0, 0}, w2_angle[] = {0, 0}; // [0] - current, [1] - previous
double w1_spd = 0, w2_spd = 0; // angular velocity
double robot_angle[] = {0, 0}; // [0] - current, [1] - previous
double robot_ang_vel = 0; // angular velocity
double robot_pos_x = 0, robot_pos_y = 0;
double robot_vel_x[] = {0, 0}, robot_vel_y[] = {0, 0};
std::string base_frame_name;
std::string port_name;

void velocityCallback(const geometry_msgs::TwistConstPtr &msg) {
   double lin = msg->linear.x;
   double ang = msg->angular.z;
   left_spd = lin - ang * 0.5 * robot_width;
   right_spd = lin + ang * 0.5 * robot_width;

   //left_spd = (0.1 * lin - (0.05 * ang + ang * 0.1 * abs(lin)));
   //right_spd = (0.1 * lin + (0.05 * ang + ang * 0.1 * abs(lin)));
}

int main(int argc, char **argv) {
   //std::cout << "Begin, init" << std::endl;
   ros::init(argc, argv, "roboclaw_driver");

   //std::cout << "Get node handle" << std::endl;
   ros::NodeHandle n("~");

   n.param<std::string>("base_frame", base_frame_name, "roboclaw");

   /*if (n.getParam("base_frame", base_frame_name)) {
      std::cout << "Base frame: " << base_frame_name << std::endl;
   } else {
      std::cout << "Use default base frame: " << base_frame_name << std::endl;
   }*/

   n.param<int32_t>("encoder_res", ticks_per_rev, 132000);

   /*if (n.getParam("encoder_res", ticks_per_rev)) {
      std::cout << "Base frame: " << ticks_per_rev << std::endl;
   } else {
      std::cout << "Use default base frame: " << ticks_per_rev << std::endl;
   }*/

   n.param<double>("acceleration_limit", accel_limit, 0.1);

   n.param<std::string>("port", port_name, "/dev/ttyUSB0");

   static tf::TransformBroadcaster br;
   tf::StampedTransform transform;
   tf::Quaternion quaternion;

   transform.child_frame_id_ = base_frame_name;
   transform.frame_id_ = "world";

   //std::cout << "Calc rad_per_tick" << std::endl;
   rad_per_tick = 2 * PI / (double) ticks_per_rev;
   wheel_dst = 2 * PI * wheel_radius;
   rev_per_meter = 1.0 / wheel_dst;
   pulses_per_meter = (uint32_t) rev_per_meter * ticks_per_rev;

   roboclaw = new Roboclaw(roboclaw_address, port_name, pulses_per_meter, 1000);

   //std::cout << "odom publisher" << std::endl;
   odometry_publisher = n.advertise<nav_msgs::Odometry>("odom", 1);

   //std::cout << "velocity subscriber" << std::endl;
   velocity_subscriber = n.subscribe("/cmd_vel", 1, velocityCallback);

   //std::cout << "Frequency" << std::endl;
   freq = new ros::Rate(update_frequency);
   std::cout << "Roboclaw started" << std::endl;

   while (ros::ok()) {
      //if (roboclaw->set_speed(left_spd, right_spd)) {
      if (roboclaw->set_speed_with_accel(left_spd, right_spd, accel_limit)) {
         std::cout << "Set spd L: " << left_spd << " Set spd R: " << right_spd;
      } else {
         std::cout << "Set speed communication ERROR ";
      }

      if (roboclaw->read_encoders(&encoder1, &encoder2)) {
         std::cout << "Enc1: " << encoder1 << " Enc2: " << encoder2;
      } else {
         std::cout << "read_encoders communication ERROR ";
      }
      // update wheel angles based on encoder reads
      w1_angle[1] = w1_angle[0];
      w2_angle[1] = w2_angle[0];
      w2_angle[0] = encoder1 * rad_per_tick;
      w1_angle[0] = encoder2 * rad_per_tick;
      //std::cout << " W1_a: " << w1_angle[0] << " W2_a: " << w2_angle[0];

      // update robot orientation based on wheel angles
      robot_angle[1] = robot_angle[0];
      robot_angle[0] = (w1_angle[0] - w2_angle[0]) * wheel_radius / (robot_width);
      //std::cout << " robot_angle: " << robot_angle[0];
      w2_spd = (w2_angle[0] - w2_angle[1]) * update_frequency;
      robot_ang_vel = (robot_angle[0] - robot_angle[1]) * update_frequency;
      robot_vel_x[1] = robot_vel_x[0];
      robot_vel_y[1] = robot_vel_y[0];
      robot_vel_x[0] = ((w2_spd * wheel_radius) + (robot_ang_vel * robot_width * 0.5)) * cos
              (robot_angle[0]);
      robot_vel_y[0] = ((w2_spd * wheel_radius) + (robot_ang_vel * robot_width * 0.5)) * sin
              (robot_angle[0]);
      robot_pos_x = robot_pos_x + (0.5 * (robot_vel_x[0] + robot_vel_x[1]) / update_frequency);
      robot_pos_y = robot_pos_y + (0.5 * (robot_vel_y[0] + robot_vel_y[1]) / update_frequency);

      std::cout << std::endl;
      nav_msgs::Odometry odometry;
      tf::Vector3 vector(0, 0, 1);
      tf::Quaternion quat(vector, robot_angle[0]);

      odometry.pose.pose.position.x = robot_pos_x;
      odometry.pose.pose.position.y = robot_pos_y;
      odometry.pose.pose.position.z = 0;
      odometry.pose.pose.orientation.x = quat.x();
      odometry.pose.pose.orientation.y = quat.y();
      odometry.pose.pose.orientation.z = quat.z();
      odometry.pose.pose.orientation.w = quat.w();
      odometry.header.frame_id = "world";
      odometry.child_frame_id = base_frame_name;
      odometry_publisher.publish(odometry);

      transform.setOrigin(tf::Vector3(robot_pos_x, robot_pos_y, 0.0));
      transform.stamp_ = ros::Time::now();

      tf::Quaternion q;
      q.setRPY(0, 0, robot_angle[0]);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", base_frame_name));

      ros::spinOnce();
      freq->sleep();
   }
   return 0;
}


