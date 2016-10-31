//
// Created by lukasz on 14.10.16.
//

#include "roboclaw_driver_node.h"

int port_num = 16; // /dev/ttyUSB0
int roboclaw_address = 0x80;
long encoder1, encoder2;
Roboclaw *roboclaw;
ros::Rate *freq;
int update_frequency = 50;
ros::Publisher odometry_publisher;
ros::Subscriber velocity_subscriber;
float robot_width = 0.3; // meters
float wheel_radius = 0.05; // meters
u_int16_t ticks_per_rev = 33000;
double rad_per_tick;
double wheel_dst = 0;
double rev_per_meter = 0;
uint16_t pulses_per_meter = 0;
float left_spd;
float right_spd;
double w1_angle[] = {0, 0}, w2_angle[] = {0, 0}; // [0] - current, [1] - previous
double w1_spd = 0, w2_spd = 0; // angular velocity
double robot_angle[] = {0, 0}; // [0] - current, [1] - previous
double robot_ang_vel = 0; // angular velocity
double robot_pos_x = 0, robot_pos_y = 0;
double robot_vel_x[] = {0, 0}, robot_vel_y[] = {0, 0};

void velocityCallback(const geometry_msgs::TwistConstPtr &msg) {
   left_spd = msg->linear.x - (msg->angular.z * 0.5 * robot_width);
   right_spd = msg->linear.x + (msg->angular.z * 0.5 * robot_width);
   roboclaw->set_speed(left_spd, right_spd);
}

int main(int argc, char **argv) {
   //std::cout << "Begin, init" << std::endl;
   ros::init(argc, argv, "roboclaw_driver");

   //std::cout << "Get node handle" << std::endl;
   ros::NodeHandle n("roboclaw_driver");

   //std::cout << "Calc rad_per_tick" << std::endl;
   rad_per_tick = 2 * PI / (double) ticks_per_rev;
   wheel_dst = 2 * PI * wheel_radius;
   rev_per_meter = 1.0 / wheel_dst;
   pulses_per_meter = (uint16_t) rev_per_meter * ticks_per_rev;

   roboclaw = new Roboclaw(roboclaw_address, port_num, pulses_per_meter);

   if (roboclaw->has_acces_to_ComPort()){
      std::cout << "Roboclaw instance obtained access to ComPort" << std::endl;
   } else {
      std::cout << "Can not get access to ComPort, closing" << std::endl;
      return 0;
   }

   //std::cout << "odom publisher" << std::endl;
   odometry_publisher = n.advertise<nav_msgs::Odometry>("odom", 1);

   //std::cout << "velocity subscriber" << std::endl;
   velocity_subscriber = n.subscribe("/cmd_vel", 1, velocityCallback);

   //std::cout << "Frequency" << std::endl;
   freq = new ros::Rate(update_frequency);
   std::cout << "Roboclaw started" << std::endl;

   while (ros::ok()) {
      roboclaw->read_encoders(&encoder1, &encoder2);
      //std::cout << "Enc1: " << encoder1 << " Enc2: " << encoder2;

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

      //std::cout << std::endl;
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
      odometry.child_frame_id = "roboclaw";
      odometry_publisher.publish(odometry);

      ros::spinOnce();
      freq->sleep();
   }
   return 0;
}


