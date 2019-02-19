#include <ros/ros.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <string>
using namespace std;


KDL::JntArray right_arm_joint_positions;
KDL::JntArray left_arm_joint_positions;

int urdf_order[7] = {1,2,7,3,4,5,6};

void joint_state_callback(const sensor_msgs::JointState & msg)
{
    int right_arm_indecis[7] = {1,3,13,5,7,9,11};
    int left_arm_indecis[7] = {0,2,12,4,6,8,10};
    for(int i = 0;i < 7; i++)
    {
        right_arm_joint_positions(i) = msg.position[right_arm_indecis[i]];
        left_arm_joint_positions(i) = msg.position[left_arm_indecis[i]];
    }
}

void reset(vector<ros::Publisher> r_velocity_command_pub){
  double right_target_position[7] = {1.39,-1.75,-1.42,0.648,1.92,0.808,-2.94};

  // for (int i = 0 ; i < 7; i++)
  //   cout << right_arm_joint_positions(i) << "\t";
  // cout << endl;
  std_msgs::Float64 cmd;

  for (int i = 6; i >= 0; i--)
  {
    while (abs(right_target_position[i] - right_arm_joint_positions(i))>0.05)
    {
      cmd.data = 0.3*(right_target_position[i] - right_arm_joint_positions(i));
      r_velocity_command_pub[i].publish(cmd);
      usleep(100000);
    }
    cmd.data = 0;
    r_velocity_command_pub[i].publish(cmd);
    cout << "reseting joint " << i << endl;
  }

}

void step(vector<ros::Publisher> r_velocity_command_pub, vector<ros::Publisher> l_velocity_command_pub, KDL::Vector target_v){
  std_msgs::Float64 cmd;
  KDLWrapper right_arm_kdl_wrapper;
  KDLWrapper left_arm_kdl_wrapper;

  if(!right_arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
      ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");
  if(!left_arm_kdl_wrapper.init("yumi_body", "yumi_link_7_l"))
      ROS_ERROR("Error initiliazing left_arm_kdl_wrapper");

  right_arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);
  left_arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);

  KDL::JntArray right_arm_joint_velocity(7);
  KDL::JntArray left_arm_joint_velocity(7);
  KDL::Frame right_tool_tip_frame, left_tool_tip_frame;


  KDL::Twist right_arm_cart_velocity;
  // right_arm_cart_velocity.vel = target_v; //KDL::Vector(0.0, 0.0, 0.05);
  right_arm_cart_velocity.vel = KDL::Vector(0.0, 0.0, 0.05);
  // right_arm_cart_velocity.rot = KDL::Vector(0.0, 0.0, 0.0);

  KDL::Twist left_arm_cart_velocity;
  left_arm_cart_velocity.vel = KDL::Vector(0.0, 0.0, -0.05);
  // left_arm_cart_velocity.rot = KDL::Vector(0.0, 0.0, 0.0);

  // timing
  double current_time, start_time, time_to_next_motor_cmd, time_to_next_cmd, time_to_next_print, run_duration, dt;
  run_duration = 30; // in sec
  start_time = ros::Time::now().toSec();
  time_to_next_motor_cmd = start_time;
  time_to_next_cmd = start_time;
  time_to_next_print = start_time;
  current_time = start_time;

  while (ros::ok()) {
    dt = ros::Time::now().toSec() - current_time;

    current_time += dt;
    if (current_time > (start_time + run_duration))
      break;

    if (current_time > time_to_next_cmd)
    {
      time_to_next_cmd += 1.0;
      //right_arm_cart_velocity.vel(0) = target_v[0]*-1;
      //right_arm_cart_velocity.vel(1) = target_v[1]*-1;
      //right_arm_cart_velocity.vel(2) = target_v[2]*-1;
      
      right_arm_cart_velocity.vel(2) *= -1.0;
      left_arm_cart_velocity.vel(2) *= -1.0;


      //std::cout << "Right Joints: ";
      //for(int i = 0; i < 7; i++)
      //  std::cout << right_arm_joint_positions(i) << "\t";
      //std::cout << std::endl;
      //std::cout << "Left Joints: ";
      //for(int i = 0; i < 7; i++)
      //  std::cout << left_arm_joint_positions(i) << "\t";
      //std::cout << std::endl;
    }

    if (current_time > time_to_next_motor_cmd)
    {
      time_to_next_motor_cmd += 0.01;

      right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
      left_arm_kdl_wrapper.fk_solver_pos->JntToCart(left_arm_joint_positions, left_tool_tip_frame, -1);
      right_arm_kdl_wrapper.ik_solver_vel->CartToJnt(right_arm_joint_positions, right_arm_cart_velocity, right_arm_joint_velocity);
      left_arm_kdl_wrapper.ik_solver_vel->CartToJnt(left_arm_joint_positions, left_arm_cart_velocity, left_arm_joint_velocity);

      for(int i = 0; i < 7; i++)
      {
        cmd.data = right_arm_joint_velocity(i);
        r_velocity_command_pub[i].publish(cmd);
        cmd.data = left_arm_joint_velocity(i);
        l_velocity_command_pub[i].publish(cmd);
      }
    }

    if(current_time > time_to_next_print)
    {

      cout << right_tool_tip_frame.p(0) << "\t\t" << right_tool_tip_frame.p(1) << "\t\t" << right_tool_tip_frame.p(2) << "\t\t\t\t";
      cout << left_tool_tip_frame.p(0) << "\t\t" << left_tool_tip_frame.p(1) << "\t\t" << left_tool_tip_frame.p(2) << endl;
      time_to_next_print += 0.2;
    }
  }
  cmd.data = 0;
  for(int i = 0; i < 7; i++)
  {
    r_velocity_command_pub[i].publish(cmd);
    l_velocity_command_pub[i].publish(cmd);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kdl_controller"); // init the ROS node
  right_arm_joint_positions.resize(7);
  left_arm_joint_positions.resize(7);
  ros::NodeHandle joint_node;
  ros::Subscriber joint_subscriber = joint_node.subscribe("/yumi/joint_states", 1, joint_state_callback);
  string command_topic;
  vector<ros::NodeHandle> r_velocity_command_node(7);
  vector<ros::Publisher> r_velocity_command_pub(7);
  vector<ros::NodeHandle> l_velocity_command_node(7);
  vector<ros::Publisher> l_velocity_command_pub(7);
  for(int i = 0; i < 7; i++)
  {
    command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_r/command";
    r_velocity_command_pub[i] = r_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
    command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_l/command";
    l_velocity_command_pub[i] = l_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(2);
  srand( time(NULL) );
  std::cout << std::setprecision(3);

  KDL::Vector target_v = KDL::Vector(0.05, 0.05, 0.0);

  //reset(r_velocity_command_pub);
  step(r_velocity_command_pub, l_velocity_command_pub, target_v);

  std::cout << "I am done!" << std::endl;
  return 0;
}
