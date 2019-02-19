#include <ros/ros.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/WrenchStamped.h"
#include <sstream>
#include <vector>
#include <string>

#include "yumi_control_server/GameCmd.h"

using namespace std;

KDL::JntArray right_arm_joint_positions;
KDL::JntArray left_arm_joint_positions;
KDL::Frame right_tool_tip_frame, left_tool_tip_frame;
KDLWrapper *right_arm_kdl_wrapper = NULL;
KDLWrapper *left_arm_kdl_wrapper = NULL;
vector<ros::Publisher> r_velocity_command_pub(7);
vector<ros::Publisher> l_velocity_command_pub(7);

float force_state[8] = {0,0,0,0,0,0,0,0};
float state[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

bool moving = false;
bool in_reset = false;
int urdf_order[7] = {1,2,7,3,4,5,6};

float z_limit_up = 0.3;
float z_limit_down = 0.2;
float z_limit_diff = 0.1;

bool l_sensor_in = false;
bool r_sensor_in = false;
bool ipad_in = false;

void ipad_callback(const std_msgs::Float64MultiArray& msg)
{
  if (ipad_in == false)
  {
    cout << "ipad connected!" << endl;
    ipad_in = true;
  }
  state[10] = msg.data[0];
  state[11] = msg.data[1];
  state[12] = msg.data[2];
  state[13] = msg.data[3];
}

void l_sensor_callback(const geometry_msgs::WrenchStamped& msg)
{
  if (l_sensor_in == false)
  {
    cout << "left sensor connected!" << endl;
    l_sensor_in = true;
  }  
  force_state[0] = msg.wrench.force.x;
  force_state[1] = msg.wrench.force.y;
  force_state[2] = msg.wrench.torque.x;
  force_state[3] = msg.wrench.torque.y;

  state[2] = msg.wrench.force.x;
  state[3] = msg.wrench.force.y;
  state[4] = msg.wrench.torque.x;
  state[5] = msg.wrench.torque.y;  
	// ROS_INFO("%u Fx:%.2f Fy:%.2f Fz:%.2f Tx:%.2f Ty:%.2f Tz:%.2f T:%.2f ms S: %.2f Hz\r\n", msg.header.seq, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z, durationTime, frequency);
}
void r_sensor_callback(const geometry_msgs::WrenchStamped& msg)
{
  if (r_sensor_in == false)
  {
    cout << "right sensor connected!" << endl;
    r_sensor_in = true;
  }  
  force_state[4] = msg.wrench.force.x;
  force_state[5] = msg.wrench.force.y;
  force_state[6] = msg.wrench.torque.x;
  force_state[7] = msg.wrench.torque.y;

  state[6] = msg.wrench.force.x;
  state[7] = msg.wrench.force.y;
  state[8] = msg.wrench.torque.x;
  state[9] = msg.wrench.torque.y; 
	// ROS_INFO("%u Fx:%.2f Fy:%.2f Fz:%.2f Tx:%.2f Ty:%.2f Tz:%.2f T:%.2f ms S: %.2f Hz\r\n", msg.header.seq, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z, durationTime, frequency);
}
void joint_state_callback(const sensor_msgs::JointState & msg)
{
    // cout << "in callback" << endl;
    int right_arm_indecis[7] = {1,3,13,5,7,9,11};
    int left_arm_indecis[7] = {0,2,12,4,6,8,10};
    for(int i = 0;i < 7; i++)
    {
        right_arm_joint_positions(i) = msg.position[right_arm_indecis[i]];
        left_arm_joint_positions(i) = msg.position[left_arm_indecis[i]];
    }

    right_arm_kdl_wrapper->fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
    left_arm_kdl_wrapper->fk_solver_pos->JntToCart(left_arm_joint_positions, left_tool_tip_frame, -1);

    state[0] = left_tool_tip_frame.p(2);
    state[1] = right_tool_tip_frame.p(2);

    // cout << right_tool_tip_frame.p(0) << "  " << right_tool_tip_frame.p(1) << "  " << right_tool_tip_frame.p(2) << endl;
    // cout << left_tool_tip_frame.p(0) << "  " << left_tool_tip_frame.p(1) << "  " << left_tool_tip_frame.p(2) << endl << endl;

    double diff_z = abs(right_tool_tip_frame.p(2)-left_tool_tip_frame.p(2));

    if (diff_z>z_limit_diff || right_tool_tip_frame.p(2) > z_limit_up || right_tool_tip_frame.p(2) < z_limit_down || left_tool_tip_frame.p(2) > z_limit_up || left_tool_tip_frame.p(2) < z_limit_down)
    {
      if (moving == true && in_reset == false)
      {
        cout << "pose limit, set motion to 0" << endl;
        std_msgs::Float64 cmd;
        cmd.data = 0;
        for(int i = 0; i < 7; i++)
        {
          r_velocity_command_pub[i].publish(cmd);
          l_velocity_command_pub[i].publish(cmd);
        }
        moving = false;
      } 
    }
}


void step(KDL::Vector l_target_v, KDL::Vector r_target_v){
  moving = true;
  std_msgs::Float64 cmd;

  double l_z_pred = left_tool_tip_frame.p(2) + l_target_v[2]*0.1;
  double r_z_pred = right_tool_tip_frame.p(2) + r_target_v[2]*0.1;
  double z_diff_pred = abs(l_z_pred-r_z_pred);

  if (l_z_pred > z_limit_up || l_z_pred < z_limit_down)
    l_target_v[2] = 0;
  if (r_z_pred > z_limit_up || r_z_pred < z_limit_down)
    r_target_v[2] = 0;  
  if (z_diff_pred > z_limit_diff)
  {
    l_target_v[2] = 0;
    r_target_v[2] = 0; 
  }
  

  right_arm_kdl_wrapper->ik_solver_vel->setLambda(0.3);
  left_arm_kdl_wrapper->ik_solver_vel->setLambda(0.3);

  KDL::JntArray right_arm_joint_velocity(7);
  KDL::JntArray left_arm_joint_velocity(7);
  KDL::Frame right_tool_tip_frame, left_tool_tip_frame;


  KDL::Twist right_arm_cart_velocity;
  right_arm_cart_velocity.vel = r_target_v; //KDL::Vector(0.0, 0.0, 0.05);
  // right_arm_cart_velocity.vel = KDL::Vector(0.0, 0.0, 0.05);
  // right_arm_cart_velocity.rot = KDL::Vector(0.0, 0.0, 0.0);

  KDL::Twist left_arm_cart_velocity;
  left_arm_cart_velocity.vel = l_target_v; //KDL::Vector(0.0, 0.0, -0.05);
  // left_arm_cart_velocity.rot = KDL::Vector(0.0, 0.0, 0.0);

  right_arm_kdl_wrapper->fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
  left_arm_kdl_wrapper->fk_solver_pos->JntToCart(left_arm_joint_positions, left_tool_tip_frame, -1);
  right_arm_kdl_wrapper->ik_solver_vel->CartToJnt(right_arm_joint_positions, right_arm_cart_velocity, right_arm_joint_velocity);
  left_arm_kdl_wrapper->ik_solver_vel->CartToJnt(left_arm_joint_positions, left_arm_cart_velocity, left_arm_joint_velocity);

  for(int i = 0; i < 7; i++)
  {
    cmd.data = right_arm_joint_velocity(i);
    r_velocity_command_pub[i].publish(cmd);
    cmd.data = left_arm_joint_velocity(i);
    l_velocity_command_pub[i].publish(cmd);
  }
  usleep(100000);
}

void reset(){
  // double right_target_position[7] = {1.39,-1.75,-1.42,0.648,1.92,0.808,-2.94};
  double right_target_pose[3] = {0.5, -0.25, 0.25};
  double left_target_pose[3] = {0.5, 0.25, 0.25};
  in_reset = true;
  int count = 0;

  double r_vel_cmd[3] = {1, 1, 1};
  double l_vel_cmd[3] = {1, 1, 1};

  double r_vel_cmd_send[3] = {1, 1, 1};
  double l_vel_cmd_send[3] = {1, 1, 1};

  float d_threshold = 0.03;
  while (abs(r_vel_cmd[0])>d_threshold || abs(r_vel_cmd[1])>d_threshold || abs(r_vel_cmd[2])>d_threshold || abs(l_vel_cmd[0])>d_threshold || abs(l_vel_cmd[1])>d_threshold || abs(l_vel_cmd[2])>d_threshold)
  {
    count = count +1;
    for(int i = 0; i < 3; i++)
    {
      r_vel_cmd[i] = right_target_pose[i] - right_tool_tip_frame.p(i);
      l_vel_cmd[i] = left_target_pose[i] - left_tool_tip_frame.p(i);

      r_vel_cmd_send[i] = r_vel_cmd[i];
      l_vel_cmd_send[i] = l_vel_cmd[i];

      float v_max = 0.03;
      if (r_vel_cmd[i] > 0.03)
        r_vel_cmd_send[i] = 0.03;
      if (r_vel_cmd[i] < -0.03)
        r_vel_cmd_send[i] = -0.03;
      if (l_vel_cmd[i] > 0.03)
        l_vel_cmd_send[i] = 0.03;
      if (l_vel_cmd[i] < -0.03)
        l_vel_cmd_send[i] = -0.03;                      
    }

    KDL::Vector left_target_v = KDL::Vector(l_vel_cmd_send[0], l_vel_cmd_send[1], l_vel_cmd_send[2]);
    KDL::Vector right_target_v = KDL::Vector(r_vel_cmd_send[0], r_vel_cmd_send[1], r_vel_cmd_send[2]);

    step(left_target_v, right_target_v);
    for(int i = 0; i < 3; i++)
    {
      cout << r_vel_cmd[i] << " ";
    }
    cout << endl;
    for(int i = 0; i < 3; i++)
    {
      cout << l_vel_cmd[i] << " ";
    }
    cout << endl << endl;

    usleep(100000);
    ros::spinOnce();
    // cout << count << endl;

    // if (count > 10)
    //   break;
  }
  std_msgs::Float64 cmd;
  cmd.data = 0;
  for(int i = 0; i < 7; i++)
  {
    r_velocity_command_pub[i].publish(cmd);
    l_velocity_command_pub[i].publish(cmd);
  }

  in_reset = false;
  cout << "reset done!" << endl;
}


bool cmd_service(yumi_control_server::GameCmd::Request  &req,
         yumi_control_server::GameCmd::Response &res)
{
  cout << "service called" << endl;
  if (req.cmd_type == 1)
  {
    reset();
  }  
  if (req.cmd_type == 2)
  {
    cout << req.cmd_data[0] << " " << req.cmd_data[1] <<endl;
    KDL::Vector l_target_v = KDL::Vector(0.0, 0.0, req.cmd_data[0]);
    KDL::Vector r_target_v = KDL::Vector(0.0, 0.0, req.cmd_data[1]);

    step(l_target_v, r_target_v);
  }
  ros::spinOnce();
  for (int i =0; i<14; i++)
  {
    res.obs.push_back(state[i]);
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kdl_controller_server"); // init the ROS node
  cout << "after init" << endl;

  ros::NodeHandle joint_node;
  ros::Subscriber joint_subscriber = joint_node.subscribe("/yumi/joint_states", 1, joint_state_callback);
	ros::Subscriber sub_left = joint_node.subscribe("/left_sensor/ethdaq_data", 1, l_sensor_callback);
  ros::Subscriber sub_right = joint_node.subscribe("/right_sensor/ethdaq_data", 1, r_sensor_callback);
  ros::Subscriber ball_state = joint_node.subscribe("/circlerun_ball_state", 1, ipad_callback);

  right_arm_joint_positions.resize(7);
  left_arm_joint_positions.resize(7);
  string command_topic;
  vector<ros::NodeHandle> r_velocity_command_node(7);
  vector<ros::NodeHandle> l_velocity_command_node(7);
  for(int i = 0; i < 7; i++)
  {
    command_topic = "yumi/joint_vel_controller_" + std::to_string(urdf_order[i]) + "_r/command";
    r_velocity_command_pub[i] = r_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
    command_topic = "yumi/joint_vel_controller_" + std::to_string(urdf_order[i]) + "_l/command";
    l_velocity_command_pub[i] = l_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
  }


  ros::ServiceServer service = joint_node.advertiseService("cmd_service", cmd_service);

  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  KDLWrapper r_wrapper;
  KDLWrapper l_wrapper;

  right_arm_kdl_wrapper = &r_wrapper;
  left_arm_kdl_wrapper = &l_wrapper;

  if(!right_arm_kdl_wrapper->init("yumi_body", "yumi_link_7_r"))
      ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");
  if(!left_arm_kdl_wrapper->init("yumi_body", "yumi_link_7_l"))
      ROS_ERROR("Error initiliazing left_arm_kdl_wrapper");

  std_msgs::Float64 cmd;
  cmd.data = 0;
  for(int i = 0; i < 7; i++)
  {
    r_velocity_command_pub[i].publish(cmd);
    l_velocity_command_pub[i].publish(cmd);
  }

  // sleep(2);
  // srand( time(NULL) );
  // std::cout << std::setprecision(3);

  KDL::Vector target_v = KDL::Vector(0.05, 0.05, 0.0);

  ros::spin();

  //reset(r_velocity_command_pub);
  // step(r_velocity_command_pub, l_velocity_command_pub, target_v);

  // std::cout << "I am done!" << std::endl;
  return 0;
}
