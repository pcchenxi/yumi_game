#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <kdl_wrapper/kdl_wrapper.h>


int main(int argc, char *argv[])
{
    /// initialize ROS, specify name of node
    ros::init(argc, argv, "yumi_kdl_wrapper_example");


    KDLWrapper yumi_kdl_wrapper;

    if(!yumi_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
    {
        ROS_ERROR("Error initiliazing yumi_kdl_wrapper");
    }

    KDL::JntArray q_in(7);
    q_in(0) = 0.0;
    q_in(1) = M_PI/2;
    q_in(2) = 0.0;
    q_in(3) = M_PI/4;
    q_in(4) = 0.0;
    q_in(5) = 0.0;
    q_in(6) = M_PI/3;


    KDL::Twist v_in;
    v_in.vel = KDL::Vector(0.0, 0.2, 0.2);
    v_in.rot = KDL::Vector(0.0, 0.0, 0.0);

    KDL::JntArray q_dot_out;
    ROS_INFO("Calculating inverse kinematics");
    yumi_kdl_wrapper.ik_solver_vel->setLambda(0.3);
    yumi_kdl_wrapper.ik_solver_vel->CartToJnt(q_in, v_in, q_dot_out);

    ROS_INFO("Output q_dot: (%f, %f, %f, %f, %f, %f, %f)",
             q_dot_out(0),
             q_dot_out(1),
             q_dot_out(2),
             q_dot_out(3),
             q_dot_out(4),
             q_dot_out(5),
             q_dot_out(6));

    KDL::Jacobian jac(7);
    int res = 0;
    res = yumi_kdl_wrapper.jnt_jac_solver->JntToJac(q_in, jac, -1);
    std::cout << "result: " << res <<std::endl;
    std::cout<<"jacobian cols: " << jac.data.cols() << std::endl;
    std::cout<<"jacobian rows: " << jac.data.rows() << std::endl;
    std::cout<< "jacobian: " << jac.data << std::endl;

    return 0;
}
