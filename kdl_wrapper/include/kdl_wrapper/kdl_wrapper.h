
#ifndef KDL_WRAPPER_H_
#define KDL_WRAPPER_H_

#include <ros/ros.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
//#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include "kdl_acc_solver/chainfksolveracc_recursive.hpp"
//#include <kdl/chainfksolveracc_recursive.hpp>

class KDLWrapper{
public:

    ros::NodeHandle n;
    KDL::ChainIkSolverVel_wdls *ik_solver_vel;
    KDL::ChainJntToJacSolver *jnt_jac_solver;
    KDL::ChainFkSolverPos *fk_solver_pos;
    KDL::ChainFkSolverVel *fk_solver_vel;
    KDL::ChainFkSolverAcc *fk_solver_acc;

    KDL::Jacobian jacobian;


    KDLWrapper();
    virtual ~KDLWrapper();


    // initialize the KDL chain and the kinematic solvers
    // chain_root: ID of the root link of the kinematic chain
    // chain_tip: ID of the tip link of the kinematic chain
    // returns true if initialized correctly
    bool init(const std::string &chain_root, const std::string &chain_tip);

    // returns true if the object was initialized correctly
    bool isInitialized();

    // returns the KDL chain
    KDL::Chain getKDLChain();

private:

    bool m_initialized;
    KDL::Chain m_chain;

    bool getTreeFromURDF(KDL::Tree &tree);


};

#endif /* KDL_WRAPPER_H_ */
