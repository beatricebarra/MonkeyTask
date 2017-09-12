/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef monkeytask_test_1_H_
#define monkeytask_test_1_H_

#include "RobotLib/RobotInterface.h"

#include "RobotLib/RobotInterface.h"
#include "MathLib/MathLib.h"
#include "MathLib/IKGroupSolver.h"
#include "RobotLib/ForwardDynamics.h"
#include "RobotLib/InverseDynamics.h"
#include "RobotLib/KinematicChain.h"
#include "sKinematics.h"
#include "sensor_msgs/JointState.h"
#include "kuka_fri_bridge/JointStateImpedance.h"

#include "eigen3/Eigen/Dense"
#include "sg_filter.h"

#define KUKA_DOF 7
#define FINGER_DOF 0
#define IK_CONSTRAINTS 9
#define _dt (1.0/500.)
double cJob[]  = {0.0, -PI/4.0, 0.0, -PI/2.0, 0.0, -PI/4.0, 0.0};

double back[] = {0.0, -PI/4.0, 0.0, -PI/2.0, 0.0, -PI/4.0, 0.0};

// Target Points
double P1[] = {6.3, 12.38, 4.34, -69.36, 104.19, -11.11, -97.61};
double P2[] = {26.38, 13.28, 4.34, -74.36, 115.62, -27.38, -112.72};
double P3[] = {30.68, 13.28, 4.35, -79.08, 75.88, -29.50, -68.70};
double P4[] = {25.96, 32.57, 4.35, -104.19, 32.80, -43.93, -22.47};
double P5[] = {37.47, 49.99, -35.62, -105.54, -163.64, 59.56, -154.20};
double P6[] = {6.56, 28.91, -25.29, -112.69, 148.33, 48.44, -146.40};
double P7[] = {-54.95, 25.93, 45.84, -91.25, 108.02, 14.81, -127.92};
double P8[] = {-54.66, 7.94, 45.85, -82.96, 46.75, 19.07, -51.08};
double C[] = {0, 3.65, 0 -106.29, 0 -24.88, 4.6};



enum ENUM_COMMAND{COMMAND_2Position, COMMAND_spring, COMMAND_Back, COMMAND_Home, NONE_comand};
enum ENUM_PLANNER{PLANNER_CARTESIAN, PLANNER_JOINT, NONE_planner};
enum ENUM_AXIS{AXIS_X=0, AXIS_Y, AXIS_Z};

//Target points

double p0[]  = {-0.3, 0, 0.5};
double p1[]  = {-0.3, 0, 0.75};//north
double p2[]  = {-0.3, -0.3, 0.5};//east
double p3[]  = {-0.3, 0, 0.25};//south
double p4[]  = {-0.3, 0.3, 0.5};//west
// They have all the same direction of the end effector
double pYdir[] = {0.0, 1.0, 0.0};
double pZdir[] = {-1.0, 0.0, 0.0};

class monkeytask_test_1 : public RobotInterface
{
public:
	monkeytask_test_1();
	virtual ~monkeytask_test_1();

	virtual Status              RobotInit();
	virtual Status              RobotFree();

	virtual Status              RobotStart();
	virtual Status              RobotStop();

	virtual Status              RobotUpdate();
	virtual Status              RobotUpdateCore();

	virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);

private:

	void 						Send_Postion_To_Robot(Vector Position);

	ros::Publisher	 				pub_command_robot_real;
	ros::Subscriber 				sub_position_robot;


	void 						chatterCallback_position(const sensor_msgs::JointState & msg);
	sKinematics                 *mSKinematicChain;

	RevoluteJointSensorGroup    mSensorsGroup;
	RevoluteJointActuatorGroup  mActuatorsGroup;
	KinematicChain              mKinematicChain;

	IKGroupSolver               mIKSolver;

	int                         mEndEffectorId;

//	Vector                      mJointPos;
//	Vector                      mJointPosAll;
//	Vector                      mJointDesPos;
//	Vector                      mJointTargetPos;
//	Vector						mJointDesVel;


	// My set of variables
	// Target vectors
	Vector						BackPosition;
	Vector 						fCartTargetPos; // Final cartesian target position

	Vector						fCartTargetDirY; //Final cartesian target y direction
	Vector						fCartTargetDirZ; //Final cartesian target z direction
	Vector						fJointTargetPos; //Final Joint target position

	Vector 						cCartTargetPos;	// Current cartesian target position
	Vector 						cCartTargetVel; // Current cartesian target velocity
	Vector 						cCartTargetAcc; // Current cartesian target acceleration
	Vector 						cJointTargetPos;// Current joint target position
	Vector 						cJointTargetVel;// Current joint target velocity

	// Position and velocity vectors
	Vector 						cCartPos;	// Current cartesian position
	Vector 						cCartVel; 	// Current cartesian velocity
	Vector 						pCartPos;	// Previous cartesian position

	Vector						JointPos_handle;
	Vector 						cJointPos;	// Current joint position
	Vector 						cJointVel;	// Current joint velocity


	//Direction vectors

	Vector 						cCartDirX; 	// Current cartesian y direction
	Vector 						cCartDirY; 	// Current cartesian y direction
	Vector 						cCartDirZ; 	// Current cartesian z direction
	Vector						cCartTargetDirY; // Target cartesian y direction
	Vector						cCartTargetDirZ; // Target cartesian z direction

	Vector 						pCartDirY; 	// Previous cartesian y direction
	Vector 						pCartDirZ; 	// Previous cartesian z direction


	double						Stiffness;
	double						Damping;




	// Variables for filter usage
	Vector									copyVector;
	int 									retCode; // ?????
	int 									sgFilterOrder;
	int 									sgFilterWindowL;
	int 									inputDataDim;// Dimension of the vector that you feed as input data (in my case a position)
	SGF::Vec 								inputData; // Vector that you feed as input data (in my case a position).
	SGF::Vec 								outputData; // Vector that you feed to save output data (in my case a position).
	SGF::SavitzkyGolayFilter				*Beafilter;

	// Support variables
	Vector 						J_distance2P0;
	Vector 						J_distance2Back;
	Vector 						jP0;
	Vector 						jBack;
	// end of my set of variables


	Vector                      mJointVelLimitsUp;
	Vector                      mJointVelLimitsDn;
	Vector       				mJointVelocityLimitsDT;
	Vector 						mTargetVelocity; // for inverse kinematics input
	Vector 						lJointWeight;
	ENUM_COMMAND 				mCommand;
	ENUM_PLANNER 				mPlanner;
	Vector 						lPos;
	Vector3 					lDirX, lDirY, lDirZ;
	Vector 						lJoints;
	Matrix 						lJacobianDirY;
	Matrix 						lJacobianDirZ;
	// Jacobian Variables
	Matrix 						mJacobian3;
	Matrix 						mJacobian6;
	Matrix 						mJacobian9;
	Vector 						mJobJoints;
	Vector						mreleaseJoints;
	Vector 						mJointWeights;
	Vector					    mreleaseJoints2;
	Vector 						lTargetPos;
	Vector						lTDirection;
	Vector3 					lTargetDirX;
	Vector3						lTargetDirY;
	Vector3 					lTargetDirZ;
};



#endif 
