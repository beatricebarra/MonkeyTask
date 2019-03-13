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

#ifndef SlowMovementApplication_H_
#define SlowMovementApplication_H_

// Including libraries
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
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include "stdlib.h"
#include <cstdlib>
#include "/usr/include/opencv/cv.h"
#include "/usr/include/stdio.h"
#include "/usr/include/fcntl.h"
#include "/usr/include/errno.h"
#include "/usr/include/string.h"
#include <termios.h>
#include <unistd.h>
#include <bitset>
#include <climits>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_audio.h>
#include <time.h>

// Define constants
#define KUKA_DOF 7
#define FINGER_DOF 0
#define IK_CONSTRAINTS 9
#define _dt (1.0/500.)

//Enums
enum ENUM_COMMAND{COMMAND_2Position, COMMAND_spring, COMMAND_Back, COMMAND_Home, COMMAND_Wait4Go, NONE_comand};
enum ENUM_PLANNER{PLANNER_CARTESIAN, PLANNER_JOINT, NONE_planner};
enum ENUM_AXIS{AXIS_X=0, AXIS_Y, AXIS_Z};

// Point Sequences
// Central point repeated
/*double tenxp0[10][KUKA_DOF] = {
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49}
};*/

// here position modified by Marion for accounting for high error threshold

double tenxp0[10][KUKA_DOF] = {
		{-1.25, 6.35, -1.92, -90, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -90 -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -90, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -90, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -90, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -90, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -90, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -90, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -90, -1.33, -18.42, +1.49},
		{-1.25, 6.35, -1.92, -90, -1.33, -18.42, +1.49}
};

// Return point repeated
double allBack[10][KUKA_DOF] = {
		{-1.26, -15.20, -2.21, -110.06, -1.36, -9.25, 1.75},
		{-1.26, -15.20, -2.21, -110.06, -1.36, -9.25, 1.75},
		{-1.26, -15.20, -2.21, -110.06, -1.36, -9.25, 1.75},
		{-1.26, -15.20, -2.21, -110.06, -1.36, -9.25, 1.75},
		{-1.26, -15.20, -2.21, -110.06, -1.36, -9.25, 1.75},
		{-1.26, -15.20, -2.21, -110.06, -1.36, -9.25, 1.75},
		{-1.26, -15.20, -2.21, -110.06, -1.36, -9.25, 1.75},
		{-1.26, -15.20, -2.21, -110.06, -1.36, -9.25, 1.75},
		{-1.26, -15.20, -2.21, -110.06, -1.36, -9.25, 1.75},
		{-1.26, -15.20, -2.21, -110.06, -1.36, -9.25, 1.75}
};

//Sequence 1 is : O, N, S, E, W, NE, SW, NW, SE, O
//CARDINAL POINTS LOOKING AT THE ROBOT

double P1[] = {-0.74, 4.66, -1.89, -91.96, 0.09, -3.66, 1.07};//N
double P2[] = {-4.17, 5.23, -1.89, -93.58, -30.37, -6.75, 31.19};//NW
double P3[] = {-5.93, 6.15, -1.89, -96.40, -28.49, -10.88, 29.00};//W
double P4[] = {-4.57, 7.04, -1.89, -99.70, -15.62, -14.26, -16.20};//SW
double P5[] = {-0.53, 9.53, -1.87, -104.93, 0.34, -21.62, 1.08};//S
double P6[] = {2.77, 6.76, -1.89, -100.27, 14.23, -14.44, -12.40};//SE
double P7[] = {4.42, 5.84, -1.89, -97.48, 26.79, 11.51, -24.87};//E
double P8[] = {3.19, 4.99, -1.89, -94.25, 32.30, -7.38, -30.71};//NE

// Looking from the robot back: !!! opposite of the points up !!!
double O[] = {-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49};
double N[] = {-0.72, 4.07, -1.92, -97.61, 0.59, -10.15, -0.42};
double S[] = {-1.29, 9.32, -1.92, -107.97, -1.03, -25.78, 1.25};
double W[] = {6.71, 6.44, -1.92, -103.35, 22.93, -19.66, -21.27};//
double E[] = {-6.98, 6.91, -1.92, -102.82, -19.01, -19.19, 18.11};//S
double NE[] = {-7.57, 4.37, -1.99, -97.0, -33.01, -12.47, +32.40};//S
double NW[] = {6.25, 3.73, -1.99, -98.49, 34.19, -12.64, -33.14};
double SE[] = {-8.93, 9.77, -1.99, -108.38, -18.15, -26.65, 16.00};//S
double SW[] = {4.86, 8.87, -1.99, -109.44, 13.08, -26.21, -11.50};



double Sequence1[10][KUKA_DOF] = {
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},//O
		{-0.72, 4.07, -1.92, -97.61, 0.59, -10.15, -0.42},//N
		{-1.29, 9.32, -1.92, -107.97, -1.03, -25.78, 1.25},//S
		{6.71, 6.44, -1.92, -103.35, 22.93, -19.66, -21.27},//E
		{-6.98, 6.91, -1.92, -102.82, -19.01, -19.19, 18.11},//W
		{-7.57, 4.37, -1.99, -97.0, -33.01, -12.47, +32.40},//NE
		{6.25, 3.73, -1.99, -98.49, 34.19, -12.64, -33.14},//NW
		{-8.93, 9.77, -1.99, -108.38, -18.15, -26.65, 16.00},//SE
		{4.86, 8.87, -1.99, -109.44, 13.08, -26.21, -11.50},//SW
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},//0
};
double FourPointSequence[6][KUKA_DOF] = {
		{-0.72, 4.07, -1.92, -97.61, 0.59, -10.15, -0.42},//N
		{6.71, 6.44, -1.92, -103.35, 22.93, -19.66, -21.27},//E
		{-6.98, 6.91, -1.92, -102.82, -19.01, -19.19, 18.11},//W
		{-1.29, 9.32, -1.92, -107.97, -1.03, -25.78, 1.25},//S
};

double FivePointSequence[6][KUKA_DOF] = {
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},//O
		{-0.72, 4.07, -1.92, -97.61, 0.59, -10.15, -0.42},//N
		{6.71, 6.44, -1.92, -103.35, 22.93, -19.66, -21.27},//E
		{-6.98, 6.91, -1.92, -102.82, -19.01, -19.19, 18.11},//W
		{-1.29, 9.32, -1.92, -107.97, -1.03, -25.78, 1.25},//S
};

double SixPointSequence[6][KUKA_DOF] = {
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},//O
		{-0.72, 4.07, -1.92, -97.61, 0.59, -10.15, -0.42},//N
		{6.71, 6.44, -1.92, -103.35, 22.93, -19.66, -21.27},//E
		{-6.98, 6.91, -1.92, -102.82, -19.01, -19.19, 18.11},//W
		{-7.57, 4.37, -1.99, -97.0, -33.01, -12.47, +32.40},//NE
		{6.25, 3.73, -1.99, -98.49, 34.19, -12.64, -33.14},//NW
};
double ThreePointSequence[3][KUKA_DOF] = {
		{-1.25, 6.35, -1.92, -103.59, -1.33, -18.42, +1.49},//O
		{6.71, 6.44, -1.92, -103.35, 22.93, -19.66, -21.27},//E
		{-6.98, 6.91, -1.92, -102.82, -19.01, -19.19, 18.11},//W
};

class SlowMovementApplication : public RobotInterface
{
public:
            SlowMovementApplication();
    virtual ~SlowMovementApplication();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);


private:

	void 						Send_Postion_To_Robot(Vector Position);
	void 						chatterCallback_position(const sensor_msgs::JointState & msg);
	ros::Publisher	 				pub_command_robot_real;
	ros::Subscriber 				sub_position_robot;
	sKinematics                 *mSKinematicChain;

	////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////  INPUT / OUTPUT  ///////////////////////////////
	////////////////////////////////////////////////////////////////////////////////

	// For USB communication
	int 						DACfile;
	int 						modidxPoint;
	int 						arduinoFD;
	ssize_t 					size;
	struct termios 				options;
	// For file writing
	ofstream 					myfile;
	string 						force_fileName;
	// For DAC writing
	double 						forceValue;
	double						maxDACScale;
	double 						maxForceValue;

	// Timing variables
	int 						tGoing2Home;
	int 						timeSet;
	// For point sequence
	int 						nP;
	int 						idxPoint;
	int 						badTrial;

	////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////  SIMULATOR /SOLVER /////////////////////////////
	////////////////////////////////////////////////////////////////////////////////

	RevoluteJointSensorGroup    mSensorsGroup;
	RevoluteJointActuatorGroup  mActuatorsGroup;
	KinematicChain              mKinematicChain;
	IKGroupSolver               mIKSolver;
	int                         mEndEffectorId;

	////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////// ROBOT CONTROL ////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////

	// My set of variables
	Matrix 						pointSequence;
	Matrix 						backSequence;
	Matrix 						homeSequence;

	// Target vectors
	Vector						BackPosition;
	Vector 						fCartTargetPos; // Final cartesian target position
	Vector 						ffCartTargetPos;
	Vector						fCartTargetDirY; //Final cartesian target y direction
	Vector						fCartTargetDirZ; //Final cartesian target z direction
	Vector						fJointTargetPos; //Final Joint target position
	Vector 						cCartTargetPos;	// Current cartesian target position
	Vector 						cCartTargetVel; // Current cartesian target velocity
	Vector 						cCartTargetAcc; // Current cartesian target acceleration
	Vector 						cJointTargetPos;// Current joint target position
	Vector 						cJointTargetVel;// Current joint target velocity
	Vector						home;
	Vector 						InitialDistance;
	Vector 						CurrentDistance;
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

	double						gain_ori;
	double						gain_ori1;
	double						gain_ori2;

	double						Stiffness1;
	double						Damping1;

	double						Stiffness2;
	double						Damping2;

	// Force variables
	Vector						cJointTORs;
	Vector						JointEffort_handle;
	Vector						eeForce;
	double 						eeForceMod;
	int		 					eeForceModInt;

	int							maxeeForceInt;
	int							mineeForceInt;
	double						maxeeForceDouble;
	double						mineeForceDouble;

	int 						maxeeDist2TargInt;
	int							mineeDist2TargInt;
	double						maxeeDist2TargDouble;
	double						mineeDist2TargDouble;

	double						eeDist2TargMod;
	int 						eeDist2TargInt;
	int							eeDist2TargIntShifted;
	int							eeDistForceCombined;

	char 						testvariable[2];
	char 						testread;
	int							num;


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
	Vector						J_distance2Home;
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

	double secs;
	double Constant_joint;


	// Velocity-precision
	double maxSat;
	double minSat;
	double minROM;
	double maxROM;

	// Parameters input from the user
	float pullThreshold;
	float timeout;
	float jointPosAccuracy;
	float cartPosAccuracy;
};



#endif 
