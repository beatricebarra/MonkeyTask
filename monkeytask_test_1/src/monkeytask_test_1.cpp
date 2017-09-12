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

#include "monkeytask_test_1.h"


bool closed_loop=true;
bool True_robot=true;
bool Position_of_the_robot_recieved=false;

monkeytask_test_1::monkeytask_test_1()
:RobotInterface(){
}
monkeytask_test_1::~monkeytask_test_1(){
}

void monkeytask_test_1::chatterCallback_position(const sensor_msgs::JointState & msg)
{
	if (True_robot)
	{
		JointPos_handle(0)=msg.position[0];
		JointPos_handle(1)=msg.position[1];
		JointPos_handle(2)=msg.position[2];
		JointPos_handle(3)=msg.position[3];
		JointPos_handle(4)=msg.position[4];
		JointPos_handle(5)=msg.position[5];
		JointPos_handle(6)=msg.position[6];
		cJointPos=JointPos_handle;
		Position_of_the_robot_recieved=true;
	}
}

void monkeytask_test_1::Send_Postion_To_Robot(Vector Position)
{
	if (True_robot)
	{
		kuka_fri_bridge::JointStateImpedance msg;
		msg.position.resize(KUKA_DOF);
		msg.stiffness.resize(KUKA_DOF);
		for (int i=0; i<KUKA_DOF;i=i+1)
		{
			msg.position[i]  = Position(i);
			msg.stiffness[i] = 2000;
		}
		pub_command_robot_real.publish(msg);
	}
}

RobotInterface::Status monkeytask_test_1::RobotInit(){

	// Resizing the vectors for my number of joints
	BackPosition.Resize(KUKA_DOF);

	cJointTargetPos.Resize(KUKA_DOF);
	cJointTargetVel.Resize(KUKA_DOF);
	fJointTargetPos.Resize(KUKA_DOF);
	cJointPos.Resize(KUKA_DOF);
	JointPos_handle.Resize(KUKA_DOF);
	cJointVel.Resize(KUKA_DOF);

	// Resizing the cartesian positions and direction vectors
	fCartTargetPos.Resize(3); // Final cartesian target position
	cCartTargetPos.Resize(3);	// Current cartesian target position
	cCartPos.Resize(3);	// Current cartesian position
	pCartPos.Resize(3);	// Previous cartesian position

	cCartTargetDirY.Resize(3); //Final cartesian target y direction
	cCartTargetDirZ.Resize(3); //Final cartesian target z direction
	cCartDirX.Resize(3); 	// Current cartesian y direction
	cCartDirY.Resize(3); 	// Current cartesian y direction
	cCartDirZ.Resize(3); 	// Current cartesian z direction
	pCartDirY.Resize(3); 	// Previous cartesian y direction
	pCartDirZ.Resize(3); 	// Previous cartesian z direction



	cCartTargetVel.Resize(9); // Current cartesian target velocity
	cCartTargetAcc.Resize(9); // Current cartesian target acceleration
	cCartVel.Resize(9); 	// Current cartesian velocity

	copyVector.Resize(3);
	J_distance2P0.Resize(KUKA_DOF);
	J_distance2Back.Resize(KUKA_DOF);
	jP0.Resize(KUKA_DOF);
	jBack.Resize(KUKA_DOF);
	/*mJointPos.Resize(KUKA_DOF);
	mJointDesPos.Resize(KUKA_DOF);
	mJointDesVel.Resize(KUKA_DOF);
	mJointTargetPos.Resize(KUKA_DOF);*/

	lJointWeight.Resize(KUKA_DOF);
	lPos.Resize(3);
	lJoints.Resize(KUKA_DOF);
	lJacobianDirY.Resize(3,KUKA_DOF);
	lJacobianDirZ.Resize(3,KUKA_DOF);

	// coordinates and directions of the reference frame correponding to the target point
	lTargetPos.Resize(3);
	lTDirection.Resize(3);



	// for inverse kinematics
	mJointVelLimitsUp.Resize(KUKA_DOF);
	mJointVelLimitsDn.Resize(KUKA_DOF);
	mJointVelocityLimitsDT.Resize(KUKA_DOF);
	mTargetVelocity.Resize(IK_CONSTRAINTS);

	// initialize sensor group
	mSensorsGroup.SetSensorsList(mRobot->GetSensors());//mRobot I have it because of inheritance from class RobotInterface
	mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());
	mSensorsGroup.ReadSensors();

	// Kinematic Chain for real robot
	mEndEffectorId = mRobot->GetLinksCount()-1;
	mKinematicChain.SetRobot(mRobot);
	mKinematicChain.Create(0,0,mEndEffectorId);

	// kinematic chain for virtual robot
	mSKinematicChain = new sKinematics(KUKA_DOF, _dt);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed
	double Rgain=0.90;

	mSKinematicChain->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD( -170.)*Rgain, DEG2RAD( 170.)*Rgain, DEG2RAD(110.0)*Rgain);
	mSKinematicChain->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -120.)*Rgain, DEG2RAD( 120.)*Rgain, DEG2RAD(110.0)*Rgain);
	mSKinematicChain->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.)*Rgain, DEG2RAD(170.)*Rgain, DEG2RAD(128.0)*Rgain);
	mSKinematicChain->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.)*Rgain, DEG2RAD(120.)*Rgain, DEG2RAD(128.0)*Rgain);
	mSKinematicChain->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.)*Rgain, DEG2RAD(170.)*Rgain, DEG2RAD(204.0)*Rgain);
	mSKinematicChain->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -120.)*Rgain, DEG2RAD( 120.)*Rgain, DEG2RAD(184.0)*Rgain); // reduced joint angle to save the fingers
	mSKinematicChain->setDH(6, -0.05,   0.126+0.0,    0.0, 0.0, 1,  DEG2RAD(-170.)*Rgain, DEG2RAD(170.)*Rgain, DEG2RAD(184.0)*Rgain); // with Alegro Hand
	mSKinematicChain->readyForKinematics();




	// T0 is a transformation matrix from global basement to base coordinate of 0th links
	// T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
	double T0[4][4];
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			T0[i][j] = 0.0;

	T0[0][0] = 1;
	T0[1][1] = 1;
	T0[2][2] = 1;
	T0[3][3] = 1;
	mSKinematicChain->setT0(T0);


	MathLib::Matrix3 mTF;
	double TF[4][4];
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			TF[i][j] = 0.0;
	TF[3][3] = 1;
	mTF = Matrix3::SRotationY(0);

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			TF[i][j] = mTF(i,j);


	// ready for kinematics
	mSKinematicChain->readyForKinematics();

	mSKinematicChain->setTF(TF);

	// variable for ik
	mJacobian3.Resize(3,KUKA_DOF);
	mJacobian6.Resize(6,KUKA_DOF);
	mJacobian9.Resize(9,KUKA_DOF);

	// Inverse kinematics
	mIKSolver.SetSizes(KUKA_DOF);  // Dof counts
	mIKSolver.AddSolverItem(IK_CONSTRAINTS);
	mIKSolver.SetVerbose(false);                // No comments
	mIKSolver.SetThresholds(0.0001,0.00001);    // Singularities thresholds
	mIKSolver.Enable(true,0);                   // Enable first solver
	mIKSolver.SetDofsIndices(mKinematicChain.GetJointMapping(),0); // Joint maps for first solver

	lJointWeight(0) = 1.0;
	lJointWeight(1) = 1.0;
	lJointWeight(2) = 1.0;
	lJointWeight(3) = 1.0;
	lJointWeight(4) = 1.0;
	lJointWeight(5) = 1.0;
	lJointWeight(6) = 1.0;

	mIKSolver.SetDofsWeights(lJointWeight);

	Vector lMaxVel(KUKA_DOF);
	mSKinematicChain->getMaxVel(lMaxVel.Array());
	mJointVelocityLimitsDT = lMaxVel*_dt;

	//mJobJoints.Resize(KUKA_DOF);
	//mJobJoints.Set(cJob, KUKA_DOF);
	mJointWeights.Resize(KUKA_DOF);


	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %%%%%%%%%%%%%%%%%%%%%%%%% My Initialization %%%%%%%%%%%%%%%%%%%%%%%%%
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	//Directions
	cCartDirY(0) = 0;
	cCartDirY(1) = 1;
	cCartDirY(2) = 0;

	cCartDirZ(0) = 0;
	cCartDirZ(1) = 0;
	cCartDirZ(2) = 1;

	// Filter
	sgFilterOrder = 2;
	sgFilterWindowL = 10;
	inputDataDim = 3;
	outputData.resize(inputDataDim);
	inputData.resize(inputDataDim);
	Beafilter= new SGF::SavitzkyGolayFilter(inputDataDim, sgFilterOrder, sgFilterWindowL, _dt);

	AddConsoleCommand("p0");
	AddConsoleCommand("p1");
	AddConsoleCommand("p2");
	AddConsoleCommand("p3");
	AddConsoleCommand("p4");
	AddConsoleCommand("p5");
	AddConsoleCommand("p6");
	AddConsoleCommand("p7");
	AddConsoleCommand("p8");

	mPlanner =NONE_planner;
	mCommand = NONE_comand;


	// Dynamical system parameters
	Stiffness= -100;
	Damping = -20;

	// Points:

	// Pulling position
	jP0(0) = 0.0;
	jP0(1) = 0.0;
	jP0(2) = 0.0;
	jP0(3) = -PI/2.0;
	jP0(4) = 0.0;
	jP0(5) = 0.0;
	jP0(6) = 0.0;


	// Back Position
	jBack(0) = 0.0;
	jBack(1) =-PI/4.0;
	jBack(2) = 0.0;
	jBack(3) = -PI/2.0;
	jBack(4) = 0.0;
	jBack(5) = -PI/4.0;
	jBack(6) = 0.0;

	/*lTargetPos(0)=-0.3;
	lTargetPos(1)=0.3;
	lTargetPos(2)=0.75;
	lTargetDirY(0)=0;
	lTargetDirY(1)=0;
	lTargetDirY(2)=-1;
	lTargetDirZ(0)=-1;
	lTargetDirZ(1)=0;
	lTargetDirZ(2)=0;*/

	mRobot->SetControlMode(Robot::CTRLMODE_POSITION);
	ros::NodeHandle *n = mRobot->InitializeROS();
	if (True_robot)
	{
		pub_command_robot_real =  n->advertise<kuka_fri_bridge::JointStateImpedance>("/real_r_arm_controller/joint_imp_cmd", 3);
		sub_position_robot = n->subscribe("/real_r_arm_pos_controller/joint_states", 3, & monkeytask_test_1::chatterCallback_position,this);
	}

	return STATUS_OK;
}
RobotInterface::Status monkeytask_test_1::RobotFree(){
	return STATUS_OK;
}
RobotInterface::Status monkeytask_test_1::RobotStart(){

	// Setting the position of all joints to 0
	//mJointPosAll.Zero();
	cJointPos.Zero();
	if (True_robot)
	{
	while (Position_of_the_robot_recieved==false)
	{
		ros::spinOnce();
	}
	}
	else
	{
		mSensorsGroup.ReadSensors();
		cJointPos = mSensorsGroup.GetJointAngles();
	}

	/*For real robot experiment!*/
	/*	while (mJointPosAll.Norm()==0)
	{*/
	//mSensorsGroup.ReadSensors();
	//mJointPosAll = mSensorsGroup.GetJointAngles();
	//cJointPos = mSensorsGroup.GetJointAngles();
	//	}
	//Writing the joint position in the kinematic chain
	//mSKinematicChain->setJoints(mJointPosAll.Array());
	mSKinematicChain->setJoints(cJointPos.Array());
	// Output of the joint values in the simulator console
	//mJointPosAll.Print("mJointPosAll");
	cJointPos.Print("cJointPos");
	// Target position = current position
	//mJointTargetPos=mJointPosAll;
	cJointTargetPos = cJointPos;
	// Null command and planner are set
	mCommand=NONE_comand;
	mPlanner=NONE_planner;

	// Setting speed limits for the joints : I am setting variables of the monkey task class, from the kinemtic chain, is it useful?
	for(int i=0;i<KUKA_DOF;i++){
		mJointVelLimitsDn(i) = -mSKinematicChain->getMaxVel(i);
		mJointVelLimitsUp(i) =  mSKinematicChain->getMaxVel(i);
	}
	cout<<"End of Robot Start"<<endl;
	return STATUS_OK;
}    
RobotInterface::Status monkeytask_test_1::RobotStop(){
	return STATUS_OK;
}
RobotInterface::Status monkeytask_test_1::RobotUpdate(){

	ros::spinOnce();

	//Local variables
	float distance2target;
	float distance2target_x;
	// Setting the joints to values currently stored in mJointPosAll
	//mSKinematicChain->setJoints(mJointPosAll.Array());
	mSKinematicChain->setJoints(cJointPos.Array());
	//Saving target point position and relative axis direction in the correspondent attribute in the KinematicChain object
	mSKinematicChain->getEndPos(lPos.Array());
	//mSKinematicChain->getEndDirAxis(AXIS_X, lDirX.Array());
	//mSKinematicChain->getEndDirAxis(AXIS_Y, lDirY.Array());
	//mSKinematicChain->getEndDirAxis(AXIS_Z, lDirZ.Array());
	mSKinematicChain->getEndDirAxis(AXIS_X, cCartDirX.Array());
	mSKinematicChain->getEndDirAxis(AXIS_Y, cCartDirY.Array());
	mSKinematicChain->getEndDirAxis(AXIS_Z, cCartDirZ.Array());
//	cCartDirX.Print("cCartDirX");
//	cCartDirY.Print("cCartDirY");
//	cCartDirZ.Print("cCartDirZ");


	//Setting different planners for different commands
	switch(mCommand){
	case COMMAND_2Position :
		//fJointTargetPos[3] =  -PI/2.0;

		//fJointTargetPos.Set(P1, KUKA_DOF);
		//fJointTargetPos.Set(P2, KUKA_DOF);
		//fJointTargetPos.Set(P3, KUKA_DOF);
		//fJointTargetPos.Set(P4, KUKA_DOF);
		//fJointTargetPos.Set(P5, KUKA_DOF);
		//fJointTargetPos.Set(P6, KUKA_DOF);
		//fJointTargetPos.Set(P7, KUKA_DOF);
		//fJointTargetPos.Set(P8, KUKA_DOF);
		//fJointTargetPos.Set(C, KUKA_DOF);

		//fJointTargetPos.Mult((PI/180.0), fJointTargetPos);

		/*fJointTargetPos[0] =  0.0;
		fJointTargetPos[1] =  PI/4.0;
		fJointTargetPos[2] =  -0.0;
		fJointTargetPos[3] =  -PI/2.0;
		fJointTargetPos[4] =  0.0;
		fJointTargetPos[5] =  -PI/4.0;
		fJointTargetPos[6] =  0.0;*/

		mPlanner = PLANNER_JOINT;
		break;
	case COMMAND_spring :
		mPlanner = PLANNER_CARTESIAN;//Add termination condition in planner
		//mCommand=NONE_comand;
		break;
	case COMMAND_Back:
		mPlanner = PLANNER_JOINT;
		//cCartPos[1] = cCartPos[1]-10;//Go Back of 10 cm
		fJointTargetPos.Set(back, KUKA_DOF);
		//mCommand=COMMAND_Home;
		break;
	case COMMAND_Home:
		mPlanner = PLANNER_JOINT;
		fJointTargetPos.Zero();
		//mCommand=NONE_comand;
		break;
	}


	// Definition and implementation of type of movement
	switch(mPlanner){
	// PLANNER CARTESIAN implements the impedance control with a stifness and damping factor
	case PLANNER_CARTESIAN:


		// Time scale of the loop:
		// p --> t-1
		// c --> t
		// d --> t+1
		cout<<"IN PLANNER CARTESIAN"<<endl;
		fCartTargetPos.Print("fCartTargetPos");


		//Updating target joint position variable with the JointPosition variable
		//lJoints=mJointPosAll;

		//Dealing with Jacobian
		//NB: mJacobian3 is NOT the input, is the variable in which the computed Jacobian is written, it is a pointer
		mSKinematicChain->getJacobianPos(mJacobian3);
		mSKinematicChain->getJacobianDirection(AXIS_Y, lJacobianDirY);
		mSKinematicChain->getJacobianDirection(AXIS_Z, lJacobianDirZ);

		// Setting joint speed limits for the IK solver --> I suppose they will be checked in the function of the solver that sets the Jacobian, But I did not check
		mIKSolver.SetLimits(mJointVelLimitsDn,mJointVelLimitsUp);
		mJacobian3.Mult(1.0,mJacobian3);

		for(int i=0; i<3; i++){
			mJacobian9.SetRow(mJacobian3.GetRow(i)   , i  );
			mJacobian9.SetRow(lJacobianDirY.GetRow(i), i+3);
			mJacobian9.SetRow(lJacobianDirZ.GetRow(i), i+6);
		}

		// Computing istantaneous velocity by means of:
		// sg filter....
		mSKinematicChain->getEndPos(cCartPos.Array());
		inputData(0) = cCartPos(0);
			inputData(1) = cCartPos(1);
				inputData(2) = cCartPos(2);

		Beafilter->AddData(inputData);
		retCode = Beafilter->GetOutput(1, outputData);

		copyVector(0)=outputData(0);
				copyVector(1)=outputData(1);
						copyVector(2)=outputData(2);
	//	copyVector.Vector(outputData, inputDataDim);
		cCartVel.SetSubVector(0, copyVector);

		//... or in case os the first samples
		if (retCode == -1){//Until I reach the window length I do it the noisy way
			//Current Velocity
			//	cCartVel.SetSubVector(0, (cCartPos -pCartPos ) / _dt);
			//cCartVel.SetSubVector(3, (cCartDirY-pCartDirY) / _dt );
			//cCartVel.SetSubVector(6, (cCartDirZ-pCartDirZ) / _dt );
		}

		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%% Dynamic system %%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		// Desired acceleration (== force)
		cCartTargetAcc.SetSubVector(0, (cCartPos-fCartTargetPos)*Stiffness + cCartTargetVel.GetSubVector(0,3)*Damping);
		//cCartTargetAcc.SetSubVector(3, (fCartTargetDirY-cCartDirY) / pow(_dt, 2));
		//cCartTargetAcc.SetSubVector(6, (fCartTargetDirZ-cCartDirZ) / pow(_dt, 2));

		// Desired velocity
		cCartTargetVel.SetSubVector(0,cCartTargetVel.GetSubVector(0,3) + cCartTargetAcc.GetSubVector(0,3)*_dt);
		//cCartTargetVel.SetSubVector(3,cCartTargetAcc.GetSubVector(3,3)*_dt);
		//cCartTargetVel.SetSubVector(6,cCartTargetAcc.GetSubVector(6,3)*_dt);// I put the target direction of th velocity as the target direction of the acceleration but multiplied by dt

		//This is not necessary because this passage from velocity to position is done in the joint space
		cCartTargetPos=cCartPos + cCartTargetVel.GetSubVector(0,3)*_dt;

		mTargetVelocity.SetSubVector(0, (cCartTargetPos -cCartPos )/_dt);
		mTargetVelocity.SetSubVector(3, (cCartTargetDirY-cCartDirY)*100);
		mTargetVelocity.SetSubVector(6, (cCartTargetDirZ-cCartDirZ)*100);

		//mTargetVelocity.SetSubVector(3, (cCartTargetDirY-cCartDirY) / _dt );
		//mTargetVelocity.SetSubVector(6, (cCartTargetDirZ-cCartDirZ) / _dt );
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%% Inverse kinematics %%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		// Giving to the inverse kinematics the Jacobian and the target velocity
		mIKSolver.SetJacobian(mJacobian9);
		mIKSolver.SetTarget(mTargetVelocity, 0);
		//Solving IK
		mIKSolver.Solve();
		cJointTargetVel = mIKSolver.GetOutput();
		//mJointDesVel

		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%% Variable update %%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		// Reading current Joint position and saving it in lJoints
		mSKinematicChain->getEndPos(pCartPos.Array());//Saving pCartPos for next cycle


		//Update the joints value with the joint velocities computed from the Jacobian
		// This is the variable used to write to the simulator the values of the joints at this time step
		cJointTargetPos = cJointPos + cJointTargetVel*_dt;
		mSKinematicChain->setJoints(cJointTargetPos.Array());


		//Updating the variables for current cartesian and joint position for next cycle
		mSKinematicChain->getEndPos(cCartPos.Array());
	//	mSKinematicChain->getJoints(cJointPos.Array());
		//Reading the reached position
/*		mSKinematicChain->getEndPos(lPos.Array());

		mSKinematicChain->setJoints(mJointDesPos.Array());*/

		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%% Check for distance condition %%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		//distance2target = sqrt(pow(fCartTargetPos[0]-cCartPos[0], 2) + pow(fCartTargetPos[1]-cCartPos[1], 2) +pow(fCartTargetPos[2]-cCartPos[2], 2));
		distance2target_x = fCartTargetPos[0]-cCartPos[0];
		cout<<distance2target_x<<endl;
	//	if(abs(distance2target)>0.01)
	//		mCommand = COMMAND_Back;

		break;

	case PLANNER_JOINT:

		// If the Planner is held in the joint space joint values are directly updated
		//mJointTargetPos=mJointPosAll+(mJobJoints-mJointPosAll)*_dt*10;

		//My implementation
		cout<<"IN PLANNER JOINT"<<endl;
		//Output joint values
	//	mSKinematicChain->getJoints(cJointPos.Array());
	//	cJointPos.Print("cJointPos");

		//Update joint command
		cJointTargetPos = cJointPos + (fJointTargetPos-cJointPos)*_dt*10;

		J_distance2P0 = cJointTargetPos-fJointTargetPos;
		cout<<J_distance2P0.Norm()<<endl;
		if (J_distance2P0.Norm() < 0.1){
			mSKinematicChain->setJoints(cJointTargetPos.Array());
			mSKinematicChain->getEndPos(fCartTargetPos.Array());
			mSKinematicChain->getEndDirAxis(AXIS_Y, cCartTargetDirY.Array());
			mSKinematicChain->getEndDirAxis(AXIS_Z, cCartTargetDirZ.Array());
			mCommand=COMMAND_spring;// when it gets very close to the target position it passes to behave as a spring

		}
//		J_distance2Back = cJointTargetPos-BackPosition;
//		if (J_distance2Back.Norm() < 0.01 && mCommand == COMMAND_Back)
//					mCommand=COMMAND_Home;// when it gets very close to the target position it passes to behave as a spring

		break;





	}
	return STATUS_OK;
}
RobotInterface::Status monkeytask_test_1::RobotUpdateCore(){

	ros::spinOnce();

	// Read current Joints position and Sensors values

	//mJointPosAll    = mSensorsGroup.GetJointAngles();

	if (True_robot==false)
	{
		mSensorsGroup.ReadSensors();
		cJointPos = mSensorsGroup.GetJointAngles();
	}

	// Setting robot in position control
	if(mRobot->GetControlMode()!=Robot::CTRLMODE_POSITION)
		mRobot->SetControlMode(Robot::CTRLMODE_POSITION);

	// Write joint values stored in mJointTargetPos in the simulator
//	cJointTargetPos.Print("cJointTargetPos");
	Send_Postion_To_Robot(cJointTargetPos);
	mActuatorsGroup.SetJointAngles(cJointTargetPos);
	mActuatorsGroup.WriteActuators();
	mKinematicChain.Update();


	return STATUS_OK;
}

int monkeytask_test_1::RespondToConsoleCommand(const string cmd, const vector<string> &args){

	cout<<"Write your command"<<endl;

	if(cmd=="p0"){

		// set target point in joint space
		fJointTargetPos.Set(C, KUKA_DOF);
		fJointTargetPos.Mult((PI/180.0), fJointTargetPos);
		BackPosition.Set(C,KUKA_DOF);
		BackPosition.Mult((PI/180.0), BackPosition);
		//Start the contrl chain
		mCommand = COMMAND_2Position;

		}
	else if(cmd=="p1"){
		// set target point in joint space
		fJointTargetPos.Set(P1, KUKA_DOF);
		fJointTargetPos.Mult((PI/180.0), fJointTargetPos);
		BackPosition.Set(P1,KUKA_DOF);
		BackPosition.Mult((PI/180.0), BackPosition);
		//Start the contrl chain
		mCommand = COMMAND_2Position;
		}
	else if(cmd=="p2"){
		// set target point in joint space
		fJointTargetPos.Set(P2, KUKA_DOF);
		fJointTargetPos.Mult((PI/180.0), fJointTargetPos);
		BackPosition.Set(P2,KUKA_DOF);
		BackPosition.Mult((PI/180.0), BackPosition);
		//Start the contrl chain
		mCommand = COMMAND_2Position;

		}
	else if(cmd=="p3"){
		// set target point in joint space
		fJointTargetPos.Set(P3, KUKA_DOF);
		fJointTargetPos.Mult((PI/180.0), fJointTargetPos);
		BackPosition.Set(P3,KUKA_DOF);
		BackPosition.Mult((PI/180.0), BackPosition);
		//Start the contrl chain
		mCommand = COMMAND_2Position;

		}

	else if(cmd=="p4"){
		// set target point in joint space
		fJointTargetPos.Set(P4, KUKA_DOF);
		fJointTargetPos.Mult((PI/180.0), fJointTargetPos);
		BackPosition.Set(P4,KUKA_DOF);
		BackPosition.Mult((PI/180.0), BackPosition);
		//Start the contrl chain
		mCommand = COMMAND_2Position;

		}

	else if(cmd=="p5"){
		// set target point in joint space
		fJointTargetPos.Set(P5, KUKA_DOF);
		fJointTargetPos.Mult((PI/180.0), fJointTargetPos);
		BackPosition.Set(P5,KUKA_DOF);
		BackPosition.Mult((PI/180.0), BackPosition);
		//Start the contrl chain
		mCommand = COMMAND_2Position;

		}
	else if(cmd=="p6"){
		// set target point in joint space
		fJointTargetPos.Set(P6, KUKA_DOF);
		fJointTargetPos.Mult((PI/180.0), fJointTargetPos);
		BackPosition.Set(P6,KUKA_DOF);
		BackPosition.Mult((PI/180.0), BackPosition);
		//Start the contrl chain
		mCommand = COMMAND_2Position;


		}
	else if(cmd=="p7"){
			// set target point in joint space
			fJointTargetPos.Set(P7, KUKA_DOF);
			fJointTargetPos.Mult((PI/180.0), fJointTargetPos);
			BackPosition.Set(P7,KUKA_DOF);
			BackPosition.Mult((PI/180.0), BackPosition);
			//Start the contrl chain
			mCommand = COMMAND_2Position;


		}
	else if(cmd=="p8"){
				// set target point in joint space
				fJointTargetPos.Set(P8, KUKA_DOF);
				fJointTargetPos.Mult((PI/180.0), fJointTargetPos);
				BackPosition.Set(P8,KUKA_DOF);
				BackPosition.Mult((PI/180.0), BackPosition);
				//Start the contrl chain
				mCommand = COMMAND_2Position;
		}



	return STATUS_OK;
	//return 0;
}



extern "C"{
// These two "C" functions manage the creation and destruction of the class
monkeytask_test_1* create(){return new monkeytask_test_1();}
void destroy(monkeytask_test_1* module){delete module;}
}

