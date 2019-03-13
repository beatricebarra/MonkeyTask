// to do: 1. insert writeUDP calls throughout
// 2. check if following compiles

/*
 * Copyright (C) 2018 Swiss Primate Competence Center for Research, University of Fribourg, Switzerland
 * Author: Beatrice Barra (edited by andrew bogaard)
 * email:   beatrice.barra@unifr.ch and andrewrobert.bogaard@unifr.ch
 * phone: + 41 26 300 87 69
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

#include "monkeytask_arb_visual.h"

//Global variables
bool closed_loop=true;
bool True_robot=true;
bool Position_of_the_robot_recieved=false;
double k_v;
double maxSat = 90;//100;//120;//100; // 50 10 .1 .7 in slow move // was 90 40 .1 .9 in monkeytask2
double minSat = 40;//70;
double minROM = 0.1;
double maxROM = 0.9;

// for communicating with matlab
void readVisualizer(int *sockfd, int *udpreadsize, float *udpread, int *bytes2Read, struct sockaddr_in *cli_addr, socklen_t *cli_len, float *targetPosition, bitset<32> *trialStatus, float *targetThreshold);
void writeVisualizer(int *sockfd, float *writeForcePos, struct sockaddr_in *cli_adds, socklen_t *cli_len);

using namespace std;

monkeytask_arb_visual::monkeytask_arb_visual()
:RobotInterface(){
}
monkeytask_arb_visual::~monkeytask_arb_visual(){
}

void monkeytask_arb_visual::chatterCallback_position(const sensor_msgs::JointState & msg)
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

		JointEffort_handle(0)=msg.effort[0];
		JointEffort_handle(1)=msg.effort[1];
		JointEffort_handle(2)=msg.effort[2];
		JointEffort_handle(3)=msg.effort[3];
		JointEffort_handle(4)=msg.effort[4];
		JointEffort_handle(5)=msg.effort[5];
		JointEffort_handle(6)=msg.effort[6];
		cJointTORs=JointEffort_handle;

	}
}

void monkeytask_arb_visual::Send_Postion_To_Robot(Vector Position)
{
	if (True_robot)
	{
		kuka_fri_bridge::JointStateImpedance msg;
		msg.position.resize(KUKA_DOF);
		msg.stiffness.resize(KUKA_DOF);
		for (int i=0; i<KUKA_DOF;i=i+1)
		{
			msg.position[i]  = Position(i);
			msg.stiffness[i] = 2;
		}
		pub_command_robot_real.publish(msg);
	}
}

RobotInterface::Status monkeytask_arb_visual::RobotInit(){

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
	ffCartTargetPos.Resize(3);
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

//	Stiffness.Resize(3);
//	Damping.Resize(3);

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


	// Force variables
	eeForce.Resize(3);
	// Vectors for torques
	cJointTORs.Resize(KUKA_DOF);
	JointEffort_handle.Resize(KUKA_DOF);

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
	mIKSolver.SetThresholds(0.001,0.001);    // Singularities thresholds
	mIKSolver.Enable(true,0);                   // Enable first solver
	mIKSolver.SetDofsIndices(mKinematicChain.GetJointMapping(),0); // Joint maps for first solver

	lJointWeight(0) = 1.0;
	lJointWeight(1) = 1.0;
	lJointWeight(2) = 1.0;
	lJointWeight(3) = 0.5; // check
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


	cCartTargetDirY(0) = 0;
	cCartTargetDirY(1) = 1;
	cCartTargetDirY(2) = 0;

	cCartTargetDirZ(0) = 0;
	cCartTargetDirZ(1) = 0;
	cCartTargetDirZ(2) = 1;

	// Filter
	sgFilterOrder = 2;
	sgFilterWindowL = 50;
	inputDataDim = 3;
	outputData.resize(inputDataDim);
	inputData.resize(inputDataDim);
	Beafilter= new SGF::SavitzkyGolayFilter(inputDataDim, sgFilterOrder, sgFilterWindowL, _dt);

	mPlanner =NONE_planner;
	mCommand = NONE_comand;

	// Dynamical system parameters

	gain_ori=1;
	gain_ori1=1;
	gain_ori2=1;

	Stiffness2 = -8100;
	Damping2 = -180;

	Stiffness1 = -10;
	Damping1 = -1;

	Stiffness = Stiffness1;
	Damping = Damping1;

	home.Resize(KUKA_DOF);

	// Force sensors calibration

	// Velocity an stiffness

	timeout = 10000.0;
	targetThreshold = 0.02;

	maxeeForceInt = 255;
	mineeForceInt = 14;
	maxeeForceDouble = 12;
	mineeForceDouble = 0;

	// PointSequence Variables initialization
	idxPoint = 0;
	badTrial = 0;

	mRobot->SetControlMode(Robot::CTRLMODE_POSITION);
	ros::NodeHandle *n = mRobot->InitializeROS();
	if (True_robot)
	{
		pub_command_robot_real =  n->advertise<kuka_fri_bridge::JointStateImpedance>("/real_r_arm_controller/joint_imp_cmd", 3);
		sub_position_robot = n->subscribe("/real_r_arm_pos_controller/joint_states", 3, & monkeytask_arb_visual::chatterCallback_position,this);
	}

    return STATUS_OK;
}
RobotInterface::Status monkeytask_arb_visual::RobotFree(){
    return STATUS_OK;
}

RobotInterface::Status monkeytask_arb_visual::RobotStart(){

	// Setting the position of all joints to 0
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
		mJointVelLimitsDn(i) = -10*mSKinematicChain->getMaxVel(i);
		mJointVelLimitsUp(i) =  10*mSKinematicChain->getMaxVel(i);
	}

	//////////////////////////////////////////////////////////////////////////////
	// Open communication with visualizer ////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////
	bytes2Write = 56; // 3 floats pos, 3 floats force, 1 float relative pos, 7 floats joint angles (float=4bytes)
	bytes2Read = 36; // 1 float=32bools, 7 floats joint pos, 1 float for targetThreshold
	serverportno = 2001;
	cli_len = sizeof(cli_addr); // maybe fix this later?
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	if (sockfd < 0)
	  printf("ERROR opening socket\n");

	bzero((char *) &serv_addr, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(serverportno);

    cli_addr.sin_addr.s_addr = inet_addr("192.168.0.11");
    cli_addr.sin_family = AF_INET;
    cli_addr.sin_port = htons( 2003 );

    cli_adds.sin_addr.s_addr = inet_addr("192.168.0.11");
	cli_adds.sin_family = AF_INET;
	cli_adds.sin_port = htons( 2002 );

	if (bind(sockfd, (struct sockaddr *) &serv_addr,
		  sizeof(serv_addr)) < 0)
		  printf("ERROR on binding\n");


/* loop for testing udp
	float i;
	i=1;
	float f=2;
	int usecs = int((1/f)*1000000);
	cout<< usecs <<endl;
	while (1) {
		//sleep();
		usleep(usecs);
		writeForcePos[6] = (float)(i);
		writeVisualizer(&sockfd, writeForcePos, &cli_addr, &cli_len); // update status here
		readVisualizer(&sockfd, &udpreadsize, udpread, &bytes2Read, &cli_addr, &cli_len, &pullThreshold, &trialStatus);

		i=i+1;
		if (i>10000)
			i=0;
	}
*/

// initialize as if tenxp0 was entered in console

	waiting4robot = 0;
	bool_plannerjoint = 0;

	cout<<"End of Robot Start"<<endl;
    return STATUS_OK;
}

RobotInterface::Status monkeytask_arb_visual::RobotStop(){
    return STATUS_OK;
}

RobotInterface::Status monkeytask_arb_visual::RobotUpdate(){ //

	ros::spinOnce();

	//Local variables
	float distance2target;


	// Setting the joints to values currently stored in mJointPosAll
	//mSKinematicChain->setJoints(mJointPosAll.Array());
	mSKinematicChain->setJoints(cJointPos.Array());
	//Saving target point position and relative axis direction in the correspondent attribute in the KinematicChain object
	mSKinematicChain->getEndPos(cCartPos.Array());
	mSKinematicChain->getEndDirAxis(AXIS_X, cCartDirX.Array());
	mSKinematicChain->getEndDirAxis(AXIS_Y, cCartDirY.Array());
	mSKinematicChain->getEndDirAxis(AXIS_Z, cCartDirZ.Array());


	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %%%%%%%%%%%%%%%%%%%%% R/W command from/to MATLAB %%%%%%%%%%%%%%%%%%
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	lasttrialStatus = trialStatus; // save last command from matlab

	// Distance on x axis

	readVisualizer(&sockfd, &udpreadsize, udpread, &bytes2Read, &cli_addr, &cli_len, targetPosition, &trialStatus, &targetThreshold);

	// handle command from  matlab
	if (waiting4robot){// do nothing, keep same command pattern

		//cout << "in waiting for robot" << endl;
	}
	else if ((lasttrialStatus[17] != trialStatus[17]) || (lasttrialStatus[16] != trialStatus[16])) { // do nothing if no change (check syntax)

		cout << "made it" << endl;
		if (trialStatus[16]){ // tf rigid (cartesian and joint off)
			bool_plannercartesian = 0;
			bool_plannerjoint = 0;
			//cout << "in planner joint" << endl;
		}
		else if (!trialStatus[17]){// in which cartesian mode was just turned on and movement not requested, run following lines, and cycle robot
			bool_plannercartesian = 1;
			bool_plannerjoint = 0;
			//cout << "in planner cartesian" << endl;
			// Writing joints

			//mSKinematicChain->setJoints(cJointPos.Array());
			// Reading end-effector pose and orientation
			mSKinematicChain->getEndPos(fCartTargetPos.Array());
			mSKinematicChain->getEndPos(ffCartTargetPos.Array());
			mSKinematicChain->getEndDirAxis(AXIS_Y, cCartTargetDirY.Array());
			mSKinematicChain->getEndDirAxis(AXIS_Z, cCartTargetDirZ.Array());
			// Setting planner
			Stiffness=Stiffness1;

			Damping=Damping1;
			gain_ori=gain_ori1;

			//return STATUS_OK;
		}
		if (trialStatus[17]) { // want move
			// safety limits on target movements here?

			trialStatus[17] = 0;
			bool_plannerjoint = 1;
			copy(targetPosition, targetPosition+7, tmptargetPosition);
			fJointTargetPos.Set(tmptargetPosition, KUKA_DOF);
			fJointTargetPos.Mult((PI/180.0), fJointTargetPos);

			//cout << typeid(fJointTargetPos).name() << endl; // want N7MathLib6VectorE
			mCommand = COMMAND_2Position;
			//cout << "in sending move command" << endl;

		}
	}
	else {
		//cout << "did nothing" << endl;
		//trialStatus[17] = 0; // ensure that previous move command is overwritten (this shouldnt be necessary)
	}

	// Definition and implementation of type of movement
	if (bool_plannerjoint) {// used for when the robot is moving to positions, not 'moveable'
		// If the Planner is held in the joint space joint values are directly updated

		//cout << "in planner joint" << endl;
		// Target position in joint space
		home.Zero();

		InitialDistance.Resize(KUKA_DOF);
		InitialDistance= (fJointTargetPos-home);
		//cout<<InitialDistance<<endl;
		CurrentDistance.Resize(KUKA_DOF);
		CurrentDistance = (fJointTargetPos-cJointPos);

		if (CurrentDistance.Norm()/InitialDistance.Norm()<minROM)
			k_v = maxSat;
		else if (CurrentDistance.Norm()/InitialDistance.Norm()>maxROM)
			k_v = minSat;
		else
			k_v = (minSat-maxSat)/(maxROM-minROM)*CurrentDistance.Norm()/InitialDistance.Norm()+ maxSat;

		//k_v = 50;

		//cout << k_v << endl;

		cJointTargetPos = cJointPos + (fJointTargetPos-cJointPos)*_dt*k_v;//(InitialDistance.Norm()/CurrentDistance.Norm());//*Constant_joint;
		// Readin end effector pose
		mSKinematicChain->getEndPos(cCartPos.Array());

		distance2target_x = 0;
		//cout << distance2target_x << endl;
		writeForcePos[6] = distance2target_x;
		writeVisualizer(&sockfd, writeForcePos, &cli_adds, &cli_len); // update status here

		switch (mCommand){

			case COMMAND_2Position:

				// Going to target point
				J_distance2P0 = cJointPos-fJointTargetPos;

				waiting4robot = 1;

				//cout << J_distance2P0.Norm() << endl;

				if (J_distance2P0.Norm() < targetThreshold){

					cout << "found position" << endl;

					waiting4robot = 0;

					bool_plannerjoint = 0;

					//secs = ros::Time::now().toSec();

				}

			break;
		}
	}	else if (bool_plannercartesian){ // PLANNER CARTESIAN implements the impedance control with a stiffness and damping factor
		//cout << "in planner cartesian" << endl;

		//Dealing with Jacobian
		//NB: mJacobian3 is NOT the input, is the variable in which the computed Jacobian is written, it is a pointer
		mSKinematicChain->getJacobianPos(mJacobian3);
		mSKinematicChain->getJacobianDirection(AXIS_Y, lJacobianDirY);
		mSKinematicChain->getJacobianDirection(AXIS_Z, lJacobianDirZ);

		// Setting joint speed limits for the IK solver
		mIKSolver.SetLimits(mJointVelLimitsDn,mJointVelLimitsUp);
		mJacobian3.Mult(1.0,mJacobian3);
		for(int i=0; i<3; i++){
			mJacobian9.SetRow(mJacobian3.GetRow(i)   , i  );
			mJacobian9.SetRow(lJacobianDirY.GetRow(i), i+3);
			mJacobian9.SetRow(lJacobianDirZ.GetRow(i), i+6);
		}

		// Force
		eeForce= mJacobian3.Transpose()*cJointTORs;
		eeForceMod = eeForce.Norm();
		eeForceModInt = floor(eeForceMod*(maxeeForceInt-mineeForceInt)/(maxeeForceDouble-mineeForceDouble) - mineeForceDouble + mineeForceInt);


		//Updating the variables for current cartesian and joint position for next cycle
		mSKinematicChain->getEndPos(cCartPos.Array());
		mSKinematicChain->getEndDirAxis(AXIS_Y, cCartDirY.Array());
		mSKinematicChain->getEndDirAxis(AXIS_Z, cCartDirZ.Array());

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
		cCartVel.SetSubVector(0, copyVector);

		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%% Dynamic system %%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		// Desired acceleration (== force)

		cCartTargetAcc.SetSubVector(0, (cCartPos-fCartTargetPos)*Stiffness + cCartTargetVel.GetSubVector(0,3)*Damping);
		// Acceleration on the z axis is prevented
		cCartTargetAcc(2)=(cCartPos(2)-fCartTargetPos(2))*Stiffness*25 + cCartTargetVel(2)*Damping*5;
		//cCartTargetAcc.SetSubVector(0, 	cCartPos);
		// Desired velocity
		cCartTargetVel.SetSubVector(0,cCartTargetVel.GetSubVector(0,3) + cCartTargetAcc.GetSubVector(0,3)*_dt);
		// Desired position
		cCartTargetPos=cCartPos + cCartTargetVel.GetSubVector(0,3)*_dt;

		// Setting target velocity for ikine
		mTargetVelocity.SetSubVector(0, (cCartTargetPos -cCartPos )/_dt);
		mTargetVelocity.SetSubVector(3, (cCartTargetDirY-cCartDirY)*100);
		mTargetVelocity.SetSubVector(6, (cCartTargetDirZ-cCartDirZ)*100);

		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%% Inverse kinematics %%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		// Giving to the inverse kinematics the Jacobian and the target velocity
		mIKSolver.SetJacobian(mJacobian9);
		mIKSolver.SetTarget(mTargetVelocity, 0);
		//Solving IK
		mIKSolver.Solve();
		cJointTargetVel = mIKSolver.GetOutput();

		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%% Variable update %%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


		//Update the joints value with the joint velocities computed from the Jacobian
		// This is the variable used to write to the simulator the values of the joints at this time step
		cJointTargetPos = cJointPos + cJointTargetVel*_dt;
		mSKinematicChain->setJoints(cJointTargetPos.Array());

		//Updating the variables for current cartesian and joint position for next cycle
		mSKinematicChain->getEndPos(cCartPos.Array());
		mSKinematicChain->getEndDirAxis(AXIS_Y, cCartDirY.Array());
		mSKinematicChain->getEndDirAxis(AXIS_Z, cCartDirZ.Array());

		distance2target_x = ffCartTargetPos[0]-cCartPos[0];
		//cout << distance2target_x << endl;

		writeForcePos[0] = ffCartTargetPos[0]-cCartPos[0];
		writeForcePos[1] = ffCartTargetPos[1]-cCartPos[1];
		writeForcePos[2] = ffCartTargetPos[2]-cCartPos[2];
		writeForcePos[3] = eeForce[0];
		writeForcePos[4] = eeForce[1];
		writeForcePos[5] = eeForce[2];
		writeForcePos[6] = distance2target_x;
		writeVisualizer(&sockfd, writeForcePos, &cli_adds, &cli_len); // update status here

		switch(mCommand){

		}

	}

return STATUS_OK;
}
RobotInterface::Status monkeytask_arb_visual::RobotUpdateCore(){
	ros::spinOnce();

	if (True_robot==false)
	{
		mSensorsGroup.ReadSensors();
		cJointPos = mSensorsGroup.GetJointAngles();
	}

	// Setting robot in position control
	if(mRobot->GetControlMode()!=Robot::CTRLMODE_POSITION)
		mRobot->SetControlMode(Robot::CTRLMODE_POSITION);

	// Write joint values stored in mJointTargetPos in the simulator
	Send_Postion_To_Robot(cJointTargetPos);
	mActuatorsGroup.SetJointAngles(cJointTargetPos);
	mActuatorsGroup.WriteActuators();
	mKinematicChain.Update();

    return STATUS_OK;
}

void readVisualizer(int *sockfd, int *udpreadsize, float *udpread, int *bytes2Read, struct sockaddr_in *cli_addr, socklen_t *cli_len, float *targetPosition, bitset<32> *trialStatus, float *targetThreshold){

	// 1 float (32 bools trialStatus), 7 floats (target position in joint angle space)

	int buffersize = *bytes2Read*1000;
	float buffer[(int)(buffersize/sizeof(float))];

	*udpreadsize = recvfrom(*sockfd, buffer, buffersize, MSG_DONTWAIT, (struct sockaddr *)cli_addr, cli_len); // 20 bytes

	//cout << "bytes in read buffer " << *udpreadsize << endl; // double check that the first or last are the most recent?

	if(*udpreadsize >= (int)(*bytes2Read/sizeof(float))) {

		udpread[0] = buffer[0];
		udpread[1] = buffer[1];
		udpread[2] = buffer[2];
		udpread[3] = buffer[3];
		udpread[4] = buffer[4];
		udpread[5] = buffer[5];
		udpread[6] = buffer[6];
		udpread[7] = buffer[7];
		udpread[8] = buffer[8];

		*trialStatus = bitset<32>(udpread[0]);

		targetPosition[0] = udpread[1];
		targetPosition[1] = udpread[2];
		targetPosition[2] = udpread[3];
		targetPosition[3] = udpread[4];
		targetPosition[4] = udpread[5];
		targetPosition[5] = udpread[6];
		targetPosition[6] = udpread[7];

		*targetThreshold = udpread[8];

		//cout << "target threshold is: " << *targetThreshold << endl;

		//cout << targetPosition[6] << endl;

	}

}

int monkeytask_arb_visual::RespondToConsoleCommand(const string cmd, const vector<string> &args){


	return STATUS_OK;
}


void writeVisualizer(int *sockfd, float *writeForcePos, struct sockaddr_in *cli_adds, socklen_t *cli_len) {
//float writeForcePos[7]; // 3d pos, 3d force, 1d rel_pos

	writeForcePos[6] = (float)(writeForcePos[6]*1000); // writeForcePos[6] is meters, thus convert to mm here
	writeForcePos[0] = (float)(writeForcePos[0]*1000); // these are displacement from startining point in meters, convert to mm
	writeForcePos[1] = (float)(writeForcePos[1]*1000);
	writeForcePos[2] = (float)(writeForcePos[2]*1000);

	//cout << "relative position " << writeForcePos[6] << endl;

	int succ;

	succ = sendto(*sockfd, writeForcePos, 28, MSG_DONTWAIT, (struct sockaddr *)cli_adds, *cli_len); //

}

extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    monkeytask_arb_visual* create(){return new monkeytask_arb_visual();}
    void destroy(monkeytask_arb_visual* module){delete module;}
}
