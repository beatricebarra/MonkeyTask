/*
 * Copyright (C) 2018 Swiss Primate Competence Center for Research, University of Fribourg, Switzerland
 * Author: Beatrice Barra
 * email:   beatrice.barra@unifr.ch
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

#include "monkeytask_arb_data.h"

//Global variables
bool closed_loop=true;
bool True_robot=true;
bool Position_of_the_robot_recieved=false;
bool isfirsttrial = 1; // andrew
double k_v;
double maxSat = 90;//100;//120;//100;
double minSat = 40;//70;
double minROM = 0.1;
double maxROM = 0.9;

//For audio
const int AMPLITUDE = 28000;
const int SAMPLE_RATE = 44100;
const int SAMPLE_RATE_2 = 22050;
void audio_callback(void *user_data, Uint8 *raw_buffer, int bytes);
void playToneGo();
void playToneStart();


using namespace std;

monkeytask_arb_data::monkeytask_arb_data()
:RobotInterface(){
}
monkeytask_arb_data::~monkeytask_arb_data(){
}

void monkeytask_arb_data::chatterCallback_position(const sensor_msgs::JointState & msg)
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

void monkeytask_arb_data::Send_Postion_To_Robot(Vector Position)
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

RobotInterface::Status monkeytask_arb_data::RobotInit(){

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

	//Types of possible commands and trials
	AddConsoleCommand("tenxp0");
	AddConsoleCommand("tenxpC");
	AddConsoleCommand("TenPointSequence");
	AddConsoleCommand("SixPointSequence");
	AddConsoleCommand("ThreePointSequence");
	AddConsoleCommand("FourPointSequence");
	AddConsoleCommand("FivePointSequence");
	AddConsoleCommand("chair");
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
	Constant_joint=1;


	// Force sensors calibration


	// Velocity an stiffness

	timeout = 10000.0;
	pullThreshold = 0.08;
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
		sub_position_robot = n->subscribe("/real_r_arm_pos_controller/joint_states", 3, & monkeytask_arb_data::chatterCallback_position,this);
	}


    return STATUS_OK;
}
RobotInterface::Status monkeytask_arb_data::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status monkeytask_arb_data::RobotStart(){


	//system("modprobe usbserial");
	//system("stty -F /dev/ttyUSB5 ospeed 230400 -parenb tostop -ixon");

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
		mJointVelLimitsDn(i) = -10*mSKinematicChain->getMaxVel(i);
		mJointVelLimitsUp(i) =  10*mSKinematicChain->getMaxVel(i);
	}

	// Opening communication with arduino
	arduinoFD = open("/dev/ttyACM0", O_RDWR| O_NOCTTY | O_NONBLOCK);//| O_NOCTTY | O_NDELAY, O_RDWR| O_NOCTTY , O_RDWR | O_NONBLOCK
	struct termios settings, oldsettings;
	if (arduinoFD < 0)
	{
		perror("/dev/ttyACM0-->open()");
		exit(EXIT_FAILURE);

	}
	tcgetattr(arduinoFD, &oldsettings);
	memset(&settings, 0, sizeof(settings));

	cfsetispeed(&settings, B115200);
	cfsetospeed(&settings, B115200);
	settings.c_cflag     &=  ~PARENB;            // Make 8n1
	settings.c_cflag     &=  ~CSTOPB;
	settings.c_cflag     &=  ~CSIZE;
	settings.c_cflag     |=  CS8;
	settings.c_cflag     &=  ~CRTSCTS;           // no flow control
	settings.c_cc[VMIN]   =  1;                  // read doesn't block
	settings.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	settings.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
	/* Make raw */
	cfmakeraw(&settings);
	/* Flush Port, then applies attributes */
	tcflush(arduinoFD, TCIFLUSH);
	tcsetattr(arduinoFD,TCSANOW,&settings);
	// Communicating success or failure
	int _errno = errno;
	printf("%s\n", strerror(_errno));
	cout<<"Arduino has file descriptor number";
	cout<<arduinoFD<<endl;

	// Opening communication with DAC

//	DACFD = open("/dev/ttyUSB5", O_RDWR| O_NOCTTY );//| O_NOCTTY | O_NDELAY, O_RDWR| O_NOCTTY , O_RDWR | O_NONBLOCK
//	//struct termios settings, oldsettings;
//	if (DACFD < 0)
//	{
//		perror("/dev/ttyUSB5-->open()");
//		exit(EXIT_FAILURE);
//
//	}
//	tcgetattr(DACFD, &oldsettings);
//	memset(&settings, 0, sizeof(settings));
//
//	cfsetispeed(&settings, B230400);
//	cfsetospeed(&settings, B230400);
//	settings.c_cflag     &=  ~PARENB;            // Make 8n1
//	settings.c_cflag     &=  ~CSTOPB;
//	settings.c_cflag     &=  ~CSIZE;
//	settings.c_cflag     |=  CS8;
//	settings.c_cflag     &=  ~CRTSCTS;           // no flow control
//	settings.c_cc[VMIN]   =  1;                  // read doesn't block
//	settings.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
//	settings.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
//	/* Make raw */
//	cfmakeraw(&settings);
//	/* Flush Port, then applies attributes */
//	tcflush(DACFD, TCIFLUSH);
//	tcsetattr(DACFD,TCSANOW,&settings);
//	// Communicating success or failure
//	int _errno_DAC = errno;
//	printf("%s\n", strerror(_errno_DAC));
//	cout<<"DAC has file descriptor number";
//	cout<<"DACFD"<<endl;



	DACfile = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	if (DACfile<0) printf("Error: comunication with vicon not open\n");

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if(tcgetattr(DACfile, &tty)!= 0) printf("Error\n");

	cfsetospeed(&tty, (speed_t)B230400);
	cfsetispeed(&tty, (speed_t)B230400);
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= ~CS8;

	tty.c_cc[VMIN]  =1;
	tty.c_cc[VTIME]  =5;
	tty.c_cflag |= CREAD | CLOCAL;

	cfmakeraw(&tty);
	tcflush(DACfile, TCIFLUSH);
	system("modprobe usbserial");
	system("stty ospeed 230400 </dev/ttyUSB0");

	char monkeyName[20];
	printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
	printf("Enter the name of the Monkey\n");
	scanf("%s", monkeyName);
	printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");


	char userAnswer;
	printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
	printf("Input the desired pull threshold in cm\n");
	scanf("%f", &pullThreshold);
	scanf("%c", &userAnswer);
	pullThreshold = pullThreshold / 100; // converting in meters
	printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
	// Asking for timeout vs button


	printf("User needs have been recorded! \n Have a happy experimental session and acquire good data!!!\n");
	printf("BETTER THAN YESTERDAY, BUT WORSE THAN TOMORROW!\n");
	printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");


	// FILE TXT--> name is date and time
	time_t t = time(0);
	struct tm *now = localtime(&t);
	cout << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' << (now->tm_mday)<<endl;
	char currentDateTime[50];
	strftime(currentDateTime, sizeof(currentDateTime), "%Y_%m_%d_%X", now);
	char newName[50];
	sprintf (newName, "%s_%d%.2d%.2d_%d-%.2d-%.2d.txt", monkeyName, now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
    myfile.open (newName);
    cout<<myfile.is_open()<<endl;
    // Writing labels
    myfile<<"Fx"<<"," <<"Fy"<<","<<"Fz"<<","<<"Trigger"<<","<<"Time[s]"<<","<<"Time[us]"<<","<<"PositionID"<<","<<"px"<<","<<"py"<<","<<"pz"<<endl;

    gettimeofday(&t0, NULL);


	cout<<"End of Robot Start"<<endl;
    return STATUS_OK;
}    
RobotInterface::Status monkeytask_arb_data::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status monkeytask_arb_data::RobotUpdate(){
	ros::spinOnce();

	//Local variables
	float distance2target;
	float distance2target_x;
	long seconds, useconds;
	timeval curTime;
	int milli;
	struct tm *now;
	char thisInstantseconds[50];
	char thisInstant[70];


	// Setting the joints to values currently stored in mJointPosAll
	//mSKinematicChain->setJoints(mJointPosAll.Array());
	mSKinematicChain->setJoints(cJointPos.Array());
	//Saving target point position and relative axis direction in the correspondent attribute in the KinematicChain object
	mSKinematicChain->getEndPos(cCartPos.Array());
	mSKinematicChain->getEndDirAxis(AXIS_X, cCartDirX.Array());
	mSKinematicChain->getEndDirAxis(AXIS_Y, cCartDirY.Array());
	mSKinematicChain->getEndDirAxis(AXIS_Z, cCartDirZ.Array());
	//	cCartDirX.Print("cCartDirX");
	//	cCartDirY.Print("cCartDirY");
	//	cCartDirZ.Print("cCartDirZ");


	//Setting different planners for different commands
	switch(mCommand){
	case COMMAND_2Position :
		mPlanner = PLANNER_JOINT;
		break;
	case COMMAND_spring :
		//if (ros::Time::now().toSec()-secs>1 )
		//{

			// Writing on arduino
			char mywrite[1];
			mywrite[0]= 1;
			size = write(arduinoFD, &mywrite, sizeof(mywrite));


			// Writing joints
			mSKinematicChain->setJoints(cJointPos.Array());
			// Reading end-effector pose and orientation
			mSKinematicChain->getEndPos(fCartTargetPos.Array());
			mSKinematicChain->getEndPos(ffCartTargetPos.Array());
			mSKinematicChain->getEndDirAxis(AXIS_Y, cCartTargetDirY.Array());
			mSKinematicChain->getEndDirAxis(AXIS_Z, cCartTargetDirZ.Array());
			// Setting planner
			Stiffness=Stiffness1;

			Damping=Damping1;
			gain_ori=gain_ori1;
			mPlanner = PLANNER_CARTESIAN;//Add termination condition in planner
			mCommand = NONE_comand;

			// Play a tone at 440 Hz
			playToneGo();


		//}
		break;
	case COMMAND_Back:
		fJointTargetPos = backSequence.GetRow(idxPoint-1, fJointTargetPos);
		if(idxPoint == 0){
			//cout<<"I set the index at "<< nP<<endl;
			fJointTargetPos = backSequence.GetRow(nP-1, fJointTargetPos);
		}
		//cout<<"back to position: "<<idxPoint<<"  "<<fJointTargetPos<<endl;
		mPlanner = PLANNER_JOINT;
		//cout<<"commandback"<<endl;
		// hw modifcation for Sina was to comment the line planner joint and uncomment the following 2 lines
		//Stiffness=Stiffness2;
		//Damping=Damping2;
		//gain_ori=gain_ori2;
	//	mPlanner = PLANNER_CARTESIAN;

		break;
	case COMMAND_Home:
		//mPlanner = PLANNER_JOINT;
		//fJointTargetPos.Zero(); //added by andrew 17-9-2018. helped but robot still moves back a bit. need to find right positipon here
		break;
	case COMMAND_Wait4Go:
		mPlanner = PLANNER_JOINT;
			//mSKinematicChain->setJoints(fJointTargetPos.Array());
		break;
	}

	//time_t t = time(0);
	//gettimeofday(&curTime, NULL);
	//milli= curTime.tv_usec / 1000;
	//now = localtime(&t);
	//strftime(thisInstantseconds, sizeof(thisInstantseconds), "%X", now);
	//sprintf(thisInstant, "%s:%d", thisInstantseconds, milli);
	// Alternative approach
	gettimeofday(&currentTime, NULL);
	seconds  = currentTime.tv_sec  - t0.tv_sec;
	useconds = currentTime.tv_usec;// - t0.tv_usec;

	//cout<< "Sec and usec "<<seconds<<useconds<< "currentTime"<<currentTime.tv_sec << currentTime.tv_usec <<"t0"<< t0.tv_sec<< t0.tv_usec<<endl;
	/*time_t t = time(0);
	struct tm *now = localtime(&t);
	char thisInstant[50];
	strftime(thisInstant, sizeof(thisInstant), "%X", now);*/
	char forcewrite[8];

	 //Force modulus
//	eeForce = mJacobian3*cJointTORs;
//	//eeForceMod = eeForce.Norm();
//	eeForceMod = eeForce.Norm();
//	eeForceModInt = floor(eeForceMod*(maxeeForceInt-mineeForceInt)/(maxeeForceDouble-mineeForceDouble) - mineeForceDouble + mineeForceInt);
//	write(arduinoFD, &eeForceModInt, 1);


//cout << "iter" << endl;
	// Definition and implementation of type of movement
	switch(mPlanner){
	// PLANNER CARTESIAN implements the impedance control with a stiffness and damping factor
	case PLANNER_CARTESIAN:

		// Time scale of the loop:
		// p --> t-1
		// c --> t
		// d --> t+1
		cout<<"IN PLANNER CARTESIAN"<<endl;

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

		// Force modulus
		eeForce = mJacobian3*cJointTORs;
		// Correction of the force
		JT = mJacobian3.Transpose();
		temp_JJT = mJacobian3*JT;
		temp_JJTI = temp_JJT.Inverse();
		temp_JJT.IsInverseOk();
		//if (temp_JJT.IsInverseOk()) cout<< "siamosalvi"<<endl;
		//else if (!temp_JJT.IsInverseOk()) cout<< "siamofottuti"<<endl;

		eeForceCorrected = temp_JJTI*eeForce;
		eeForceCORRECT= JT*cJointTORs;

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


		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%% Check for distance condition %%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


		// write file
		myfile << eeForce[0] <<","<< eeForce[1] << "," << eeForce[2] << "," ;
		myfile << eeForceCorrected[0] <<","<< eeForceCorrected[1] << "," << eeForceCorrected[2] << "," ;
		myfile << eeForceCORRECT[0] <<","<< eeForceCORRECT[1] << "," << eeForceCORRECT[2] << ",";
		myfile << cJointTORs[0] <<","<< cJointTORs[1] << "," << cJointTORs[2]<< "," <<cJointTORs[3]<< ","<<cJointTORs[4]<< ","<<cJointTORs[5]<< ","<<cJointTORs[6]<< ",";
		myfile << mJacobian3.GetRow(0)<<"," <<mJacobian3.GetRow(1)<<","<<mJacobian3.GetRow(2)<<",";
		myfile << cJointPos[0] <<","<< cJointPos[1] << "," << cJointPos[2]<< "," <<cJointPos[3]<< ","<<cJointPos[4]<< ","<<cJointPos[5]<< ","<<cJointPos[6]<<",";
		myfile<< 2 << "," ;
		myfile<< seconds << "," << useconds << ","<< idxPoint << ",";
		myfile<<cCartPos[0]<<"," <<cCartPos[1]<<","<<cCartPos[2]<<endl;

		 //Force modulus

		eeForceMod = eeForceCORRECT.Norm();
		eeForceModInt = floor(eeForceMod*(maxeeForceInt-mineeForceInt)/(maxeeForceDouble-mineeForceDouble) - mineeForceDouble + mineeForceInt);

		write(arduinoFD, &eeForceModInt, 1);

		// Distance on x axis
		distance2target_x = ffCartTargetPos[0]-cCartPos[0];
		//cout << distance2target_x << endl;
		switch(mCommand){
		case NONE_comand:

			// Writing force to file
			//myfile << eeForce[0] <<","<< eeForce[1] << "," << eeForce[2] << "," << 2 << "," << thisInstant << "," << idxPoint << ",";
			//myfile<<cCartPos[0]<<"," <<cCartPos[1]<<","<<cCartPos[2]<<endl;
			//cout<<distance2target_x<<endl;

			// Condition for going back home:
			// - The robot has been pulled of 10 cm OR 00 seconds elapsed without any accomplishment from the monkey
			if((distance2target_x)> pullThreshold){ // in the final application I need a major sign


				myfile << 0 <<","<< 0 << "," << 0 << "," ;
				myfile << 0<<","<< 0 << "," << 0<< "," ;
				myfile << 0 <<","<< 0 << "," << 0 << ",";
				myfile << 0 <<","<< 0<< "," << 0<< "," <<0<< ","<<0<< ","<<0<< ","<<0<< ",";
				myfile << 0<<"," <<0<<","<<0<<",";
				myfile << 0<<","<< 0 << "," <<0<< "," <<0<< ","<<0<< ","<<0<< ","<<0<<",";
				myfile<< 3 << "," ;
				myfile<< seconds << "," << useconds << ","<< idxPoint << ",";
				myfile<<0<<"," <<0<<","<<0<<endl;

				//Trig out
				char mywrite[1];
				mywrite[0]= 2;
				size = write(arduinoFD, &mywrite, sizeof(mywrite));

				mCommand = COMMAND_Back;
				idxPoint = idxPoint + 1;
				if (idxPoint>nP-1) idxPoint =0;
				//cout<<"The next point index is "<<idxPoint<<endl;
			}
			else {

				char readBytes[1];
				char trashBytes[5];
				size = read(arduinoFD, &readBytes, 1);
				//if(ros::Time::now().toSec()-secs>timeout || readBytes[0] == 'N'){
				if (readBytes[0] == 'N'){
					badTrial = badTrial + 1;

					myfile << 0 <<","<< 0 << "," << 0 << "," ;
					myfile << 0<<","<< 0 << "," << 0<< "," ;
					myfile << 0 <<","<< 0 << "," << 0 << ",";
					myfile << 0 <<","<< 0<< "," << 0<< "," <<0<< ","<<0<< ","<<0<< ","<<0<< ",";
					myfile << 0<<"," <<0<<","<<0<<",";
					myfile << 0<<","<< 0 << "," <<0<< "," <<0<< ","<<0<< ","<<0<< ","<<0<<",";
					myfile<< 1<< "," ;
					myfile<< seconds << "," << useconds << ","<< idxPoint << ",";
					myfile<<0<<"," <<0<<","<<0<<endl;

					if (badTrial >=3) {
						//cout<<"more than 3 trial"<<endl;
						idxPoint = idxPoint+1;
						if (idxPoint>nP-1) idxPoint =0;
						badTrial = 0;
					}
					//cout<<"The next point index is "<<idxPoint<<endl;
					mCommand = COMMAND_Back;
				}
			}

			break;

		case COMMAND_Back:

			// Writing force to file
			//myfile << eeForce[0] <<","<< eeForce[1] << "," << eeForce[2] << "," << 0 << ","<<thisInstant << "," << idxPoint << ",";
			//myfile<<cCartPos[0]<<"," <<cCartPos[1]<<","<<cCartPos[2]<<endl;
			// Going back to the original target point after the monkey pulled
			if((distance2target_x)< targetThreshold ){ // in the final application I need a major sign
					mCommand = COMMAND_Home;
			}
			break;

		default : cout << "test" << endl;

		}

	break;// Closing PLANNER CARTESIAN

	case PLANNER_JOINT:
		// If the Planner is held in the joint space joint values are directly updated

		//My implementation
		cout<<"IN PLANNER JOINT"<<endl;
		// Target position in joint space
		home.Zero();

		InitialDistance.Resize(KUKA_DOF);
		InitialDistance= (fJointTargetPos-home);
		//cout<<InitialDistance<<endl;
		CurrentDistance.Resize(KUKA_DOF);
		CurrentDistance = (fJointTargetPos-cJointPos);
		cout << CurrentDistance << "and" << InitialDistance << endl;
		if (CurrentDistance.Norm()/InitialDistance.Norm()<minROM)
			k_v = maxSat;
		else if (CurrentDistance.Norm()/InitialDistance.Norm()>maxROM)
			k_v = minSat;
		else
			k_v = (minSat-maxSat)/(maxROM-minROM)*CurrentDistance.Norm()/InitialDistance.Norm()+ maxSat;

		cJointTargetPos = cJointPos + (fJointTargetPos-cJointPos)*_dt*k_v;//(InitialDistance.Norm()/CurrentDistance.Norm());//*Constant_joint;
		// Readin end effector pose
		mSKinematicChain->getEndPos(cCartPos.Array());

		// Force modulus
		//eeForce = mJacobian3*cJointTORs;
		//eeForceMod = eeForce.Norm();
		//eeForceMod = eeForce.Norm();





		//time_t thisInstant  = time(0);
		switch (mCommand){
		case NONE_comand:
			// Writing force to file
			//myfile << eeForce[0] <<","<< eeForce[1] << "," << eeForce[2] << "," << 0 << ","<<thisInstant << "," << idxPoint << ",";
			//myfile<<cCartPos[0]<<"," <<cCartPos[1]<<","<<cCartPos[2]<<endl;
			// attempt to write
			//forceValue= eeForceMod*maxDACScale/maxForceValue;
			//size = write(DACFD, &forceValue, 8);

			break;

		case COMMAND_2Position:

			// Sending trigger to file
			// Writing force to file
			//myfile << eeForce[0] <<","<< eeForce[1] << "," << eeForce[2] << "," << 1 << ","<<thisInstant << "," << idxPoint << ",";
			//myfile<<cCartPos[0]<<"," <<cCartPos[1]<<","<<cCartPos[2]<<endl;

			// Going to target point
			J_distance2P0 = cJointTargetPos-fJointTargetPos;

			if (J_distance2P0.Norm() < targetThreshold){//was 0.016 //if (abs(J_distance2P0[0])*(180.0/PI) < 1 && abs(J_distance2P0[1])*(180.0/PI)<1 && abs(J_distance2P0[2])*(180.0/PI) < 1 && abs(J_distance2P0[3])*(180.0/PI)<1 && abs(J_distance2P0[4])*(180.0/PI) < 1 && abs(J_distance2P0[5])*(180.0/PI)<1 && abs(J_distance2P0[6])*(180.0/PI) < 1){//if (J_distance2P0.Norm() < 0.09)

				secs =ros::Time::now().toSec();
				cJointTargetPos=cJointPos;
				// Set Joints
				mSKinematicChain->setJoints(cJointPos.Array());
				// Reading end-effector pose and orientation
				mSKinematicChain->getEndPos(fCartTargetPos.Array());
				mSKinematicChain->getEndDirAxis(AXIS_Y, cCartTargetDirY.Array());
				mSKinematicChain->getEndDirAxis(AXIS_Z, cCartTargetDirZ.Array());
				mCommand=COMMAND_spring;
				char trash[5];
				size = read(arduinoFD, &trash, 5);

			}

			break;
		case COMMAND_Back:

			// Writing force to file
			//myfile << eeForce[0] <<","<< eeForce[1] << "," << eeForce[2] << "," << 0 << ","<<thisInstant << "," << idxPoint << ",";
			//myfile<<cCartPos[0]<<"," <<cCartPos[1]<<","<<cCartPos[2]<<endl;

			// Going back to the original target point after the monkey pulled
			//Constant_joint=1;
			J_distance2Back = cJointPos-fJointTargetPos;

			if (J_distance2Back.Norm() < targetThreshold ) // was 0.1 andrew
			{
				//mCommand=COMMAND_Home;
				mCommand=COMMAND_Wait4Go; // to use button
				//mCommand=COMMAND_spring; // to go straight to next trial
			}
			break;

		case COMMAND_Home:
			//Writing force file
			// Writing force to file
			//myfile << eeForce[0] <<","<< eeForce[1] << "," << eeForce[2] << "," << 0 << ","<<thisInstant << "," << idxPoint << "," ;
			//myfile<<cCartPos[0]<<"," <<cCartPos[1]<<","<<cCartPos[2]<<endl;

			// Going back to the original target point after the monkey pulled
			Constant_joint=1;
			J_distance2Home = cJointPos-fJointTargetPos;
			if (J_distance2Home.Norm() < targetThreshold ) // was 0.1 andrew
			{
				if (idxPoint >= nP){
					//cout<<"putting idxPoint to 0"<<endl;
					idxPoint = 0;
				}

				mCommand=COMMAND_Wait4Go;
			}
			break;

		case COMMAND_Wait4Go:

			cout<<"Waiting for command"<<endl;

			int readFlag = 1;
			int buffersize = 1;
			char readBytes[buffersize];

			// Empty the buffer
			char trashBytes[100000];
			size = read(arduinoFD, &trashBytes, 100000);
			// Wait for button
			while(readFlag){//change

				// Writing force to file
				//myfile << eeForce[0] <<","<< eeForce[1] << "," << eeForce[2] << "," << 0 << ","<<thisInstant << "," << idxPoint << ",";
				//myfile<<cCartPos[0]<<"," <<cCartPos[1]<<","<<cCartPos[2]<<endl;

				size = read(arduinoFD, &readBytes, buffersize);

				if (readBytes[0]=='N'){//readBytes[0]
					//cout<<"Going to position"<<endl; //this is the start pos

					// Writing to file a 0
					myfile << 0 <<","<< 0 << "," << 0 << "," ;
					myfile << 0<<","<< 0 << "," << 0<< "," ;
					myfile << 0 <<","<< 0 << "," << 0 << ",";
					myfile << 0 <<","<< 0<< "," << 0<< "," <<0<< ","<<0<< ","<<0<< ","<<0<< ",";
					myfile << 0<<"," <<0<<","<<0<<",";
					myfile << 0<<","<< 0 << "," <<0<< "," <<0<< ","<<0<< ","<<0<< ","<<0<<",";
					myfile<< 7 << "," ;
					myfile<< seconds << "," << useconds << ","<< idxPoint << ",";
					myfile<<0<<"," <<0<<","<<0<<endl;

					if (isfirsttrial){
					fJointTargetPos = pointSequence.GetRow(idxPoint, fJointTargetPos); // i think this is it andrew
					mCommand = COMMAND_2Position; // just did these 2 lines
					isfirsttrial=0;
					}
					else {
					// andrew added
						playToneStart();
									mCommand=COMMAND_spring;
									char trash[5];
									size = read(arduinoFD, &trash, 5);
									//andrew added now

					}
					readFlag = 0;

					// This part has been added by Elvira to send an output to the Arduino

					char mywrite[1];
					//mywrite[0]= 3;
					//size = write(arduinoFD, &mywrite, sizeof(mywrite));

					modidxPoint = idxPoint+4;
					//cout<<"Point"<<modidxPoint<<endl;
					playToneStart();
					size = write(arduinoFD, &modidxPoint, sizeof(mywrite));
				}
			}
			break;
		} // end switch on planner commands

		break; // closing planner joint


		default :

			//cout << "tttttttttttttttttttttttttttttttttttttt" << endl;

			break;

	} // end switch on planner

    return STATUS_OK;
}
RobotInterface::Status monkeytask_arb_data::RobotUpdateCore(){
	ros::spinOnce();

	if (True_robot==false)
	{
		mSensorsGroup.ReadSensors();
		cJointPos = mSensorsGroup.GetJointAngles();
	}

	// Setting robot in position control
	if(mRobot->GetControlMode()!=Robot::CTRLMODE_POSITION)
		mRobot->SetControlMode(Robot::CTRLMODE_POSITION);

	// Write joint values stored in mJointTargetPos in the simulatorj
	Send_Postion_To_Robot(cJointTargetPos);
	mActuatorsGroup.SetJointAngles(cJointTargetPos);
	mActuatorsGroup.WriteActuators();
	mKinematicChain.Update();

    return STATUS_OK;
}
int monkeytask_arb_data::RespondToConsoleCommand(const string cmd, const vector<string> &args){

	cout<<"Write your command"<<endl;
	if(cmd=="tenxp0"){
		Constant_joint=1;
		nP = 10;
		idxPoint=0;
		pointSequence.Resize(nP, KUKA_DOF);
		backSequence.Resize(nP, KUKA_DOF);

		pointSequence.Set(*tenxp0, nP, KUKA_DOF);
		pointSequence.Mult((PI/180.0), pointSequence);

		backSequence.Set(*allBack, nP, KUKA_DOF);
		backSequence.Mult((PI/180.0), backSequence);

		//Start the control chain
		mPlanner = PLANNER_JOINT;
		mCommand = COMMAND_Wait4Go;
		cout<<"switched to command wait4go"<<endl;

	}
	if(cmd=="tenxpC"){
		Constant_joint=1;
		nP = 10;
		idxPoint=0;
		pointSequence.Resize(nP, KUKA_DOF);
		backSequence.Resize(nP, KUKA_DOF);

		pointSequence.Set(*tenxpC, nP, KUKA_DOF);
		pointSequence.Mult((PI/180.0), pointSequence);

		backSequence.Set(*tenxpC, nP, KUKA_DOF);
		backSequence.Mult((PI/180.0), backSequence);

		//Start the control chain
		mPlanner = PLANNER_JOINT;
		mCommand = COMMAND_Wait4Go;
		cout<<"switched to command wait4go"<<endl;

	}
	if(cmd=="TenPointSequence"){
		Constant_joint=1;
		nP = 10;
		idxPoint=0;
		pointSequence.Resize(nP, KUKA_DOF);
		backSequence.Resize(nP, KUKA_DOF);

		pointSequence.Set(*Sequence1, nP, KUKA_DOF);
		pointSequence.Mult((PI/180.0), pointSequence);

		backSequence.Set(*Sequence1, nP, KUKA_DOF);
		backSequence.Mult((PI/180.0), backSequence);

		//Start the control chain
		mPlanner = PLANNER_JOINT;
		mCommand = COMMAND_Wait4Go;
		cout<<"switched to command wait4go"<<endl;

	}
	if(cmd=="SixPointSequence"){
		Constant_joint=1;
		nP = 6;
		idxPoint=0;
		pointSequence.Resize(nP, KUKA_DOF);
		backSequence.Resize(nP, KUKA_DOF);

		pointSequence.Set(*SixPointSequence, nP, KUKA_DOF);
		pointSequence.Mult((PI/180.0), pointSequence);

		backSequence.Set(*SixPointSequence, nP, KUKA_DOF);
		backSequence.Mult((PI/180.0), backSequence);

		//Start the control chain
		mPlanner = PLANNER_JOINT;
		mCommand = COMMAND_Wait4Go;
		cout<<"switched to command wait4go"<<endl;

	}
	if(cmd=="ThreePointSequence"){
			Constant_joint=1;
			nP = 3;
			idxPoint=0;
			pointSequence.Resize(nP, KUKA_DOF);
			backSequence.Resize(nP, KUKA_DOF);

			pointSequence.Set(*ThreePointSequence, nP, KUKA_DOF);
			pointSequence.Mult((PI/180.0), pointSequence);

			backSequence.Set(*ThreePointSequence, nP, KUKA_DOF);
			backSequence.Mult((PI/180.0), backSequence);


			//Start the control chain
			mPlanner = PLANNER_JOINT;
			mCommand = COMMAND_Wait4Go;
			cout<<"switched to command wait4go"<<endl;

		}
	if(cmd=="FourPointSequence"){
				Constant_joint=1;
				nP = 4;
				idxPoint=0;
				pointSequence.Resize(nP, KUKA_DOF);
				backSequence.Resize(nP, KUKA_DOF);

				pointSequence.Set(*FourPointSequence, nP, KUKA_DOF);
				pointSequence.Mult((PI/180.0), pointSequence);

				backSequence.Set(*FourPointSequence, nP, KUKA_DOF);
				backSequence.Mult((PI/180.0), backSequence);


				//Start the control chain
				mPlanner = PLANNER_JOINT;
				mCommand = COMMAND_Wait4Go;
				cout<<"switched to command wait4go"<<endl;

			}
	if(cmd=="FivePointSequence"){
				Constant_joint=1;
				nP = 5;
				idxPoint=0;
				pointSequence.Resize(nP, KUKA_DOF);
				backSequence.Resize(nP, KUKA_DOF);

				pointSequence.Set(*FivePointSequence, nP, KUKA_DOF);
				pointSequence.Mult((PI/180.0), pointSequence);

				backSequence.Set(*FivePointSequence, nP, KUKA_DOF);
				backSequence.Mult((PI/180.0), backSequence);


				//Start the control chain
				mPlanner = PLANNER_JOINT;
				mCommand = COMMAND_Wait4Go;
				cout<<"switched to command wait4go"<<endl;

			}
	if(cmd=="chair"){
					Constant_joint=1;
					nP = 2;
					idxPoint=0;
					pointSequence.Resize(nP, KUKA_DOF);
					backSequence.Resize(nP, KUKA_DOF);

					pointSequence.Set(*chair, nP, KUKA_DOF);
					pointSequence.Mult((PI/180.0), pointSequence);

					backSequence.Set(*chair, nP, KUKA_DOF);
					backSequence.Mult((PI/180.0), backSequence);


					//Start the control chain
					mPlanner = PLANNER_JOINT;
					mCommand = COMMAND_Wait4Go;
					cout<<"switched to command wait4go"<<endl;

				}

	return STATUS_OK;
}

void audio_callback(void *user_data, Uint8 *raw_buffer, int bytes)
{
    Sint16 *buffer = (Sint16*)raw_buffer;
    int length = bytes / 2; // 2 bytes per sample for AUDIO_S16SYS
    int &sample_nr(*(int*)user_data);

    for(int i = 0; i < length; i++, sample_nr++)
    {
        double time = (double)sample_nr / (double)SAMPLE_RATE;
        buffer[i] = (Sint16)(AMPLITUDE * sin(2.0f * M_PI * 441.0f * time)); // render 441 HZ sine wave
    }
}

void playToneGo(){


    if(SDL_Init(SDL_INIT_AUDIO) != 0) SDL_Log("Failed to initialize SDL: %s", SDL_GetError());

    int sample_nr = 0;

    SDL_AudioSpec want;
    want.freq = SAMPLE_RATE; // number of samples per second
    want.format = AUDIO_S16SYS; // sample type (here: signed short i.e. 16 bit)
    want.channels = 1; // only one channel
    want.samples = 2048; // buffer-size
    want.callback = audio_callback; // function SDL calls periodically to refill the buffer
    want.userdata = &sample_nr; // counter, keeping track of current sample number

    SDL_AudioSpec have;
    if(SDL_OpenAudio(&want, &have) != 0) SDL_LogError(SDL_LOG_CATEGORY_AUDIO, "Failed to open audio: %s", SDL_GetError());
    if(want.format != have.format) SDL_LogError(SDL_LOG_CATEGORY_AUDIO, "Failed to get the desired AudioSpec");

    SDL_PauseAudio(0); // start playing sound
    SDL_Delay(200); // wait while sound is playing
    SDL_PauseAudio(1); // stop playing sound

    SDL_CloseAudio();

}
void playToneStart(){



    if(SDL_Init(SDL_INIT_AUDIO) != 0) SDL_Log("Failed to initialize SDL: %s", SDL_GetError());

    int sample_nr = 0;

    SDL_AudioSpec want;
    want.freq = SAMPLE_RATE_2; // number of samples per second
    want.format = AUDIO_S16SYS; // sample type (here: signed short i.e. 16 bit)
    want.channels = 1; // only one channel
    want.samples = 2048; // buffer-size
    want.callback = audio_callback; // function SDL calls periodically to refill the buffer
    want.userdata = &sample_nr; // counter, keeping track of current sample number

    SDL_AudioSpec have;
    if(SDL_OpenAudio(&want, &have) != 0) SDL_LogError(SDL_LOG_CATEGORY_AUDIO, "Failed to open audio: %s", SDL_GetError());
    if(want.format != have.format) SDL_LogError(SDL_LOG_CATEGORY_AUDIO, "Failed to get the desired AudioSpec");

    SDL_PauseAudio(1); // start playing sound
    SDL_Delay(200); // wait while sound is playing
    SDL_PauseAudio(1); // stop playing sound

    SDL_CloseAudio();

}



extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    monkeytask_arb_data* create(){return new monkeytask_arb_data();}
    void destroy(monkeytask_arb_data* module){delete module;}
}

