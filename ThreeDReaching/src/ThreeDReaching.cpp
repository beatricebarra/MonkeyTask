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

#include "ThreeDReaching.h"

//Global variables
bool closed_loop=true;
bool True_robot=true;
bool Position_of_the_robot_recieved=false;


//For audio
const int AMPLITUDE = 28000;
const int SAMPLE_RATE = 44100;
const int SAMPLE_RATE_2 = 22050;
void audio_callback(void *user_data, Uint8 *raw_buffer, int bytes);
void playToneGo();
void playToneStart();
int generatePosition(int nPositions);
// for communicating with matlab
void writeVisualizer(int *sockfd, float *writeForcePos, struct sockaddr_in *cli_adds, socklen_t *cli_len);


using namespace std;

ThreeDReaching::ThreeDReaching()
:RobotInterface(){
}
ThreeDReaching::~ThreeDReaching(){
}

void ThreeDReaching::chatterCallback_position(const sensor_msgs::JointState & msg)
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

void ThreeDReaching::Send_Postion_To_Robot(Vector Position)
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

RobotInterface::Status ThreeDReaching::RobotInit(){

	// Resizing the vectors for my number of joints

	BackPosition.Resize(KUKA_DOF);

	cJointTargetPos.Resize(KUKA_DOF);
	cJointTargetVel.Resize(KUKA_DOF);
	fJointTargetPos.Resize(KUKA_DOF);
	fCartTargetPos.Resize(3);
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
	AddConsoleCommand("Cube");
	AddConsoleCommand("cylCenterPos");



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
	maxeeForceInt = 255;
	mineeForceInt = 14;
	maxeeForceDouble = 30;// it was 12 before
	mineeForceDouble = 0;

	// Velocity an stiffness

	timeout = 10000.0;
	pullThreshold = 0.08;
	targetThreshold = 0.025;
	targetThresholdCart = 0.01;



	// PointSequence Variables initialization
	idxPoint = 0;
	badTrial = 0;
	failureThreshold = 3;

	mRobot->SetControlMode(Robot::CTRLMODE_POSITION);
	ros::NodeHandle *n = mRobot->InitializeROS();
	if (True_robot)
	{
		pub_command_robot_real =  n->advertise<kuka_fri_bridge::JointStateImpedance>("/real_r_arm_controller/joint_imp_cmd", 3);
		sub_position_robot = n->subscribe("/real_r_arm_pos_controller/joint_states", 3, & ThreeDReaching::chatterCallback_position,this);
	}

	// Flags
	isfirsttrial = 1;


	return STATUS_OK;
}
RobotInterface::Status ThreeDReaching::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status ThreeDReaching::RobotStart(){


	// Setting the position of all joints to 0
	cJointPos.Zero();

	// This differentiate the behaviour on the real robot or just on the simulator
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


	//Writing the joint position in the kinematic chain
	mSKinematicChain->setJoints(cJointPos.Array());
	// Output of the joint values in the simulator console
	cJointPos.Print("cJointPos");
	// Target position = current position
	cJointTargetPos = cJointPos;
	// Null command and planner are set
	mCommand=COMMAND_Wait4Go;
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




	//////////////////////////////////////////////////////////////////////////////
	// Open communication with Matlab-NI board////////////////////////////////////////
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
    //cout<<myfile.is_open()<<endl;
    // Writing labels
    myfile<<"Fx"<<"," <<"Fy"<<","<<"Fz"<<","<<"Trigger"<<","<<"Time"<<","<<"PositionID"<<","<<"px"<<","<<"py"<<","<<"pz"<<endl;
	// Define settings for curses I/O
	initscr();
	cbreak();
	//noecho();
	nodelay(stdscr, TRUE);


	cout<<"End of Robot Start"<<endl;
    return STATUS_OK;
}    
RobotInterface::Status ThreeDReaching::RobotStop(){
	// Resetting options for the terminal
	endwin();
    return STATUS_OK;
}
RobotInterface::Status ThreeDReaching::RobotUpdate(){
	ros::spinOnce();

	//Local variables
	float distance2target;
	float distance2target_x;// just on the x axis


	// Setting the joints to values currently stored in mJointPosAll
	mSKinematicChain->setJoints(cJointPos.Array());
	//Saving target point position and relative axis direction in the correspondent 
	//attribute in the KinematicChain object
	mSKinematicChain->getEndPos(cCartPos.Array());
	mSKinematicChain->getEndDirAxis(AXIS_X, cCartDirX.Array());
	mSKinematicChain->getEndDirAxis(AXIS_Y, cCartDirY.Array());
	mSKinematicChain->getEndDirAxis(AXIS_Z, cCartDirZ.Array());
	

	//Setting different planners for different commands
	// Different Commands : 
	// COMMAND_2Position: Going to commanded position joint position controlspace
	// COMMAND_spring : impedance control phase, the robot behaves like a spring
	// COMMAND_Wait4Go : The robot stays rigid, waiting for the experimenter command to move to the next position
	switch(mCommand){

	case COMMAND_2Position : 
		mPlanner = PLANNER_JOINT;
		break;

	case COMMAND_spring :
		
		
		// Move all of this in the Going to Position, in the case  I arrive to the position
		// Writing joints
		mSKinematicChain->setJoints(cJointPos.Array());
		// Reading end-effector pose and orientation
		mSKinematicChain->getEndPos(fCartTargetPos.Array());
		mSKinematicChain->getEndPos(ffCartTargetPos.Array());
		mSKinematicChain->getEndDirAxis(AXIS_Y, cCartTargetDirY.Array());
		mSKinematicChain->getEndDirAxis(AXIS_Z, cCartTargetDirZ.Array());
		
		// Setting dynamical system parameters
		Stiffness=Stiffness1;
		Damping=Damping1;
		gain_ori=gain_ori1;
		// Setting planner and command
		mPlanner = PLANNER_CARTESIAN;//Add termination condition in planner
		mCommand = NONE_comand;

		// Writing on arduino: this command is then translated in 
		// 1) The green light turning on
		// 2) The analog trigger goin high
//		char mywrite[1];
//		mywrite[0]= 1;
//		size = write(arduinoFD, &mywrite, sizeof(mywrite));

		// Play a tone at 440 Hz
		//playToneGo();
		break;

	case COMMAND_Wait4Go:
			mPlanner = PLANNER_JOINT;
			break;
	}


	// Current time is acquired everytime
	time_t t = time(0);
	struct tm *now = localtime(&t);
	char thisInstant[50];
	strftime(thisInstant, sizeof(thisInstant), "%X", now);




	// Alternative approach
	gettimeofday(&currentTime, NULL);
	systemSeconds  = currentTime.tv_sec  - t0.tv_sec;
	systemUseconds = currentTime.tv_usec - t0.tv_usec;

	 //Force modulus
//	eeForce = mJacobian3*cJointTORs;
//	//eeForceMod = eeForce.Norm();
//	eeForceMod = eeForce.Norm();
//	eeForceModInt = floor(eeForceMod*(maxeeForceInt-mineeForceInt)/(maxeeForceDouble-mineeForceDouble) - mineeForceDouble + mineeForceInt);
//	write(arduinoFD, &eeForceModInt, 1);



	// Definition and implementation of type of movement. There are two possible types
	// PLANNER_CARTESIAN: impedance control mode, done solving the dynamicaal system in terms 
	// 					of velocities and accelerations inverting the kinematics in joint space, 
	//					and therefore commanding joint positions.
	// PLANNER_JOINT: control is done directly in joint space and with the kinematic chain 
	//					the position is continuosly updated (controlla)
	switch(mPlanner){
	
	case PLANNER_CARTESIAN:

		// Time scale of the loop:
		// p --> t-1
		// c --> t
		// d --> t+1

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
		eeForce = mJacobian3*cJointTORs; // WRONG
		eeForceMod = eeForce.Norm();
		eeForceModInt = floor(eeForceMod*(maxeeForceInt-mineeForceInt)/(maxeeForceDouble-mineeForceDouble) - mineeForceDouble + mineeForceInt);

		// Writing to arduino
		write(arduinoFD, &eeForceModInt, 1);
		

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

		// Writing filtered velocity
		cCartVel.SetSubVector(0, copyVector);

		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%% Dynamic system %%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		// Desired acceleration (== force)

		cCartTargetAcc.SetSubVector(0, (cCartPos-fCartTargetPos)*Stiffness + cCartTargetVel.GetSubVector(0,3)*Damping);
		// Acceleration on the z axis is prevented
		cCartTargetAcc(2)=(cCartPos(2)-fCartTargetPos(2))*Stiffness*25 + cCartTargetVel(2)*Damping*5;
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
		// Writing force to file: we are in impedance phase: it is a good idea
		myfile << eeForce[0] <<","<< eeForce[1] << "," << eeForce[2] << "," ;
		myfile << cJointTORs[0] <<","<< cJointTORs[1] << "," << cJointTORs[2]<< "," <<cJointTORs[3]<< ","<<cJointTORs[4]<< ","<<cJointTORs[5]<< ","<<cJointTORs[6]<< ",";
		myfile << mJacobian3.GetRow(0)<<"," <<mJacobian3.GetRow(1)<<","<<mJacobian3.GetRow(0)<<",";
		myfile << cJointPos[0] <<","<< cJointPos[1] << "," << cJointPos[2]<< "," <<cJointPos[3]<< ","<<cJointPos[4]<< ","<<cJointPos[5]<< ","<<cJointPos[6]<<",";
		myfile<< 2 << "," ;
		myfile<< systemSeconds << "," << systemUseconds << ","<< idxPoint << ",";
		myfile<<cCartPos[0]<<"," <<cCartPos[1]<<","<<cCartPos[2]<<endl;

		// Check joint Pos

		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%% Check for distance condition %%%%%%%%%%%%%%%%
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		// Distance on x axis
		distance2target_x = ffCartTargetPos[0]-cCartPos[0];
		//cout << distance2target_x << endl;
		
		// Writing also to NI board
		writeForcePos[3] = eeForce[0];
		writeForcePos[4] = eeForce[1];
		writeForcePos[5] = eeForce[2];
		writeForcePos[6] = distance2target_x;
		writeVisualizer(&sockfd, writeForcePos, &cli_adds, &cli_len); // update status here

		// Condition for going to next position 
		// - The robot has been pulled of 10 cm OR 00 seconds elapsed without any accomplishment from the monkey
		if((distance2target_x)> pullThreshold){
			//cout<<nP<<endl;

			//Writing a 2 on the arduino it would send a analog trigger out to the VICON
			char mywrite[1];
			mywrite[0]= 2;
			size = write(arduinoFD, &mywrite, sizeof(mywrite));

			idxPoint = generatePosition(nP)-1;
			mCommand = COMMAND_2Position;
			fJointTargetPos = pointSequence.GetRow(idxPoint, fJointTargetPos); 
			//fCartTargetPos = pointSequenceCart.GetRow(idxPoint, fCartTargetPos);
			cout<<"Success"<<endl;
		}
		else {
			// If the object has not been pulled beyond the threshold...
			char readBytes[1];
			char trashBytes[5];
			//size = read(arduinoFD, &readBytes, 1);

			//if(ros::Time::now().toSec()-secs>timeout || readBytes[0] == 'N'){
			if(ros::Time::now().toSec()-secs>timeout ){//|| getch() == 'n'

				// and the available to complete the task is over OR the experimenter pushed the button...
				// ..The trial is counted as a bad trial
				badTrial = badTrial + 1;
				if (badTrial >=failureThreshold) {
					// If the monkey did not do the trials correctly for three times
					// the target position is updated anyway
					idxPoint = generatePosition(nP)-1;
					mCommand = COMMAND_2Position;
					badTrial = 0;
				}
				fJointTargetPos = pointSequence.GetRow(idxPoint, fJointTargetPos); 
				//fCartTargetPos = pointSequenceCart.GetRow(idxPoint, fCartTargetPos);
				mCommand = COMMAND_2Position;

			}
			else if(getch() == 'n'){
				idxPoint = generatePosition(nP)-1;
				fJointTargetPos = pointSequence.GetRow(idxPoint, fJointTargetPos);
				//fCartTargetPos = pointSequenceCart.GetRow(idxPoint, fCartTargetPos);
				mCommand = COMMAND_2Position;


			}
		}


	break;// Closing PLANNER CARTESIAN

	case PLANNER_JOINT:
		// If the Planner is held in the joint space joint values are directly updated

		// Target position in joint space
		//home.Zero();
		//cout<<fJointTargetPos<<endl;
		InitialDistance.Resize(KUKA_DOF);
		InitialDistance= (fJointTargetPos-home);
		CurrentDistance.Resize(KUKA_DOF);
		CurrentDistance = (fJointTargetPos-cJointPos);
		double k_v;
		double maxSat = 90;//100;//120;//100;
		double minSat = 40;//70;
		double minROM = 0.1;
		double maxROM = 0.9;


		if (CurrentDistance.Norm()/InitialDistance.Norm()<minROM)
			k_v = maxSat;
		else if (CurrentDistance.Norm()/InitialDistance.Norm()>maxROM)
			k_v = minSat;
		else
			k_v = (minSat-maxSat)/(maxROM-minROM)*CurrentDistance.Norm()/InitialDistance.Norm()+ maxSat;

		cJointTargetPos = cJointPos + (fJointTargetPos-cJointPos)*_dt*k_v;//(InitialDistance.Norm()/CurrentDistance.Norm());//*Constant_joint;
		// Reading end effector pose
		mSKinematicChain->getEndPos(cCartPos.Array());
		// Force modulus
		eeForce = mJacobian3.Transpose()*cJointTORs;// Right
		eeForceMod = eeForce.Norm();
		// Writing also to NI board
		writeForcePos[3] = eeForce[0];
		writeForcePos[4] = eeForce[1];
		writeForcePos[5] = eeForce[2];
		writeForcePos[6] = 0;

		writeVisualizer(&sockfd, writeForcePos, &cli_adds, &cli_len); // update status here

		switch (mCommand){

		case COMMAND_2Position:

			// Writing force to file: we are goin to position phase: it is a good idea
			myfile << eeForce[0] <<","<< eeForce[1] << "," << eeForce[2] << "," ;
			myfile << cJointTORs[0] <<","<< cJointTORs[1] << "," << cJointTORs[2]<< "," <<cJointTORs[3]<< ","<<cJointTORs[4]<< ","<<cJointTORs[5]<< ","<<cJointTORs[6]<< ",";
			myfile << mJacobian3.GetRow(0)<<"," <<mJacobian3.GetRow(1)<<","<<mJacobian3.GetRow(0)<<",";
			myfile << cJointPos[0] <<","<< cJointPos[1] << "," << cJointPos[2]<< "," <<cJointPos[3]<< ","<<cJointPos[4]<< ","<<cJointPos[5]<< ","<<cJointPos[6]<<",";
			myfile<< 1 << "," ;
			myfile<< systemSeconds << "," << systemUseconds << ","<< idxPoint << ",";
			myfile<<cCartPos[0]<<"," <<cCartPos[1]<<","<<cCartPos[2]<<endl;

			// Going to target point
			J_distance2P0 = cJointTargetPos-fJointTargetPos;
			C_distance2P0  = (cCartPos(1)- fCartTargetPos(1))+(cCartPos(2)- fCartTargetPos(1))+(cCartPos(3)- fCartTargetPos(3));



			if (J_distance2P0.Norm() < targetThreshold){//was 0.016 //if (abs(J_distance2P0[0])*(180.0/PI) < 1 && abs(J_distance2P0[1])*(180.0/PI)<1 && abs(J_distance2P0[2])*(180.0/PI) < 1 && abs(J_distance2P0[3])*(180.0/PI)<1 && abs(J_distance2P0[4])*(180.0/PI) < 1 && abs(J_distance2P0[5])*(180.0/PI)<1 && abs(J_distance2P0[6])*(180.0/PI) < 1){//if (J_distance2P0.Norm() < 0.09)
			//if (C_distance2P0.Norm() < targetThresholdCart){
				if (userCommand == 'h'){// It should not exist this if /else
					mCommand = COMMAND_Wait4Go;
				}
				else{
					secs =ros::Time::now().toSec();
					cJointTargetPos=cJointPos;
					// Set Joints
					mSKinematicChain->setJoints(cJointPos.Array());
					// Reading end-effector pose and orientation
					mSKinematicChain->getEndPos(fCartTargetPos.Array());
					mSKinematicChain->getEndPos(cCartTargetPos.Array());
					mSKinematicChain->getEndDirAxis(AXIS_Y, cCartTargetDirY.Array());
					mSKinematicChain->getEndDirAxis(AXIS_Z, cCartTargetDirZ.Array());
					mCommand = COMMAND_Wait4Go;
					// I write the current index of the point to the VICON
					char mywrite[1];
					modidxPoint = idxPoint+4;
					size = write(arduinoFD, &modidxPoint, sizeof(mywrite));
				}


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
			//change
			userCommand=ERR;
				//size = read(arduinoFD, &readBytes, buffersize);
				// Alternative based on keyboard
				//scanf("%c", &readBytes);
			while(userCommand==ERR){
				userCommand = getch();
				//if (readBytes[0]=='N'){//readBytes[0]// Based on arduino
				if (userCommand=='g'){

					if (isfirsttrial){// The first time has to move from home to the position
						// I set the point here
						fJointTargetPos = pointSequence.GetRow(idxPoint, fJointTargetPos); 
						mCommand = COMMAND_2Position; // just did these 2 lines
						isfirsttrial=0;
					}
					else {
						cout<<"I pressed g"<<endl;
						// Play tone and enter in impedance mode
						// Play a tone at 440 Hz
						playToneGo();
						// Writing on arduino: this command is then translated in
						// 1) The green light turning on
						// 2) The analog trigger goin high
						char mywrite[1];
						mywrite[0]= 1;
						size = write(arduinoFD, &mywrite, sizeof(mywrite));

						mCommand=COMMAND_spring;
						char trash[5];
						size = read(arduinoFD, &trash, 5);
						
					}
					readFlag = 0;
					// Sending output to Arduino--> check if number is correct
					//char mywrite[1];
					//size = write(arduinoFD, &modidxPoint, sizeof(mywrite));
					// Updating the index so that it is possible to output it in VICON
					modidxPoint = idxPoint+4;
				}
				else if (userCommand=='n'){
					cout<<"Next Position"<<endl;

					idxPoint = generatePosition(nP)-1;
					mCommand = COMMAND_2Position;
					mPlanner= PLANNER_JOINT;
					fJointTargetPos = pointSequence.GetRow(idxPoint, fJointTargetPos);
					//fCartTargetPos = pointSequenceCart.GetRow(idxPoint, fCartTargetPos);
					cout<<idxPoint<<"\n"<<endl;
					//cout<<fJointTargetPos<<endl;
					readFlag = 0;
				}
				else if (userCommand=='h'){
					cout<<"Going home\n"<<endl;
					//cout<<fJointTargetPos<<endl;
					idxPoint = generatePosition(nP)-1;
					mCommand = COMMAND_2Position;
					mPlanner= PLANNER_JOINT;
					home.Zero();
					fJointTargetPos = home;
					//fCartTargetPos = pointSequenceCart.GetRow(idxPoint, fCartTargetPos);
					readFlag = 0;
				}
				else if (userCommand=='d'){
					cout<<"Going down\n"<<endl;

					idxPoint = generatePosition(nP)-1;
					mCommand = COMMAND_2Position;
					mPlanner= PLANNER_JOINT;
					fJointTargetPos = pointSequence.GetRow(idxPoint, fJointTargetPos);
					fCartTargetPos = pointSequenceCart.GetRow(idxPoint, fCartTargetPos);

					//fCartTargetPos = pointSequenceCart.GetRow(idxPoint, fCartTargetPos);
					readFlag = 0;
				}
			}
			break;
		} // end switch on planner commands

	break; // closing planner joint

	} // end switch on planner

    return STATUS_OK;
}
RobotInterface::Status ThreeDReaching::RobotUpdateCore(){
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
int ThreeDReaching::RespondToConsoleCommand(const string cmd, const vector<string> &args){

	cout<<"Write your command\n"<<endl;
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
		cout<<"switched to command wait4go\n"<<endl;

	}
	if(cmd=="cylCenterPos"){
			Constant_joint=1;
			nP = 10;
			idxPoint=0;
			pointSequence.Resize(nP, KUKA_DOF);
			backSequence.Resize(nP, KUKA_DOF);

			pointSequence.Set(*cylCenterPos, nP, KUKA_DOF);
			pointSequence.Mult((PI/180.0), pointSequence);

			backSequence.Set(*cylCenterPos, nP, KUKA_DOF);
			backSequence.Mult((PI/180.0), backSequence);

			//Start the control chain
			mPlanner = PLANNER_JOINT;
			mCommand = COMMAND_Wait4Go;
			cout<<"switched to command wait4go\n"<<endl;

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

			pointSequenceCart.Set(*ThreePointSequenceCart, nP, 3);

			backSequence.Set(*ThreePointSequence, nP, KUKA_DOF);
			backSequence.Mult((PI/180.0), backSequence);


			//Start the control chain
			mPlanner = PLANNER_JOINT;
			mCommand = COMMAND_Wait4Go;
			cout<<"switched to command wait4go"<<endl;

		}

	if(cmd=="Cube"){
				Constant_joint=1;
				nP = 13;
				idxPoint=0;
				pointSequence.Resize(nP, KUKA_DOF);
				backSequence.Resize(nP, KUKA_DOF);

				pointSequence.Set(*Cube, nP, KUKA_DOF);
				pointSequence.Mult((PI/180.0), pointSequence);

				//pointSequenceCart.Set(*Cube, nP, 3);

				backSequence.Set(*Cube, nP, KUKA_DOF);
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

int generatePosition(int nPositions){
	int nextPosition;
	nextPosition = floor(1 + (rand()/(double)RAND_MAX*(double)(nPositions) ));
	return nextPosition;
}

void writeVisualizer(int *sockfd, float *writeForcePos, struct sockaddr_in *cli_adds, socklen_t *cli_len) {
//float writeForcePos[7]; // 3d pos, 3d force, 1d rel_pos

	writeForcePos[6] = (float)(writeForcePos[6]*1000); // writeForcePos[6] is meters, thus convert to mm here

	//cout << "relative position " << writeForcePos[6] << endl;

	int succ;

	succ = sendto(*sockfd, writeForcePos, 28, MSG_DONTWAIT, (struct sockaddr *)cli_adds, *cli_len); //

}
extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    ThreeDReaching* create(){return new ThreeDReaching();}
    void destroy(ThreeDReaching* module){delete module;}
}

