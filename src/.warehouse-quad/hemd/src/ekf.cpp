#include "kalman.h"

using namespace std;

namespace HMDETECTION{

EKF::EKF():currentImutime(0.0), previousImutime(0.0), imuState(WAITING), magState(WAITING), countHeight(0), imuSeq(0), magSeq(0), ekfCurrentState(EKF_WAITING),X(19), X_(19), sigma(19,19), sigma_(19,19), sonarState(WAITING){
}

void EKF::subscriber(){

	ros::NodeHandle nh_;
	ros::Subscriber sub1 = nh_.subscribe("/mavros/imu/data_raw",100, &EKF::imuCallback, this);
	ros::Subscriber sub2 = nh_.subscribe("/mavros/imu/mag",100, &EKF::magCallback, this);
	ros::Subscriber sub4 = nh_.subscribe("/px4flow/opt_flow",10, &EKF::sonarCallback, this);
    ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &EKF::stateCallback, this);

	anglesPub = nh_.advertise<geometry_msgs::Vector3Stamped>("/ekf/rpy",10);
	posePub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",10);
	setpointPub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
	ros::spin();
//	ros::Rate r(80);
//	while(ros::ok()) {
//		r.sleep();
//	}
}


void EKF::stateCallback(const mavros_msgs::State::ConstPtr& msg){

	geometry_msgs::PoseStamped setPoint;
	setPoint.header.stamp = ros::Time::now();
	setPoint.header.frame_id = "quad";

	if(msg->mode !="OFFBOARD"){
		height = X(15);
	}

	setPoint.pose.position.x = 0;
	setPoint.pose.position.y = 0;
	setPoint.pose.position.z = height;

	//setpointPub.publish(setPoint);

}

void EKF::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

	accelbf << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	omegabf << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
	currentImutime = msg->header.stamp.toSec();

	time = msg->header.stamp;
	deltaImutime = currentImutime-previousImutime;
	previousImutime = currentImutime;

	if(imuState==WAITING){
		imuState = INITIALIZED;
		return;
	}
	else if(imuState==INITIALIZED && imuSeq!=msg->header.seq){
		imuState = ACQUIRED;
		imuSeq++;
	}
	else if(imuState==ACQUIRED && imuSeq==msg->header.seq){
		imuState = INITIALIZED;
		return;
	}


	if(imuState==ACQUIRED){

		if(ekfCurrentState==EKF_WAITING&&magState==ACQUIRED){
			ekfInitialize();
			initialtime = msg->header.stamp.toSec();
			ekfCurrentState = EKF_INITIALIZED;
			//return;
		}

		if(ekfCurrentState==EKF_INITIALIZED){
			ekfPrediction();
		}
		imuState = INITIALIZED;
	}


}

void EKF::magCallback(const sensor_msgs::MagneticField::ConstPtr& msg){

	//acquire mag data
	magbf << msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z;
	magbf.normalize();

	if(magState==WAITING){
		magState=INITIALIZED;
	}
	else if(magState==INITIALIZED && magSeq!=msg->header.seq){
		magState=ACQUIRED;
		magSeq++;
	}
	else if(magState==ACQUIRED && magSeq==msg->header.seq){
		magState=INITIALIZED;
	}


	if(ekfCurrentState==EKF_INITIALIZED){
		ekfUpdate(); //ekf update step
		if(sonarState==INITIALIZED){
			ekfUpdateHeight(sonarRaw);
		}
	}
}

void EKF::sonarCallback(const px_comm::OpticalFlow::ConstPtr& msg){

	double sonarDistance = sonarVal.getSonarFilteredData(msg->ground_distance);

	int goal =1;
	if (sonarDistance>5){
		goal =0;
	}
	//ROS_INFO("OK");

	if(sonarDistance>0.2 && sonarDistance<5){

		if(sonarState==WAITING){
			sonarState=INITIALIZED;
			return;
		}
		sonarRaw = msg->ground_distance;
		//ROS_INFO("OK1");
	}
}

void EKF::ekfInitialize(){
	magbf.normalize();
	X << 1, 0, 0, 0, magbf(0), magbf(1), magbf(2), 0, 0, 0, 0, 0, 0, 0, 0, 0.3, 0, 0, 0;
	sigma = MatrixXd::Identity(19,19);
	sigma.block<4,4>(0,0) = MatrixXd::Identity(4,4);
	sigma.block<3,3>(4,4) = 0.2*MatrixXd::Identity(3,3);
	sigma.block<3,3>(7,7) = 0*MatrixXd::Identity(3,3);
	sigma(15,15) = 5;
	sigma(18,18) = 20;
}

void EKF::ekfPrediction(){
	X_ = X; //copy the previous state to predicted state for mag and bias variables

	//quaternion prediction
	double q0, q1, q2, q3;
	q0 = X(0);
	q1 = X(1);
	q2 = X(2);
	q3 = X(3);

	//q matrix for performing quaternion multiplication
	MatrixXd q(4,3);
	q << -q1, -q2, -q3,
		  q0, -q3,  q2,
		  q3,  q0, -q1,
		 -q2,  q1,  q0;

	//update the quaternion
	X_.head(4) = X.head(4) + deltaImutime*0.5*q*(omegabf);

	//normalize the quaternion
	float norm = sqrt(X_(0)*X_(0)+X_(1)*X_(1)+X_(2)*X_(2)+X_(3)*X_(3));
	X_.head(4) = X_.head(4)/norm;

	double wx, wy, wz;
	wx = omegabf(0);//-X(7);
	wy = omegabf(1);//-X(8);
	wz = omegabf(2);//-X(9);

	double bwx, bwy, bwz;
	bwx = X(7);
	bwy = X(8);
	bwz = X(9);

	Quaterniond quat(q0, q1, q2, q3);
	quat.normalize();

	Vector3d accelef;
	accelef = quat.toRotationMatrix() * accelbf;
	if(accelef(2)>5){
		accelef(2) = accelef(2)-9.81;
	}
	else
		accelef(2) = accelef(2);

	geometry_msgs::Vector3Stamped accel_ef;
	accel_ef.vector.x = accelef(0);
	accel_ef.vector.y = accelef(1);
	accel_ef.vector.z = accelef(2);
	accel_ef.header.stamp = ros::Time::now();

	/*if(lidarState!=WAITING){
		X_.segment<2>(13) = X.segment<2>(13) + X.segment<2>(16) * deltaImutime;
		X_.segment<2>(16) = X.segment<2>(16) + accelef.segment<2>(0)*deltaImutime;
	}*/

	if(sonarState!=WAITING){
		X_(15) = X(15) + X(18)*deltaImutime;
		X_(18) = X(18) + accelef(2) * deltaImutime;
	}

	MatrixXd J(19, 19);
	J << 	 1, 				  -deltaImutime*wx*0.5, -deltaImutime*wy*0.5, -deltaImutime*wz*0.5, 0, 0, 0,  /*deltaImutime*q1*0.5,  deltaImutime*q2*0.5,  deltaImutime*q3*0.5*/0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			 deltaImutime*wx*0.5,  1, 	     			 deltaImutime*wz*0.5, -deltaImutime*wy*0.5, 0, 0, 0, /*-deltaImutime*q0*0.5,  deltaImutime*q3*0.5, -deltaImutime*q2*0.5*/0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			 deltaImutime*wy*0.5, -deltaImutime*wz*0.5,  1,   		 		   deltaImutime*wx*0.5, 0, 0, 0, /*-deltaImutime*q3*0.5, -deltaImutime*q0*0.5,  deltaImutime*q1*0.5*/0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			 deltaImutime*wz*0.5,  deltaImutime*wy*0.5, -deltaImutime*wx*0.5,  1,         			0, 0, 0,  /*deltaImutime*q2*0.5, -deltaImutime*q1*0.5, -deltaImutime*q0*0.5*/0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			 0,         			0,          		 0,          		   0,         			1, 0, 0,  0, 		  			0, 		  			  0, 		 		   0, 0, 0, 0, 0, 0, 0, 0, 0,
			 0,          			0,          		 0,          		   0,         			0, 1, 0,  0,		  			0,		  			  0,		 		   0, 0, 0, 0, 0, 0, 0, 0, 0,
			 0,          			0,          		 0,          		   0,         			0, 0, 1,  0,		  			0,		  		 	  0,		 		   0, 0, 0, 0, 0, 0, 0, 0, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  1,		  			0, 		  			  0,		 		   0, 0, 0, 0, 0, 0, 0, 0, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			1, 		  			  0, 		 		   0, 0, 0, 0, 0, 0, 0, 0, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			0, 		  			  1, 		 		   0, 0, 0, 0, 0, 0, 0, 0, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			0, 		  			  0, 		 		   1, 0, 0, 0, 0, 0, 0, 0, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			0, 		  			  0, 		 		   0, 1, 0, 0, 0, 0, 0, 0, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			0, 		  			  0, 		 		   0, 0, 1, 0, 0, 0, 0, 0, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			0, 		 			  0, 		 		   0, 0, 0, 1, 0, 0, deltaImutime, 0, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			0, 		  			  0, 		 		   0, 0, 0, 0, 1, 0, 0, deltaImutime, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			0, 		  			  0, 		 		   0, 0, 0, 0, 0, 1, 0, 0, deltaImutime,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			0, 		  			  0, 		 		   0, 0, 0, 0, 0, 0, 1, 0, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			0, 		  			  0, 		 		   0, 0, 0, 0, 0, 0, 0, 1, 0,
			 0,			 			0,			 		 0,			 		   0,					0, 0, 0,  0, 		  			0, 		  			  0, 		 		   0, 0, 0, 0, 0, 0, 0, 0, 1;

	MatrixXd Q(19,19);
	Q = MatrixXd::Zero(19,19);

	Q.block<4,4>(0,0) = 0.00001*MatrixXd::Identity(4,4);
	Q.block<3,3>(7,7) = 0.001*MatrixXd::Identity(3,3);
	/*if(lidarState!=WAITING){
		Q.block<2,2>(16,16) = deltaImutime*deltaImutime*2*MatrixXd::Identity(2,2);
	}*/

	if(sonarState!=WAITING){
		Q(18,18) = deltaImutime*deltaImutime;
		Q(15,15) = 0.08;

	}
	//Q(18,18) = 0.008;

	sigma_ = J*sigma*J.transpose() + Q;
}

void EKF::ekfUpdate(){

	double q0, q1, q2, q3;
	q0 = X_(0);
	q1 = X_(1);
	q2 = X_(2);
	q3 = X_(3);

	float mx, my, mz;
	mx = magbf(0);
	my = magbf(1);
	mz = magbf(2);

	VectorXd z_(6);
	MatrixXd H(6,19);

	double hx, hy, bx, bz, by;
	hx = mx * q0*q0 - 2*q0*my * q3 + 2*q0*mz * q2 + mx * q1*q1 + 2*q1 * my*q2 + 2*q1*mz * q3 - mx*q2*q2 - mx * q3*q3;
	hy = 2*q0*mx * q3 + my * q0*q0 - 2*q0*mz * q1 + 2*q1*mx * q2 - my * q1*q1 + my * q2*q2 + 2*q2 * mz * q3 - my * q3*q3;


	bx = sqrt(hx * hx + hy * hy);
	bz = -2*q0*mx * q2 + 2*q0*my * q1 + mz * q0*q0 + 2*q1*mx * q3 - mz * q1*q1 + 2*q2 * my * q3 - mz * q2*q2 + mz * q3*q3;
	by = bx;

	z_ << 2*(q1*q3-q0*q2),
			2*(q0*q1+q2*q3),
			2*(0.5-q1*q1-q2*q2),
			2*bx*(0.5-q2*q2-q3*q3)+2*bz*(q1*q3-q0*q2),
			2*bx*(q1*q2-q0*q3)+2*bz*(q0*q1+q2*q3),
			2*bx*(q0*q2+q1*q3)+2*bz*(0.5-q1*q1-q2*q2);

	double dhx_mx, dhx_my, dhx_mz, dhy_mx, dhy_my, dhy_mz;

	dhx_mx = q0*q0  + q1*q1  - q2*q2 -  q3*q3;
	dhx_my = -2*q0*q3+2*q1*q2;
	dhx_mz = 2*q0*q2 +2*q1*q3;

	dhy_mx = 2*q0*q3+2*q1*q2;
	dhy_my = q0*q0  - q1*q1  + q2*q2 -  q3*q3;
	dhy_mz =  - 2*q0*q1  + 2*q2*q3;

	double dbx_mx, dbx_my, dbx_mz, dbz_mx, dbz_my, dbz_mz;

	dbx_mx = 1.0/sqrt(hx*hx+hy*hy) * (hx*dhx_mx + hy*dhy_mx);
	dbx_my = 1.0/sqrt(hx*hx+hy*hy) * (hx*dhx_my + hy*dhy_my);
	dbx_mz = 1.0/sqrt(hx*hx+hy*hy) * (hx*dhx_mz + hy*dhy_mz);

	dbz_mx = -2*q0*q2+2*q1*q3;
	dbz_my = 2*q0*q1+2*q2*q3;
	dbz_mz = q0*q0 - q1*q1 -q2*q2 +q3*q3;

	H <<   -2*q2,			 2*q3, 			  -2*q0, 			 2*q1, 				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			2*q1,     		 2*q0, 			   2*q3, 			 2*q2, 				0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0,       	    -4*q1, 			  -4*q2, 			 0, 				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		   -2*bz*q2,  		 2*bz*q3, 		  -4*bx*q2-2*bz*q0, -4*bx*q3+2*bz*q1,   2*dbx_mx*(0.5-q2*q2-q3*q3)+2*dbz_mx*(q1*q3-q0*q2), 2*dbx_my*(0.5-q2*q2-q3*q3)+2*dbz_my*(q1*q3-q0*q2), 2*dbx_mz*(0.5-q2*q2-q3*q3)+2*dbz_mz*(q1*q3-q0*q2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		   -2*bx*q3+2*bz*q1, 2*bx*q2+2*bz*q0,  2*bx*q1+2*bz*q3, -2*bx*q0+2*bz*q2,   2*dbx_mx*(q1*q2-q0*q3)+2*dbz_mx*(q0*q1+q2*q3),     2*dbx_my*(q1*q2-q0*q3)+2*dbz_my*(q0*q1+q2*q3),     2*dbx_mz*(q1*q2-q0*q3)+2*dbz_mz*(q0*q1+q2*q3),     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			2*bx*q2, 		 2*bx*q3-4*bz*q1,  2*bx*q0-4*bz*q2,  2*bx*q1,			2*dbx_mx*(q0*q2+q1*q3)+2*dbz_mx*(0.5-q1*q1-q2*q2), 2*dbx_my*(q0*q2+q1*q3)+2*dbz_my*(0.5-q1*q1-q2*q2), 2*dbx_mz*(q0*q2+q1*q3)+2*dbz_mz*(0.5-q1*q1-q2*q2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

	MatrixXd R(6,6);

	R<< 0.05,0,0,0,0,0,
		0,0.05,0,0,0,0,
		0,0,0.05,0,0,0,
		0,0,0,0.05,0,0,
		0,0,0,0,0.05,0,
		0,0,0,0,0,0.05;

	MatrixXd K(19,6), S(6,6);

	S = (H*sigma_*H.transpose()+R);
	K = sigma_*H.transpose()*S.inverse();


	accelbf.normalize();
	magbf.normalize();

	VectorXd z(6);
	z << accelbf, magbf;

	X = X_ + K*(z-z_);
	sigma = (MatrixXd::Identity(19,19)-K*H)*sigma_;

	X.head(4).normalize();
	X.segment<3>(4).normalize();

	X_ = X;
	sigma_ = sigma;

    Quaterniond q(X(0), X(1), X(2), X(3));

	tf::Quaternion q_(X(1),X(2),X(3),X(0));

	tf::Matrix3x3 RotMat(q_);
	RotMat.getEulerYPR(yaw,pitch,roll);

	geometry_msgs::Vector3Stamped rpy;

	rpy.vector.x = roll*180/3.14;
	rpy.vector.y = pitch*180/3.14;
	rpy.vector.z = yaw*180/3.14;

	//myfile << ros::Time::now()<<"\t"<<rpy.vector.z <<endl;
	rpy.header.stamp = ros::Time::now();
	if(anglesPub.getNumSubscribers()!=0){
		anglesPub.publish(rpy);
	}

}

void EKF::ekfUpdateHeight(double sonarDistance){

	double q0, q1, q2, q3;
	q0 = X_(0);
	q1 = X_(1);
	q2 = X_(2);
	q3 = X_(3);

	double z_ = X_(15)/(q0*q0+q3*q3-q1*q1-q2*q2);

	MatrixXd H(1,19);
	H << -X_(15)*2*q0/((q0*q0+q3*q3-q1*q1-q2*q2)*(q0*q0+q3*q3-q1*q1-q2*q2)), X_(15)*2*q1/((q0*q0+q3*q3-q1*q1-q2*q2)*(q0*q0+q3*q3-q1*q1-q2*q2)), X_(15)*2*q2/((q0*q0+q3*q3-q1*q1-q2*q2)*(q0*q0+q3*q3-q1*q1-q2*q2)), -X_(15)*2*q3/((q0*q0+q3*q3-q1*q1-q2*q2)*(q0*q0+q3*q3-q1*q1-q2*q2)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/(q0*q0+q3*q3-q1*q1-q2*q2), 0, 0, 0;

	MatrixXd R(1,1);
	R<< 0.001;

	MatrixXd S= H*sigma_*H.transpose() + R;

	MatrixXd K(19,1);
	K = sigma_*H.transpose()*S.inverse();

	X = X_ + K*(sonarDistance - z_);
	sigma = (MatrixXd::Identity(19,19) - K*H)*sigma_;

	sigma_ =sigma;
	X_ = X;

	geometry_msgs::PoseStamped pose;

	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "quadPose";

	pose.pose.position.x = sonarDistance;
	pose.pose.position.y = X_(15); 
	double mean=0;

	switch(countHeight){
	case 0:
		a1 = 0.2*X_(15);
		countHeight++;	
		break;
	case 1:
		a2 = 0.2*X_(15);
		countHeight++;
		break;
	case 2: 
		a3 = 0.2*X_(15);
		countHeight++;
		break;
	case 3:
		a4 = 0.2*X_(15);
		countHeight++;
		break;
	default:
		mean = a1+a2+a3+a4+0.2*X_(15);
		a1=a2;
		a2=a3;
		a3=a4;
		a4=0.2*X_(15);
		pose.pose.position.z = mean;
		posePub.publish(pose);

	}
	//if(heightArray.size()<5){
	//	heightArray.push_back(X_(15));
	//	return;
	//}
	//else{
	//for(int a=0; a<5;a++){
		
	//	mean = mean + (0.2* heightArray[a-1]);	
	//	heightArray[a-1] =  heightArray[a];
	//}
	//pose.pose.position.z = mean+(0.2*X_(15));
	//heightArray[4] = X_(15);
	//}

	}


}
