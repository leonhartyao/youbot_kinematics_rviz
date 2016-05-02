#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>
#include <youbot_kinematics_rviz/arm_joint_stateConfig.h>
#include <iostream>


// global variables
const double d1 = 0.147;
const double a1 = 0.033;
const double a2 = 0.155;
const double a3 = 0.135;
const double d5 = 0.2175;
const double DEGREE = M_PI/180.0;
double offset[5] = {2.96700156811167,
							1.134568928529628,
							-2.5480925878999994,
							1.789009218025524,
							2.931414};
ros::Publisher joint_pub;
sensor_msgs::JointState joint_state;

// callback function of dynamic reconfigure
void callback_recfg(youbot_kinematics_rviz::arm_joint_stateConfig config, uint32_t level)
{
	ROS_INFO("Reconfigure joint state:");
	std::cout << config.arm1 << " "
	          << config.arm2 << " "
	          << config.arm3 << " "
	          << config.arm4 << " "
	          << config.arm5 << " "
	          << config.gripper_l_param << " "
	          << config.gripper_r_param << std::endl;
	// convert degree to radiant
	std::vector<double> joint_value;
	joint_value.push_back(config.arm1*DEGREE);
	joint_value.push_back(config.arm2*DEGREE);
	joint_value.push_back(config.arm3*DEGREE);
	joint_value.push_back(config.arm4*DEGREE);
	joint_value.push_back(config.arm5*DEGREE);
	for(int i=0; i<5; i++)
	{
		joint_state.position[i] = joint_value[i] + offset[i];
	}
	joint_state.position[5] = config.gripper_l_param;
	joint_state.position[6] = config.gripper_r_param;
	// calculate forward kinematics
	double T[4][4];
	T[0][0] = cos(joint_value[1]+joint_value[2]+joint_value[3])*cos(joint_value[0])*cos(joint_value[4])
			-sin(joint_value[0])*sin(joint_value[4]);
	T[0][1] = -cos(joint_value[1]+joint_value[2]+joint_value[3])*cos(joint_value[0])*sin(joint_value[4])
			-sin(joint_value[0])*cos(joint_value[4]);
	T[0][2] = -sin(joint_value[1]+joint_value[2]+joint_value[3])*cos(joint_value[0]);
	T[0][3] = -cos(joint_value[0])*(a3*sin(joint_value[1]+joint_value[2])-a1+a2*sin(joint_value[1])
			+d5*sin(joint_value[1]+joint_value[2]+joint_value[3]));
	T[1][0] = cos(joint_value[0])*sin(joint_value[4])+cos(joint_value[1]+joint_value[2]+joint_value[3])
			*cos(joint_value[4])*sin(joint_value[0]);
	T[1][1] = cos(joint_value[0])*cos(joint_value[4])-cos(joint_value[1]+joint_value[2]+joint_value[3])
			*sin(joint_value[0])*sin(joint_value[4]);
	T[1][2] = -sin(joint_value[1]+joint_value[2]+joint_value[3])*sin(joint_value[0]);
	T[1][3] = -sin(joint_value[0])*(a3*sin(joint_value[1]+joint_value[2])-a1+a2*sin(joint_value[1])
			+d5*sin(joint_value[1]+joint_value[2]+joint_value[3]));
	T[2][0] = sin(joint_value[1]+joint_value[2]+joint_value[3])*cos(joint_value[4]);
	T[2][1] = -sin(joint_value[1]+joint_value[2]+joint_value[3])*sin(joint_value[4]);
	T[2][2] = cos(joint_value[1]+joint_value[2]+joint_value[3]);
	T[2][3] = d1+a3*cos(joint_value[1]+joint_value[2])+a2*cos(joint_value[3])
			+d5*cos(joint_value[1]+joint_value[2]+joint_value[3]);
	T[3][0] = 0;
	T[3][1] = 0;
	T[3][2] = 0;
	T[3][3] = 1;
	std::cout << "End Effector Pose:" << std::endl;
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			std::cout << std::right << std::setw(12) << std::setfill(' ') << T[i][j];
		}
		std::cout << std::endl;
	}
}

int main(int argc, char** argv) {
ros::init(argc, argv, "joint_state_publisher");
ros::NodeHandle n;
joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
ros::Rate loop_rate(30);

joint_state.name.resize(7);
joint_state.position.resize(7);
joint_state.name[0] ="arm_joint_1";
joint_state.name[1] ="arm_joint_2";
joint_state.name[2] ="arm_joint_3";
joint_state.name[3] ="arm_joint_4";
joint_state.name[4] ="arm_joint_5";
joint_state.name[5] ="gripper_finger_joint_l";
joint_state.name[6] ="gripper_finger_joint_r";
// initial joint state (up straight)
for(int i = 0; i < 5; i++)
{
	joint_state.position[i] = offset[i];
}
joint_state.position[5] = 0.008;
joint_state.position[6] = 0.008;

dynamic_reconfigure::Server<youbot_kinematics_rviz::arm_joint_stateConfig> server;
dynamic_reconfigure::Server<youbot_kinematics_rviz::arm_joint_stateConfig>::CallbackType f;
f = boost::bind(&callback_recfg, _1, _2);
server.setCallback(f);

while(n.ok())
{
	joint_state.header.stamp = ros::Time::now();
	joint_pub.publish(joint_state);
	ros::spinOnce();
	loop_rate.sleep();
}
return 0;
}
