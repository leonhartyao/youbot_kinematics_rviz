#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>
#include <youbot_kinematics_rviz/work_space_stateConfig.h>
#include <ik_solver_service/SolveRollPitchIK.h>
#include <iostream>


// global variables
bool config_changed = false;
const double DEGREE = M_PI/180.0;
double offset[5] = {2.96700156811167,
							1.134568928529628,
							-2.5480925878999994,
							1.789009218025524,
							2.931414};
ros::Publisher joint_pub;
ik_solver_service::SolveRollPitchIK rp_srv;
sensor_msgs::JointState joint_state;

// callback function of dynamic reconfigure
void callback_recfg(youbot_kinematics_rviz::work_space_stateConfig config, uint32_t level)
{
	config_changed = true;
	ROS_INFO("Reconfigure cartesian coordinates:");
	std::cout << config.xd << " "
	          << config.yd << " "
	          << config.zd << " "
	          << config.rolld << " "
	          << config.pitchd << " "
	          << config.arm_to_front << " "
	          << config.arm_bended_up << std::endl;
	// assign values to IK request
	rp_srv.request.des_position[0] = config.xd;
	rp_srv.request.des_position[1] = config.yd;
	rp_srv.request.des_position[2] = config.zd;
	rp_srv.request.roll = config.rolld*DEGREE;
	rp_srv.request.pitch = -config.pitchd*DEGREE;
	if(config.arm_to_front)
	{
		if(config.arm_bended_up)
		{
			rp_srv.request.id = 1;
		}
		else
		{
			rp_srv.request.id = 2;
		}
	}
	else
	{
		if(config.arm_bended_up)
		{
			rp_srv.request.id = 3;
		}
		else
		{
			rp_srv.request.id = 4;
		}
	}
}

int main(int argc, char** argv) {
ros::init(argc, argv, "joint_state_publisher");
ros::NodeHandle n;
ros::ServiceClient solve_roll_pitch_ik_client = n.serviceClient<ik_solver_service::SolveRollPitchIK>("solve_roll_pitch_ik");

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

dynamic_reconfigure::Server<youbot_kinematics_rviz::work_space_stateConfig> server;
dynamic_reconfigure::Server<youbot_kinematics_rviz::work_space_stateConfig>::CallbackType f;
f = boost::bind(&callback_recfg, _1, _2);
server.setCallback(f);

while(n.ok())
{
	// call IK service if desired Cartesian coordinates changed
	if(config_changed)
	{
		if(solve_roll_pitch_ik_client.call(rp_srv))
		{
			printf("Solve Inverse Kinematics:\n");
			for (int i = 0; i < 5; i++)
		    {
		      printf("%f\t", rp_srv.response.joint_angles[i]);
		    }
		    printf("\n feasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", rp_srv.response.feasible,
		    		  rp_srv.response.arm_to_front, rp_srv.response.arm_bended_up, rp_srv.response.gripper_downwards);

			if(rp_srv.response.feasible)
			{
				for(int i=0; i<5; i++)
				{
					joint_state.position[i] = rp_srv.response.joint_angles[i];
				}
			}
		}
		else
		{
		  ROS_ERROR("Failed to call service solve_roll_pitch_ik!");
		}

		config_changed = false;
	}
	joint_state.header.stamp = ros::Time::now();
	joint_pub.publish(joint_state);
	ros::spinOnce();
	loop_rate.sleep();
}
return 0;
}
