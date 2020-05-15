/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <math.h>
#include <iostream>///

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros/mavros_plugin.h>
#include <GeographicLib/Geocentric.hpp>


mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;//
geometry_msgs::TwistStamped current_twist; //1
//geometry_msgs::Twist setpoint_anguler; //1
std_msgs::Float64 current_heading;
mavros_msgs::WaypointList waypoint;
sensor_msgs::NavSatFix current_gps;

const double R = 6371000;
const double pi = 3.141592653589793238;
bool reached = false;
bool ret = false;
//bool need_heading_change = true;

float gps_to_local_current_x;
float gps_to_local_current_y;
float gps_to_local_x;
float gps_to_local_y;

float enu_x;
float enu_y;
float dist ;
float ned_x;
float ned_y;
float ned_vx;
float ned_vy;

float desired_heading;
//float how_many;
float heading_x;
float heading_y;

float desired_z;

float temp_x;
float temp_y;
float avoid_mode = false;

float left_aera;
float right_aera;
float middle_aera;
int left;
int center;
int right;
float avoid_theta;

geometry_msgs::Twist setpoint_twist;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pos){//
	current_pose = *pos;//
}
void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& twi){ //2
	current_twist = *twi; //2
}
void compass_cb(const std_msgs::Float64::ConstPtr& deg){
	current_heading = *deg;
}
void waypoint_cb(const mavros_msgs::WaypointList::ConstPtr& wp){
	waypoint = *wp;
}
void current_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& gps){
	current_gps = *gps;
}

// gps -> local xyz
void global_to_local(){
	GeographicLib::Geocentric earth(
		GeographicLib::Constants::WGS84_a(), 
		GeographicLib::Constants::WGS84_f());
	Eigen::Vector3d current_gps_vector(
		current_gps.latitude, 
		current_gps.longitude, 
		current_gps.altitude);

	// current gps -> curent ECEF
	Eigen::Vector3d current_ecef;
	earth.Forward(current_gps_vector.x(), current_gps_vector.y(), current_gps_vector.z(),
		current_ecef.x(), current_ecef.y(), current_ecef.z());
	// goal gps -> goal ECEF
	Eigen::Vector3d goal_ecef;
	earth.Forward(waypoint.waypoints[0].x_lat, waypoint.waypoints[0].y_long, 2.5,
	    goal_ecef.x(), goal_ecef.y(), goal_ecef.z());
    // get ENU offset from ECEF offset
	Eigen::Vector3d ecef_offset = goal_ecef - current_ecef;
    Eigen::Vector3d enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, current_gps_vector);

    // set position setpoint
    ned_x = current_pose.pose.position.x + enu_offset.x();
    ned_y = current_pose.pose.position.y + enu_offset.y();
}

//calculate linear velocity
bool cal_linear_vel(float speed, float error){

	dist = std::sqrt(
		std::pow(ned_x - current_pose.pose.position.x, 2)+
		std::pow(ned_y - current_pose.pose.position.y, 2));
	setpoint_twist.linear.x = (ned_x - current_pose.pose.position.x) / dist * speed;
	setpoint_twist.linear.y = (ned_y - current_pose.pose.position.y) / dist * speed;
	setpoint_twist.linear.z = 0;

	if( std::sqrt(std::pow((ned_x - current_pose.pose.position.x), 2) 
		+ std::pow(ned_y - current_pose.pose.position.y, 2)) < error ){

		setpoint_twist.linear.x = 0; //2
		setpoint_twist.linear.y = 0; //2
		setpoint_twist.linear.z = 0; //2
		return false;
	}
	//ROS_INFO("velocity  x : %f   y : %f", ned_vx, ned_vy);
	return true;
}

//calculate yaw
bool cal_yaw(float speed, float error){
	
	float cos_theta = (ned_y - current_pose.pose.position.y) / dist;
	if(ned_x - current_pose.pose.position.x >= 0){
		if(cos_theta > 1){
			desired_heading = 0.0;
		}else if(cos_theta < -1){
			desired_heading = 180.0;
		}else{
			desired_heading = std::acos(cos_theta) * 180.0 / pi;	
		}
		
	}
	else{
		if(cos_theta > 1){
			desired_heading = 0.0;
		}else if(cos_theta < -1){
			desired_heading = 180.0;
		}else{
			desired_heading = std::acos(-cos_theta) * 180.0 / pi +180;	
		}
		
	}
	// else{
	// 	desired_heading = std::asin(ned_x / dist) * 180.0 / pi + 360;
	// }

	float how_many = desired_heading - current_heading.data;
	if(std::fabs(how_many) >= error){
		setpoint_twist.linear.x = 0;
		setpoint_twist.linear.y = 0;
		setpoint_twist.linear.z = 0;
	}
	//ROS_INFO("degrees   diff : %f   desired : %f   current : %f"
	//	,how_many, desired_heading, current_heading.data);
	if(how_many >= 0){
		if(how_many <= 180){
			setpoint_twist.angular.z = -speed;
		}
		else{
			setpoint_twist.angular.z = speed;
		}
	}
	else{
		if (how_many >= -180){
			setpoint_twist.angular.z = speed;
		}
		else{
			setpoint_twist.angular.z = -speed;			
		}
	}

	if(std::fabs(how_many) < error){
		
		setpoint_twist.angular.z = 0;
		return false;
	}
	return true;
}

// calculate altitude
bool cal_alt(float speed, float error){
	
	float how_many = desired_z - current_pose.pose.position.z;
	ROS_INFO("altitude   diff : %f   desired : %f   current : %f"
		, how_many, desired_z, current_pose.pose.position.z);

	if(how_many >= error){
		setpoint_twist.linear.z = speed;
	} else if(how_many <= -error){
		setpoint_twist.linear.z = -speed;
	}
	else{
		setpoint_twist.linear.z = 0;
		return false;
	}
	return true;
}

// calculate new temp setpoint
bool avoid(float theta, float degree_speed, float linear_speed, float degree_err){
	
	if(center == 1 && avoid_mode == false){
		avoid_theta = current_heading.data - theta;
		ROS_INFO("once?");
		avoid_mode = true;
		
		if(avoid_theta < 0)
			avoid_theta +=360;
		temp_x = current_pose.pose.position.x;
		temp_y = current_pose.pose.position.y;
	}

	ROS_INFO("avoid  %f   current %f",  avoid_theta,  current_heading.data );

	if(avoid_mode == true){
		if(std::sqrt(std::pow(temp_x - current_pose.pose.position.x, 2) 
			+ std::pow(temp_y - current_pose.pose.position.y, 2)) < middle_aera/std::cos(theta)){
			setpoint_twist.linear.x = std::sin(current_heading.data) * linear_speed;
			setpoint_twist.linear.y = std::cos(current_heading.data) * linear_speed;
			setpoint_twist.angular.z = 0;
			// if(std::fabs(avoid_theta - current_heading.data) >= degree_err){
			// 	setpoint_twist.linear.x = 0;
			// 	setpoint_twist.linear.y = 0;
			// 	setpoint_twist.angular.z = -degree_speed;
			// 	ROS_INFO("degree avoid");
			// }
		}
		
		else{
			avoid_mode = false;
			return false;
		}
	}
	
	
	return true;

}


void get_safe_aera(rs2::pipeline pipe){
	rs2::frameset frames = pipe.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        float width = depth.get_width();
        float height = depth.get_height();
        int aera_width_size = (int) (width/3);
        int aera_height_size = (int) (height/3);
        //This code is added
	left_aera = 0;
	right_aera = 0;
	middle_aera = 0;


	for (int i = 0 ; i < width ; i ++ ){
		for(int j = 0 ; j < height ; j ++){
			if(i < aera_width_size*2){
				if( i < aera_width_size){
					left_aera += depth.get_distance(i, j);
				}			
				else{
					middle_aera += depth.get_distance(i, j);
				}
			}
			else{
				right_aera += depth.get_distance(i, j);
			}
		}
	}
        if( left_aera/(aera_width_size*height) < 2.0 ){
        		left_aera /= aera_width_size*height;
        		left = 1;
        }
        else{
			left = 0;
        }
        if (middle_aera/(aera_width_size*height)< 2.0){
        	middle_aera /= aera_width_size*height;
			center = 1;
        }
        else{
        	center = 0;
        }
        if (right_aera/(aera_width_size*height)< 2.0){
        	middle_aera /= aera_width_size*height;
        	right = 1;
        }
        else{
        	right = 0;
        }
        ROS_INFO("left_safe : %d    middle_safe : %d     right_safe : %d", left, center, right);
}

int main(int argc, char **argv)
{
	
	rs2::pipeline pipe;
	rs2::config cfg;
	rs2::colorizer color_map(2);
	
	try{
		
		cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 6);	
		
		pipe.start(cfg);
	}catch(const rs2::error &e){
		
		std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    	return EXIT_FAILURE;	 
	}
	
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		("mavros/state", 10, state_cb);

	ros::Subscriber local_pos = nh.subscribe<geometry_msgs::PoseStamped>//
		("mavros/local_position/pose", 10, pose_cb);//
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
		("mavros/setpoint_position/local", 10);
	ros::Subscriber local_vel = nh.subscribe<geometry_msgs::TwistStamped> //1
		("mavros/local_position/velocity_local", 10, twist_cb); //1
	ros::Publisher local_twist_pub = nh.advertise<geometry_msgs::Twist> //1
		("mavros/setpoint_velocity/cmd_vel_unstamped", 10); //1

	ros::Subscriber compas_heading = nh.subscribe<std_msgs::Float64>
		("mavros/global_position/compass_hdg", 10, compass_cb);
	ros::Subscriber get_waypoint = nh.subscribe<mavros_msgs::WaypointList> //1
		("mavros/mission/waypoints", 10, waypoint_cb); //1
	ros::Subscriber get_current_gps = nh.subscribe<sensor_msgs::NavSatFix> //1
		("mavros/global_position/global", 10, current_gps_cb); //1

	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
		("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");

	ROS_INFO("OK!!!");
		//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);

	geometry_msgs::PoseStamped setpoint_pose;//0

	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}
	//send a few setpoints before starting
	setpoint_twist.linear.x = 0;
	setpoint_twist.linear.y = 0;
	setpoint_twist.linear.z = 0;
	for(int i = 100; ros::ok() && i > 0; --i){
		local_twist_pub.publish(setpoint_twist);
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();
	
	desired_z = 3;//current_pose.pose.position.z;
	global_to_local();
	cal_linear_vel(0.7, 1.5);
	cal_yaw(0.4, 3);
	ROS_INFO("x : %f y : %f", ned_x, ned_y);

	// rotate heading to mission point
	while( ros::ok() ){

		if( current_state.mode != "OFFBOARD" &&
			(ros::Time::now() - last_request > ros::Duration(5.0))){
			if( set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.mode_sent){
				ROS_INFO("Offboard enabled");
		}
		last_request = ros::Time::now();
		}/* else {
			if( !current_state.armed &&
				(ros::Time::now() - last_request > ros::Duration(5.0))){
				if( arming_client.call(arm_cmd) &&
					arm_cmd.response.success){
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}
		*/
		ROS_INFO("desired heading : %f   current heading : %f"
			, desired_heading
			, current_heading.data);
		if(!cal_yaw(0.4, 3)){
			break;
		}
		//cal_alt(0.1, 0.3);

		local_twist_pub.publish(setpoint_twist);
		ros::spinOnce();
		rate.sleep();
	}

	// go to mission point
	while( ros::ok() ){

		ROS_INFO("current pose x : %f y : %f", current_pose.pose.position.x,
			current_pose.pose.position.y);

		if(!cal_linear_vel(0.7, 1)){
			break;
		}
		else{
			cal_yaw(0.2, 3);
			if(!avoid_mode)
				get_safe_aera(pipe);
			avoid(45, 0.3, 0.7, 3);
		}
		//cal_alt(0.1, 0.3);

		// ROS_INFO("------   %f   %f", setpoint_twist.linear.x, setpoint_twist.linear.y);
		local_twist_pub.publish(setpoint_twist);
		ros::spinOnce();
		rate.sleep();
	}

	// return zero point
	ned_x = 0; ned_y = 0;
	cal_linear_vel(0.7, 1);
	cal_yaw(0.3, 3);


	// rotate heading to zero point 
	while( ros::ok() ){
		
		ROS_INFO("desired heading : %f   current heading :%f"
			, desired_heading
			, current_heading.data);
		if(!cal_yaw(0.2, 3)){
			break;
		}
		//cal_alt(0.1, 0.3);

		local_twist_pub.publish(setpoint_twist);
		ros::spinOnce();
		rate.sleep();
	}

	// go to zero point
	while( ros::ok() ){

		ROS_INFO("current pose x : %f y : %f", current_pose.pose.position.x,
			current_pose.pose.position.y);

		if(!cal_linear_vel(0.7, 1)){
			setpoint_twist.linear.x = 0; //2
			setpoint_twist.linear.y = 0; //2
			setpoint_twist.linear.z = 0; //2
		}
		else{
			cal_yaw(0.2, 3);
			if(!avoid_mode)
				get_safe_aera(pipe);
			avoid(45, 0.3, 0.7, 3);
		}
		//cal_alt(0.1, 0.3);

		local_twist_pub.publish(setpoint_twist);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
