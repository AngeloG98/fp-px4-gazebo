/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Run in Gazebo SITL
 */
#include "offb_node.hpp"


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void planWaypoint_callback(const quadrotor_msgs::PositionCommand &msg){
    planned_point.x= msg.position.x;
    planned_point.y= msg.position.y;
    planned_point.z= msg.position.z;
    planned_velocity.x = msg.velocity.x;
    planned_velocity.y = msg.velocity.y;
    planned_velocity.z = msg.velocity.z;
    planned_yaw_angle = msg.yaw;
    planned_yaw_dot = msg.yaw_dot;
    planned_update_flag = true;
    // ROS_INFO("planned_velocity %f %f %f %f",planned_velocity.x,planned_velocity.y,planned_velocity.z,planned_yaw_dot*rad2deg);
    ROS_INFO("planned_position %f %f %f %f",planned_point.x,planned_point.y,planned_point.z,planned_yaw_angle*rad2deg);
}

void odometry_callback(const nav_msgs::Odometry &current_info)
{
    current_point.x= current_info.pose.pose.position.x;
    current_point.y= current_info.pose.pose.position.y;
    current_point.z= current_info.pose.pose.position.z;
    current_angle.x= current_info.pose.pose.orientation.x;
    current_angle.y= current_info.pose.pose.orientation.y;
    current_angle.z= current_info.pose.pose.orientation.z;
    current_angle.w= current_info.pose.pose.orientation.w;
    curr_angle = toEulerAngle(current_angle);
    current_yaw_angle = curr_angle.z;
    //   ROS_INFO("current position : %f %f %f",current_point.x,current_point.y,current_point.z);
    //  ROS_INFO("current angle :   %f %f %f",curr_angle.x*rad2deg,curr_angle.y*rad2deg,curr_angle.z*rad2deg);
    current_update_flag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    planWaypointsub = nh.subscribe("/planning/pos_cmd", 10, planWaypoint_callback);
    odometrysub     = nh.subscribe("/mavros/local_position/odom",10,odometry_callback);

    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //set init pose
    geometry_msgs::PoseStamped init_pose;
    init_pose.pose.position.x = -4.5;
    init_pose.pose.position.y = 3;
    init_pose.pose.position.z = 1.8;
    init_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.1415926/2);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if(current_state.armed && !IFPLANNER && (ros::Time::now() - last_request > ros::Duration(10.0))){
            ROS_INFO("Vehicle takeoff");
            IFPLANNER = true;
        }
        if (IFPLANNER)
        {
            if (planned_update_flag)
            {
                //vel cmd
                // linear.x = planned_velocity.x;
                // linear.y = planned_velocity.y;
                // linear.z = planned_velocity.z;
                // angular.x = 0;
                // angular.y = 0;
                // angular.z = planned_yaw_dot;
                // msgtwist.header.stamp = ros::Time::now();
                // msgtwist.header.seq=1;
                // msgtwist.twist.linear=linear;
                // msgtwist.twist.angular=angular;
                // ROS_INFO("send to vechile: %f %f %f %f",msgtwist.twist.linear.x,msgtwist.twist.linear.y,msgtwist.twist.linear.z,msgtwist.twist.angular.z*rad2deg);
                // local_vel_pub.publish(msgtwist);
                // planned_update_flag = false;
                //pos cmd
                pos = planned_point;
                ori = tf::createQuaternionMsgFromYaw(planned_yaw_angle);
                msgpose.header.stamp = ros::Time::now();
                msgpose.header.seq=1;
                msgpose.pose.position = pos;
                msgpose.pose.orientation = ori;
                ROS_INFO("send to vechile: %f %f %f %f",msgpose.pose.position.x,msgpose.pose.position.y,msgpose.pose.position.z,tf::getYaw(ori)*rad2deg);
                local_pos_pub.publish(msgpose);
                planned_update_flag = false;
            }
            else
            {
                ROS_INFO("Waiting for planned data...");
                //vel cmd
                // linear.x = 0;
                // linear.y = 0;
                // linear.z = 0;
                // angular.x = 0;
                // angular.y = 0;
                // angular.z = 0;
                // msgtwist.header.stamp = ros::Time::now();
                // msgtwist.header.seq=1;
                // msgtwist.twist.linear=linear;
                // msgtwist.twist.angular=angular;
                // ROS_INFO("send to vechile: %f %f %f %f",msgtwist.twist.linear.x,msgtwist.twist.linear.y,msgtwist.twist.linear.z,msgtwist.twist.angular.z*rad2deg);
                // local_vel_pub.publish(msgtwist);
                //pos cmd
                pos = current_point;
                ori = current_angle;
                msgpose.header.stamp = ros::Time::now();
                msgpose.header.seq=1;
                msgpose.pose.position = pos;
                msgpose.pose.orientation = ori;
                ROS_INFO("send to vechile: %f %f %f %f",msgpose.pose.position.x,msgpose.pose.position.y,msgpose.pose.position.z,tf::getYaw(ori)*rad2deg);
                local_pos_pub.publish(msgpose);
            }
        }
        
        else
        {
            local_pos_pub.publish(init_pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void position_pid_control(geometry_msgs::Point current_set_point,geometry_msgs::Point current_local_point,float velocity_limit,float target_yaw, float dead_zone)
{
    float vx = 0.0, vy = 0.0, vz = 0, vxp = 0.0, vyp = 0.0, vxi = 0.0, vyi = 0.0, vxd = 0.0, vyd = 0.0;
    float yaw_rate = 0.0, yaw_rate_p = 0.0, yaw_rate_i = 0.0, yaw_rate_d = 0.0;
    float roll = 0.0, pitch = 0.0;
    //position control  mode & 0x01
    if (1) {
        //calculate velocity, P control
        geometry_msgs::Point error_pos;
        error_pos.x = current_set_point.x - current_local_point.x;
        error_pos.y = current_set_point.y - current_local_point.y;
        vxp = P_pos * error_pos.x;
        vyp = P_pos * error_pos.y;
        vxd = D_pos * (error_pos.x - error_pos_last.x);
        vyd = D_pos * (error_pos.y - error_pos_last.y);
        vxd = limit(vxd, D_VEL_LIMIT);
        vyd = limit(vyd, D_VEL_LIMIT);
        /*if(abs(error_pos.x) < abs(error_pos_last.x) && (abs(error_pos.x) >= 1.5 * DEAD_ZONE))
        {
          vxd = 0.0;
        }
        if(abs(error_pos.y) < abs(error_pos_last.y) && (abs(error_pos.y) >= 1.5 * DEAD_ZONE))
        {
          vyd = 0.0;
        }*/
        if (abs(error_pos.x) >= POS_I_DEAD_ZONE) {
            error_pos_integrated.x += error_pos.x;
        } else {
            error_pos_integrated.x = 0.0;
        }
        if (abs(error_pos.y) >= POS_I_DEAD_ZONE) {
            error_pos_integrated.y += error_pos.y;
        } else {
            error_pos_integrated.y = 0.0;
        }
        if (I_pos > 0.0001) {
            error_pos_integrated.x = limit((float) error_pos_integrated.x, I_VEL_LIMIT / I_pos);
            error_pos_integrated.y = limit((float) error_pos_integrated.y, I_VEL_LIMIT / I_pos);
        }
        vxi = I_pos * error_pos_integrated.x;
        vyi = I_pos * error_pos_integrated.y;
        vx = (1-control_alpha)*(vxp + vxi + vxd)+control_alpha*(planned_velocity.x);
        vy = (1-control_alpha)*(vyp + vyi + vyd)+control_alpha*(planned_velocity.y);

        float x_offset = current_set_point.x - current_local_point.x;
        float y_offset = current_set_point.y - current_local_point.y;
        float distance = sqrt(x_offset * x_offset + y_offset * y_offset);
        if (distance <= dead_zone) {
            dead_zone_flag = true;
            error_pos_integrated.x = 0.0;
            error_pos_integrated.y = 0.0;
            vx = 0;
            vy = 0;
        } else {
            dead_zone_flag = false;
        }

        //limit the speed
        vx = limit_velocity(vx, vy, velocity_limit).x;
        vy = limit_velocity(vx, vy, velocity_limit).y;
        //ROS_INFO("vx_exp vy_exp %f %f",vx,vy);

        error_pos_last = error_pos;
    }
    //yaw control  mode & 0x02
    if (1) {
        //use the flight controller yaw in default
        
        float current_yaw = current_yaw_angle;
        //Drone turns at the smallest angle
        float error_yaw = (target_yaw - current_yaw) * rad2deg;
        if (error_yaw < -180)
            error_yaw += 360;
        else if (error_yaw > 180)
            error_yaw -= 360;
        yaw_rate_p = P_yaw * error_yaw;
        yaw_rate_d = D_yaw * (error_yaw - error_yaw_last);
        error_yaw_integrated += error_yaw;
        if (I_yaw > 0.0001) {
            error_yaw_integrated = limit(error_yaw_integrated, YAW_I_LIMIT / I_yaw);
        }
        yaw_rate_i = I_yaw * error_yaw_integrated;
        if (abs(error_yaw) <= YAW_DEAD_ZONE) {
            yaw_rate_p = 0.0;
            yaw_rate_i = 0.0;
            yaw_rate_d = 0.0;
            error_yaw_integrated = 0.0;
        }
        yaw_rate = (1-control_alpha)* (yaw_rate_p + yaw_rate_i + yaw_rate_d)+control_alpha*planned_yaw_dot;
        if (abs(error_yaw) >= YAW_RATE_LIMIT) {
            yaw_rate = limit(yaw_rate, YAW_RATE_LIMIT);
        } 
        // else {
        //     yaw_rate = limit(yaw_rate, (float) (YAW_RATE_LIMIT * 0.4));
        // }
        error_yaw_last = error_yaw;
    }

    //height control  mode & 0x04
    if (1) {
        //control the height around 1.5m, the height is from flight controller
        float deltaz = current_local_point.z - current_set_point.z;
        if (fabs(deltaz) >= HEIGHT_DEAD_ZONE) {
            vz = P_z * (current_set_point.z - current_local_point.z);
        } else {
            vz = 0.0;
        }
    }

    velocity_expected.x = vx * cos(curr_angle.z) + vy * sin(curr_angle.z);
    velocity_expected.y = vy * cos(curr_angle.z) - vx * sin(curr_angle.z);
    velocity_expected.z = vz;
    attitude_expect.z = yaw_rate * deg2rad;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 ans;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    return ans;
}

geometry_msgs::Point limit_velocity(float vx, float vy,float maximum)
{
	geometry_msgs::Point vel;
	float velocity = sqrt(vx * vx + vy * vy);
	if(maximum <= 0)
	{
		vel.x = 0;
		vel.y = 0;
	}
	if(velocity <= maximum)
	{
		vel.x = vx;
		vel.y = vy;
	}
	//if velocity is bigger than maximum, then limit the vx and vy, and keep the direction at meanwhile.
	else
	{
		//the velocity must not be zero when in this step
		vel.x = vx / velocity * maximum;
		vel.y = vy / velocity * maximum;
	}
	return vel;
}