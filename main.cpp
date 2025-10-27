#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib-tarball/api.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"

#include "pros/colors.hpp"
#include <iostream>
#include <fstream> 
#include <sstream>
#include <filesystem> 
#include <string> 
#include <vector>
#include <random>
//time
#include <chrono>
#include <thread>
//lemlib
#include "liblvgl/core/lv_disp.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/widgets/lv_img.h"
#include "pros/misc.h"
#include "pros/misc.hpp"


namespace fs = std::filesystem;

// red team vs blue team
enum Color 
{
	RED = 1,
	BLUE = -1
};
// red team vs blue team
enum IntakeDirection 
{
	FORWARD = 1,
	REVERSE = -1,
};

// setup

// a negative number shows that the motor is reversed
// if a 3 wire has a number for the port, the number is just the letter of the port. 
//ex: 1 = A, 2 = B, 3 = C, 4 = D

// drive motors
pros::Motor front_left_motor(-11, pros::MotorGears::blue); // front left motor on port 9
pros::Motor middle_left_motor(12, pros::MotorGears::blue); // middle left motor on port 20
pros::Motor front_motor(-13, pros::MotorGears::blue); // back left motor on port 10
pros::Motor front_right_motor(2, pros::MotorGears::blue); // front right motor on port 2
pros::Motor middle_right_motor(-3, pros::MotorGears::blue); // middle right motor on port 11
pros::Motor back_motor(5, pros::MotorGears::blue); // back right motor on port 5

//pros::Motor teammate_motor(4, pros::MotorGears::blue); // back right motor on port 5


//intake_motors
pros::Motor intake_bottom(8, pros::MotorGears::blue); // intake motor on port 19
pros::Motor intake_top(10, pros::MotorGears::blue); // lift motor on port 12
pros::Motor intake_index(9, pros::MotorGears::blue); // lift motor on port 12

// condensed motors into motor groups
pros::MotorGroup left_motor_group({-11, 12, -13}, pros::MotorGears::blue); //the right side of the drivetrain
pros::MotorGroup right_motor_group({14, -15, 16}, pros::MotorGears::blue); //the left side of the drivetrain

lemlib::Drivetrain drivetrain_6m(&left_motor_group, //motors that are on the left channel
                              &right_motor_group, //motors that a	re on the right channel
                              11.25, // track width
                              lemlib::Omniwheel::NEW_275, //the specific vex wheels used
                              600, // the rpm of the driven axels
                              8 //horizontal drift
); 

// create an imu on port 12
pros::Imu imu(7);

pros::ADIDigitalOut ears(3);
pros::ADIDigitalOut scraper(2);
pros::ADIDigitalOut hood(1);

// left tracking wheel encoder
pros::Rotation vertical_encoder(-17);

//pros::Distance distance_front(5);

// left tracking wheel (&what sensor it is tracking, &what type of omniwheel, offset, gear ratio)
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 2.75, +0 /*-1*/, 1);
// right tracking wheel(&what sensor it is tracking, &what type of omniwheel, offset, gear ratio

//auton_selector
pros::ADIAnalogIn auton_seletor (7);

//color sensor
pros::Optical optical_sensor (6);

pros::Clock timer;

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr 
                        	nullptr,// horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr
                            &imu // inertial sensor
);


// lateral PID controller
lemlib::ControllerSettings lateral_controller(19, // proportional gain (kP)10
                                              0.15, // integral gain (kI)0.1
                                              150, // derivative gain (kD)53
                                              2, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              90//250 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.2, // proportional gain (kP)
                                              0.25, // integral gain (kI)
                                              28,// derivative gain (kD)
                                              4, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     3, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve	(3, // joystick deadband out of 127
                                  3, // minimum output where drivetrain will move out of 127
                                  1.019 //1.03 expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain_6m, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, // forward/backward driver movement
                        &steer_curve // left/right driver movement
						);

					


pros::Controller controller(pros::E_CONTROLLER_MASTER); // the controller that will be used for driving

//global variables
//TODO is auto_started still needed?
bool g_auton_started = false; // determines whether the autonamous code has been started
bool g_op_control_started = false; // determines whether the driver code has been started

// get and set in autonomous()
int g_current_auton_select = 2; // determines which auton path is going to be run

// set in color_select(), get in color_sorting() and autonomous()
int g_team_color = BLUE; //red = 1, blue = -1

// set in color_sorting() and get in autonomous()
float g_intake_direction = 1;

//pneumatics toggles
float g_ears_state = false;
float g_scraper_state = false;
float g_hood_state = false;


float distance_kp = 8, distance_ki = 0, distance_kd = 25;
float heading_correction_kp = 8, heading_correction_ki = 0, heading_correction_kd = 70;
int max_slew_accel_fwd = 4, max_slew_decel_rev = 4;

float front_distance_from_center = 4.5;

// tool to check if 2 values are "close enough" to each other
bool check_equal(float val1, float val2, int residual)
{
	if (fabs(val2-val1) < residual) // if the difference is small enough
	{
		return true;
	}
	else
	{
		return false;
	}
}

// tool to change heading based on team color
// DOES NOT WORK

/*
void ramsete(float target_x, float target_y, float target_theta, float input_v, float input_curv)
{

	float beta = 2; // greater than 0
	float zeta = 0.7; // between 0 and 1
	float  input_w = input_v * input_curv;
	lemlib::Pose abs_pose_err = lemlib::Pose(target_x, target_y, target_theta) - chassis.getPose();
	
	float actual_theta = chassis.getPose().theta;

	lemlib::Pose relative_pose_err = lemlib::Pose(abs_pose_err.x*cos(actual_theta) + abs_pose_err.y*sin(actual_theta), 
													-abs_pose_err.x*sin(actual_theta) + abs_pose_err.y*cos(actual_theta), 
													abs_pose_err.theta);
	
	float k = 2 * zeta * sqrt(pow(input_v, 2) + beta*pow(input_w, 2));

	float output_v = input_v*cos(relative_pose_err.theta) + k*relative_pose_err.x;
	float output_w = input_w + k*relative_pose_err.theta + beta*input_v*sin(relative_pose_err.theta)*relative_pose_err.y/relative_pose_err.theta;

	float linear_velocity = output_v*308;

	float left_velocity = linear_velocity + output_w*30;
	float right_velocity = linear_velocity - output_w*30;
	
	left_motor_group.move_velocity(left_velocity);
	right_motor_group.move_velocity(right_velocity);
}
*/

float set_limit(float input, float limit)
{
	if (input > limit)
		input = limit;
	else if(input < -limit)
		input = -limit;
	return input;
}

void intake_high(float volts)
{
	intake_bottom.move(volts);
	intake_top.move(volts);
	intake_index.move(volts);
}
void intake_middle(float volts)
{
	intake_bottom.move(volts);
	intake_top.move(volts);
	intake_index.move(-volts);
}
void intake_low(float volts)
{
	intake_bottom.move(-volts);
	intake_top.move(-volts);
	intake_index.move(-volts);
}

std::vector<lemlib::Pose> poses_holder;

/*
void distance_set_odom(int range)
{
	
	lemlib::Pose current_pose = chassis.getPose();
	float current_x = current_pose.x; // x to use in calculations
	float current_y = current_pose.y; // y to use in calculations
	float current_angle = (90-current_pose.theta)*M_PI/180; // convert theta to cartetian angles


	// array to store the new x and y values
 	float new_x;
  	float new_y;
	
	// angle of laser out of left sensor

	float front_angle = current_angle;
	float front_distance_y = (distance_front.get()*0.0394+front_distance_from_center)*sin(front_angle)+0;
	float front_distance_x = (distance_front.get()*0.0394+front_distance_from_center)*cos(front_angle)+0;



	// front sensor

		//top wall
		if (check_equal(current_y, 72-front_distance_y, range) && front_distance_y < 72)
		{
			new_y = 72-front_distance_y;
		}
	 
	  //right wall
	   if (check_equal(current_x, 72-front_distance_x, range) && front_distance_x < 72)
		{
			new_x = 72-front_distance_x;
	}
	   
	 //bottom wall
	   if (check_equal(current_y, -72+front_distance_y, range) && front_distance_y < 72)
		{
			new_y = -72+front_distance_y;
	}
	
	   //left wall
	   if (check_equal(current_x, -72+front_distance_x, range) && front_distance_x < 72)
		{
			new_x = -72+front_distance_x;
	}



		if(new_x != NULL)
		{
			chassis.setPose(new_x, current_pose.y, current_pose.theta);
			ears.set_value(true);
		}
		if(new_y != NULL)
		{
			chassis.setPose(current_pose.x, new_y, current_pose.theta);
		}
}
*/
void mtp_v_cancel(float x, float y, float final_v, bool ballpile = false, bool forward = true, float max_speed = 127, float min_speed = 0, 
				  float timeout = 5000)
{
	chassis.moveToPoint(x, y, timeout, {.forwards = forward, .maxSpeed = max_speed, .minSpeed = min_speed});
	
	float target_theta_rot = atan2(y, x);
	float rotated_target_y = -sin(target_theta_rot) * x + cos(target_theta_rot) * y;

	float rotated_current_y;
	
	float dist = hypot(x-chassis.getPose().x, y-chassis.getPose().y);

	chassis.waitUntil(dist/2);

	float max_velocity = 550;
	float right_velocity = 550;
	float left_velocity = 550;
	float final_v_rpm = final_v/100 * max_velocity;

	while(left_velocity > final_v_rpm || right_velocity > final_v_rpm)
	{
		right_velocity = fabs(left_motor_group.get_actual_velocity(0));
		left_velocity = fabs(right_motor_group.get_actual_velocity(0));

		if(ballpile)
		{
			dist = hypot(x-chassis.getPose().x, y-chassis.getPose().y);
			if(dist < 13)
			{
				scraper.set_value(true);
			}
		}
		pros::Task::delay(10);
	}
	if(ballpile) {scraper.set_value(false);}
	chassis.cancelMotion();

}

void ttp_v_cancel(float x, float y, float final_v, bool forward = true, 
				  int max_speed = 127, int min_speed = 0, float timeout = 5000)
{
	chassis.turnToPoint(x, y, timeout, {.forwards = forward, .maxSpeed = max_speed, .minSpeed = min_speed});
	float dist = fabs(chassis.getPose().theta -180/M_PI*atan2(y-chassis.getPose().y, x-chassis.getPose().x));
	pros::Task::delay(100);

	float max_velocity = 550;
	float right_velocity = 550;
	float left_velocity = 550;
	float final_v_rpm = final_v/100 * max_velocity;

	while(left_velocity > final_v_rpm || right_velocity > final_v_rpm)
	{
		right_velocity = fabs(left_motor_group.get_actual_velocity(0));
		left_velocity = fabs(right_motor_group.get_actual_velocity(0));

		pros::Task::delay(10);
	}
	ears.set_value(true);

	chassis.cancelMotion();

}


void stp_v_cancel(float x, float y, float final_v, char lockchar, bool forward = true, 
				  float max_speed = 127, float min_speed = 0, float timeout = 5000)
{
	lemlib::DriveSide lockside; 
	if(lockchar == 'l')
	{
		lockside = lemlib::DriveSide::LEFT;
	}
	else if(lockchar == 'r')
	{
		lockside = lemlib::DriveSide::RIGHT;
	}

	chassis.swingToPoint(x, y, lockside, timeout, {.forwards = forward, .maxSpeed = max_speed, .minSpeed = min_speed});
	float dist = fabs(chassis.getPose().theta -180/M_PI*atan2(y-chassis.getPose().y, x-chassis.getPose().x));
	pros::Task::delay(100);

	float max_velocity = 550;
	float right_velocity = 550;
	float left_velocity = 550;
	float final_v_rpm = final_v/100 * max_velocity;

	while(left_velocity > final_v_rpm || right_velocity > final_v_rpm)
	{
		right_velocity = fabs(left_motor_group.get_actual_velocity(0));
		left_velocity = fabs(right_motor_group.get_actual_velocity(0));

		pros::Task::delay(10);
	}

	chassis.cancelMotion();

}

void matchload_shake(int time)
{
	chassis.cancelMotion();
	float field_side = fabs(chassis.getPose().x)/chassis.getPose().x;

	for(int i = 0; i < time/400; i++)
	{
	chassis.moveToPoint(chassis.getPose().x+1*field_side, chassis.getPose().y, 200, {}, false);
	chassis.moveToPoint(chassis.getPose().x-1*field_side, chassis.getPose().y, 200, {}, false);
	}

}

void extractPose(std::string address, int num)
{
	std::ifstream path(address, std::ios::in);
	//FILE* path = fopen(address.c_str(), "r");

	lemlib::Pose lastPose(0, 0, 0);
	lemlib::Pose thisPose(0, 0, 0);
	lemlib::Pose nextPose(0, 0, 0);

	poses_holder.clear();

	float poseX;
	float poseY;
	float speed = NULL;
	float heading = 0;
		//pros::screen::print(pros::E_TEXT_SMALL, 0, "p", chassis.getPose().x); // x
	
	std::string line = "";
	std::string token = "";
	std::string path_to_read = "#PATH-POINTS-START Path " + std::to_string(num);
    
	//pros::screen::print(pros::E_TEXT_SMALL, 0, "2", chassis.getPose().x); // x


	while(getline(path, line))
	{
		if(line == path_to_read)
		
		{
			//pros::screen::print(pros::E_TEXT_SMALL, 0, "got line", chassis.getPose().x); // x
			break;
		}
		//pros::screen::print(pros::E_TEXT_SMALL, 0, "%s", line); // x

	}
	getline(path, line);
	sscanf(line.c_str(), "%f, %f, %f", &poseX, &poseY, &speed);


	lastPose = lemlib::Pose(poseX, poseY, 0);
	
	getline(path, line);
	sscanf(line.c_str(), "%f, %f, %f", &poseX, &poseY, &speed);

	thisPose = lemlib::Pose(poseX, poseY, 0);

	getline(path, line);
	sscanf(line.c_str(), "%f, %f, %f", &poseX, &poseY, &speed);
	
	nextPose = lemlib::Pose(poseX, poseY, 0);

	heading = lemlib::sanitizeAngle(lemlib::radToDeg(-(lastPose.angle(nextPose)-M_PI_2)), false);
	if (fabs(heading) < 0.01) {heading = 0;}

	lastPose = lemlib::Pose(lastPose.x, lastPose.y, heading);
	thisPose = lemlib::Pose(thisPose.x, thisPose.y, heading);
	nextPose = lemlib::Pose(nextPose.x, nextPose.y, heading);
	poses_holder.push_back(lastPose);
	poses_holder.push_back(thisPose);

    while (getline(path, line)) 
	{


		getline(path, line);
		sscanf(line.c_str(), "%f, %f, %f", &poseX, &poseY, &speed);
		


		lastPose = thisPose;
		thisPose = nextPose;
		nextPose = lemlib::Pose(poseX, poseY, 0);
		heading = lemlib::sanitizeAngle(lemlib::radToDeg(-(lastPose.angle(nextPose)-M_PI_2)), false);
		if (fabs(heading) < 0.01) {heading = 0;}
		thisPose = lemlib::Pose(thisPose.x, thisPose.y, heading);
		poses_holder.push_back(thisPose);
		
		if(speed == 0)
		{
			break;
		}

    }
	nextPose = lemlib::Pose(nextPose.x, nextPose.y, heading);
	poses_holder.push_back(nextPose);
	path.close();
}

void followPath(float time_limit_msec, bool exit, float max_output, std::string address, int num, int dir) {

	//pros::screen::print(pros::E_TEXT_SMALL, 0, "0", chassis.getPose().x); // x


	extractPose(address, num);
		//pros::screen::print(pros::E_TEXT_SMALL, 0, "1", chassis.getPose().x); // x
	if (poses_holder.size() == 0) return;

	for(int i = 0; i < poses_holder.size(); i++)
	{
		lemlib::Pose poos = poses_holder[i];
		std::cout << "x: " << poos.x << " y: " << poos.y << " theta: " << poos.theta << std::endl;
	}

	double kpStanley = 14; // Stanley gain for lateral correction
	double kdStanley = 0;

	const double max_steering_angle = 179.0; // Max steering correction (degrees)

	const double stop_tolerance = 0; // Final stop distance

	float lookahead = 4;

	lemlib::PID pid_distance = lemlib::PID(distance_kp, distance_ki, distance_kd, 3, true);

	lemlib::PID pid_heading = lemlib::PID(heading_correction_kp, heading_correction_ki, heading_correction_kd, 1, true);
/*
	//pid_distance.update(0);
	//pid_heading.update(0);


	//pid_distance.setTarget(0);

	//pid_distance.setIntegralMax(3);

	//pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);

	//pid_distance.setSmallBigErrorDuration(50, 250);

	//pid_distance.setDerivativeTolerance(5);


	//pid_heading.setTarget(normalizeTarget(getInertialHeading()));

	//pid_heading.setIntegralMax(0);

	//pid_heading.setIntegralRange(1);

	//pid_heading.setSmallBigErrorTolerance(0, 0);

	//pid_heading.setSmallBigErrorDuration(0, 0);

	//pid_heading.setDerivativeTolerance(0);

	//pid_heading.setArrive(false);
   */

	float start_time = pros::millis(); 
	//pros::screen::print(pros::E_TEXT_SMALL, 4, "timer started"); // x

	size_t target_index = 0;

	double prev_left_output = 0;

	double prev_right_output = 0;

	double last_cross_track_error = 0;


	while (target_index < poses_holder.size() && (float)pros::millis() - start_time < time_limit_msec) {

		double robot_x = chassis.getPose().x;

		double robot_y = chassis.getPose().y;

		double robot_heading_deg = chassis.getPose().theta;

		double robot_heading_rad = lemlib::degToRad(robot_heading_deg);


		// Find closest pose

		double min_dist = 1e6;

		for (size_t i = target_index-lookahead; i < poses_holder.size(); i++) 
		{
			double dx = poses_holder[i].x - robot_x;

			double dy = poses_holder[i].y - robot_y;

			double dist = hypot(dx, dy);

			if (dist < min_dist) {

				min_dist = dist;

				target_index = i;
			}
		}
		
		target_index += lookahead;

		if (target_index >= poses_holder.size()-1) 
		{ 
			target_index = poses_holder.size()-1;  
			break; 
		}

		const lemlib::Pose target = poses_holder[target_index];

		double target_x = target.x;

		double target_y = target.y;

		double target_heading_deg = lemlib::sanitizeAngle(target.theta-90+90*dir, false);
		pros::screen::print(pros::E_TEXT_SMALL, 3, "%2f", target_heading_deg); // x

		double target_heading_rad = lemlib::degToRad(target_heading_deg);

		// distance2target

		double dx = target_x - robot_x;

		double dy = target_y - robot_y;

		double distance_error = hypot(dx, dy) + (poses_holder.size()-1- target_index);//dx*sin(robot_heading_rad)+dy*cos(robot_heading_rad);



		double speed = 127;//pid_distance.update(distance_error);


		//speed = clamp(speed, -max_output, max_output);
		speed = set_limit(speed, max_output);

		// Heading error

		//double heading_error = lemlib::sanitizeAngle(target_heading_rad - robot_heading_rad, true);
		double heading_error = lemlib::angleError(target_heading_rad, robot_heading_rad, true);//target_heading_rad - robot_heading_rad;

		// Cross track error

		double cross_track_error = dx * cos(target_heading_rad) - dy * sin(target_heading_rad);

		double cross_track_derivative = cross_track_error-last_cross_track_error;

		//correction

		double steering_rad = heading_error + atan2((kpStanley * cross_track_error + kdStanley * cross_track_derivative) * dir, speed);

		//steering_rad = clamp(steering_rad, -lemlib::degToRad(max_steering_angle), lemlib::degToRad(max_steering_angle));
		
		steering_rad = set_limit(steering_rad, lemlib::degToRad(max_steering_angle));

		double steering_deg = lemlib::radToDeg(steering_rad);

		// heading

		//pid_heading.update(lemlib::sanitizeAngle(robot_heading_deg + steering_deg, false));//setTarget(normalizeTarget(robot_heading_deg + steering_deg));
		
		double heading_correction = pid_heading.update(-steering_deg);

		heading_correction = set_limit(heading_correction, max_output);

		lemlib::Pose target_pose = {poses_holder[target_index].x, poses_holder[target_index].y, lemlib::degToRad(-poses_holder[target_index].theta+90)};
		lemlib::Pose next_target_pose = {poses_holder[target_index+1].x, poses_holder[target_index+1].y, lemlib::degToRad(-poses_holder[target_index+1].theta+90)};

		double curvature_scale = 0.05*1/fabs(lemlib::getCurvature(target_pose, next_target_pose));
		if(curvature_scale > 1) { curvature_scale = 1;}
		if(curvature_scale < 0.2) { curvature_scale = 0.2;}

		pros::screen::print(pros::E_TEXT_SMALL, 4, "%2f", curvature_scale);
		// output

		double left_output = (speed*dir - heading_correction)*curvature_scale;

		double right_output = (speed*dir + heading_correction)*curvature_scale;
		
		//pros::screen::print(pros::E_TEXT_SMALL, 3, "%d", speed); // x

		// slew
/*
		if (left_output - prev_left_output > max_slew_accel_fwd) {

		left_output = prev_left_output + max_slew_accel_fwd;

		} else if (prev_left_output - left_output > max_slew_decel_rev) {

		left_output = prev_left_output - max_slew_decel_rev;

		}
*/
		//left_output = set_limit(left_output, max_slew_accel_fwd+prev_left_output);
/*
		if (right_output - prev_right_output > max_slew_accel_fwd) {

		right_output = prev_right_output + max_slew_accel_fwd;

		} else if (prev_right_output - right_output > max_slew_decel_rev) {

		right_output = prev_right_output - max_slew_decel_rev;

		}
		*/
		//right_output = set_limit(right_output, max_slew_accel_fwd + prev_right_output);

		prev_left_output = left_output;

		prev_right_output = right_output;

		last_cross_track_error = cross_track_error;


		// Clamp and drive
/*
		//scaleToMax(left_output, right_output, max_output);
		
		if (left_output > max_output)
			left_output = max_output;
		else if(left_output < -max_output)
			left_output = -max_output;

		if (right_output > max_output)
			right_output = max_output;
		else if(right_output < -max_output)
			right_output = -max_output;
*/
		left_output = set_limit(left_output, max_output);
		right_output = set_limit(right_output, max_output);

		//driveChassis(left_output, right_output);
		left_motor_group.move(left_output);
		right_motor_group.move(right_output);

		// final stop
		if ((exit && target_index == poses_holder.size() - 1) && distance_error < stop_tolerance) {

		break;

		}

		pros::Task::delay(10);

	}

	left_motor_group.move(0);
	right_motor_group.move(0);
	
	if(dir == 1) 
	{chassis.moveToPoint(poses_holder[target_index].x, poses_holder[target_index].y, 1000, {.maxSpeed = max_output}, false);}
	else if(dir == -1) 
	{chassis.moveToPoint(poses_holder[target_index].x, poses_holder[target_index].y, 1000, {.forwards = false, .maxSpeed = max_output}, false);}

}

//WIP function to set the robot's position based on the distance sensors

void brain_data_output()
{
	// print position to brain screen
    pros::Task screen_task([&]() {

        while (true) {
            // print robot location to the brain screen
            pros::screen::print(pros::E_TEXT_SMALL, 0, "X: %f", chassis.getPose().x); // x
            pros::screen::print(pros::E_TEXT_SMALL, 1, "Y: %f", chassis.getPose().y); // y
            pros::screen::print(pros::E_TEXT_SMALL, 2, "Theta: %f", chassis.getPose().theta); // heading
			
			float angle_error = chassis.getPose().theta -180/M_PI*atan2(24-chassis.getPose().y, 24-chassis.getPose().x);
            pros::screen::print(pros::E_TEXT_SMALL, 5, "Angle Error: %f", angle_error);

            // delay to save resources
            pros::delay(20);
        }
		
	});
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

	chassis.calibrate(); // calibrate sensor
	vertical_encoder.set_position(0); // set the vertical encoder to 0

}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() 
{

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	g_auton_started = true;
	std::cout<< "zero" <<std::endl;
	

	brain_data_output(); // print values to the brain screen
	/*
	{
	chassis.setPose(48, -18, 180); 
	//chassis.moveToPoint(48, -49, 3000, {.maxSpeed = 127});
	scraper.set_value(true);
	hood.set_value(false);
	mtp_v_cancel(48, -49, 12);
	//chassis.turnToPoint(72, -49, 1000, {.maxSpeed = 127});
	ttp_v_cancel(60, -49, 15);
	intake_high(127);
	chassis.moveToPoint(60, -49, 1000, {.maxSpeed = 127}, false);
	pros::Task::delay(400);
	chassis.moveToPoint(30, -49, 800, {.forwards = false, .maxSpeed = 127}, false);
	scraper.set_value(false);
	chassis.setPose(33, -48, chassis.getPose().theta); 
	hood.set_value(true);
	pros::Task::delay(1000);
	

	//chassis.swingToPoint(20, -24,lemlib::DriveSide::LEFT, 3000, {.maxSpeed = 127}, false);
	stp_v_cancel(20, -24, 15, 'l');
	hood.set_value(false);
	//chassis.turnToPoint(28, -28, 1500, {.maxSpeed = 60});

	//chassis.moveToPoint(28, -28, 3000, {.maxSpeed = 127});
	intake_middle(0);
	intake_high(127);

	chassis.moveToPoint(20, -24, 300, {});
	chassis.moveToPoint(20, -24, 500, {.maxSpeed = 50});
	
	chassis.turnToPoint(9, -13, 2000, {.forwards = false});

	//chassis.moveToPoint(9, -13, 3000, {.forwards = false}, false);
	mtp_v_cancel(9, -13, 10, false, false);
	chassis.moveToPoint(0, -4, 700, {.forwards = false, .maxSpeed = 30});

	intake_high(0);
	intake_middle(127);
	pros::Task::delay(1000);
	//chassis.swingToPoint(20, 24,lemlib::DriveSide::LEFT, 3000, {.maxSpeed = 127});
	stp_v_cancel(19, 16, 15, 'l');
	//distance_set_odom(6);
	
	intake_middle(0);
	intake_high(127);
	//chassis.moveToPoint(20, 20, 3000, {.maxSpeed = 127});
	mtp_v_cancel(19, 16, 15);

	//chassis.swingToPoint(44, 48, lemlib::DriveSide::RIGHT, 500,{}, false);
	stp_v_cancel(44, 49, 15, 'r');

	//distance_set_odom(6);
	//chassis.moveToPoint(44, 48, 3000,{}, false);
	mtp_v_cancel(44, 49, 15);

	scraper.set_value(true);

	//chassis.turnToPoint(72, 48, 1000, {.maxSpeed = 127});
	ttp_v_cancel(72, 49, 15);
	//distance_set_odom(6);

	chassis.moveToPoint(57, 49, 800, {.maxSpeed = 127}, false);
	pros::Task::delay(400);
	chassis.moveToPoint(28, 49, 800, {.forwards = false, .maxSpeed = 127}, false);
	scraper.set_value(false);
	hood.set_value(true);
	}
*/
	
// left middle + long
	chassis.setPose(45, -12, 270); 
	intake_high(127);
	hood.set_value(false);
	stp_v_cancel(24, -24, 15, 'l');
	mtp_v_cancel(24, -24, 25, true);

	ttp_v_cancel(6, -43, 10, 'l');
	mtp_v_cancel(6, -43, 15);
	scraper.set_value(true);
	ttp_v_cancel(24, -24, 15, false);
	mtp_v_cancel(24, -24, 15, false, false);
	ttp_v_cancel(14, -14, 10, false);
	mtp_v_cancel(14, -14, 15, false, false);
	chassis.moveToPoint(-72, 72, 1000, {.forwards = false, .maxSpeed = 20});
	intake_high(0);
	intake_middle(127);
	pros::Task::delay(1000);
	intake_middle(0);
	mtp_v_cancel(48, -47, 15);
	intake_high(127);
	ttp_v_cancel(67, -47, 10);
 	chassis.moveToPoint(65, -47, 300, {.maxSpeed = 127}, false);
	chassis.moveToPoint(65, -47, 500, {.maxSpeed = 60}, false);
	chassis.moveToPoint(65, -47, 400, {.maxSpeed = 20}, false);
	chassis.moveToPoint(27, -47, 800, {.forwards = false, .maxSpeed = 127}, false);
	chassis.moveToPoint(27, -47, 1700, {.forwards = false, .maxSpeed = 20});
	hood.set_value(true);
/*	
	//part 1

	chassis.setPose(48, 18, 0); 
	scraper.set_value(true);
	hood.set_value(false);
	mtp_v_cancel(48, 49, 12);
	ttp_v_cancel(72, 49, 15);
	intake_high(127);
	chassis.moveToPoint(65, 48, 300, {.maxSpeed = 127}, false);
	chassis.moveToPoint(65, 48, 500, {.maxSpeed = 60}, false);
	chassis.moveToPoint(65, 48, 900, {.maxSpeed = 20}, false);

	//pros::Task::delay(900);
	chassis.moveToPoint(30, 48, 800, {.forwards = false, .maxSpeed = 127}, false);
	chassis.moveToPoint(30, 48, 1700, {.forwards = false, .maxSpeed = 20});

	scraper.set_value(false);
	hood.set_value(true);
	pros::Task::delay(1700);
	chassis.setPose(33, 48, chassis.getPose().theta); 


	stp_v_cancel(25, 24, 15, 'r');
	hood.set_value(false);
	mtp_v_cancel(25, 24, 15, true);
	stp_v_cancel(-24, 24, 20, 'r');
	mtp_v_cancel(-24, 24, 15, true);


	stp_v_cancel(-40, 48, 20, 'r');
	mtp_v_cancel(-40, 48, 10);
	ttp_v_cancel(-30, 48, 15, false);
	chassis.moveToPoint(30, 48, 800, {.forwards = false}, false);
	chassis.moveToPoint(30, 48, 1500, {.forwards = false, .maxSpeed = 20});

	hood.set_value(true);
	pros::Task::delay(1500);

	//part 2

	chassis.setPose(-33, 48, chassis.getPose().theta); 
	intake_high(127);
	scraper.set_value(true);
	chassis.moveToPoint(-65, 48, 400);
	chassis.moveToPoint(-65, 48, 800, {.maxSpeed = 60});
	chassis.moveToPoint(-65, 48, 900, {.maxSpeed = 30});

	hood.set_value(false);

	//pros::Task::delay(1700);
	chassis.moveToPoint(-60, 48, 300, {.maxSpeed = 60});

	stp_v_cancel(-15.5, 14.5, 15, 'l', false);
	chassis.moveToPoint(-15.5, 14.5, 3000, {.forwards = false});
	scraper.set_value(false);

	chassis.moveToPoint(72, -72, 3000, {.forwards = false, .maxSpeed = 40});
	intake_middle(90);	
	pros::Task::delay(3000);
	intake_middle(0);

	intake_high(127);
	stp_v_cancel(-23, -21, 10, 'l');
	mtp_v_cancel(-23, -21, 15, true);
	stp_v_cancel(-48, -49, 15, 'r');
	scraper.set_value(true);

	mtp_v_cancel(-48, -49, 15);
	ttp_v_cancel(-65, -49, 10);
	chassis.moveToPoint(-67, -49, 400);
	chassis.moveToPoint(-67, -49, 400, {.maxSpeed = 60});
	chassis.moveToPoint(-67, -49, 500, {.maxSpeed = 30});

	//pros::Task::delay(900);
	chassis.moveToPoint(-30, -49, 800, {.forwards = false, .maxSpeed = 127}, false);
	chassis.moveToPoint(-30, -49, 1700, {.forwards = false, .maxSpeed = 20});
	scraper.set_value(false);
	hood.set_value(true);
	pros::Task::delay(1700);


	//part 3

	intake_high(127);
	chassis.setPose(-33, -48, 270); 
	chassis.moveToPoint(-36, -48, 300, {}, false);
	stp_v_cancel(24, -56, 10, 'l');
	mtp_v_cancel(24, -56, 10);
	scraper.set_value(true);

	stp_v_cancel(48, -46, 10, 'l');
	mtp_v_cancel(48, -46, 15);
	ttp_v_cancel(67, -46, 10);
	chassis.moveToPoint(67, -46, 2000, {.maxSpeed = 60});
	//pros::Task::delay(900);
	chassis.moveToPoint(30, -46, 800, {.forwards = false, .maxSpeed = 127}, false);
	chassis.moveToPoint(30, -46, 1700, {.forwards = false, .maxSpeed = 20});	
	scraper.set_value(false);
	hood.set_value(true);
	pros::Task::delay(1700);


	// part 4

	chassis.setPose(33, -48, chassis.getPose().theta); 

	stp_v_cancel(63, -24, 15, 'l');
	mtp_v_cancel(63, -24, 15);
	ttp_v_cancel(63, 0, 10, false);
	chassis.moveToPoint(63, 0, 5500, {.forwards = false});
	intake_high(0);
*/
	//followPath(50000, true, 100, "win_point_pb.txt", 1, 1);
	//followPath(50000, true, 127, "win_point_pb.txt", 2, -1);
	//followPath(50000, true, 127, "win_point_pb.txt", 3, 1);
	//followPath(50000, true, 127, "win_point_pb.txt", 4, -1);


	// set position to x:0, y:0, heading:0
    // turn to face heading 90 with a very long timeout
	//chassis.setPose(0,0,0);
    //chassis.turnToHeading(90, 10000);
	//chassis.moveToPoint(0, 36, 9999);

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	
	//followPath(5000, true, 127, "stanley.txt", 1);
	intake_high(0);
	controller.print(0, 0, "program started");
	g_auton_started = true;
	g_op_control_started = true;
	brain_data_output();


// this task runs the lights during the opc period
//////////////pros::Task run_lights(run_lights_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "run_lights");

	// this is a TASK. they run independently of the rest of the code. the brain will cycle through the code and update each task sequencially
	// the code has a lot of tasks in it. Every task is a loop that will continuously run untill the code is ended
	// tasks do not end at the end of auton
	// when the brain reads the code, when it reaches a task it will break it off and then keep reading through the code

	// this task updates inputs from the controller
/*
	pros::Task odom_reset_task([&]() {
		while (true)
		{
			distance_set_odom(5);
			pros::Task::delay(100);
		}
	});
*/
	pros::Task driver_systems_task([&]() {
	while (true)
	{
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intake_bottom.move(127);
			intake_top.move(127);
			intake_index.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			intake_bottom.move(-127);
			intake_top.move(-127);
			intake_index.move(-127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			intake_bottom.move(127);
			intake_top.move(127);
			intake_index.move(-127);
		}
		else
		{
			intake_bottom.move(0);
			intake_top.move(0);
			intake_index.move(0);
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			if(g_ears_state)
			{
				ears.set_value(false);
				g_ears_state = false;
			}
			else
			{

				ears.set_value(true);
				g_ears_state = true;

			}
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			if(g_hood_state)
			{
				hood.set_value(false);
				g_hood_state = false;
			}
			else
			{
				hood.set_value(true);
				g_hood_state = true;

			}
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
		{
			if(g_scraper_state)
			{
				scraper.set_value(false);
				g_scraper_state = false;
			}
			else
			{
				scraper.set_value(true);
				g_scraper_state = true;
			}
		}
		
		pros::Task::delay(25);
		}
	});



    while (true) {
		// get left y and right x positions
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.curvature(leftY, rightX);
		// delay to save resources
        pros::delay(25);
    }
}	
