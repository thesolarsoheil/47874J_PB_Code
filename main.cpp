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


//pros::Motor teammate_motor(4, pros::MotorGears::blue); // back right motor on port 5


//intake_motors
pros::Motor intake_bottom(8, pros::MotorGears::blue); // intake motor on port 19
pros::Motor intake_top(10, pros::MotorGears::green); // lift motor on port 12
pros::Motor intake_index(-9, pros::MotorGears::green); // lift motor on port 12

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

pros::ADIDigitalOut park(4);
pros::ADIDigitalOut ears(3);
pros::ADIDigitalOut scraper(2);
pros::ADIDigitalOut hood(1);

// left tracking wheel encoder
pros::Rotation vertical_encoder(-17);

pros::Distance distance_left(5);
pros::Distance distance_right(4);

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
lemlib::ControllerSettings lateral_controller_large(19, // proportional gain (kP)19
                                              0.15, // integral gain (kI)0.15
                                              150, // derivative gain (kD)150
                                              2, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              90//250 // maximum acceleration (slew)
);

lemlib::ControllerSettings lateral_controller_small(18, // proportional gain (kP)19
                                              0.35, // integral gain (kI)0.15
                                              150, // derivative gain (kD)150
                                              2, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0//250 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller_small(2.9, // proportional gain (kP)//3.2
                                              0.07, // integral gain (kI)//0.25
                                              20,// derivative gain (kD)//28
                                              4, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller_large(3.2, // proportional gain (kP)//3.2
                                              0.15, // integral gain (kI)//0.25
                                              28,// derivative gain (kD)//28
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

// create the chassis_large
lemlib::Chassis chassis_large(drivetrain_6m, // drivetrain settings
                        lateral_controller_large, // lateral PID settings
                        angular_controller_small, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, // forward/backward driver movement
                        &steer_curve // left/right driver movement
						);

lemlib::Chassis chassis_small(drivetrain_6m, // drivetrain settings
                        lateral_controller_small, // lateral PID settings
                        angular_controller_small, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, // forward/backward driver movement
                        &steer_curve // left/right driver movement
						);

lemlib::Chassis chassis_large_turn(drivetrain_6m, // drivetrain settings
                        lateral_controller_small, // lateral PID settings
                        angular_controller_large, // angular PID settings
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
int chosen_auto = 0; // determines which auton path is going to be run

// set in color_select(), get in color_sorting() and autonomous()
int g_team_color = BLUE; //red = 1, blue = -1

// set in color_sorting() and get in autonomous()
float g_intake_direction = 1;

//pneumatics toggles
float g_ears_state = false;
float g_scraper_state = false;
float g_hood_state = false;
float g_park_state = false;

float top_target_speed = 0, bottom_target_speed = 0, index_target_speed = 0;

float distance_kp = 8, distance_ki = 0, distance_kd = 25;
float heading_correction_kp = 8, heading_correction_ki = 0, heading_correction_kd = 70;
int max_slew_accel_fwd = 4, max_slew_decel_rev = 4;

float side_distance_from_center = 6.3;

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
	lemlib::Pose abs_pose_err = lemlib::Pose(target_x, target_y, target_theta) - chassis_large.getPose();
	
	float actual_theta = chassis_large.getPose().theta;

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
	float output = input;
	if (output > limit)
		output = limit;
	else if(input < -limit)
		output = -limit;
	return output;
}
float set_upperlower_limit(float input, float ceiling, float floor)
{
	float output = input;
	if (output > ceiling)
		output = ceiling;
	else if(input < floor)
		output = floor;
	return output;
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
	intake_index.move(-volts*0.5);

}

void intake_middle_skills(float volts)
{
	intake_bottom.move(volts);

	intake_top.move(volts*0.35);
	intake_index.move(-volts*0.25);

}

void intake_low(float volts)
{
	intake_bottom.move(-volts);
	intake_top.move(-volts);
	intake_index.move(-volts);
}

std::vector<lemlib::Pose> poses_holder;

void simple_dist_reset()
{
	float dist_to_center = 0;
	if(chassis_large.getPose().x > 0 && chassis_large.getPose().y > 0)
	{
		dist_to_center = distance_left.get()*0.0394+side_distance_from_center;
		chassis_large.setPose(chassis_large.getPose().x, 72-dist_to_center, chassis_large.getPose().theta);
	}
	else if(chassis_large.getPose().x < 0 && chassis_large.getPose().y > 0)
	{
		dist_to_center = distance_right.get()*0.0394+side_distance_from_center;
		chassis_large.setPose(chassis_large.getPose().x, 72-dist_to_center, chassis_large.getPose().theta);
	}
	else if(chassis_large.getPose().x < 0 && chassis_large.getPose().y < 0)
	{
		dist_to_center = distance_left.get()*0.0394+side_distance_from_center;
		chassis_large.setPose(chassis_large.getPose().x, -72+dist_to_center, chassis_large.getPose().theta);
	}
	else if(chassis_large.getPose().x > 0 && chassis_large.getPose().y < 0)
	{
		dist_to_center = distance_right.get()*0.0394+side_distance_from_center;
		chassis_large.setPose(chassis_large.getPose().x, -72+dist_to_center, chassis_large.getPose().theta);
	}

	
}


/*
void distance_set_odom(int range)
{
	
	lemlib::Pose current_pose = chassis_large.getPose();
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
			chassis_large.setPose(new_x, current_pose.y, current_pose.theta);
			ears.set_value(true);
		}
		if(new_y != NULL)
		{
			chassis_large.setPose(current_pose.x, new_y, current_pose.theta);
		}
}
*/
void mtp_v_cancel(float x, float y, float final_v, bool ballpile = false, bool forward = true, float max_speed = 127, 
				  float timeout = 5000)
{

	while ((chassis_large.isInMotion() || chassis_large_turn.isInMotion()) || chassis_small.isInMotion())
	{
		pros::Task::delay(10);
	}

	float start_x = chassis_large.getPose().x;
	float slope = (y-chassis_large.getPose().y)/(x-chassis_large.getPose().x);
	float line_y = -1/slope*(chassis_large.getPose().x - x) + y;
	

	bool above_below = false;
	if(y < chassis_large.getPose().y)
	{
		above_below = true;
	}
	else
	{
		above_below = false;
	}

	bool crossed_line = false;

	float final_v_volt = final_v/100 * 127;

	chassis_large.moveToPoint(x, y, timeout, {.forwards = forward, .maxSpeed = max_speed, .minSpeed = final_v_volt});

	pros::Task::delay(100);
	float dist = hypot(x-chassis_large.getPose().x, y-chassis_large.getPose().y);

	while((!crossed_line ) && chassis_large.isInMotion())
	{

		if(above_below && -1/slope*(chassis_large.getPose().x - x) + y >= chassis_large.getPose().y)
		{
			crossed_line = true;
		}
		else if(!above_below && -1/slope*(chassis_large.getPose().x - x) + y <= chassis_large.getPose().y)
		{
			crossed_line = true;
		}

		if(ballpile)
		{
			dist = hypot(x-chassis_large.getPose().x, y-chassis_large.getPose().y);
			if(dist < 13)
			{
				scraper.set_value(true);
			}
		}
		pros::Task::delay(10);
	}

	if(ballpile) {scraper.set_value(false);}
	chassis_large.cancelMotion();

}

void ttp_v_cancel(float x, float y, float final_v, bool forward = true, 
				  int max_speed = 127, int min_speed = 0, float timeout = 5000)
{
	while ((chassis_large.isInMotion() || chassis_large_turn.isInMotion()) || chassis_small.isInMotion())
	{
		pros::Task::delay(10);
	}

	chassis_large.turnToPoint(x, y, timeout, {.forwards = forward, .maxSpeed = max_speed, .minSpeed = min_speed});
	float dist = fabs(chassis_large.getPose().theta -180/M_PI*atan2(y-chassis_large.getPose().y, x-chassis_large.getPose().x));
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

	chassis_large.cancelMotion();

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

	chassis_large.swingToPoint(x, y, lockside, timeout, {.forwards = forward, .maxSpeed = max_speed, .minSpeed = min_speed});
	float dist = fabs(chassis_large.getPose().theta -180/M_PI*atan2(y-chassis_large.getPose().y, x-chassis_large.getPose().x));
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

	chassis_large.cancelMotion();

}

void moveto_matchload(int quadrant, int balls)
{
	int x = 0;
	int y = 0;
	int theta = 0;

	switch (quadrant)
	{
	case 1:
		x = 67;
		y = 48;
		theta = 90;
		break;
	
	case 2:
		x = -67;
		y = 48;
		theta = 270;
		break;
	
	case 3: 
		x = -67;
		y = -48;
		theta = 270;
		break;
	
	case 4:
		x = 67;
		y = -48;
		theta = 90;
		break;
	}
	chassis_large.moveToPose(x, y, theta, 5000);
	
	while(hypot(chassis_large.getPose().x - x, chassis_large.getPose().y - y) > 16)
	{
		pros::Task::delay(10);
	}
	chassis_large.cancelMotion();

	if(x > 0)
	{
		x += 7;
	}
	else
	{
		x -= 7;
	}

	chassis_large.moveToPose(x, y, theta, 5000, {.maxSpeed = 60});
	float max_velocity = 550;
	float right_velocity = 550;
	float left_velocity = 550;
	float final_v_rpm = 30;

	while(left_velocity > final_v_rpm || right_velocity > final_v_rpm)
	{
		right_velocity = fabs(left_motor_group.get_actual_velocity(0));
		left_velocity = fabs(right_motor_group.get_actual_velocity(0));
		pros::Task::delay(10);
	}
	chassis_large.cancelMotion();

	if(balls == 3)
	{
	chassis_large.moveToPoint(0, y, 30, {.forwards = false, .maxSpeed = 30}, false);

	chassis_large.moveToPoint(x, y, 400, {.maxSpeed = 20}, false);
	}
	else if(balls == 6)
	{
	chassis_large.moveToPoint(0, y, 30, {.forwards = false, .maxSpeed = 30}, false);
	chassis_large.moveToPoint(x, y, 400, {.maxSpeed = 20}, false);
	for(int i = 0; i < 4; i++)
	{
		chassis_large.moveToPoint(0, y, 30, {.forwards = false, .maxSpeed = 20}, false);
		chassis_large.moveToPoint(x, y, 200, {.maxSpeed = 20}, false);
	}
	}

}

void quarterstack()
{

	intake_high(127);
	pros::Task::delay(400);
	float intake_speed = intake_bottom.get_actual_velocity();
	float last_intake_speed = intake_bottom.get_actual_velocity();
	float timer = 0;
	while ((fabs(intake_speed - last_intake_speed) < 60) && (timer < 2500))
	{
		last_intake_speed = intake_speed;
		intake_speed = intake_bottom.get_actual_velocity();
		pros::Task::delay(10);
		timer +=10;
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
		//pros::screen::print(pros::E_TEXT_SMALL, 0, "p", chassis_large.getPose().x); // x
	
	std::string line = "";
	std::string token = "";
	std::string path_to_read = "#PATH-POINTS-START Path " + std::to_string(num);
    
	//pros::screen::print(pros::E_TEXT_SMALL, 0, "2", chassis_large.getPose().x); // x


	while(getline(path, line))
	{
		if(line == path_to_read)
		
		{
			//pros::screen::print(pros::E_TEXT_SMALL, 0, "got line", chassis_large.getPose().x); // x
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

	//pros::screen::print(pros::E_TEXT_SMALL, 0, "0", chassis_large.getPose().x); // x


	extractPose(address, num);
		//pros::screen::print(pros::E_TEXT_SMALL, 0, "1", chassis_large.getPose().x); // x
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


	float start_time = pros::millis(); 
	//pros::screen::print(pros::E_TEXT_SMALL, 4, "timer started"); // x

	size_t target_index = 0;

	double prev_left_output = 0;

	double prev_right_output = 0;

	double last_cross_track_error = 0;


	while (target_index < poses_holder.size() && (float)pros::millis() - start_time < time_limit_msec) {

		double robot_x = chassis_large.getPose().x;

		double robot_y = chassis_large.getPose().y;

		double robot_heading_deg = chassis_large.getPose().theta;

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

		//drivechassis_large(left_output, right_output);
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
	{chassis_large.moveToPoint(poses_holder[target_index].x, poses_holder[target_index].y, 1000, {.maxSpeed = max_output}, false);}
	else if(dir == -1) 
	{chassis_large.moveToPoint(poses_holder[target_index].x, poses_holder[target_index].y, 1000, {.forwards = false, .maxSpeed = max_output}, false);}

}

//WIP function to set the robot's position based on the distance sensors

void auto_selector()
{
	pros::Task auton_seletor_task([&]() {	
	pros::screen_touch_status_s_t status;

	while(true)
	{
	 	status = pros::screen::touch_status();
		if(status.press_count > 0);
		{
			chosen_auto = 1+(status.press_count % 5);
		}
		pros::Task::delay(10);
	}
		
	});
}

void brain_data_output()
{
	// print position to brain screen
    pros::Task screen_task([&]() {

        while (true) {
            // print robot location to the brain screen
            pros::screen::print(pros::E_TEXT_SMALL, 0, "X: %f", chassis_large.getPose().x); // x
            pros::screen::print(pros::E_TEXT_SMALL, 1, "Y: %f", chassis_large.getPose().y); // y
            pros::screen::print(pros::E_TEXT_SMALL, 2, "Theta: %f", chassis_large.getPose().theta); // heading
            pros::screen::print(pros::E_TEXT_SMALL, 3, "Auto: %d", chosen_auto); // auto
			float distance_reading = distance_left.get()*0.0394;
            pros::screen::print(pros::E_TEXT_SMALL, 4, "dist left: %f", distance_reading); // dist sensor
			distance_reading = distance_right.get()*0.0394;            
			pros::screen::print(pros::E_TEXT_SMALL, 5, "dist right: %f", distance_reading); // dist sensor
			

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
	chosen_auto = 5;

	chassis_large.calibrate(); // calibrate sensor
	vertical_encoder.set_position(0); // set the vertical encoder to 0
	brain_data_output();
	
	auto_selector(); // select the auton mode

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
	pros::Task::delay(20);
	


	chosen_auto = 5; // for testing purposes only

	if(chosen_auto == 1)
	{

		chassis_large.setPose(48, -18, 180); 
		
		scraper.set_value(true);
		hood.set_value(false);

		mtp_v_cancel(48, -46, 10);

		ttp_v_cancel(66, -48, 15);
		intake_high(127);

		moveto_matchload(4, 3);

		chassis_large.moveToPose(28, -47, 90, 1200, {.forwards = false});
		pros::Task::delay(350);
		hood.set_value(true);
		chassis_large.moveToPoint(28, -47, 550, {.forwards = false});
		scraper.set_value(false);

		chassis_large.setPose(33, -48, chassis_large.getPose().theta);

		mtp_v_cancel(36, -48, 40);
		hood.set_value(false);

		stp_v_cancel(22, -23, 15, 'l');

		intake_high(127);

		mtp_v_cancel(22, -23, 10, true);
		scraper.set_value(true);
		//chassis_large.turnToPoint(12, -13, 1500, {.forwards = false}, false);
		ttp_v_cancel(12, -13, 10, false);
		//intake_high(0);
		//mtp_v_cancel(12, -13, 15, false, false);
		chassis_large.moveToPose(0, 0, 135, 500, {.forwards = false, .maxSpeed = 90});

		pros::Task::delay(450);
		intake_middle(90);

		chassis_large.moveToPoint(0, 0, 700, {.forwards = false, .maxSpeed = 30});
		
		pros::Task::delay(700);
		scraper.set_value(false);
		//chassis_large.setPose(14, -14, chassis_large.getPose().theta);
		stp_v_cancel(21, 23, 15, 'l');
		
		intake_middle(0);
		intake_high(127);
		mtp_v_cancel(21, 23, 15, true);
		scraper.set_value(true);

		stp_v_cancel(48, 48, 15, 'r');

		mtp_v_cancel(48, 48, 10);
		


		//ttp_v_cancel(72, 48, 10);
		chassis_large.turnToHeading(90, 500, {.minSpeed = 30}, false);
		//angular_pid_chooser(90);
		simple_dist_reset();
		/*
		chassis_large.moveToPoint(65, 48, 100, {.maxSpeed = 127}, false);
		chassis_large.moveToPoint(65, 48, 550, {.maxSpeed = 50}, false);
		chassis_large.moveToPoint(65, 48, 400, {.maxSpeed = 20}, false);
		*/
		moveto_matchload(1, 3);

		chassis_large.moveToPose(27, 48.5, 90, 1100, {.forwards = false});
		pros::Task::delay(400);
		hood.set_value(true);
		chassis_large.moveToPoint(0, 48.5, 5000, {.forwards = false});
		scraper.set_value(false);
		hood.set_value(true);
	}
	
	// left middle + long
	if(chosen_auto == 2)
	{
		chassis_large.setPose(43.5, -12, 270); 
		intake_high(127);
		hood.set_value(false);
		stp_v_cancel(22, -24, 15, 'l');
		mtp_v_cancel(22, -24, 25, true);

		ttp_v_cancel(7, -43, 10, 'l');
		mtp_v_cancel(7, -43, 15);
		ttp_v_cancel(24, -24, 15, false);
		mtp_v_cancel(24, -24, 15, false, false);
		ttp_v_cancel(13, -13, 10, false);
		mtp_v_cancel(13, -13, 15, false, false);
		chassis_large.moveToPoint(-72, 72, 1000, {.forwards = false, .maxSpeed = 30});
		
		intake_middle(50);
		pros::Task::delay(600);
		intake_middle(127);
		pros::Task::delay(350);
		intake_middle(0);
		intake_index.move(-127);
		pros::Task::delay(100);
		intake_index.move(0);
		chassis_large.moveToPoint(48, -48, 3000);
		intake_high(127);
		scraper.set_value(true);
		//ttp_v_cancel(67, -48, 10);
		
		chassis_large.turnToHeading(90, 500, {}, false);
		//angular_pid_chooser(90, 127, 0, 500);
		simple_dist_reset();

		moveto_matchload(4, 3);
		//chassis_large.moveToPoint(68, -48, 150, {.maxSpeed = 127}, false);
		//chassis_large.moveToPoint(68, -48, 550, {.maxSpeed = 50}, false);
		//chassis_large.moveToPoint(68, -48, 400, {.maxSpeed = 20}, false);

		chassis_large.moveToPose(28, -47, 90, 1500, {.forwards = false});
		pros::Task::delay(450);
		hood.set_value(true);
		chassis_large.moveToPoint(0, -47, 700, {.forwards = false}, false);

		
		chassis_large.setPose(33, -48, chassis_large.getPose().theta);
		scraper.set_value(false);

		mtp_v_cancel(38, -48, 10);
		ttp_v_cancel(36, -37, 5);
		mtp_v_cancel(36, -37, 5);
		ttp_v_cancel(19, -37, 5, false);
		mtp_v_cancel(19, -37, 5, false, false, 40);
	}
	// right low + long
	if(chosen_auto == 3)
	{
		chassis_large.setPose(43.5, 12, 270);
		intake_high(127);
		hood.set_value(false);
				stp_v_cancel(24, 24, 15, 'r');
		mtp_v_cancel(24, 24, 25, true);

		ttp_v_cancel(7, 43, 10, 'l');
		mtp_v_cancel(7, 43, 15);
		intake_high(0);
		ttp_v_cancel(24, 24, 15, false);
		mtp_v_cancel(24, 24, 15, false, false);
		ttp_v_cancel(15, 15, 10);
		mtp_v_cancel(15, 15, 15);
		intake_bottom.move(-127);
		pros::Task::delay(700);
		intake_bottom.move(0);

		ttp_v_cancel(48, 48, 10, false, false);
		chassis_large.moveToPoint(48, 48, 3000, {.forwards = false});
		intake_high(127);
		scraper.set_value(true);
		chassis_large.turnToHeading(90, 800, {}, false);
		//angular_pid_chooser(90, 127, 0, 800);
		simple_dist_reset();

		moveto_matchload(1, 3);

		chassis_large.moveToPose(27, 48, 90, 1200, {.forwards = false, .maxSpeed = 127});
		pros::Task::delay(400);
		hood.set_value(true);
		pros::Task::delay(1800);
		
		chassis_large.setPose(33, 48, chassis_large.getPose().theta); 
		scraper.set_value(false);
	}
	// right long
	if(chosen_auto == 4)
	{
		chassis_large.setPose(43.5, 12, 270); 
		intake_high(127);
		hood.set_value(false);
		stp_v_cancel(24, 24, 15, 'r');
		mtp_v_cancel(24, 24, 25, true);

		ttp_v_cancel(7, 43, 10, 'l');
		mtp_v_cancel(7, 43, 15);
		ttp_v_cancel(24, 24, 15, false);
		mtp_v_cancel(24, 24, 15, false, false);
		ttp_v_cancel(48, 48, 10);
		chassis_large.moveToPoint(48, 48, 3000);
		intake_high(127);
		scraper.set_value(true);
		chassis_large.turnToHeading(90, 500, {}, false);
		//angular_pid_chooser(90, 127, 0, 500);
		simple_dist_reset();
/*
		chassis_large.moveToPoint(65, 48, 100, {.maxSpeed = 127}, false);
		chassis_large.moveToPoint(65, 48, 550, {.maxSpeed = 60}, false);
		chassis_large.moveToPoint(65, 48, 400, {.maxSpeed = 20}, false);
*/
		moveto_matchload(1, 3);

		chassis_large.moveToPose(27, 48, 90, 1200, {.forwards = false, .maxSpeed = 127});
		pros::Task::delay(400);
		hood.set_value(true);
		pros::Task::delay(1800);
		
		chassis_large.setPose(33, 48, chassis_large.getPose().theta); 
		scraper.set_value(false);
		/*
		mtp_v_cancel(38, 48, 10);
		ttp_v_cancel(36, 58, 5);
		mtp_v_cancel(36, 58, 5);
		ttp_v_cancel(14, 5, 5, false);
		mtp_v_cancel(14, 57, 5, false, false, 70);
		*/
	}
	// skills
	if(chosen_auto == 5)
	{
		/*
		//part 1
		chassis_large.setPose(43.5, 12, 270); 
		intake_high(127);
		hood.set_value(false);
		ears.set_value(true);
		stp_v_cancel(27, 24, 10, 'r');
		mtp_v_cancel(27, 24, 15);
		ttp_v_cancel(50, 46, 15, false);
		mtp_v_cancel(50, 46, 10, false, false);
		//chassis_large.moveToPoint(47, 48, 2000, {.forwards = false}, false);
		//mtp_v_cancel(48, 48, 5, false, false);
		scraper.set_value(true);
		chassis_large.turnToHeading(90, 1000, {}, false);
		//angular_pid_chooser(90, 127, 0, 1000);
		simple_dist_reset();
		moveto_matchload(1, 6);
		
		intake_high(0);
		intake_bottom.move(127);
		mtp_v_cancel(53, 47, 40, false, false);
		intake_bottom.move(0);
		stp_v_cancel(36, 57, 25, 'l', false);
		mtp_v_cancel(36, 57, 25, false, false);
		stp_v_cancel(-24, 60, 25, 'r', false);
		mtp_v_cancel(-24, 60, 25, false, false, 127, 800);
		mtp_v_cancel(-24, 60, 25, false, false, 80);

		stp_v_cancel(-48, 47, 25, 'r', false);
		mtp_v_cancel(-48, 47, 15, false, false);

		//ttp_v_cancel(-72, 48, 15);
		chassis_large.turnToHeading(270, 1300, {}, false);
		//angular_pid_chooser(270, 127, 0, 1300);
		simple_dist_reset();

		intake_high(127);
		moveto_matchload(2, 6);
		chassis_large.moveToPose(-27, 47, 270, 1500, {.forwards = false, .lead = 0.6});
		pros::Task::delay(1000);
		hood.set_value(true);
		intake_high(-127);
		pros::Task::delay(50);
		intake_high(127);
		chassis_large.moveToPoint(0, 48, 4000, {.forwards = false});
		pros::Task::delay(4500);
		scraper.set_value(false);
		*/
		intake_high(127);
		chassis_large.setPose(-33, 48, 270); 
		
		//part 2
		
		mtp_v_cancel(-37, 48, 40);
		hood.set_value(false);
		stp_v_cancel(-44, -2, 15, 'l');
		
		
		chassis_large.moveToPoint(-44, 0, 2000);
		chassis_large.turnToHeading(270, 600, {}, false);
		//angular_pid_chooser(270, 127, 0, 600);

		intake_high(127);
		chassis_large.moveToPoint(-72, chassis_large.getPose().y, 2300);
		pros::Task::delay(600);
		scraper.set_value(true);
		pros::Task::delay(300);
		scraper.set_value(false);
		pros::Task::delay(600);
		for(int i = 0; i < 1; i++){
			chassis_large.turnToHeading(240, 200);
			chassis_large.turnToHeading(300, 200);
		}
		chassis_large.turnToHeading(270, 300, {}, false);
		//angular_pid_chooser(270, 127, 0, 300);

		chassis_large.moveToPoint(-72, chassis_large.getPose().y, 500, {}, false);
		chassis_large.setPose(0, 0, chassis_large.getPose().theta);

		for(int i = 0; i < 2; i++){
			chassis_large.turnToPoint(-24, 18, 100);
			chassis_large.moveToPoint(-24, 18, 150, {}, false);
			chassis_large.moveToPoint(2, 0, 150, {.forwards = false});
			chassis_large.turnToPoint(-24, -18, 100);
			chassis_large.moveToPoint(-24, -18, 150, {}, false);
			chassis_large.moveToPoint(2, 0, 150, {.forwards = false});
			//chassis_large.turnToHeading(210, 200);
			//chassis_large.turnToHeading(320, 200);
		}
		
		chassis_large.turnToHeading(270, 300, {}, false);
		//angular_pid_chooser(270, 127, 0, 300);

		chassis_large.moveToPoint(40, chassis_large.getPose().y, 800, {.forwards = false});



		chassis_large.moveToPoint(-72, chassis_large.getPose().y, 1400, {.maxSpeed = 20}, false);
		
		simple_dist_reset();
		chassis_large.setPose(-49, chassis_large.getPose().y, chassis_large.getPose().theta);
		mtp_v_cancel(-45, 0, 10, false, false);
		
		ttp_v_cancel(-33, 19, 10);
		chassis_large.moveToPose(-31, 21, 45, 400);

		chassis_large.moveToPose(-31, 21, 45, 2000, {.maxSpeed = 30});

		quarterstack();
		chassis_large.cancelAllMotions();
		pros::Task::delay(100);
		intake_high(0);
		intake_bottom.move(-30);

		mtp_v_cancel(-26, 22, 10);
		ttp_v_cancel(-13, 13, 10, false);
		intake_high(127);

		chassis_large.moveToPose(-11, 13, 315, 600, {.forwards = false, .lead = 0.6});
		pros::Task::delay(100);
		intake_top.move(-127);
		intake_index.move(-127);
		pros::Task::delay(80);
		intake_low(0);
		chassis_large.moveToPoint(0, 0, 600, {.forwards = false, .maxSpeed = 30});
		intake_middle_skills(127);
		pros::Task::delay(1500);
		//mtp_v_cancel(chassis_large.getPose().x-1,chassis_large.getPose().y+1 , 15, false, false, 50);
		intake_index.move(-127*0.175);
		pros::Task::delay(1800);
		intake_middle_skills(0);
	//	chassis_large.moveToPoint(0, 0, 250, {.forwards = false, .maxSpeed = 30});
		chassis_large.moveToPoint(-20, 20, 250);

		//part 3
		stp_v_cancel(-24, -24, 15, 'l');
		intake_high(127);
		mtp_v_cancel(-24, -24, 10, true);
		scraper.set_value(true);
		stp_v_cancel(-48, -48, 15, 'r');
		chassis_large.moveToPoint(-48, -48, 1500);



		//ttp_v_cancel(-72, -47, 10);
		chassis_large.turnToHeading(270, 600, {}, false);
		//angular_pid_chooser(270, 127, 0, 600);

		simple_dist_reset();
		moveto_matchload(3, 6);
		intake_high(0);
		intake_bottom.move(127);
		
		mtp_v_cancel(-53, -48, 40, false, false);
		intake_bottom.move(0);
		stp_v_cancel(-36, -57, 25, 'l', false);
		mtp_v_cancel(-36, -57, 25, false, false);
		stp_v_cancel(-24, -60, 25, 'r', false);
		mtp_v_cancel(24, -60, 25, false, false, 127, 800);
		mtp_v_cancel(24, -60, 25, false, false, 80);
		stp_v_cancel(48, -46, 25, 'r', false);
		mtp_v_cancel(48, -46, 15, false, false);
		
		//ttp_v_cancel(72, -48, 15);
		chassis_large.turnToHeading(90, 1300, {}, false);
		//angular_pid_chooser(90, 127, 0, 1300);

		simple_dist_reset();
		intake_high(127);
		moveto_matchload(4, 6);
		
		chassis_large.moveToPose(27, -47, 90, 1500, {.forwards = false, .lead = 0.6});
		pros::Task::delay(1000);
		hood.set_value(true);
		intake_high(-127);
		pros::Task::delay(50);
		intake_high(127);
		chassis_large.moveToPoint(27, -47, 4000, {.forwards = false});
		
		hood.set_value(true);
		pros::Task::delay(4500);
		scraper.set_value(false);

		chassis_large.setPose(33, -48, chassis_large.getPose().theta); 
		
		//part 4
		mtp_v_cancel(38, -48, 40);		

		stp_v_cancel(60, -17, 10, 'l');
		mtp_v_cancel(60, -17, 15);
		ttp_v_cancel(63, 0, 15);
		scraper.set_value(true);
		pros::Task::delay(200);
		chassis_large.moveToPoint(63, 0, 1400, {}, false);
		scraper.set_value(false);
		
		/*
		stp_v_cancel(60, -36, 15, 'l');
		mtp_v_cancel(60, -36, 15);
		ttp_v_cancel(66, 0, 10, false);
		chassis_large.moveToPoint(66, 0, 6500, {.forwards = false});
		intake_middle(0);
		*/
	}



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
	chassis_large.cancelAllMotions();
	g_auton_started = true;
	g_op_control_started = true;
	hood.set_value(false);
	g_hood_state = false;

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
			intake_high(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			intake_low(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			intake_middle_skills(127);
			
		}
		else
		{
			intake_high(0);
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
		
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			if(g_park_state)
			{
				park.set_value(false);
				g_park_state = false;
			}
			else
			{
				park.set_value(true);
				g_park_state = true;
			}
		}
		pros::Task::delay(25);
		}
	});


	int leftY;
	int rightX;
	bool cutdrive = false;
    while (true) {
		// get left y and right x positions
		leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if(!cutdrive)
		{
			chassis_large.curvature(leftY, rightX);
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			cutdrive = !cutdrive;
		}


		// delay to save resources
        pros::delay(25);
    }
}	
