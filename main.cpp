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

// these will not work when you copy the code, but they are not currently used in the autonomous code
ASSET(curve_txt);

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
pros::Motor front_left_motor(-19, pros::MotorGears::blue); // front left motor on port 9
pros::Motor middle_left_motor(9, pros::MotorGears::blue); // middle left motor on port 20
pros::Motor front_motor(-10, pros::MotorGears::blue); // back left motor on port 10
pros::Motor front_right_motor(2, pros::MotorGears::blue); // front right motor on port 2
pros::Motor middle_right_motor(-3, pros::MotorGears::blue); // middle right motor on port 11
pros::Motor back_motor(5, pros::MotorGears::blue); // back right motor on port 5

//intake_motors
pros::Motor intake_motor_left(20, pros::MotorGears::blue); // intake motor on port 19
pros::Motor intake_motor_right(-11, pros::MotorGears::blue); // lift motor on port 12

// condensed motors into motor groups
pros::MotorGroup left_motor_group({-2, 3, -4}, pros::MotorGears::blue); //the right side of the drivetrain
pros::MotorGroup right_motor_group({6, -7, 8}, pros::MotorGears::blue); //the left side of the drivetrain
pros::MotorGroup intake_motor_group({9}, pros::MotorGears::blue); 

// pneumatics 
pros::ADIDigitalOut mogo (5); // mogo on port 5 (letter E)
pros::ADIDigitalOut pto (6); // pto on port 7 (letter G)
pros::ADIDigitalOut hook (4); // hook on port 3 (letter C)

lemlib::Drivetrain drivetrain_6m(&left_motor_group, //motors that are on the left channel
                              &right_motor_group, //motors that are on the right channel
                              11.25, // track width
                              lemlib::Omniwheel::NEW_275, //the specific vex wheels used
                              600, // the rpm of the driven axels
                              2 //horizontal drift
); 

// create an imu on port 12
pros::Imu imu(10);

pros::Distance distance_front(16);
pros::Distance distance_back(14);
pros::Distance distance_left(17);
pros::Distance distance_right(15);

//distance sensor initialization
pros::Distance distance_sort(13);

//limit switches
//pros::ADIDigitalIn mogo_switch(2);
pros::ADIDigitalIn color_switch(8);

//pros::ADIDigitalIn left_switch(1);

// arm tracking encoder
pros::Rotation arm_encoder(19);
// left tracking wheel encoder
pros::Rotation vertical_encoder(-1);

// left tracking wheel (&what sensor it is tracking, &what type of omniwheel, offset, gear ratio)
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, +0 /*-1*/, 1);
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
lemlib::ControllerSettings lateral_controller_large(25, // proportional gain (kP)10
                                              0.13, // integral gain (kI)0.1
                                              225, // derivative gain (kD)53
                                              0.5, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              90 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller_large(2.8, // proportional gain (kP)
                                              0.2, // intgral gain (kI)
                                              23, //derivative gain (kD)
                                              6, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve	(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.05 //1.03 expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain_6m, // drivetrain settings
                        lateral_controller_large, // lateral PID settings
                        angular_controller_large, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, // forward/backward driver movement
                        &steer_curve // left/right driver movement
						);

					
/*
lemlib::Chassis chassis_small(drivetrain_6m, // drivetrain settings
                        lateral_controller_small, // lateral PID settings
                        angular_controller_small, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, // forward/backward driver movement
                        &steer_curve // left/right driver movement
						);
*/

pros::Controller controller(pros::E_CONTROLLER_MASTER); // the controller that will be used for driving

//global variables
//TODO is auto_started still needed?
bool g_auton_started = false; // determines whether the autonamous code has been started
bool g_op_control_started = false; // determin es whether the driver code has been started

// get and set in autonomous()
int g_current_auton_select = 2; // determines which auton path is going to be run

// set in color_select(), get in color_sorting() and autonomous()
int g_team_color = BLUE; //red = 1, blue = -1

// set in both auto_clamp and op_control
// TODO: Do we need to coordinate bteween these 2 functions?
bool g_mogo_setting = false;


//defines the pto value
//set in auton and driver
bool g_pto_setting = false;

bool g_high_stake = false;

//defines the hook value
//set in auton and driver
bool g_hook_setting = false;

// set in color_sorting() and get in autonomous()
float g_intake_direction = 1;

// get/set in opcontrol() 
bool g_claw_mode = false;
//bool seen_blue = false;

const float gc_arm_kP = 0.043;  // Proportional gain for arm
const float gc_arm_kD = 0.0;  // Derivative gain for arm
const float gc_max_volt = 110;

// Read in arm_pid_control, set in autonomous and opcontrol
int g_arm_target_position = 0; // Target encoder position for the arm

// distance sensor offsets
float back_distance_from_center = 2;
float back_distance_between = 9;
float side_distance_from_center = 1.5;
float side_distance_between = 11;
float front_distance_from_center = 6.25;
float front_distance_between = 9.5;

float vision_x = 0;
float vision_y = 0;

float distance_kp = 15, distance_ki = 0, distance_kd = 30;
float heading_correction_kp = 8, heading_correction_ki = 0, heading_correction_kd = 80;
int max_slew_accel_fwd = 4, max_slew_decel_rev = 4;

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

std::vector<lemlib::Pose> poses_holder;

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

void followPath(float time_limit_msec, bool exit, double max_output, std::string address, int num, int dir) {

	//pros::screen::print(pros::E_TEXT_SMALL, 0, "0", chassis.getPose().x); // x


	extractPose(address, num);
		//pros::screen::print(pros::E_TEXT_SMALL, 0, "1", chassis.getPose().x); // x
	if (poses_holder.size() == 0) return;

	for(int i = 0; i < poses_holder.size(); i++)
	{
		lemlib::Pose poos = poses_holder[i];
		std::cout << "x: " << poos.x << " y: " << poos.y << " theta: " << poos.theta << std::endl;
	}

	double kStanley = 10; // Stanley gain for lateral correction

	const double max_steering_angle = 179.0; // Max steering correction (degrees)

	const double stop_tolerance = 0; // Final stop distance

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

	while (target_index < poses_holder.size() && (float)pros::millis() - start_time < time_limit_msec) {

		double robot_x = chassis.getPose().x;

		double robot_y = chassis.getPose().y;

		double robot_heading_deg = chassis.getPose().theta;

		double robot_heading_rad = lemlib::degToRad(robot_heading_deg);


		// Find closest pose

		double min_dist = 1e6;

		for (size_t i = target_index-4; i < poses_holder.size(); i++) 
		{
			double dx = poses_holder[i].x - robot_x;

			double dy = poses_holder[i].y - robot_y;

			double dist = hypot(dx, dy);

			if (dist < min_dist) {

				min_dist = dist;

				target_index = i;
			}
		}
		
		target_index+=4;

		if (target_index >= poses_holder.size()-1) { target_index = poses_holder.size()-1;  break; }

		const lemlib::Pose target = poses_holder[target_index];

		double target_x = target.x;

		double target_y = target.y;

		double target_heading_deg = lemlib::sanitizeAngle(target.theta-90+90*dir, false);
		pros::screen::print(pros::E_TEXT_SMALL, 3, "%2f", target_heading_deg); // x

		double target_heading_rad = lemlib::degToRad(target_heading_deg);

		// distance2target

		double dx = target_x - robot_x;

		double dy = target_y - robot_y;

		double distance_error = hypot(dx, dy);//dx*sin(robot_heading_rad)+dy*cos(robot_heading_rad);



		double speed = pid_distance.update(distance_error);


		//speed = clamp(speed, -max_output, max_output);
		speed = set_limit(speed, max_output);

		// Heading error

		//double heading_error = lemlib::sanitizeAngle(target_heading_rad - robot_heading_rad, true);
		double heading_error = lemlib::angleError(target_heading_rad, robot_heading_rad, true);//target_heading_rad - robot_heading_rad;

		// Cross track error

		double cross_track_error = dx * cos(target_heading_rad) - dy * sin(target_heading_rad);

		//correction

		double steering_rad = heading_error + atan2(kStanley * cross_track_error * dir, speed);

		//steering_rad = clamp(steering_rad, -lemlib::degToRad(max_steering_angle), lemlib::degToRad(max_steering_angle));
		
		steering_rad = set_limit(steering_rad, lemlib::degToRad(max_steering_angle));

		double steering_deg = lemlib::radToDeg(steering_rad);

		// heading

		//pid_heading.update(lemlib::sanitizeAngle(robot_heading_deg + steering_deg, false));//setTarget(normalizeTarget(robot_heading_deg + steering_deg));
		
		double heading_correction = pid_heading.update(-steering_deg);

		heading_correction = set_limit(heading_correction, max_output);

		lemlib::Pose target_pose = {poses_holder[target_index].x, poses_holder[target_index].y, lemlib::degToRad(-poses_holder[target_index].theta+90)};
		lemlib::Pose next_target_pose = {poses_holder[target_index+1].x, poses_holder[target_index+1].y, lemlib::degToRad(-poses_holder[target_index+1].theta+90)};

		double curvature_scale = 0.03*1/fabs(lemlib::getCurvature(target_pose, next_target_pose));
		if(curvature_scale > 1) { curvature_scale = 1;}
		if(curvature_scale < 0.4) { curvature_scale = 0.4;}

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
	{chassis.moveToPoint(poses_holder[target_index].x, poses_holder[target_index].y, 1000);}
	else if(dir == -1) 
	{chassis.moveToPoint(poses_holder[target_index].x, poses_holder[target_index].y, 1000, {.forwards = false});}

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

	//std::ifstream constants_holder("/usd/constants_holder.txt", std::ios::in);
	//int large_kp = 25;
	//constants_holder >> large_kp;
	//constants_holder >> large_ki;
	//constants_holder >> large_kd;
	
	//constants_holder.close();
		//std::cout<< "zero" <<std::endl;

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
	chassis.setPose(-48, 24, 270); 

	brain_data_output(); // print values to the brain screen
	chassis.moveToPoint(-24, 24, 5000, {.forwards = false, .maxSpeed = 100});
	chassis.turnToHeading(55, 5000, {.maxSpeed = 100}, false);

	followPath(50000, true, 100, "HS_neg.txt", 2, 1);



	// set position to x:0, y:0, heading:0
    // turn to face heading 90 with a very long timeout
	//chassis.setPose(0,0,0);
    //chassis.turnToHeading(180, 10000, {.maxSpeed = 30});


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

		 
	//pros::screen::print(pros::E_TEXT_SMALL, 0, "%s" , std::filesystem::current_path()); // x

	
	controller.print(0, 0, "program started");
	g_auton_started = true;
	g_op_control_started = true;
	brain_data_output();
    chassis.setPose(0, 0, 0);


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
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			intake_motor_group.move(-127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intake_motor_group.move(127);


		}
		else
		{
			intake_motor_group.move(0);
		}

		// toggle claw mode
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			if(g_claw_mode)
			{
				g_claw_mode = false;
				controller.print(0, 0, "claw setting: %i", g_claw_mode);

			}
			else
			{
				g_claw_mode = true;
				controller.print(0, 0,"claw setting: %i", g_claw_mode);

			}
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
			g_high_stake = true;
			intake_motor_group.move(-95);

			pros::Task::delay(300);
			intake_motor_group.move(60);

			int counter = 0;

			while(distance_sort.get() > 50 && counter < 2000)
			{
				pros::Task::delay(5);
				counter +=5;
			}

			intake_motor_group.move(0);
			g_arm_target_position = 0;
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			if (g_pto_setting == false)
			{
				mogo.set_value(true);
				g_mogo_setting = true;
			}
			else
			{
				hook.set_value(true);
				g_hook_setting = true;
			}
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			if (g_pto_setting == false)
			{
				mogo.set_value(false);
				g_mogo_setting = false;
			}
			else
			{
				hook.set_value(false);
				g_hook_setting = false;
			}

		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			
			if (g_hook_setting == false){
				hook.set_value(true);
				g_hook_setting = true;
			} else {
				hook.set_value(false);
				g_hook_setting = false;
			}
		}
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			
			if (g_intake_direction != 0.5){

				g_intake_direction = 0.5;
			} else {
				g_intake_direction = 1;
			}
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			
			if (g_pto_setting == false){
				pto.set_value(true);
				g_pto_setting = true;

			} else {
				pto.set_value(false);
				g_pto_setting = false;

			}
		}
		
		pros::Task::delay(25);
		}
	});



    // loop till climb
    while (true) {
		// get left y and right x positions
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.curvature(leftY, rightX);
		// delay to save resources
        pros::delay(25);
    }
}	