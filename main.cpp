#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pid.hpp"
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

// setup

// a negative number shows that the motor is reversed
// if a 3 wire has a number for the port, the number is just the letter of the port. 
//ex: 1 = A, 2 = B, 3 = C, 4 = D

//intake_motors
pros::Motor intake_bottom(12, pros::MotorGears::blue); // intake motor on port 19
pros::Motor intake_top(11, pros::MotorGears::green); // lift motor on port 12
pros::Motor intake_index(-19, pros::MotorGears::green); // lift motor on port 12

// condensed motors into motor groups
pros::MotorGroup left_motor_group({-8, 9, -10}, pros::MotorGears::blue); //the right side of the drivetrain
pros::MotorGroup right_motor_group({1, -2, 3}, pros::MotorGears::blue); //the left side of the drivetrain

lemlib::Drivetrain drivetrain_6m(&left_motor_group, //motors that are on the left channel
                              &right_motor_group, //motors that a	re on the right channel
                              11.25, // track width
                              lemlib::Omniwheel::NEW_275, //the specific vex wheels used
                              600, // the rpm of the driven axels
                              8 //horizontal drift
);

// create an imu on port 12
pros::Imu imu(12);

pros::ADIDigitalOut ears(6);
pros::ADIDigitalOut scraper(8);
pros::ADIDigitalOut hood(7);
pros::ADIDigitalOut funnel(5);


// left tracking wheel encoder
pros::Rotation vertical_encoder(13);

pros::Distance distance_left(14);
pros::Distance distance_right(21);
pros::Distance distance_front(1);

// left tracking wheel (&what sensor it is tracking, &what type of omniwheel, offset, gear ratio)
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 2.75, +0 /*-1*/, 1);
// right tracking wheel(&what sensor it is tracking, &what type of omniwheel, offset, gear ratio



lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr 
                        	nullptr,// horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller_large(19, // proportional gain (kP)19
                                              0.15, // integral gain (kI)0.15
                                              170, // derivative gain (kD)150
                                              2, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20//250 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller_small(2.9, // proportional gain (kP)//3.2
                                              0.07, // integral gain (kI)//0.25
                                              21,// derivative gain (kD)//20
                                              4, //4 anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
											  
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     3, // minimum output where drivetrain will move out of 127
                                     1.02 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve	(6, // joystick deadband out of 127
                                  6, // minimum output where drivetrain will move out of 127
                                  1.02 //1.03 expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain_6m, // drivetrain settings
                        lateral_controller_large, // lateral PID settings
                        angular_controller_small, // angular PID settings
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
int chosen_auto = 1; // determines which auton path is going to be run

//pneumatics toggles
float g_ears_state = false;
float g_scraper_state = false;
float g_hood_state = false;
float g_funnel_state = false;

float left_distance_from_center = 6.4;
float right_distance_from_center = 5.5;
float front_distance_from_center = 6;
float dist_to_center = 0;



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

	intake_top.move(volts*0.37);
	intake_index.move(-volts*0.28);

}
void intake_low(float volts)
{
	intake_bottom.move(-volts);
	intake_top.move(-volts);
	intake_index.move(0);
	if (g_auton_started == true)
	{
		intake_index.move(volts);
	}
}

float weighted_distance(int sensor_index)
{
	float distances[5];
	float confidences[5];
	float total_confidence = 0;
	float weighted_distance = 0;

	for(int i = 0; i < 5; i++)
	{
		if(sensor_index == 1)
		{
			distances[i] = distance_left.get()*0.0394; //convert mm to inches
			confidences[i] = distance_left.get_confidence() / 63.0;

		}
		else if(sensor_index == 2)
		{
			distances[i] = distance_right.get()*0.0394; //convert mm to inches
			confidences[i] = distance_right.get_confidence() / 63.0;
		}
		else if (sensor_index == 3)
		{
			distances[i] = distance_front.get()*0.0394; //convert mm to inches
			confidences[i] = distance_front.get_confidence() / 63.0;
		}
		total_confidence += confidences[i];
		pros::Task::delay(10);
	}

	float weighed_confidence = 0;
	for(int i = 0; i < 5; i++)
	{
		weighed_confidence = confidences[i] / total_confidence;
		weighted_distance += distances[i]*weighed_confidence;
	}
	return weighted_distance;
}

void simple_dist_reset()
{
	float rotated_dist_to_center = 0;

	if(chassis.getPose().x > 0 && chassis.getPose().y > 0)
	{
		dist_to_center = distance_left.get()*0.0394+left_distance_from_center;
		
		chassis.setPose(chassis.getPose().x, 72-dist_to_center, chassis.getPose().theta);
	}
	else if(chassis.getPose().x < 0 && chassis.getPose().y > 0)
	{
		dist_to_center = distance_right.get()*0.0394+right_distance_from_center;
		chassis.setPose(chassis.getPose().x, 72-dist_to_center, chassis.getPose().theta);
	}
	else if(chassis.getPose().x < 0 && chassis.getPose().y < 0)
	{
		dist_to_center = distance_left.get()*0.0394+left_distance_from_center;
		chassis.setPose(chassis.getPose().x, -72+dist_to_center, chassis.getPose().theta);
	}
	else if(chassis.getPose().x > 0 && chassis.getPose().y < 0)
	{
		dist_to_center = distance_right.get()*0.0394+right_distance_from_center;
		chassis.setPose(chassis.getPose().x, -72+dist_to_center, chassis.getPose().theta);
	}


}

void weighted_dist_reset()
{
	float rotated_dist_to_center = 0;

	if(chassis.getPose().x > 0 && chassis.getPose().y > 0)
	{
		dist_to_center = weighted_distance(1)+left_distance_from_center;
		
		chassis.setPose(chassis.getPose().x, 72-dist_to_center, chassis.getPose().theta);
	}
	else if(chassis.getPose().x < 0 && chassis.getPose().y > 0)
	{
		dist_to_center = weighted_distance(2)+right_distance_from_center;
		chassis.setPose(chassis.getPose().x, 72-dist_to_center, chassis.getPose().theta);
	}
	else if(chassis.getPose().x < 0 && chassis.getPose().y < 0)
	{
		dist_to_center = weighted_distance(1)+left_distance_from_center;
		chassis.setPose(chassis.getPose().x, -72+dist_to_center, chassis.getPose().theta);
	}
	else if(chassis.getPose().x > 0 && chassis.getPose().y < 0)
	{
		dist_to_center = weighted_distance(2)+right_distance_from_center;
		chassis.setPose(chassis.getPose().x, -72+dist_to_center, chassis.getPose().theta);
	}


}

void kd_function(char pid, float error)
{
		
	float kkD_lat = 53.5;
	float kkD_ang = 4.5;

	if(pid == 'l') //lateral
	{
		chassis.lateralPID.kD = std::max(kkD_lat*std::log(fabs(error)), 1.0);
	}
	else if(pid == 'a') //angular
	{
		chassis.angularPID.kD = std::max(kkD_ang*std::log(fabs(error)), 4.0);
	}
}

void moveto_point(float x, float y, float timeout, lemlib::MoveToPointParams params = {}, bool async = true)
{

	float dist = hypot(x-chassis.getPose().x, y-chassis.getPose().y);
	kd_function('l', dist);

	float dx = x - chassis.getPose().x;
	float dy = y - chassis.getPose().y;
	float heading_to_point = atan2(dx, dy) * 180.0/M_PI;
	float robot_theta;
	if(params.forwards)
	{
		robot_theta = chassis.getPose().theta;
	}
	else
	{
		robot_theta = lemlib::sanitizeAngle(chassis.getPose().theta + 180);
	}
	float angleError = lemlib::angleError(heading_to_point, robot_theta, false);
	kd_function('a', angleError);
	float angular_kp = chassis.angularPID.kP;
 

		chassis.moveToPoint(x, y, timeout, params, async);
		float initial_dist = dist;
		while (chassis.isInMotion() && dist > initial_dist*0.10)
		{
			dist = hypot(x-chassis.getPose().x, y-chassis.getPose().y);
			pros::Task::delay(10);
		}
		

		chassis.angularPID.kP = 0;
		while(chassis.isInMotion())
		{
			pros::Task::delay(10);
		}
		chassis.angularPID.kP = angular_kp;

}

void turnto_point(float x, float y, float timeout, lemlib::TurnToPointParams params = {}, bool async = true)
{
	float dx = x - chassis.getPose().x;
	float dy = y - chassis.getPose().y;

	float heading_to_point = atan2(dx, dy) * 180.0/M_PI;

	float angleError = lemlib::angleError(heading_to_point, chassis.getPose().theta, false);

	kd_function('a', angleError);
	chassis.turnToPoint(x, y, timeout, params, async);
}

void turnto_heading(float heading, float timeout, lemlib::TurnToHeadingParams params = {}, bool async = true)
{

	float angleError = lemlib::angleError(heading, chassis.getPose().theta, false);

	kd_function('a', angleError);
	chassis.turnToHeading(heading, timeout, params, async);
}

void swingto_point(float x, float y, lemlib::DriveSide lockside, float timeout, lemlib::SwingToPointParams params = {}, bool async = true)
{
	float dx = x - chassis.getPose().x;
	float dy = y - chassis.getPose().y;

	float heading_to_point = atan2(dx, dy) * 180.0/M_PI;

	float angleError = lemlib::angleError(heading_to_point, chassis.getPose().theta, false);

	kd_function('a', angleError);
	chassis.swingToPoint(x, y, lockside, timeout, params, async);
}

void swingto_heading(float heading, lemlib:: DriveSide lockside, float timeout, lemlib::SwingToHeadingParams params = {}, bool async = true)
{

	float angleError = lemlib::angleError(heading, chassis.getPose().theta, false);

	kd_function('a', angleError);
	chassis.swingToHeading(heading, lockside, timeout, params, async);
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
		y = 47.5;
		theta = 90;
		break;
	
	case 2:
		x = -67;
		y = 47.5;
		theta = 270;
		break;
	
	case 3: 
		x = -67;
		y = -47.5;
		theta = 270;
		break;
	
	case 4:
		x = 67;
		y = -47.5;
		theta = 90;
		break;
	}
	/*
	chassis.moveToPose(x, y, theta, 5000, {.lead = 0.2});
	
	while(fabs(chassis.getPose().x) < 47)
	{
		pros::Task::delay(10);
	}
	chassis.cancelMotion();
*/


	chassis.moveToPose(x, y, theta, 5000, {.lead = 0.2, .maxSpeed = 60});
	pros::Task::delay(300);
	float max_velocity = 550;
	float tracker_velocity = 550;
	float final_v_rpm = 40;

	while(tracker_velocity > final_v_rpm)
	{
		tracker_velocity = fabs(vertical_encoder.get_velocity()*100*360*60);
		pros::Task::delay(10);
	}
	pros::Task::delay(00);
	chassis.cancelMotion();

	if(x > 0)
	{
		x += 12;
	}
	else
	{
		x -= 12;
	}

	if(balls == 3)
	{
	chassis.moveToPoint(0, y, 30, {.forwards = false, .maxSpeed = 30}, false);
	chassis.moveToPoint(x, y, 170, {.maxSpeed = 20}, false);

	}
	else if(balls == 6)
	{
	//chassis.moveToPoint(0, y, 30, {.forwards = false, .maxSpeed = 30}, false);
	chassis.turnToHeading(theta+15, 60, {.minSpeed = 60});
	for(int i = 0; i < 2; i++)
	{
		chassis.turnToHeading(theta-15, 125, {.minSpeed = 60});

		chassis.moveToPoint(x, y, 80, {.minSpeed = 127}, false);
		//chassis.moveToPoint(0, y, 30, {.forwards = false, .maxSpeed = 20}, false);

		chassis.turnToHeading(theta+15, 125, {.minSpeed = 60});
		chassis.moveToPoint(x, y, 80, {.minSpeed = 127}, false);
		
	}
	chassis.turnToHeading(theta, 30, {.minSpeed = 30}, false);
	chassis.moveToPoint(x, y, 600, {.minSpeed = 30}, false);
	dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
	chassis.setPose(fabs(x)/x*(72-dist_to_center), chassis.getPose().y, chassis.getPose().theta);
	
	}

}

void moveto_stack(float x, float y)
{
	float dist = hypot(x-chassis.getPose().x, y-chassis.getPose().y);


	while (chassis.isInMotion())
	{
		dist = hypot(x-chassis.getPose().x, y-chassis.getPose().y);
		if(dist < 13)
		{
			scraper.set_value(true);
		}
		pros::Task::delay(10);
	}
	scraper.set_value(false);
}

void move_skills_low()
{
		
	chassis.setPose(0, 0, 90);
		intake_high(127);
		intake_index.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		ears.set_value(true);
		moveto_point(20, 0, 300, {.maxSpeed = 90});

		for(int i = 0; i < 2; i++)
		{
			turnto_heading(100, 100);
			turnto_heading(80, 100);
		}

		moveto_point(10, 0, 400, {.maxSpeed = 90});
		moveto_point(-72, 0, 100, {.forwards = false, .maxSpeed = 90});
		moveto_point(10, 0, 200, {.maxSpeed = 90});


		for(int i = 0; i < 2; i++)
		{
			turnto_heading(100, 100);
			turnto_heading(80, 100);
		}
		//leave park zone
		turnto_heading(90, 300, {}, false);
		moveto_point(-72, chassis.getPose().y, 200, {.forwards = false, .maxSpeed = 40}, false);
		moveto_point(-72, chassis.getPose().y, 700, {.forwards = false}, false);
		pros::Task::delay(400);

		// reset position after park
		turnto_heading(90, 1000, {.minSpeed = 5, .earlyExitRange = 0.5}, false);
		chassis.setPose(0, 1, chassis.getPose().theta);
		weighted_dist_reset();
		dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
		chassis.setPose(72-dist_to_center, 
							chassis.getPose().y, 
							chassis.getPose().theta);
		
		//move to stack 1					
		moveto_point(30, chassis.getPose().y, 2000, {.forwards = false,.minSpeed = 10, .earlyExitRange = 0.5}, false);
		turnto_point(30, 18, 2500, {.minSpeed = 1, .earlyExitRange = 3});
		//chassis.moveToPose(30, 24, 0, 800);
		moveto_point(30, 18, 1400, {}, false);
		intake_bottom.move(-40);
		moveto_point(30, 24, 500, {.maxSpeed = 40}, false);

		
		//move and score on lower goal
		turnto_point(0, 4, 2000, {.minSpeed = 1, .earlyExitRange = 3});
		intake_high(127);

		//chassis.moveToPose(9, 11, 225, 1000, {.lead = 0.7});
		moveto_point(0, 4, 300, {}, false);
		moveto_point(0, 4, 400, {.maxSpeed = 40}, false);

		chassis.moveToPoint(0, 4, 600, {.maxSpeed = 40});
		
		funnel.set_value(true);
		intake_low(127*0.7);
		
		//pros::Task::delay(100);
		chassis.turnToHeading(chassis.getPose().theta+10, 1000);

		pros::Task::delay(1000);
		intake_low(127*0.45);
		pros::Task::delay(800);
		intake_low(127*0.45); 
		pros::Task::delay(500);
		intake_bottom.move(127);

		pros::Task::delay(50);
		intake_low(127*0.40);

		pros::Task::delay(1000);


		moveto_point(72, 72, 200, {.forwards = false, .maxSpeed = 40});

		moveto_point(0, 4, 1000, {.maxSpeed = 30});
		//chassis.moveToPoint(24, 24, 2000, {.forwards = false});
		

		//move to 1st matchloader
		//chassis.setPose(24, 24, 270);
		//turnto_heading(225, 2000, {}, false);
		
		moveto_point(48, 44, 200, {.forwards = false, .minSpeed = 20, .earlyExitRange = 7});
		
}

void descore()
{
	chassis.setPose(31, 48, 90);

	chassis.moveToPoint(37, 48, 1000, {.minSpeed = 50, .earlyExitRange = 2});
	chassis.turnToHeading(135, 2000, {.minSpeed = 20, .earlyExitRange = 5}, false);
	chassis.moveToPose(19, 53, 80, 2000, {.forwards = false, .lead = 0.5, .minSpeed = 100});
	chassis.moveToPoint(13, 54, 600, {.forwards = false,.maxSpeed = 100, .minSpeed = 100}, false);
	chassis.turnToHeading(130, 300, {.minSpeed = 100}, false);
	scraper.set_value(true);
	chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
}

void undergoal_balls_matchloader(int field_side, bool hold_matchloads)
{
	lemlib::DriveSide outside;
	lemlib::DriveSide inside;
	if(field_side == 1)
	{
		outside = lemlib::DriveSide::RIGHT;
		inside = lemlib::DriveSide::LEFT;
	}
	else if(field_side == -1)
	{
		outside = lemlib::DriveSide::LEFT;
		inside = lemlib::DriveSide::RIGHT;
	}

	//1st stack of 4
	chassis.setPose(45, 12*field_side, 270); 
	intake_high(127);
	chassis.moveToPose(11, 40*field_side, 90-field_side*90, 1900, {.lead = 0.25, .minSpeed = 120});

	pros::Task::delay(400);
	scraper.set_value(true);
	pros::Task::delay(150);
	scraper.set_value(false);
	chassis.waitUntilDone();
	//stack under goal
\	
	scraper.set_value(true);
	pros::Task::delay(150);
	chassis.moveToPoint(10, 36*field_side, 300, {.forwards = false, .maxSpeed = 60});
	pros::Task::delay(80);
	scraper.set_value(false);
	chassis.moveToPoint(9, 40*field_side, 300, {.maxSpeed = 60}, false);

	// 1st matchloader
	chassis.swingToPoint(24, 34*field_side, outside, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 10});
	chassis.moveToPoint(24, 34*field_side, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
	chassis.swingToPoint(46, 44*field_side, outside, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 10});
	chassis.moveToPoint(46, 44*field_side, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
	scraper.set_value(true);
	chassis.turnToHeading(90, 1000, {.maxSpeed = 100, .minSpeed = 1, .earlyExitRange = 3}, false);
	simple_dist_reset();
	int choose_matchloader = 2.5-field_side*1.5;
	moveto_matchload(choose_matchloader, 3);

	// move and score on goal
	chassis.moveToPose(27, 48*field_side, 270, 3000, {.forwards = false});
	if(hold_matchloads)
	{
		intake_bottom.move(0);
	}
	chassis.waitUntil(17);
	hood.set_value(true);
	chassis.cancelMotion();
	chassis.moveToPoint(0, 48*field_side, 500, {.forwards = false, .maxSpeed = 50}, false);
	pros::Task::delay(1000);
	
	
	chassis.setPose(31, 48*field_side, chassis.getPose().theta);
	scraper.set_value(false);	

}

void auto_selector()
{
	pros::Task auton_seletor_task([&]() {	
	pros::screen_touch_status_s_t status = pros::screen::touch_status();
	float press_count =0;
	while(true)
	{
	 	status = pros::screen::touch_status();
		press_count = status.press_count;

		if(press_count > 0.5)
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
            pros::screen::print(pros::E_TEXT_SMALL, 0, "X: %f", chassis.getPose().x); // x
            pros::screen::print(pros::E_TEXT_SMALL, 1, "Y: %f", chassis.getPose().y); // y
            pros::screen::print(pros::E_TEXT_SMALL, 2, "Theta: %f", chassis.getPose().theta); // heading
            pros::screen::print(pros::E_TEXT_SMALL, 3, "Auto: %d", chosen_auto); // auto
			float distance_reading = distance_left.get()*0.0394;
            pros::screen::print(pros::E_TEXT_SMALL, 4, "dist left: %f", distance_reading); // dist sensor
			distance_reading = distance_right.get()*0.0394;            
			pros::screen::print(pros::E_TEXT_SMALL, 5, "dist right: %f", distance_reading); // dist sensor
			distance_reading = distance_front.get()*0.0394;            
			pros::screen::print(pros::E_TEXT_SMALL, 6, "dist front: %f", distance_reading); // dist sensor
			
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
	intake_top.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	chosen_auto = 1;
	
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


	switch(chosen_auto) {
	
	//skills
	case 0:
	{
		
		move_skills_low();
		
		moveto_point(48, 44, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 7});
		scraper.set_value(true);
		funnel.set_value(false);
		intake_high(127);
		ears.set_value(true);
		turnto_heading(90, 1000, {.minSpeed = 5, .earlyExitRange = 5}, false);
		simple_dist_reset();
		moveto_matchload(1, 6);
		
		//move down the 1st alley
		moveto_point(52, 46, 3000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3}, false);
		intake_high(0);
		intake_index.move(127);
		turnto_point(27, 60, 3000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 10}, false);

		moveto_point(27, 60, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
		turnto_heading(90, 1000, {.minSpeed = 10, .earlyExitRange = 3}, false);
		simple_dist_reset();

		scraper.set_value(false);
		moveto_point(-23, 62, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 4});
		
		//move to 2nd matchloader
		//turnto_point(-44, 48, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 5}, false);
		moveto_point(-42, 52, 600, {.forwards = false, .minSpeed = 40, .earlyExitRange = 6});
		scraper.set_value(true);
		intake_high(127);
		turnto_heading(270, 1200, {.minSpeed = 4, .earlyExitRange = 3}, false);
		simple_dist_reset();
		moveto_matchload(2, 6);

		//move to 1st goal for the 1st time

		chassis.moveToPose(-27, 48, 270, 3000, {.forwards = false});
		chassis.waitUntil(19);
		chassis.cancelMotion();
		chassis.moveToPoint(0, 48, 800, {.forwards = false, .maxSpeed = 30});

		intake_high(0);
		//intake_index.move(30);
		hood.set_value(true);
		pros::Task::delay(200);
		chassis.cancelMotion();
		intake_high(127);
		//chassis.moveToPoint(0, 48, 1900, {.forwards = false, .maxSpeed = 15});

		pros::Task::delay(3000);
		chassis.setPose(-31, 48, chassis.getPose().theta);
		
		

		//chassis.setPose(-31, 48, 270);
		intake_high(127);
		ears.set_value(true);
		scraper.set_value(false);

		//move to in front of blue park zone

		moveto_point(-39, 48, 800, {.minSpeed = 1, .earlyExitRange = 2}, false);
		
		swingto_heading(180, lemlib::DriveSide::LEFT, 2000, {});
		hood.set_value(false);
			
		moveto_point(-43, 4, 2000);
		dist_to_center = weighted_distance(3)+front_distance_from_center;
		
		chassis.setPose(chassis.getPose().x, -72+dist_to_center, chassis.getPose().theta);

		moveto_point(-43, 0, 2000);
		
		//chassis.waitUntil(25);
		//chassis.cancelAllMotions();
		// moveto_point(-42, 0, 800, {.maxSpeed = 30});
		
		//enter park zone and clear balls
		turnto_heading(270, 600, {}, false);
		intake_high(127);
		moveto_point(-72, chassis.getPose().y, 700, {.maxSpeed = 50});
		chassis.moveToPoint(-72, chassis.getPose().y, 800, {.maxSpeed = 110});

		pros::Task::delay(400);
		scraper.set_value(true);
		pros::Task::delay(400);
		for(int i = 0; i < 1; i++){
			turnto_heading(255, 200);
			turnto_heading(285, 200);
		}
		scraper.set_value(false);

		turnto_heading(270, 300, {}, false);

		moveto_point(-72, chassis.getPose().y, 300, {.maxSpeed = 90}, false);
		chassis.moveToPoint(72, chassis.getPose().y, 100, {.forwards = false, .maxSpeed = 90}, false);
		chassis.setPose(0, 0, chassis.getPose().theta);
		moveto_point(-72, chassis.getPose().y, 300, { .maxSpeed = 90}, false);
		
		for(int i = 0; i < 1; i++){
			turnto_heading(255, 200);
			turnto_heading(285, 200);
		}

		/*
		for(int i = 0; i < 1; i++){
			//turnto_heading(250, 200);
			//turnto_heading(290, 200);
			
			moveto_point(4, 0, 200, {.forwards = false});
			turnto_point(-24, 28, 200);
			//chassis.moveToPoint(-24, 12, 200, {}, false);
			swingto_heading(270, lemlib::DriveSide::LEFT, 200);
			turnto_point(4, 0, 100, {.forwards = false, .minSpeed = 30});
			moveto_point(4, 0, 200, {.forwards = false});
			turnto_point(-24, -28, 200);
			//chassis.moveToPoint(-24, -12, 200, {}, false);
			swingto_heading(270, lemlib::DriveSide::RIGHT, 200);
			turnto_point(4, 0, 100, {.forwards = false, .minSpeed = 30});

		}
		*/
		
		//leave park zone
		turnto_heading(270, 200, {}, false);
		moveto_point(-72, chassis.getPose().y, 300, {.minSpeed = 127});
		if(check_equal(chassis.getPose().theta, 270, 10))
		{
			chassis.setPose(chassis.getPose().x, chassis.getPose().y, 270);
		}
		moveto_point(20, chassis.getPose().y, 200, {.forwards = false, .maxSpeed = 30}, false);
		moveto_point(20, chassis.getPose().y, 2000, {.forwards = false}, false);
		pros::Task::delay(200);


		// reset position after park
		turnto_heading(270, 1000, {.minSpeed = 30}, false);
		chassis.setPose(0, 1, chassis.getPose().theta);

		weighted_dist_reset();
		dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
		chassis.setPose(-72+dist_to_center, 
							chassis.getPose().y, 
							chassis.getPose().theta);

		//move to middle goal
		/*
		turnto_point(-21, 21, 600, {.forwards = false, .minSpeed = 5, .earlyExitRange = 2});
		moveto_point(-21, 21, 1000, {.forwards = false});
		turnto_point(-10, 10, 700, {.forwards = false});
		*/
		moveto_point(-30, chassis.getPose().y, 2000, {.forwards = false,.minSpeed = 10, .earlyExitRange = 0.5}, false);
		turnto_point(-29, 18, 2500, {.earlyExitRange = 3});
		//chassis.moveToPose(30, 24, 0, 800);
		moveto_point(-29, 18, 1400, {}, false);
		intake_high(0);
		intake_bottom.move(-50);
		moveto_point(-29, 30, 600, {.maxSpeed = 90}, false);
		turnto_point(-10, 10, 1000, {.forwards = false, .minSpeed = 1, .earlyExitRange = 4});
		intake_high(127);

		chassis.moveToPose(-10, 10, 315, 600, {.forwards = false, .lead = 0.7, .minSpeed = 60});
		//moveto_point(-10, 10, 300, {.forwards = false, .maxSpeed = 20});
		chassis.moveToPoint(-10, 10, 800, {.forwards = false, .maxSpeed = 10});

		//score on middle goal
		intake_middle_skills(127);
		intake_bottom.move(0);
		intake_top.move(0);
		pros::Task::delay(400);
		intake_middle_skills(127);
		pros::Task::delay(600);
		intake_middle_skills(120);
		pros::Task::delay(800);
		pros::Task::delay(400);
		intake_middle_skills(127);

		pros::Task::delay(200);
		pros::Task::delay(200);
		intake_middle_skills(127);

		//move to matchloader
		moveto_point(-20, 20, 250, {}, false);
		intake_middle_skills(0);
		//swingto_heading(180, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 20, .earlyExitRange = 3}, false);
		swingto_point(-48, -46, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 20, .earlyExitRange = 5});

		//dist_to_center = distance_right.get()*0.0394+right_distance_from_center;
		//chassis.setPose(-72+dist_to_center, chassis.getPose().y, chassis.getPose().theta);
		intake_high(127);
		ears.set_value(true);
		

		moveto_point(-48, -46, 2000, {.minSpeed = 20, .earlyExitRange = 20});
		moveto_point(-48, -46, 2000, {.maxSpeed = 40, .minSpeed = 5, .earlyExitRange = 2});
		scraper.set_value(true);
		turnto_heading(270, 1000, {.maxSpeed = 100, .minSpeed = 1, .earlyExitRange = 3}, false);
		simple_dist_reset();
		moveto_matchload(3, 6);

		//move down the 2nd alley
		moveto_point(-52, -48, 3000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3}, false);
		intake_high(0);
		intake_index.move(127);
		turnto_point(-27, -60, 3000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 10}, false);

		moveto_point(-27, -60, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
		turnto_heading(270, 1000, {.minSpeed = 1, .earlyExitRange = 0.5}, false);
		simple_dist_reset();

		scraper.set_value(false);
		moveto_point(23, -62, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 4});
		
		//move to 4th matchloader
		//turnto_point(-44, 48, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 5}, false);
		moveto_point(42, -52, 600, {.forwards = false, .minSpeed = 40, .earlyExitRange = 4});
		scraper.set_value(true);
		intake_high(127);
		turnto_heading(90, 1200, {.minSpeed = 2, .earlyExitRange = 5}, false);
		simple_dist_reset();
		moveto_matchload(4, 6);

		//move to 2nd goal for the 1st time

		chassis.moveToPose(27, -48, 90, 3000, {.forwards = false});
		chassis.waitUntil(20);
		chassis.cancelMotion();
		moveto_point(0, -48, 800, {.forwards = false, .maxSpeed = 30});

		intake_high(0);
		intake_index.move(30);
		hood.set_value(true);
		intake_high(127);
		//chassis.moveToPoint(0, 48, 1900, {.forwards = false, .maxSpeed = 15});

		pros::Task::delay(3000);
		chassis.setPose(31, -48, chassis.getPose().theta);

		//park
		scraper.set_value(false); 
		chassis.moveToPose(65, -15, 0, 2000, {.lead = 0.4, .minSpeed = 100});
		pros::Task::delay(200);
		hood.set_value(false);

	//scraper.set_value(true);
	moveto_point(66, 0, 1300);

	}
	break;
	//SAWP
	case 1:
	{
		
		chassis.setPose(47, 0, 180); 
		intake_high(127);

		//push teamamtes
		//chassis.moveToPoint(47, -5, 300, {.minSpeed = 40, .earlyExitRange = 3});
		moveto_point(47, -5, 1000, {.minSpeed = 40, .earlyExitRange = 3}, false);
		
		//move to 1st matchload
		
		moveto_point(53, 43, 3000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 3}, false);
		scraper.set_value(true);
		turnto_heading(90, 1000, {.minSpeed = 1, .earlyExitRange = 2}, false);
		simple_dist_reset();
		moveto_matchload(1, 3);


		// move and score on goal
		moveto_point(48, 48, 100, {.forwards = false, .minSpeed = 40}, false);
		chassis.moveToPoint(27, 48, 3000, {.forwards = false, .minSpeed = 100});
		//simple_dist_reset();    
		chassis.waitUntil(18);
		hood.set_value(true);
		chassis.cancelMotion();
		moveto_point(0, 48, 600, {.forwards = false, .maxSpeed = 30}, false);
		pros::Task::delay(500);


		chassis.setPose(31, 48, chassis.getPose().theta);
		scraper.set_value(false);
		
		//move to stacks 1 and 2
		//moveto_point(38, 48, 1000, {.minSpeed = 20, .earlyExitRange = 3}, false);
		swingto_heading(180, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 50, .earlyExitRange = 10});
		intake_high(127);
		
		turnto_point(23, 23, 2000, {.minSpeed = 10, .earlyExitRange = 5});
		hood.set_value(false);
		chassis.moveToPoint(23, 23, 1000, {.minSpeed = 100, .earlyExitRange = 5});
		moveto_stack(23, 23);

		//turnto_point(22, -24, 2000, {.minSpeed = 10, .earlyExitRange = 3}, false);

		chassis.moveToPoint(22, -18, 2000, {.minSpeed = 90, .earlyExitRange = 4});
		moveto_stack(22, -23);
		scraper.set_value(true);
		moveto_point(38, -44, 2000, {.minSpeed = 10, .earlyExitRange = 5}, false);
		turnto_point(24, -45, 2000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 5});
		moveto_point(24, -45, 300, {.forwards = false, .minSpeed = 100});
		hood.set_value(true);
		moveto_point(0, -48, 200, {.forwards = false}, false);
		pros::Task::delay(1000);
		chassis.setPose(31, -48, chassis.getPose().theta);

		moveto_point(52, -48, 2000, {.minSpeed = 60, .earlyExitRange = 4});
		hood.set_value(false);
		moveto_matchload(4, 3);

		moveto_point(50, -48, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3}, false);
		swingto_point(10, -10, lemlib::DriveSide::LEFT, 2000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 10});

		//chassis.moveToPose(0, -0, 135, 400, {.forwards = false, .lead = 0.7});
		moveto_point(10, -10, 1000, {.forwards = false});
		chassis.moveToPoint(10, -10, 400, {.forwards = false, .maxSpeed = 20});
		scraper.set_value(false);
		pros::Task::delay(0);
		//score on middle goal
		intake_middle_skills(127);
		pros::Task::delay(500);
		
		
		turnto_point(38, -38, 200, {.earlyExitRange = 3});
		moveto_point(38, -38, 2000, {.minSpeed = 60, .earlyExitRange = 2});
		turnto_point(0, -38, 1500, {.forwards = false, .minSpeed = 1, .earlyExitRange = 15});
		moveto_point(0, -38, 500, {.forwards = false});
		moveto_point(12, -39, 1000, {.forwards = false});


		/*
		swingto_point(46, -48, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 15, .earlyExitRange = 5});
		moveto_point(46, -48, 3000, {.minSpeed = 20, .earlyExitRange = 7});
		scraper.set_value(true);
	
		chassis.swingToHeading(90, lemlib::DriveSide::LEFT,  2000, {.minSpeed = 40, .earlyExitRange = 3}, false);
		simple_dist_reset();
		moveto_matchload(4, 3);
		intake_bottom.move(0);
		chassis.moveToPose(27, -48, 90, 3000, {.forwards = false});
		chassis.waitUntil(18);
		hood.set_value(true);
		chassis.cancelMotion();
		moveto_point(0, -48, 700, {.forwards = false, .maxSpeed = 50}, false);
		intake_top.move(0);
		pros::Task::delay(300);
		scraper.set_value(false);
		hood.set_value(false);
		moveto_point(42, -48, 1000, {.minSpeed = 20, .earlyExitRange = 3}, false);
		intake_high(127);
		
		turnto_point(0, -0, 1500, {.forwards = false, .minSpeed = 1, .earlyExitRange = 3});
		chassis.moveToPose(0, -0, 135, 400, {.forwards = false, .lead = 0.7});
		moveto_point(10, -10, 500, {.forwards = false});
		chassis.moveToPoint(10, -10, 1000, {.forwards = false, .maxSpeed = 20});
		pros::Task::delay(0);
		//score on middle goal
		intake_middle_skills(127);
		pros::Task::delay(1200);
	`	*/
	}
	break;
	//left_side middle + long
	case 2:
	{
		
		//1st stack of 4
		chassis.setPose(45, -12, 270); 
		intake_high(127);
		chassis.swingToPoint(23, -23, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, -23, 3000, {.minSpeed = 60, .earlyExitRange = 4});
		moveto_stack(23, -23);

		// 1st matchloader
		chassis.turnToPoint(50, -47, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		//chassis.moveToPose(53, 45, 90, 1200, {.lead = 0.5, .minSpeed = 70}, false);
		chassis.moveToPoint(50, -47, 3000, {.minSpeed = 20, .earlyExitRange = 11});
		scraper.set_value(true);
		chassis.swingToHeading(90, lemlib::DriveSide::LEFT,  2000, {.minSpeed = 1, .earlyExitRange = 2}, false);

		simple_dist_reset();
		//pros::Task::delay(99999);
		moveto_matchload(4, 3);
		intake_bottom.move(0);
		chassis.moveToPose(27, -48, 270, 3000, {.forwards = false});
		
		chassis.waitUntil(16);
		hood.set_value(true);
		chassis.cancelMotion();
		chassis.moveToPoint(0, -48, 500, {.forwards = false, .maxSpeed = 50}, false);
		pros::Task::delay(1000);
		
		scraper.set_value(false);
		// move to middle goal
		chassis.moveToPoint(38, -48, 1000, {.minSpeed = 20, .earlyExitRange = 3}, false);
		intake_high(127);
		hood.set_value(false);
		chassis.turnToPoint(21, -21, 1000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 5});
		chassis.moveToPoint(21, -21, 1000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
		chassis.turnToPoint(0, -0, 1500, {.forwards = false, .minSpeed = 10, .earlyExitRange = 3});
		chassis.moveToPose(0, -0, 135, 300, {.forwards = false, .lead = 0.7});
		chassis.moveToPoint(0, -0, 600, {.forwards = false, .maxSpeed = 20});
		chassis.moveToPoint(0, -0, 1000, {.forwards = false, .maxSpeed = 20});
		//score on middle goal
		intake_middle_skills(160);
		pros::Task::delay(2000);

		//descore
		chassis.moveToPoint(chassis.getPose().x-0.5,
					chassis.getPose().y+0.5, 
					400, {.maxSpeed = 30});
		
		chassis.swingToPoint(14, -37, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 20, .earlyExitRange = 5});
		chassis.moveToPoint(14, -37, 2000, {.minSpeed = 20, .earlyExitRange = 2});
		chassis.turnToHeading(315, 300,{.minSpeed = 100, .earlyExitRange = 25} , false);
		chassis.turnToHeading(90, 5000, {.maxSpeed = 60});
		chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

		

	}
	break;
	//left_side 7
	case 3:
	{		
		//1st stack of 4
		chassis.setPose(45, -12, 270); 
		intake_high(127);
		chassis.swingToPoint(23, -23, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, -23, 3000, {.minSpeed = 60, .earlyExitRange = 4});
		moveto_stack(23, -23);

		// 1st matchloader
		chassis.turnToPoint(50, -47, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		//chassis.moveToPose(53, 45, 90, 1200, {.lead = 0.5, .minSpeed = 70}, false);
		chassis.moveToPoint(50, -47, 3000, {.minSpeed = 20, .earlyExitRange = 11});
		scraper.set_value(true);
		chassis.swingToHeading(90, lemlib::DriveSide::LEFT,  2000, {.minSpeed = 1, .earlyExitRange = 2}, false);

		simple_dist_reset();
		//pros::Task::delay(99999);
		moveto_matchload(4, 3);

		chassis.moveToPose(27, -48, 270, 3000, {.forwards = false});
		
		chassis.waitUntil(14);
		hood.set_value(true);
		chassis.cancelMotion();
		chassis.moveToPoint(0, -48, 500, {.forwards = false, .maxSpeed = 50}, false);
		pros::Task::delay(1000);
		scraper.set_value(false);
		
		descore();


	}
	break;
	//left_side 4
	case 4:
	{
		//1st stack of 4
		chassis.setPose(45, -12, 270);
		intake_high(127);
		chassis.swingToPoint(23, -23, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, -23, 3000, {.minSpeed = 20, .earlyExitRange = 2});
		moveto_stack(23, -23);

		chassis.turnToPoint(36, -46, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		chassis.moveToPoint(36, -46, 3000, {.minSpeed = 20, .earlyExitRange = 8});
		chassis.turnToPoint(27, -46, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 20});
		chassis.moveToPoint(27, -46, 700, {.forwards = false});
		pros::Task::delay(200);
		hood.set_value(true);
		pros::Task::delay(1000);

		descore();

	}
	break;
	//right_side low + long
	case 5:
	{
		
		//1st stack of 4
		chassis.setPose(45, 12, 270); 
		intake_high(127);
		chassis.swingToPoint(23, 23, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, 23, 3000, {.minSpeed = 20, .earlyExitRange = 2});
		moveto_stack(23, 23);

		// 1st matchloader
		chassis.turnToPoint(46, 48, 2000, {.minSpeed = 10, .earlyExitRange = 15});
		//chassis.moveToPose(53, 45, 90, 1200, {.lead = 0.5, .minSpeed = 70}, false);
		//chassis.moveToPoint(46, 48, 3000, {.minSpeed = 20, .earlyExitRange = 11});
		moveto_point(46, 48, 3000, {.minSpeed = 20, .earlyExitRange = 6}, false);
		scraper.set_value(true);
		chassis.swingToHeading(90, lemlib::DriveSide::RIGHT,  2000, {.minSpeed = 1, .earlyExitRange = 3}, false);
		simple_dist_reset();
		moveto_matchload(1, 3);
		intake_bottom.move(0);
		chassis.moveToPose(27, 48, 270, 3000, {.forwards = false});
		
		chassis.waitUntil(14);
		hood.set_value(true);
		pros::Task::delay(200);
		chassis.cancelMotion();
		chassis.moveToPoint(0, 48, 600, {.forwards = false, .maxSpeed = 30}, false);
		pros::Task::delay(900);
		scraper.set_value(false);

		// move to middle goal
		moveto_point(43, 48, 1000, {.minSpeed = 20, .earlyExitRange = 3}, false);
		intake_high(127);
		hood.set_value(false);
		turnto_point(0, 0, 1000, {.minSpeed = 10, .earlyExitRange = 5});
		//chassis.turnToPoint(8.5, 11.5, 2000, {.maxSpeed = 90, .minSpeed = 1, .earlyExitRange = 2});
		chassis.moveToPoint(0, 0, 600, {});
		chassis.moveToPoint(0, 0, 2100, {.maxSpeed = 40});

		//score on low goal
		pros::Task::delay(800);
		funnel.set_value(true);
		intake_low(127);
		intake_index.move(-60);
		pros::Task::delay(1300);

		//descore
		ears.set_value(true);
		moveto_point(33, 35, 2000, {.forwards = false});
		chassis.turnToHeading(270, 500,{} , false);
		ears.set_value(false);
		funnel.set_value(false);
		chassis.moveToPoint(6, chassis.getPose().y, 1500);
		chassis.turnToHeading(250, 5000, {.maxSpeed = 60});
		chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	}
	break;
	//right_side 7
	case 6:
	{	
		//1st stack of 4
		chassis.setPose(45, 12, 270); 
		intake_high(127);
		chassis.swingToPoint(23, 23, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, 23, 3000, {.minSpeed = 20, .earlyExitRange = 2});
		moveto_stack(23, 23);

		// 1st matchloader
		chassis.turnToPoint(46, 48, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		//chassis.moveToPose(53, 45, 90, 1200, {.lead = 0.5, .minSpeed = 70}, false);
		chassis.moveToPoint(46, 48, 3000, {.minSpeed = 20, .earlyExitRange = 11});
		scraper.set_value(true);
		chassis.swingToHeading(90, lemlib::DriveSide::RIGHT,  2000, {.minSpeed = 1, .earlyExitRange = 3}, false);
		simple_dist_reset();
		moveto_matchload(1, 3);

		chassis.moveToPose(27, 48, 270, 3000, {.forwards = false});
		
		chassis.waitUntil(18);
		hood.set_value(true);
		chassis.cancelMotion();
		chassis.moveToPoint(0, 48, 500, {.forwards = false, .maxSpeed = 30}, false);
		pros::Task::delay(1000);
		scraper.set_value(false);
		descore();



	}
	break;
	//right_side 4
	case 7:
	{
		
		//1st stack of 4
		chassis.setPose(45, 12, 270);
		intake_high(127);
		chassis.swingToPoint(23, 23, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, 23, 3000, {.minSpeed = 20, .earlyExitRange = 3});
		moveto_stack(23, 23);

		chassis.turnToPoint(36, 45, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		chassis.moveToPoint(36, 45, 3000, {.minSpeed = 20, .earlyExitRange = 8});
		chassis.turnToPoint(27, 45, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 20});
		chassis.moveToPoint(27, 45, 700, {.forwards = false, .minSpeed = 60});
		pros::Task::delay(200);
		hood.set_value(true);
		pros::Task::delay(1000);

		descore();


	}
	break;
	default:
		break;
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
	
	//move_skills_low();

	intake_high(0);
	chassis.cancelAllMotions();
	//g_auton_started = true;
	g_op_control_started = true;
	hood.set_value(false);
	g_hood_state = false;
	scraper.set_value(false);
	g_scraper_state = false;
	chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	intake_index.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	

	bool index_low = true;

	pros::Task driver_systems_task([&]() {
	while (true)
	{
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intake_high(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			if(g_auton_started)
			{
				intake_low(127);
			}
			else
			{
				intake_low(127*0.44);
				intake_index.move(127);


			}
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			if(!g_auton_started)
			{
				intake_middle_skills(120);
				
			}
			else
			{
				intake_middle(100);
			}


		}
		else
		{
			intake_high(0);
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
		{
			if(g_funnel_state)
			{
				funnel.set_value(false);
				g_funnel_state = false;
			}
			else
			{
				funnel.set_value(true);
				g_funnel_state = true;
			}
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



		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			hood.set_value(true);
		}
		else
		{
			hood.set_value(false);
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


	int leftY = 0;
	int rightX = 0;
    while (true) {
		// get left y and right x positions
		leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.curvature(leftY, rightX);
	}
	
		
	// delay to save resources
    pros::delay(25);
}
