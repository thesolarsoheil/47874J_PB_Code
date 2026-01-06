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

// setup

// a negative number shows that the motor is reversed
// if a 3 wire has a number for the port, the number is just the letter of the port. 
//ex: 1 = A, 2 = B, 3 = C, 4 = D

//intake_motors
pros::Motor intake_bottom(10, pros::MotorGears::blue); // intake motor on port 19
pros::Motor intake_top(9, pros::MotorGears::green); // lift motor on port 12
pros::Motor intake_index(-4, pros::MotorGears::green); // lift motor on port 12

// condensed motors into motor groups
pros::MotorGroup left_motor_group({-18, -19, 20}, pros::MotorGears::blue); //the right side of the drivetrain
pros::MotorGroup right_motor_group({6, -7, 8}, pros::MotorGears::blue); //the left side of the drivetrain

lemlib::Drivetrain drivetrain_6m(&left_motor_group, //motors that are on the left channel
                              &right_motor_group, //motors that a	re on the right channel
                              11.25, // track width
                              lemlib::Omniwheel::NEW_275, //the specific vex wheels used
                              600, // the rpm of the driven axels
                              8 //horizontal drift
);

// create an imu on port 12
pros::Imu imu(15);

pros::ADIDigitalOut ears(4);
pros::ADIDigitalOut scraper(2);
pros::ADIDigitalOut hood(1);
pros::ADIDigitalOut funnel(3);

// left tracking wheel encoder
pros::Rotation vertical_encoder(13);

pros::Distance distance_left(14);
pros::Distance distance_right(21);
pros::Distance distance_front(3);

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
                                              150, // derivative gain (kD)150
                                              2, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              90//250 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller_small(2.9, // proportional gain (kP)//3.2
                                              0.07, // integral gain (kI)//0.25
                                              23,// derivative gain (kD)//20
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
lemlib::ExpoDriveCurve steer_curve	(6, // joystick deadband out of 127
                                  6, // minimum output where drivetrain will move out of 127
                                  1.005 //1.03 expo curve gain
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
int chosen_auto = 0; // determines which auton path is going to be run

//pneumatics toggles
float g_ears_state = false;
float g_scraper_state = false;
float g_hood_state = false;
float g_funnel_state = false;

float left_distance_from_center = 4;
float right_distance_from_center = 4;
float front_distance_from_center = 7;
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

	intake_top.move(volts*0.35);
	intake_index.move(-volts*0.25);

}
void intake_low(float volts)
{
	intake_bottom.move(-volts);
	intake_top.move(-volts);
	intake_index.move(-volts);
}


void simple_dist_reset()
{
    pros::screen::print(pros::E_TEXT_SMALL, 7, "angle read 1: %f", chassis.getPose().theta); // x
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

     pros::screen::print(pros::E_TEXT_SMALL, 8, "angle read 2: %f", chassis.getPose().theta); // x
	
	
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
	chassis.moveToPose(x, y, theta, 5000, {.lead = 0.2});
	
	while(hypot(chassis.getPose().x - x, chassis.getPose().y - y) > 16)
	{
		pros::Task::delay(10);
	}
	chassis.cancelMotion();



	chassis.moveToPose(x, y, theta, 5000, {.lead = 0.2, .maxSpeed = 60});
	float max_velocity = 550;
	float tracker_velocity = 550;
	float final_v_rpm = 10;

	while(tracker_velocity > final_v_rpm)
	{
		tracker_velocity = fabs(vertical_encoder.get_velocity()*100*360*60);
		pros::Task::delay(10);
	}
	chassis.cancelMotion();

	if(x > 0)
	{
		x += 7;
	}
	else
	{
		x -= 7;
	}

	if(balls == 3)
	{
	chassis.moveToPoint(0, y, 30, {.forwards = false, .maxSpeed = 30}, false);
	chassis.moveToPoint(x, y, 200, {.maxSpeed = 20}, false);

	}
	else if(balls == 6)
	{
	//chassis.moveToPoint(0, y, 30, {.forwards = false, .maxSpeed = 30}, false);
	chassis.turnToHeading(theta+20, 60, {.minSpeed = 60});
	for(int i = 0; i < 4; i++)
	{
		chassis.turnToHeading(theta-20, 125, {.minSpeed = 60});

		chassis.moveToPoint(x, y, 40, {}, false);
		//chassis.moveToPoint(0, y, 30, {.forwards = false, .maxSpeed = 20}, false);

		chassis.turnToHeading(theta+20, 125, {.minSpeed = 60});
		chassis.moveToPoint(x, y, 40, {.minSpeed = 80}, false);
		
	}
	chassis.turnToHeading(theta, 30, {.minSpeed = 30}, false);
	chassis.moveToPoint(x, y, 300, {.minSpeed = 30}, false);
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

	chosen_auto = 9;

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

	if(chosen_auto == 0)
	{
	
	//1st stack of 4
	chassis.setPose(45, 12, 270); 
	intake_high(127);
	ears.set_value(true);
	chassis.swingToPoint(24, 24, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
	chassis.moveToPoint(24, 24, 3000, {.minSpeed = 20, .earlyExitRange = 2});
	moveto_stack(24, 24);

	// 1st matchloader
	chassis.turnToPoint(46, 48, 2000, {.minSpeed = 10, .earlyExitRange = 10});
	//chassis.moveToPose(53, 45, 90, 1200, {.lead = 0.5, .minSpeed = 70}, false);
	chassis.moveToPoint(46, 48, 3000, {.minSpeed = 20, .earlyExitRange = 13});
	scraper.set_value(true);
	chassis.swingToHeading(90, lemlib::DriveSide::RIGHT,  2000, {.minSpeed = 1, .earlyExitRange = 3}, false);
	simple_dist_reset();
	moveto_matchload(1, 6);

	//move down the 1st alley
	chassis.moveToPoint(52, 48, 3000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3}, false);
	intake_high(0);
	intake_index.move(127);
	chassis.turnToPoint(24, 60, 3000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 10}, false);

	chassis.moveToPoint(24, 60, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
	chassis.turnToHeading(90, 1000, {.minSpeed = 20, .earlyExitRange = 3}, false);
	simple_dist_reset();
	scraper.set_value(false);
	chassis.moveToPoint(-23, 60, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 4});
	
	//move to 1st goal for the 1st time
	chassis.turnToPoint(-40, 48, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 5}, false);
	chassis.moveToPoint(-40, 48, 500, {.forwards = false, .minSpeed = 80});
	chassis.turnToHeading(270, 1200, {.maxSpeed = 100, .minSpeed = 10, .earlyExitRange = 3}, false);
	simple_dist_reset();
	//score on 1st goal for the 1st time

	chassis.moveToPose(0, 48, 270, 800, {.forwards = false}, false);
	chassis.moveToPoint(0, 48, 500, {.forwards = false, .maxSpeed = 30});
	intake_high(127);
	intake_bottom.move(0);
	hood.set_value(true);
	pros::Task::delay(500);
	intake_high(127);
	pros::Task::delay(2500);
	
	chassis.setPose(-31, 48, chassis.getPose().theta);

	//2nd matchloader
	scraper.set_value(true);

	chassis.moveToPoint(-52, 48, 3000, {.maxSpeed = 60, .minSpeed = 1, .earlyExitRange = 2});
	pros::Task::delay(200);
	hood.set_value(false);
	chassis.waitUntilDone();
	moveto_matchload(2, 6);

	//move and score on 1st goal for the 2nd time
	chassis.moveToPose(-27, 48, 270, 3000, {.forwards = false});
	chassis.waitUntil(18);
	chassis.cancelMotion();
	chassis.moveToPoint(0, 48, 600, {.forwards = false, .maxSpeed = 30});

	intake_high(0);
	intake_index.move(30);
	hood.set_value(true);
	pros::Task::delay(400);
	intake_high(127);
	
	pros::Task::delay(600);
	intake_high(90);
	pros::Task::delay(1300);
	chassis.setPose(-31, 48, chassis.getPose().theta);
	
	//chassis.setPose(-31, 48, 270);
	intake_high(127);
	ears.set_value(true);
	scraper.set_value(false);

	//move to in front of blue park zone

	chassis.moveToPoint(-39, 48, 800, {.minSpeed = 1, .earlyExitRange = 2}, false);
	hood.set_value(false);
	chassis.swingToPoint(-42, 0, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 20, .earlyExitRange = 5});
	chassis.moveToPoint(-42, 0, 2000);
	chassis.waitUntil(25);
	chassis.cancelAllMotions();
	chassis.moveToPoint(-42, 0, 800, {.maxSpeed = 30});
	
	//enter park zone and clear balls
	chassis.turnToHeading(270, 600, {}, false);
	intake_high(127);
	chassis.moveToPoint(-72, chassis.getPose().y, 1200, {.maxSpeed = 110});
	pros::Task::delay(600);
	scraper.set_value(true);
	pros::Task::delay(300);
	pros::Task::delay(300);
	for(int i = 0; i < 1; i++){
		chassis.turnToHeading(260, 200);
		chassis.turnToHeading(280, 200);
	}
	scraper.set_value(false);

	chassis.turnToHeading(270, 300, {}, false);

	chassis.moveToPoint(-72, chassis.getPose().y, 400, {.maxSpeed = 90}, false);
	//chassis.moveToPoint(72, chassis.getPose().y, 100, {.forwards = false, .maxSpeed = 90}, false);
	chassis.setPose(0, 0, chassis.getPose().theta);

	for(int i = 0; i < 2; i++){
		//chassis.turnToHeading(250, 200);
		//chassis.turnToHeading(290, 200);
		
		chassis.moveToPoint(2, 0, 200, {.forwards = false});
		chassis.turnToPoint(-24, 24, 200);
		//chassis.moveToPoint(-24, 12, 200, {}, false);
		chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 200);
		chassis.turnToPoint(2, 0, 100, {.forwards = false, .minSpeed = 30});
		chassis.moveToPoint(2, 0, 200, {.forwards = false});
		chassis.turnToPoint(-24, -24, 200);
		//chassis.moveToPoint(-24, -12, 200, {}, false);
		chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 200);
		chassis.turnToPoint(2, 0, 100, {.forwards = false, .minSpeed = 30});

	}
	
	//leave park zone
	chassis.turnToHeading(270, 300, {}, false);
	chassis.moveToPoint(32, chassis.getPose().y, 700, {.forwards = false}, false);
	pros::Task::delay(400);


	// reset position after park
	chassis.turnToHeading(270, 1000, {.minSpeed = 35, .earlyExitRange = 2}, false);
	simple_dist_reset();
	dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
	chassis.setPose(-72+dist_to_center, 
						chassis.getPose().y, 
						chassis.getPose().theta);

	//move to middle goal
	chassis.turnToPoint(-21, 21, 1000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 2});
	chassis.moveToPoint(-21, 21, 1000, {.forwards = false});
	chassis.turnToPoint(-10, 10, 1500, {.forwards = false});
	intake_high(127);
	chassis.moveToPose(-10, 10, 315, 600, {.forwards = false, .lead = 0.7});
	//chassis.moveToPoint(-10, 10, 300, {.forwards = false, .maxSpeed = 20});
	chassis.moveToPoint(-10, 10, 1200, {.forwards = false, .maxSpeed = 10});

	//score on middle goal
	intake_middle_skills(127);
	intake_bottom.move(0);
	intake_top.move(0);
	pros::Task::delay(400);
	intake_middle_skills(127);
	pros::Task::delay(600);
	intake_middle_skills(80);
	pros::Task::delay(500);
	chassis.moveToPoint(chassis.getPose().x-1,
				chassis.getPose().y+1, 
				200, {.maxSpeed = 30});
	pros::Task::delay(700);
	//intake_top.move(-50);
	pros::Task::delay(70);
	//intake_top.move(127);
	pros::Task::delay(300);
	intake_middle_skills(127);
	
	chassis.moveToPoint(0, 0, 450, {.forwards = false, .maxSpeed = 30}, false);
	pros::Task::delay(700);
	intake_index.move(127);
	pros::Task::delay(100);
	intake_middle(127);
	pros::Task::delay(300);

	//move to 3rd stack of 4
	chassis.moveToPoint(-20, 20, 250, {}, false);
	intake_middle_skills(0);
	chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 20, .earlyExitRange = 3}, false);
	
	
	dist_to_center = distance_right.get()*0.0394+right_distance_from_center;
	chassis.setPose(-72+dist_to_center, chassis.getPose().y, chassis.getPose().theta);
	chassis.moveToPoint(-22, -26, 3000, {.minSpeed = 20, .earlyExitRange = 2});
	intake_high(127);
	moveto_stack(-22, -26);
	
	// 3rd matchloader
	chassis.swingToPoint(-46, -48, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 10, .earlyExitRange = 5});
	scraper.set_value(true);
	chassis.moveToPose(-46, -46, 225, 200);
	chassis.moveToPoint(-46, -46, 3000, {.minSpeed = 20, .earlyExitRange = 7});
	
	chassis.swingToHeading(270,lemlib::DriveSide::RIGHT,  2000, {.minSpeed = 40, .earlyExitRange = 3}, false);
	simple_dist_reset();
	moveto_matchload(3, 6);

	//move down the 2nd alley
	chassis.moveToPoint(-52, -48, 3000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3});
	chassis.turnToPoint(-24, -60, 3000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 10});
	intake_high(0);
	intake_index.move(127);
	chassis.moveToPoint(-24, -60, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
	chassis.turnToHeading(270, 1000, {.minSpeed = 20, .earlyExitRange = 3}, false);
	simple_dist_reset();
	scraper.set_value(false);
	chassis.moveToPoint(23, -60, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 4});

	//move to 2nd goal for the 1st time
	chassis.turnToPoint(40, -48, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 5}, false);
	chassis.moveToPoint(40, -48, 500, {.forwards = false, .minSpeed = 80});
	chassis.turnToHeading(90, 1200, {.maxSpeed = 100, .minSpeed = 10, .earlyExitRange = 3}, false);
	simple_dist_reset();

	//score on 2nd goal for the 1st time

	chassis.moveToPose(0, -48, 90, 600, {.forwards = false}, false);
	chassis.moveToPoint(0, -48, 500, {.forwards = false, .maxSpeed = 30});
	intake_high(127);
	hood.set_value(true);
	pros::Task::delay(3000);
	chassis.setPose(31, -48, chassis.getPose().theta);

	//4th matchloader
	scraper.set_value(true);
	chassis.moveToPoint(52, -48, 3000, {.maxSpeed = 40, .minSpeed = 1, .earlyExitRange = 2}, false);
	hood.set_value(false);
	moveto_matchload(4, 6);

	//move and score on 2nd goal for the 2nd time
	chassis.moveToPose(27, -48, 90, 3000, {.forwards = false});
	chassis.waitUntil(14);
	hood.set_value(true);
	pros::Task::delay(400);
	intake_high(127);
	chassis.cancelMotion();
	pros::Task::delay(600);
	intake_high(90);
	pros::Task::delay(1300);
	
	chassis.setPose(31, -48, 90);
	intake_high(127);
	//park
	scraper.set_value(false); 
	chassis.moveToPose(65, -15, 0, 2000, {.lead = 0.4, .minSpeed = 100});
	pros::Task::delay(200);
	hood.set_value(false);
	
	

	//scraper.set_value(true);
	chassis.moveToPoint(66, 0, 4000, {.maxSpeed = 70}, false);
	while (true)
	{
		if((distance_front.get()*0.0394+front_distance_from_center) < 64)
		{
			break;
		}
		pros::Task::delay(10);
	}
	chassis.cancelMotion();
	
	scraper.set_value(false);

	}

	else if(chosen_auto == 1)
	{
		chassis.setPose(47, 0, 180); 
		intake_high(127);

		//push teamamtes
		chassis.moveToPoint(47, -5, 300, {.minSpeed = 40, .earlyExitRange = 3});
		
		//move to 1st matchload
		chassis.moveToPoint(47, 40, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2}, false);
		scraper.set_value(true);
		chassis.turnToHeading(90, 1000, {.minSpeed = 10, .earlyExitRange = 8}, false);
		simple_dist_reset();
		moveto_matchload(1, 3);
		
		// move and score on goal
		chassis.moveToPoint(48, 48, 100, {.forwards = false, .minSpeed = 40}, false);
		chassis.moveToPose(27, 48, 270, 3000, {.forwards = false});
		//simple_dist_reset();    
		chassis.waitUntil(18);
		hood.set_value(true);
		chassis.cancelMotion();
		chassis.moveToPoint(0, 48, 600, {.forwards = false, .maxSpeed = 30}, false);
		pros::Task::delay(800);
		chassis.setPose(31, 48, chassis.getPose().theta);
		scraper.set_value(false);
		
		//move to stacks 1 and 2
		chassis.moveToPoint(38, 48, 1000, {.minSpeed = 20, .earlyExitRange = 3}, false);
		intake_high(127);
		hood.set_value(false);
		chassis.turnToPoint(23, 23, 2000, {.minSpeed = 10, .earlyExitRange = 5});
		chassis.moveToPoint(23, 23, 1000, {.minSpeed = 20, .earlyExitRange = 3});
		moveto_stack(23, 23);

		chassis.turnToHeading(180, 2000, {.minSpeed = 10, .earlyExitRange = 3}, false);
		dist_to_center = distance_left.get()*0.0394+left_distance_from_center;
		chassis.setPose(72-dist_to_center, 
						chassis.getPose().y, 
						chassis.getPose().theta);

		chassis.moveToPoint(22, -24, 2000, {.minSpeed = 20, .earlyExitRange = 2});
		moveto_stack(22, -24);
		
		//move to middle goal
		chassis.turnToPoint(0, 0, 1500, {.forwards = false, .minSpeed = 10, .earlyExitRange = 3});
		//pros::Task::delay(999999);
		chassis.moveToPose(0, 0, 135, 400, {.forwards = false, .lead = 0.7});
		chassis.moveToPoint(0, 0, 600, {.forwards = false, .maxSpeed = 40});
		chassis.moveToPoint(0, 0, 1000, {.forwards = false, .maxSpeed = 20});
		//score on middle goal
		intake_middle_skills(127);
		pros::Task::delay(950);
		intake_middle(0);
		intake_index.move(-70);
		pros::Task::delay(100);
		intake_index.move(0);

		//move to 2nd matchload
		chassis.moveToPoint(46, -48, 3000, {.minSpeed = 20, .earlyExitRange = 17});
		scraper.set_value(true);
		chassis.swingToHeading(90, lemlib::DriveSide::LEFT,  2000, {.minSpeed = 1, .earlyExitRange = 3}, false);
		simple_dist_reset();
		intake_high(127);
		moveto_matchload(4, 3);	
		//move and score on goal
		chassis.moveToPose(27, -48, 270, 3000, {.forwards = false});
		chassis.waitUntil(19);
		hood.set_value(true);
		chassis.cancelMotion();
		chassis.moveToPoint(0, -48, 500, {.forwards = false, .maxSpeed = 30}, false);
		pros::Task::delay(1000);


	}
	else if(chosen_auto == 2)
	{
		//left_side middle + long
		
		//1st stack of 4
		chassis.setPose(45, -12, 270); 
		intake_high(127);
		chassis.swingToPoint(23, -23, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, -23, 3000, {.minSpeed = 40, .earlyExitRange = 3});
		moveto_stack(23, -23);

		//stack under goal
		chassis.moveToPoint(13, -30, 2000, {.minSpeed = 20, .earlyExitRange = 2});
		chassis.swingToPoint(8, -41, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 20, .earlyExitRange = 5});
		chassis.moveToPoint(8, -41, 2000, {.minSpeed = 20, .earlyExitRange = 2}, false);		
		scraper.set_value(true);
		pros::Task::delay(150);
		chassis.moveToPoint(9, -36, 300, {.forwards = false, .maxSpeed = 60});
		pros::Task::delay(80);
		scraper.set_value(false);
		chassis.moveToPoint(7, -40, 300, {.maxSpeed = 60}, false);

		// 1st matchloader
		chassis.swingToPoint(24, -34, lemlib::DriveSide::LEFT, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 10});
		chassis.moveToPoint(24, -34, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
		chassis.swingToPoint(46, -44, lemlib::DriveSide::LEFT, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 10});
		chassis.moveToPoint(46, -44, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
		scraper.set_value(true);
		chassis.turnToHeading(90, 1000, {.maxSpeed = 100, .minSpeed = 1, .earlyExitRange = 3}, false);
		simple_dist_reset();
		moveto_matchload(4, 3);

		// move and score on goal
		chassis.moveToPose(27, -48, 270, 3000, {.forwards = false});
		intake_bottom.move(0);
		chassis.waitUntil(17);
		hood.set_value(true);
		chassis.cancelMotion();
		chassis.moveToPoint(0, -48, 500, {.forwards = false, .maxSpeed = 50}, false);
		pros::Task::delay(1000);
		chassis.setPose(31, -48, chassis.getPose().theta);
		scraper.set_value(false);

		// move to middle goal
		chassis.moveToPoint(38, -48, 1000, {.minSpeed = 20, .earlyExitRange = 3}, false);
		intake_high(127);
		hood.set_value(false);
		chassis.turnToPoint(19, -23, 1000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 5});
		chassis.moveToPoint(19, -23, 1000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
		chassis.turnToPoint(10, -14, 1500, {.forwards = false, .minSpeed = 10, .earlyExitRange = 3});
		chassis.moveToPose(10, -14, 135, 400, {.forwards = false, .lead = 0.7});
		chassis.moveToPoint(10, -14, 600, {.forwards = false, .maxSpeed = 20});
		chassis.moveToPoint(10, -14, 1000, {.forwards = false, .maxSpeed = 20});
		//score on middle goal
		intake_middle_skills(127);
		pros::Task::delay(1200);

		//descore
		chassis.moveToPoint(chassis.getPose().x-0.5,
					chassis.getPose().y+0.5, 
					400, {.maxSpeed = 30});
		
		chassis.swingToPoint(14, -37, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 20, .earlyExitRange = 5});
		chassis.moveToPoint(14, -37, 2000, {.minSpeed = 20, .earlyExitRange = 2});
		chassis.turnToHeading(315, 2000,{.minSpeed = 100, .earlyExitRange = 25} , false);
		chassis.turnToHeading(90, 5000,{} , false);
		
		

	}
	else if(chosen_auto == 3)
	{
		//left_side 9

		//1st stack of 4
		chassis.setPose(45, -12, 270); 
		intake_high(127);
		chassis.swingToPoint(23, -23, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, -23, 3000, {.minSpeed = 40, .earlyExitRange = 3});
		moveto_stack(23, -23);

		//stack under goal
		chassis.moveToPoint(13, -30, 2000, {.minSpeed = 20, .earlyExitRange = 2});
		chassis.swingToPoint(8, -41, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 20, .earlyExitRange = 5});
		chassis.moveToPoint(8, -41, 2000, {.minSpeed = 20, .earlyExitRange = 2}, false);		
		scraper.set_value(true);
		pros::Task::delay(150);
		chassis.moveToPoint(9, -36, 300, {.forwards = false, .maxSpeed = 60});
		pros::Task::delay(150);
		scraper.set_value(false);
		chassis.moveToPoint(7, -40, 300, {.maxSpeed = 60}, false);

		// 1st matchloader
		chassis.swingToPoint(24, -32, lemlib::DriveSide::LEFT, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 10});
		chassis.moveToPoint(24, -32, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
		chassis.swingToPoint(46, -45, lemlib::DriveSide::LEFT, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 10});
		chassis.moveToPoint(46, -45, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
		scraper.set_value(true);
		chassis.turnToHeading(90, 1000, {.maxSpeed = 100, .minSpeed = 1, .earlyExitRange = 3}, false);
		simple_dist_reset();
		moveto_matchload(4, 3);

		// move and score on goal
		chassis.moveToPose(27, -48, 270, 3000, {.forwards = false});
		
		chassis.waitUntil(18);
		hood.set_value(true);
		chassis.cancelMotion();
		chassis.moveToPoint(0, -48, 500, {.forwards = false, .maxSpeed = 30}, false);
		pros::Task::delay(1000);
		chassis.setPose(31, -48, chassis.getPose().theta);
		scraper.set_value(false);

		
		chassis.turnToHeading(10, 2000, {.minSpeed = 100,.earlyExitRange = 10}, false);
		chassis.turnToPoint(22, -46, 2000, {.forwards = false, .minSpeed = 3, .earlyExitRange = 5}, false);
		chassis.moveToPoint(22, -46, 300, {.forwards = false, .minSpeed = 100});
		chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 500, {.minSpeed = 10, .earlyExitRange = 5}, false);
		chassis.moveToPoint(24, chassis.getPose().y, 9000, {.forwards = false,.maxSpeed = 60, .minSpeed = 10, .earlyExitRange = 3}, false);
		

	}
	else if(chosen_auto == 4)
	{
		//left_side 7
		
		//1st stack of 4
		chassis.setPose(45, -12, 270); 
		intake_high(127);
		chassis.swingToPoint(23, -23, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, -23, 3000, {.minSpeed = 20, .earlyExitRange = 2});
		moveto_stack(23, -23);

		// 1st matchloader
		chassis.turnToPoint(46, -47, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		//chassis.moveToPose(53, 45, 90, 1200, {.lead = 0.5, .minSpeed = 70}, false);
		chassis.moveToPoint(46, -47, 3000, {.minSpeed = 20, .earlyExitRange = 15});
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
		chassis.setPose(31, -48, chassis.getPose().theta);
		scraper.set_value(false);
		
		chassis.turnToHeading(10, 2000, {.minSpeed = 100,.earlyExitRange = 10}, false);
		chassis.turnToPoint(22, -46, 2000, {.forwards = false, .minSpeed = 3, .earlyExitRange = 5}, false);
		chassis.moveToPoint(22, -46, 300, {.forwards = false, .minSpeed = 100});
		chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 500, {.minSpeed = 10, .earlyExitRange = 5}, false);
		chassis.moveToPoint(24, chassis.getPose().y, 9000, {.forwards = false,.maxSpeed = 60, .minSpeed = 10, .earlyExitRange = 3}, false);

	}		
	else if(chosen_auto == 5)
	{
		//left_side 4

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

		chassis.setPose(31, -48, chassis.getPose().theta);

		chassis.turnToHeading(10, 2000, {.minSpeed = 100,.earlyExitRange = 5}, false);
		chassis.turnToPoint(22, -46, 2000, {.forwards = false, .minSpeed = 3, .earlyExitRange = 5}, false);
		chassis.moveToPoint(22, -46, 300, {.forwards = false, .minSpeed = 100});
		chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 500, {.minSpeed = 10, .earlyExitRange = 5}, false);
		chassis.moveToPoint(25, chassis.getPose().y, 9000, {.forwards = false,.maxSpeed = 60, .minSpeed = 10}, false);
		
	}
	else if(chosen_auto == 6)
	{
		//right_side 9
		
		//1st stack of 4
		chassis.setPose(45, 12, 270); 
		intake_high(127);
		chassis.swingToPoint(23, 23, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, 23, 3000, {.minSpeed = 40, .earlyExitRange = 3});
		moveto_stack(23, 23);

		//stack under goal
		chassis.moveToPoint(13, 30, 2000, {.minSpeed = 20, .earlyExitRange = 2});
		chassis.swingToPoint(8, 41, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 20, .earlyExitRange = 5});
		chassis.moveToPoint(8, 41, 2000, {.minSpeed = 20, .earlyExitRange = 2}, false);		
		scraper.set_value(true);
		pros::Task::delay(150);
		chassis.moveToPoint(9, 36, 300, {.forwards = false, .maxSpeed = 60});
		pros::Task::delay(150);
		scraper.set_value(false);
		chassis.moveToPoint(7, 40, 300, {.maxSpeed = 60}, false);

		// 1st matchloader
		chassis.swingToPoint(24, 32, lemlib::DriveSide::RIGHT, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 10});
		chassis.moveToPoint(24, 32, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
		chassis.swingToPoint(46, 45, lemlib::DriveSide::RIGHT, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 10});
		chassis.moveToPoint(46, 45, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
		scraper.set_value(true);
		chassis.turnToHeading(90, 1000, {.maxSpeed = 100, .minSpeed = 1, .earlyExitRange = 3}, false);
		simple_dist_reset();
		moveto_matchload(1, 3);

		// move and score on goal
		chassis.moveToPose(27, 48, 270, 3000, {.forwards = false});
		
		chassis.waitUntil(18);
		hood.set_value(true);
		chassis.cancelMotion();
		chassis.moveToPoint(0, 48, 500, {.forwards = false, .maxSpeed = 30}, false);
		pros::Task::delay(1000);
		chassis.setPose(31, -48, chassis.getPose().theta);
		scraper.set_value(false);

		
		chassis.turnToHeading(10, 2000, {.minSpeed = 100,.earlyExitRange = 10}, false);
		chassis.turnToPoint(22, -46, 2000, {.forwards = false, .minSpeed = 3, .earlyExitRange = 5}, false);
		chassis.moveToPoint(22, -46, 300, {.forwards = false, .minSpeed = 100});
		chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 500, {.minSpeed = 10, .earlyExitRange = 5}, false);
		chassis.moveToPoint(24, chassis.getPose().y, 9000, {.forwards = false,.maxSpeed = 60, .minSpeed = 10, .earlyExitRange = 3}, false);
		
	}
	else if(chosen_auto == 7)
	{
		//right_side 7
	
		//1st stack of 4
		chassis.setPose(45, 12, 270); 
		intake_high(127);
		chassis.swingToPoint(23, 23, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, 23, 3000, {.minSpeed = 20, .earlyExitRange = 2});
		moveto_stack(23, 23);

		// 1st matchloader
		chassis.turnToPoint(46, 48, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		//chassis.moveToPose(53, 45, 90, 1200, {.lead = 0.5, .minSpeed = 70}, false);
		chassis.moveToPoint(46, 48, 3000, {.minSpeed = 20, .earlyExitRange = 15});
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
		chassis.setPose(31, -48, chassis.getPose().theta);
		scraper.set_value(false);
		
		chassis.turnToHeading(10, 2000, {.minSpeed = 100,.earlyExitRange = 5}, false);
		chassis.turnToPoint(22, -46, 2000, {.forwards = false, .minSpeed = 3, .earlyExitRange = 5}, false);
		chassis.moveToPoint(22, -46, 300, {.forwards = false, .minSpeed = 100});
		chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 500, {.minSpeed = 10, .earlyExitRange = 5}, false);
		chassis.moveToPoint(24, chassis.getPose().y, 9000, {.forwards = false,.maxSpeed = 60, .minSpeed = 10, .earlyExitRange = 3}, false);
		

	}
	else if(chosen_auto == 8)
	{
		//right_side 4

		//1st stack of 4
		chassis.setPose(45, 12, 270);
		intake_high(127);
		chassis.swingToPoint(23, 23, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, 23, 3000, {.minSpeed = 20, .earlyExitRange = 2});
		moveto_stack(23, 23);

		chassis.turnToPoint(36, 45, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		chassis.moveToPoint(36, 45, 3000, {.minSpeed = 20, .earlyExitRange = 8});
		chassis.turnToPoint(27, 46, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 20});
		chassis.moveToPoint(27, 46, 700, {.forwards = false, .minSpeed = 60});
		pros::Task::delay(200);
		hood.set_value(true);
		pros::Task::delay(1000);

		chassis.setPose(31, -48, chassis.getPose().theta);

		chassis.turnToHeading(10, 2000, {.minSpeed = 100,.earlyExitRange = 5}, false);
		chassis.turnToPoint(22, -46, 2000, {.forwards = false, .minSpeed = 3, .earlyExitRange = 5}, false);
		chassis.moveToPoint(22, -46, 300, {.forwards = false, .minSpeed = 100});
		chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 500, {.minSpeed = 10, .earlyExitRange = 5}, false);
		chassis.moveToPoint(26, chassis.getPose().y, 9000, {.forwards = false,.maxSpeed = 60, .minSpeed = 10, .earlyExitRange = 3}, false);
		
	}
	else if(chosen_auto == 9)
	{
		//skills_testing

		chassis.setPose(0, 0, 90);
		intake_high(127);
		ears.set_value(true);
		chassis.moveToPoint(2, 0, 700, {.maxSpeed = 30});
		for(int i = 0; i < 2; i++)
		{
			chassis.turnToHeading(100, 100);
			chassis.turnToHeading(80, 100);
		}
		chassis.moveToPoint(10, 0, 700, {.maxSpeed = 90});

		//leave park zone
		chassis.turnToHeading(90, 300, {}, false);
		chassis.moveToPoint(-72, chassis.getPose().y, 700, {.forwards = false}, false);
		pros::Task::delay(400);

		// reset position after park
		chassis.turnToHeading(90, 1000, {.minSpeed = 35, .earlyExitRange = 2}, false);
		simple_dist_reset();
		dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
		chassis.setPose(72-dist_to_center, 
							chassis.getPose().y, 
							chassis.getPose().theta);
		
		//move to stack 1					
		chassis.moveToPoint(30, chassis.getPose().y, 2000, {.forwards = false,.minSpeed = 10, .earlyExitRange = 0.5}, false);
		chassis.turnToPoint(30, 23, 2500, {.minSpeed = 10, .earlyExitRange = 3});
		chassis.moveToPose(30, 23, 0, 2000);
		chassis.turnToPoint(9, 11, 2000, {.maxSpeed = 90, .minSpeed = 1, .earlyExitRange = 2});
		chassis.moveToPose(9, 11, 225, 2000, {.lead = 0.5}, false);
		funnel.set_value(true);
		intake_low(127*0.5);
		pros::Task::delay(5000);
		chassis.moveToPoint(24, 24, 2000, {.forwards = false});

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
	
	intake_high(0);
	chassis.cancelAllMotions();
	//g_auton_started = true;
	g_op_control_started = true;
	hood.set_value(false);
	g_hood_state = false;

	pros::Task driver_systems_task([&]() {
	while (true)
	{
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intake_high(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			intake_low(127*0.5);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			intake_middle_skills(120);
			intake_index.move(-127*0.2);

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


	int leftY = 0;
	int rightX = 0;
    while (true) {
		// get left y and right x positions
		leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.curvature(leftY, rightX);

		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			chassis.setPose(31, -48, 90);
			ears.set_value(false);
			chassis.turnToHeading(10, 1000, {.minSpeed = 100,.earlyExitRange = 5}, false);
			chassis.turnToPoint(22, -46, 750, {.forwards = false, .minSpeed = 3, .earlyExitRange = 5}, false);
			chassis.moveToPoint(22, -46, 300, {.forwards = false, .minSpeed = 100});
			chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 500, {.minSpeed = 100, .earlyExitRange = 5}, false);
			chassis.moveToPoint(23, chassis.getPose().y, 400, {.forwards = false,.minSpeed = 100, .earlyExitRange = 3}, false);
		}
		
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
		{
			chassis.setPose(31, -48, 90);
			ears.set_value(false);
			chassis.turnToHeading(135, 1000, {.minSpeed = 100,.earlyExitRange = 10}, false);
			chassis.moveToPoint(38, -56, 750, {.minSpeed = 10, .earlyExitRange = 2}, false);
			
			chassis.turnToHeading(270, 700, {.minSpeed = 100, .earlyExitRange = 15}, false);
			chassis.moveToPoint(6, chassis.getPose().y, 1000, {.minSpeed = 100, .earlyExitRange = 3}, false);
		}
	}
		
	// delay to save resources
    pros::delay(25);
}
