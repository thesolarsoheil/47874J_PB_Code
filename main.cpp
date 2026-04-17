#include "main.h"
#include "g_var.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pid.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "intake_functions.h"
#include "distance_functions.h"
#include "movement_functions.h"
#include "helper_functions.h"

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

#include "pros/misc.h"
#include "pros/misc.hpp"




namespace fs = std::filesystem;

extern pros::Motor intake_bottom;
extern pros::Motor intake_top;
extern pros::Motor intake_index;
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern lemlib::Drivetrain drivetrain_6m;
extern pros::Imu imu;
extern pros::ADIDigitalOut ears;
extern pros::ADIDigitalOut scraper;
extern pros::ADIDigitalOut hood;
extern pros::ADIDigitalOut funnel;
extern pros::ADIDigitalOut intake_lift;
extern pros::ADIDigitalOut mid_descore;
extern pros::Rotation vertical_encoder;
extern pros::Distance distance_left;
extern pros::Distance distance_right;
extern pros::Distance distance_front;
extern pros::Optical color_sensor;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller_large;
extern lemlib::ControllerSettings angular_controller_small;
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;
extern lemlib::Chassis chassis;
extern pros::Controller controller;


void descore()
{
	chassis.setPose(31, 48, 90);

	chassis.moveToPoint(37, 48, 1000, {.minSpeed = 100, .earlyExitRange = 4});
	chassis.turnToHeading(135, 2000, {.minSpeed = 100, .earlyExitRange = 10});
	chassis.moveToPose(19, 53, 80, 2000, {.forwards = false, .lead = 0.5, .minSpeed = 100});

	chassis.moveToPoint(13, 54, 1200, {.forwards = false,.maxSpeed = 101, .minSpeed = 100}, false);
	chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	pros::Task::delay(50);
	chassis.turnToHeading(160, 2000, {.maxSpeed = 51, .minSpeed = 50});
	pros::Task::delay(300);
	scraper.set_value(true);
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
			distance_reading = color_sensor.get_proximity();            

			pros::screen::print(pros::E_TEXT_SMALL, 7, "dist color: %f", distance_reading); // dist sensor
			
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
	color_sensor.set_led_pwm(50);
	vertical_encoder.set_position(0); // set the vertical encoder to 0
	//intake_top.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	chosen_auto = 0;
	
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

	if(chosen_auto != 0)
	{
		if((color_sensor.get_hue() < 70 || color_sensor.get_hue() > 300))
			team_color = 'r';
		else
			team_color = 'b';
	}

	switch(chosen_auto) {
	
	//skills
	case 0:
	{
		
		move_skills_low();
 
		chassis.moveToPoint(22, 22, 500, {.forwards = false, .minSpeed = 10, .earlyExitRange = 2}, false);
		chassis.turnToHeading(180, 800, {.minSpeed = 10, .earlyExitRange = 2}, false);
		dist_to_center = distance_left.get()*0.0394+left_distance_from_center;
		chassis.setPose(72-dist_to_center, 
							chassis.getPose().y, 
							chassis.getPose().theta);
							
		funnel.set_value(false);
		intake_lift.set_value(false);

		chassis.moveToPose(22, -22, 180, 600, {.lead = .7, .minSpeed = 127, .earlyExitRange = 3});
		chassis.moveToPoint(22, -22, 4000, {.minSpeed = 30, .earlyExitRange = 3});
		intake_high(127);
		hood.set_value(true);

		moveto_stack(22, -22);

		//moveto_stack(22, -22);
		chassis.waitUntilDone();
		scraper.set_value(true);
		hood.set_value(false);
		
		chassis.swingToPoint(48, -45, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 70, .earlyExitRange = 10}, false);
		//pros::Task::delay(999999);

		moveto_point(48, -45, 2000, {.minSpeed = 40, .earlyExitRange = 4}, false);
		intake_high(127);
		ears.set_value(true);
		turnto_heading(90, 1000, {.minSpeed = 5, .earlyExitRange = 5}, false);
		simple_dist_reset();
		moveto_matchload(4, 6);
		
		//move down the 1st alley

		chassis.moveToPoint(36, -57, 3000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 4});
		intake_high(0);
		intake_index.move(127);
		hood.set_value(true);
		pros::Task::delay(400);
		hood.set_value(false);
		intake_high(127);
		swingto_point(-23, -61, lemlib::DriveSide::LEFT, 1000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 10}, false);

		scraper.set_value(false);
		chassis.moveToPoint(-23, -61, 3000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 4});
	
		intake_high(0);
		intake_index.move(-100);
		pros::Task::delay(500);
		intake_high(127);
		chassis.waitUntilDone();

		//move to 2nd matchloader
		moveto_point(-44, -48, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 4});
		scraper.set_value(true);
		intake_high(127);
		turnto_heading(270, 2000, {.minSpeed = 1, .earlyExitRange = 0.5}, false);\

		simple_dist_reset();
		dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
		chassis.setPose(-72+dist_to_center, 
							chassis.getPose().y, 
							chassis.getPose().theta);

		moveto_matchload(3, 6);
		//simple_dist_reset();


		//move to 1st goal for the 1st time
		
		chassis.moveToPose(-27, -48, 270, 3000, {.forwards = false, .minSpeed = 100});
		
		//intake_high(0);
		//intake_index.move(-127);
		//pros::Task::delay(500);
		//intake_high(127);
		chassis.waitUntil(16);
		chassis.cancelMotion();
		
		chassis.moveToPoint(0, -48, 800, {.forwards = false, .maxSpeed = 40});
		intake_high(0);
		//intake_index.move(30);
		hood.set_value(true);
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

		pros::Task::delay(400);
		intake_high(127);
		intake_lift.set_value(true);

		pros::Task::delay(200);
		chassis.cancelMotion();
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

		//chassis.moveToPoint(0, 48, 1900, {.forwards = false, .maxSpeed = 15});
		pros::Task::delay(100);
		intake_lift.set_value(false);
		pros::Task::delay(2300);

		intake_high(100);
		intake_index.move(127);
		
		pros::Task::delay(100);
		mid_descore.set_value(true);
		intake_high(-20);
		intake_index.move(127);
		pros::Task::delay(400);
		intake_index.move(90);
		pros::Task::delay(200);
		mid_descore.set_value(false);

		//reset after scoring
		chassis.turnToHeading(270, 300, {.minSpeed = 50, .earlyExitRange = 1}, false);
		simple_dist_reset();
		dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
		chassis.setPose(-72+dist_to_center, 
							chassis.getPose().y, 
							chassis.getPose().theta);
		
		
		chassis.setPose(-31, -48, 270);
		ears.set_value(true);
		scraper.set_value(false);
		
		//move to in front of blue park zone

		moveto_point(-37, -48, 300, {.minSpeed = 20, .earlyExitRange = 2}, false);
		intake_high(127);

		turnto_point(-26, -26, 800, {.minSpeed = 1, .earlyExitRange = 2}, false);
		hood.set_value(false);
		chassis.moveToPose(-30, -24, 0, 400, {.lead = 0.65, .minSpeed = 60, .earlyExitRange = 9});
		chassis.moveToPose(-30, -24, 0, 1200, {.maxSpeed = 50, .minSpeed = 1, .earlyExitRange = 2});

		chassis.waitUntilDone();
		intake_high(0);


		swingto_heading(90, lemlib::DriveSide::LEFT, 200);

		turnto_point(-40, -2, 800, {.minSpeed = 5, .earlyExitRange = 3}, false);
		scraper.set_value(true);
		moveto_point(-40, -2, 2500, {.minSpeed = 15, .earlyExitRange = 2});
		scraper.set_value(false);

		//pros::Task::delay(9999999);
		
		//enter park zone and clear balls
		turnto_heading(270, 600, {.minSpeed = 1, .earlyExitRange = 3}, false);
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
		intake_high(127);
		moveto_point(-72, chassis.getPose().y, 100, {.maxSpeed = 70});

		moveto_point(-72, chassis.getPose().y, 100, {.maxSpeed = 30});
		chassis.moveToPoint(-72, chassis.getPose().y, 1200, {.maxSpeed = 70});

		pros::Task::delay(400);
		scraper.set_value(true);
		pros::Task::delay(400);
		scraper.set_value(false);

		chassis.moveToPoint(72, chassis.getPose().y, 300, {.forwards = false, .maxSpeed = 30}, false);
		turnto_heading(270, 300, {}, false);

		moveto_point(-72, chassis.getPose().y, 600, {}, false);
		chassis.moveToPoint(72, chassis.getPose().y, 100, {.forwards = false, .maxSpeed = 90}, false);
		chassis.setPose(0, 0, chassis.getPose().theta);
		moveto_point(-72, chassis.getPose().y, 300, { .maxSpeed = 90}, false);
		chassis.moveToPoint(72, chassis.getPose().y, 100, {.forwards = false, .maxSpeed = 90}, false);

		for(int i = 0; i < 1; i++){
			turnto_heading(255, 200);
			turnto_heading(285, 200);
		}

		
		//leave park zone
		turnto_heading(270, 200, {}, false);
		moveto_point(-72, chassis.getPose().y, 300, {.minSpeed = 127});
		if(check_equal(chassis.getPose().theta, 270, 3))
		{
			chassis.setPose(chassis.getPose().x, chassis.getPose().y, 270);
		}

		moveto_point(20, chassis.getPose().y, 200, {.forwards = false, .maxSpeed = 30}, false);
		moveto_point(20, chassis.getPose().y, 800, {.forwards = false}, false);
		pros::Task::delay(200);


		// reset position after park
		turnto_heading(270, 500, {.minSpeed = 15, .earlyExitRange = 2}, false);
		chassis.setPose(0, 1, chassis.getPose().theta);

		weighted_dist_reset();
		dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
		
		chassis.setPose(-72+dist_to_center, 
							chassis.getPose().y, 
							chassis.getPose().theta);

		//move to middle goal
		scraper.set_value(true);
		turnto_point(-24, 24, 1000, {.minSpeed = 1, .earlyExitRange = 4}, false);
		scraper.set_value(false);
		intake_high(127);
		chassis.moveToPoint(-24, 24, 1000, {.minSpeed = 20, .earlyExitRange = 5});
		moveto_stack(-24, 24);
		scraper.set_value(true);
		turnto_point(-10, 10, 1000, {.forwards = false, .minSpeed = 1, .earlyExitRange = 3});
		chassis.moveToPoint(-10, 10, 400, {.forwards = false, .maxSpeed = 100});
		chassis.moveToPoint(-10, 10, 800, {.forwards = false, .maxSpeed = 30});
		
		//score on middle goal
		
		//chassis.setPose(-14, 14, 315);
		pros::Task::delay(150);
		mid_descore.set_value(true);
		intake_high(-127); 
		pros::Task::delay(100);
		intake_middle_skills(127);
		intake_bottom.move(0);
		intake_top.move(0);
		pros::Task::delay(50);
		intake_middle_skills(160);
		chassis.turnToHeading(315, 200, {.minSpeed = 100, .earlyExitRange = 1}, false);

		chassis.moveToPoint(-10, 10, 600, {.forwards = false, .maxSpeed = 50});
		middle_color_wait(600);
		chassis.cancelMotion();
		
		intake_middle_skills(90);


		ears.set_value(true);
		middle_color_wait(800);
		
		intake_middle_skills(90);

		middle_color_wait(1600);

	
		ears.set_value(false);
		intake_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		intake_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		intake_high(-60);
		intake_index.move(-100*0.25);
		pros::Task::delay(200);
		intake_high(-30);
		intake_index.move(-100*0.25);
		pros::Task::delay(250);

		intake_high(0);

		//move to matchloader
		chassis.moveToPoint(-46, 48, 1500, {.minSpeed = 10, .earlyExitRange = 1});
		intake_top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		intake_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		//intake_middle_skills(0);
		//swingto_heading(180, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 20, .earlyExitRange = 3}, false);

		intake_high(-50);
		intake_index.move(127);
		pros::Task::delay(200);
		intake_high(0);
		intake_index.move(127);

		ears.set_value(true);
		mid_descore.set_value(false);

		scraper.set_value(true);
		swingto_heading(270, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 2, .earlyExitRange = 2}, false);
		simple_dist_reset();
		dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
		
		chassis.setPose(-72+dist_to_center, 
							chassis.getPose().y, 
							chassis.getPose().theta);

		intake_high(127);
		moveto_matchload(2, 6);



		//move down the 2nd alley
		chassis.moveToPoint(-36, 60, 3000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 4});
		//intake_high(0);
		//intake_index.move(127);
		//hood.set_value(true);
		//pros::Task::delay(400);
		//hood.set_value(false);
		//intake_high(127);
		swingto_heading(270, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 80, .earlyExitRange = 5}, false);
		dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
		
		chassis.setPose(-72+dist_to_center, 
							chassis.getPose().y, 
							chassis.getPose().theta);
		//simple_dist_reset();

		scraper.set_value(false);
		chassis.moveToPoint(23, 62, 3000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 4});
		intake_high(0);
		intake_index.move(-100);
		pros::Task::delay(500);
		intake_high(127);
		chassis.waitUntilDone();
		//move to 4th matchloader
		//turnto_point(-44, 48, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 5}, false);
		moveto_point(42, 48, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 4});
		scraper.set_value(true);
		intake_high(127);
		turnto_heading(90, 1200, {.minSpeed = 2, .earlyExitRange = 2}, false);
		simple_dist_reset();
		moveto_matchload(1, 6);

		//move to 2nd goal for the 1st time

		chassis.moveToPose(-27, 48, 90, 3000, {.forwards = false, .minSpeed = 100});
		
		chassis.waitUntil(16);
		chassis.cancelMotion();
		
		chassis.moveToPoint(0, 48, 800, {.forwards = false, .maxSpeed = 40});
		intake_high(0);
		//intake_index.move(30);
		hood.set_value(true);
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

		pros::Task::delay(400);
		intake_high(127);
		intake_lift.set_value(true);

		pros::Task::delay(200);
		chassis.cancelMotion();
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

		pros::Task::delay(300);
		intake_lift.set_value(false);
		pros::Task::delay(2300);

		intake_high(100);
		intake_index.move(127);
		pros::Task::delay(400);
		mid_descore.set_value(true);
		intake_high(-20);
		intake_index.move(127);
		pros::Task::delay(200);
		mid_descore.set_value(false);
		intake_index.move(90);
		pros::Task::delay(600);


		//reset after scoring
		chassis.turnToHeading(90, 300, {.minSpeed = 50, .earlyExitRange = 1}, false);
		chassis.setPose(31, 48, chassis.getPose().theta);
		
		//park
		scraper.set_value(false); 
		chassis.moveToPose(65, 15, 180, 2000, {.lead = 0.4, .minSpeed = 100});
		pros::Task::delay(600);
		hood.set_value(false);

		//scraper.set_value(true);
		moveto_point(66, 0, 1000);

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
		
		moveto_point(48, 43, 3000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 6}, false);
		scraper.set_value(true);
		turnto_heading(90, 1000, {.minSpeed = 3, .earlyExitRange = 2}, false);
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

		moveto_point(33, 48, 100);
		swingto_heading(180, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 50, .earlyExitRange = 10});
		intake_high(127);
		
		turnto_point(23, 23, 2000, {.minSpeed = 10, .earlyExitRange = 5});
		hood.set_value(false);
		chassis.moveToPoint(23, 23, 1000, {.minSpeed = 100, .earlyExitRange = 5});
		moveto_stack(23, 23);


		chassis.moveToPoint(22, -18, 2000, {.minSpeed = 90, .earlyExitRange = 4});
		moveto_stack(22, -23);
		scraper.set_value(true);
		moveto_point(38, -46, 2000, {.minSpeed = 10, .earlyExitRange = 5}, false);
		turnto_point(24, -46, 2000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 5});
		moveto_point(24, -46, 300, {.forwards = false, .minSpeed = 100});
		hood.set_value(true);
		moveto_point(0, -46, 200, {.forwards = false}, false);
		pros::Task::delay(1000);
		chassis.setPose(31, -48, chassis.getPose().theta);

		moveto_point(52, -48, 2000, {.minSpeed = 60, .earlyExitRange = 4});
		hood.set_value(false);
		moveto_matchload(4, 3);

		moveto_point(50, -48, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3}, false);
		swingto_point(10, -10, lemlib::DriveSide::LEFT, 2000, {.forwards = false, .minSpeed = 30, 
																.earlyExitRange = 10});

		//chassis.moveToPose(0, -0, 135, 400, {.forwards = false, .lead = 0.7});
		moveto_point(10, -10, 1000, {.forwards = false});
		chassis.moveToPoint(10, -10, 400, {.forwards = false, .maxSpeed = 20});
		scraper.set_value(false);
		pros::Task::delay(0);
		//score on middle goal
		mid_descore.set_value(true);
		intake_middle_skills(127);
		pros::Task::delay(2000);
		
		
		turnto_point(38, -38, 200, {.earlyExitRange = 3});
		moveto_point(38, -38, 2000, {.minSpeed = 60, .earlyExitRange = 2});
		turnto_point(0, -38, 1500, {.forwards = false, .minSpeed = 1, .earlyExitRange = 15});
		moveto_point(0, -38, 500, {.forwards = false});
		moveto_point(12, -39, 1000, {.forwards = false});

	}
	break;
	//left_side middle + long
	case 2:
	{
		//matchloader

		dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
		chassis.setPose(48, -72 + dist_to_center, 180);
		intake_high(127);
		ears.set_value(true);
		scraper.set_value(true);

		moveto_point(48, -40, 2000, {.minSpeed = 60, .earlyExitRange = 5});
		swingto_point(72, -48, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 40, .earlyExitRange = 20});
		moveto_matchload(4, 3);

		//move to 1st goal for the 1st time
		chassis.moveToPose(27, -48, 270, 3000, {.forwards = false, .minSpeed = 100});

		chassis.waitUntil(16);
		chassis.cancelMotion();
		
		chassis.moveToPoint(0, -48, 400, {.forwards = false, .maxSpeed = 40});
		intake_high(0);
		//intake_index.move(30);
		hood.set_value(true);
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

		pros::Task::delay(200);
		intake_high(127);
		pros::Task::delay(200);
		chassis.cancelMotion();
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);


		long_color_wait(500, 'r');
		pros::Task::delay(25);
		hood.set_value(false);
		pros::Task::delay(50);
		hood.set_value(true);


		//move to 1st stack of 4

		chassis.setPose(31, -48, chassis.getPose().theta);
		scraper.set_value(false);
		moveto_point(33, -48, 100);
		swingto_heading(0, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 50, .earlyExitRange = 10});
		intake_high(127);
		
		turnto_point(21, -21, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		hood.set_value(false);
		chassis.moveToPoint(21, -21, 1000, {.minSpeed = 10, .earlyExitRange = 5});
		moveto_stack(21, -21);
		turnto_point(12, -12, 4000, {.minSpeed = 20, .earlyExitRange = 5}, false);
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

		mid_descore.set_value(true);
		moveto_point(12, -12, 1000, {.forwards = false, .maxSpeed = 80});
		moveto_point(0, 0, 500, {.forwards = false, .maxSpeed = 80});

		chassis.turnToHeading(135, 600, {.minSpeed = 50, .earlyExitRange = 5});
		intake_middle(80);
		pros::Task::delay(300);
		moveto_point(24, -24, 300, {.maxSpeed = 40});

		mid_descore.set_value(false);
		while(true)
		{
		moveto_point(8, -8, 400, {.forwards = false});
		moveto_point(24, -24, 400, {.maxSpeed = 20});
			
		}


	}
	break;
	//left_side 9
	case 3:
	{
		chassis.setPose(45, -12, 270); 
		intake_high(127);
		ears.set_value(true);
		chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
		chassis.moveToPose(10, -43, 180, 2500, {.lead = 0.25, .minSpeed = 120});
		pros::Task::delay(400);
		scraper.set_value(true);
		pros::Task::delay(150);
		scraper.set_value(false);

		chassis.waitUntilDone();

		scraper.set_value(true);
		turnto_heading(180, 300, {.minSpeed = 5, .earlyExitRange = 2}, false);
		chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
		moveto_point(24, -24, 500, {.forwards = false}, false);
		moveto_point(48, -48, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3}, false);
		turnto_heading(90, 1300, {.minSpeed = 1, .earlyExitRange = 2}, false);
		simple_dist_reset();
		moveto_matchload(4, 3);

		chassis.moveToPose(27, -48, 90, 3000, {.forwards = false});
			
		chassis.waitUntil(20);
		hood.set_value(true);
		chassis.cancelMotion();
		chassis.moveToPoint(0, -48, 500, {.forwards = false, .maxSpeed = 50}, false);
		pros::Task::delay(1000);
		scraper.set_value(false);
		ears.set_value(false);
		descore();
		
	break;
	}
	//left_side 7
	case 4:
	{		
		//1st stack of 4
		chassis.setPose(45, -12, 270);
		intake_high(127);
		chassis.moveToPoint(28, -18, 3000, {.minSpeed = 100, .earlyExitRange = 4}, false);
		scraper.set_value(true);
		//move to goal and score

		swingto_point(52, -46, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 100, .earlyExitRange = 10}, false);
		chassis.waitUntilDone();

		//move to matchloader

		moveto_point(52, -46, 2000, {.minSpeed = 100, .earlyExitRange = 10});
		swingto_heading(90, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 50, .earlyExitRange = 3}, false);

		simple_dist_reset();
		moveto_matchload(4, 3);

		//move to 1st goal for the 1st time
		
		chassis.moveToPose(-27, -48, 270, 3000, {.forwards = false, .minSpeed = 100});
		chassis.waitUntil(16);
		chassis.cancelMotion();
		
		chassis.moveToPoint(0, -48, 800, {.forwards = false, .maxSpeed = 40});
		intake_high(0);
		hood.set_value(true);
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

		pros::Task::delay(100);
		intake_high(127);
		pros::Task::delay(200);
		chassis.cancelMotion();
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

		pros::Task::delay(1200);
		ears.set_value(false);

		descore();

	}
	break;
	//left_side 4
	case 5:
	{
		//1st stack of 4
		chassis.setPose(45, -12, 270);
		intake_high(127);
		chassis.moveToPoint(28, -18, 3000, {.minSpeed = 100, .earlyExitRange = 4}, false);
		scraper.set_value(true);
		//move to goal and score
		//chassis.turnToPoint(36, -46, 2000, {.minSpeed = 80, .earlyExitRange = 10});
		swingto_point(36, -44, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 100, .earlyExitRange = 10}, false);
		chassis.waitUntilDone();
		scraper.set_value(false);

		chassis.moveToPoint(36, -44, 3000, {.minSpeed = 100, .earlyExitRange = 8});
		chassis.turnToPoint(27, -44, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 20});
		chassis.moveToPoint(27, -44, 700, {.forwards = false});
		pros::Task::delay(200);
		hood.set_value(true);
		pros::Task::delay(1000);

		descore();

	}
	break;
	//right_side low + long
	case 6:
	{
		
		//matchloader

		dist_to_center = distance_front.get()*0.0394+front_distance_from_center;
		chassis.setPose(48, 72 - dist_to_center, 0);
		intake_high(127);
		ears.set_value(true);
		scraper.set_value(true);

		moveto_point(48, 40, 2000, {.minSpeed = 60, .earlyExitRange = 5});
		swingto_point(72, 48, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 40, .earlyExitRange = 20});
		moveto_matchload(1, 3);

		//move to 1st goal for the 1st time
		chassis.moveToPose(27, 48, 270, 3000, {.forwards = false, .minSpeed = 100});

		chassis.waitUntil(16);
		chassis.cancelMotion();
		
		chassis.moveToPoint(0, 48, 400, {.forwards = false, .maxSpeed = 40});
		//intake_index.move(30);
		hood.set_value(true);
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

		pros::Task::delay(200);
		intake_high(127);
		pros::Task::delay(200);
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);


		pros::Task::delay(500);

		//move to 1st stack of 4

		chassis.setPose(31, 48, chassis.getPose().theta);
		scraper.set_value(false);
		moveto_point(33, 48, 100);
		swingto_heading(180, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 50, .earlyExitRange = 10});
		intake_high(127);
		
		turnto_point(21, 21, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		hood.set_value(false);
		chassis.moveToPoint(21, 21, 1000, {.minSpeed = 10, .earlyExitRange = 5}, false);

		turnto_point(8, 12, 4000, {.minSpeed = 20, .earlyExitRange = 5}, false);
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

		chassis.moveToPoint(8, 12, 1000, {.maxSpeed = 80});
		pros::Task::delay(700);
		intake_lift.set_value(true);
		funnel.set_value(true);
		intake_low(100);
		intake_index.move(-80);
		pros::Task::delay(900);
		funnel.set_value(false);
		intake_lift.set_value(false);

		turnto_heading(180, 500, {}, false);
		//descore

		while(distance_right.get()* 0.0394 > 25)
		{
			pros::Task::delay(10);
		}

		ears.set_value(true);
		moveto_point(33, 35, 2000, {.forwards = false});
		chassis.turnToHeading(280, 500,{} , false);
		ears.set_value(false);
		funnel.set_value(false);
		intake_lift.set_value(false);

		chassis.moveToPoint(6, chassis.getPose().y+1, 1500);
		chassis.turnToHeading(250, 5000, {.maxSpeed = 60});
		chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);


	}
	break;
	//right_side 9
	case 7:
	{
		chassis.setPose(45, 12, 270); 
		intake_high(127);
		ears.set_value(true);
		chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
		chassis.moveToPose(10, 43, 0, 2500, {.lead = 0.25, .minSpeed = 120});
		pros::Task::delay(400);
		scraper.set_value(true);
		pros::Task::delay(150);
		scraper.set_value(false);

		chassis.waitUntilDone();

		scraper.set_value(true);
		turnto_heading(0, 300, {.minSpeed = 5, .earlyExitRange = 2}, false);
		chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
		moveto_point(24, 24, 500, {.forwards = false}, false);
		moveto_point(48, 48, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3}, false);
		turnto_heading(90, 1300, {.minSpeed = 1, .earlyExitRange = 2}, false);
		simple_dist_reset();
		moveto_matchload(1, 3);

		chassis.moveToPose(27, 48, 270, 3000, {.forwards = false});
			
		chassis.waitUntil(18);
		hood.set_value(true);
		chassis.cancelMotion();
		chassis.moveToPoint(0, 48, 500, {.forwards = false, .maxSpeed = 30}, false);
		pros::Task::delay(1400);
		scraper.set_value(false);
		ears.set_value(false);
		descore();
	}
	break;
	//right_side 6
	case 8:
	{	

		chassis.setPose(45, 12, 270); 
		intake_high(127);
		ears.set_value(true);
		chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
		chassis.moveToPose(11.25, 42, 0, 2500, {.lead = 0.25, .minSpeed = 120});
		pros::Task::delay(500);
		scraper.set_value(true);
		pros::Task::delay(150);
		scraper.set_value(false);

		chassis.waitUntilDone();

		scraper.set_value(true);
		turnto_heading(0, 300, {.minSpeed = 5, .earlyExitRange = 2}, false);
		chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
		//swingto_point(24, -40, lemlib::DriveSide::LEFT, 300, {.forwards = false, .minSpeed = 15, .earlyExitRange = 10});
		//turnto_point(24, -40, 800, {.forwards = false, .minSpeed = 15, .earlyExitRange = 10}, false);
		//moveto_point(24, -40, 500, {.forwards = false}, false);
		chassis.moveToPose(39, 54, 180, 1500, {.forwards = false, .lead = 0.8, .minSpeed = 110});
		while(chassis.getPose().theta > 210 && chassis.getPose().theta < 150){
			pros::Task::delay(10);
		}

		moveto_point(39, 52, 500, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2}, false);
		//turnto_heading(90, 900, {.minSpeed = 1, .earlyExitRange = 5}, false);
		chassis.turnToPoint(24, 50, 2000, {.forwards = false, .minSpeed = 1, .earlyExitRange = 5}, false);
		chassis.moveToPoint(0, 50, 500, {.forwards = false});
		pros::Task::delay(100);
		hood.set_value(true);
		pros::Task::delay(1200);
		scraper.set_value(false);
		ears.set_value(false);
		descore();



	}
	break;
	//right_side 4
	case 9:
	{
		
		//1st stack of 4
		chassis.setPose(45, 12, 270);
		intake_high(127);
		chassis.swingToPoint(23, 23, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 40, .earlyExitRange = 10});
		chassis.moveToPoint(23, 23, 3000, {.minSpeed = 40, .earlyExitRange = 3});
		moveto_stack(23, 23);

		chassis.turnToPoint(36, 45, 2000, {.minSpeed = 10, .earlyExitRange = 10});
		chassis.moveToPoint(36, 45, 3000, {.minSpeed = 20, .earlyExitRange = 8});
		chassis.turnToPoint(27, 45, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 20});
		chassis.moveToPoint(27, 45, 700, {.forwards = false, .minSpeed = 60});
		pros::Task::delay(200);
		hood.set_value(true);
		pros::Task::delay(900);

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

	intake_high(0);
	chassis.cancelAllMotions();
	//g_auton_started = true;
	g_op_control_started = true;
	hood.set_value(false);
	g_hood_state = false;
	scraper.set_value(false);
	g_scraper_state = false;
	int timer = 0;

	chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	//intake_index.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	pros::Task driver_systems_task([&]() {
	while (true)
	{
		//intake high scoring
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intake_high(127);
		}
		
		//intake low scoring
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			if(g_auton_started)
			{
				intake_low(127);
			}
			else
			{
				intake_low(127*0.50);
				intake_index.move(127*-0.2);

			}
		}
		
		//intake middle scoring
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{

			mid_descore.set_value(true);
			g_mid_descore_state = true;
			if(!g_auton_started)
			{
				intake_middle_skills(120);
			}
			else
			{
				intake_middle(100);
			}

		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
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
		
		//no intake
		else
		{

			if(g_mid_descore_state)
			{
				timer+=25;
				if(timer >= 1000)
				{
					timer = 0;
					g_mid_descore_state = false;
					mid_descore.set_value(false);
				}
			}
			intake_high(0);
			if(intake_index.get_efficiency() > 5 || g_hood_state)
			{
				intake_index.move(40);
			}

			//intake_index.move(8);
		}

		//funnel toggle
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
		{
			if(g_funnel_state)
			{
				funnel.set_value(false);
				intake_lift.set_value(false);

				g_funnel_state = false;
			}
			else
			{
				funnel.set_value(true);
				intake_lift.set_value(true);

				g_funnel_state = true;
			}
		}
		
		//ears hold 
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
		{
			ears.set_value(false);
			g_ears_state = false;
		}
		else
		{
			ears.set_value(true);
			g_ears_state = true;
		}
		
		//hood hold
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			hood.set_value(true);
		}
		else
		{
			hood.set_value(false);
		}

		//scraper toggle
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
			chassis.cancelAllMotions();
			chassis.setPose(31, 48, 90);

			chassis.moveToPoint(37, 48, 1000, {.minSpeed = 100, .earlyExitRange = 4});
			macro_cancel();
			chassis.turnToHeading(135, 2000, {.minSpeed = 100, .earlyExitRange = 10});
			macro_cancel();
			chassis.moveToPose(19, 53, 80, 2000, {.forwards = false, .lead = 0.5, .minSpeed = 100});
			macro_cancel();

		}
	}
	
		
	// delay to save resources
    pros::delay(25);
}
