#include "main.h"
#include "fmt/core.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

using pros::E_CONTROLLER_DIGITAL_L1;

static bool toggle{false};
#include "robodash/api.h"

bool redAlliance = true;
static bool toggle2{false};
bool colorSort = false;


rd::Console console;
// controller
pros::Motor intakemotors = pros::Motor(10, pros::MotorGearset::green);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Optical sortOpt(5);
pros::adi::Pneumatics doinker(3,LOW);
// motor groups

pros::MotorGroup
    leftMotors({-1, 2, -3},
               pros::MotorGearset::blue); // left motor group - ports 3
// (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors(
    {12, -13, -14},
    pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
//pros::MotorGroup intakemotorsa({10, 19}, pros::MotorGearset::blue);
// Inertial Sensor on port 10
pros::Imu imu(15);
pros::adi::Pneumatics Clamp(1, HIGH);
pros::adi::Pneumatics Clamp2(2, HIGH);

// drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,                // left motor group
    &rightMotors,               // right motor group
    13.5,                       // 10 inch track width
    lemlib::Omniwheel::NEW_325, // using new 4" omnis
    360,                        // drivetrain rpm is 360
    2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings
    linearController(10,  // proportional gain (kP)
                     0,   // integral gain (kI)
                     3,   // derivative gain (kD)
                     3,   // anti windup
                     1,   // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     20   // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(2,   // proportional gain (kP)
                      0,   // integral gain (kI)
                      10,  // derivative gain (kD)
                      3,   // anti windup
                      1,   // small error range, in degrees
                      100, // small error range timeout, in milliseconds
                      3,   // large error range, in degrees
                      500, // large error range timeout, in milliseconds
                      0    // maximum acceleration (slew)
    );

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to
                            // nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to
                            // nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttleCurve(47,   // joystick deadband out of 127
                  39.4, // minimum output where drivetrain will move out of 127
                  1.028 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steerCurve(37,   // joystick deadband out of 127
               39.4, // minimum output where drivetrain will move out of 127
               1.028 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void redSoloWP() { 
	redAlliance = true;
}
void blueSoloWP() {
  chassis.setPose(
      {10.0, 10.0,
       45.0}); // Example: Set position (10, 10) and heading 45 degrees
}
void skills() {
  chassis.setPose(
      {10.0, 10.0,
       45.0}); // Example: Set position (10, 10) and heading 45 degrees
}
rd::Selector selector({
    {"Red Solo Winpoint", &redSoloWP},
    {"Blue Solo Winpoint", &blueSoloWP},
    {"Skills", &skills},
});
void clamp() {
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
    if (!toggle) {
      Clamp.set_value(false);
      Clamp2.set_value(false);
      toggle = !toggle;
    } else {
      Clamp.set_value(true);
      Clamp2.set_value(true);
      toggle = !toggle;
    }
  }
}
void doinkerMech() {
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
		if (!toggle2) {
			doinker.set_value(true);
			toggle2 = !toggle2;

		} else {
			doinker.set_value(false);	
			toggle = !toggle;
		}
	}
}

void IntakeGo() {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    intakemotors.move_velocity(600);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    intakemotors.move_velocity(-600);
  } else {
    intakemotors.move_velocity(0);
  }
}
void intakeReverse() {
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		intakemotors.move_velocity(-600);
	} else {
		intakemotors.move_velocity(0);
	}
}


void initialize() {
    console.println("Initializing robot...");
    chassis.calibrate();

    // Create the intake task
    pros::Task driverIntake([]() ->  void {
    while (true) {  // Add continuous loop
    
         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
          
            if (!colorSort) {
                intakemotors.move_velocity(-200);  // Changed to negative for intake
                if (redAlliance) {
                    if (sortOpt.get_hue() > 200 && sortOpt.get_hue() < 260) {
                        // Blue ring detected when on red alliance
                        
                        colorSort = true;
                    }
                } else {
                    if (sortOpt.get_hue() > 1 && sortOpt.get_hue() < 30) {
                        // Red ring detected when on blue alliance
                       
                        colorSort = true;
                    }
                }
            } else {
                        intakemotors.brake();
                        pros::delay(50);
                        intakemotors.move_velocity(-200);  // Resume intake
                        pros::delay(300);
						colorSort = false;
			}
        } 
        pros::delay(20);  // delay
    }
});

    // Screen update task
    pros::Task screenTask([]() {
        while (true) {
            console.clear();
            console.printf("X: %f\n", chassis.getPose().x);
            console.printf("Y: %f\n", chassis.getPose().y);
            console.printf("Theta: %f\n", chassis.getPose().theta);
            console.printf("Hue: %f\n", sortOpt.get_hue());
            pros::delay(50);
        }
    });
}





  // the default rate is 50. however, if you need to change the rate, you
  // can do the following.
  // lemlib::bufferedStdout().setRate(...);
  // If you use bluetooth or a wired connection, you will want to have a rate of
  // 10ms

  // for more information on how the formatting for the loggers
  // works, refer to the fmtlib docs

  // thread to for brain screen and position logging


/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() { selector.focus(); }

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the
 * features LemLib has to offer
 */
void autonomous() {
  console.println("Running auton...");
  selector.run_auton();
}

/**
 * Runs in driver control
 */
void opcontrol() {

  // controller
  // loop to continuously update motors
  
    

    // get joystick positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    // move the chassis with curvature drive
    chassis.arcade(rightX, leftY, false, 0.5);
    
    clamp();
	doinkerMech();
  
  
		
	
    // delay to save resources
    pros::delay(10);
  }
}
