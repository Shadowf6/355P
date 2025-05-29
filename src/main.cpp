#include "main.h"

// Get motors
pros::MotorGroup leftMotors({-10, -9, -8}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({4, 3, 2}, pros::MotorGearset::blue);

// Create drivetrain
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, // motors
                              16, // track width
                              lemlib::Omniwheel::NEW_275, // wheels
                              450, // rpm
                              2 // horizontal drift
);

// Get inertial sensor                              
pros::Imu imu(5);

// Get encoders
pros::Rotation horizontalEncoder(6);
pros::Rotation verticalEncoder(-16);

// Create tracking wheels
lemlib::TrackingWheel horizontalTrackingWheel(&horizontalEncoder, lemlib::Omniwheel::NEW_2, 0); // *encoder, wheels, offset
lemlib::TrackingWheel verticalTrackingWheel(&verticalEncoder, lemlib::Omniwheel::NEW_2, 0); // *encoder, wheels, offset

// Finalize odom
lemlib::OdomSensors odom(&verticalTrackingWheel, nullptr, &horizontalTrackingWheel, nullptr, &imu); // vert1, vert2, hori1, hori2

// PID
lemlib::ControllerSettings lateralController(10, // kP
                                             0, // kI
                                             3, // kD
                                             3, // anti windup
                                             1, // small error range
                                             100, // small error range timeout
                                             3, // large error range
                                             500, // large error range timeout
                                             20 // max acceleration
);
    
lemlib::ControllerSettings angularController(2, // kP
                                             0, // kI
                                             10, // kD
                                             3, // anti windup
                                             1, // small error range
                                             100, // small error range timeout
                                             3, // large error range
                                             500, // large error range timeout
                                             0 // max acceleration
);

// Finalize chassis                                             
lemlib::Chassis chassis(drivetrain, lateralController, angularController, odom);


void on_center_button() {}

void initialize() {
    // Initialize robot
    pros::lcd::initialize();
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.calibrate();

    pros::Task print([&]() {
        while (true) {
            // Print out coordinates of the robot on the screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);

            // Delay to save resources
            pros::delay(25);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Reset inertial sensor
    imu.set_heading(0);
}

void opcontrol() {
    // Get controller
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        // Move robot
        leftMotors.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        rightMotors.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        // Delay to save resources
        pros::delay(20);
    }
}
