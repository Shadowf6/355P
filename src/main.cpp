#include "main.h"

// Get motors
pros::MotorGroup leftMotors({-0, -0, -0}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({0, 0, 0}, pros::MotorGearset::blue);

// Create drivetrain
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, // motors
                              16, // track width
                              lemlib::Omniwheel::NEW_275, // wheels
                              450, // rpm
                              2 // horizontal drift
);

// Get inertial sensor                              
pros::Imu imu(0);

// Get encoders
pros::Rotation horizontalEncoder(0);
pros::Rotation verticalEncoder(0);

// Create tracking wheels
lemlib::TrackingWheel horizontalTrackingWheel(&horizontalEncoder, lemlib::Omniwheel::NEW_275, 0); // encoder, wheels, offset
lemlib::TrackingWheel verticalTrackingWheel(&verticalEncoder, lemlib::Omniwheel::NEW_275, 0); // encoder, wheels, offset

// Finalize odometry
lemlib::OdomSensors odom(&verticalTrackingWheel, nullptr, &horizontalTrackingWheel, nullptr, &imu); // vert, vert, hori, hori

// PID
lemlib::ControllerSettings lateralController(10, // kP
                                             0, // kI
                                             3, // kD
                                             3, // anti windup
                                             1, 100, // small error range, timeout
                                             3, 500, // large error range, timeout
                                             20 // max acceleration
);
    
lemlib::ControllerSettings angularController(2, // kP
                                             0, // kI
                                             10, // kD
                                             3, // anti windup
                                             1, 100, // small error range, timeout
                                             3, 500, // large error range, timeout
                                             0 // max acceleration
);

// Create drive curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019); // deadband, min output, exponential curve gain
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019); // deadband, min output, exponential curve gain

// Finalize chassis                                             
lemlib::Chassis chassis(drivetrain, lateralController, angularController, odom, &throttleCurve, &steerCurve);

// Get motors
pros::Motor intake(0);
pros::Motor lift(0);
pros::Motor high(0);
pros::Motor low(0);

// Get pneumatics
pros::adi::Pneumatics wing(0, false);
pros::adi::Pneumatics puncher(0, false);

// Competition Specific
int goon = 1;
int total = 6;
bool comp = false;

// Brain display
lv_obj_t *screen;
lv_obj_t *lx, *ly, *lt; // Coordinate labels
lv_obj_t *curr; // Autonomous selector labels
lv_obj_t *lb, *rb; // Autonomous selector buttons


void initialize() {
    // Initialize robot
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.calibrate();

    // Initialize brain screen
    lv_init();
    screen = lv_scr_act();

    // Create coordinate labels
    lx = lv_label_create(screen);
    ly = lv_label_create(screen);
    lt = lv_label_create(screen);
    lv_obj_align(lx, LV_ALIGN_LEFT_MID, 0, 20);
    lv_obj_align(ly, LV_ALIGN_LEFT_MID, 0, 50);
    lv_obj_align(lt, LV_ALIGN_LEFT_MID, 0, 80);

    pros::Task screen([&]() {
        while (true) {
            // Prepare coordinates
            char x[8], y[8], t[8];
            snprintf(x, sizeof(x), "X: %.3f", chassis.getPose().x);
            snprintf(y, sizeof(x), "Y: %.3f", chassis.getPose().y);
            snprintf(t, sizeof(t), "Theta: %.3f", chassis.getPose().theta);

            // Display coordinates of the robot on the screen
            lv_label_set_text(lx, x);
            lv_label_set_text(ly, y);
            lv_label_set_text(lt, t);

            lv_task_handler();

            // Delay to save resources
            pros::delay(20);
        }
    });
}

void disabled() {}

void lclick(lv_event_t *e) {
    // Cycle to previous auton
    goon = (goon + total - 2) % total + 1; 

    char g[2];
    snprintf(g, sizeof(g), "%d", goon);
    lv_label_set_text(curr, g);
}

void rclick(lv_event_t *e) { 
    // Cycle to next auton
    goon = goon % total + 1;
    
    char g[2];
    snprintf(g, sizeof(g), "%d", goon);
    lv_label_set_text(curr, g);
}

void competition_initialize() {
    // Competition flag
    comp = true;

    // Reset odometry
    horizontalEncoder.reset_position();
    pros::delay(100);
    horizontalTrackingWheel.reset();
    pros::delay(100);
    verticalEncoder.reset_position();
    pros::delay(100);
    verticalTrackingWheel.reset();

    // Auton selector
    curr = lv_label_create(screen);
    lb = lv_btn_create(screen);
    rb = lv_btn_create(screen);
    lv_obj_align(curr, LV_ALIGN_TOP_RIGHT, 0, 50);
    lv_obj_align(lb, LV_ALIGN_CENTER, 100, 90);
    lv_obj_align(rb, LV_ALIGN_CENTER, 100, 130);
    lv_label_set_text(curr, "1");
    lv_obj_add_event_cb(lb, lclick, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(rb, rclick, LV_EVENT_CLICKED, NULL);
}

void autonomous() {
    // Clear screen
    if (comp) {
        lv_obj_del(curr);
        lv_obj_del(lb);
        lv_obj_del(rb);
    }

    // Reset inertial sensor
    imu.set_heading(0);

    // Run auton
    switch (goon) {
        case 1: break; // Red, top
        case 2: break; // Red, bottom
        case 3: break; // Blue, top
        case 4: break; // Blue, bottom
        case 5: break; // Red
        case 6: break; // Blue
    }
}

void opcontrol() {
    // Get controller
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    while (true) {
        // Move robot (tank drive)
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
                     controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        // Delay to save resources
        pros::delay(20);
    }
}
