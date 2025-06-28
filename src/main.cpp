#include "main.h"

// Get motors
pros::MotorGroup leftMotors({-0, -0, -0}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({0, 0, 0}, pros::MotorGearset::blue);

pros::Motor intake(0);
pros::Motor lift(0);
pros::Motor high(0);
pros::Motor low(0);

// Create drivetrain
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, // motors
                              16, // track width
                              lemlib::Omniwheel::NEW_325, // wheels
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
lemlib::ExpoDriveCurve throttleCurve(5, 10, 1.04); // deadband, min output, exponential curve gain
lemlib::ExpoDriveCurve steerCurve(5, 10, 1.04); // deadband, min output, exponential curve gain

// Finalize chassis                                             
lemlib::Chassis chassis(drivetrain, lateralController, angularController, odom, &throttleCurve, &steerCurve);

// Get pneumatics
pros::adi::Pneumatics wing(0, false);
pros::adi::Pneumatics puncher(0, false);

// Get controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Autonomous selector
int auton = 1; // can also modify this for test runs
int total = 1;
const char* descriptions[] = {"I am currently gooning to Shakey."};

// Brain display (using LGVL)
lv_obj_t *screen;
lv_obj_t *lx, *ly, *lt; // Coordinate labels
lv_obj_t *curr, *desc; // Autonomous selector labels
lv_obj_t *next, *prev; // Autonomous selector buttons
lv_obj_t *lnext, *lprev; // Autonomous selector button labels

// Autonomous selector callback functions
void update() { char a[30]; snprintf(a, sizeof(a), "Selected Auton: %d", auton); lv_label_set_text(curr, a); lv_label_set_text(desc, descriptions[auton - 1]); }
void forward(lv_event_t* e) { auton = auton % total + 1; update(); }
void back(lv_event_t* e) { auton = (auton + total - 2) % total + 1; update(); }

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
    lv_obj_align(lx, LV_ALIGN_TOP_LEFT, 20, 20);
    lv_obj_align(ly, LV_ALIGN_TOP_LEFT, 20, 50);
    lv_obj_align(lt, LV_ALIGN_TOP_LEFT, 20, 80);

    // Create autonomous selector labels
    curr = lv_label_create(screen);
    desc = lv_label_create(screen);
    lv_obj_align(curr, LV_ALIGN_TOP_RIGHT, -50, 50);
    lv_obj_align(desc, LV_ALIGN_TOP_RIGHT, -50, 80);
    update();
    
    // Create autonomous selector buttons
    next = lv_btn_create(screen);
    prev = lv_btn_create(screen);
    lv_obj_align(next, LV_ALIGN_TOP_RIGHT, -50, 120);
    lv_obj_align(prev, LV_ALIGN_TOP_RIGHT, -120, 120);
    lv_obj_set_style_bg_color(next, lv_color_hex(0x808080), 0);
    lv_obj_set_style_bg_color(prev, lv_color_hex(0x808080), 0);
    lv_obj_add_event_cb(next, forward, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(prev, back, LV_EVENT_CLICKED, nullptr);

    // Create autonomous selector labels
    lnext = lv_label_create(next);
    lprev = lv_label_create(prev);
    lv_obj_align(lnext, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align(lprev, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(lnext, ">");
    lv_label_set_text(lprev, "<");

    pros::Task screenTask([&]() {
        while (true) {
            lv_task_handler();

            // lv_label_set_text() cannot directly take a string variable, so we convert it into a char[] type (character array)
            char x[20], y[20], t[20];
            snprintf(x, sizeof(x), "X: %.3f", chassis.getPose().x);
            snprintf(y, sizeof(y), "Y: %.3f", chassis.getPose().y);
            snprintf(t, sizeof(t), "Theta: %.3f", chassis.getPose().theta);

            // Display coordinates of the robot on the screen
            lv_label_set_text(lx, x);
            lv_label_set_text(ly, y);
            lv_label_set_text(lt, t);
             
            // Delay to save resources
            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Clear screen
    lv_obj_del(next);
    lv_obj_del(prev);

    // Reset inertial sensor
    imu.set_heading(0);

    // Run autonomous
    switch (auton) {
        case 1: break;
    }
}

void opcontrol() {
    while (true) {
        // Tank Drive
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        // Delay to save resources
        pros::delay(20);
    }
}
