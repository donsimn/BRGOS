#include "main.h"

// ======== Config ========

// Motor ports
constexpr int LEFT_MOTOR_PORT = 1;
constexpr int RIGHT_MOTOR_PORT = 2;
constexpr int SHOULDER_MOTOR1_PORT = 3;
constexpr int SHOULDER_MOTOR2_PORT = 4;
constexpr int ELBOW_MOTOR_PORT = 5;

// Piston ports
constexpr char INNER_PISTON_PORT = 'A';
constexpr char OUTER_PISTON_PORT = 'B';

// Movement
constexpr int REGULAR_DRIVETRAIN_SPEED = 70; // Drivetrain speed for regular movement in percent
constexpr int SLOW_DRIVETRAIN_SPEED = 30; // Drivetrain speed for slow movement in percent

constexpr int REGULAR_TURN_SPEED = 70; // Drivetrain turn speed for regular movement in percent
constexpr int SLOW_TURN_SPEED = 30; // Drivetrain turn speed for slow movement in percent


// Arm 
constexpr int SHOULDER_SPEED = 65; // Shoulder motor speed in percent
constexpr int ELBOW_SPEED = 100; // Elbow motor speed in percent

constexpr int MIN_SHOULDER_POSITION = 0; // Minimum shoulder position in degrees
constexpr int MAX_SHOULDER_POSITION = 100; // Maximum shoulder position in degrees

constexpr int MIN_ELBOW_POSITION = 0; // Minimum elbow position in degrees
constexpr int MAX_ELBOW_POSITION = 100; // Maximum elbow position in degrees
                                        

const std::pair<int, int> PRESET_POS_PICKUP = {0, 0};
const std::pair<int, int> PRESET_POS_TOWER = {50, 30};
const std::pair<int, int> PRESET_POS_SWEEP = {75, 90};
const std::pair<int, int> PRESET_POS_CLIMB = {10, 10};
                                        
std::map<int, std::pair<int, int>> arm_positions = {
  {DIGITAL_A, PRESET_POS_PICKUP},
  {DIGITAL_B, PRESET_POS_TOWER},
  {DIGITAL_X, PRESET_POS_SWEEP},
  {DIGITAL_Y, PRESET_POS_CLIMB}
};

// ========================

void initialize() {
	pros::lcd::initialize();
  pros::lcd::set_background_color(255, 0, 0);
	pros::lcd::set_text(1, "Loading BRGOS...");
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
void competition_initialize() {}

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
void autonomous() {}


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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	pros::Motor left_m(LEFT_MOTOR_PORT * -1);
	pros::Motor right_m(RIGHT_MOTOR_PORT);

  pros::MotorGroup shoulder({
      SHOULDER_MOTOR1_PORT * -1,
      SHOULDER_MOTOR2_PORT
  });

  pros::Motor elbow(ELBOW_MOTOR_PORT);

  pros::ADIDigitalOut inner_piston(INNER_PISTON_PORT, false);
  pros::ADIDigitalOut outer_piston(OUTER_PISTON_PORT, false);

  // Set motor brake brake mode 
  left_m.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_m.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  std::uint32_t shoulder_hold = 0, elbow_hold = 0;

  bool attack_mode = false;

  bool a_was_pressed = false;
  bool b_was_pressed = false;
  bool x_was_pressed = false;
  bool y_was_pressed = false;

  bool was_pressed = false;

  bool l1_was_pressed = false;
  bool l2_was_pressed = false;

  bool precision_mode = false;

  std::uint32_t flash_counter = 0;

	while (true) {

    // ======== Drivetrain control ========
		int dir = master.get_analog(ANALOG_LEFT_Y) * (master.get_digital(DIGITAL_R2) ? SLOW_DRIVETRAIN_SPEED : attack_mode ? 100 : REGULAR_DRIVETRAIN_SPEED)
                                                  / 100;
		int turn = master.get_analog(ANALOG_LEFT_X) * (master.get_digital(DIGITAL_R2) ? SLOW_TURN_SPEED : attack_mode ? (100 / (dir/2)) : REGULAR_TURN_SPEED) // Turn speed is slower in attack mode when driving fast
                                                  / 100;
		left_m.move(dir - turn);
		right_m.move(dir + turn);
    // ===================================


    // Using seperate if statements for simultaneous button presses

    if (master.get_digital(DIGITAL_L1) && !l1_was_pressed) {
      inner_piston.set_value(!inner_piston.get_value());      // Toggle inner inner_piston
      l1_was_pressed = true;
    } else if (!master.get_digital(DIGITAL_L1) && l1_was_pressed) {
      l1_was_pressed = false;
    }

    if (master.get_digital(DIGITAL_L2) && !l2_was_pressed) {
      outer_piston.set_value(!outer_piston.get_value());       // Toggle outer inner_piston
      l2_was_pressed = true;
    } else if (!master.get_digital(DIGITAL_L2) && l2_was_pressed) {
      l2_was_pressed = false;
    }

   
    // ======== Manual arm control ========

    // Shoulder Movement (Controlled by Right Joystick Y-axis)
    if (master.get_analog(ANALOG_RIGHT_Y) != 0) {
      // Limit shoulder movement
      if (
          (master.get_analog(ANALOG_RIGHT_Y) > 0 && (shoulder.get_position() < MAX_SHOULDER_POSITION || master.get_digital(DIGITAL_LEFT))) ||
          (master.get_analog(ANALOG_RIGHT_Y) < 0 && (shoulder.get_position() > MIN_SHOULDER_POSITION || master.get_digital(DIGITAL_LEFT)))
      ) {
        shoulder.move(master.get_analog(ANALOG_RIGHT_Y) * (attack_mode ? 100 : SHOULDER_SPEED) / 100);
        shoulder_hold = shoulder.get_position();
      }
    } else {
      // Hold shoulder position if no input
      shoulder.move_absolute(shoulder_hold, 127);
    }

    // Elbow Movement (Controlled by Right Joystick X-axis)
    if (master.get_analog(ANALOG_RIGHT_X) != 0) {
      // Limit elbow movement
      if (
          (master.get_analog(ANALOG_RIGHT_X) > 0 && (elbow.get_position() < MAX_ELBOW_POSITION) || master.get_digital(DIGITAL_LEFT)) ||
          (master.get_analog(ANALOG_RIGHT_X) < 0 && (elbow.get_position() > MIN_ELBOW_POSITION || master.get_digital(DIGITAL_LEFT)))
      ) {
        elbow.move(master.get_analog(ANALOG_RIGHT_X) * (attack_mode ? 100 : ELBOW_SPEED) / 100);
        elbow_hold = elbow.get_position();
      }
    } else {
      // Hold elbow position if no input
      elbow.move_absolute(elbow_hold, 127);
    }

    // ===================================

    // ======== Misc ========
    // Tare arm motors
    if (master.get_digital(DIGITAL_RIGHT)) {
      shoulder.tare_position();
      elbow.tare_position();

      shoulder_hold = 0;
      elbow_hold = 0;
    }

    // Precision mode 
    if (master.get_digital(DIGITAL_UP)) {
      precision_mode = !precision_mode;
    }

    // Flash screens when in attack mode 

    if (attack_mode) {
      if (flash_counter % 10 == 0) {
        pros::lcd::set_background_color(255, 0, 0);
      } else {
        pros::lcd::set_background_color(0, 0, 0);
      }

      flash_counter++;
    } else {
    }

    // ========================
    

    // TODO:
    // add temperature checks for motors on controller screen,
    // add movement presets for; preparing to climb, climbing, preparing to pick up ring, prepare to place ring, prepare to sweep ring, attack mode etc.
    
    // ======== Presets ========
    
    if (master.get_digital(DIGITAL_R1)) {
      
      // Attack mode
      if (master.get_digital(DIGITAL_X) && !x_was_pressed) {
        attack_mode = !attack_mode;

        if (!attack_mode) {
          pros::lcd::set_background_color(0, 0, 0);
        }

        x_was_pressed = true;
      } else if (!master.get_digital(DIGITAL_X) && x_was_pressed) {
        x_was_pressed = false;
      }

      // Climb
      if (master.get_digital(DIGITAL_Y) && !y_was_pressed) {

        // Script for climbing
        shoulder.move_absolute(0, 127); // TODO: Replace 0 with actual value
        elbow.move_absolute(0, 127);

        y_was_pressed = true;
      } else if (!master.get_digital(DIGITAL_Y) && y_was_pressed) {
        y_was_pressed = false;
      }

    } else {

      // Preset positions
      for (auto const& [key, value] : arm_positions) {
        if (master.get_digital(key) && !was_pressed) {

          shoulder.move_absolute(value.first, 127);
          elbow.move_absolute(value.second, 127);

          was_pressed = true;
        } else if (!master.get_digital(key) && was_pressed) {
          was_pressed = false;
        }
      }

    }

    // =========================


		pros::delay(20);
	}
}
