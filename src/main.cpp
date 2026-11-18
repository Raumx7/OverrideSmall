/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       raulmomox                                                 */
/*    Created:      10/13/2026, 9:59:16 AM                                    */
/*    Description:  Override Small Robot competition code                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <cmath>

#include "vex.h"

using namespace vex;

// Drive constants
constexpr int MAX_DRIVE_SPEED = 100;
constexpr int DEAD_ZONE = 5;
constexpr int MAX_SPEED = 60;

// Gear constants
constexpr double GEAR_RATIO = 48.0 / 12.0;
constexpr double TARGET_DEG = GEAR_RATIO * 90.0;

// The claw counts as "down" when it is within this many motor degrees of 0
constexpr double CLAW_DOWN_TOLERANCE = TARGET_DEG * 0.1;

// Control the intake and claw returning states
bool intake_mode = false;
bool lift_returning = false;

// Brain
brain Brain;

// Controller
controller Controller = controller(primary);

// Drivetrain motors left
motor mtr_l1 = motor(PORT19, true);
motor mtr_l2 = motor(PORT20, true);
motor mtr_l3 = motor(PORT8);

// Drivetrain motors right
motor mtr_r1 = motor(PORT11);
motor mtr_r2 = motor(PORT12);
motor mtr_r3 = motor(PORT3, true);

// Lift motors
motor lift_l = motor(PORT10);
motor lift_r = motor(PORT1, true);

// Drivetrain motor groups
motor_group motors_l = motor_group(mtr_l1, mtr_l2, mtr_l3);
motor_group motors_r = motor_group(mtr_r1, mtr_r2, mtr_r3);

// Lift motor group
motor_group lift(lift_l, lift_r);

// In-take motor
motor intake = motor(PORT18);

// Claw motors
motor claw_intake = motor(PORT9);
motor claw_lift = motor(PORT2);

bool claw_is_down() {
    return claw_lift.position(rotationUnits::deg) <= CLAW_DOWN_TOLERANCE;
}

void claw_to(double target_deg) {
    claw_lift.spinToPosition(target_deg,
        rotationUnits::deg,
        MAX_DRIVE_SPEED,
        velocityUnits::pct,
        false
    );
}

void move_claw() {
    claw_to(claw_is_down() ? TARGET_DEG : 0.0);
}

// Callback ButtonY: toggle automatic intake mode
void toggle_intake() {
    intake_mode = !intake_mode;

    if (intake_mode) {
        if (!claw_is_down()) {
            claw_to(0.0); 
        }
        lift.spinToPosition(0.0, rotationUnits::deg, MAX_SPEED, velocityUnits::pct, false);
        lift_returning = true;
    }
}

int clamp(int val) {
    if (val > MAX_DRIVE_SPEED) return MAX_DRIVE_SPEED;
    if (val < -MAX_DRIVE_SPEED) return -MAX_DRIVE_SPEED;
    return val;
}

int main() {

    lift.setStopping(brakeType::hold);

    claw_lift.setBrake(brakeType::hold);
    claw_lift.setPosition(0.0, rotationUnits::deg);

    // The arm's power-on position is its starting position (0 deg)
    lift.setPosition(0.0, rotationUnits::deg);

    // Claw down or up (0° | 90°)
    Controller.ButtonX.pressed(move_claw);

    // Intake mode (ButtonY toggles it): claw down, intake/claw_intake running and lift dowm
    Controller.ButtonY.pressed(toggle_intake);

    while(true) {

        int forward = Controller.Axis3.position();
        int turn = Controller.Axis1.position();

        if (std::abs(forward) < DEAD_ZONE)
            forward = 0;
        if (std::abs(turn) < DEAD_ZONE)
            turn = 0;

        int power_l = clamp(forward + turn);
        int power_r = clamp(forward - turn);

        motors_l.spin(directionType::fwd, power_l, velocityUnits::pct);
        motors_r.spin(directionType::fwd, power_r, velocityUnits::pct);

        // Pressing R1 or R2 cancels the automatic lift return.
        if (Controller.ButtonR1.pressing())
        {
            lift_returning = false;
            lift.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
        }

        else if (Controller.ButtonR2.pressing())
        {
            lift_returning = false;
            lift.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
        }

        // Don't stop the arm while it is returning, or the return would be cancelled
        else if (!lift_returning)
            lift.stop();

        // Intake: A in, B out. Buttons override intake mode while held.
        if (Controller.ButtonA.pressing())
            intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);

        else if (Controller.ButtonB.pressing())
            intake.spin(directionType::fwd, -MAX_DRIVE_SPEED, velocityUnits::pct);

        else if (intake_mode)
            intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);

        else
            intake.stop();

        // Claw: L1 grab, L2 release. Buttons override intake mode while held.
        if (Controller.ButtonL1.pressing())
            claw_intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);

        else if (Controller.ButtonL2.pressing())
            claw_intake.spin(directionType::fwd, -MAX_DRIVE_SPEED, velocityUnits::pct);

        else if (intake_mode)
            claw_intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);

        else
            claw_intake.stop();

        wait(30, timeUnits::msec);
    }
}
