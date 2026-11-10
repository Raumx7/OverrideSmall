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

// Motor groups
motor_group motors_l = motor_group(mtr_l1, mtr_l2);
motor_group motors_r = motor_group(mtr_r1, mtr_r2);

// Arm motors
motor arm_l = motor(PORT10);
motor arm_r = motor(PORT1, true);

// In-take motor
motor intake = motor(PORT18);

// Claw motors
motor claw_intake = motor(PORT9);
motor claw_lift = motor(PORT2);

// The claw counts as "down" when it is within this many motor degrees of 0
constexpr double CLAW_DOWN_TOLERANCE = TARGET_DEG * 0.1;

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

// ButtonX: raise the claw if it is down, lower it otherwise.
// Uses the real position, so it stays correct after intake mode lowers the claw.
void move_claw() {
    claw_to(claw_is_down() ? TARGET_DEG : 0.0);
}

// Intake mode: send both arm (lift) motors back to where they were at power-on
void arm_to_start() {
    arm_l.spinToPosition(0.0, rotationUnits::deg, MAX_SPEED, velocityUnits::pct, false);
    arm_r.spinToPosition(0.0, rotationUnits::deg, MAX_SPEED, velocityUnits::pct, false);
}

int clamp(int val) {
    if (val > MAX_DRIVE_SPEED) return MAX_DRIVE_SPEED;
    if (val < -MAX_DRIVE_SPEED) return -MAX_DRIVE_SPEED;
    return val;
}

int main() {

    arm_l.setBrake(brakeType::hold);
    arm_r.setBrake(brakeType::hold);

    claw_lift.setBrake(brakeType::hold);

    claw_lift.setPosition(0.0, rotationUnits::deg);

    // The arm's power-on position is its starting position (0 deg)
    arm_l.setPosition(0.0, rotationUnits::deg);
    arm_r.setPosition(0.0, rotationUnits::deg);

    Controller.ButtonX.pressed(move_claw);

    // Intake mode (ButtonY toggles it): claw down, intake and claw running
    bool intake_mode = false;
    bool y_was_pressed = false;

    // True while intake mode is bringing the arm back to its starting position
    bool arm_returning = false;

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
        
        // ButtonY: toggle intake mode (acts once per press, not while held)
        bool y_pressed = Controller.ButtonY.pressing();
        if (y_pressed && !y_was_pressed) {
            intake_mode = !intake_mode;
            if (intake_mode)
            {
                if (!claw_is_down())
                    claw_to(0.0);
                arm_to_start();
                arm_returning = true;
            }
        }
        y_was_pressed = y_pressed;

        // Arm: R1 up, L1 down
        // Pressing R1 or L1 cancels the automatic return.
        if (Controller.ButtonR1.pressing())
        {
            arm_returning = false;
            arm_l.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
            arm_r.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
        }

        else if (Controller.ButtonL1.pressing())
        {
            arm_returning = false;
            arm_l.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
            arm_r.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
        }

        // Don't stop the arm while it is returning, or the return would be cancelled
        else if (!arm_returning)
        {
            arm_l.stop();
            arm_r.stop();
        }

        // Intake: A in, B out. Buttons override intake mode while held.
        if (Controller.ButtonA.pressing())
            intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);

        else if (Controller.ButtonB.pressing())
            intake.spin(directionType::fwd, -MAX_DRIVE_SPEED, velocityUnits::pct);

        else if (intake_mode)
            intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);

        else
            intake.stop();

        // Claw: R2 grab, L2 release. Buttons override intake mode while held.
        if (Controller.ButtonR2.pressing())
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
