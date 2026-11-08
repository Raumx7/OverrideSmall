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
constexpr double TARGET_DEG = GEAR_RATIO * 90;

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

void move_claw() {

    static bool is_lifted = false;

    double target_deg = is_lifted ? 0.0 : TARGET_DEG;

    claw_lift.spinToPosition(target_deg,
        rotationUnits::deg,
        MAX_DRIVE_SPEED,
        velocityUnits::pct,
        false
    );

    is_lifted = !is_lifted;

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

    Controller.ButtonX.pressed(move_claw);
   
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
        
        if (Controller.ButtonR1.pressing())
        {
            arm_l.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
            arm_r.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
        }

        else if (Controller.ButtonR2.pressing())
        {
            arm_l.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
            arm_r.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
        }

        else
        {
            arm_l.stop();
            arm_r.stop();
        }

        if (Controller.ButtonA.pressing())
            intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);

        else if (Controller.ButtonB.pressing())
            intake.spin(directionType::fwd, -MAX_DRIVE_SPEED, velocityUnits::pct);

        else
            intake.stop();

        if (Controller.ButtonL1.pressing())
            claw_intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);
        

        else if (Controller.ButtonL2.pressing())
            claw_intake.spin(directionType::fwd, -MAX_DRIVE_SPEED, velocityUnits::pct);
        

        else
            claw_intake.stop();


        wait(30, timeUnits::msec);
    }
}
