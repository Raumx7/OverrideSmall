/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       raulmomox & [Tu Nombre]                                   */
/*    Created:      10/13/2026                                                */
/*    Description:  Override Small Robot competition code - Unified Callbacks */
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

// Brain & Controller
brain Brain;
controller Controller = controller(primary);

// Drivetrain motors
motor mtr_l1 = motor(PORT19, true);
motor mtr_l2 = motor(PORT20, true);
motor mtr_l3 = motor(PORT8);
motor mtr_r1 = motor(PORT11);
motor mtr_r2 = motor(PORT12);
motor mtr_r3 = motor(PORT3, true);

motor_group motors_l = motor_group(mtr_l1, mtr_l2);
motor_group motors_r = motor_group(mtr_r1, mtr_r2);

// Mechanism motors
motor arm_l = motor(PORT10);
motor arm_r = motor(PORT1, true);
motor intake = motor(PORT18);
motor claw_intake = motor(PORT9);
motor claw_lift = motor(PORT2);

// State variables for Callbacks
constexpr double CLAW_DOWN_TOLERANCE = TARGET_DEG * 0.1;
bool intake_mode = false; 

bool claw_is_down() {
    return claw_lift.position(rotationUnits::deg) <= CLAW_DOWN_TOLERANCE;
}

void claw_to(double target_deg) {
    claw_lift.spinToPosition(target_deg,
        rotationUnits::deg,
        MAX_DRIVE_SPEED,
        velocityUnits::pct,
        false // 'false' para que no bloquee el hilo y el robot pueda moverse mientras se eleva
    );
}

// Callback ButtonX: raise/lower claw
void move_claw() {
    claw_to(claw_is_down() ? TARGET_DEG : 0.0);
}

// Callback ButtonY: toggle automatic intake mode
void toggle_intake() {
    intake_mode = !intake_mode;
    if (intake_mode && !claw_is_down()) {
        claw_to(0.0); // Baja la garra automáticamente si no está abajo
    }
}

int clamp(int val) {
    if (val > MAX_DRIVE_SPEED) return MAX_DRIVE_SPEED;
    if (val < -MAX_DRIVE_SPEED) return -MAX_DRIVE_SPEED;
    return val;
}

int main() {
    // Configuración inicial de frenos
    arm_l.setBrake(brakeType::hold);
    arm_r.setBrake(brakeType::hold);
    claw_lift.setBrake(brakeType::hold);
    claw_lift.setPosition(0.0, rotationUnits::deg);

    // Registro de Callbacks (Eventos de hilos de Git/VEX)
    Controller.ButtonX.pressed(move_claw);
    Controller.ButtonY.pressed(toggle_intake);

    while(true) {
        // --- CHASIS (Cálculo de potencias) ---
        int forward = Controller.Axis3.position();
        int turn = Controller.Axis1.position();

        if (std::abs(forward) < DEAD_ZONE) forward = 0;
        if (std::abs(turn) < DEAD_ZONE) turn = 0;

        int power_l = clamp(forward + turn);
        int power_r = clamp(forward - turn);

        motors_l.spin(directionType::fwd, power_l, velocityUnits::pct);
        motors_r.spin(directionType::fwd, power_r, velocityUnits::pct);

        // --- BRAZO (R1 Sube, R2 Baja) ---
        if (Controller.ButtonR1.pressing()) {
            arm_l.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
            arm_r.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
        } else if (Controller.ButtonR2.pressing()) {
            arm_l.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
            arm_r.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
        } else {
            arm_l.stop();
            arm_r.stop();
        }

        // --- INTAKE (A mete, B saca, o se activa por modo automático) ---
        if (Controller.ButtonA.pressing()) {
            intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);
        } else if (Controller.ButtonB.pressing()) {
            intake.spin(directionType::fwd, -MAX_DRIVE_SPEED, velocityUnits::pct);
        } else if (intake_mode) {
            intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);
        } else {
            intake.stop();
        }

        // --- RODILLOS DE GARRA (L1 agarra, L2 suelta, o sigue al modo automático) ---
        if (Controller.ButtonL1.pressing()) {
            claw_intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);
        } else if (Controller.ButtonL2.pressing()) {
            claw_intake.spin(directionType::fwd, -MAX_DRIVE_SPEED, velocityUnits::pct);
        } else if (intake_mode) {
            claw_intake.spin(directionType::fwd, MAX_DRIVE_SPEED, velocityUnits::pct);
        } else {
            claw_intake.stop();
        }

        wait(20, timeUnits::msec); // 20ms es el estándar óptimo para actualización física en V5
    }
}
