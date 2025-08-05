#include "Robot.h"

Robot::Robot() : 
    legs{Leg(UPPER_RIGHT), Leg(LOWER_RIGHT), Leg(LOWER_LEFT), Leg(UPPER_LEFT)},
    current_gait(GAIT_STANDBY),
    current_direction(STOP),
    is_moving(false),
    movement_speed(0.5),
    gait_start_time(0),
    step_duration(DEFAULT_GAIT_SPEED),
    current_step(0),
    total_steps(4),
    stride_length(DEFAULT_STRIDE_LENGTH),
    step_height(DEFAULT_STEP_HEIGHT)
{
    // Initialize leg phases (all in stance initially)
    for (int i = 0; i < LEG_COUNT; i++) {
        leg_phase[i] = false;
    }
}

void Robot::begin() {
    DEBUG_PRINTLN("🕷️ Initializing Robot...");
    
    // Initialize all legs
    for (int i = 0; i < LEG_COUNT; i++) {
        DEBUG_PRINT("Initializing leg ");
        DEBUG_PRINTLN(i);
        legs[i].begin();
        delay(200);
    }
    
    DEBUG_PRINTLN("✅ Robot initialized!");
    standby();
}

void Robot::calibrateAll() {
    DEBUG_PRINTLN("🔧 Calibrating all legs...");
    
    for (int i = 0; i < LEG_COUNT; i++) {
        DEBUG_PRINT("Calibrating leg ");
        DEBUG_PRINTLN(i);
        legs[i].calibrate();
        delay(500);
    }
    
    DEBUG_PRINTLN("✅ Calibration complete!");
    standby();
}

void Robot::standby() {
    current_gait = GAIT_STANDBY;
    current_direction = STOP;
    is_moving = false;
    
    DEBUG_PRINTLN("🦵 Moving to standby position...");
    
    // Move all legs to standby
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].standby();
        leg_phase[i] = false; // All in stance phase
    }
}

void Robot::moveForward(float speed) {
    movement_speed = constrain(speed, 0.0, 1.0);
    current_gait = GAIT_TROT;
    current_direction = FORWARD;
    is_moving = true;
    
    // Reset gait timing
    gait_start_time = millis();
    current_step = 0;
    step_duration = DEFAULT_GAIT_SPEED / (movement_speed + 0.1); // Faster speed = shorter duration
    
    DEBUG_PRINT("🚶 Moving forward at speed ");
    DEBUG_PRINTLN(speed);
}

void Robot::moveBackward(float speed) {
    movement_speed = constrain(speed, 0.0, 1.0);
    current_gait = GAIT_TROT;
    current_direction = BACKWARD;
    is_moving = true;
    
    gait_start_time = millis();
    current_step = 0;
    step_duration = DEFAULT_GAIT_SPEED / (movement_speed + 0.1);
    
    DEBUG_PRINT("🔙 Moving backward at speed ");
    DEBUG_PRINTLN(speed);
}

void Robot::turnLeft(float speed) {
    movement_speed = constrain(speed, 0.0, 1.0);
    current_gait = GAIT_TURN_LEFT;
    current_direction = LEFT;
    is_moving = true;
    
    gait_start_time = millis();
    current_step = 0;
    step_duration = DEFAULT_GAIT_SPEED / (movement_speed + 0.1);
    
    DEBUG_PRINT("↺ Turning left at speed ");
    DEBUG_PRINTLN(speed);
}

void Robot::turnRight(float speed) {
    movement_speed = constrain(speed, 0.0, 1.0);
    current_gait = GAIT_TURN_RIGHT;
    current_direction = RIGHT;
    is_moving = true;
    
    gait_start_time = millis();
    current_step = 0;
    step_duration = DEFAULT_GAIT_SPEED / (movement_speed + 0.1);
    
    DEBUG_PRINT("↻ Turning right at speed ");
    DEBUG_PRINTLN(speed);
}

void Robot::stop() {
    DEBUG_PRINTLN("⏹️ Stopping...");
    is_moving = false;
    current_direction = STOP;
    
    // Return to standby after a short delay
    delay(100);
    standby();
}

void Robot::update() {
    // Update all individual legs first
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].update();
    }
    
    // Execute current gait if moving
    if (!is_moving) return;
    
    switch (current_gait) {
        case GAIT_STANDBY:
            executeStandby();
            break;
        case GAIT_TROT:
            executeTrot();
            break;
        case GAIT_WALK:
            executeWalk();
            break;
        case GAIT_TURN_LEFT:
            executeTurnLeft();
            break;
        case GAIT_TURN_RIGHT:
            executeTurnRight();
            break;
        case GAIT_STRAFE_LEFT:
            executeStrafe(true);
            break;
        case GAIT_STRAFE_RIGHT:
            executeStrafe(false);
            break;
    }
}

void Robot::executeStandby() {
    // Already handled in standby() function
    is_moving = false;
}

void Robot::executeTrot() {
    unsigned long elapsed = millis() - gait_start_time;
    
    // Simpler trot: just alternate diagonal pairs lifting
    if (elapsed > step_duration) {
        // Move to next step
        current_step = (current_step + 1) % 2;
        gait_start_time = millis();
        
        DEBUG_PRINT("Trot step ");
        DEBUG_PRINT(current_step);
        DEBUG_PRINTLN();
        
        // EXTREME alternating pattern for maximum visibility
        if (current_step == 0) {
            // Lift diagonal pair 1 to MAXIMUM
            legs[UPPER_RIGHT].setAnglesImmediate(JointAngles(90, 170));  // Almost max
            legs[LOWER_LEFT].setAnglesImmediate(JointAngles(90, 170));   // Almost max
            legs[LOWER_RIGHT].setAnglesImmediate(JointAngles(90, 10));   // Almost min
            legs[UPPER_LEFT].setAnglesImmediate(JointAngles(90, 10));    // Almost min
        } else {
            // Lift diagonal pair 2 to MAXIMUM
            legs[LOWER_RIGHT].setAnglesImmediate(JointAngles(90, 170));  // Almost max
            legs[UPPER_LEFT].setAnglesImmediate(JointAngles(90, 170));   // Almost max
            legs[UPPER_RIGHT].setAnglesImmediate(JointAngles(90, 10));   // Almost min
            legs[LOWER_LEFT].setAnglesImmediate(JointAngles(90, 10));    // Almost min
        }
    }
}

void Robot::executeWalk() {
    // 4-beat gait: one leg at a time
    // Implementation similar to trot but with 4 phases
    // For now, use trot as basis
    executeTrot();
}

void Robot::executeTurnLeft() {
    // Similar to trot but with different stride patterns for turning
    executeTrot(); // Simplified for now
}

void Robot::executeTurnRight() {
    // Similar to trot but with different stride patterns for turning  
    executeTrot(); // Simplified for now
}

void Robot::executeStrafe(bool left) {
    // Sideways movement
    executeTrot(); // Simplified for now
}

void Robot::setLegPhases(bool ur, bool lr, bool ll, bool ul) {
    leg_phase[UPPER_RIGHT] = ur;
    leg_phase[LOWER_RIGHT] = lr;
    leg_phase[LOWER_LEFT] = ll;
    leg_phase[UPPER_LEFT] = ul;
}

void Robot::moveLegToPhase(LegId leg_id, bool swing_phase, float progress) {
    if (swing_phase) {
        // Swing phase: lift leg and move forward
        if (progress < 0.3) {
            // Lift phase
            legs[leg_id].liftUp(step_height);
        } else if (progress < 0.7) {
            // Move forward phase
            Point3D swing_pos = getSwingPosition(leg_id, (progress - 0.3) / 0.4);
            legs[leg_id].moveTo(swing_pos, 50);
        } else {
            // Put down phase
            legs[leg_id].putDown();
        }
    } else {
        // Stance phase: keep leg down and push backward
        Point3D stance_pos = getStancePosition(leg_id);
        legs[leg_id].moveTo(stance_pos, 50);
    }
}

Point3D Robot::getStancePosition(LegId leg_id) {
    // Default stance position for each leg
    // These are approximate positions - adjust based on your robot's geometry
    switch (leg_id) {
        case UPPER_RIGHT:
            return Point3D(60, 0, -30);
        case LOWER_RIGHT:
            return Point3D(60, 0, -30);
        case LOWER_LEFT:
            return Point3D(60, 0, -30);
        case UPPER_LEFT:
            return Point3D(60, 0, -30);
        default:
            return Point3D(60, 0, -30);
    }
}

Point3D Robot::getSwingPosition(LegId leg_id, float progress) {
    // Calculate swing trajectory
    Point3D stance = getStancePosition(leg_id);
    
    // Simple forward movement for now
    float forward_offset = stride_length * progress;
    if (current_direction == BACKWARD) {
        forward_offset = -forward_offset;
    }
    
    return Point3D(stance.x + forward_offset, stance.y, stance.z);
}

void Robot::centerAll() {
    DEBUG_PRINTLN("🎯 Centering all legs...");
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].center();
    }
    is_moving = false;
    current_gait = GAIT_STANDBY;
}

void Robot::emergencyStop() {
    DEBUG_PRINTLN("🚨 EMERGENCY STOP!");
    is_moving = false;
    current_direction = STOP;
    
    // Stop all leg movements
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].stop();
    }
}

void Robot::setSpeed(float speed) {
    movement_speed = constrain(speed, 0.0, 1.0);
    step_duration = DEFAULT_GAIT_SPEED / (movement_speed + 0.1);
}

void Robot::setStepHeight(float height) {
    step_height = height;
}

void Robot::setStrideLength(float length) {
    stride_length = length;
}

void Robot::printStatus() {
    DEBUG_PRINTLN("\n🕷️ Robot Status:");
    DEBUG_PRINT("Gait: ");
    DEBUG_PRINTLN(current_gait);
    DEBUG_PRINT("Direction: ");
    DEBUG_PRINTLN(current_direction);
    DEBUG_PRINT("Moving: ");
    DEBUG_PRINTLN(is_moving ? "YES" : "NO");
    DEBUG_PRINT("Speed: ");
    DEBUG_PRINTLN(movement_speed);
    DEBUG_PRINT("Step: ");
    DEBUG_PRINT(current_step);
    DEBUG_PRINT("/");
    DEBUG_PRINTLN(total_steps);
    
    DEBUG_PRINTLN("Leg phases:");
    for (int i = 0; i < LEG_COUNT; i++) {
        DEBUG_PRINT("  Leg ");
        DEBUG_PRINT(i);
        DEBUG_PRINT(": ");
        DEBUG_PRINTLN(leg_phase[i] ? "SWING" : "STANCE");
    }
    DEBUG_PRINTLN();
}