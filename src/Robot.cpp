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
    
    // Initialize all leg servos
    for (int i = 0; i < LEG_COUNT; i++) {
        DEBUG_PRINT("Initializing leg ");
        DEBUG_PRINTLN(i);
        legs[i].begin();
        delay(100);
    }
    
    DEBUG_PRINTLN("📍 Moving all servos to center position (90°,90°)...");
    // Step 1: Move all servos to center position like tutorial
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].setAnglesImmediate(JointAngles(90, 90));
    }
    delay(1000); // Wait for servos to reach position
    
    DEBUG_PRINTLN("🦵 Moving to standby position...");
    // Step 2: Move to standby position
    standby();
    delay(1000); // Wait for servos to reach standby
    
    DEBUG_PRINTLN("✅ Robot initialized and ready!");
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
    
    // Use exact tutorial standby positions from text.h
    // G14, G12, G13, G15, G16, G5, G4, G2
    // {60, 90, 90, 120, 120, 90, 90, 60}
    legs[UPPER_RIGHT].setAnglesImmediate(JointAngles(90, 60));   // G12=90°, G14=60°
    legs[LOWER_RIGHT].setAnglesImmediate(JointAngles(90, 120));  // G13=90°, G15=120°
    legs[LOWER_LEFT].setAnglesImmediate(JointAngles(90, 60));    // G4=90°, G2=60°
    legs[UPPER_LEFT].setAnglesImmediate(JointAngles(90, 120));   // G5=90°, G16=120°
    
    // Reset all leg phases
    for (int i = 0; i < LEG_COUNT; i++) {
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
    step_duration = 200; // Match tutorial timing (200ms per step)
    
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
    step_duration = 200; // Match tutorial timing
    
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
    step_duration = 200; // Match tutorial timing
    
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
    step_duration = 200; // Match tutorial timing
    
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
    
    // Use the proven walking pattern from tutorials
    if (elapsed > step_duration) {
        // Move to next step in the sequence
        current_step = (current_step + 1) % 11; // 11 steps in the tutorial sequence
        gait_start_time = millis();
        
        DEBUG_PRINT("Trot step ");
        DEBUG_PRINT(current_step);
        DEBUG_PRINTLN();
        
        // Execute the current step based on direction
        if (current_direction == FORWARD) {
            executeForwardStep(current_step);
        } else if (current_direction == BACKWARD) {
            executeBackwardStep(current_step);
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
    unsigned long elapsed = millis() - gait_start_time;
    
    if (elapsed > step_duration) {
        // Move to next step in the turn sequence
        current_step = (current_step + 1) % 8; // 8 steps in turn sequence
        gait_start_time = millis();
        
        DEBUG_PRINT("Turn left step ");
        DEBUG_PRINT(current_step);
        DEBUG_PRINTLN();
        
        executeTurnLeftStep(current_step);
    }
}

void Robot::executeTurnRight() {
    unsigned long elapsed = millis() - gait_start_time;
    
    if (elapsed > step_duration) {
        // Move to next step in the turn sequence
        current_step = (current_step + 1) % 8; // 8 steps in turn sequence  
        gait_start_time = millis();
        
        DEBUG_PRINT("Turn right step ");
        DEBUG_PRINT(current_step);
        DEBUG_PRINTLN();
        
        executeTurnRightStep(current_step);
    }
}

// Forward walking sequence from tutorial (4.2forward.ino)
void Robot::executeForwardStep(int step) {
    // Tutorial forward sequence - direct servo angle mapping
    // Pin mapping: G14=UR_PAW, G12=UR_ARM, G13=LR_ARM, G15=LR_PAW, G16=UL_PAW, G5=UL_ARM, G4=LL_ARM, G2=LL_PAW
    int forward_sequence[11][8] = {
        {70, 90, 90, 110, 110, 90, 90, 70},  // Standby
        {90, 90, 90, 110, 110, 90, 90, 90},  // Right upper paw and left lower paw raised
        {90, 120, 90, 110, 110, 90, 60, 90}, // Right upper arm and left lower arm forward
        {70, 120, 90, 110, 110, 90, 60, 70}, // Right upper paw and left lower paw drop
        {70, 120, 90, 90, 90, 90, 60, 70},   // Left upper paw and right lower paw raised
        {70, 90, 90, 90, 90, 90, 90, 70},    // Right upper arm and left lower arm back
        {70, 90, 120, 90, 90, 60, 90, 70},   // Left upper arm and right lower arm forward
        {70, 90, 120, 110, 110, 60, 90, 70}, // Left upper paw and right lower paw drop
        {90, 90, 120, 110, 110, 60, 90, 90}, // Right upper paw and left lower paw raised
        {90, 90, 90, 110, 110, 90, 90, 90},  // Left upper arm and right lower arm back
        {70, 90, 90, 110, 110, 90, 90, 70}   // Right upper paw and left lower paw drop
    };
    
    // Apply the angles to legs (UR_PAW, UR_ARM, LR_ARM, LR_PAW, UL_PAW, UL_ARM, LL_ARM, LL_PAW)
    legs[UPPER_RIGHT].setAnglesImmediate(JointAngles(forward_sequence[step][1], forward_sequence[step][0])); // ARM, PAW
    legs[LOWER_RIGHT].setAnglesImmediate(JointAngles(forward_sequence[step][2], forward_sequence[step][3])); // ARM, PAW
    legs[LOWER_LEFT].setAnglesImmediate(JointAngles(forward_sequence[step][6], forward_sequence[step][7]));  // ARM, PAW
    legs[UPPER_LEFT].setAnglesImmediate(JointAngles(forward_sequence[step][5], forward_sequence[step][4]));  // ARM, PAW
}

// Backward walking sequence from tutorial (4.3back.ino)
void Robot::executeBackwardStep(int step) {
    // Tutorial backward sequence
    int backward_sequence[11][8] = {
        {70, 90, 90, 110, 110, 90, 90, 70},  // Standby
        {90, 90, 90, 110, 110, 90, 90, 90},  // Right upper paw and left lower paw raised
        {90, 60, 90, 110, 110, 90, 120, 90}, // Right upper arm and left lower arm back
        {70, 60, 90, 110, 110, 90, 120, 70}, // Right upper paw and left lower paw drop
        {70, 60, 90, 90, 90, 90, 120, 70},   // Left upper paw and right lower paw raised
        {70, 90, 90, 90, 90, 90, 90, 70},    // Right upper arm and left lower arm forward
        {70, 90, 60, 90, 90, 120, 90, 70},   // Left upper arm and right lower arm back
        {70, 90, 60, 110, 110, 120, 90, 70}, // Left upper paw and right lower paw drop
        {90, 90, 60, 110, 110, 120, 90, 90}, // Right upper paw and left lower paw raised
        {90, 90, 90, 110, 110, 90, 90, 90},  // Left upper arm and right lower arm forward
        {70, 90, 90, 110, 110, 90, 90, 70}   // Right upper paw and left lower paw drop
    };
    
    // Apply the angles to legs
    legs[UPPER_RIGHT].setAnglesImmediate(JointAngles(backward_sequence[step][1], backward_sequence[step][0])); // ARM, PAW
    legs[LOWER_RIGHT].setAnglesImmediate(JointAngles(backward_sequence[step][2], backward_sequence[step][3])); // ARM, PAW
    legs[LOWER_LEFT].setAnglesImmediate(JointAngles(backward_sequence[step][6], backward_sequence[step][7]));  // ARM, PAW
    legs[UPPER_LEFT].setAnglesImmediate(JointAngles(backward_sequence[step][5], backward_sequence[step][4]));  // ARM, PAW
}

// Turn left sequence from tutorial (5.1turn_left.ino)
void Robot::executeTurnLeftStep(int step) {
    // Tutorial turn left sequence
    int turn_left_sequence[8][8] = {
        {70, 90, 90, 110, 110, 90, 90, 70},   // Standby
        {90, 90, 90, 110, 110, 90, 90, 90},   // Right upper paw and left lower paw raised
        {90, 135, 90, 110, 110, 90, 135, 90}, // Right upper arm forward, left lower arm back
        {70, 135, 90, 110, 110, 90, 135, 70}, // Right upper paw and left lower paw drop
        {70, 135, 90, 90, 90, 90, 135, 70},   // Left upper paw and right lower paw raised
        {70, 135, 135, 90, 90, 135, 135, 70}, // Left upper arm back and right lower arm forward
        {70, 135, 135, 110, 110, 135, 135, 70}, // Left upper paw and right lower paw drop
        {70, 90, 90, 110, 110, 90, 90, 70}    // Return to standby
    };
    
    // Apply the angles to legs
    legs[UPPER_RIGHT].setAnglesImmediate(JointAngles(turn_left_sequence[step][1], turn_left_sequence[step][0])); // ARM, PAW
    legs[LOWER_RIGHT].setAnglesImmediate(JointAngles(turn_left_sequence[step][2], turn_left_sequence[step][3])); // ARM, PAW
    legs[LOWER_LEFT].setAnglesImmediate(JointAngles(turn_left_sequence[step][6], turn_left_sequence[step][7]));  // ARM, PAW
    legs[UPPER_LEFT].setAnglesImmediate(JointAngles(turn_left_sequence[step][5], turn_left_sequence[step][4]));  // ARM, PAW
}

// Turn right sequence (mirror of turn left)
void Robot::executeTurnRightStep(int step) {
    // Mirror the turn left sequence for right turns
    int turn_right_sequence[8][8] = {
        {70, 90, 90, 110, 110, 90, 90, 70},   // Standby
        {90, 90, 90, 110, 110, 90, 90, 90},   // Left upper paw and right lower paw raised
        {90, 45, 90, 110, 110, 90, 45, 90},   // Left upper arm forward, right lower arm back
        {70, 45, 90, 110, 110, 90, 45, 70},   // Left upper paw and right lower paw drop
        {70, 45, 90, 90, 90, 90, 45, 70},     // Right upper paw and left lower paw raised
        {70, 45, 45, 90, 90, 45, 45, 70},     // Right upper arm back and left lower arm forward
        {70, 45, 45, 110, 110, 45, 45, 70},   // Right upper paw and left lower paw drop
        {70, 90, 90, 110, 110, 90, 90, 70}    // Return to standby
    };
    
    // Apply the angles to legs
    legs[UPPER_RIGHT].setAnglesImmediate(JointAngles(turn_right_sequence[step][1], turn_right_sequence[step][0])); // ARM, PAW
    legs[LOWER_RIGHT].setAnglesImmediate(JointAngles(turn_right_sequence[step][2], turn_right_sequence[step][3])); // ARM, PAW
    legs[LOWER_LEFT].setAnglesImmediate(JointAngles(turn_right_sequence[step][6], turn_right_sequence[step][7]));  // ARM, PAW
    legs[UPPER_LEFT].setAnglesImmediate(JointAngles(turn_right_sequence[step][5], turn_right_sequence[step][4]));  // ARM, PAW
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
    DEBUG_PRINTLN("🎯 Centering all legs to standby...");
    is_moving = false;
    current_gait = GAIT_STANDBY;
    
    // Use exact tutorial standby positions from text.h  
    // G14, G12, G13, G15, G16, G5, G4, G2
    // {60, 90, 90, 120, 120, 90, 90, 60}
    legs[UPPER_RIGHT].setAnglesImmediate(JointAngles(90, 60));   // G12=90°, G14=60°
    legs[LOWER_RIGHT].setAnglesImmediate(JointAngles(90, 120));  // G13=90°, G15=120°
    legs[LOWER_LEFT].setAnglesImmediate(JointAngles(90, 60));    // G4=90°, G2=60°
    legs[UPPER_LEFT].setAnglesImmediate(JointAngles(90, 120));   // G5=90°, G16=120°
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