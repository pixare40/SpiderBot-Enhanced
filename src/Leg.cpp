#include "Leg.h"

// Pin mapping for each leg based on hardware configuration
static const uint8_t LEG_ARM_PINS[LEG_COUNT] = {
    UPPER_RIGHT_ARM, LOWER_RIGHT_ARM, LOWER_LEFT_ARM, UPPER_LEFT_ARM
};

static const uint8_t LEG_PAW_PINS[LEG_COUNT] = {
    UPPER_RIGHT_PAW, LOWER_RIGHT_PAW, LOWER_LEFT_PAW, UPPER_LEFT_PAW
};

// Calibration offset mapping for each leg
static const int8_t LEG_ARM_OFFSETS[LEG_COUNT] = {
    UPPER_RIGHT_ARM_OFFSET, LOWER_RIGHT_ARM_OFFSET, LOWER_LEFT_ARM_OFFSET, UPPER_LEFT_ARM_OFFSET
};

static const int8_t LEG_PAW_OFFSETS[LEG_COUNT] = {
    UPPER_RIGHT_PAW_OFFSET, LOWER_RIGHT_PAW_OFFSET, LOWER_LEFT_PAW_OFFSET, UPPER_LEFT_PAW_OFFSET
};

// Servo direction multipliers - DISABLED for now to match tutorial behavior exactly
// All servos will use direct angle mapping as in the working tutorials
static const int8_t LEG_ARM_DIR[LEG_COUNT] = { 1, 1, 1, 1 };      // No inversion
static const int8_t LEG_PAW_DIR[LEG_COUNT] = { 1, 1, 1, 1 };      // No inversion

Leg::Leg(LegId id) : leg_id(id), is_moving(false), move_duration(0) {
    arm_pin = LEG_ARM_PINS[id];
    paw_pin = LEG_PAW_PINS[id];
    
    // Initialize to safe defaults
    current_angles = JointAngles(90, 90);
    target_angles = current_angles;
    current_position = forwardKinematics(current_angles);
    target_position = current_position;
}

void Leg::begin() {
    // Initialize servos only - don't move to position yet
    arm_servo.attach(arm_pin, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    paw_servo.attach(paw_pin, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    
    DEBUG_PRINT("Leg ");
    DEBUG_PRINT(leg_id);
    DEBUG_PRINT(" servos attached to pins ");
    DEBUG_PRINT(arm_pin);
    DEBUG_PRINT(", ");
    DEBUG_PRINTLN(paw_pin);
}

void Leg::calibrate() {
    // Move through calibration sequence
    DEBUG_PRINT("Calibrating leg ");
    DEBUG_PRINTLN(leg_id);
    
    // Center position (all servos 90°)
    setAnglesImmediate(JointAngles(90, 90));
    delay(1000);
    
    // Test range of motion
    setAnglesImmediate(JointAngles(60, 60));
    delay(800);
    setAnglesImmediate(JointAngles(120, 120));
    delay(800);
    setAnglesImmediate(JointAngles(90, 90));
    delay(800);
    
    // Move to standby position
    standby();
    delay(500);
}

void Leg::manualCalibration() {
    DEBUG_PRINT("\n🔧 Manual Calibration for Leg ");
    DEBUG_PRINTLN(leg_id);
    DEBUG_PRINTLN("==========================================");
    DEBUG_PRINTLN("Goal: Set servos to mechanical zero position");
    DEBUG_PRINTLN("- ARM servo should point FORWARD");
    DEBUG_PRINTLN("- PAW servo should point DOWN");
    DEBUG_PRINTLN("");
    
    // Start at center position
    DEBUG_PRINTLN("Setting servos to 90° (center position)...");
    setAnglesImmediate(JointAngles(90, 90));
    delay(2000);
    
    // Test different angles to help user identify correct zero
    int test_angles[] = {60, 70, 80, 90, 100, 110, 120};
    int num_angles = sizeof(test_angles) / sizeof(test_angles[0]);
    
    DEBUG_PRINTLN("Testing ARM servo positions:");
    for (int i = 0; i < num_angles; i++) {
        DEBUG_PRINT("ARM angle: ");
        DEBUG_PRINT(test_angles[i]);
        DEBUG_PRINTLN("° - Check if arm points forward");
        setAnglesImmediate(JointAngles(test_angles[i], 90));
        delay(1500);
    }
    
    DEBUG_PRINTLN("\nTesting PAW servo positions:");
    for (int i = 0; i < num_angles; i++) {
        DEBUG_PRINT("PAW angle: ");
        DEBUG_PRINT(test_angles[i]);
        DEBUG_PRINTLN("° - Check if paw points down");
        setAnglesImmediate(JointAngles(90, test_angles[i]));
        delay(1500);
    }
    
    // Return to neutral
    setAnglesImmediate(JointAngles(90, 90));
    
    DEBUG_PRINTLN("\n✅ Calibration test complete!");
    DEBUG_PRINTLN("📝 If servos don't align correctly at 90°:");
    DEBUG_PRINTLN("   Update offset values in RobotConfig.h");
    DEBUG_PRINT("   Current offsets - ARM: ");
    DEBUG_PRINT(LEG_ARM_OFFSETS[leg_id]);
    DEBUG_PRINT("°, PAW: ");
    DEBUG_PRINT(LEG_PAW_OFFSETS[leg_id]);
    DEBUG_PRINTLN("°");
    DEBUG_PRINTLN("==========================================\n");
}

bool Leg::inverseKinematics(const Point3D& target, JointAngles& angles) {
    // 2D inverse kinematics for leg in X-Z plane (Y is lateral offset)
    float x = target.x;
    float z = target.z;
    
    // Calculate distance from shoulder to target
    float distance = sqrt(x*x + z*z);
    
    // Check if target is reachable
    float L1 = LEG_SEGMENT_1_LENGTH;
    float L2 = LEG_SEGMENT_2_LENGTH;
    
    if (distance > (L1 + L2) || distance < abs(L1 - L2)) {
        DEBUG_PRINTLN("Target unreachable by leg");
        return false;
    }
    
    // Calculate angles using law of cosines
    float cos_angle2 = (L1*L1 + L2*L2 - distance*distance) / (2*L1*L2);
    
    // Clamp to valid range to avoid numerical errors
    cos_angle2 = constrain(cos_angle2, -1.0, 1.0);
    
    float angle2 = acos(cos_angle2);
    float angle1 = atan2(z, x) - atan2(L2*sin(angle2), L1 + L2*cos(angle2));
    
    // Convert to degrees and apply servo mapping
    angles.arm_angle = degrees(angle1) + 90; // Offset for servo zero position
    angles.paw_angle = degrees(angle2);
    
    // Apply constraints
    angles.arm_angle = constrainArmAngle(angles.arm_angle);
    angles.paw_angle = constrainPawAngle(angles.paw_angle);
    
    return true;
}

Point3D Leg::forwardKinematics(const JointAngles& angles) {
    // Convert angles to radians and adjust for servo offset
    float theta1 = radians(angles.arm_angle - 90);
    float theta2 = radians(angles.paw_angle);
    
    float L1 = LEG_SEGMENT_1_LENGTH;
    float L2 = LEG_SEGMENT_2_LENGTH;
    
    // Calculate end-effector position
    float x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    float z = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    
    return Point3D(x, 0, z); // Y is always 0 for leg plane
}

float Leg::constrainArmAngle(float angle) {
    // Different constraints based on leg position
    switch(leg_id) {
        case UPPER_RIGHT:
        case LOWER_RIGHT:
            return constrain(angle, 30, 150);
        case UPPER_LEFT:
        case LOWER_LEFT:
            return constrain(angle, 30, 150);
        default:
            return constrain(angle, 0, 180);
    }
}

float Leg::constrainPawAngle(float angle) {
    // STRICT paw angle constraints to prevent servo damage
    // Never allow extreme angles that can burn out servos
    return constrain(angle, 30, 150);
}

bool Leg::moveTo(const Point3D& position, unsigned long duration_ms) {
    if (!isValidPosition(position)) {
        DEBUG_PRINTLN("Invalid target position");
        return false;
    }
    
    JointAngles new_angles;
    if (!inverseKinematics(position, new_angles)) {
        return false;
    }
    
    setAngles(new_angles, duration_ms);
    target_position = position;
    return true;
}

bool Leg::moveToSmooth(const Point3D& position, unsigned long duration_ms) {
    // Same as moveTo but could add additional smoothing algorithms
    return moveTo(position, duration_ms);
}

void Leg::setAngles(const JointAngles& angles, unsigned long duration_ms) {
    start_angles = current_angles;
    target_angles = angles;
    is_moving = true;
    move_start_time = millis();
    move_duration = duration_ms;
}

void Leg::setAnglesImmediate(const JointAngles& angles) {
    current_angles = angles;
    target_angles = angles;
    is_moving = false;
    
    // Apply orientation mapping, calibration offsets and safety constraints
    float arm_tmp = constrainArmAngle(angles.arm_angle);
    float paw_tmp = constrainPawAngle(angles.paw_angle);

    // Apply direction mapping for mirrored servos + calibration offsets
    float final_arm = 90 + LEG_ARM_DIR[leg_id] * (arm_tmp - 90) + LEG_ARM_OFFSETS[leg_id];
    float final_paw = 90 + LEG_PAW_DIR[leg_id] * (paw_tmp - 90) + LEG_PAW_OFFSETS[leg_id];

    // Final safety constraint after offset & direction mapping
    final_arm = constrain(final_arm, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    final_paw = constrain(final_paw, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    // Debug output before writing to servos
    DEBUG_PRINT("Leg ");
    DEBUG_PRINT(leg_id);
    DEBUG_PRINT(" - Requested: (");
    DEBUG_PRINT(angles.arm_angle);
    DEBUG_PRINT("°, ");
    DEBUG_PRINT(angles.paw_angle);
    DEBUG_PRINT("°) → Hardware: (");
    DEBUG_PRINT(final_arm);
    DEBUG_PRINT("°, ");
    DEBUG_PRINT(final_paw);
    DEBUG_PRINTLN("°)");
    
    arm_servo.write(final_arm);
    paw_servo.write(final_paw);
    
    // Update current position
    current_position = forwardKinematics(current_angles);
}

void Leg::update() {
    if (!is_moving) return;
    
    unsigned long elapsed = millis() - move_start_time;
    
    if (elapsed >= move_duration) {
        // Movement complete
        current_angles = target_angles;
        is_moving = false;
        setAnglesImmediate(current_angles);
        return;
    }
    
    // Calculate interpolation factor (0.0 to 1.0)
    float progress = (float)elapsed / move_duration;
    
    // Smooth interpolation using ease-in-out
    progress = progress * progress * (3.0 - 2.0 * progress);
    
    // Interpolate angles
    JointAngles interpolated;
    interpolated.arm_angle = start_angles.arm_angle + 
        progress * (target_angles.arm_angle - start_angles.arm_angle);
    interpolated.paw_angle = start_angles.paw_angle + 
        progress * (target_angles.paw_angle - start_angles.paw_angle);
    
    // Apply to servos
    setAnglesImmediate(interpolated);
}

void Leg::standby() {
    // Use tutorial standby angles EXACTLY as specified
    // From text.h: G14=60, G12=90, G13=90, G15=120, G16=120, G5=90, G4=90, G2=60
    JointAngles standby_angles;
    
    switch(leg_id) {
        case UPPER_RIGHT:  // pins 12(arm), 14(paw)
            standby_angles = JointAngles(90, 60);  // G12=90°, G14=60°
            break;
        case LOWER_RIGHT:  // pins 13(arm), 15(paw) 
            standby_angles = JointAngles(90, 120); // G13=90°, G15=120°
            break;
        case LOWER_LEFT:   // pins 4(arm), 2(paw)
            standby_angles = JointAngles(90, 60);  // G4=90°, G2=60°
            break;
        case UPPER_LEFT:   // pins 5(arm), 16(paw)
            standby_angles = JointAngles(90, 120); // G5=90°, G16=120°
            break;
    }
    
    setAnglesImmediate(standby_angles); // Use immediate positioning like tutorials
}

void Leg::liftUp(float height) {
    // Simple lift: move paw servo up by reducing angle
    JointAngles current = getCurrentAngles();
    JointAngles lifted;
    
    lifted.arm_angle = current.arm_angle;
    
    // Lift by adjusting paw angle (hardware-specific)
    switch(leg_id) {
        case UPPER_RIGHT:  // pins 12(arm), 14(paw) - lift means higher angle
        case LOWER_LEFT:   // pins 4(arm), 2(paw)
            lifted.paw_angle = 90;  // Lift position
            break;
        case LOWER_RIGHT:  // pins 13(arm), 15(paw) - lift means lower angle  
        case UPPER_LEFT:   // pins 5(arm), 16(paw)
            lifted.paw_angle = 90;  // Lift position
            break;
    }
    
    setAngles(lifted, 300);
}

void Leg::putDown() {
    // Return to standby ground position
    standby();
}

void Leg::center() {
    setAngles(JointAngles(90, 90), 500);
}

void Leg::stop() {
    is_moving = false;
    target_angles = current_angles;
    target_position = current_position;
}

bool Leg::isValidPosition(const Point3D& position) {
    // Check workspace boundaries
    float distance = sqrt(position.x*position.x + position.z*position.z);
    float max_reach = LEG_SEGMENT_1_LENGTH + LEG_SEGMENT_2_LENGTH;
    float min_reach = abs(LEG_SEGMENT_1_LENGTH - LEG_SEGMENT_2_LENGTH);
    
    return (distance >= min_reach && distance <= max_reach);
}

void Leg::printStatus() {
    DEBUG_PRINT("Leg ");
    DEBUG_PRINT(leg_id);
    DEBUG_PRINT(" - Angles: (");
    DEBUG_PRINT(current_angles.arm_angle);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(current_angles.paw_angle);
    DEBUG_PRINT(") Position: (");
    DEBUG_PRINT(current_position.x);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(current_position.y);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(current_position.z);
    DEBUG_PRINT(") Moving: ");
    DEBUG_PRINTLN(is_moving ? "YES" : "NO");
}