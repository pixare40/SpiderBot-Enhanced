#pragma once
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include "RobotConfig.h"

struct Point3D {
    float x, y, z;
    Point3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

struct JointAngles {
    float arm_angle;  // Upper joint angle
    float paw_angle;  // Lower joint angle
    JointAngles(float arm = 90, float paw = 90) : arm_angle(arm), paw_angle(paw) {}
};

class Leg {
private:
    LegId leg_id;
    Servo arm_servo;
    Servo paw_servo;
    uint8_t arm_pin;
    uint8_t paw_pin;
    
    // Current positions and targets
    JointAngles current_angles;
    JointAngles target_angles;
    Point3D current_position;
    Point3D target_position;
    
    // Movement interpolation
    bool is_moving;
    unsigned long move_start_time;
    unsigned long move_duration;
    JointAngles start_angles;
    
    // Inverse kinematics helper
    bool inverseKinematics(const Point3D& target, JointAngles& angles);
    
    // Forward kinematics helper
    Point3D forwardKinematics(const JointAngles& angles);
    
    // Angle constraints based on leg position
    float constrainArmAngle(float angle);
    float constrainPawAngle(float angle);

public:
    Leg(LegId id);
    
    // Initialization
    void begin();
    void calibrate();
    
    // Position control (inverse kinematics)
    bool moveTo(const Point3D& position, unsigned long duration_ms = 500);
    bool moveToSmooth(const Point3D& position, unsigned long duration_ms = 500);
    
    // Direct angle control
    void setAngles(const JointAngles& angles, unsigned long duration_ms = 500);
    void setAnglesImmediate(const JointAngles& angles);
    
    // Update function (call in main loop)
    void update();
    
    // Getters
    Point3D getCurrentPosition() const { return current_position; }
    JointAngles getCurrentAngles() const { return current_angles; }
    Point3D getTargetPosition() const { return target_position; }
    bool isMoving() const { return is_moving; }
    
    // Predefined positions
    void standby();
    void liftUp(float height = DEFAULT_STEP_HEIGHT);
    void putDown();
    void center();
    
    // Utility
    void stop();
    bool isValidPosition(const Point3D& position);
    
    // Debug
    void printStatus();
};