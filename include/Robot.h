#pragma once
#include <Arduino.h>
#include "Leg.h"
#include "RobotConfig.h"

enum GaitType {
    GAIT_STANDBY,
    GAIT_TROT,
    GAIT_WALK,
    GAIT_TURN_LEFT,
    GAIT_TURN_RIGHT,
    GAIT_STRAFE_LEFT,
    GAIT_STRAFE_RIGHT
};

enum MovementDirection {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
};

class Robot {
private:
    Leg legs[LEG_COUNT];
    
    // Current state
    GaitType current_gait;
    MovementDirection current_direction;
    bool is_moving;
    float movement_speed;  // 0.0 to 1.0
    
    // Gait timing
    unsigned long gait_start_time;
    unsigned long step_duration;
    int current_step;
    int total_steps;
    
    // Movement parameters
    float stride_length;
    float step_height;
    
    // Gait state tracking
    bool leg_phase[LEG_COUNT]; // true = swing phase, false = stance phase
    
    // Internal gait functions
    void executeStandby();
    void executeTrot();
    void executeWalk();
    void executeTurnLeft();
    void executeTurnRight();
    void executeStrafe(bool left);
    
    // Step-by-step movement functions (based on tutorial sequences)
    void executeForwardStep(int step);
    void executeBackwardStep(int step);
    void executeTurnLeftStep(int step);
    void executeTurnRightStep(int step);
    
    // Utility functions
    void setLegPhases(bool ur, bool lr, bool ll, bool ul);
    void moveLegToPhase(LegId leg_id, bool swing_phase, float progress);
    Point3D getStancePosition(LegId leg_id);
    Point3D getSwingPosition(LegId leg_id, float progress);
    
public:
    Robot();
    
    // Initialization
    void begin();
    void calibrateAll();
    
    // High-level movement commands
    void standby();
    void moveForward(float speed = 0.5);
    void moveBackward(float speed = 0.5);
    void turnLeft(float speed = 0.5);
    void turnRight(float speed = 0.5);
    void strafeLeft(float speed = 0.5);
    void strafeRight(float speed = 0.5);
    void stop();
    
    // Direct gait control
    void setGait(GaitType gait);
    void setSpeed(float speed); // 0.0 to 1.0
    void setStepHeight(float height);
    void setStrideLength(float length);
    
    // Update function (call in main loop)
    void update();
    
    // Status and control
    bool isMoving() const { return is_moving; }
    GaitType getCurrentGait() const { return current_gait; }
    MovementDirection getCurrentDirection() const { return current_direction; }
    
    // Individual leg access (for advanced control)
    Leg& getLeg(LegId leg_id) { return legs[leg_id]; }
    
    // Utility
    void emergencyStop();
    void centerAll();
    
    // Debug
    void printStatus();
};