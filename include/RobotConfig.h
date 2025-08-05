#pragma once

// Hardware Configuration
#define SERVO_COUNT 8
#define LEG_COUNT 4

// Servo Pin Mapping (matches original hardware)
enum ServoPin {
    UPPER_RIGHT_PAW = 14,   // Upper right paw
    UPPER_RIGHT_ARM = 12,   // Upper right arm  
    LOWER_RIGHT_ARM = 13,   // Lower right arm
    LOWER_RIGHT_PAW = 15,   // Lower right paw
    UPPER_LEFT_PAW = 16,    // Upper left paw
    UPPER_LEFT_ARM = 5,     // Upper left arm
    LOWER_LEFT_ARM = 4,     // Lower left arm
    LOWER_LEFT_PAW = 2      // Lower left paw
};

// Servo Limits
#define SERVO_MIN_PULSE 400
#define SERVO_MAX_PULSE 2400
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Servo Calibration Offsets (adjust these values for your specific robot)
// Positive values rotate servo clockwise, negative counter-clockwise
#define UPPER_RIGHT_ARM_OFFSET 0   // Adjust if arm doesn't point forward at 90°
#define UPPER_RIGHT_PAW_OFFSET 0   // Adjust if paw doesn't point down at 90°
#define LOWER_RIGHT_ARM_OFFSET 0   
#define LOWER_RIGHT_PAW_OFFSET 0   
#define LOWER_LEFT_ARM_OFFSET 0    
#define LOWER_LEFT_PAW_OFFSET 0    
#define UPPER_LEFT_ARM_OFFSET 0    
#define UPPER_LEFT_PAW_OFFSET 0

// Physical dimensions (mm) - adjust based on your robot
#define LEG_SEGMENT_1_LENGTH 40  // Upper arm length
#define LEG_SEGMENT_2_LENGTH 60  // Lower arm + paw length
#define BODY_WIDTH 80            // Distance between left and right legs
#define BODY_LENGTH 100          // Distance between front and rear legs

// Movement Parameters
#define DEFAULT_STEP_HEIGHT 20   // mm
#define DEFAULT_STRIDE_LENGTH 30 // mm
#define DEFAULT_GAIT_SPEED 200   // ms per step
#define SERVO_INTERPOLATION_STEPS 20

// WiFi Configuration
#define WIFI_AP_SSID "SpiderBot-Enhanced"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_CHANNEL 5
#define WIFI_SERVER_PORT 80
#define WEBSOCKET_PORT 81

// Debug and Telemetry
#ifdef DEBUG_ENABLED
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
#endif

// Leg IDs for easy reference
enum LegId {
    UPPER_RIGHT = 0,
    LOWER_RIGHT = 1, 
    LOWER_LEFT = 2,
    UPPER_LEFT = 3
};

// Joint IDs within each leg
enum JointId {
    ARM = 0,  // Upper joint (closer to body)
    PAW = 1   // Lower joint (foot)
};