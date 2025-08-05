#include <Arduino.h>
#include "Leg.h"
#include "RobotConfig.h"

// Calibration program - use this to find correct servo offsets
// To use: rename main.cpp to main_demo.cpp and this file to main.cpp, then upload

Leg legs[LEG_COUNT] = {
    Leg(UPPER_RIGHT),
    Leg(LOWER_RIGHT), 
    Leg(LOWER_LEFT),
    Leg(UPPER_LEFT)
};

int current_leg = 0;
bool calibration_mode = true;

void printHelp() {
    Serial.println("\n🔧 SERVO CALIBRATION MODE");
    Serial.println("========================");
    Serial.println("Commands:");
    Serial.println("  'n' - Next leg");
    Serial.println("  'p' - Previous leg");
    Serial.println("  'c' - Run auto calibration for current leg");
    Serial.println("  'z' - Set current leg to zero position (90°, 90°)");
    Serial.println("  'h' - Show this help");
    Serial.println("");
    Serial.print("Current leg: ");
    Serial.println(current_leg);
    Serial.println("Watch the robot and note which angles look correct!");
    Serial.println("Update offset values in RobotConfig.h accordingly.");
    Serial.println("========================\n");
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n🕷️ SpiderBot Enhanced - Servo Calibration");
    Serial.println("========================================");
    Serial.println("Use this program to find correct servo offsets");
    Serial.println("for your specific robot mounting.\n");
    
    // Initialize all legs
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].begin();
        delay(200);
    }
    
    printHelp();
    
    // Start with all legs in zero position
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].setAnglesImmediate(JointAngles(90, 90));
    }
}

void loop() {
    // Update all legs (for smooth movements)
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].update();
    }
    
    // Check for serial commands
    if (Serial.available()) {
        char command = Serial.read();
        
        switch (command) {
            case 'n':
                current_leg = (current_leg + 1) % LEG_COUNT;
                Serial.print("Switched to leg ");
                Serial.println(current_leg);
                break;
                
            case 'p':
                current_leg = (current_leg - 1 + LEG_COUNT) % LEG_COUNT;
                Serial.print("Switched to leg ");
                Serial.println(current_leg);
                break;
                
            case 'c':
                Serial.print("Running calibration for leg ");
                Serial.println(current_leg);
                legs[current_leg].manualCalibration();
                break;
                
            case 'z':
                Serial.print("Setting leg ");
                Serial.print(current_leg);
                Serial.println(" to zero position (90°, 90°)");
                legs[current_leg].setAnglesImmediate(JointAngles(90, 90));
                break;
                
            case 'h':
                printHelp();
                break;
                
            default:
                Serial.println("Unknown command. Type 'h' for help.");
                break;
        }
    }
    
    delay(10);
}