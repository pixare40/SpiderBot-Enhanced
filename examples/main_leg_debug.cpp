#include <Arduino.h>
#include "Leg.h"
#include "RobotConfig.h"

Leg legs[LEG_COUNT] = {
    Leg(UPPER_RIGHT),
    Leg(LOWER_RIGHT), 
    Leg(LOWER_LEFT),
    Leg(UPPER_LEFT)
};

void debugSetAngles(int leg_id, float arm_angle, float paw_angle) {
    Serial.println("\n🔍 DEBUG: Setting leg " + String(leg_id) + " to (" + String(arm_angle) + "°, " + String(paw_angle) + "°)");
    
    // Show current state before setting
    JointAngles current = legs[leg_id].getCurrentAngles();
    Serial.println("  Before: (" + String(current.arm_angle) + "°, " + String(current.paw_angle) + "°)");
    
    // Set the angles
    legs[leg_id].setAnglesImmediate(JointAngles(arm_angle, paw_angle));
    
    // Show state after setting
    current = legs[leg_id].getCurrentAngles();
    Serial.println("  After:  (" + String(current.arm_angle) + "°, " + String(current.paw_angle) + "°)");
    
    delay(1000);
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n🔍 SIMPLE DEBUG TEST");
    Serial.println("===================");
    
    // Initialize all legs
    for (int i = 0; i < LEG_COUNT; i++) {
        Serial.print("Initializing leg ");
        Serial.println(i);
        legs[i].begin();
        delay(200);
    }
    
    Serial.println("✅ All legs initialized!");
    Serial.println("\nTesting each leg individually...");
    
    // Test each leg going to 60,60 (to see direction mapping effect)
    for (int i = 0; i < LEG_COUNT; i++) {
        debugSetAngles(i, 60, 60);
    }
    
    Serial.println("\nCommands:");
    Serial.println("  '0', '1', '2', '3' - Test specific leg");
    Serial.println("  'c' - Center all legs (90°)");
    Serial.println("  's' - Standby position"); 
    Serial.println("  'l' - Lift all legs");
    Serial.println("  'd' - Put down all legs");
    Serial.println("  'r' - Reset all legs");
}

void loop() {
    // Update all legs
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].update();
    }
    
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch(cmd) {
            case '0':
            case '1':
            case '2':
            case '3':
                {
                    int leg_id = cmd - '0';
                    debugSetAngles(leg_id, 60, 60);
                }
                break;
                
            case 'c':
                Serial.println("Centering all legs to 90°...");
                for (int i = 0; i < LEG_COUNT; i++) {
                    debugSetAngles(i, 90, 90);
                }
                break;
                
            case 's':
                Serial.println("Moving to standby position...");
                for (int i = 0; i < LEG_COUNT; i++) {
                    legs[i].standby();
                }
                break;
                
            case 'l':
                Serial.println("Lifting all legs...");
                for (int i = 0; i < LEG_COUNT; i++) {
                    legs[i].liftUp(25);
                }
                break;
                
            case 'd':
                Serial.println("Putting down all legs...");
                for (int i = 0; i < LEG_COUNT; i++) {
                    legs[i].putDown();
                }
                break;
                
            case 'r':
                Serial.println("Resetting all legs...");
                for (int i = 0; i < LEG_COUNT; i++) {
                    legs[i].begin();
                    delay(100);
                }
                break;
        }
    }
    
    delay(10);
}