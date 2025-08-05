#include <Arduino.h>
#include "Leg.h"
#include "RobotConfig.h"

// Comprehensive Leg class testing program
Leg legs[LEG_COUNT] = {
    Leg(UPPER_RIGHT),
    Leg(LOWER_RIGHT), 
    Leg(LOWER_LEFT),
    Leg(UPPER_LEFT)
};

int current_leg = 0;
int test_phase = 0;

void printMenu() {
    Serial.println("\n🦵 LEG CLASS TESTING SUITE");
    Serial.println("==========================");
    Serial.println("Basic Commands:");
    Serial.println("  'n' - Next leg (current: " + String(current_leg) + ")");
    Serial.println("  'a' - Test all legs");
    Serial.println("");
    Serial.println("Movement Tests:");
    Serial.println("  'c' - Center position (90°, 90°)");
    Serial.println("  's' - Standby position");
    Serial.println("  'l' - Lift up");
    Serial.println("  'd' - Put down");
    Serial.println("");
    Serial.println("Advanced Tests:");
    Serial.println("  'i' - Test inverse kinematics");
    Serial.println("  't' - Test smooth timing");
    Serial.println("  'p' - Print leg status");
    Serial.println("  'r' - Run full test sequence");
    Serial.println("  'h' - Show this menu");
    Serial.println("==========================");
}

void testBasicMovements(int leg_id) {
    Serial.println("🔄 Testing basic movements for leg " + String(leg_id));
    
    Serial.println("  → Center position");
    legs[leg_id].center();
    delay(1000);
    
    Serial.println("  → Standby position");
    legs[leg_id].standby();
    delay(1000);
    
    Serial.println("  → Lift up");
    legs[leg_id].liftUp(25);
    delay(1000);
    
    Serial.println("  → Put down");
    legs[leg_id].putDown();
    delay(1000);
    
    Serial.println("✅ Basic movements complete for leg " + String(leg_id));
}

void testInverseKinematics(int leg_id) {
    Serial.println("🧮 Testing inverse kinematics for leg " + String(leg_id));
    
    // Test reachable positions
    Point3D test_positions[] = {
        Point3D(60, 0, -30),   // Forward, down
        Point3D(40, 0, -20),   // Back, up  
        Point3D(50, 0, -40),   // Forward, low
        Point3D(70, 0, -25)    // Far forward
    };
    
    for (int i = 0; i < 4; i++) {
        Serial.print("  → Moving to position ");
        Serial.print(i);
        Serial.print(" (");
        Serial.print(test_positions[i].x);
        Serial.print(", ");
        Serial.print(test_positions[i].y); 
        Serial.print(", ");
        Serial.print(test_positions[i].z);
        Serial.println(")");
        
        bool success = legs[leg_id].moveTo(test_positions[i], 800);
        if (!success) {
            Serial.println("  ❌ Position unreachable");
        }
        delay(1200);
    }
    
    Serial.println("  → Return to center");
    legs[leg_id].center();
    delay(800);
    
    Serial.println("✅ Inverse kinematics test complete");
}

void testSmoothTiming(int leg_id) {
    Serial.println("⏱️ Testing smooth interpolation for leg " + String(leg_id));
    
    Serial.println("  → Fast movement (200ms)");
    legs[leg_id].setAngles(JointAngles(60, 60), 200);
    delay(400);
    
    Serial.println("  → Slow movement (1000ms)");
    legs[leg_id].setAngles(JointAngles(120, 120), 1000);
    delay(1200);
    
    Serial.println("  → Medium movement (500ms)");
    legs[leg_id].setAngles(JointAngles(90, 90), 500);
    delay(700);
    
    Serial.println("✅ Timing test complete");
}

void runFullSequence(int leg_id) {
    Serial.println("🎬 Running full test sequence for leg " + String(leg_id));
    
    testBasicMovements(leg_id);
    delay(500);
    testInverseKinematics(leg_id);
    delay(500);
    testSmoothTiming(leg_id);
    
    Serial.println("🎉 Full sequence complete for leg " + String(leg_id));
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n🕷️ SpiderBot Enhanced - Leg Class Testing");
    Serial.println("=========================================");
    
    // Initialize all legs
    for (int i = 0; i < LEG_COUNT; i++) {
        Serial.print("Initializing leg ");
        Serial.println(i);
        legs[i].begin();
        delay(200);
    }
    
    Serial.println("✅ All legs initialized!");
    
    // Set all to center position
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].center();
    }
    
    printMenu();
}

void loop() {
    // Update all legs (for smooth movements)
    for (int i = 0; i < LEG_COUNT; i++) {
        legs[i].update();
    }
    
    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch(cmd) {
            case 'n': // Next leg
                current_leg = (current_leg + 1) % LEG_COUNT;
                Serial.println("Switched to leg " + String(current_leg));
                break;
                
            case 'c': // Center
                Serial.println("Center position for leg " + String(current_leg));
                legs[current_leg].center();
                break;
                
            case 's': // Standby
                Serial.println("Standby position for leg " + String(current_leg));
                legs[current_leg].standby();
                break;
                
            case 'l': // Lift
                Serial.println("Lift leg " + String(current_leg));
                legs[current_leg].liftUp(25);
                break;
                
            case 'd': // Down
                Serial.println("Put down leg " + String(current_leg));
                legs[current_leg].putDown();
                break;
                
            case 'i': // Inverse kinematics test
                testInverseKinematics(current_leg);
                break;
                
            case 't': // Timing test
                testSmoothTiming(current_leg);
                break;
                
            case 'p': // Print status
                legs[current_leg].printStatus();
                break;
                
            case 'r': // Run full sequence
                runFullSequence(current_leg);
                break;
                
            case 'a': // Test all legs
                Serial.println("🔄 Testing all legs...");
                for (int i = 0; i < LEG_COUNT; i++) {
                    testBasicMovements(i);
                    delay(500);
                }
                Serial.println("✅ All legs tested!");
                break;
                
            case 'h': // Help
                printMenu();
                break;
                
            default:
                Serial.println("Unknown command. Type 'h' for help.");
                break;
        }
    }
    
    delay(10);
}