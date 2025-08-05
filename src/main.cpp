#include <Arduino.h>
#include "Robot.h"
#include "RobotConfig.h"

Robot robot;

void printMenu() {
    Serial.println("\n🕷️ SPIDERBOT ROBOT CONTROL");
    Serial.println("===========================");
    Serial.println("Movement Commands:");
    Serial.println("  'w' - Move forward (tutorial sequence)");
    Serial.println("  's' - Move backward (tutorial sequence)");
    Serial.println("  'a' - Turn left (tutorial sequence)");
    Serial.println("  'd' - Turn right (tutorial sequence)");
    Serial.println("  'x' - Stop");
    Serial.println("  'e' - EMERGENCY STOP");
    Serial.println("");
    Serial.println("Control Commands:");
    Serial.println("  'c' - Center all legs (standby position)");
    Serial.println("  'm' - Center all servos to 90°,90° (mechanical center)");
    Serial.println("  'b' - Standby position");
    Serial.println("  'r' - Calibrate all legs");
    Serial.println("  'f' - FORCE RESET to proper standby");
    Serial.println("  't' - TEST damaged servo (gentle movements)");
    Serial.println("  'q' - SINGLE STEP forward test");
    Serial.println("  'p' - Print robot status");
    Serial.println("  'h' - Show this menu");
    Serial.println("");
    Serial.println("Speed Commands:");
    Serial.println("  '1' - Slow speed (0.3)");
    Serial.println("  '2' - Medium speed (0.6)");
    Serial.println("  '3' - Fast speed (0.9)");
    Serial.println("===========================");
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n🕷️ SpiderBot Enhanced - Robot Demo");
    Serial.println("==================================");
    
    // Initialize robot
    robot.begin();
    
    printMenu();
}

void loop() {
    // Update robot (handles gait execution and leg coordination)
    robot.update();
    
    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch(cmd) {
            // Movement commands
            case 'w':
                Serial.println("🚶 Moving forward...");
                robot.moveForward();
                break;
                
            case 's':
                Serial.println("🔙 Moving backward...");
                robot.moveBackward();
                break;
                
            case 'a':
                Serial.println("↺ Turning left...");
                robot.turnLeft();
                break;
                
            case 'd':
                Serial.println("↻ Turning right...");
                robot.turnRight();
                break;
                
            case 'x':
                Serial.println("⏹️ Stopping...");
                robot.stop();
                break;
                
            case 'e':
                Serial.println("🚨 EMERGENCY STOP!");
                robot.emergencyStop();
                break;
                
            // Control commands
            case 'c':
                Serial.println("🎯 Centering all legs...");
                robot.centerAll();
                break;
                
            case 'm':
                Serial.println("🔧 MECHANICAL CENTER - All servos to 90°,90°");
                robot.emergencyStop(); // Stop any movement first
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 90));
                robot.getLeg(LOWER_RIGHT).setAnglesImmediate(JointAngles(90, 90));
                robot.getLeg(LOWER_LEFT).setAnglesImmediate(JointAngles(90, 90));
                robot.getLeg(UPPER_LEFT).setAnglesImmediate(JointAngles(90, 90));
                Serial.println("All servos set to 90°,90° - Check leg positions");
                break;
                
            case 'b':
                Serial.println("🦵 Standby position...");
                robot.standby();
                break;
                
            case 'r':
                Serial.println("🔧 Calibrating all legs...");
                robot.calibrateAll();
                break;
                
            case 'p':
                robot.printStatus();
                break;
                
            // Speed commands
            case '1':
                Serial.println("🐌 Slow speed");
                robot.setSpeed(0.3);
                break;
                
            case '2':
                Serial.println("🚶 Medium speed");
                robot.setSpeed(0.6);
                break;
                
            case '3':
                Serial.println("🏃 Fast speed");
                robot.setSpeed(0.9);
                break;
                
            case 'h':
                printMenu();
                break;
                
            // Individual leg tests
            case '0':
                Serial.println("🦵 Testing UPPER_RIGHT leg - SAFE movement test");
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 140));
                delay(1000);
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 40));
                delay(1000);
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 90));
                break;
                
            case '9':
                Serial.println("🦵 Testing LOWER_RIGHT leg - SAFE movement test");
                robot.getLeg(LOWER_RIGHT).setAnglesImmediate(JointAngles(90, 140));
                delay(1000);
                robot.getLeg(LOWER_RIGHT).setAnglesImmediate(JointAngles(90, 40));
                delay(1000);
                robot.getLeg(LOWER_RIGHT).setAnglesImmediate(JointAngles(90, 90));
                break;
                
            case 'f':
                Serial.println("🔧 FORCE RESET - All legs to proper standby");
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 60));
                robot.getLeg(LOWER_RIGHT).setAnglesImmediate(JointAngles(90, 120));
                robot.getLeg(LOWER_LEFT).setAnglesImmediate(JointAngles(90, 60));
                robot.getLeg(UPPER_LEFT).setAnglesImmediate(JointAngles(90, 120));
                break;
                
            case 't':
                Serial.println("🔧 SERVO TEST - Testing individual servos");
                Serial.println("Testing UPPER_RIGHT paw servo (pin 14)...");
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 90));
                delay(1000);
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 100));
                delay(1000);
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 80));
                delay(1000);
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 90));
                Serial.println("Did the paw move? Check for any movement or sounds.");
                break;
                
            case 'q':
                Serial.println("🦵 SINGLE STEP FORWARD TEST - Using tutorial sequence");
                robot.emergencyStop(); // Stop any movement first
                delay(200);
                
                // Execute first 3 steps of forward sequence manually for testing
                Serial.println("Step 0 - Standby position");
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 70));  // G12=90°, G14=70°
                robot.getLeg(LOWER_RIGHT).setAnglesImmediate(JointAngles(90, 110)); // G13=90°, G15=110°
                robot.getLeg(LOWER_LEFT).setAnglesImmediate(JointAngles(90, 70));   // G4=90°, G2=70°
                robot.getLeg(UPPER_LEFT).setAnglesImmediate(JointAngles(90, 110));  // G5=90°, G16=110°
                delay(1500);
                
                Serial.println("Step 1 - Lift right upper & left lower paws");
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(90, 90));  // G12=90°, G14=90°
                robot.getLeg(LOWER_LEFT).setAnglesImmediate(JointAngles(90, 90));   // G4=90°, G2=90°
                delay(1500);
                
                Serial.println("Step 2 - Move arms forward/back");
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(120, 90)); // G12=120°, G14=90°
                robot.getLeg(LOWER_LEFT).setAnglesImmediate(JointAngles(60, 90));   // G4=60°, G2=90°
                delay(1500);
                
                Serial.println("Step 3 - Put paws down");
                robot.getLeg(UPPER_RIGHT).setAnglesImmediate(JointAngles(120, 70)); // G12=120°, G14=70°
                robot.getLeg(LOWER_LEFT).setAnglesImmediate(JointAngles(60, 70));   // G4=60°, G2=70°
                delay(1500);
                
                Serial.println("Returning to standby...");
                robot.standby();
                break;
                
            default:
                Serial.println("Unknown command. Type 'h' for help.");
                break;
        }
    }
    
    delay(10);
}