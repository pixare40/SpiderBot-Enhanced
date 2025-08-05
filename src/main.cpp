#include <Arduino.h>
#include "Robot.h"
#include "RobotConfig.h"

Robot robot;

void printMenu() {
    Serial.println("\n🕷️ SPIDERBOT ROBOT CONTROL");
    Serial.println("===========================");
    Serial.println("Movement Commands:");
    Serial.println("  'w' - Move forward");
    Serial.println("  's' - Move backward");
    Serial.println("  'a' - Turn left");
    Serial.println("  'd' - Turn right");
    Serial.println("  'x' - Stop");
    Serial.println("  'e' - EMERGENCY STOP");
    Serial.println("");
    Serial.println("Control Commands:");
    Serial.println("  'c' - Center all legs");
    Serial.println("  'b' - Standby position");
    Serial.println("  'r' - Calibrate all legs");
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
                Serial.println("🦵 Testing Leg 0 - Extreme movement");
                robot.legs[0].setAnglesImmediate(JointAngles(90, 170));
                delay(1000);
                robot.legs[0].setAnglesImmediate(JointAngles(90, 10));
                break;
                
            case '9':
                Serial.println("🦵 Testing Leg 1 - Extreme movement");
                robot.legs[1].setAnglesImmediate(JointAngles(90, 170));
                delay(1000);
                robot.legs[1].setAnglesImmediate(JointAngles(90, 10));
                break;
                
            default:
                Serial.println("Unknown command. Type 'h' for help.");
                break;
        }
    }
    
    delay(10);
}