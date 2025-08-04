# SpiderBot Enhanced 🕷️🤖

An advanced, modular robotics platform built on the ACEBOTT QD020 Quadruped Spider Robot, featuring sophisticated movement algorithms, inverse kinematics, and modern software architecture.

## 🚀 Project Overview

This project transforms the basic Arduino examples into a professional-grade robotics platform with:

- **Inverse Kinematics**: Mathematical precision for leg positioning
- **Adaptive Gaits**: Dynamic walking patterns that adapt to terrain
- **Modular Architecture**: Clean, extensible C++ design
- **Real-time Control**: WebSocket-based responsive control
- **Sensor Integration**: Ready for IMU, distance sensors, cameras
- **AI/ML Ready**: Framework for reinforcement learning

## 🔧 Hardware Specifications

- **Microcontroller**: ESP8266 (NodeMCU v2)
- **Servos**: 8x servos (2 per leg: arm + paw)
- **Connectivity**: WiFi AP mode with WebSocket support
- **Power**: External power supply recommended for servos

### Pin Configuration
```
Upper Right: Pin 14 (paw), Pin 12 (arm)
Lower Right: Pin 15 (paw), Pin 13 (arm)  
Lower Left:  Pin 2  (paw), Pin 4  (arm)
Upper Left:  Pin 16 (paw), Pin 5  (arm)
```

## 🏗️ Architecture Design

### Core Components

1. **Leg Class** (`src/Leg.cpp`)
   - Individual leg control with inverse kinematics
   - Smooth interpolated movements
   - Position and angle-based control

2. **Robot Class** (`src/Robot.cpp`)
   - Coordinated 4-leg movement
   - Gait pattern engine
   - Balance and stability control

3. **GaitEngine** (`src/GaitEngine.cpp`)
   - Walking pattern algorithms
   - Adaptive gait selection
   - Real-time gait adjustment

4. **Communication** (`src/Communication.cpp`)
   - WiFi management
   - WebSocket server
   - Command parsing and execution

5. **Sensors** (`src/Sensors.cpp`)
   - IMU integration (future)
   - Distance sensors (future)
   - Feedback systems

## 🚶‍♂️ Movement Capabilities

### Basic Movements
- Forward/Backward walking
- Left/Right turning  
- Sideways movement (crab walk)
- Stationary rotation

### Advanced Movements
- Adaptive speed control
- Obstacle avoidance (with sensors)
- Terrain adaptation
- Custom choreographed sequences
- Emergency stop and recovery

### Gait Patterns
- **Trot**: Diagonal leg pairs
- **Pace**: Lateral leg pairs  
- **Bound**: Front/rear pairs
- **Walk**: Sequential single legs
- **Custom**: User-defined patterns

## 🛠️ Development Setup

### Prerequisites
```bash
# Install PlatformIO
pip3 install platformio

# Clone and setup
git clone <repository>
cd SpiderBot-Enhanced
```

### Building and Uploading
```bash
# Build the project
pio run

# Upload to ESP8266
pio run --target upload

# Monitor serial output
pio device monitor
```

### Configuration
Edit `include/RobotConfig.h` to customize:
- Servo pin mappings
- Physical dimensions
- Movement parameters
- WiFi settings

## 📡 Control Interface

### WiFi Connection
- **SSID**: `SpiderBot-Enhanced`
- **Password**: `12345678`
- **Web Interface**: `http://192.168.4.1`
- **WebSocket**: `ws://192.168.4.1:81`

### Command Protocol
```json
{
  "command": "move",
  "direction": "forward",
  "speed": 100,
  "duration": 2000
}
```

### Available Commands
- `move`: Basic movement (forward, back, left, right)
- `turn`: Rotation (left, right)
- `gait`: Change walking pattern
- `position`: Direct leg positioning
- `sequence`: Execute choreographed movement
- `stop`: Emergency stop

## 🧮 Mathematical Models

### Inverse Kinematics
For each leg with segments L1 (arm) and L2 (paw):

```cpp
// Target position (x, y, z) -> Joint angles (θ1, θ2)
float distance = sqrt(x*x + y*y);
float angle1 = atan2(y, x);
float angle2 = acos((L1*L1 + distance*distance - L2*L2) / (2*L1*distance));
```

### Gait Coordination
Phase relationships for stable walking:
- **Trot**: Legs 1,3 vs 2,4 (180° phase)
- **Walk**: Sequential 90° phase offsets
- **Bound**: Front vs rear pairs (180° phase)

## 🎯 Roadmap

### Phase 1: Core Architecture ✅
- [x] Project setup with PlatformIO
- [x] Modular class design
- [ ] Basic leg control implementation
- [ ] Inverse kinematics integration

### Phase 2: Advanced Movement
- [ ] Gait engine implementation
- [ ] Smooth movement interpolation
- [ ] Balance and stability control
- [ ] Adaptive speed control

### Phase 3: Smart Features
- [ ] IMU integration for balance
- [ ] Distance sensors for obstacles
- [ ] Machine learning for gait optimization
- [ ] Voice control integration

### Phase 4: Ecosystem
- [ ] Mobile app development
- [ ] Cloud telemetry
- [ ] Multi-robot coordination
- [ ] Computer vision integration

## 🔬 Advanced Features (Future)

### Sensor Integration
- **IMU**: Real-time balance and orientation
- **Ultrasonic**: Obstacle detection and avoidance
- **Camera**: Computer vision and navigation
- **Force**: Ground contact feedback

### AI/ML Capabilities
- **Reinforcement Learning**: Optimal gait discovery
- **Neural Networks**: Terrain classification
- **Computer Vision**: Object recognition and tracking
- **Predictive Control**: Anticipatory movement

## 🤝 Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ACEBOTT for the original QD020 hardware design
- PlatformIO for the excellent development platform
- Arduino community for extensive library support

---

**Ready to make this spider bot extraordinary!** 🚀

For questions and support, check the [Issues](../../issues) page or start a [Discussion](../../discussions).