# NGV DCAS - Motion Sensor Simulation

## Overview
Educational motion sensor emulator featuring **steering wheel angle** and **wheel speed pulse** sensors with realistic noise modeling and digital signal generation for autonomous vehicle development.

## Sensor Motion Node

### Description
The `sensor_motion_node.py` implements a comprehensive motion sensor simulator featuring:
- Steering wheel angle sensor with Gaussian noise modeling
- Wheel speed sensor (WSS) with digital pulse generation
- Vehicle dynamics-based sensor signal synthesis
- Educational TODO exercises for automotive sensor understanding

### Key Features

#### ✨ Motion Sensor Simulation
- **Steering Angle Sensor**: ±720° range with configurable noise characteristics
- **Wheel Speed Sensor**: Pulse-per-revolution digital signal generation
- **Real-time Output**: High-frequency sensor data at 50 Hz (configurable)
- **Synthetic Signals**: Fallback signal generation when vehicle state unavailable

#### 🎯 Physics-Based Signal Generation
- **WSS Pulse Logic**: Encoder-based digital pulse generation from wheel rotation
- **Speed-to-Pulse Conversion**: Linear velocity to angular position mapping
- **Quantization Effects**: Digital signal discretization simulation
- **Noise Modeling**: Gaussian noise for steering angle measurements

#### 🔧 Educational Framework
- **TODO Exercises**: Student implementation of sensor signal processing
- **Sensor Physics**: Understanding encoder principles and signal characteristics
- **Parameter Learning**: Automotive sensor specifications and calibration
- **Signal Processing**: Digital pulse generation and noise analysis

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/motion/steering_wheel_angle` | `std_msgs/Float32` | Steering wheel angle in degrees |
| `/sensors/motion/wheel_speed_pulses` | `std_msgs/Bool` | Digital wheel speed pulse signal |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vehicle/state` | `VehicleState` | Vehicle dynamics for sensor signal generation |

### Message Formats

#### Steering Wheel Angle
```
std_msgs/Float32
  float32 data    # Steering wheel angle [degrees]
                  # Range: ±720° (typical automotive range)
                  # Positive: left turn, Negative: right turn
```

#### Wheel Speed Pulses
```
std_msgs/Bool
  bool data       # Digital pulse signal
                  # true: pulse high, false: pulse low
                  # Frequency proportional to wheel speed
```

## Configuration

### Sensor Update Parameters
```yaml
sensor_motion_node:
  # Basic sensor configuration
  enabled: true                  # Enable/disable motion sensors
  period_s: 0.02                # Update frequency: 50 Hz (0.02s)
```

### Vehicle Parameters
```yaml
vehicle_node:
  # Physical vehicle parameters for pulse calculation
  wheel_radius: 0.3             # Wheel radius [meters]
  pulses_per_revolution: 20     # WSS pulses per wheel revolution
```

### Noise Model Parameters
```yaml
sensor_motion_node:
  # Steering angle sensor noise
  enable_noise: true            # Enable measurement noise
  angle_noise_std: 0.01         # Steering angle noise std dev [degrees]
```

## Technical Implementation

### TODO Exercise Framework

The motion sensor simulator includes educational exercises for students:

#### 1. Vehicle State Extraction
```python
# TODO: Extract steering angle from vehicle state
# Extract steering wheel angle from vehicle dynamics
angle = 10.0 * math.sin(self.t)  # Student implementation needed

# TODO: Extract vehicle speed for pulse generation
# Extract forward velocity from vehicle state
base_speed = max(0.0, 0.0)  # Student implementation needed
```

#### 2. Wheel Speed Sensor Implementation
```python
def _generate_pulse_logic(self, dt, is_synthetic):
    """Generate digital pulses based on wheel rotation"""
    
    # TODO: Calculate position change from velocity
    delta_position = 0.0  # Student implementation
    
    # TODO: Convert linear distance to wheel angle
    wheel_angle = 0.0  # θ = s / r (radians)
    
    # TODO: Calculate cumulative wheel rotation angle
    curr_angle = 0.0  # Student implementation
    
    # TODO: Normalize angle to [0, 2π] range
    curr_angle = np.remainder(0.0, 2 * math.pi)
    
    # TODO: Calculate pulse position in cycle
    position_in_pulse_cycle = 0.0  # Student implementation
    
    # TODO: Quantize continuous position to discrete pulse
    quant_position = np.round(0.0)  # Student implementation
    
    # TODO: Generate pulse signal (0 or 1)
    pulse = 0.0  # Student implementation
```

#### 3. Noise Application
```python
def _apply_noise(self, angle):
    """Apply Gaussian noise to steering angle measurement"""
    
    # TODO: Generate Gaussian noise
    noisy = self.rng.normal(0.0, 0.0)  # Student implementation
    
    # Apply noise to measurement
    angle_noisy = angle + noisy
    return angle_noisy
```

### Steering Angle Sensor Physics

Automotive steering angle sensors typically use contactless position sensing:

```
Measured Angle = True Angle + Noise

Where:
True Angle = Actual steering wheel rotation [degrees]
Noise = Gaussian noise ~ N(0, σ²_angle)

Sensor Range: Typically ±720° (two full turns)
Resolution: 0.1° to 1.0° depending on sensor grade
Update Rate: 50-100 Hz for automotive applications
```

**Physical Implementation:**
- **Hall Effect Sensors**: Magnetic field variation with rotation
- **Optical Encoders**: Light interruption by rotating disc patterns  
- **Potentiometric**: Resistance change with angular position
- **Resolver**: AC-excited rotational transformer

### Wheel Speed Sensor (WSS) Physics

WSS generates digital pulses proportional to wheel rotation speed:

```
Pulse Generation Process:
1. Linear velocity (v) → Angular velocity (ω = v/r)
2. Angular velocity → Cumulative rotation angle (θ)
3. Rotation angle → Pulse position in cycle
4. Continuous position → Quantized pulse signal

Mathematical Model:
θ(t) = ∫ ω(t) dt = ∫ (v(t)/r) dt
pulse_position = (θ / 2π) × 2 × pulses_per_rev
pulse_signal = round(pulse_position) mod 2
```

**Physical Implementation:**
- **Magnetic Ring**: Magnetized segments trigger Hall sensors
- **Toothed Wheel**: Ferromagnetic teeth change magnetic field
- **Optical Encoder**: Light/dark pattern interrupts photodiode
- **Variable Reluctance**: Magnetic flux variation with gear teeth

### Signal Processing Characteristics

#### Pulse Frequency Analysis
```python
# Relationship between vehicle speed and pulse frequency
f_pulse = (v × pulses_per_rev) / (2π × r)

Where:
f_pulse = Pulse frequency [Hz]
v = Vehicle speed [m/s]
pulses_per_rev = Encoder resolution [pulses/revolution]
r = Wheel radius [m]

# Example: v=10 m/s, r=0.3m, pulses_per_rev=20
f_pulse = (10 × 20) / (2π × 0.3) = 106.1 Hz
```

#### Speed Resolution and Accuracy
```python
# Minimum detectable speed change
Δv_min = (2π × r) / (pulses_per_rev × Δt)

Where:
Δt = Measurement time window [s]

# Speed measurement accuracy depends on:
# 1. Encoder resolution (pulses per revolution)
# 2. Measurement time window
# 3. Wheel radius accuracy
# 4. Tire slip effects
```

### Digital Signal Characteristics

#### Pulse Pattern Analysis
```
Wheel Speed Sensor Output:
Time:     0    0.1   0.2   0.3   0.4   0.5   [s]
Pulse:    0     1     0     1     0     1    [bool]
          ↑     ↑     ↑     ↑     ↑     ↑
          Low  High  Low  High  Low  High

Frequency = 5 Hz → Speed calculation possible
Duty Cycle = 50% (ideal square wave)
```

#### Quantization Effects
```python
# Continuous rotation angle discretized to pulse positions
continuous_angle = 45.7°    # Actual wheel rotation
pulse_positions = 2.54      # Pulse cycle position
quantized_pulse = 3         # Nearest integer pulse
quantization_error = 0.46   # Error in pulse position

# This introduces speed measurement uncertainty
# Especially significant at low speeds
```

## Usage Examples

### High-Resolution WSS
```yaml
# Racing/research grade wheel speed sensor
wheel_radius: 0.305           # Precise wheel radius measurement
pulses_per_revolution: 60     # High resolution encoder
angle_noise_std: 0.001        # Very low noise steering sensor
```

### Automotive Grade Sensors
```yaml
# Production vehicle motion sensors
wheel_radius: 0.3             # Standard tire radius
pulses_per_revolution: 20     # Typical automotive WSS
angle_noise_std: 0.01         # Standard steering sensor noise
```

### Low-Cost Configuration
```yaml
# Budget/aftermarket sensors
wheel_radius: 0.3             # Nominal wheel radius
pulses_per_revolution: 10     # Lower resolution encoder
angle_noise_std: 0.1          # Higher noise levels
```

### Debug Mode (Perfect Sensors)
```yaml
# Noise-free measurements for testing
enable_noise: false           # No steering angle noise
# WSS inherently noise-free (digital signal)
```

## Educational Objectives

### 1. Automotive Sensor Fundamentals
- **Steering Sensors**: Understanding position measurement principles
- **Wheel Speed Sensors**: Learning rotational encoder technology
- **Signal Types**: Analog vs digital sensor output characteristics
- **Sensor Integration**: How motion sensors support vehicle systems

### 2. Signal Processing Concepts
- **Digital Pulse Generation**: Continuous-to-discrete signal conversion
- **Quantization**: Understanding discretization effects and resolution
- **Noise Modeling**: Gaussian noise characteristics in real sensors
- **Frequency Analysis**: Relating physical motion to signal frequency

### 3. Vehicle Dynamics Integration
- **Steering Kinematics**: Wheel angle to vehicle path relationship
- **Speed Measurement**: Wheel rotation to vehicle velocity conversion
- **Coordinate Frames**: Sensor mounting and reference frame considerations
- **Calibration**: Sensor offset and scale factor determination

### 4. Control System Applications
- **Feedback Sensors**: Motion sensors in closed-loop control
- **State Estimation**: Using motion sensors for vehicle state observation
- **Safety Systems**: WSS applications in ABS, ESC, and traction control
- **Autonomous Driving**: Motion sensor role in localization and control

## Implementation Exercises

### Exercise 1: Vehicle State to Sensor Conversion
**Objective**: Extract motion sensor signals from vehicle dynamics

**Tasks**:
- Extract steering wheel angle from vehicle state
- Convert vehicle forward velocity to wheel rotation
- Handle coordinate frame transformations
- Understand sensor mounting conventions

### Exercise 2: Wheel Speed Sensor Implementation
**Objective**: Implement complete WSS pulse generation logic

**Tasks**:
- Calculate wheel rotation from linear motion
- Implement cumulative angle tracking
- Generate quantized digital pulse signal
- Handle angle normalization and wraparound

### Exercise 3: Pulse Frequency Analysis
**Objective**: Understand speed-to-frequency relationship

**Tasks**:
- Calculate expected pulse frequency from vehicle speed
- Implement pulse counting for speed measurement
- Analyze quantization effects at low speeds
- Understand resolution limitations

### Exercise 4: Sensor Noise Modeling
**Objective**: Apply realistic sensor noise characteristics

**Tasks**:
- Generate Gaussian noise with specified parameters
- Understand noise impact on control systems
- Implement noise filtering concepts
- Compare clean vs noisy sensor performance

## Performance Characteristics

- **Update Rate**: 50 Hz (typical automotive sensor frequency)
- **Computational Load**: Very low (simple arithmetic operations)
- **Memory Usage**: Minimal state storage (previous angle/position)
- **Real-time Performance**: Suitable for real-time control applications

## Sensor Applications in Vehicle Systems

### Anti-lock Braking System (ABS)
```python
# WSS provides wheel speed for slip detection
wheel_slip_ratio = (v_vehicle - v_wheel) / v_vehicle
if wheel_slip_ratio > threshold:
    apply_brake_modulation()
```

### Electronic Stability Control (ESC)
```python
# Steering angle sensor provides driver intent
steering_intent = steering_wheel_angle / steering_ratio
yaw_rate_intent = f(steering_intent, vehicle_speed)
if abs(yaw_rate_actual - yaw_rate_intent) > threshold:
    apply_corrective_braking()
```

### Autonomous Vehicle Localization
```python
# Dead reckoning using wheel speed integration
distance_traveled += wheel_speed * dt
heading_change = (steering_angle / steering_ratio) * distance_traveled / wheelbase
```

## Troubleshooting

### Common Issues
1. **TODO Not Implemented**: Functions return default/zero values
2. **Pulse Generation Errors**: Incorrect angle calculation or normalization
3. **Speed Calculation Issues**: Wrong wheel radius or pulse resolution
4. **Coordinate Frame Confusion**: Mixing sensor and vehicle reference frames

### Debugging Tips
- Monitor pulse frequency vs expected speed relationship
- Check angle normalization for proper [0, 2π] range
- Verify wheel radius units (meters vs millimeters)
- Compare synthetic vs real vehicle state signals
- Use oscilloscope-like tools to visualize pulse patterns

## Motion Sensors vs Other Sensors Comparison

| Sensor | Update Rate | Signal Type | Measurement | Drift | Cost |
|--------|-------------|-------------|-------------|-------|------|
| **Steering Angle** | 50-100 Hz | Analog | Position | Low | Low |
| **Wheel Speed** | 50-100 Hz | Digital | Velocity | None | Very Low |
| **IMU** | 100-1000 Hz | Analog | Acceleration/Rate | High | Medium |
| **GPS** | 1-10 Hz | Digital | Position | Low | Medium |
| **LiDAR** | 10 Hz | Digital | Distance | None | High |

## Automotive Standards and Specifications

### Typical Sensor Specifications

| Parameter | Steering Angle Sensor | Wheel Speed Sensor |
|-----------|----------------------|-------------------|
| **Range** | ±720° to ±1440° | 0-300 km/h |
| **Resolution** | 0.1° to 1.0° | 0.1 km/h |
| **Accuracy** | ±1° to ±3° | ±1% to ±3% |
| **Update Rate** | 50-100 Hz | 50-100 Hz |
| **Operating Temp** | -40°C to +125°C | -40°C to +125°C |
| **Supply Voltage** | 5V or 12V | 5V or 12V |

### Industry Standards
- **ISO 26262**: Functional safety for automotive systems
- **SAE J1939**: CAN bus communication standards
- **ISO 11898**: Controller Area Network (CAN) specification
- **SAE J2012**: Diagnostic trouble codes (DTC) definitions

---

*This motion sensor simulation provides comprehensive educational experience in automotive sensor technology, signal processing, and vehicle dynamics integration for autonomous vehicle development.*