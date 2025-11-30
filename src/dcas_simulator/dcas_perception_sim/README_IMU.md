# NGV DCAS - IMU Sensor Simulation

## Overview
Educational IMU (Inertial Measurement Unit) sensor emulator featuring **physics-based inertial measurements** with realistic noise modeling and bias drift simulation for autonomous vehicle development.

## Sensor IMU Node

### Description
The `sensor_imu_node.py` implements a comprehensive IMU sensor simulator featuring:
- 3-axis gyroscope and accelerometer simulation with bias drift
- Quaternion-based orientation measurement with rotational noise
- Random walk bias modeling for realistic sensor characteristics
- Educational TODO exercises for sensor fusion understanding

### Key Features

#### ✨ Inertial Measurement Simulation
- **Orientation**: Quaternion-based attitude measurement with rotational noise
- **Angular Velocity**: 3-axis gyroscope with bias drift and Gaussian noise
- **Linear Acceleration**: 3-axis accelerometer with bias and noise modeling
- **Real-time Output**: High-frequency IMU data at 50 Hz (configurable)

#### 🎯 Physics-Based Sensor Modeling
- **Bias Drift**: Random walk process for realistic sensor characteristics
- **Gaussian Noise**: Thermal and quantization noise simulation
- **Covariance Matrices**: Uncertainty quantification for sensor fusion
- **Physical Limits**: Bias saturation and sensor range constraints

#### 🔧 Educational Framework
- **TODO Exercises**: Student implementation of noise and bias models
- **Sensor Fusion Ready**: Standard ROS sensor_msgs/Imu format
- **Parameter Learning**: Understanding IMU specifications and characteristics
- **Noise Modeling**: Hands-on experience with stochastic sensor models

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/imu/data` | `sensor_msgs/Imu` | Complete IMU measurements with covariance |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vehicle/state` | `VehicleState` | Vehicle dynamics for ground truth IMU data |

### IMU Message Structure

Each IMU measurement contains:
```
Header header                    # Timestamp and coordinate frame
Quaternion orientation          # Vehicle attitude (roll, pitch, yaw)
float64[9] orientation_covariance
Vector3 angular_velocity        # Body-frame rotation rates (rad/s)
float64[9] angular_velocity_covariance
Vector3 linear_acceleration     # Body-frame accelerations (m/s²)
float64[9] linear_acceleration_covariance
```

## Configuration

### Sensor Update Parameters
```yaml
sensor_imu_node:
  # Basic sensor configuration
  enabled: true                  # Enable/disable IMU sensor
  period_s: 0.02                # Update frequency: 50 Hz (0.02s)
  gravity: 9.81                 # Gravitational acceleration [m/s²]
```

### Noise Model Parameters
```yaml
  # Gaussian noise characteristics
  enable_gaussian_noise: true   # Enable measurement noise
  orientation_noise_std: 0.01   # Attitude noise std dev [rad]
  angular_velocity_noise_std: 0.01  # Gyro noise std dev [rad/s]
  linear_acceleration_noise_std: 0.01  # Accelerometer noise std dev [m/s²]
```

### Bias Drift Parameters
```yaml
  # Random walk bias modeling
  enable_bias: true             # Enable bias drift simulation
  gyro_bias_rw_std: 0.001      # Gyro bias random walk [rad/s/√s]
  accel_bias_rw_std: 0.02      # Accel bias random walk [m/s²/√s]
  
  # Bias saturation limits
  enable_bias_limits: true      # Enable bias magnitude limits
  gyro_bias_limit: 0.005       # Maximum gyro bias [rad/s]
  accel_bias_limit: 0.05       # Maximum accel bias [m/s²]
```

## Technical Implementation

### TODO Exercise Framework

The IMU simulator includes educational exercises for students:

#### 1. Vehicle State Extraction
```python
# TODO: Extract IMU data from vehicle state
# Extract orientation (quaternion) from vehicle state
msg.orientation = Quaternion(0, 0, 0, 1)  # Student implementation

# Extract angular velocity from vehicle dynamics
msg.angular_velocity.x = 0.0  # Roll rate
msg.angular_velocity.y = 0.0  # Pitch rate  
msg.angular_velocity.z = 0.0  # Yaw rate (from vehicle state)

# Extract linear acceleration from vehicle motion
msg.linear_acceleration.x = 0.0  # Longitudinal acceleration
msg.linear_acceleration.y = 0.0  # Lateral acceleration
msg.linear_acceleration.z = 0.0  # Vertical acceleration (gravity + motion)
```

#### 2. Noise Application
```python
def _apply_noise(self, msg):
    """Apply realistic sensor noise to IMU measurements"""
    # TODO: Add bias and Gaussian noise to measurements
    # Angular velocity noise: measurement = true_value + bias + noise
    angular_velocity_x_noisy = msg.angular_velocity.x + 0.0  # Student implementation
    angular_velocity_y_noisy = msg.angular_velocity.y + 0.0
    angular_velocity_z_noisy = msg.angular_velocity.z + 0.0
    
    # Linear acceleration noise
    linear_acceleration_x_noisy = msg.linear_acceleration.x + 0.0
    linear_acceleration_y_noisy = msg.linear_acceleration.y + 0.0  
    linear_acceleration_z_noisy = msg.linear_acceleration.z + 0.0
```

#### 3. Bias Evolution
```python
def _update_biases(self):
    """Update sensor biases using random walk model"""
    # TODO: Implement random walk bias evolution
    # Gyroscope bias: b_k+1 = b_k + N(0, σ²_rw * dt)
    self.gyro_bias = np.zeros(3)  # Student implementation
    
    # Accelerometer bias evolution
    self.accel_bias = np.zeros(3)  # Student implementation
    
    # Apply bias limits (sensor saturation)
    self.gyro_bias = np.clip(self.gyro_bias, 0.0, 0.0)
    self.accel_bias = np.clip(self.accel_bias, 0.0, 0.0)
```

#### 4. Covariance Matrices
```python
def _set_covariances(self, msg):
    """Set uncertainty covariance matrices"""
    # TODO: Set diagonal covariance matrices
    # 3x3 matrix represented as 9-element array (row-major)
    oc[0] = 0.0; oc[4] = 0.0; oc[8] = 0.0    # Orientation variances
    avc[0] = 0.0; avc[4] = 0.0; avc[8] = 0.0  # Angular velocity variances  
    lac[0] = 0.0; lac[4] = 0.0; lac[8] = 0.0  # Linear acceleration variances
```

### IMU Sensor Physics

#### Gyroscope Model
Measures angular velocity with bias drift and noise:

```
ω_measured = ω_true + b_gyro + n_gyro

Where:
ω_true = True angular velocity [rad/s]
b_gyro = Time-varying bias (random walk)
n_gyro = Gaussian measurement noise ~ N(0, σ²_gyro)

Bias Evolution:
b_k+1 = b_k + N(0, σ²_rw * dt)
```

#### Accelerometer Model
Measures linear acceleration including gravity:

```
a_measured = a_true + g_body + b_accel + n_accel

Where:
a_true = True vehicle acceleration in body frame [m/s²]
g_body = Gravity vector rotated to body frame
b_accel = Time-varying bias (random walk)
n_accel = Gaussian measurement noise ~ N(0, σ²_accel)
```

#### Orientation Measurement
Provides vehicle attitude as quaternion:

```
q_measured = q_true ⊗ q_noise

Where:
q_true = True vehicle orientation quaternion
q_noise = Rotational noise quaternion from rotation vector
q_noise = exp(0.5 * n_rot), n_rot ~ N(0, σ²_orient)
```

### Random Walk Bias Model

IMU sensors exhibit bias drift over time due to thermal effects and aging:

```python
# Bias evolution (discrete-time random walk)
b[k+1] = b[k] + w[k]

where w[k] ~ N(0, Q)
Q = σ²_rw * dt * I   # Process noise covariance

# Continuous-time interpretation
db/dt = w(t), where w(t) is white noise with PSD = σ²_rw
```

**Physical Interpretation:**
- **Short Term**: Bias appears constant (useful for calibration)
- **Long Term**: Bias drifts unpredictably (requires re-calibration)
- **Random Walk**: Variance grows linearly with time: Var(b(t)) = σ²_rw * t

### Coordinate Frame Conventions

The IMU follows standard automotive coordinate conventions:

```
Body Frame (IMU sensor frame):
X-axis: Forward (vehicle front)
Y-axis: Right (vehicle right side) 
Z-axis: Down (toward ground)

Angular Velocity:
ωx: Roll rate (rotation about X-axis)
ωy: Pitch rate (rotation about Y-axis)
ωz: Yaw rate (rotation about Z-axis)

Linear Acceleration:
ax: Longitudinal acceleration (forward/backward)
ay: Lateral acceleration (left/right)
az: Vertical acceleration (up/down, includes gravity)
```

## Usage Examples

### High-Performance IMU
```yaml
# Racing/research grade IMU
orientation_noise_std: 0.001      # 0.001 rad ≈ 0.06°
angular_velocity_noise_std: 0.001  # 0.001 rad/s
linear_acceleration_noise_std: 0.01  # 0.01 m/s²
gyro_bias_rw_std: 0.0001          # Very stable bias
accel_bias_rw_std: 0.001
```

### Automotive Grade IMU
```yaml
# Production vehicle IMU
orientation_noise_std: 0.01       # 0.01 rad ≈ 0.6°
angular_velocity_noise_std: 0.01   # 0.01 rad/s
linear_acceleration_noise_std: 0.01  # 0.01 m/s²
gyro_bias_rw_std: 0.001           # Moderate bias drift
accel_bias_rw_std: 0.02
```

### Low-Cost IMU
```yaml
# Consumer/MEMS IMU
orientation_noise_std: 0.05       # 0.05 rad ≈ 3°
angular_velocity_noise_std: 0.05   # 0.05 rad/s
linear_acceleration_noise_std: 0.1  # 0.1 m/s²
gyro_bias_rw_std: 0.01            # Significant bias drift
accel_bias_rw_std: 0.1
```

### Perfect IMU (Debug Mode)
```yaml
# Noise-free measurements for testing
enable_gaussian_noise: false
enable_bias: false
```

## Educational Objectives

### 1. IMU Sensor Fundamentals
- **Inertial Sensing**: Understanding gyroscope and accelerometer principles
- **Sensor Fusion**: How IMU data combines with other sensors (GPS, odometry)
- **Coordinate Frames**: Body-fixed vs inertial reference frames
- **Orientation Representation**: Quaternions vs Euler angles vs rotation matrices

### 2. Stochastic Sensor Models
- **Gaussian Noise**: Thermal and quantization noise characteristics
- **Bias Models**: Understanding deterministic vs random bias components
- **Random Walk**: Stochastic processes in sensor modeling
- **Covariance Matrices**: Uncertainty quantification and propagation

### 3. Signal Processing Applications
- **Noise Filtering**: Kalman filtering and complementary filters
- **Calibration**: Bias estimation and compensation techniques
- **Dead Reckoning**: Integration of IMU measurements for navigation
- **Sensor Fusion**: Combining IMU with GPS, odometry, and vision

### 4. Autonomous Vehicle Integration
- **Vehicle Dynamics**: How IMU measurements relate to vehicle motion
- **Localization**: IMU role in SLAM and state estimation
- **Control Systems**: IMU feedback for vehicle stability and control
- **Safety Systems**: IMU applications in rollover detection and ESC

## Implementation Exercises

### Exercise 1: Vehicle State to IMU Conversion
**Objective**: Extract IMU measurements from vehicle dynamics

**Tasks**:
- Convert vehicle yaw rate to IMU angular velocity
- Transform vehicle acceleration to body frame
- Handle gravity vector transformation
- Understand coordinate frame relationships

### Exercise 2: Sensor Noise Implementation
**Objective**: Apply realistic sensor noise models

**Tasks**:
- Generate Gaussian noise with specified standard deviation
- Add bias to measurements before noise
- Apply noise to all three axes consistently
- Understand noise impact on sensor fusion

### Exercise 3: Bias Drift Modeling
**Objective**: Implement random walk bias evolution

**Tasks**:
- Update bias states using random walk model
- Apply proper time scaling (√dt) for discrete implementation
- Implement bias saturation limits
- Understand long-term bias behavior

### Exercise 4: Covariance Matrix Setup
**Objective**: Quantify measurement uncertainty

**Tasks**:
- Set up diagonal covariance matrices
- Convert variance to covariance format
- Understand uncertainty propagation
- Prepare for Kalman filter applications

## Performance Characteristics

- **Update Rate**: 50 Hz (typical IMU frequency, configurable)
- **Computational Load**: Very low (simple noise generation)
- **Memory Usage**: Minimal state storage (bias vectors)
- **Real-time Performance**: Suitable for real-time applications

## Sensor Fusion Applications

### Complementary Filter
Simple sensor fusion combining IMU with other sensors:
```python
# High-pass filter IMU, low-pass filter other sensors
attitude = α * (attitude + ω * dt) + (1-α) * other_attitude
```

### Kalman Filter Integration
IMU serves as prediction step in extended Kalman filter:
```python
# Prediction step using IMU
x_pred = f(x, u_imu)  # State prediction with IMU input
P_pred = F*P*F' + Q   # Covariance prediction with IMU noise
```

### Dead Reckoning
Integration of IMU for short-term navigation:
```python
# Position integration (double integration)
velocity += acceleration * dt
position += velocity * dt
```

## Troubleshooting

### Common Issues
1. **TODO Not Implemented**: Functions return zero/default values
2. **Coordinate Frame Errors**: Wrong sign conventions or frame transformations
3. **Noise Scale Issues**: Incorrect standard deviation units or scaling
4. **Bias Divergence**: Random walk without proper limits

### Debugging Tips
- Verify coordinate frame conventions (NED, ENU, body frame)
- Check noise units (rad vs degrees, m/s² vs g-units)  
- Monitor bias evolution over time
- Compare with real IMU specifications
- Use RViz IMU plugin for visualization

## IMU vs Other Sensors Comparison

| Sensor | Update Rate | Measurement | Drift | Weather Dependency |
|--------|------------|-------------|-------|-------------------|
| **IMU** | 50-1000 Hz | Inertial motion | High (bias drift) | None |
| **GPS** | 1-10 Hz | Absolute position | Low | High (satellites) |
| **LiDAR** | 10 Hz | 3D environment | None | Medium (rain/fog) |
| **Camera** | 30 Hz | Visual features | None | High (lighting) |
| **Odometry** | 100 Hz | Relative motion | Medium (wheel slip) | Low |

## Sensor Specifications Reference

### Typical IMU Performance Levels

| Grade | Gyro Bias Stability | Accel Bias Stability | Noise Level | Cost |
|-------|-------------------|-------------------|-------------|------|
| **Navigation** | 0.01°/hr | 10 μg | Very Low | $10k+ |
| **Tactical** | 1°/hr | 100 μg | Low | $1k-10k |
| **Automotive** | 10°/hr | 1 mg | Medium | $100-1k |
| **Consumer** | 100°/hr | 10 mg | High | $1-100 |

---

*This IMU sensor simulation provides comprehensive educational experience in inertial sensing, stochastic modeling, and sensor fusion preparation for autonomous vehicle development.*