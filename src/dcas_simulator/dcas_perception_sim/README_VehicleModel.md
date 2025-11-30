# NGV DCAS - Vehicle Dynamics Model

## Overview
Educational vehicle dynamics simulation module featuring **bicycle model-based physics** with integrated longitudinal and lateral dynamics for autonomous vehicle algorithm development and learning.

## Vehicle Model Module

### Description
The `vehicleModel.py` implements a comprehensive educational vehicle dynamics simulator featuring:
- Bicycle model-based vehicle dynamics with coupled longitudinal/lateral motion
- Automatic kinematic-dynamic model switching based on vehicle speed
- Euler integration for real-time simulation with educational TODO exercises
- Physics-based powertrain modeling with motor torque and resistance forces

### Key Features

#### ✨ Educational Vehicle Dynamics
- **Longitudinal Dynamics**: Motor torque, brake torque, resistance forces modeling with TODO exercises
- **Lateral Dynamics**: Bicycle model with cornering stiffness and slip angle calculation
- **Model Switching**: Automatic kinematic-dynamic model transition at 5.0 m/s
- **Real-time Simulation**: Euler integration optimized for control system applications

#### 🎯 Physics-Based Modeling
- **Equivalent Inertia**: Motor, transmission, and wheel inertia effects
- **Resistance Forces**: Rolling resistance, aerodynamic drag, gravitational forces
- **Torque Mapping**: Pedal input to motor/brake torque conversion
- **Physical Constraints**: Velocity limits, slip angle limits, torque saturation

#### 🔧 Educational Framework
- **TODO Exercises**: Incomplete functions for student implementation
- **Parameter Learning**: Physical meaning of vehicle parameters
- **Model Comparison**: Kinematic vs dynamic model differences
- **Numerical Methods**: Euler integration implementation and stability

### State Vector Structure

The vehicle model uses an 8-dimensional state vector:
```
states = [x, y, ψ, vx, vy, ψ̇, ax, ay]

Where:
x, y        # Vehicle position in global coordinates (meters)
ψ           # Yaw angle (radians)
vx, vy      # Longitudinal and lateral velocities (m/s)
ψ̇           # Yaw rate (rad/s)
ax, ay      # Longitudinal and lateral accelerations (m/s²)
```

### Input Vector Structure

Control inputs for the vehicle model:
```
inputs = [steering_angle, pedal_input]

Where:
steering_angle    # Steering wheel angle (radians, ±30° max)
pedal_input      # Combined throttle/brake command (-1.0 to 1.0)
                 # Positive: acceleration, Negative: braking
```

## Configuration

### Vehicle Physical Parameters
```python
# Mass and inertia properties
m = 1319.91                    # Vehicle mass [kg]
Iz = 2093.38                   # Yaw moment of inertia [kg⋅m²]
Iw = 0.3312                    # Wheel inertia [kg⋅m²]
Im = 0.015                     # Motor inertia [kg⋅m²]
It = 0.06                      # Transmission inertia [kg⋅m²]

# Geometric properties
lf = 1.302                     # Distance from CG to front axle [m]
lr = 1.398                     # Distance from CG to rear axle [m]
width = 1.6                    # Vehicle width [m]
h = 0.5                        # Center of gravity height [m]
Rw = 0.305                     # Effective wheel radius [m]
```

### Powertrain Parameters
```python
# Motor and drivetrain
motor_torque_max = 2000.0      # Maximum motor torque [Nm]
brake_torque_max = 6000.0      # Maximum brake torque [Nm]
gear_ratio = 1.0 / 7.98        # Gear ratio (motor/wheel)
accel_const = 293.1872055      # Acceleration pedal constant
brake_const = 4488.075         # Brake pedal constant

# Resistance coefficients
k_R = 0.015                    # Rolling resistance coefficient
Cd = 0.32                      # Aerodynamic drag coefficient
A = 2.79                       # Frontal area [m²] (1.55 × 1.8)
rho = 1.225                    # Air density [kg/m³]
slope = 0.0                    # Road slope [rad]
g = 9.81                       # Gravitational acceleration [m/s²]
```

### Tire and Handling Parameters
```python
# Cornering stiffness
Caf = 154700.0                 # Front axle cornering stiffness [N/rad]
Car = 183350.0                 # Rear axle cornering stiffness [N/rad]
mu = 0.9                       # Friction coefficient

# Model switching and limits
v_switch = 5.0                 # Kinematic/dynamic model switch speed [m/s]
max_steer = 30.0               # Maximum steering angle [degrees]
max_alpha = 7.0                # Maximum tire slip angle [degrees]
beta_max = 30.0                # Maximum body slip angle [degrees]
yaw_rate_max = 120.0           # Maximum yaw rate [degrees/s]
```

## Technical Implementation

### TODO Exercise Framework

The vehicle model includes educational TODO exercises for students to complete:

#### 1. Equivalent Inertia Calculation
```python
def calculate_equivalent_inertia(self):
    """Calculate equivalent inertia moment for motor shaft reference"""
    # TODO: Complete equivalent inertia calculation
    # J_eq = Im + It + (Iw + m*Rw²) * gear_ratio²
    J_eq = 1.0  # Student implementation required
    return J_eq
```

#### 2. Resistance Forces Modeling
```python
def calculate_resistances(self, velocity):
    """Calculate vehicle resistance forces"""
    # TODO: Complete resistance calculations
    # F_roll = k_R * m * g * cos(slope) * sgn(v)
    # F_aero = 0.5 * ρ * Cd * A * v²
    # F_grav = m * g * sin(slope)
    F_roll = 1.0  # Student implementation required
    F_aero = 1.0  # Student implementation required  
    F_grav = 1.0  # Student implementation required
    return F_roll, F_aero, F_grav
```

#### 3. Pedal-to-Torque Mapping
```python
def _pedal_to_torques(self, pedal_input):
    """Convert pedal input to motor and brake torques"""
    # TODO: Complete torque mapping
    # Tm = accel_const * pedal_input (if positive)
    # Tb = brake_const * |pedal_input| (if negative)
    Tm = 1.0  # Student implementation required
    Tb = 1.0  # Student implementation required
    return Tm, Tb
```

### Longitudinal Dynamics Model

The longitudinal dynamics follow Newton's second law with rotational motion:

```
J_eq⋅ω̇ = Tmotor - (gear_ratio × Tbrake) - Tload

Where:
J_eq = Equivalent inertia moment [kg⋅m²]
Tmotor = Motor driving torque [Nm]
Tbrake = Brake torque [Nm] 
Tload = Load torque from resistance forces [Nm]
ω̇ = Motor angular acceleration [rad/s²]
```

**Load Torque Calculation:**
```
Tload = gear_ratio × Rw × (Froll + Faero + Fgrav)
ax = ω̇ × gear_ratio × Rw    # Vehicle acceleration
```

**Resistance Forces:**
```
Froll = kR × m × g × cos(slope) × sign(v)    # Rolling resistance
Faero = 0.5 × ρ × Cd × A × v²               # Aerodynamic drag
Fgrav = m × g × sin(slope)                   # Gravitational force
```

### Lateral Dynamics Model

#### Kinematic Model (Low Speed, v < 5.0 m/s)
Pure geometric relationships without tire slip:

```
β = arctan(lr × tan(δ) / (lf + lr))          # Body slip angle
ψ̇ = v × cos(β) × tan(δ) / (lf + lr)         # Yaw rate
```

**Educational Benefits:**
- Simple geometric relationships easy to understand
- No complex tire modeling required
- Suitable for low-speed parking and maneuvering scenarios

#### Dynamic Model (High Speed, v ≥ 5.0 m/s)
Bicycle model with tire slip and cornering forces:

```
# Tire slip angles
αf = δ - β - lf×ψ̇/v                         # Front tire slip angle
αr = -β + lr×ψ̇/v                            # Rear tire slip angle

# Tire lateral forces (linear tire model)
Fyf = Caf × αf                              # Front lateral force
Fyr = Car × αr                              # Rear lateral force

# Vehicle dynamics equations
β̇ = (Fyf + Fyr)/(m×v) - ψ̇                  # Body slip angle rate
ψ̈ = (lf×Fyf - lr×Fyr) / Iz                 # Yaw acceleration
```

**Educational Benefits:**
- Introduces tire slip angle concept
- Shows cornering stiffness effects
- Demonstrates lateral force balance
- Prepares for advanced vehicle dynamics

### Position Kinematics

Global position integration from vehicle-fixed velocities:

```
# Vehicle heading direction
heading = ψ + β                             # Yaw angle + slip angle

# Global coordinate velocities
ẋ = v × cos(heading)                        # Global x velocity
ẏ = v × sin(heading)                        # Global y velocity
```

### Euler Integration Implementation

Complete integration step with educational structure:

```python
def euler_step(self, states, inputs, dt):
    """Educational Euler integration step"""
    
    # Input parsing
    steering_angle = inputs[0]
    pedal_input = inputs[1]
    
    # Synchronize internal states
    self._sync_from_legacy_states(states)
    
    # 1. Longitudinal dynamics (TODO exercises)
    ax = self._longitudinal_step(pedal_input, dt)
    
    # 2. Lateral dynamics (kinematic/dynamic switching)
    yaw, yaw_rate = self._lateral_step(steering_angle, dt)
    
    # 3. Position kinematics
    x_dot, y_dot = self._position_rates()
    self._x += x_dot * dt
    self._y += y_dot * dt
    
    # Output state vector construction
    return np.array([self._x, self._y, yaw, self._v, 
                    vy, yaw_rate, ax, ay])
```

## Educational Objectives

### 1. Vehicle Dynamics Fundamentals
- **Longitudinal Motion**: Understanding powertrain forces and resistance modeling
- **Lateral Motion**: Learning cornering behavior and slip angle concepts
- **Coordinate Systems**: Body-fixed vs global coordinate transformations
- **Physical Parameters**: Interpreting vehicle specifications and their effects

### 2. Mathematical Modeling
- **Differential Equations**: Deriving motion equations from first principles
- **Numerical Integration**: Implementing and understanding Euler method
- **Model Validation**: Comparing results with physical intuition
- **Parameter Sensitivity**: Analyzing effects of parameter changes

### 3. Programming Skills
- **Object-Oriented Design**: Class structure and method organization
- **Function Implementation**: Completing TODO exercises step by step
- **Debugging**: Identifying and fixing implementation errors
- **Code Documentation**: Understanding comprehensive documentation

### 4. Automotive Engineering
- **Bicycle Model**: Foundation for vehicle dynamics analysis
- **Tire Modeling**: Introduction to tire slip and cornering forces
- **Control Applications**: Preparing for vehicle control algorithm development
- **Safety Systems**: Understanding vehicle limits and constraints

## Implementation Exercises

### Exercise 1: Complete Equivalent Inertia
**Objective**: Implement rotational inertia effects in powertrain

**Task**: Complete `calculate_equivalent_inertia()` function
```python
J_eq = self.Im + self.It + (self.Iw + self.m * self.Rw**2) * self.gear_ratio**2
```

**Learning**: Understanding gear ratio effects and inertia transformation

### Exercise 2: Implement Resistance Forces
**Objective**: Model vehicle resistance forces accurately

**Tasks**: 
- Rolling resistance with road slope effects
- Aerodynamic drag with velocity squared dependency
- Gravitational component for inclined roads

**Learning**: Physical modeling and force analysis

### Exercise 3: Pedal Input Mapping
**Objective**: Convert driver inputs to actuator commands

**Tasks**:
- Linear torque mapping for acceleration
- Separate brake torque calculation
- Torque saturation and limits

**Learning**: Control system interfaces and actuator modeling

### Exercise 4: Lateral Dynamics TODO
**Objective**: Complete kinematic and dynamic bicycle models

**Tasks**:
- Kinematic slip angle calculation
- Dynamic tire slip angles
- Force balance equations
- Physical constraint application

**Learning**: Advanced vehicle dynamics and tire modeling

## Usage Examples

### Basic Simulation
```python
from vehicleModel import VehicleModel

# Initialize vehicle
vehicle = VehicleModel()

# Initial state: [x, y, yaw, vx, vy, yaw_rate, ax, ay]
state = [0, 0, 0, 0, 0, 0, 0, 0]

# Simulation loop
dt = 0.01  # 100 Hz
for step in range(1000):
    # Control inputs: [steering, pedal]
    inputs = [np.deg2rad(5), 0.3]  # 5° steering, 30% throttle
    
    # Integrate one step
    state = vehicle.euler_step(state, inputs, dt)
    
    # Extract results
    x, y, yaw = state[0], state[1], state[2]
    velocity = state[3]
```

### Parameter Study
```python
# Study effect of cornering stiffness
for Caf in [100000, 150000, 200000]:
    vehicle.Caf = Caf
    # Run simulation and compare results
```

### Model Validation
```python
# Compare kinematic vs dynamic models
vehicle.v_switch = 0.0    # Always dynamic
# vs
vehicle.v_switch = 100.0  # Always kinematic
```

## Performance Characteristics

- **Computational Speed**: Optimized for real-time simulation (>1000 Hz)
- **Educational Focus**: Clear structure with TODO exercises
- **Parameter Range**: Realistic automotive vehicle parameters
- **Numerical Stability**: Stable with dt < 0.01s for typical maneuvers

## Model Validation

### Test Scenarios
1. **Straight Line**: Acceleration and braking validation
2. **Constant Radius**: Steady-state cornering behavior  
3. **Step Steer**: Transient response characteristics
4. **Combined Maneuver**: Simultaneous steering and acceleration

### Expected Behaviors
- **Low Speed**: Kinematic model provides geometric relationships
- **High Speed**: Dynamic model shows tire slip effects
- **Transitions**: Smooth switching between model types
- **Limits**: Proper saturation at physical constraints

## Troubleshooting

### Common Issues
1. **TODO Not Implemented**: Functions return placeholder values (1.0)
2. **Numerical Instability**: Time step too large (reduce dt)
3. **Unrealistic Behavior**: Check parameter values and units
4. **Model Switching**: Verify v_switch threshold effects

### Debugging Tips
- Print intermediate values in TODO functions
- Check units consistency (radians vs degrees)
- Validate against simple test cases (straight line, pure cornering)
- Compare kinematic and dynamic model outputs

## Vehicle Model vs LiDAR Sensor Comparison

| Aspect | Vehicle Model | LiDAR Sensor |
|--------|---------------|--------------|
| **Physics** | Vehicle dynamics equations | Light reflection and ray casting |
| **Key Output** | State vector (position, velocity) | Point cloud with intensity |
| **Update Rate** | ~100-1000 Hz (control) | ~10 Hz (typical LiDAR) |
| **Computational Load** | Low (analytical) | Medium (ray casting) |
| **Educational Focus** | Dynamics and control | Perception and sensing |
| **Model Type** | Differential equations | Geometric ray tracing |

---

*This vehicle dynamics model provides a comprehensive educational foundation for understanding vehicle motion, implementing numerical simulation methods, and developing autonomous vehicle algorithms.*