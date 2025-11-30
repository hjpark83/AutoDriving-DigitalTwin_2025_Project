# NGV DCAS - Radar Sensor Simulation

## Overview
Automotive radar sensor emulator for autonomous vehicle development using **physically accurate radar equations** and realistic signal processing.

## Sensor Radar Node

### Description
The `sensor_radar_node.py` implements a sophisticated automotive radar simulation using the **radar equation** to calculate received power based on:
- Object distance, RCS (Radar Cross Section), and material reflectivity
- Real radar hardware parameters (transmit power, antenna gain, frequency)
- Physical propagation losses and detection thresholds

### Key Features

#### ✨ Physics-Based Simulation
- **Radar Equation**: P_r = (P_t × G_t × G_r × λ² × σ) / ((4π)³ × R⁴ × L)
- **Power-based Detection**: Uses received power threshold instead of simple RCS filtering
- **Material Properties**: Realistic reflectivity values for different surfaces

#### 🎯 Multi-Object Detection
- **Vehicles**: Moving objects with doppler velocity calculation
- **Static Objects**: Buildings, guardrails, traffic signs with ego-motion doppler
- **Ground Clutter**: False positive noise simulation
- **Ray Casting**: Accurate surface intersection for precise positioning

#### 🔧 Realistic Hardware Modeling
- **77 GHz Automotive Radar**: Standard FMCW radar parameters
- **Configurable Hardware**: Transmit power, antenna gains, system losses
- **Noise Simulation**: Gaussian noise on range and azimuth measurements

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/radar/points` | `PointCloud2` | Ground truth radar point cloud |
| `/sensors/radar/points_noise` | `PointCloud2` | Noise-corrupted radar measurements |
| `/sensors/radar/fov` | `MarkerArray` | Field-of-view visualization |

### PointCloud2 Fields

Each radar detection contains 6 fields:
```
x, y, z          # 3D position in radar frame (meters)
power            # Received power in dBm
doppler          # Doppler velocity (m/s, positive = approaching)
rcs              # Radar Cross Section (m²)
```

## Configuration

### Radar Hardware Parameters
```yaml
sensor_radar_node:
  # Physical radar parameters
  transmit_power_dbm: 20.0        # TX power (typical: 10-30 dBm)
  transmit_antenna_gain_db: 25.0  # TX antenna gain (typical: 20-30 dB)
  receive_antenna_gain_db: 25.0   # RX antenna gain (typically = TX)
  frequency_ghz: 77.0             # Operating frequency (76-81 GHz automotive band)
  system_losses_db: 10.0          # Total system losses (cables, processing, etc.)
  
  # Detection threshold
  power_threshold_dbm: -90.0      # Minimum detectable power
  detection_probability: 0.99     # Detection probability above threshold
```

### Physical Mounting
```yaml
  # Sensor positioning (ego_frame relative)
  offset_x: 1.0          # Forward offset (meters)
  offset_y: 0.0          # Lateral offset (meters)
  offset_z: 0.5          # Vertical offset (meters)
  yaw_deg: 0.0           # Sensor yaw angle (degrees)
  
  # Detection characteristics
  fov_deg: 120.0         # Field of view (degrees)
  max_range: 50.0        # Maximum detection range (meters)
  period_s: 0.1          # Update frequency (10 Hz)
```

### Object Properties
```yaml
sensor_shared_params:
  object_properties:
    vehicle:
      base_rcs: 10.0       # Vehicle RCS (m²)
      reflectivity: 0.3    # Surface reflectivity (0.0-1.0)
    building:
      base_rcs: 100.0      # Large flat surfaces  
      reflectivity: 0.8    # Concrete/brick (high)
    guardrail:
      base_rcs: 5.0        # Metal barriers
      reflectivity: 0.9    # Metal (very high)
    traffic_sign:
      base_rcs: 8.0        # Retroreflective signs
      reflectivity: 0.7    # Retroreflective material
```

### Noise Simulation
```yaml
  # Measurement noise
  enable_noise: true
  range_noise_std: 0.1        # Range accuracy (meters)
  azimuth_noise_std: 0.5      # Angular accuracy (degrees)
  
  # Clutter noise (false positives)
  enable_clutter: true
  clutter_probability: 0.05   # Clutter occurrence probability
  clutter_density: 2.0        # Average clutter points per occurrence
```

### Clutter Generation Model

Radar clutter (false positive detections) is generated using a **Poisson distribution** to model realistic environmental interference:

```python
# Clutter occurrence decision
if random() < clutter_probability:
    # Number of clutter points follows Poisson distribution
    num_clutter = max(1, poisson(clutter_density))
    
    for each clutter point:
        # Random position within FOV and range limits
        range = uniform(clutter_range_min, clutter_range_max)
        azimuth = uniform(-half_fov, +half_fov)
        
        # Random RCS and power calculation
        rcs = uniform(clutter_rcs_min, clutter_rcs_max)
        power = radar_equation(range, rcs, 'ground')
```

**Why Poisson Distribution?**
- **Physical Basis**: Clutter sources (rain droplets, dust, insects) occur randomly in space
- **Realistic Clustering**: Poisson naturally models sporadic bursts of multiple false detections
- **Automotive Relevance**: Real radar systems experience this type of environmental interference

**Clutter Characteristics:**
- **Occurrence Rate**: `clutter_probability` controls how often clutter appears per scan
- **Intensity**: `clutter_density` sets average number of false points when clutter occurs
- **Spatial Distribution**: Random within sensor FOV and range limits
- **Power Levels**: Calculated using radar equation with random RCS values

## Technical Implementation

### Radar Equation Breakdown

The radar equation calculates received power based on physical parameters:

```
P_r = (P_t × G_t × G_r × λ² × σ_effective) / ((4π)³ × R⁴ × L)

Where:
- P_t = Transmit power (Watts)
- G_t = Transmit antenna gain (linear)
- G_r = Receive antenna gain (linear)  
- λ = Wavelength (c/f = 3×10⁸/77×10⁹ ≈ 3.9mm)
- σ_effective = RCS × reflectivity (m²)
- R = Range to target (meters)
- L = System losses (linear)
```

**Power Calculation Example:**
```python
# 77 GHz radar detecting vehicle at 30m
P_t = 0.1W (20 dBm)
G_t = G_r = 316 (25 dB each)
λ = 0.0039m
σ = 10 m² × 0.3 = 3 m² (vehicle with 30% reflectivity)
R = 30m
L = 10 (10 dB losses)

P_r = (0.1 × 316 × 316 × (0.0039)² × 3) / ((4π)³ × 30⁴ × 10)
    = -85.2 dBm → DETECTED (above -90 dBm threshold)
```

### Object RCS Values

RCS represents the **inherent radar signature** of objects:

| Object Type | RCS (m²) | Physical Basis |
|-------------|----------|----------------|
| **Small Car** | 10 | Typical frontal cross-section |
| **Large Building** | 100 | Large flat surface reflection |
| **Metal Guardrail** | 5 | Linear metal reflector |
| **Traffic Sign** | 8 | Corner reflector design |
| **Ground/Road** | 0.05 | Rough asphalt surface |

### Doppler Velocity Calculation

Calculates relative velocity component along line-of-sight:

```python
# For moving vehicles
doppler = (target_velocity - ego_velocity) · line_of_sight_unit_vector

# For static objects (buildings, signs)
doppler = -ego_velocity · line_of_sight_unit_vector  # Ego motion only
```

### Coordinate Transformations

1. **Map Frame** → **Ego Frame**: Vehicle pose transformation
2. **Ego Frame** → **Radar Frame**: Sensor mounting offset + yaw rotation  
3. **Cartesian** ↔ **Polar**: For noise application in range/azimuth domain

## Usage Examples

### High-Precision Radar (Research)
```yaml
transmit_power_dbm: 30.0      # Higher power
power_threshold_dbm: -100.0   # More sensitive
range_noise_std: 0.02         # 2cm accuracy
azimuth_noise_std: 0.1        # 0.1° accuracy
```

### Automotive-Grade Radar (Production)
```yaml  
transmit_power_dbm: 20.0      # Standard power
power_threshold_dbm: -90.0    # Typical sensitivity
range_noise_std: 0.1          # 10cm accuracy  
azimuth_noise_std: 0.5        # 0.5° accuracy
```

### Debug Mode (Perfect Measurements)
```yaml
enable_noise: false           # No measurement noise
enable_clutter: false         # No false positives
power_threshold_dbm: -200.0   # Detect everything
```

## RViz Visualization

1. **Ground Truth Points** (`/sensors/radar/points`):
   - Color by power: Brighter = stronger signal
   - Size by RCS: Larger = bigger radar signature

2. **Noisy Measurements** (`/sensors/radar/points_noise`):
   - Shows realistic sensor performance
   - Includes range/azimuth noise and clutter

3. **FOV Visualization** (`/sensors/radar/fov`):
   - Shows detection coverage area
   - Transparent cone visualization

## Performance Notes

- **Update Rate**: 10 Hz (typical automotive radar)
- **Processing**: Optimized ray casting with early termination
- **Memory**: Efficient point cloud generation
- **Accuracy**: Physics-based power calculation ensures realistic detection ranges

---

*This radar simulation provides physically accurate modeling suitable for autonomous vehicle algorithm development and testing.*