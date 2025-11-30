# NGV DCAS - LiDAR Sensor Simulation

## Overview
Ring-based LiDAR sensor emulator for autonomous vehicle development using **physically-based intensity calculation** and realistic multi-channel scanning patterns.

## Sensor LiDAR Node

### Description
The `sensor_lidar_node.py` implements a sophisticated multi-ring LiDAR simulation featuring:
- Ring-based scanning pattern with configurable vertical channels
- Ray casting for accurate surface intersection and occlusion
- Physics-based intensity calculation with distance and angle factors
- Advanced noise modeling and clutter simulation

### Key Features

#### ✨ Ring-Based LiDAR Simulation
- **Multi-Channel Design**: 8/16/32/64 ring configurations
- **Rotating Pattern**: Continuous 360° horizontal scanning
- **Real Ray Casting**: True geometric intersection for each laser beam
- **Occlusion Handling**: Objects behind other objects are naturally hidden

#### 🎯 Physics-Based Intensity
- **Material-Specific**: Different base intensities for various surfaces
- **Distance Attenuation**: Inverse square law approximation
- **Angular Effects**: Lambert's cosine law for incident angle
- **Surface Roughness**: Random variation modeling material properties

#### 🔧 Advanced Noise Modeling
- **Polar Coordinate Noise**: Gaussian noise in range/azimuth/elevation
- **Clutter Simulation**: False positive points from environmental interference
- **Dual Output**: Ground truth and noisy point clouds

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/lidar/points` | `PointCloud2` | Ground truth LiDAR point cloud |
| `/sensors/lidar/points_noise` | `PointCloud2` | Noise-corrupted LiDAR measurements |
| `/sensors/lidar/fov` | `MarkerArray` | Field-of-view visualization |

### PointCloud2 Fields

Each LiDAR detection contains 4 fields:
```
x, y, z          # 3D position in sensor frame (meters)
intensity        # Surface return intensity (0-255)
```

## Configuration

### LiDAR Hardware Parameters
```yaml
sensor_lidar_node:
  # Physical sensor parameters
  num_rings: 16                     # Number of vertical laser rings
  horizontal_resolution: 6.0        # Angular resolution (degrees per ray)
  vertical_fov_min: -8.0           # Minimum elevation angle (degrees)
  vertical_fov_max: 8.0            # Maximum elevation angle (degrees)
  rotation_frequency: 10.0         # Rotation speed (Hz)
  
  # Detection characteristics
  radius: 50.0                     # Maximum detection range (meters)
  period_s: 0.1                    # Update frequency (10 Hz)
```

### Physical Mounting
```yaml
  # Sensor positioning (ego_frame relative)
  offset_x: 1.2          # Forward offset (meters)
  offset_y: 0.0          # Lateral offset (meters)
  offset_z: 1.5          # Vertical offset (meters)
  roll_deg: 0.0          # Sensor roll angle (degrees)
  pitch_deg: 0.0         # Sensor pitch angle (degrees)
  yaw_deg: 0.0           # Sensor yaw angle (degrees)
```

### Object Material Properties
```yaml
sensor_shared_params:
  object_properties:
    vehicle:
      reflectivity: 0.3      # Vehicle surfaces (moderate reflectivity)
      base_intensity: 160    # Legacy intensity value (for reference)
    building:
      reflectivity: 0.8      # Concrete/brick surfaces (high reflectivity)
      base_intensity: 180    # Legacy intensity value (for reference)
    guardrail:
      reflectivity: 0.9      # Metal surfaces (very high reflectivity)
      base_intensity: 190    # Legacy intensity value (for reference)
    traffic_sign:
      reflectivity: 0.7      # Retroreflective materials (high reflectivity)
      base_intensity: 230    # Legacy intensity value (for reference)
    ground:
      reflectivity: 0.1      # Asphalt/concrete roads (low reflectivity)
      base_intensity: 70     # Legacy intensity value (for reference)
    lane_marking:
      reflectivity: 0.8      # Road paint (high reflectivity)
      base_intensity: 120    # Legacy intensity value (for reference)
```

### Noise Simulation
```yaml
  # Gaussian measurement noise
  enable_noise: true
  range_noise_std: 0.02        # Range accuracy (2cm)
  azimuth_noise_std: 0.1       # Horizontal angle accuracy (degrees)
  elevation_noise_std: 0.1     # Vertical angle accuracy (degrees)
  
  # Clutter simulation (false positives)
  enable_clutter: true
  clutter_probability: 0.3     # Clutter occurrence probability
  clutter_density: 10.0        # Average clutter points per occurrence
```

### Clutter Generation Model

LiDAR clutter (false positive detections) is generated using a **Poisson distribution** to model realistic environmental interference:

```python
# Clutter occurrence decision
if random() < clutter_probability:
    # Number of clutter points follows Poisson distribution
    num_clutter = max(1, poisson(clutter_density))
    
    for each clutter point:
        # Random position within sensor range
        range = uniform(clutter_range_min, clutter_range_max)
        azimuth = uniform(0°, 360°)  # Full 360° coverage
        elevation = uniform(vertical_fov_min, vertical_fov_max)
        
        # Random intensity within clutter range
        intensity = uniform(intensity_min, intensity_max)
```

**Why Poisson Distribution?**
- **Physical Basis**: Atmospheric particles (dust, fog, insects) occur randomly in 3D space
- **Realistic Clustering**: Poisson models natural grouping of scattered returns
- **LiDAR Physics**: Mirrors real-world backscatter from atmospheric conditions

**Clutter Characteristics:**
- **Occurrence Rate**: `clutter_probability` controls frequency of clutter events per scan
- **Point Density**: `clutter_density` sets average number of false points when clutter occurs
- **3D Distribution**: Random spherical coordinates within sensor coverage
- **Intensity Range**: Lower intensity values (10-50) to distinguish from real objects

## Technical Implementation

### LiDAR Intensity Formula

The intensity calculation uses a power-based physical model:

```
intensity = α × (Power_receive) / (range²)

Where:
- α = Scaling constant for intensity normalization (1e12)
- Power_receive = Received optical power from target (Watts)
- range = Distance to target (meters)
```

**Power Receive Calculation:**
```
Power_receive = P_laser × reflectivity × cos(θ) × roughness / range²

Where:
- P_laser = Laser transmit power (0.001W = 1mW typical)
- reflectivity = Material reflectivity coefficient (0.0-1.0)
- cos(θ) = Lambert's cosine law for incident angle
- roughness = Surface roughness variation (0.8-1.2)
- range = Distance to target (meters)
```

**Complete Intensity Formula:**
```
intensity = α × (P_laser × reflectivity × cos(θ) × roughness / range²) / range²
         = α × P_laser × reflectivity × cos(θ) × roughness / range⁴
```

**Example Calculation:**
```python
# LiDAR hitting traffic sign at 20m, 5° elevation
P_laser = 0.001W (1mW)
reflectivity = 0.7 (retroreflective sign)
cos(θ) = cos(5°) = 0.996
roughness = 1.1 (random variation)
range = 20m
α = 1e12

Power_receive = (0.001 × 0.7 × 0.996 × 1.1) / 20² = 0.000767 / 400 = 1.92e-6W

intensity = 1e12 × 1.92e-6 / 20² = 1.92e6 / 400 = 4800 → 255 (clamped)

# For closer objects (5m):
range = 5m
Power_receive = 0.000767 / 25 = 3.07e-5W
intensity = 1e12 × 3.07e-5 / 25 = 3.07e7 / 25 = 1.23e6 → 255 (clamped)

# Note: α constant is tuned to provide realistic intensity ranges (0-255)
```

### Ring-Based Scanning Pattern

LiDAR generates rings by:

1. **Ring Elevation Calculation**: Uniformly distribute rings across vertical FOV
2. **Sector-Based Scanning**: Each update covers angular sector (optimized for real-time)
3. **Ray Generation**: For each ring-azimuth combination, cast ray
4. **Intersection Detection**: Find closest surface hit point
5. **Intensity Calculation**: Apply physics-based intensity model

### Material Reflectivity Values

Different materials have characteristic reflectivity coefficients used in power calculation:

| Material Type | Reflectivity | Physical Basis |
|---------------|-------------|----------------|
| **Guardrails** | 0.9 | Metal surfaces (strong specular reflection) |
| **Lane Markings** | 0.8 | Road paint (retroreflective) |
| **Buildings** | 0.8 | Concrete/brick (good diffuse reflection) |
| **Traffic Signs** | 0.7 | Retroreflective materials (designed for high return) |
| **Vehicles** | 0.3 | Mixed materials (paint, metal, plastic, glass) |
| **Ground/Roads** | 0.1 | Asphalt (low diffuse reflection, dark surface) |

### Ray Casting Implementation

```python
def CastRay(azimuth, elevation):
    """Cast single ray and return closest intersection with intensity"""
    
    # 1. Generate ray direction from spherical coordinates
    # 2. Test intersection with all geometries:
    #    - Ground plane (z=0)
    #    - Building polygons  
    #    - Vehicle/guardrail bounding boxes
    #    - Traffic sign poles
    #    - Lane marking lines
    # 3. Find closest valid intersection
    # 4. Calculate intensity based on hit object type
    # 5. Return (x, y, z, intensity)
```

### Coordinate Transformations

1. **Map Frame** → **Ego Frame**: Vehicle pose transformation
2. **Ego Frame** → **LiDAR Frame**: Sensor mounting offset + orientation
3. **Spherical** → **Cartesian**: Convert (range, azimuth, elevation) to (x, y, z)

## Usage Examples

### High-Resolution LiDAR (Research)
```yaml
num_rings: 64                     # Dense vertical sampling
horizontal_resolution: 0.1        # 0.1° angular resolution
range_noise_std: 0.005           # 5mm range accuracy
azimuth_noise_std: 0.01          # 0.01° angular accuracy
```

### Automotive LiDAR (Production)  
```yaml
num_rings: 16                     # Standard automotive config
horizontal_resolution: 0.2        # 0.2° angular resolution
range_noise_std: 0.02            # 2cm range accuracy
azimuth_noise_std: 0.1           # 0.1° angular accuracy
```

### Fast Performance (Real-time)
```yaml
num_rings: 8                      # Reduced rings for speed
horizontal_resolution: 1.0        # Lower angular resolution
radius: 30.0                     # Shorter range for performance
```

### Debug Mode (Perfect Measurements)
```yaml
enable_noise: false               # No measurement noise
enable_clutter: false             # No false positives
noise_seed: 42                   # Reproducible results
```

## RViz Visualization

1. **Ground Truth Points** (`/sensors/lidar/points`):
   - Color by intensity: Brighter = higher intensity return
   - Shows true geometric intersections

2. **Noisy Measurements** (`/sensors/lidar/points_noise`):
   - Includes realistic measurement errors
   - Shows clutter points (false positives)

3. **Ring Pattern**: Visualize characteristic ring structure from multi-channel scanning

## Performance Notes

- **Ring-Based Processing**: Only active ring sector processed per update (10 Hz)
- **Ray Casting Optimization**: Early termination and distance-based culling
- **Memory Efficiency**: Point-wise processing without large intermediate buffers
- **Scalable Configuration**: Adjust rings/resolution for performance vs accuracy trade-off

## LiDAR vs Radar Comparison

| Aspect | LiDAR | Radar |
|--------|-------|-------|
| **Physics** | Light reflection intensity | RF power calculation |
| **Key Metric** | Intensity (0-255) | Received power (dBm) |
| **Range** | ~100m (typical) | ~200m (typical) |
| **Angular Resolution** | ~0.1° (high) | ~1° (moderate) |
| **Weather Sensitivity** | High (rain/fog) | Low (all weather) |
| **Material Response** | Optical properties | Radar cross section |

---

*This LiDAR simulation provides physically accurate intensity modeling and ring-based scanning patterns suitable for autonomous vehicle algorithm development and testing.*