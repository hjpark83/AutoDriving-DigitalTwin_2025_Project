# NGV DCAS - GNSS Sensor Simulation

## Overview
Educational GNSS (Global Navigation Satellite System) sensor emulator featuring **geodetic coordinate transformations** with realistic noise modeling and shadow zone simulation for autonomous vehicle development.

## Sensor GNSS Node

### Description
The `sensor_gnss_node.py` implements a comprehensive GNSS sensor simulator featuring:
- ENU to WGS84 coordinate system transformation with geodetic calculations
- Realistic noise modeling for position, velocity, and altitude measurements
- GNSS shadow zone simulation for urban canyon effects
- Educational TODO exercises for understanding satellite navigation systems

### Key Features

#### ✨ Geodetic Coordinate Transformation
- **ENU to WGS84**: East-North-Up local coordinates to latitude-longitude-altitude global coordinates
- **Earth Curvature**: Meridional and normal radius of curvature calculations
- **WGS84 Ellipsoid**: Accurate modeling using World Geodetic System 1984 parameters
- **Real-time Conversion**: High-frequency coordinate transformation at 10 Hz

#### 🎯 Physics-Based GNSS Modeling
- **Position Accuracy**: Gaussian noise simulation for horizontal and vertical positioning
- **Velocity Estimation**: Doppler-based velocity measurements with noise
- **Shadow Zones**: Urban canyon and tunnel effects on signal quality
- **Covariance Matrices**: Uncertainty quantification for sensor fusion applications

#### 🔧 Educational Framework
- **TODO Exercises**: Student implementation of geodetic calculations and coordinate transformations
- **Satellite Navigation**: Understanding GNSS principles and error sources
- **Parameter Learning**: Real-world GNSS specifications and performance characteristics
- **Geodesy Fundamentals**: Earth ellipsoid modeling and coordinate system theory

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/gnss/fix` | `sensor_msgs/NavSatFix` | GNSS position fix (latitude, longitude, altitude) |
| `/sensors/gnss/odom` | `nav_msgs/Odometry` | GNSS-based odometry with pose and velocity |
| `/sensors/gnss/shadow_zones` | `visualization_msgs/MarkerArray` | Shadow zone visualization markers |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vehicle/state` | `VehicleState` | Vehicle dynamics for coordinate transformation |

### Message Formats

#### NavSatFix Message
```
Header header                    # Timestamp and coordinate frame
NavSatStatus status             # Fix status and service type
float64 latitude                # Latitude [degrees]
float64 longitude               # Longitude [degrees] 
float64 altitude                # Altitude above mean sea level [meters]
float64[9] position_covariance  # Position uncertainty matrix
uint8 position_covariance_type  # Covariance matrix type
```

#### Odometry Message
```
Header header                   # Timestamp and coordinate frame
string child_frame_id          # Moving coordinate frame
PoseWithCovariance pose        # Position and orientation with uncertainty
TwistWithCovariance twist      # Linear and angular velocity with uncertainty
```

## Configuration

### GNSS Reference Parameters
```yaml
sensor_gnss_node:
  # Basic sensor configuration
  enabled: true                 # Enable/disable GNSS sensor
  period_s: 0.1                # Update frequency: 10 Hz (0.1s)
  
  # WGS84 reference point (origin of local ENU frame)
  lat0: 37.0                   # Reference latitude [degrees]
  lon0: 127.0                  # Reference longitude [degrees] 
  alt0: 50.0                   # Reference altitude [meters MSL]
```

### Noise Model Parameters
```yaml
  # Position and velocity noise characteristics
  enable_noise: true           # Enable measurement noise
  pos_noise_std: 0.5          # Horizontal position noise std dev [meters]
  alt_noise_std: 1.0          # Vertical position noise std dev [meters]
  vel_noise_std: 0.05         # Velocity noise std dev [m/s]
  ang_noise_std: 0.002        # Angular velocity noise std dev [rad/s]
  ori_noise_std: 0.01         # Orientation noise std dev [radians]
```

### Shadow Zone Parameters
```yaml
  # GNSS shadow zone simulation
  enable_shadow_zones: false   # Enable shadow zone effects
  enable_shadow_viz: false     # Enable shadow zone visualization
  shadow_map_csv: ""          # Path to shadow zone CSV file
  
  # Shadow zone CSV format: x, y, pos_std, radius
  # x, y: Center coordinates in ENU frame [meters]
  # pos_std: Additional position error std dev [meters]
  # radius: Zone influence radius [meters]
```

## Technical Implementation

### TODO Exercise Framework

The GNSS simulator includes educational exercises for students:

#### 1. Coordinate Transformation Setup
```python
# TODO: ENU to WGS84 coordinate transformation
enu_coords = np.array([e, n, u])         # East-North-Up coordinates
ref_llh = np.array([lat0, lon0, alt0])   # Reference WGS84 coordinates
llh_result = self.enu_to_wgs84(enu_coords, ref_llh)  # Student implementation needed
```

#### 2. Earth Curvature Calculations
```python
def meridional_radius(self, ref_latitude_deg):
    """Calculate meridional radius of curvature"""
    # TODO: Meridional radius calculation
    # M = (ab)² / ((a cos φ)² + (b sin φ)²)^(3/2)
    num = 1.0  # Student implementation needed
    den = 1.0  # Student implementation needed
    radius = num / den
    return radius

def normal_radius(self, ref_latitude_deg):
    """Calculate normal radius of curvature"""
    # TODO: Normal radius calculation  
    # N = a² / ((a cos φ)² + (b sin φ)²)^(1/2)
    num = 1.0  # Student implementation needed
    den = 1.0  # Student implementation needed
    return num / den
```

#### 3. ENU to WGS84 Transformation
```python
def enu_to_wgs84(self, enu, ref_llh):
    """Convert ENU coordinates to WGS84"""
    # TODO: Calculate angle changes from ENU displacements
    # Δφ = ΔN / M (latitude change from north displacement)
    # Δλ = ΔE / (N × cos(φ)) (longitude change from east displacement)
    delta_lat_deg = np.rad2deg(0.0)  # Student implementation
    delta_lon_deg = np.rad2deg(0.0)  # Student implementation
    
    llh[0] = delta_lat_deg + ref_llh[0]  # Final latitude
    llh[1] = delta_lon_deg + ref_llh[1]  # Final longitude
    return llh
```

#### 4. GNSS Noise Application
```python
def _apply_noise(self, msg, msg_odom, shadow_pos_std):
    """Apply realistic GNSS measurement noise"""
    # TODO: Apply Gaussian noise to position measurements
    n_lat_m = self.rng.normal(0.0, 0.0)  # Student implementation
    n_lon_m = self.rng.normal(0.0, 0.0)  # Student implementation  
    n_alt_m = self.rng.normal(0.0, 0.0)  # Student implementation
    
    # TODO: Apply shadow zone additional noise
    if shadow_pos_std > 0.0:
        sh_lat_m = self.rng.normal(0.0, 0.0)  # Student implementation
        sh_lon_m = self.rng.normal(0.0, 0.0)  # Student implementation
```

#### 5. Shadow Zone Query
```python
def _query_shadow_pos_std(self, x, y):
    """Check if position is within shadow zone"""
    # TODO: Calculate distance to shadow zone centers
    for zx, zy, zstd, rr in self.shadow_zones:
        dx = 0.0  # Student implementation
        dy = 0.0  # Student implementation
        if dx * dx + dy * dy <= 1.0:  # Student: check distance condition
            max_std = 0.0  # Student implementation
    return max_std
```

### Geodetic Coordinate Systems

#### WGS84 Ellipsoid Parameters
```
World Geodetic System 1984 (WGS84):
Semi-major axis (a): 6,378,137.0 meters
Semi-minor axis (b): 6,356,752.314245 meters
Flattening (f): 1/298.257223563
Eccentricity²: 0.00669437999014

Reference ellipsoid used by GPS and most GNSS systems
```

#### Coordinate System Definitions
```
ENU (East-North-Up):
- Local Cartesian coordinate system
- Origin at reference point (lat0, lon0, alt0)
- X-axis: East direction (positive eastward)
- Y-axis: North direction (positive northward)  
- Z-axis: Up direction (positive upward)
- Units: meters

WGS84 (World Geodetic System 1984):
- Global geodetic coordinate system
- Latitude: Angular distance north of equator [-90°, +90°]
- Longitude: Angular distance east of prime meridian [-180°, +180°]
- Altitude: Height above mean sea level [meters]
- Units: degrees for lat/lon, meters for altitude
```

### Earth Curvature Mathematics

#### Meridional Radius of Curvature (M)
Radius of curvature in the north-south direction:

```
M = a(1 - e²) / (1 - e² sin²φ)^(3/2)

Alternative form:
M = (ab)² / ((a cos φ)² + (b sin φ)²)^(3/2)

Where:
φ = latitude [radians]
a = semi-major axis [meters] 
b = semi-minor axis [meters]
e² = first eccentricity squared
```

#### Normal Radius of Curvature (N)
Radius of curvature in the east-west direction:

```
N = a / √(1 - e² sin²φ)

Alternative form:
N = a² / √((a cos φ)² + (b sin φ)²)

Where:
φ = latitude [radians]
a = semi-major axis [meters]
b = semi-minor axis [meters]
```

### Coordinate Transformation Mathematics

#### ENU to WGS84 Conversion
Converting local East-North-Up coordinates to global latitude-longitude-altitude:

```
Step 1: Calculate radius of curvature at reference point
M = meridional_radius(lat0)
N = normal_radius(lat0)

Step 2: Convert displacements to angular changes
Δφ = ΔN / M                    # Latitude change [radians]
Δλ = ΔE / (N × cos(lat0))      # Longitude change [radians]

Step 3: Add to reference coordinates
latitude = lat0 + Δφ × (180/π)   # Convert to degrees
longitude = lon0 + Δλ × (180/π)  # Convert to degrees
altitude = alt0 + ΔU             # Direct addition for height

Where:
ΔE = East displacement [meters]
ΔN = North displacement [meters] 
ΔU = Up displacement [meters]
lat0, lon0 = Reference point [degrees]
```

### GNSS Error Sources and Modeling

#### Position Accuracy Factors
```
Typical GNSS Position Errors:
1. Satellite geometry (PDOP): 1-3 meters
2. Atmospheric delays: 1-5 meters
3. Multipath effects: 0.5-2 meters  
4. Receiver noise: 0.1-0.5 meters
5. Selective availability: 0 meters (discontinued)

Combined horizontal accuracy: 1-5 meters (95% confidence)
Combined vertical accuracy: 2-10 meters (95% confidence)
```

#### Velocity Estimation
```
GNSS Velocity Measurement:
- Doppler shift measurement from satellite signals
- Typical accuracy: 0.03-0.1 m/s (95% confidence)
- Update rate: 1-10 Hz depending on receiver
- More accurate than position differentiation
```

### Shadow Zone Effects

#### Urban Canyon Phenomenon
```
GNSS Signal Degradation in Urban Areas:
- Building reflections cause multipath errors
- Satellite signals blocked by tall structures
- Position accuracy can degrade to 10-50 meters
- Vertical accuracy particularly affected
- May cause complete signal loss in tunnels
```

#### Shadow Zone CSV Format
```
Example shadow_zones.csv:
x,y,pos_std,radius
50.0,100.0,5.0,10.0    # Building shadow
-20.0,30.0,10.0,15.0   # Tunnel entrance
75.0,-40.0,3.0,8.0     # Bridge overpass

Fields:
x, y: Shadow zone center in ENU coordinates [meters]
pos_std: Additional position error std deviation [meters] 
radius: Zone influence radius [meters]
```

## Usage Examples

### High-Precision GNSS (RTK/PPP)
```yaml
# Real-Time Kinematic or Precise Point Positioning
pos_noise_std: 0.01          # 1 cm horizontal accuracy
alt_noise_std: 0.02          # 2 cm vertical accuracy
vel_noise_std: 0.01          # 1 cm/s velocity accuracy
enable_shadow_zones: false   # Minimal environmental effects
```

### Standard Automotive GNSS
```yaml
# Typical automotive grade receiver
pos_noise_std: 0.5           # 0.5 meter horizontal accuracy
alt_noise_std: 1.0           # 1 meter vertical accuracy  
vel_noise_std: 0.05          # 5 cm/s velocity accuracy
enable_shadow_zones: true    # Include urban effects
```

### Consumer Grade GNSS
```yaml
# Smartphone or low-cost receiver
pos_noise_std: 2.0           # 2 meter horizontal accuracy
alt_noise_std: 5.0           # 5 meter vertical accuracy
vel_noise_std: 0.1           # 10 cm/s velocity accuracy
enable_shadow_zones: true    # Significant environmental effects
```

### Debug Mode (Perfect GNSS)
```yaml
# Noise-free measurements for testing
enable_noise: false          # No measurement noise
enable_shadow_zones: false   # No environmental effects
```

## Educational Objectives

### 1. Satellite Navigation Fundamentals
- **GNSS Principles**: Understanding satellite-based positioning systems
- **Coordinate Systems**: ENU vs WGS84 coordinate frame relationships
- **Geodesy**: Earth ellipsoid modeling and coordinate transformations
- **Error Sources**: Atmospheric effects, multipath, and geometric dilution

### 2. Geodetic Mathematics
- **Earth Curvature**: Meridional and normal radius calculations
- **Coordinate Conversion**: Mathematical foundations of map projections
- **Spherical Trigonometry**: Great circle calculations and geodesic problems
- **Precision vs Accuracy**: Understanding measurement uncertainty

### 3. Sensor Fusion Applications  
- **Kalman Filtering**: GNSS as absolute position reference
- **Dead Reckoning**: Combining with IMU and odometry
- **Map Matching**: Constraining position estimates to road networks
- **Integrity Monitoring**: Detecting and handling GNSS failures

### 4. Autonomous Vehicle Integration
- **Localization**: GNSS role in vehicle positioning systems
- **Path Planning**: Global coordinates for route navigation
- **Geo-fencing**: Location-based safety and operational zones
- **Fleet Management**: Vehicle tracking and logistics applications

## Implementation Exercises

### Exercise 1: Geodetic Coordinate Transformation
**Objective**: Implement ENU to WGS84 coordinate conversion

**Tasks**:
- Calculate Earth curvature radii at reference latitude
- Convert horizontal displacements to angular coordinates
- Handle coordinate system transformations accurately
- Understand geodetic vs geocentric coordinates

### Exercise 2: Earth Ellipsoid Calculations
**Objective**: Implement radius of curvature calculations

**Tasks**:
- Meridional radius calculation using WGS84 parameters
- Normal radius calculation for east-west curvature
- Understand variation of Earth's shape with latitude
- Compare spherical vs ellipsoidal Earth models

### Exercise 3: GNSS Noise Modeling
**Objective**: Apply realistic measurement uncertainties

**Tasks**:
- Generate Gaussian noise with appropriate scales
- Convert between meters and degrees for lat/lon noise
- Apply different noise levels for horizontal vs vertical
- Understand covariance matrix structure and meaning

### Exercise 4: Shadow Zone Implementation
**Objective**: Model environmental effects on GNSS accuracy

**Tasks**:
- Implement spatial queries for shadow zone detection
- Calculate distance-based signal degradation
- Load shadow zone data from CSV files
- Visualize shadow zones using RViz markers

## Performance Characteristics

- **Update Rate**: 10 Hz (typical GNSS receiver frequency)
- **Coordinate Accuracy**: Sub-meter with proper geodetic calculations
- **Computational Load**: Low (primarily trigonometric functions)
- **Real-time Performance**: Suitable for vehicle navigation applications

## GNSS Applications in Autonomous Vehicles

### Global Localization
```python
# Absolute position reference for map-based navigation
global_pose = gnss_to_map_transform(gnss_fix)
if gnss_accuracy < threshold:
    enable_dead_reckoning_mode()
```

### Geo-fencing and Safety Zones
```python
# Location-based operational constraints
if distance_to_restricted_zone(current_position) < safety_margin:
    apply_speed_limit_or_stop()
```

### Route Planning and Navigation
```python
# Global path planning using lat/lon waypoints
route_waypoints = [
    (37.1234, 127.5678),  # Start point
    (37.2345, 127.6789),  # Intermediate waypoint
    (37.3456, 127.7890)   # Destination
]
```

## Troubleshooting

### Common Issues
1. **TODO Not Implemented**: Functions return default/zero values
2. **Coordinate Frame Errors**: Wrong ENU to WGS84 transformation
3. **Earth Curvature Mistakes**: Incorrect radius calculations
4. **Unit Conversion Errors**: Mixing degrees, radians, and meters

### Debugging Tips
- Verify WGS84 ellipsoid parameters (a = 6,378,137m, b = 6,356,752m)
- Check latitude/longitude bounds (±90°, ±180°)
- Test coordinate transformations with known reference points
- Monitor GNSS accuracy using covariance values
- Use online geodetic calculators for verification

## GNSS vs Other Sensors Comparison

| Sensor | Update Rate | Coverage | Accuracy | Drift | Weather Dependency |
|--------|-------------|----------|----------|-------|--------------------|
| **GNSS** | 1-10 Hz | Global | 0.5-5 m | None | Medium (ionosphere) |
| **IMU** | 100-1000 Hz | Local | High rate | High | None |
| **LiDAR** | 10 Hz | Local | cm-level | None | High (rain/snow) |
| **Camera** | 30 Hz | Local | Variable | None | Very High (lighting) |
| **Odometry** | 100 Hz | Relative | Good | Medium | Low |

## Industry Standards and Specifications

### GNSS Performance Classes

| Class | Horizontal Accuracy | Vertical Accuracy | Typical Applications |
|-------|-------------------|------------------|---------------------|
| **Survey** | 1-5 cm | 2-10 cm | Mapping, construction |
| **Navigation** | 1-3 m | 2-5 m | Aviation, marine |
| **Automotive** | 0.5-2 m | 1-3 m | Vehicle navigation |
| **Consumer** | 2-5 m | 5-10 m | Smartphones, fitness |

### Coordinate Reference Systems
- **WGS84**: World Geodetic System 1984 (GPS standard)
- **ITRF**: International Terrestrial Reference Frame
- **Local Grids**: UTM, State Plane, national systems
- **Map Datums**: Transformation between coordinate systems

---

*This GNSS sensor simulation provides comprehensive educational experience in satellite navigation, geodetic coordinate systems, and global positioning for autonomous vehicle development.*