# Physics API Reference

Reference for functions in simulator/physics.py.

---

## Overview

The physics module provides:

- Great-circle distance calculations using the Haversine formula
- Distance conversion helpers
- Stepwise movement toward a target coordinate
- 50 m proximity checks for cargo load/unload operations
- Terrain lookup (land vs water) using simplified world polygons

Coordinate format used throughout:

- GPSCoords: (latitude, longitude)
- Latitude and longitude are decimal degrees

---

## Public Functions

### haversine_distance(coord1, coord2) -> float

Compute great-circle distance between two coordinates in kilometers.

Parameters:

| Name | Type | Description |
|------|------|-------------|
| coord1 | tuple[float, float] | Source coordinate as (lat, lon) |
| coord2 | tuple[float, float] | Destination coordinate as (lat, lon) |

Returns:

- Distance in kilometers as float

Example:

```python
from simulator.physics import haversine_distance

la = (34.0522, -118.2437)
nyc = (40.7128, -74.0060)
km = haversine_distance(la, nyc)
print(km)
```

---

### haversine_distance_meters(coord1, coord2) -> float

Compute great-circle distance between two coordinates in meters.

Parameters:

| Name | Type | Description |
|------|------|-------------|
| coord1 | tuple[float, float] | Source coordinate as (lat, lon) |
| coord2 | tuple[float, float] | Destination coordinate as (lat, lon) |

Returns:

- Distance in meters as float

Example:

```python
from simulator.physics import haversine_distance_meters

meters = haversine_distance_meters((34.0, -118.0), (34.001, -118.002))
print(meters)
```

---

### move_toward(current, target, distance_km) -> tuple[float, float]

Move from current toward target by up to distance_km.

Behavior:

- Returns target exactly if target is already reached
- Returns target exactly if remaining distance is less than or equal to distance_km
- Otherwise returns an interpolated coordinate between current and target

Parameters:

| Name | Type | Description |
|------|------|-------------|
| current | tuple[float, float] | Current coordinate as (lat, lon) |
| target | tuple[float, float] | Target coordinate as (lat, lon) |
| distance_km | float | Maximum movement distance for this step |

Returns:

- New coordinate as (lat, lon)

Example:

```python
from simulator.physics import move_toward

current = (34.0522, -118.2437)
target = (36.1699, -115.1398)
next_pos = move_toward(current, target, distance_km=50.0)
print(next_pos)
```

---

### is_within_loading_range(vehicle_location, item_location) -> bool

Check whether a vehicle is within cargo handling range of an item.

Current threshold:

- 50 meters (LOADING_PROXIMITY_METERS)

Parameters:

| Name | Type | Description |
|------|------|-------------|
| vehicle_location | tuple[float, float] | Vehicle coordinate as (lat, lon) |
| item_location | tuple[float, float] | Item coordinate as (lat, lon) |

Returns:

- True if within 50 m, else False

Example:

```python
from simulator.physics import is_within_loading_range

can_load = is_within_loading_range((34.0522, -118.2437), (34.0523, -118.2436))
print(can_load)
```

---

### is_over_land(coord) -> bool

Determine whether a coordinate falls over land.

Implementation notes:

- Uses simplified continent polygons from simulator/world_map_data.py
- Uses a ray-casting point-in-polygon test
- Intended for simulation-level terrain checks, not survey-grade GIS precision

Parameters:

| Name | Type | Description |
|------|------|-------------|
| coord | tuple[float, float] | Coordinate as (lat, lon) |

Returns:

- True when the point is classified as land
- False when classified as water

Example:

```python
from simulator.physics import is_over_land

print(is_over_land((34.0522, -118.2437)))  # likely True (Los Angeles)
print(is_over_land((0.0, -140.0)))         # likely False (Pacific Ocean)
```

---

## Internal Helper

### _point_in_polygon(lat, lon, polygon) -> bool

Internal ray-casting helper used by is_over_land.

This function is module-internal (leading underscore) and not part of the stable public API.

---

## Package-level Imports

The simulator package currently re-exports:

- haversine_distance
- haversine_distance_meters

So you can import either from simulator.physics or simulator.

Example:

```python
from simulator import haversine_distance, haversine_distance_meters
```

---

## Constants

Defined in simulator/physics.py:

- EARTH_RADIUS_KM = 6371.0
- LOADING_PROXIMITY_METERS = 50.0

These are implementation constants and may be treated as read-only configuration values.
