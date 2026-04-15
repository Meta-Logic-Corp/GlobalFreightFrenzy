from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Define hub/port/airport locations (discovered from boxes)
WATER_COORDINATES = [
    # Major crossing points (you can discover these dynamically)
    (40.7128, -74.0060),   # NYC - Atlantic hub
    (33.9425, -118.4081),  # LA - Pacific hub
    (51.5074, -0.1278),    # London
]

def is_water_crossing(from_loc, to_loc):
    """Detect if path likely crosses ocean (simplified)"""
    # Check if distance is very long (>1000km) and crosses major bodies of water
    dist = haversine_distance_meters(from_loc, to_loc)
    
    # Long distance over 500km likely needs water crossing or air
    if dist > 500000:  # 500km
        return True
    
    # Check lat/lon ranges that indicate ocean crossings
    from_lat, from_lon = from_loc
    to_lat, to_lon = to_loc
    
    # Crossing Atlantic (East-West between Americas and Europe)
    if (from_lon < -60 and to_lon > -10) or (from_lon > -10 and to_lon < -60):
        if abs(from_lat - to_lat) < 50:  # Similar latitude
            return True
    
    # Crossing Pacific
    if (from_lon < -120 and to_lon > 120) or (from_lon > 120 and to_lon < -120):
        return True
    
    return False

def find_nearest_hub(location, hubs):
    """Find nearest hub/port/airport to location"""
    nearest = None
    min_dist = float('inf')
    for hub in hubs:
        dist = haversine_distance_meters(location, hub)
        if dist < min_dist:
            min_dist = dist
            nearest = hub
    return nearest

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Discover hubs from all box locations
    all_locations = set()
    for box in boxes.values():
        all_locations.add(box["location"])
        all_locations.add(box["destination"])
    hubs = list(all_locations)
    
    # UNLOAD: Deliver boxes at destination
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = []
            for bid in v["cargo"]:
                if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M:
                    to_unload.append(bid)
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()
    
    # LOAD, MOVE, AND TRANSHIP
    for vid, v in vehicles.items():
        if v["destination"] is None:
            loc = v["location"]
            vtype = VehicleType[v["vehicle_type"]]
            capacity_left = vtype.value.capacity - len(v["cargo"])
            
            # Load available boxes at current location
            if capacity_left > 0:
                loadable = [
                    bid for bid, box in boxes.items()
                    if not box["delivered"] and box["vehicle_id"] is None
                    and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M
                ]
                if loadable:
                    to_load = loadable[:capacity_left]
                    sim_state.load_vehicle(vid, to_load)
                    boxes = sim_state.get_boxes()
            
            if v["cargo"]:
                # Check if current vehicle can reach destination without terrain issues
                # Get first box destination
                first_bid = v["cargo"][0]
                dest = boxes[first_bid]["destination"]
                
                # Check if truck would need to cross water
                if vtype == VehicleType.SemiTruck and is_water_crossing(loc, dest):
                    # Need to transload! Find nearest hub to switch to ship/plane
                    nearest_hub = find_nearest_hub(loc, hubs)
                    if nearest_hub and nearest_hub != loc:
                        # Go to hub first
                        sim_state.move_vehicle(vid, nearest_hub)
                    else:
                        # Try to create ship at nearest water point
                        try:
                            # Unload current cargo
                            if v["cargo"]:
                                sim_state.unload_vehicle(vid, v["cargo"])
                            # Create ship at nearby water location
                            ship_vid = sim_state.create_vehicle(VehicleType.CargoShip, loc)
                            # Transfer cargo
                            sim_state.load_vehicle(ship_vid, v["cargo"])
                            sim_state.move_vehicle(ship_vid, dest)
                        except ValueError:
                            # Fallback to airplane
                            try:
                                plane_vid = sim_state.create_vehicle(VehicleType.Airplane, loc)
                                sim_state.load_vehicle(plane_vid, v["cargo"])
                                sim_state.move_vehicle(plane_vid, dest)
                            except ValueError:
                                sim_state.move_vehicle(vid, dest)
                else:
                    # Normal movement
                    sim_state.move_vehicle(vid, dest)
            
            # If empty, go to nearest undelivered box
            elif not v["cargo"]:
                nearest = None
                min_dist = float('inf')
                for bid, box in boxes.items():
                    if not box["delivered"] and box["vehicle_id"] is None:
                        dist = haversine_distance_meters(loc, box["location"])
                        if dist < min_dist:
                            min_dist = dist
                            nearest = box["location"]
                if nearest:
                    sim_state.move_vehicle(vid, nearest)
    
    # SPAWN: Create appropriate vehicles based on route
    if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                
                # Check if route requires water crossing
                needs_water_crossing = False
                for bid in box_ids[:5]:  # Sample a few boxes
                    if is_water_crossing(origin, boxes[bid]["destination"]):
                        needs_water_crossing = True
                        break
                
                # Choose vehicle based on route
                if needs_water_crossing and num_boxes >= 10:
                    # Try ship first (cheapest for water)
                    try:
                        vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                        to_load = box_ids[:VehicleType.CargoShip.value.capacity]
                        sim_state.load_vehicle(vid, to_load)
                        if to_load:
                            dest = boxes[to_load[0]]["destination"]
                            sim_state.move_vehicle(vid, dest)
                        break
                    except ValueError:
                        # Try airplane
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Airplane, origin)
                            to_load = box_ids[:VehicleType.Airplane.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            if to_load:
                                dest = boxes[to_load[0]]["destination"]
                                sim_state.move_vehicle(vid, dest)
                            break
                        except ValueError:
                            pass
                
                # Land route - use train for long distance, truck for short
                if num_boxes >= 20:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.Train, origin)
                        to_load = box_ids[:VehicleType.Train.value.capacity]
                        sim_state.load_vehicle(vid, to_load)
                        if to_load:
                            dest = boxes[to_load[0]]["destination"]
                            sim_state.move_vehicle(vid, dest)
                        break
                    except ValueError:
                        pass
                
                # Fallback to truck
                try:
                    vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                    to_load = box_ids[:VehicleType.SemiTruck.value.capacity]
                    sim_state.load_vehicle(vid, to_load)
                    if to_load:
                        dest = boxes[to_load[0]]["destination"]
                        sim_state.move_vehicle(vid, dest)
                    break
                except ValueError:
                    continue
