from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Discovered airports and ports from boxes (will grow over time)
airports = set()
ocean_ports = set()

def find_nearest_facility(location, facilities):
    """Find nearest airport or port to a location"""
    nearest = None
    min_dist = float('inf')
    for facility in facilities:
        dist = haversine_distance_meters(location, facility)
        if dist < min_dist:
            min_dist = dist
            nearest = facility
    return nearest, min_dist

def is_water_crossing(from_loc, to_loc):
    """Detect if path likely requires water crossing"""
    dist = haversine_distance_meters(from_loc, to_loc)
    if dist > 800000:
        return True
    
    from_lat, from_lon = from_loc
    to_lat, to_lon = to_loc
    
    # Atlantic crossing
    if (from_lon < -60 and to_lon > -10) or (from_lon > -10 and to_lon < -60):
        if abs(from_lat - to_lat) < 50:
            return True
    
    return False

def step(sim_state):
    global airports, ocean_ports
    
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Discover airports and ports from box locations (if they serve as hubs)
    for box in boxes.values():
        airports.add(box["location"])
        airports.add(box["destination"])
        ocean_ports.add(box["location"])
        ocean_ports.add(box["destination"])
    
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
    
    # LOAD & MOVE: For idle vehicles
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
                # Get first box destination
                first_bid = v["cargo"][0]
                dest = boxes[first_bid]["destination"]
                
                # Check if vehicle needs to go to specific facility
                if vtype == VehicleType.Airplane:
                    # Must be at airport to load/unload
                    nearest_airport, dist_to_airport = find_nearest_facility(loc, airports)
                    if dist_to_airport > 5000:  # Not within 5km of airport
                        # Go to nearest airport first
                        if nearest_airport:
                            sim_state.move_vehicle(vid, nearest_airport)
                            continue
                
                elif vtype == VehicleType.CargoShip:
                    # Must be at ocean port
                    nearest_port, dist_to_port = find_nearest_facility(loc, ocean_ports)
                    if dist_to_port > 5000:
                        if nearest_port:
                            sim_state.move_vehicle(vid, nearest_port)
                            continue
                
                # Normal movement to destination
                sim_state.move_vehicle(vid, dest)
            
            # If empty, go to nearest undelivered box or facility
            elif not v["cargo"]:
                # First try to find undelivered box
                nearest = None
                min_dist = float('inf')
                for bid, box in boxes.items():
                    if not box["delivered"] and box["vehicle_id"] is None:
                        dist = haversine_distance_meters(loc, box["location"])
                        if dist < min_dist:
                            min_dist = dist
                            nearest = box["location"]
                
                # If no boxes, go to nearest airport/port based on vehicle type
                if not nearest:
                    if vtype == VehicleType.Airplane:
                        nearest, _ = find_nearest_facility(loc, airports)
                    elif vtype == VehicleType.CargoShip:
                        nearest, _ = find_nearest_facility(loc, ocean_ports)
                
                if nearest:
                    sim_state.move_vehicle(vid, nearest)
    
    # SPAWN: Create vehicles with facility routing
    if tick == 0 or (len(vehicles) < 20 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                
                # Calculate average distance
                total_dist = 0
                for bid in box_ids:
                    total_dist += haversine_distance_meters(origin, boxes[bid]["destination"])
                avg_dist = total_dist / num_boxes if num_boxes > 0 else 0
                
                # Check if needs water crossing
                needs_water = False
                for bid in box_ids[:3]:
                    if is_water_crossing(origin, boxes[bid]["destination"]):
                        needs_water = True
                        break
                
                # Try vehicles in order of cost efficiency
                candidates = []
                
                if needs_water and num_boxes >= 10:
                    candidates.append(VehicleType.CargoShip)  # Cheapest for water
                    candidates.append(VehicleType.Airplane)   # Fast but expensive
                
                if avg_dist > 50000 and num_boxes >= 20:
                    candidates.append(VehicleType.Train)      # Cheap for long land
                
                candidates.append(VehicleType.SemiTruck)      # Universal fallback
                
                for vtype in candidates:
                    # For planes/ships, check if we need to route to facility first
                    if vtype == VehicleType.Airplane:
                        nearest_airport, dist_to_airport = find_nearest_facility(origin, airports)
                        if dist_to_airport > 5000 and nearest_airport:
                            # Spawn at origin, but will route to airport first
                            pass
                    
                    try:
                        vid = sim_state.create_vehicle(vtype, origin)
                        to_load = box_ids[:vtype.value.capacity]
                        sim_state.load_vehicle(vid, to_load)
                        
                        if to_load:
                            # If plane/ship not at correct facility, go there first
                            if vtype == VehicleType.Airplane:
                                nearest_airport, dist = find_nearest_facility(origin, airports)
                                if dist > 5000 and nearest_airport:
                                    sim_state.move_vehicle(vid, nearest_airport)
                                else:
                                    dest = boxes[to_load[0]]["destination"]
                                    sim_state.move_vehicle(vid, dest)
                            elif vtype == VehicleType.CargoShip:
                                nearest_port, dist = find_nearest_facility(origin, ocean_ports)
                                if dist > 5000 and nearest_port:
                                    sim_state.move_vehicle(vid, nearest_port)
                                else:
                                    dest = boxes[to_load[0]]["destination"]
                                    sim_state.move_vehicle(vid, dest)
                            else:
                                dest = boxes[to_load[0]]["destination"]
                                sim_state.move_vehicle(vid, dest)
                        break
                    except ValueError:
                        continue
