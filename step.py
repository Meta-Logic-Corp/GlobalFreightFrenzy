from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Vehicle cost per km
COST_PER_KM = {
    VehicleType.CargoShip: 0.01,
    VehicleType.Train: 0.02,
    VehicleType.SemiTruck: 0.05,
    VehicleType.Drone: 0.30,
    VehicleType.Airplane: 0.50,
}

CAPACITY = {
    VehicleType.SemiTruck: 50,
    VehicleType.Train: 500,
    VehicleType.Airplane: 100,
    VehicleType.CargoShip: 1000,
    VehicleType.Drone: 5,
}

# Facility requirements for loading/unloading
FACILITY_REQUIRED = {
    VehicleType.SemiTruck: "hub",
    VehicleType.Train: "hub",
    VehicleType.Airplane: "airport",
    VehicleType.CargoShip: "ocean_port",
    VehicleType.Drone: "airport",
}

# Cache
_dist_cache = {}
_airports = set()
_ocean_ports = set()
_hubs = set()

def distance_m(loc1, loc2):
    key = (loc1, loc2) if loc1 < loc2 else (loc2, loc1)
    if key not in _dist_cache:
        _dist_cache[key] = haversine_distance_meters(loc1, loc2)
    return _dist_cache[key]

def is_overseas(origin, dest):
    dist = distance_m(origin, dest)
    if dist > 500000:
        return True
    o_lat, o_lon = origin
    d_lat, d_lon = dest
    if (o_lon < -60 and d_lon > -10) or (o_lon > -10 and d_lon < -60):
        return abs(o_lat - d_lat) < 50
    return False

def find_nearest_facility(loc, facilities):
    if not facilities:
        return None, float('inf')
    nearest = min(facilities, key=lambda f: distance_m(loc, f))
    return nearest, distance_m(loc, nearest)

def step(sim_state):
    global _airports, _ocean_ports, _hubs
    
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Discover facilities from box locations
    for box in boxes.values():
        _hubs.add(box["location"])
        _hubs.add(box["destination"])
        _airports.add(box["location"])
        _airports.add(box["destination"])
        _ocean_ports.add(box["location"])
        _ocean_ports.add(box["destination"])
    
    # ===== 1. UNLOAD =====
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [
                bid for bid in v["cargo"]
                if distance_m(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M
            ]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()
    
    # ===== 2. MANAGE VEHICLES =====
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue
        
        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        capacity_left = CAPACITY[vtype] - len(v["cargo"])
        
        # Load boxes at current location if possible
        if capacity_left > 0:
            loadable = [
                bid for bid, box in boxes.items()
                if not box["delivered"] and box["vehicle_id"] is None
                and distance_m(loc, box["location"]) <= _PROXIMITY_M
            ]
            if loadable:
                to_load = loadable[:capacity_left]
                sim_state.load_vehicle(vid, to_load)
                boxes = sim_state.get_boxes()
        
        if v["cargo"]:
            target = boxes[v["cargo"][0]]["destination"]
            facility_needed = FACILITY_REQUIRED.get(vtype, "hub")
            
            # Check if current location is valid for this vehicle type
            if facility_needed == "airport":
                nearest_airport, dist_to_airport = find_nearest_facility(loc, _airports)
                if dist_to_airport > 5000:  # Not at airport
                    if nearest_airport:
                        sim_state.move_vehicle(vid, nearest_airport)
                    continue
            
            elif facility_needed == "ocean_port":
                nearest_port, dist_to_port = find_nearest_facility(loc, _ocean_ports)
                if dist_to_port > 5000:  # Not at port
                    if nearest_port:
                        sim_state.move_vehicle(vid, nearest_port)
                    continue
            
            # Now handle routing logic
            if vtype in [VehicleType.SemiTruck, VehicleType.Train]:
                if is_overseas(loc, target):
                    # Need to transfer to ship/plane
                    # First, go to appropriate facility
                    nearest_port, port_dist = find_nearest_facility(loc, _ocean_ports)
                    nearest_airport, airport_dist = find_nearest_facility(loc, _airports)
                    
                    # Choose closer facility
                    if port_dist < airport_dist and port_dist < 50000:
                        sim_state.move_vehicle(vid, nearest_port)
                    elif airport_dist < 50000:
                        sim_state.move_vehicle(vid, nearest_airport)
                    else:
                        # Just go to nearest hub
                        nearest_hub = min(_hubs, key=lambda h: distance_m(loc, h))
                        sim_state.move_vehicle(vid, nearest_hub)
                else:
                    sim_state.move_vehicle(vid, target)
            
            elif vtype in [VehicleType.CargoShip, VehicleType.Airplane]:
                if not is_overseas(loc, target):
                    # Reached land, need to transfer to land vehicle
                    # First, unload cargo
                    sim_state.unload_vehicle(vid, v["cargo"])
                    boxes = sim_state.get_boxes()
                    
                    # Find nearest hub for land vehicle
                    nearest_hub = min(_hubs, key=lambda h: distance_m(loc, h))
                    
                    # Create land vehicle at the hub
                    for new_type in [VehicleType.Train, VehicleType.SemiTruck]:
                        try:
                            new_vid = sim_state.create_vehicle(new_type, nearest_hub)
                            sim_state.load_vehicle(new_vid, v["cargo"])
                            sim_state.move_vehicle(new_vid, target)
                            break
                        except ValueError:
                            continue
                else:
                    sim_state.move_vehicle(vid, target)
        
        elif not v["cargo"]:
            # Empty vehicle - go get boxes
            nearest_box = None
            min_dist = float('inf')
            for bid, box in boxes.items():
                if not box["delivered"] and box["vehicle_id"] is None:
                    dist = distance_m(loc, box["location"])
                    if dist < min_dist:
                        min_dist = dist
                        nearest_box = box["location"]
            
            if nearest_box:
                sim_state.move_vehicle(vid, nearest_box)
            elif _hubs:
                nearest_hub = min(_hubs, key=lambda h: distance_m(loc, h))
                sim_state.move_vehicle(vid, nearest_hub)
    
    # ===== 3. SPAWN NEW VEHICLES =====
    if tick == 0 or (len(vehicles) < 20 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() 
                      if not box["delivered"] and box["vehicle_id"] is None]
        
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            for origin, box_ids in sorted(origin_boxes.items(), key=lambda x: -len(x[1])):
                num_boxes = len(box_ids)
                target = boxes[box_ids[0]]["destination"]
                overseas = is_overseas(origin, target)
                
                # For overseas routes, check if we're at correct facility
                if overseas:
                    # Try to spawn ship or plane if at correct facility
                    nearest_port, port_dist = find_nearest_facility(origin, _ocean_ports)
                    if port_dist <= 5000 and num_boxes >= 10:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                            to_load = box_ids[:CAPACITY[VehicleType.CargoShip]]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, target)
                            break
                        except ValueError:
                            pass
                    
                    nearest_airport, airport_dist = find_nearest_facility(origin, _airports)
                    if airport_dist <= 5000:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Airplane, origin)
                            to_load = box_ids[:CAPACITY[VehicleType.Airplane]]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, target)
                            break
                        except ValueError:
                            pass
                    
                    # Not at facility, use truck to get there first
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                        to_load = box_ids[:50]
                        sim_state.load_vehicle(vid, to_load)
                        # Go to nearest airport/port
                        if port_dist < airport_dist:
                            sim_state.move_vehicle(vid, nearest_port)
                        else:
                            sim_state.move_vehicle(vid, nearest_airport)
                        break
                    except ValueError:
                        pass
                
                else:
                    # Land route
                    if num_boxes >= 20:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Train, origin)
                            to_load = box_ids[:500]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, target)
                            break
                        except ValueError:
                            pass
                    
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                        to_load = box_ids[:50]
                        sim_state.load_vehicle(vid, to_load)
                        sim_state.move_vehicle(vid, target)
                        break
                    except ValueError:
                        continue
