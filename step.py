from simulator import VehicleType, haversine_distance_meters
import math
import random

_PROXIMITY_M = 50.0

def get_best_vehicle(distance_m, num_boxes, spawn_location, sim_state):
    """Try different vehicles in order of cost-efficiency"""
    
    # Strategy: based on distance and cargo size
    if num_boxes >= 100 and distance_m > 100000:  # >100km with many boxes
        candidates = [VehicleType.CargoShip, VehicleType.Train, VehicleType.SemiTruck]
    elif num_boxes >= 20 and distance_m > 50000:  # >50km with decent cargo
        candidates = [VehicleType.Train, VehicleType.SemiTruck]
    elif num_boxes <= 5 and distance_m < 20000:  # <20km with small cargo
        candidates = [VehicleType.Drone, VehicleType.SemiTruck]
    elif distance_m > 200000:  # >200km long distance
        candidates = [VehicleType.Airplane, VehicleType.Train, VehicleType.CargoShip]
    else:
        candidates = [VehicleType.SemiTruck, VehicleType.Train]
    
    # Try each candidate vehicle
    for vtype in candidates:
        try:
            vid = sim_state.create_vehicle(vtype, spawn_location)
            return vid, vtype
        except ValueError:
            continue  # Try next vehicle
    
    return None, None

def step(sim_state):
    tick = sim_state.tick
    
    if tick == 0:
        boxes = sim_state.get_boxes()
        
        # Group boxes by origin location
        origins = {}
        for bid, box in boxes.items():
            loc = box["location"]
            if loc not in origins:
                origins[loc] = []
            origins[loc].append(bid)
        
        # Spawn vehicles at each origin
        for origin_loc, box_ids in origins.items():
            num_boxes = len(box_ids)
            
            # Calculate average distance to destinations
            total_dist = 0
            for bid in box_ids:
                dest = boxes[bid]["destination"]
                total_dist += haversine_distance_meters(origin_loc, dest)
            avg_dist_m = total_dist / num_boxes if num_boxes > 0 else 0
            
            # Get best vehicle for this job
            vid, vtype = get_best_vehicle(avg_dist_m, num_boxes, origin_loc, sim_state)
            
            if vid:
                # Load as many boxes as capacity allows
                capacity = VehicleType[vtype].value.capacity
                load_count = min(num_boxes, capacity)
                to_load = box_ids[:load_count]
                
                sim_state.load_vehicle(vid, to_load)
                
                # Move to first destination
                if to_load:
                    sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                
                print(f"Spawned {vtype} at {origin_loc} with {load_count} boxes")
    
    # Every tick: handle vehicles
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    for vid, vehicle in vehicles.items():
        loc = vehicle["location"]
        vtype_str = vehicle["vehicle_type"]
        
        # Skip if moving
        if vehicle["destination"] is not None:
            continue
        
        # Unload at destination
        deliverable = []
        for bid in vehicle["cargo"]:
            if haversine_distance_meters(loc, boxes[bid]["destination"]) <= _PROXIMITY_M:
                deliverable.append(bid)
        
        if deliverable:
            try:
                sim_state.unload_vehicle(vid, deliverable)
                print(f"Unloaded {len(deliverable)} boxes at {loc}")
            except ValueError as e:
                print(f"Unload failed at {loc}: {e}")
            boxes = sim_state.get_boxes()
        
        # Load more boxes if at a facility that allows it
        vtype = VehicleType[vtype_str]
        config = vtype.value
        remaining_capacity = config.capacity - len(vehicle["cargo"])
        
        if remaining_capacity > 0:
            # Check if we're at a valid loading facility
            try:
                loadable = [
                    bid for bid, box in boxes.items()
                    if not box["delivered"]
                    and box["vehicle_id"] is None
                    and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M
                ]
                if loadable:
                    sim_state.load_vehicle(vid, loadable[:remaining_capacity])
                    boxes = sim_state.get_boxes()
            except ValueError:
                pass  # Can't load here (wrong facility type)
        
        # Move to next destination
        vehicles = sim_state.get_vehicles()
        vehicle = vehicles[vid]
        if vehicle["cargo"]:
            # Find first undelivered box
            for bid in vehicle["cargo"]:
                dest = boxes[bid]["destination"]
                if haversine_distance_meters(loc, dest) > _PROXIMITY_M:
                    sim_state.move_vehicle(vid, dest)
                    break
