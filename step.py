from simulator import VehicleType, haversine_distance_meters
import math

_PROXIMITY_M = 50.0

def get_cheapest_vehicle_type(distance_m, num_boxes):
    """Choose cheapest vehicle based on distance and cargo size"""
    # Cost per meter (converted from per km)
    costs = {
        VehicleType.CargoShip: 0.00001,  # 0.01 per km
        VehicleType.Train: 0.00002,      # 0.02 per km
        VehicleType.SemiTruck: 0.00005,  # 0.05 per km
        VehicleType.Drone: 0.00030,      # 0.30 per km
        VehicleType.Airplane: 0.00050,   # 0.50 per km
    }
    
    capacities = {
        VehicleType.SemiTruck: 50,
        VehicleType.Train: 500,
        VehicleType.Airplane: 100,
        VehicleType.CargoShip: 1000,
        VehicleType.Drone: 5,
    }
    
    # For short distances (< 50km), use truck
    if distance_m < 50000:
        return VehicleType.SemiTruck
    
    # For long distances with many boxes, use train or ship
    if num_boxes > 100:
        return VehicleType.Train
    elif num_boxes > 20:
        return VehicleType.SemiTruck
    else:
        return VehicleType.SemiTruck

def step(sim_state):
    tick = sim_state.tick
    
    # ── Tick 0: spawn vehicles at hubs with smart selection ────
    if tick == 0:
        boxes = sim_state.get_boxes()
        hubs = {}
        
        # Group boxes by origin hub
        for bid, box in boxes.items():
            loc = box["location"]
            if loc not in hubs:
                hubs[loc] = []
            hubs[loc].append(bid)
        
        # Spawn ONE vehicle per hub that can handle ALL boxes there
        for hub_loc, box_ids in hubs.items():
            num_boxes = len(box_ids)
            
            # Calculate average distance to destinations
            avg_dist = 0
            for bid in box_ids:
                dest = boxes[bid]["destination"]
                avg_dist += haversine_distance_meters(hub_loc, dest)
            avg_dist /= num_boxes if num_boxes > 0 else 1
            
            # Choose cheapest vehicle that can handle all boxes
            if num_boxes <= 50:
                vtype = VehicleType.SemiTruck
            elif num_boxes <= 500:
                vtype = VehicleType.Train
            else:
                vtype = VehicleType.CargoShip
            
            try:
                vid = sim_state.create_vehicle(vtype, hub_loc)
                # Load ALL boxes at this hub
                sim_state.load_vehicle(vid, box_ids)
                # Move to first box's destination
                sim_state.move_vehicle(vid, boxes[box_ids[0]]["destination"])
            except ValueError:
                # Fallback to truck
                try:
                    vid = sim_state.create_vehicle(VehicleType.SemiTruck, hub_loc)
                    sim_state.load_vehicle(vid, box_ids[:50])  # Limited by capacity
                    if box_ids:
                        sim_state.move_vehicle(vid, boxes[box_ids[0]]["destination"])
                except ValueError:
                    pass
    
    # ── Every tick: manage each vehicle ──────────────────────────────────
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    for vid, vehicle in vehicles.items():
        loc = vehicle["location"]
        config = VehicleType[vehicle["vehicle_type"]].value
        remaining_capacity = config.capacity - len(vehicle["cargo"])
        has_capacity = remaining_capacity > 0
        
        # Skip vehicles that are still en route
        if vehicle["destination"] is not None:
            continue
        
        # 1. Unload boxes at destination
        deliverable = [
            bid for bid in vehicle["cargo"]
            if haversine_distance_meters(loc, boxes[bid]["destination"]) <= _PROXIMITY_M
        ]
        if deliverable:
            sim_state.unload_vehicle(vid, deliverable)
            boxes = sim_state.get_boxes()
        
        # 2. Load any new boxes at this location (if capacity available)
        if has_capacity:
            loadable = [
                bid for bid, box in boxes.items()
                if not box["delivered"]
                and box["vehicle_id"] is None
                and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M
            ]
            if loadable:
                sim_state.load_vehicle(vid, loadable[:remaining_capacity])
                boxes = sim_state.get_boxes()
        
        # 3. Head toward next destination
        vehicles = sim_state.get_vehicles()
        vehicle = vehicles[vid]
        if vehicle["cargo"]:
            next_dest = boxes[vehicle["cargo"][0]]["destination"]
            sim_state.move_vehicle(vid, next_dest)
