from simulator import VehicleType, haversine_distance_meters

_PROXIMITY_M = 50.0

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # UNLOAD: Check all vehicles for deliveries
    for vid, vehicle in vehicles.items():
        if vehicle["destination"] is None and vehicle["cargo"]:
            loc = vehicle["location"]
            to_unload = []
            for bid in vehicle["cargo"]:
                if haversine_distance_meters(loc, boxes[bid]["destination"]) <= _PROXIMITY_M:
                    to_unload.append(bid)
            
            if to_unload:
                try:
                    sim_state.unload_vehicle(vid, to_unload)
                except ValueError:
                    pass
    
    # SPAWN: Only at tick 0 or when no vehicles exist
    if tick == 0 or len(vehicles) == 0:
        boxes = sim_state.get_boxes()
        
        # Group boxes by origin
        origins = {}
        for bid, box in boxes.items():
            if not box["delivered"] and box["vehicle_id"] is None:
                loc = box["location"]
                if loc not in origins:
                    origins[loc] = []
                origins[loc].append(bid)
        
        # Spawn one vehicle per origin
        for origin_loc, box_ids in origins.items():
            # Use SemiTruck - most reliable, works on land
            try:
                vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin_loc)
                
                # Load up to 50 boxes
                to_load = box_ids[:50]
                sim_state.load_vehicle(vid, to_load)
                
                # Move to first box's destination
                if to_load:
                    dest = boxes[to_load[0]]["destination"]
                    sim_state.move_vehicle(vid, dest)
            except ValueError:
                pass
    
    # MOVE: Send idle vehicles to their next destination
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    for vid, vehicle in vehicles.items():
        # If vehicle has cargo but no destination, send it somewhere
        if vehicle["destination"] is None and vehicle["cargo"]:
            # Find first undelivered box's destination
            for bid in vehicle["cargo"]:
                dest = boxes[bid]["destination"]
                if haversine_distance_meters(vehicle["location"], dest) > _PROXIMITY_M:
                    try:
                        sim_state.move_vehicle(vid, dest)
                        break
                    except ValueError:
                        pass
