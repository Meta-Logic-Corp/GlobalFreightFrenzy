from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict

_PROXIMITY_M = 50.0

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # 1. UNLOAD
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [bid for bid in v["cargo"] 
                        if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()
    
    # 2. MANAGE VEHICLES
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue
        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        capacity_left = vtype.value.capacity - len(v["cargo"])
        
        # Load boxes
        if capacity_left > 0:
            loadable = [bid for bid, box in boxes.items()
                        if not box["delivered"] and box["vehicle_id"] is None
                        and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M]
            if loadable:
                to_load = loadable[:capacity_left]
                sim_state.load_vehicle(vid, to_load)
                boxes = sim_state.get_boxes()
        
        # Move if has cargo
        if v["cargo"]:
            first_bid = v["cargo"][0]
            dest = boxes[first_bid]["destination"]
            sim_state.move_vehicle(vid, dest)
        else:
            # Empty – go to nearest undelivered box
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
    
    # 3. SPAWN – only at tick 0, and only a few vehicles
    if tick == 0:
        # Group boxes by origin
        origin_boxes = defaultdict(list)
        for bid, box in boxes.items():
            if not box["delivered"]:
                origin_boxes[box["location"]].append(bid)
        
        for origin, bids in origin_boxes.items():
            # Use a single train per origin (cheap per km, large capacity)
            try:
                vid = sim_state.create_vehicle(VehicleType.Train, origin)
                to_load = bids[:500]
                sim_state.load_vehicle(vid, to_load)
                # Go to the most common destination among these boxes
                dest_counts = defaultdict(int)
                for bid in to_load:
                    dest_counts[boxes[bid]["destination"]] += 1
                primary_dest = max(dest_counts, key=dest_counts.get)
                sim_state.move_vehicle(vid, primary_dest)
            except ValueError:
                # Fallback to truck
                try:
                    vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                    to_load = bids[:50]
                    sim_state.load_vehicle(vid, to_load)
                    dest_counts = defaultdict(int)
                    for bid in to_load:
                        dest_counts[boxes[bid]["destination"]] += 1
                    primary_dest = max(dest_counts, key=dest_counts.get)
                    sim_state.move_vehicle(vid, primary_dest)
                except ValueError:
                    pass
