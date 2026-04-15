from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict

_PROXIMITY_M = 50.0

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # UNLOAD: Deliver boxes at destination
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = []
            for bid in v["cargo"]:
                if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M:
                    to_unload.append(bid)
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
    
    # LOAD & MOVE: For idle vehicles with capacity
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
                    boxes = sim_state.get_boxes()  # Refresh
            
            # If has cargo and no destination, go somewhere
            if v["cargo"] and v["destination"] is None:
                # Go to most common destination among cargo
                dest_counts = defaultdict(int)
                for bid in v["cargo"]:
                    dest = boxes[bid]["destination"]
                    dest_counts[dest] += 1
                most_common = max(dest_counts, key=dest_counts.get)
                sim_state.move_vehicle(vid, most_common)
    
    # SPAWN: Create efficient vehicles at tick 0
    if tick == 0:
        # Group boxes by origin
        origin_boxes = defaultdict(list)
        for bid, box in boxes.items():
            if not box["delivered"]:
                origin_boxes[box["location"]].append(bid)
        
        for origin, box_ids in origin_boxes.items():
            num_boxes = len(box_ids)
            
            # Choose cheapest vehicle that fits all boxes
            if num_boxes <= 50:
                vtype = VehicleType.SemiTruck
            elif num_boxes <= 500:
                vtype = VehicleType.Train
            else:
                vtype = VehicleType.CargoShip
            
            try:
                vid = sim_state.create_vehicle(vtype, origin)
                to_load = box_ids[:vtype.value.capacity]
                sim_state.load_vehicle(vid, to_load)
                # Move to first box destination
                if to_load:
                    sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
            except ValueError:
                # Fallback to truck
                try:
                    vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                    to_load = box_ids[:50]
                    sim_state.load_vehicle(vid, to_load)
                    if to_load:
                        sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                except ValueError:
                    pass
