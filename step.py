from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict

_PROXIMITY_M = 50.0

# Cost per km for reference
# Train: 0.02/km (cheapest for land)
# SemiTruck: 0.05/km (2.5x more expensive)

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
            
            # If has cargo, go to most common destination
            if v["cargo"]:
                dest_counts = defaultdict(int)
                for bid in v["cargo"]:
                    dest = boxes[bid]["destination"]
                    dest_counts[dest] += 1
                most_common = max(dest_counts, key=dest_counts.get)
                sim_state.move_vehicle(vid, most_common)
            
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
    
    # SPAWN: Prioritize trains for cost efficiency
    if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        
        if undelivered:
            # Group by origin
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            # Calculate average distance for each origin
            origin_distances = {}
            for origin, box_ids in origin_boxes.items():
                total_dist = 0
                for bid in box_ids:
                    total_dist += haversine_distance_meters(origin, boxes[bid]["destination"])
                origin_distances[origin] = total_dist / len(box_ids) if box_ids else 0
            
            # Sort origins by: more boxes AND longer distance (trains better for long haul)
            sorted_origins = sorted(
                origin_boxes.items(),
                key=lambda x: (len(x[1]), origin_distances[x[0]]),
                reverse=True
            )
            
            for origin, box_ids in sorted_origins:
                num_boxes = len(box_ids)
                avg_dist = origin_distances[origin]
                
                # Train constraints:
                # 1. Must spawn within 5km of hub
                # 2. Best for: distance > 30km AND boxes >= 20
                # 3. If distance is very short (<10km), use truck even if many boxes
                
                use_train = False
                use_ship = False
                use_truck = False
                
                # Check if location can spawn train (near hub)
                # We don't know hub locations, so try train first, fallback if fails
                
                if avg_dist > 30000 and num_boxes >= 20:  # >30km and 20+ boxes
                    use_train = True
                elif avg_dist > 100000 and num_boxes >= 50:  # >100km long distance
                    use_train = True
                elif num_boxes > 200:  # Very large batch
                    use_train = True
                else:
                    use_truck = True
                
                # Try train first (cheapest per km)
                if use_train:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.Train, origin)
                        to_load = box_ids[:VehicleType.Train.value.capacity]
                        sim_state.load_vehicle(vid, to_load)
                        if to_load:
                            # Go to most common destination
                            dest_counts = defaultdict(int)
                            for bid in to_load:
                                dest = boxes[bid]["destination"]
                                dest_counts[dest] += 1
                            most_common = max(dest_counts, key=dest_counts.get)
                            sim_state.move_vehicle(vid, most_common)
                        break  # Spawn one per tick to control cost
                    except ValueError:
                        # Train spawn failed (not near hub), fallback to truck
                        pass
                
                # Fallback to truck
                try:
                    vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                    to_load = box_ids[:VehicleType.SemiTruck.value.capacity]
                    sim_state.load_vehicle(vid, to_load)
                    if to_load:
                        dest_counts = defaultdict(int)
                        for bid in to_load:
                            dest = boxes[bid]["destination"]
                            dest_counts[dest] += 1
                        most_common = max(dest_counts, key=dest_counts.get)
                        sim_state.move_vehicle(vid, most_common)
                    break
                except ValueError:
                    continue
