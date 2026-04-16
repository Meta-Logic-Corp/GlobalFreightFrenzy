from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Get actual facility locations from the API
    hubs = list(sim_state.get_shipping_hubs())
    airports = list(sim_state.get_airports())      # may be empty
    ports = list(sim_state.get_ocean_ports())      # may be empty
    
    # Helper: find nearest hub to a location
    def nearest_hub(loc):
        if not hubs:
            return None
        return min(hubs, key=lambda h: haversine_distance_meters(loc, h))
    
    # Helper: check if a route is overseas (would require water crossing)
    def is_overseas(origin, dest):
        dist = haversine_distance_meters(origin, dest)
        if dist > 500000:  # 500 km
            return True
        # Simple heuristic: if both points are on land and far apart, assume land route
        # For this simplified version, we only trigger overseas for very long distances.
        return False
    
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
        
        # Load boxes at current location (only if at a hub or valid facility)
        if capacity_left > 0:
            loadable = [bid for bid, box in boxes.items()
                        if not box["delivered"] and box["vehicle_id"] is None
                        and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M]
            if loadable:
                to_load = loadable[:capacity_left]
                sim_state.load_vehicle(vid, to_load)
                boxes = sim_state.get_boxes()
        
        if v["cargo"]:
            first_bid = v["cargo"][0]
            dest = boxes[first_bid]["destination"]
            # If the route is overseas, do NOT go (avoid terrain penalty) – instead, go to nearest hub and unload?
            # For simplicity, we just go to destination and hope it's land. But to be safe:
            if is_overseas(loc, dest):
                # Instead of driving over water, go to nearest hub and unload (abandon box?)
                hub = nearest_hub(loc)
                if hub:
                    sim_state.move_vehicle(vid, hub)
                else:
                    sim_state.move_vehicle(vid, dest)
            else:
                sim_state.move_vehicle(vid, dest)
        else:
            # Empty vehicle: go to nearest undelivered box's location (or hub)
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
            elif hubs:
                sim_state.move_vehicle(vid, nearest_hub(loc))
    
    # 3. SPAWN – only at tick 0, and only for boxes that are NOT overseas
    if tick == 0:
        # Group boxes by origin location (which should be near a hub)
        origin_boxes = defaultdict(list)
        for bid, box in boxes.items():
            if not box["delivered"]:
                origin_boxes[box["location"]].append(bid)
        
        for origin, bids in origin_boxes.items():
            # Check if any box from this origin goes overseas
            overseas_boxes = [bid for bid in bids if is_overseas(origin, boxes[bid]["destination"])]
            land_boxes = [bid for bid in bids if not is_overseas(origin, boxes[bid]["destination"])]
            
            # Only spawn vehicles for land boxes (overseas boxes will incur undelivered penalty, which is 1000/box)
            # This is better than terrain penalties that could be much higher.
            if land_boxes:
                # Use train if many boxes, else truck
                if len(land_boxes) >= 20 and len(land_boxes) <= 500:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.Train, origin)
                        to_load = land_boxes[:500]
                        sim_state.load_vehicle(vid, to_load)
                        # Go to most common destination among loaded boxes
                        dest_counts = defaultdict(int)
                        for bid in to_load:
                            dest_counts[boxes[bid]["destination"]] += 1
                        primary_dest = max(dest_counts, key=dest_counts.get)
                        sim_state.move_vehicle(vid, primary_dest)
                    except ValueError:
                        # fallback to truck
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                            to_load = land_boxes[:50]
                            sim_state.load_vehicle(vid, to_load)
                            dest_counts = defaultdict(int)
                            for bid in to_load:
                                dest_counts[boxes[bid]["destination"]] += 1
                            primary_dest = max(dest_counts, key=dest_counts.get)
                            sim_state.move_vehicle(vid, primary_dest)
                        except ValueError:
                            pass
                else:
                    # Use truck for smaller batches
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                        to_load = land_boxes[:50]
                        sim_state.load_vehicle(vid, to_load)
                        if to_load:
                            dest_counts = defaultdict(int)
                            for bid in to_load:
                                dest_counts[boxes[bid]["destination"]] += 1
                            primary_dest = max(dest_counts, key=dest_counts.get)
                            sim_state.move_vehicle(vid, primary_dest)
                    except ValueError:
                        pass
