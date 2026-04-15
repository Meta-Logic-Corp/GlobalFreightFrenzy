from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Vehicle capabilities and restrictions
VEHICLE_RESTRICTIONS = {
    VehicleType.SemiTruck: {
        "terrain": "land",
        "spawn": "hub",
        "load_unload": "hub",
        "max_distance": None
    },
    VehicleType.Train: {
        "terrain": "land", 
        "spawn": "hub",
        "load_unload": "hub",
        "max_distance": None
    },
    VehicleType.Airplane: {
        "terrain": "any",
        "spawn": "airport",
        "load_unload": "airport",
        "max_distance": None
    },
    VehicleType.CargoShip: {
        "terrain": "water",
        "spawn": "ocean_port",
        "load_unload": "ocean_port",
        "max_distance": None
    },
    VehicleType.Drone: {
        "terrain": "any",
        "spawn": "airport",
        "load_unload": "airport",
        "max_distance": 20000
    }
}

def is_water_crossing(from_loc, to_loc):
    """Detect if path requires water crossing"""
    dist = haversine_distance_meters(from_loc, to_loc)
    if dist > 500000:
        return True
    
    from_lat, from_lon = from_loc
    to_lat, to_lon = to_loc
    
    if (from_lon < -60 and to_lon > -10) or (from_lon > -10 and to_lon < -60):
        if abs(from_lat - to_lat) < 50:
            return True
    
    if (from_lon < -120 and to_lon > 120) or (from_lon > 120 and to_lon < -120):
        return True
    
    return False

def can_vehicle_handle_route(vtype, from_loc, to_loc, num_boxes, sim_state):
    """Check if vehicle can handle this route given restrictions"""
    restrictions = VEHICLE_RESTRICTIONS[vtype]
    
    if restrictions["max_distance"]:
        dist = haversine_distance_meters(from_loc, to_loc)
        if dist > restrictions["max_distance"]:
            return False
    
    if num_boxes > vtype.value.capacity:
        return False
    
    return True

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Discover hubs from box locations
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
    
    # MANAGE VEHICLES: Load, move, transload
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
                # Get destination for cargo
                first_bid = v["cargo"][0]
                dest = boxes[first_bid]["destination"]
                num_boxes = len(v["cargo"])
                
                # Check if current vehicle can handle the route
                if can_vehicle_handle_route(vtype, loc, dest, num_boxes, sim_state):
                    sim_state.move_vehicle(vid, dest)
                else:
                    # Need to transload at nearest hub
                    nearest_hub = None
                    min_dist = float('inf')
                    for hub in hubs:
                        if hub != loc:
                            dist = haversine_distance_meters(loc, hub)
                            if dist < min_dist:
                                min_dist = dist
                                nearest_hub = hub
                    
                    if nearest_hub:
                        # SAVE CARGO BEFORE UNLOADING
                        cargo_copy = list(v["cargo"])
                        dest_copy = dest
                        
                        # Unload current cargo
                        sim_state.unload_vehicle(vid, cargo_copy)
                        boxes = sim_state.get_boxes()
                        
                        # Try to create appropriate vehicle
                        if is_water_crossing(loc, dest):
                            # Try ship first
                            try:
                                new_vid = sim_state.create_vehicle(VehicleType.CargoShip, nearest_hub)
                                sim_state.load_vehicle(new_vid, cargo_copy)
                                sim_state.move_vehicle(new_vid, dest_copy)
                            except ValueError:
                                # Try plane - find nearest airport
                                airports = set()
                                for box in boxes.values():
                                    airports.add(box["location"])
                                    airports.add(box["destination"])
                                
                                nearest_airport = None
                                min_airport_dist = float('inf')
                                for airport in airports:
                                    dist = haversine_distance_meters(loc, airport)
                                    if dist < min_airport_dist:
                                        min_airport_dist = dist
                                        nearest_airport = airport
                                
                                if nearest_airport:
                                    try:
                                        new_vid = sim_state.create_vehicle(VehicleType.Airplane, nearest_airport)
                                        sim_state.load_vehicle(new_vid, cargo_copy)
                                        sim_state.move_vehicle(new_vid, dest_copy)
                                    except ValueError:
                                        sim_state.move_vehicle(vid, dest_copy)
                                else:
                                    sim_state.move_vehicle(vid, dest_copy)
                        else:
                            # Land route - try train
                            if num_boxes >= 20:
                                try:
                                    new_vid = sim_state.create_vehicle(VehicleType.Train, nearest_hub)
                                    sim_state.load_vehicle(new_vid, cargo_copy)
                                    sim_state.move_vehicle(new_vid, dest_copy)
                                except ValueError:
                                    sim_state.move_vehicle(vid, dest_copy)
                            else:
                                sim_state.move_vehicle(vid, dest_copy)
                    else:
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
    
    # SPAWN: Create vehicles respecting cargo constraints
    if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                
                # Check if route requires water crossing
                needs_water = False
                for bid in box_ids[:5]:
                    if is_water_crossing(origin, boxes[bid]["destination"]):
                        needs_water = True
                        break
                
                # Choose vehicle based on route and cargo size
                if needs_water:
                    # For water crossings, use ship or plane
                    if num_boxes >= 10 and num_boxes <= 1000:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                            to_load = box_ids[:1000]
                            sim_state.load_vehicle(vid, to_load)
                            if to_load:
                                dest = boxes[to_load[0]]["destination"]
                                sim_state.move_vehicle(vid, dest)
                            break
                        except ValueError:
                            pass
                    
                    # Try airplane - find nearest airport
                    if num_boxes <= 100:
                        airports = set()
                        for box in boxes.values():
                            airports.add(box["location"])
                            airports.add(box["destination"])
                        
                        nearest_airport = origin
                        min_dist = float('inf')
                        for airport in airports:
                            dist = haversine_distance_meters(origin, airport)
                            if dist < min_dist:
                                min_dist = dist
                                nearest_airport = airport
                        
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Airplane, nearest_airport)
                            to_load = box_ids[:100]
                            sim_state.load_vehicle(vid, to_load)
                            if to_load:
                                dest = boxes[to_load[0]]["destination"]
                                sim_state.move_vehicle(vid, dest)
                            break
                        except ValueError:
                            pass
                
                # Land route - use train for bulk/long distance
                if num_boxes >= 20 and num_boxes <= 500:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.Train, origin)
                        to_load = box_ids[:500]
                        sim_state.load_vehicle(vid, to_load)
                        if to_load:
                            dest = boxes[to_load[0]]["destination"]
                            sim_state.move_vehicle(vid, dest)
                        break
                    except ValueError:
                        pass
                
                # Small loads or fallback - use truck
                if num_boxes <= 50:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                        to_load = box_ids[:50]
                        sim_state.load_vehicle(vid, to_load)
                        if to_load:
                            dest = boxes[to_load[0]]["destination"]
                            sim_state.move_vehicle(vid, dest)
                        break
                    except ValueError:
                        continue
