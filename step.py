from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Global caches (set at tick 0)
_ALL_AIRPORTS = set()
_ALL_PORTS = set()
_ALL_HUBS = set()

CAPACITY = {
    VehicleType.SemiTruck: 50,
    VehicleType.Train: 500,
    VehicleType.Airplane: 100,
    VehicleType.CargoShip: 1000,
    VehicleType.Drone: 5,
}

def is_water_crossing(from_loc, to_loc):
    dist = haversine_distance_meters(from_loc, to_loc)
    if dist > 3000000:
        return True
    from_lat, from_lon = from_loc
    to_lat, to_lon = to_loc
    # Atlantic
    if (from_lon < -60 and to_lon > -10) or (from_lon > -10 and to_lon < -60):
        if abs(from_lat - to_lat) < 50:
            return True
    # Pacific
    if (from_lon < -120 and to_lon > 120) or (from_lon > 120 and to_lon < -120):
        return True
    # Middle East ↔ North America
    if (40 < from_lon < 80 and -130 < to_lon < -60) or (40 < to_lon < 80 and -130 < from_lon < -60):
        return True
    # South Asia ↔ Europe
    if (70 < from_lon < 90 and -10 < to_lon < 40) or (70 < to_lon < 90 and -10 < from_lon < 40):
        return True
    return False

def step(sim_state):
    global _ALL_AIRPORTS, _ALL_PORTS, _ALL_HUBS
    
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    if tick == 0:
        _ALL_HUBS.clear()
        _ALL_AIRPORTS.clear()
        _ALL_PORTS.clear()
        for box in boxes.values():
            _ALL_HUBS.add(box["location"])
            _ALL_HUBS.add(box["destination"])
            _ALL_AIRPORTS.add(box["location"])
            _ALL_AIRPORTS.add(box["destination"])
            _ALL_PORTS.add(box["location"])
            _ALL_PORTS.add(box["destination"])
    
    hubs = list(_ALL_HUBS)
    
    # UNLOAD
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [bid for bid in v["cargo"] if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()
    
    # MANAGE VEHICLES (with proper transloading)
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue
        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        capacity_left = CAPACITY[vtype] - len(v["cargo"])
        
        # Load available boxes at current location
        if capacity_left > 0:
            loadable = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M]
            if loadable:
                to_load = loadable[:capacity_left]
                sim_state.load_vehicle(vid, to_load)
                boxes = sim_state.get_boxes()
        
        if v["cargo"]:
            first_bid = v["cargo"][0]
            dest = boxes[first_bid]["destination"]
            num_boxes = len(v["cargo"])
            
            # Check if we are at a transfer facility
            at_port = any(haversine_distance_meters(loc, p) <= 5000 for p in _ALL_PORTS)
            at_airport = any(haversine_distance_meters(loc, a) <= 5000 for a in _ALL_AIRPORTS)
            
            # Land vehicle with overseas destination
            if vtype in [VehicleType.SemiTruck, VehicleType.Train] and is_water_crossing(loc, dest):
                if at_port or at_airport:
                    # Transfer to ship/plane
                    cargo_copy = list(v["cargo"])
                    dest_copy = dest
                    sim_state.unload_vehicle(vid, cargo_copy)
                    boxes = sim_state.get_boxes()
                    if at_port and len(cargo_copy) <= CAPACITY[VehicleType.CargoShip]:
                        try:
                            new_vid = sim_state.create_vehicle(VehicleType.CargoShip, loc)
                            sim_state.load_vehicle(new_vid, cargo_copy)
                            sim_state.move_vehicle(new_vid, dest_copy)
                        except ValueError:
                            pass
                    elif at_airport and len(cargo_copy) <= CAPACITY[VehicleType.Airplane]:
                        try:
                            new_vid = sim_state.create_vehicle(VehicleType.Airplane, loc)
                            sim_state.load_vehicle(new_vid, cargo_copy)
                            sim_state.move_vehicle(new_vid, dest_copy)
                        except ValueError:
                            pass
                    else:
                        sim_state.move_vehicle(vid, dest_copy)
                else:
                    # Go to nearest port or airport
                    nearest_port = min(_ALL_PORTS, key=lambda p: haversine_distance_meters(loc, p)) if _ALL_PORTS else None
                    nearest_airport = min(_ALL_AIRPORTS, key=lambda a: haversine_distance_meters(loc, a)) if _ALL_AIRPORTS else None
                    port_dist = haversine_distance_meters(loc, nearest_port) if nearest_port else float('inf')
                    airport_dist = haversine_distance_meters(loc, nearest_airport) if nearest_airport else float('inf')
                    transfer_target = nearest_port if port_dist < airport_dist else nearest_airport
                    if transfer_target:
                        sim_state.move_vehicle(vid, transfer_target)
                    else:
                        sim_state.move_vehicle(vid, dest)
            
            # Water/air vehicle: if destination no longer overseas (reached land)
            elif vtype in [VehicleType.CargoShip, VehicleType.Airplane] and not is_water_crossing(loc, dest):
                cargo_copy = list(v["cargo"])
                dest_copy = dest
                sim_state.unload_vehicle(vid, cargo_copy)
                boxes = sim_state.get_boxes()
                # Transfer to train or truck
                if len(cargo_copy) >= 20 and len(cargo_copy) <= CAPACITY[VehicleType.Train]:
                    try:
                        new_vid = sim_state.create_vehicle(VehicleType.Train, loc)
                        sim_state.load_vehicle(new_vid, cargo_copy)
                        sim_state.move_vehicle(new_vid, dest_copy)
                    except ValueError:
                        try:
                            new_vid = sim_state.create_vehicle(VehicleType.SemiTruck, loc)
                            sim_state.load_vehicle(new_vid, cargo_copy)
                            sim_state.move_vehicle(new_vid, dest_copy)
                        except ValueError:
                            sim_state.move_vehicle(vid, dest_copy)
                else:
                    try:
                        new_vid = sim_state.create_vehicle(VehicleType.SemiTruck, loc)
                        sim_state.load_vehicle(new_vid, cargo_copy)
                        sim_state.move_vehicle(new_vid, dest_copy)
                    except ValueError:
                        sim_state.move_vehicle(vid, dest_copy)
            
            # Normal case
            else:
                sim_state.move_vehicle(vid, dest)
        
        elif not v["cargo"]:
            # Empty vehicle: go to nearest box or hub
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
                sim_state.move_vehicle(vid, min(hubs, key=lambda h: haversine_distance_meters(loc, h)))
    
    # SPAWN new vehicles
    if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                needs_water = any(is_water_crossing(origin, boxes[bid]["destination"]) for bid in box_ids[:5])
                
                if needs_water:
                    # Spawn a truck to go to nearest port/airport
                    nearest_port = min(_ALL_PORTS, key=lambda p: haversine_distance_meters(origin, p)) if _ALL_PORTS else None
                    nearest_airport = min(_ALL_AIRPORTS, key=lambda a: haversine_distance_meters(origin, a)) if _ALL_AIRPORTS else None
                    port_dist = haversine_distance_meters(origin, nearest_port) if nearest_port else float('inf')
                    airport_dist = haversine_distance_meters(origin, nearest_airport) if nearest_airport else float('inf')
                    if port_dist < airport_dist and port_dist < 500000:
                        transfer_point = nearest_port
                    elif airport_dist < 500000:
                        transfer_point = nearest_airport
                    else:
                        transfer_point = None
                    
                    if transfer_point:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                            to_load = box_ids[:50]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, transfer_point)
                            break
                        except ValueError:
                            pass
                    # Fallback: try to spawn ship/plane directly
                    if num_boxes <= 1000:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                            to_load = box_ids[:1000]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    if num_boxes <= 100:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Airplane, origin)
                            to_load = box_ids[:100]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                
                # Land route
                if num_boxes >= 20 and num_boxes <= 500:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.Train, origin)
                        to_load = box_ids[:500]
                        sim_state.load_vehicle(vid, to_load)
                        sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                        break
                    except ValueError:
                        pass
                
                if num_boxes <= 50:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                        to_load = box_ids[:50]
                        sim_state.load_vehicle(vid, to_load)
                        sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                        break
                    except ValueError:
                        continue
