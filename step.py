from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Cache for waypoints
_waypoint_cache = {}

def nearest_facility(loc, facilities):
    if not facilities:
        return None
    return min(facilities, key=lambda f: haversine_distance_meters(loc, f))

def get_waypoint_pair(origin, dest, sim_state):
    """Return (start_waypoint, end_waypoint) for an overseas route."""
    key = (origin, dest)
    if key in _waypoint_cache:
        return _waypoint_cache[key]
    
    airports = sim_state.get_airports()
    ports = sim_state.get_ocean_ports()
    if not airports:
        airports = sim_state.get_shipping_hubs()
    
    start_airport = nearest_facility(origin, airports)
    start_port = nearest_facility(origin, ports)
    end_airport = nearest_facility(dest, airports)
    end_port = nearest_facility(dest, ports)
    
    dist_start_air = haversine_distance_meters(origin, start_airport) if start_airport else float('inf')
    dist_start_port = haversine_distance_meters(origin, start_port) if start_port else float('inf')
    start_wp = start_port if dist_start_port < dist_start_air else start_airport
    
    dist_end_air = haversine_distance_meters(dest, end_airport) if end_airport else float('inf')
    dist_end_port = haversine_distance_meters(dest, end_port) if end_port else float('inf')
    end_wp = end_port if dist_end_port < dist_end_air else end_airport
    
    _waypoint_cache[key] = (start_wp, end_wp)
    return start_wp, end_wp

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Get facilities once per tick
    hubs = sim_state.get_shipping_hubs()
    airports = sim_state.get_airports()
    ports = sim_state.get_ocean_ports()
    if not airports:
        airports = hubs  # fallback
    
    # ---------- UNLOAD ----------
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [bid for bid in v["cargo"] if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()
    
    # ---------- MANAGE VEHICLES ----------
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue
        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        capacity_left = vtype.value.capacity - len(v["cargo"])
        
        # Load available boxes at current location
        if capacity_left > 0:
            loadable = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M]
            if loadable:
                to_load = loadable[:capacity_left]
                sim_state.load_vehicle(vid, to_load)
                boxes = sim_state.get_boxes()
        
        if v["cargo"]:
            first_bid = v["cargo"][0]
            final_dest = boxes[first_bid]["destination"]
            
            # Check if current vehicle is at a transfer facility
            at_airport = any(haversine_distance_meters(loc, a) <= 5000 for a in airports)
            at_port = any(haversine_distance_meters(loc, p) <= 5000 for p in ports)
            
            # ---------- LAND VEHICLE (Truck/Train) ----------
            if vtype in [VehicleType.SemiTruck, VehicleType.Train]:
                # If destination is overseas (>500 km) and we are at a port/airport → transfer to ship/plane
                if haversine_distance_meters(loc, final_dest) > 500000 and (at_airport or at_port):
                    cargo_copy = list(v["cargo"])
                    # Determine which vehicle to create
                    new_type = VehicleType.CargoShip if at_port else VehicleType.Airplane
                    try:
                        new_vid = sim_state.create_vehicle(new_type, loc)
                        sim_state.load_vehicle(new_vid, cargo_copy)   # load first
                        # Only after successful load, unload the original
                        sim_state.unload_vehicle(vid, cargo_copy)
                        boxes = sim_state.get_boxes()
                        # Get far-side waypoint (or go directly if none)
                        _, end_wp = get_waypoint_pair(loc, final_dest, sim_state)
                        sim_state.move_vehicle(new_vid, end_wp if end_wp else final_dest)
                    except ValueError:
                        # Transfer failed – keep cargo on original vehicle and go to a better facility
                        target = nearest_facility(loc, airports if new_type == VehicleType.Airplane else ports)
                        if target:
                            sim_state.move_vehicle(vid, target)
                        else:
                            sim_state.move_vehicle(vid, final_dest)
                else:
                    # Normal land movement
                    sim_state.move_vehicle(vid, final_dest)
            
            # ---------- WATER/AIR VEHICLE (Ship/Plane) ----------
            elif vtype in [VehicleType.CargoShip, VehicleType.Airplane]:
                # If we have reached a facility near the final destination (distance < 500 km) → transfer to land vehicle
                if (at_airport or at_port) and haversine_distance_meters(loc, final_dest) < 500000:
                    cargo_copy = list(v["cargo"])
                    land_type = VehicleType.Train if len(cargo_copy) >= 20 else VehicleType.SemiTruck
                    try:
                        new_vid = sim_state.create_vehicle(land_type, loc)
                        sim_state.load_vehicle(new_vid, cargo_copy)   # load first
                        sim_state.unload_vehicle(vid, cargo_copy)
                        boxes = sim_state.get_boxes()
                        sim_state.move_vehicle(new_vid, final_dest)
                    except ValueError:
                        # Transfer failed – keep cargo and go to nearest hub
                        target = nearest_facility(loc, hubs)
                        if target:
                            sim_state.move_vehicle(vid, target)
                        else:
                            sim_state.move_vehicle(vid, final_dest)
                else:
                    # Continue moving
                    sim_state.move_vehicle(vid, final_dest)
        
        # ---------- EMPTY VEHICLE ----------
        elif not v["cargo"]:
            # Go to nearest undelivered box
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
                sim_state.move_vehicle(vid, nearest_facility(loc, hubs))
    
    # ---------- SPAWN NEW VEHICLES ----------
    if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                # Check if any box from this origin needs overseas travel
                needs_overseas = any(haversine_distance_meters(origin, boxes[bid]["destination"]) > 500000 for bid in box_ids[:5])
                
                if needs_overseas:
                    # Get the start waypoint (nearest port/airport)
                    start_wp, _ = get_waypoint_pair(origin, boxes[box_ids[0]]["destination"], sim_state)
                    if start_wp:
                        # Spawn a truck to go to the start waypoint (NOT directly overseas)
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                            to_load = box_ids[:VehicleType.SemiTruck.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, start_wp)
                            break
                        except ValueError:
                            pass
                    # Fallback: try to spawn ship/plane directly at origin
                    if num_boxes <= VehicleType.CargoShip.value.capacity:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                            to_load = box_ids[:VehicleType.CargoShip.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    if num_boxes <= VehicleType.Airplane.value.capacity:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Airplane, origin)
                            to_load = box_ids[:VehicleType.Airplane.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    # If all fails, skip this origin (do not spawn a truck to overseas)
                    continue
                else:
                    # Land route: train or truck
                    if num_boxes >= 20 and num_boxes <= VehicleType.Train.value.capacity:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Train, origin)
                            to_load = box_ids[:VehicleType.Train.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    if num_boxes <= VehicleType.SemiTruck.value.capacity:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                            to_load = box_ids[:VehicleType.SemiTruck.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            continue
