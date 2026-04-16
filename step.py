from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Cache for waypoints (to avoid recomputing)
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
    
    # Find nearest airport/port to origin and to destination
    start_airport = nearest_facility(origin, airports)
    start_port = nearest_facility(origin, ports)
    end_airport = nearest_facility(dest, airports)
    end_port = nearest_facility(dest, ports)
    
    # Choose the closest facility to origin
    dist_start_air = haversine_distance_meters(origin, start_airport) if start_airport else float('inf')
    dist_start_port = haversine_distance_meters(origin, start_port) if start_port else float('inf')
    if dist_start_port < dist_start_air:
        start_wp = start_port
    else:
        start_wp = start_airport
    
    # Choose the closest facility to destination
    dist_end_air = haversine_distance_meters(dest, end_airport) if end_airport else float('inf')
    dist_end_port = haversine_distance_meters(dest, end_port) if end_port else float('inf')
    if dist_end_port < dist_end_air:
        end_wp = end_port
    else:
        end_wp = end_airport
    
    _waypoint_cache[key] = (start_wp, end_wp)
    return start_wp, end_wp

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # UNLOAD
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [bid for bid in v["cargo"] if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()
    
    # MANAGE VEHICLES
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue
        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        capacity_left = vtype.value.capacity - len(v["cargo"])
        
        # Load
        if capacity_left > 0:
            loadable = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M]
            if loadable:
                to_load = loadable[:capacity_left]
                sim_state.load_vehicle(vid, to_load)
                boxes = sim_state.get_boxes()
        
        if v["cargo"]:
            first_bid = v["cargo"][0]
            final_dest = boxes[first_bid]["destination"]
            
            # Check if this vehicle is a truck that has reached a waypoint (port/airport)
            if vtype in [VehicleType.SemiTruck, VehicleType.Train]:
                # If at a port or airport, and the final destination is overseas from here, transfer
                airports = sim_state.get_airports()
                ports = sim_state.get_ocean_ports()
                if not airports:
                    airports = sim_state.get_shipping_hubs()
                at_airport = any(haversine_distance_meters(loc, a) <= 5000 for a in airports)
                at_port = any(haversine_distance_meters(loc, p) <= 5000 for p in ports)
                
                if (at_airport or at_port) and haversine_distance_meters(loc, final_dest) > 500000:
                    # Transfer to ship or plane
                    cargo_copy = list(v["cargo"])
                    sim_state.unload_vehicle(vid, cargo_copy)
                    boxes = sim_state.get_boxes()
                    # Get the waypoint on the other side
                    _, end_wp = get_waypoint_pair(loc, final_dest, sim_state)
                    # Spawn ship or plane
                    if at_port:
                        try:
                            new_vid = sim_state.create_vehicle(VehicleType.CargoShip, loc)
                            sim_state.load_vehicle(new_vid, cargo_copy)
                            sim_state.move_vehicle(new_vid, end_wp if end_wp else final_dest)
                        except ValueError:
                            pass
                    else:
                        try:
                            new_vid = sim_state.create_vehicle(VehicleType.Airplane, loc)
                            sim_state.load_vehicle(new_vid, cargo_copy)
                            sim_state.move_vehicle(new_vid, end_wp if end_wp else final_dest)
                        except ValueError:
                            pass
                else:
                    # Normal truck movement
                    sim_state.move_vehicle(vid, final_dest)
            
            elif vtype in [VehicleType.CargoShip, VehicleType.Airplane]:
                # Check if we've reached the destination waypoint (port/airport near final destination)
                airports = sim_state.get_airports()
                ports = sim_state.get_ocean_ports()
                if not airports:
                    airports = sim_state.get_shipping_hubs()
                at_airport = any(haversine_distance_meters(loc, a) <= 5000 for a in airports)
                at_port = any(haversine_distance_meters(loc, p) <= 5000 for p in ports)
                
                # If at a facility and the final destination is close (not overseas), transfer to truck
                if (at_airport or at_port) and haversine_distance_meters(loc, final_dest) < 500000:
                    cargo_copy = list(v["cargo"])
                    sim_state.unload_vehicle(vid, cargo_copy)
                    boxes = sim_state.get_boxes()
                    try:
                        new_vid = sim_state.create_vehicle(VehicleType.SemiTruck, loc)
                        sim_state.load_vehicle(new_vid, cargo_copy)
                        sim_state.move_vehicle(new_vid, final_dest)
                    except ValueError:
                        pass
                else:
                    sim_state.move_vehicle(vid, final_dest)
        
        elif not v["cargo"]:
            # Empty vehicle: go to nearest box
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
    
    # SPAWN
    if tick == 0 or (len(vehicles) < 20 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                # Check if any box from this origin requires overseas travel
                needs_overseas = any(haversine_distance_meters(origin, boxes[bid]["destination"]) > 500000 for bid in box_ids[:5])
                
                if needs_overseas:
                    # Get the start waypoint (nearest port/airport)
                    start_wp, _ = get_waypoint_pair(origin, boxes[box_ids[0]]["destination"], sim_state)
                    if start_wp:
                        # Spawn a truck to go to the start waypoint
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                            to_load = box_ids[:50]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, start_wp)
                            break
                        except ValueError:
                            pass
                else:
                    # Land route: use train or truck
                    if num_boxes >= 20:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Train, origin)
                            to_load = box_ids[:500]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                        to_load = box_ids[:50]
                        sim_state.load_vehicle(vid, to_load)
                        sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                        break
                    except ValueError:
                        continue
