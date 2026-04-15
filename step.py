from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Discovered facilities
airports = set()
ocean_ports = set()
hubs = set()

def find_nearest_facility(location, facilities):
    """Find nearest facility to location"""
    nearest = None
    min_dist = float('inf')
    for facility in facilities:
        dist = haversine_distance_meters(location, facility)
        if dist < min_dist:
            min_dist = dist
            nearest = facility
    return nearest, min_dist

def is_overseas_route(origin, destination):
    """Check if route requires crossing ocean"""
    dist = haversine_distance_meters(origin, destination)
    
    # Very long distance likely needs crossing
    if dist > 500000:  # 500km
        return True
    
    from_lat, from_lon = origin
    to_lat, to_lon = destination
    
    # Atlantic crossing
    if (from_lon < -60 and to_lon > -10) or (from_lon > -10 and to_lon < -60):
        if abs(from_lat - to_lat) < 50:
            return True
    
    return False

def step(sim_state):
    global airports, ocean_ports, hubs
    
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Discover facilities from box locations
    for box in boxes.values():
        hubs.add(box["location"])
        hubs.add(box["destination"])
        airports.add(box["location"])
        airports.add(box["destination"])
        ocean_ports.add(box["location"])
        ocean_ports.add(box["destination"])
    
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
    
    # MANAGE VEHICLES: Multi-modal routing
    for vid, v in vehicles.items():
        if v["destination"] is None:
            loc = v["location"]
            vtype = VehicleType[v["vehicle_type"]]
            capacity_left = vtype.value.capacity - len(v["cargo"])
            
            # Load available boxes
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
                first_bid = v["cargo"][0]
                final_dest = boxes[first_bid]["destination"]
                
                # Check if this vehicle can reach final destination
                if vtype in [VehicleType.SemiTruck, VehicleType.Train]:
                    # Land vehicle - check if overseas route
                    if is_overseas_route(loc, final_dest):
                        # Need to go to airport/port first
                        nearest_port, port_dist = find_nearest_facility(loc, ocean_ports)
                        nearest_airport, airport_dist = find_nearest_facility(loc, airports)
                        
                        # Choose closer facility
                        if port_dist < airport_dist and port_dist < 100000:
                            sim_state.move_vehicle(vid, nearest_port)
                        else:
                            sim_state.move_vehicle(vid, nearest_airport)
                    else:
                        # Land route only
                        sim_state.move_vehicle(vid, final_dest)
                
                elif vtype in [VehicleType.CargoShip, VehicleType.Airplane]:
                    # Water/air vehicle - crossing ocean, need to go to land facility after
                    if is_overseas_route(loc, final_dest):
                        # Still crossing, continue to destination
                        sim_state.move_vehicle(vid, final_dest)
                    else:
                        # Reached other side, need land vehicle
                        # Unload and let land vehicle take over
                        if v["cargo"]:
                            sim_state.unload_vehicle(vid, v["cargo"])
                            boxes = sim_state.get_boxes()
                            
                            # Spawn land vehicle at current location
                            for new_vtype in [VehicleType.Train, VehicleType.SemiTruck]:
                                try:
                                    new_vid = sim_state.create_vehicle(new_vtype, loc)
                                    sim_state.load_vehicle(new_vid, v["cargo"])
                                    sim_state.move_vehicle(new_vid, final_dest)
                                    break
                                except ValueError:
                                    continue
            
            # Empty vehicle - go get more boxes or go to hub
            elif not v["cargo"]:
                # Find nearest undelivered box
                nearest = None
                min_dist = float('inf')
                for bid, box in boxes.items():
                    if not box["delivered"] and box["vehicle_id"] is None:
                        dist = haversine_distance_meters(loc, box["location"])
                        if dist < min_dist:
                            min_dist = dist
                            nearest = box["location"]
                
                if not nearest and vtype in [VehicleType.CargoShip, VehicleType.Airplane]:
                    # Go to nearest port/airport to wait for cargo
                    if vtype == VehicleType.CargoShip:
                        nearest, _ = find_nearest_facility(loc, ocean_ports)
                    else:
                        nearest, _ = find_nearest_facility(loc, airports)
                
                if nearest:
                    sim_state.move_vehicle(vid, nearest)
    
    # SPAWN: Create vehicles based on route type
    if tick == 0 or (len(vehicles) < 25 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                
                # Check if route is overseas
                overseas = False
                for bid in box_ids[:3]:
                    if is_overseas_route(origin, boxes[bid]["destination"]):
                        overseas = True
                        break
                
                if overseas and num_boxes >= 10:
                    # For overseas: spawn land vehicle to take to port/airport
                    # Or spawn air/water vehicle directly if at facility
                    nearest_port, port_dist = find_nearest_facility(origin, ocean_ports)
                    nearest_airport, airport_dist = find_nearest_facility(origin, airports)
                    
                    # If already near facility, spawn ship/plane
                    if port_dist < 5000:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                            to_load = box_ids[:1000]
                            sim_state.load_vehicle(vid, to_load)
                            if to_load:
                                sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    
                    if airport_dist < 5000:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Airplane, origin)
                            to_load = box_ids[:100]
                            sim_state.load_vehicle(vid, to_load)
                            if to_load:
                                sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    
                    # Otherwise spawn truck to take boxes to facility
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                        to_load = box_ids[:50]
                        sim_state.load_vehicle(vid, to_load)
                        if to_load:
                            # Go to nearest port or airport
                            target = nearest_port if port_dist < airport_dist else nearest_airport
                            sim_state.move_vehicle(vid, target)
                        break
                    except ValueError:
                        pass
                
                else:
                    # Land route - use train or truck
                    if num_boxes >= 20:
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
