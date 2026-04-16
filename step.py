from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import heapq
import math

_PROXIMITY_M = 50.0

# Global caches
_edge_cost_cache = {}
_vehicle_type_cache = {}
_waypoint_cache = {}

# Persistent route storage for vehicles
vehicle_routes = {}      # vid -> (remaining_waypoints, remaining_vehicle_types)
vehicle_cargo_target = {} # vid -> final destination (for transfer checks)

# ----------------------------------------------------------------------
# Helper functions
# ----------------------------------------------------------------------
def nearest_facility(loc, facilities):
    if not facilities:
        return None
    return min(facilities, key=lambda f: haversine_distance_meters(loc, f))

def get_edge_cost(from_node, to_node, sim_state):
    """Return (min_cost, best_vehicle_type) between two points."""
    key = (from_node, to_node)
    if key in _edge_cost_cache:
        return _edge_cost_cache[key]
    
    dist = haversine_distance_meters(from_node, to_node)
    best_cost = float('inf')
    best_vehicle = None
    
    # Vehicle cost per meter (from API per-km costs)
    costs = {
        VehicleType.SemiTruck: 0.00005,
        VehicleType.Train: 0.00002,
        VehicleType.Airplane: 0.00050,
        VehicleType.CargoShip: 0.00001,
        VehicleType.Drone: 0.00030,
    }
    # We ignore terrain here; the simulator will raise ValueError if a vehicle cannot traverse.
    # For cost estimation we just use the per‑km cost.
    for vtype, cost_per_m in costs.items():
        cost = dist * cost_per_m
        if cost < best_cost:
            best_cost = cost
            best_vehicle = vtype
    
    _edge_cost_cache[key] = (best_cost, best_vehicle)
    return best_cost, best_vehicle

def heuristic(node, dest):
    """Admissible heuristic: straight‑line distance * cheapest possible cost (0.01 per km)."""
    dist = haversine_distance_meters(node, dest)
    return dist * 0.00001   # cheapest = ship (0.01/km = 0.00001/m)

def a_star_route(origin, dest, facilities, sim_state):
    """
    Returns (total_cost, list_of_waypoints, list_of_vehicle_types)
    where waypoints[0] = origin, waypoints[-1] = dest.
    """
    # Nodes: origin, dest, all facilities (hubs/airports/ports)
    nodes = list(set(facilities) | {origin, dest})
    # For A*, we treat origin and dest as normal nodes.
    start = origin
    goal = dest
    
    open_set = []
    heapq.heappush(open_set, (0, 0, start, [start], None))
    g_score = {start: 0}
    came_from = {}
    best_vehicles = {}  # (from, to) -> vehicle_type
    
    while open_set:
        f, cost, node, path, last_vehicle = heapq.heappop(open_set)
        if node == goal:
            # Reconstruct vehicle types for each leg
            vehicles = []
            for i in range(len(path)-1):
                _, v = get_edge_cost(path[i], path[i+1], sim_state)
                vehicles.append(v)
            return cost, path, vehicles
        
        for next_node in nodes:
            if next_node == node:
                continue
            edge_cost, vehicle = get_edge_cost(node, next_node, sim_state)
            tentative_g = cost + edge_cost
            if next_node not in g_score or tentative_g < g_score[next_node]:
                g_score[next_node] = tentative_g
                f_score = tentative_g + heuristic(next_node, goal)
                heapq.heappush(open_set, (f_score, tentative_g, next_node, path + [next_node], vehicle))
                came_from[next_node] = node
                best_vehicles[(node, next_node)] = vehicle
    
    # Fallback: direct with SemiTruck
    dist = haversine_distance_meters(origin, dest)
    fallback_cost = dist * 0.00005
    return fallback_cost, [origin, dest], [VehicleType.SemiTruck]

# ----------------------------------------------------------------------
# Main step function
# ----------------------------------------------------------------------
def step(sim_state):
    global vehicle_routes, vehicle_cargo_target
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Get facilities
    hubs = sim_state.get_shipping_hubs()
    airports = sim_state.get_airports()
    ports = sim_state.get_ocean_ports()
    if not airports:
        airports = hubs  # fallback
    facilities = list(set(hubs) | set(airports) | set(ports))
    
    # ------------------------------------------------------------------
    # 1. UNLOAD – deliver boxes at destination
    # ------------------------------------------------------------------
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [bid for bid in v["cargo"] if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()
                # If this vehicle had a stored route, clear it (all cargo delivered)
                if vid in vehicle_routes:
                    del vehicle_routes[vid]
                if vid in vehicle_cargo_target:
                    del vehicle_cargo_target[vid]
    
    # ------------------------------------------------------------------
    # 2. MANAGE VEHICLES – move, transfer, execute multi‑leg routes
    # ------------------------------------------------------------------
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue
        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        capacity_left = vtype.value.capacity - len(v["cargo"])
        
        # Load any boxes at this location
        if capacity_left > 0:
            loadable = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M]
            if loadable:
                to_load = loadable[:capacity_left]
                sim_state.load_vehicle(vid, to_load)
                boxes = sim_state.get_boxes()
        
        # If the vehicle has cargo, decide where to go
        if v["cargo"]:
            # If we have a stored route, follow it
            if vid in vehicle_routes:
                waypoints, vehicle_types = vehicle_routes[vid]
                if waypoints:
                    next_dest = waypoints[0]
                    # If we are already at that waypoint (within 50m), pop it and proceed
                    if haversine_distance_meters(loc, next_dest) <= _PROXIMITY_M:
                        # Arrived at intermediate waypoint
                        # If this is not the final destination, we may need to transfer to next vehicle
                        if len(waypoints) > 1:
                            # Transfer: unload cargo, create new vehicle for next leg
                            cargo_copy = list(v["cargo"])
                            next_vehicle_type = vehicle_types[1] if len(vehicle_types) > 1 else None
                            # Unload current vehicle
                            sim_state.unload_vehicle(vid, cargo_copy)
                            boxes = sim_state.get_boxes()
                            # Create new vehicle at this location for the next leg
                            if next_vehicle_type:
                                try:
                                    new_vid = sim_state.create_vehicle(next_vehicle_type, loc)
                                    sim_state.load_vehicle(new_vid, cargo_copy)
                                    # Update route for the new vehicle (remaining waypoints and vehicle types)
                                    new_waypoints = waypoints[1:] if len(waypoints) > 1 else []
                                    new_vehicle_types = vehicle_types[1:] if len(vehicle_types) > 1 else []
                                    vehicle_routes[new_vid] = (new_waypoints, new_vehicle_types)
                                    vehicle_cargo_target[new_vid] = vehicle_cargo_target.get(vid, waypoints[-1])
                                    # Move the new vehicle
                                    if new_waypoints:
                                        sim_state.move_vehicle(new_vid, new_waypoints[0])
                                    else:
                                        sim_state.move_vehicle(new_vid, waypoints[-1])  # final dest
                                except ValueError:
                                    # Transfer failed – keep original vehicle and move to next waypoint
                                    sim_state.move_vehicle(vid, next_dest)
                            else:
                                # No next vehicle type, just move to next waypoint
                                sim_state.move_vehicle(vid, next_dest)
                        else:
                            # Last waypoint (final destination) – just go there
                            sim_state.move_vehicle(vid, next_dest)
                        # Remove the current waypoint from stored route
                        vehicle_routes[vid] = (waypoints[1:], vehicle_types[1:])
                    else:
                        # Not at waypoint yet, just move there
                        sim_state.move_vehicle(vid, next_dest)
                else:
                    # No waypoints left – should have unloaded already
                    pass
            else:
                # No stored route – compute one on the fly (should not happen often)
                # For simplicity, just move directly to destination of first box
                first_bid = v["cargo"][0]
                final_dest = boxes[first_bid]["destination"]
                sim_state.move_vehicle(vid, final_dest)
        
        # Empty vehicle: go to nearest undelivered box or hub
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
            elif hubs:
                target = nearest_facility(loc, hubs)
                if target:
                    sim_state.move_vehicle(vid, target)
    
    # ------------------------------------------------------------------
    # 3. SPAWN new vehicles for undelivered boxes (at tick 0 or periodically)
    # ------------------------------------------------------------------
    if tick == 0 or (len(vehicles) < 20 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                # Pick a representative destination (most common)
                dest_counts = defaultdict(int)
                for bid in box_ids:
                    dest_counts[boxes[bid]["destination"]] += 1
                primary_dest = max(dest_counts, key=dest_counts.get)
                
                # Use A* to find optimal multi‑modal route
                total_cost, waypoints, vehicle_types = a_star_route(origin, primary_dest, facilities, sim_state)
                
                if len(waypoints) >= 2:
                    first_vehicle = vehicle_types[0]
                    first_dest = waypoints[1]
                    try:
                        vid = sim_state.create_vehicle(first_vehicle, origin)
                        # Load as many boxes as capacity allows
                        to_load = box_ids[:first_vehicle.value.capacity]
                        sim_state.load_vehicle(vid, to_load)
                        # Store the remaining route for this vehicle
                        remaining_waypoints = waypoints[1:]  # first waypoint is origin, already handled
                        remaining_vehicle_types = vehicle_types[1:] if len(vehicle_types) > 1 else []
                        vehicle_routes[vid] = (remaining_waypoints, remaining_vehicle_types)
                        vehicle_cargo_target[vid] = primary_dest
                        # Start moving
                        sim_state.move_vehicle(vid, first_dest)
                        break  # spawn one vehicle per tick to control cost
                    except ValueError:
                        # Spawn failed – fallback to direct truck
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                            to_load = box_ids[:VehicleType.SemiTruck.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, primary_dest)
                            break
                        except ValueError:
                            continue
                else:
                    # No A* route – use direct truck
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                        to_load = box_ids[:VehicleType.SemiTruck.value.capacity]
                        sim_state.load_vehicle(vid, to_load)
                        sim_state.move_vehicle(vid, primary_dest)
                        break
                    except ValueError:
                        continue
