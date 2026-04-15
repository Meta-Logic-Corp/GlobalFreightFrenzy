from simulator import VehicleType, haversine_distance_meters
import heapq
import math

_PROXIMITY_M = 50.0

# Vehicle costs per meter (converted from per km)
COST_PER_METER = {
    VehicleType.SemiTruck: 0.00005,  # 0.05 per km
    VehicleType.Train: 0.00002,      # 0.02 per km
    VehicleType.Airplane: 0.00050,   # 0.50 per km
    VehicleType.CargoShip: 0.00001,  # 0.01 per km
    VehicleType.Drone: 0.00030,      # 0.30 per km
}

# Vehicle capacities
CAPACITY = {
    VehicleType.SemiTruck: 50,
    VehicleType.Train: 500,
    VehicleType.Airplane: 100,
    VehicleType.CargoShip: 1000,
    VehicleType.Drone: 5,
}

# Terrain restrictions
TERRAIN = {
    VehicleType.SemiTruck: "land",
    VehicleType.Train: "land",
    VehicleType.Airplane: "any",
    VehicleType.CargoShip: "water",
    VehicleType.Drone: "any",
}

def can_travel_between(vtype, from_loc, to_loc, sim_state):
    """Check if vehicle can travel between two points"""
    terrain = TERRAIN[vtype]
    if terrain == "any":
        return True
    
    # Simplified: check if path is valid
    # In real implementation, you'd check if path crosses water/land
    return True

def dijkstra_route(origin, destination, hubs, num_boxes, sim_state):
    """
    Find cheapest multi-modal route from origin to destination.
    Returns: (total_cost, path, vehicle_types)
    """
    
    # All nodes = origin, destination, and all hubs
    nodes = [origin, destination] + hubs
    node_to_idx = {node: i for i, node in enumerate(nodes)}
    
    # Dijkstra: (cost, current_node, path, last_vehicle, path_vehicles)
    pq = [(0, origin, [origin], None, [])]
    visited = {}
    
    while pq:
        cost, node, path, last_vehicle, path_vehicles = heapq.heappop(pq)
        
        # Skip if we've found a better path to this node
        if node in visited and visited[node] <= cost:
            continue
        visited[node] = cost
        
        # Reached destination
        if node == destination:
            return cost, path, path_vehicles
        
        # Try all possible next nodes
        for next_node in nodes:
            if next_node == node:
                continue
            
            # Calculate distance between nodes
            dist_m = haversine_distance_meters(node, next_node)
            if dist_m < 10:  # Too close, skip
                continue
            
            # Try each vehicle type for this leg
            for vtype in VehicleType:
                # Check capacity
                if num_boxes > CAPACITY[vtype]:
                    continue
                
                # Check terrain compatibility
                if not can_travel_between(vtype, node, next_node, sim_state):
                    continue
                
                # Calculate leg cost
                leg_cost = dist_m * COST_PER_METER[vtype]
                
                # Add vehicle base cost if switching vehicles
                if last_vehicle != vtype:
                    leg_cost += vtype.value.base_cost
                
                new_cost = cost + leg_cost
                new_path = path + [next_node]
                new_vehicles = path_vehicles + [vtype]
                
                heapq.heappush(pq, (new_cost, next_node, new_path, vtype, new_vehicles))
    
    # Fallback: direct route with SemiTruck
    dist_m = haversine_distance_meters(origin, destination)
    return dist_m * COST_PER_METER[VehicleType.SemiTruck], [origin, destination], [VehicleType.SemiTruck]

def get_hubs_from_boxes(boxes):
    """Extract unique hub locations from box origins and destinations"""
    hubs = set()
    for box in boxes.values():
        hubs.add(box["location"])
        hubs.add(box["destination"])
    return list(hubs)

def find_alternative_route(vid, vehicle, boxes, sim_state):
    """Find alternative destination when disrupted"""
    vtype = vehicle["vehicle_type"]
    current_loc = vehicle["location"]
    
    # Find nearest undelivered box destination
    best = None
    best_dist = float('inf')
    
    for box in boxes.values():
        if not box["delivered"]:
            dist = haversine_distance_meters(current_loc, box["destination"])
            if dist < best_dist:
                best_dist = dist
                best = box["destination"]
    
    return best

def is_affected_by_event(vtype, location, event):
    """Check if vehicle is affected by a disruption event"""
    if event["type"] == "ground_stop_flights":
        return vtype in [VehicleType.Airplane, VehicleType.Drone]
    
    elif event["type"] == "traffic":
        if vtype not in [VehicleType.SemiTruck, VehicleType.Train]:
            return False
        if "center" in event and "radius_m" in event:
            dist = haversine_distance_meters(location, event["center"])
            return dist <= event["radius_m"]
        return True
    
    elif event["type"] == "oceanic_weather":
        if vtype != VehicleType.CargoShip:
            return False
        if "center" in event and "radius_m" in event:
            dist = haversine_distance_meters(location, event["center"])
            return dist <= event["radius_m"]
        return True
    
    return False

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Get hubs from box locations
    hubs = get_hubs_from_boxes(boxes)
    
    # Handle active events (disruptions)
    active_events = sim_state.get_active_events()
    
    for vid, vehicle in vehicles.items():
        vtype = vehicle["vehicle_type"]
        loc = vehicle["location"]
        
        # Check if affected by event
        for event in active_events:
            if is_affected_by_event(vtype, loc, event):
                # Reroute
                alt_dest = find_alternative_route(vid, vehicle, boxes, sim_state)
                if alt_dest and alt_dest != vehicle["destination"]:
                    try:
                        sim_state.move_vehicle(vid, alt_dest)
                    except ValueError:
                        pass
                break
    
    # Refresh vehicles after potential reroutes
    vehicles = sim_state.get_vehicles()
    
    # Normal vehicle management
    for vid, vehicle in vehicles.items():
        loc = vehicle["location"]
        vtype = vehicle["vehicle_type"]
        
        # Skip if moving
        if vehicle["destination"] is not None:
            continue
        
        # Unload at destination
        deliverable = []
        for bid in vehicle["cargo"]:
            if haversine_distance_meters(loc, boxes[bid]["destination"]) <= _PROXIMITY_M:
                deliverable.append(bid)
        
        if deliverable:
            try:
                sim_state.unload_vehicle(vid, deliverable)
                boxes = sim_state.get_boxes()
            except ValueError:
                pass
        
        # Load more boxes if capacity available
        remaining_capacity = CAPACITY[vtype] - len(vehicle["cargo"])
        if remaining_capacity > 0:
            loadable = [
                bid for bid, box in boxes.items()
                if not box["delivered"]
                and box["vehicle_id"] is None
                and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M
            ]
            if loadable:
                try:
                    sim_state.load_vehicle(vid, loadable[:remaining_capacity])
                    boxes = sim_state.get_boxes()
                except ValueError:
                    pass
        
        # Move to next destination
        vehicles = sim_state.get_vehicles()
        vehicle = vehicles[vid]
        if vehicle["cargo"]:
            # Get first box destination
            first_bid = vehicle["cargo"][0]
            dest = boxes[first_bid]["destination"]
            if haversine_distance_meters(loc, dest) > _PROXIMITY_M:
                try:
                    sim_state.move_vehicle(vid, dest)
                except ValueError:
                    pass
    
    # Spawn vehicles for undelivered boxes at tick 0
    if tick == 0:
        # Group boxes by origin
        origins = {}
        for bid, box in boxes.items():
            if not box["delivered"]:
                loc = box["location"]
                if loc not in origins:
                    origins[loc] = []
                origins[loc].append(bid)
        
        for origin_loc, box_ids in origins.items():
            num_boxes = len(box_ids)
            
            # Find most common destination to plan route
            dest_counts = {}
            for bid in box_ids:
                dest = boxes[bid]["destination"]
                dest_counts[dest] = dest_counts.get(dest, 0) + 1
            
            primary_dest = max(dest_counts, key=dest_counts.get)
            
            # Use Dijkstra to find optimal route
            total_cost, path, vehicle_types = dijkstra_route(
                origin_loc, primary_dest, hubs, num_boxes, sim_state
            )
            
            if len(path) >= 2 and len(vehicle_types) >= 1:
                # Spawn vehicle for first leg
                first_vehicle = vehicle_types[0]
                first_dest = path[1]
                
                try:
                    vid = sim_state.create_vehicle(first_vehicle, origin_loc)
                    
                    # Load as many boxes as capacity allows
                    capacity = CAPACITY[first_vehicle]
                    load_count = min(num_boxes, capacity)
                    to_load = box_ids[:load_count]
                    
                    sim_state.load_vehicle(vid, to_load)
                    sim_state.move_vehicle(vid, first_dest)
                    
                    # Store the full route for future legs
                    # (would need persistent storage for multi-leg routes)
                    
                except ValueError:
                    # Fallback to direct truck
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin_loc)
                        load_count = min(num_boxes, CAPACITY[VehicleType.SemiTruck])
                        sim_state.load_vehicle(vid, box_ids[:load_count])
                        sim_state.move_vehicle(vid, primary_dest)
                    except ValueError:
                        pass
