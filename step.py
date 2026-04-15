from simulator import VehicleType
import math
import heapq

def distance(coord1, coord2):
    return math.sqrt((coord1[0]-coord2[0])**2 + (coord1[1]-coord2[1])**2)

def cheapest_vehicle(dist_km, num_boxes):
    """Return cheapest vehicle for this distance and cargo size"""
    # Cost per km for each vehicle
    costs = {
        VehicleType.CargoShip: 0.01,
        VehicleType.Train: 0.02,
        VehicleType.SemiTruck: 0.05,
        VehicleType.Drone: 0.30,
        VehicleType.Airplane: 0.50,
    }
    
    capacities = {
        VehicleType.SemiTruck: 50,
        VehicleType.Train: 500,
        VehicleType.Airplane: 100,
        VehicleType.CargoShip: 1000,
        VehicleType.Drone: 5,
    }
    
    best = VehicleType.SemiTruck
    best_cost = float('inf')
    
    for vtype in costs:
        if num_boxes <= capacities[vtype]:
            cost = dist_km * costs[vtype]
            if cost < best_cost:
                best_cost = cost
                best = vtype
    
    return best

def dijkstra_route(origin, destination, hubs):
    """Dijkstra to find cheapest path through hubs"""
    # All nodes = hubs + origin + destination
    nodes = list(set(hubs + [origin, destination]))
    
    # Build graph: each node connects to all others (complete graph)
    dist = {}
    for a in nodes:
        for b in nodes:
            if a != b:
                dist[(a,b)] = distance(a, b)
    
    # Dijkstra
    pq = [(0, origin, [origin])]  # (cost, node, path)
    visited = set()
    
    while pq:
        cost, node, path = heapq.heappop(pq)
        
        if node in visited:
            continue
        visited.add(node)
        
        if node == destination:
            return path
        
        for next_node in nodes:
            if next_node not in visited and next_node != node:
                edge_cost = dist.get((node, next_node), float('inf'))
                heapq.heappush(pq, (cost + edge_cost, next_node, path + [next_node]))
    
    return [origin, destination]

def step(sim_state):
    if sim_state.tick == 0:
        boxes = sim_state.get_boxes()
        
        # Discover hubs from box locations
        hubs = set()
        for box in boxes.values():
            hubs.add(box["location"])
            hubs.add(box["destination"])
        hubs = list(hubs)
        
        for box in boxes.values():
            if not box["delivered"]:
                # Get cheapest route using Dijkstra
                route = dijkstra_route(box["location"], box["destination"], hubs)
                
                if len(route) >= 2:
                    dist_km = sum(distance(route[i], route[i+1]) for i in range(len(route)-1))
                    vtype = cheapest_vehicle(dist_km, 1)
                    
                    # Try to spawn vehicle
                    try:
                        vid = sim_state.create_vehicle(vtype, route[0])
                        sim_state.load_vehicle(vid, [box["id"]])
                        sim_state.move_vehicle(vid, route[1])
                    except ValueError:
                        # Fallback to SemiTruck
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, route[0])
                            sim_state.load_vehicle(vid, [box["id"]])
                            sim_state.move_vehicle(vid, route[1])
                        except ValueError:
                            pass
    
    # Unload at destination
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [bid for bid in v["cargo"] 
                        if distance(boxes[bid]["destination"], v["location"]) < 0.0005]
            if to_unload:
                try:
                    sim_state.unload_vehicle(vid, to_unload)
                except ValueError:
                    # Can't unload here - move to destination
                    for bid in to_unload:
                        sim_state.move_vehicle(vid, boxes[bid]["destination"])
