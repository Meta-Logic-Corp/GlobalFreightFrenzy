from simulator import VehicleType
from collections import deque
import math
import random

def distance(coord1, coord2):
    """Simple Euclidean distance"""
    return math.sqrt((coord1[0]-coord2[0])**2 + (coord1[1]-coord2[1])**2)

def plan_route(box, hubs):
    """BFS to find shortest path through hubs"""
    origin = box["location"]
    dest = box["destination"]
    
    queue = deque([[origin]])
    visited = {origin}
    
    while queue:
        path = queue.popleft()
        loc = path[-1]
        
        if distance(loc, dest) < 0.0005:
            return path + [dest]
        
        for hub in hubs:
            if hub not in visited:
                visited.add(hub)
                queue.append(path + [hub])
    
    return [origin, dest]

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
                route = plan_route(box, hubs)
                
                # Try different vehicle types until one works
                vehicle_types = list(VehicleType)
                random.shuffle(vehicle_types)
                
                for vtype in vehicle_types:
                    try:
                        vid = sim_state.create_vehicle(vtype, route[0])
                        sim_state.load_vehicle(vid, [box["id"]])
                        sim_state.move_vehicle(vid, route[1])
                        break  # Success, exit loop
                    except ValueError:
                        continue  # Try next vehicle type
    
    # Unload at destination
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            unload = [bid for bid in v["cargo"] 
                     if distance(boxes[bid]["destination"], v["location"]) < 0.0005]
            if unload:
                sim_state.unload_vehicle(vid, unload)
