# my_strategy.py
from simulator import VehicleType
from collections import deque
import math

# Global tracking
vehicle_tasks = {}  # vid -> {"boxes": [bid], "route": [(lat,lon), ...], "step": 0}

def distance(coord1, coord2):
    """Euclidean distance (simplified - API uses meters internally)"""
    return math.sqrt((coord1[0]-coord2[0])**2 + (coord1[1]-coord2[1])**2)

def plan_route(box, hubs, airports, ports):
    """BFS planning - returns list of waypoints or None"""
    origin = box["location"]
    destination = box["destination"]
    
    # Simple BFS over hubs (since that's all we can transload at)
    queue = deque([(origin, [origin])])
    visited = {origin}
    
    while queue:
        loc, path = queue.popleft()
        
        if distance(loc, destination) < 0.0005:  # ~50m in degrees
            return path + [destination]
        
        # Try all hubs as next stops
        for hub in hubs:
            if hub not in visited:
                visited.add(hub)
                queue.append((hub, path + [hub]))
        
        # Also try airports and ports for air/sea routes
        for airport in airports:
            if airport not in visited:
                visited.add(airport)
                queue.append((airport, path + [airport]))
        
        for port in ports:
            if port not in visited:
                visited.add(port)
                queue.append((port, path + [port]))
    
    return None

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Get facility locations (in real comp, you'd parse bootstrap)
    hubs = [(33.9425, -118.4081), (40.6413, -73.7781)]  # LA, NY
    airports = hubs  # Fallback
    ports = [(33.7361, -118.2639), (40.6756, -74.0869)]  # LA, NY ports
    
    # ===== INITIALIZATION: Create vehicles and plan routes =====
    if tick == 0:
        for bid, box in boxes.items():
            if not box["delivered"]:
                route = plan_route(box, hubs, airports, ports)
                if route and len(route) >= 2:
                    # Try to spawn appropriate vehicle at start
                    start = route[0]
                    for vtype in [VehicleType.SemiTruck, VehicleType.Train, 
                                  VehicleType.Airplane, VehicleType.CargoShip, 
                                  VehicleType.Drone]:
                        try:
                            vid = sim_state.create_vehicle(vtype, start)
                            sim_state.load_vehicle(vid, [bid])
                            sim_state.move_vehicle(vid, route[1])
                            vehicle_tasks[vid] = {"boxes": [bid], "route": route[1:], "step": 0}
                            break
                        except ValueError:
                            continue
    
    # ===== RUNTIME: Handle vehicles =====
    for vid, v in vehicles.items():
        # Unload if at destination
        if v["destination"] is None and v["cargo"]:
            to_unload = [bid for bid in v["cargo"] 
                        if distance(boxes[bid]["destination"], v["location"]) < 0.0005]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                
                # Update tracking
                if vid in vehicle_tasks:
                    for bid in to_unload:
                        if bid in vehicle_tasks[vid]["boxes"]:
                            vehicle_tasks[vid]["boxes"].remove(bid)
        
        # Continue along planned route
        if vid in vehicle_tasks and vehicle_tasks[vid]["boxes"] and v["destination"] is None:
            route = vehicle_tasks[vid]["route"]
            step_idx = vehicle_tasks[vid]["step"]
            
            if step_idx < len(route):
                sim_state.move_vehicle(vid, route[step_idx])
                vehicle_tasks[vid]["step"] = step_idx + 1
    
    # ===== HANDLE DISRUPTIONS =====
    for event in sim_state.get_active_events():
        if event["type"] == "ground_stop_flights":
            # Reroute airplanes to alternate airports
            for vid, v in vehicles.items():
                if v["vehicle_type"] in ["Airplane", "Drone"] and v["destination"]:
                    # Find nearest airport not affected
                    for airport in airports:
                        if airport != v["destination"]:
                            try:
                                sim_state.move_vehicle(vid, airport)
                                break
                            except:
                                continue
        
        elif event["type"] == "oceanic_weather":
            # Reroute ships to different port
            for vid, v in vehicles.items():
                if v["vehicle_type"] == "CargoShip" and v["destination"]:
                    for port in ports:
                        if port != v["destination"]:
                            try:
                                sim_state.move_vehicle(vid, port)
                                break
                            except:
                                continue
