from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # --- NEW: Get real facilities from the API ---
    hubs = sim_state.get_shipping_hubs()          # tuple of (lat,lon)
    airports = sim_state.get_airports()          # tuple of (lat,lon)
    ocean_ports = sim_state.get_ocean_ports()    # tuple of (lat,lon)
    # If no airports are configured, fall back to hubs
    if not airports:
        airports = hubs
    # -------------------------------------------------
    
    # UNLOAD
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = []
            for bid in v["cargo"]:
                if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M:
                    to_unload.append(bid)
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()
    
    # MANAGE VEHICLES
    for vid, v in vehicles.items():
        if v["destination"] is None:
            loc = v["location"]
            vtype = VehicleType[v["vehicle_type"]]
            capacity_left = vtype.value.capacity - len(v["cargo"])
            
            # LOAD
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
                dest = boxes[first_bid]["destination"]
                num_boxes = len(v["cargo"])
                
                if can_vehicle_handle_route(vtype, loc, dest, num_boxes, sim_state):
                    sim_state.move_vehicle(vid, dest)
                else:
                    # Find nearest hub (for fallback)
                    nearest_hub = None
                    min_dist = float('inf')
                    for hub in hubs:
                        if hub != loc:
                            dist = haversine_distance_meters(loc, hub)
                            if dist < min_dist:
                                min_dist = dist
                                nearest_hub = hub
                    
                    if nearest_hub:
                        cargo_to_transfer = list(v["cargo"])
                        target_dest = dest
                        
                        # unload first
                        sim_state.unload_vehicle(vid, cargo_to_transfer)
                        boxes = sim_state.get_boxes()
                        
                        if is_water_crossing(loc, dest):
                            # --- FIX: Try ship first, then airplane ---
                            ship_created = False
                            try:
                                new_vid = sim_state.create_vehicle(VehicleType.CargoShip, nearest_hub)
                                sim_state.load_vehicle(new_vid, cargo_to_transfer)
                                sim_state.move_vehicle(new_vid, target_dest)
                                ship_created = True
                            except ValueError:
                                pass
                            
                            if not ship_created:
                                # Try to create airplane at the nearest airport (not at the hub)
                                # Find the closest airport to the current location
                                nearest_airport = None
                                best_dist = float('inf')
                                for ap in airports:
                                    d = haversine_distance_meters(loc, ap)
                                    if d < best_dist:
                                        best_dist = d
                                        nearest_airport = ap
                                if nearest_airport:
                                    try:
                                        new_vid = sim_state.create_vehicle(VehicleType.Airplane, nearest_airport)
                                        sim_state.load_vehicle(new_vid, cargo_to_transfer)
                                        sim_state.move_vehicle(new_vid, target_dest)
                                    except ValueError:
                                        # ultimate fallback: send the original vehicle
                                        sim_state.move_vehicle(vid, target_dest)
                                else:
                                    sim_state.move_vehicle(vid, target_dest)
                        else:
                            if num_boxes >= 20:
                                try:
                                    new_vid = sim_state.create_vehicle(VehicleType.Train, nearest_hub)
                                    sim_state.load_vehicle(new_vid, cargo_to_transfer)
                                    sim_state.move_vehicle(new_vid, target_dest)
                                except ValueError:
                                    sim_state.move_vehicle(vid, target_dest)
                            else:
                                sim_state.move_vehicle(vid, target_dest)
                    else:
                        sim_state.move_vehicle(vid, dest)
            
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
    
    # SPAWN
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
                    if num_boxes >= 10:
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
                
                if num_boxes >= 20:
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
