from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

VEHICLE_RESTRICTIONS = {
    VehicleType.SemiTruck: {"terrain": "land", "spawn": "hub", "load_unload": "hub", "max_distance": None},
    VehicleType.Train: {"terrain": "land", "spawn": "hub", "load_unload": "hub", "max_distance": None},
    VehicleType.Airplane: {"terrain": "any", "spawn": "airport", "load_unload": "airport", "max_distance": None},
    VehicleType.CargoShip: {"terrain": "water", "spawn": "ocean_port", "load_unload": "ocean_port", "max_distance": None},
    VehicleType.Drone: {"terrain": "any", "spawn": "airport", "load_unload": "airport", "max_distance": 20000}
}

def is_water_crossing(from_loc, to_loc):
    dist = haversine_distance_meters(from_loc, to_loc)
    if dist > 500000:
        return True
    
    from_lat, from_lon = from_loc
    to_lat, to_lon = to_loc
    
    if (from_lon < -60 and to_lon > -10) or (from_lon > -10 and to_lon < -60):
        if abs(from_lat - to_lat) < 50:
            return True
    
    if (from_lon < -120 and to_lon > 120) or (from_lon > 120 and to_lon < -120):
        return True
    
    return False

def can_vehicle_handle_route(vtype, from_loc, to_loc, num_boxes, sim_state):
    if num_boxes > vtype.value.capacity:
        return False
    
    if VEHICLE_RESTRICTIONS[vtype]["max_distance"]:
        dist = haversine_distance_meters(from_loc, to_loc)
        if dist > VEHICLE_RESTRICTIONS[vtype]["max_distance"]:
            return False
    
    return True


def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    all_locations = set()
    for box in boxes.values():
        all_locations.add(box["location"])
        all_locations.add(box["destination"])
    hubs = list(all_locations)

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

    # MAIN LOGIC
    for vid, v in vehicles.items():
        if v["destination"] is None:
            loc = v["location"]
            vtype = VehicleType[v["vehicle_type"]]
            capacity_left = vtype.value.capacity - len(v["cargo"])

            # LOAD
            if capacity_left > 0:
                loadable = [
                    bid for bid, box in boxes.items()
                    if not box["delivered"]
                    and box["vehicle_id"] is None
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
                    nearest_hub = None
                    min_dist = float("inf")
                    for hub in hubs:
                        if hub != loc:
                            d = haversine_distance_meters(loc, hub)
                            if d < min_dist:
                                min_dist = d
                                nearest_hub = hub

                    cargo_to_transfer = list(v["cargo"])
                    target_dest = dest

                    sim_state.unload_vehicle(vid, cargo_to_transfer)
                    boxes = sim_state.get_boxes()

                    # ONLY SAFE TRANSSHIPMENT (NO AIRPLANES HERE)
                    if is_water_crossing(loc, dest):
                        try:
                            new_vid = sim_state.create_vehicle(VehicleType.CargoShip, nearest_hub)
                            sim_state.load_vehicle(new_vid, cargo_to_transfer)
                            sim_state.move_vehicle(new_vid, target_dest)
                        except ValueError:
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

            elif not v["cargo"]:
                nearest = None
                min_dist = float("inf")
                for bid, box in boxes.items():
                    if not box["delivered"] and box["vehicle_id"] is None:
                        d = haversine_distance_meters(loc, box["location"])
                        if d < min_dist:
                            min_dist = d
                            nearest = box["location"]
                if nearest:
                    sim_state.move_vehicle(vid, nearest)

    # SPAWN (unchanged except safe)
    if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]

        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)

            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)

                needs_water = any(
                    is_water_crossing(origin, boxes[bid]["destination"])
                    for bid in box_ids[:5]
                )

                if needs_water:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                        to_load = box_ids[:1000]
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
