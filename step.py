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
    restrictions = VEHICLE_RESTRICTIONS[vtype]
    if restrictions["max_distance"]:
        dist = haversine_distance_meters(from_loc, to_loc)
        if dist > restrictions["max_distance"]:
            return False
    if num_boxes > vtype.value.capacity:
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
            loc = v["location"]
            to_unload = [bid for bid in v["cargo"]
                if haversine_distance_meters(loc, boxes[bid]["destination"]) <= _PROXIMITY_M]
            if to_unload:
                try:
                    sim_state.unload_vehicle(vid, to_unload)
                    boxes = sim_state.get_boxes()
                except ValueError:
                    pass

    # MANAGE VEHICLES
    for vid, v in vehicles.items():
        if v["destination"] is None:
            loc = v["location"]
            vtype = VehicleType[v["vehicle_type"]]
            capacity_left = vtype.value.capacity - len(v["cargo"])

            if capacity_left > 0:
                loadable = [
                    bid for bid, box in boxes.items()
                    if not box["delivered"] and box["vehicle_id"] is None
                    and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M
                ]
                if loadable:
                    try:
                        sim_state.load_vehicle(vid, loadable[:capacity_left])
                        boxes = sim_state.get_boxes()
                    except ValueError:
                        pass

            if v["cargo"]:
                first_bid = v["cargo"][0]
                dest = boxes[first_bid]["destination"]
                num_boxes = len(v["cargo"])

                if can_vehicle_handle_route(vtype, loc, dest, num_boxes, sim_state):
                    sim_state.move_vehicle(vid, dest)
                else:
                    nearest_hub = min((h for h in hubs if h != loc),
                                      key=lambda h: haversine_distance_meters(loc, h), default=None)
                    if nearest_hub:
                        cargo_copy = list(v["cargo"])
                        target_dest = dest
                        try:
                            sim_state.unload_vehicle(vid, cargo_copy)
                            boxes = sim_state.get_boxes()
                        except ValueError:
                            sim_state.move_vehicle(vid, dest)
                            continue

                        spawned = False
                        candidates = (
                            [VehicleType.CargoShip, VehicleType.Airplane]
                            if is_water_crossing(loc, dest)
                            else ([VehicleType.Train] if num_boxes >= 20 else [VehicleType.SemiTruck])
                        )
                        for vt in candidates:
                            try:
                                new_vid = sim_state.create_vehicle(vt, nearest_hub)
                                sim_state.load_vehicle(new_vid, cargo_copy)
                                sim_state.move_vehicle(new_vid, target_dest)
                                spawned = True
                                break
                            except ValueError:
                                continue
                        if not spawned:
                            sim_state.move_vehicle(vid, target_dest)
                    else:
                        sim_state.move_vehicle(vid, dest)

            elif not v["cargo"]:
                unassigned = [(bid, box) for bid, box in boxes.items()
                              if not box["delivered"] and box["vehicle_id"] is None]
                if unassigned:
                    nearest = min(unassigned,
                                  key=lambda x: haversine_distance_meters(loc, x[1]["location"]))
                    sim_state.move_vehicle(vid, nearest[1]["location"])

    # SPAWN
    if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items()
                       if not box["delivered"] and box["vehicle_id"] is None]
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)

            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                needs_water = any(is_water_crossing(origin, boxes[bid]["destination"])
                                  for bid in box_ids[:5])

                candidates = []
                if needs_water:
                    if num_boxes <= 1000:
                        candidates.append((VehicleType.CargoShip, 1000))
                    if num_boxes <= 100:
                        candidates.append((VehicleType.Airplane, 100))
                else:
                    if 20 <= num_boxes <= 500:
                        candidates.append((VehicleType.Train, 500))
                    if num_boxes <= 50:
                        candidates.append((VehicleType.SemiTruck, 50))

                for vt, cap in candidates:
                    try:
                        vid = sim_state.create_vehicle(vt, origin)
                        to_load = box_ids[:cap]
                        sim_state.load_vehicle(vid, to_load)
                        dest = boxes[to_load[0]]["destination"]
                        sim_state.move_vehicle(vid, dest)
                        break
                    except ValueError:
                        continue
