from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict

_PROXIMITY_M = 50.0


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


def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    # build hubs from all known locations
    hubs = list({b["location"] for b in boxes.values()} | {b["destination"] for b in boxes.values()})

    # -------------------------
    # UNLOAD (deliver cargo)
    # -------------------------
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [
                bid for bid in v["cargo"]
                if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M
            ]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()

    # -------------------------
    # MAIN ROUTING
    # -------------------------
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue

        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]

        # LOAD
        capacity_left = vtype.value.capacity - len(v["cargo"])
        if capacity_left > 0:
            loadable = [
                bid for bid, box in boxes.items()
                if not box["delivered"]
                and box["vehicle_id"] is None
                and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M
            ]
            if loadable:
                sim_state.load_vehicle(vid, loadable[:capacity_left])
                boxes = sim_state.get_boxes()

        # NO CARGO → go pick something up
        if not v["cargo"]:
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
            continue

        # -------------------------
        # ROUTE DECISION
        # -------------------------
        first = v["cargo"][0]
        dest = boxes[first]["destination"]

        # direct move if valid
        if not is_water_crossing(loc, dest):
            sim_state.move_vehicle(vid, dest)
            continue

        # -------------------------
        # TRANSFER SYSTEM (SAFE)
        # -------------------------
        cargo = list(v["cargo"])

        # choose nearest hub
        nearest_hub = min(
            hubs,
            key=lambda h: haversine_distance_meters(loc, h)
        )

        # move to hub first if far away
        if haversine_distance_meters(loc, nearest_hub) > 1000:
            sim_state.move_vehicle(vid, nearest_hub)
            continue

        # unload at hub
        sim_state.unload_vehicle(vid, cargo)
        boxes = sim_state.get_boxes()

        transferred = False

        # -------------------------
        # TRY SHIP FIRST (water)
        # -------------------------
        try:
            new_vid = sim_state.create_vehicle(VehicleType.CargoShip, nearest_hub)
            sim_state.load_vehicle(new_vid, cargo)
            sim_state.move_vehicle(new_vid, dest)
            transferred = True
        except ValueError:
            pass

        # -------------------------
        # TRY AIRPLANE SECOND (only if ship fails)
        # -------------------------
        if not transferred:
            try:
                new_vid = sim_state.create_vehicle(VehicleType.Airplane, nearest_hub)
                sim_state.load_vehicle(new_vid, cargo)
                sim_state.move_vehicle(new_vid, dest)
                transferred = True
            except ValueError:
                pass

        # -------------------------
        # FINAL FALLBACK
        # -------------------------
        if not transferred:
            try:
                sim_state.load_vehicle(vid, cargo)
            except ValueError:
                pass
            sim_state.move_vehicle(vid, dest)

    # -------------------------
    # SPAWN LOGIC (SIMPLE + SAFE)
    # -------------------------
    if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):
        undelivered = [
            bid for bid, box in boxes.items()
            if not box["delivered"] and box["vehicle_id"] is None
        ]

        origin_groups = defaultdict(list)
        for bid in undelivered:
            origin_groups[boxes[bid]["location"]].append(bid)

        for origin, b_ids in origin_groups.items():
            if not b_ids:
                continue

            dest = boxes[b_ids[0]]["destination"]
            size = len(b_ids)

            try:
                if is_water_crossing(origin, dest):
                    vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                    sim_state.load_vehicle(vid, b_ids[:1000])
                elif size >= 20:
                    vid = sim_state.create_vehicle(VehicleType.Train, origin)
                    sim_state.load_vehicle(vid, b_ids[:500])
                else:
                    vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                    sim_state.load_vehicle(vid, b_ids[:50])

                sim_state.move_vehicle(vid, dest)
                break

            except ValueError:
                continue
