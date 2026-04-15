from simulator import VehicleType
import math

# ---------- distance in METERS ----------
def haversine_m(a, b):
    lat1, lon1 = a
    lat2, lon2 = b
    R = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2 * R * math.asin(math.sqrt(x))

def near_m(a, b, m):  # proximity check
    return haversine_m(a, b) <= m

def nearest_hub(target, hubs):
    return min(hubs, key=lambda h: haversine_m(target, h))

# ---------- planner ----------
AIR_THRESHOLD_KM = 1500  # only use air for "long" trips

def step(sim_state):
    # persistent state
    if not hasattr(step, "init"):
        step.init = False
        step.hubs = []
        step.air_hubs = []      # hubs within 5km of an airport
        step.plans = {}         # box_id -> plan dict

    boxes = sim_state.get_boxes()
    vehicles = sim_state.get_vehicles()

    # ---- TICK 0: build hub list + discover which hubs are airport-capable ----
    if sim_state.tick == 0 and not step.init:
        hubs = set()
        for b in boxes.values():
            hubs.add(b["location"])
            hubs.add(b["destination"])
        step.hubs = list(hubs)

        # Discover "airport hubs" cheaply: Drone spawn succeeds only within 5km of airport
        air_hubs = []
        for h in step.hubs:
            try:
                _probe = sim_state.create_vehicle(VehicleType.Drone, h)  # costs 50 each
                air_hubs.append(h)
            except ValueError:
                pass
        step.air_hubs = air_hubs

        step.init = True

    # ---- helper: make/get plan per box ----
    def get_plan_for_box(box):
        bid = box["id"]
        if bid in step.plans:
            return step.plans[bid]

        origin = box["location"]
        dest = box["destination"]
        trip_km = haversine_m(origin, dest) / 1000.0

        # default = ground only
        plan = {
            "mode": "ground",
            "truck1": None,
            "plane": None,
            "truck2": None,
            "origin_air": None,
            "dest_air": None,
            "stage": "ground_deliver",  # stages vary by mode
        }

        # if long-distance and we have at least one air hub, try air routing
        if trip_km >= AIR_THRESHOLD_KM and step.air_hubs:
            origin_air = origin if origin in step.air_hubs else nearest_hub(origin, step.air_hubs)
            dest_air = dest if dest in step.air_hubs else nearest_hub(dest, step.air_hubs)
            plan.update({
                "mode": "air",
                "origin_air": origin_air,
                "dest_air": dest_air,
                "stage": "to_origin_air",  # truck1 moves box to origin_air
            })

        step.plans[bid] = plan
        return plan

    # ---- main control loop: advance each undelivered box through its stages ----
    # IMPORTANT: refresh snapshots as we mutate
    for box in list(sim_state.get_boxes().values()):
        if box["delivered"]:
            continue

        plan = get_plan_for_box(box)
        bid = box["id"]

        # Refresh current box state each iteration
        box = sim_state.get_boxes()[bid]

        # -------- GROUND ONLY MODE --------
        if plan["mode"] == "ground":
            if plan["truck1"] is None:
                try:
                    plan["truck1"] = sim_state.create_vehicle(VehicleType.SemiTruck, box["location"])
                except ValueError:
                    continue  # can't spawn here

            vid = plan["truck1"]
            v = sim_state.get_vehicles().get(vid)
            if not v:
                continue

            # load if at origin and box not on vehicle
            if box["vehicle_id"] is None and near_m(v["location"], box["location"], 50):
                try:
                    sim_state.load_vehicle(vid, [bid])
                except ValueError:
                    pass

            # move toward final destination if carrying it and idle
            v = sim_state.get_vehicles()[vid]
            box = sim_state.get_boxes()[bid]
            if box["vehicle_id"] == vid and v["destination"] is None:
                sim_state.move_vehicle(vid, box["destination"])

            # unload if arrived
            v = sim_state.get_vehicles()[vid]
            if box["vehicle_id"] == vid and v["destination"] is None and near_m(v["location"], box["destination"], 50):
                try:
                    sim_state.unload_vehicle(vid, [bid])
                except ValueError:
                    pass

            continue

        # -------- AIR MODE (truck -> plane -> truck) --------
        origin_air = plan["origin_air"]
        dest_air = plan["dest_air"]

        # Stage 1: truck1 carries box from origin hub to origin_air hub
        if plan["stage"] == "to_origin_air":
            if plan["truck1"] is None:
                try:
                    plan["truck1"] = sim_state.create_vehicle(VehicleType.SemiTruck, box["location"])
                except ValueError:
                    continue

            t1 = plan["truck1"]
            tv = sim_state.get_vehicles()[t1]
            box = sim_state.get_boxes()[bid]

            # load at origin
            if box["vehicle_id"] is None and near_m(tv["location"], box["location"], 50):
                try:
                    sim_state.load_vehicle(t1, [bid])
                except ValueError:
                    pass

            # drive to origin_air
            tv = sim_state.get_vehicles()[t1]
            box = sim_state.get_boxes()[bid]
            if box["vehicle_id"] == t1 and tv["destination"] is None and not near_m(tv["location"], origin_air, 50):
                sim_state.move_vehicle(t1, origin_air)

            # unload at origin_air to hand off to plane
            tv = sim_state.get_vehicles()[t1]
            box = sim_state.get_boxes()[bid]
            if box["vehicle_id"] == t1 and tv["destination"] is None and near_m(tv["location"], origin_air, 50):
                try:
                    sim_state.unload_vehicle(t1, [bid])
                    plan["stage"] = "fly"
                except ValueError:
                    pass

            continue

        # Stage 2: airplane loads at origin_air, flies to dest_air, unloads
        if plan["stage"] == "fly":
            if plan["plane"] is None:
                try:
                    plan["plane"] = sim_state.create_vehicle(VehicleType.Airplane, origin_air)
                except ValueError:
                    # If we can't spawn a plane here, we can't do air mode; fall back to ground
                    plan["mode"] = "ground"
                    plan["stage"] = "ground_deliver"
                    continue

            p = plan["plane"]
            pv = sim_state.get_vehicles()[p]
            box = sim_state.get_boxes()[bid]

            # load at origin_air (box should be waiting there now)
            if box["vehicle_id"] is None and near_m(pv["location"], origin_air, 50):
                try:
                    sim_state.load_vehicle(p, [bid])
                except ValueError:
                    # still not near an airport -> means origin_air wasn't actually airport-capable
                    # fall back to ground
                    plan["mode"] = "ground"
                    plan["stage"] = "ground_deliver"
                    continue

            # fly to dest_air
            pv = sim_state.get_vehicles()[p]
            box = sim_state.get_boxes()[bid]
            if box["vehicle_id"] == p and pv["destination"] is None and not near_m(pv["location"], dest_air, 50):
                sim_state.move_vehicle(p, dest_air)

            # unload at dest_air (must be within 5km of airport)
            pv = sim_state.get_vehicles()[p]
            box = sim_state.get_boxes()[bid]
            if box["vehicle_id"] == p and pv["destination"] is None and near_m(pv["location"], dest_air, 50):
                try:
                    sim_state.unload_vehicle(p, [bid])
                    plan["stage"] = "to_final"
                except ValueError:
                    # not airport-capable after all; fall back
                    plan["mode"] = "ground"
                    plan["stage"] = "ground_deliver"

            continue

        # Stage 3: truck2 carries from dest_air to final destination hub
        if plan["stage"] == "to_final":
            if plan["truck2"] is None:
                try:
                    plan["truck2"] = sim_state.create_vehicle(VehicleType.SemiTruck, dest_air)
                except ValueError:
                    continue

            t2 = plan["truck2"]
            t2v = sim_state.get_vehicles()[t2]
            box = sim_state.get_boxes()[bid]

            # load at dest_air
            if box["vehicle_id"] is None and near_m(t2v["location"], dest_air, 50):
                try:
                    sim_state.load_vehicle(t2, [bid])
                except ValueError:
                    pass

            # move to final destination
            t2v = sim_state.get_vehicles()[t2]
            box = sim_state.get_boxes()[bid]
            if box["vehicle_id"] == t2 and t2v["destination"] is None and not near_m(t2v["location"], box["destination"], 50):
                sim_state.move_vehicle(t2, box["destination"])

            # unload delivered
            t2v = sim_state.get_vehicles()[t2]
            box = sim_state.get_boxes()[bid]
            if box["vehicle_id"] == t2 and t2v["destination"] is None and near_m(t2v["location"], box["destination"], 50):
                try:
                    sim_state.unload_vehicle(t2, [bid])
                    plan["stage"] = "done"
                except ValueError:
                    pass
