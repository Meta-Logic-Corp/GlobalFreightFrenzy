from simulator import VehicleType, haversine_distance_meters

DEBUG = True

_PROXIMITY_M = 50.0
_INFRA_M = 5000.0
_ARRIVAL_M = 20000.0
_BOX_VALUE = 1000.0
_STUCK_TICKS = 50
_LONG_ROUTE_KM = 4000.0

_vehicle_jobs = {}
_box_memory = {}

OCEAN_WAYPOINTS = {
    "panama": (9.08, -79.68),
    "suez": (30.0, 32.55),
    "gibraltar": (35.95, -5.6),
    "malacca": (2.5, 101.5),
    "good_hope": (-34.4, 18.5),
    "horn": (-55.98, -67.27),
    "bab_el": (12.58, 43.33),
    "n_atlantic_w": (35.0, -60.0),
    "n_atlantic_e": (45.0, -20.0),
    "s_atlantic": (-20.0, -20.0),
    "n_pacific_w": (35.0, 150.0),
    "n_pacific_e": (35.0, -150.0),
    "s_pacific": (-20.0, -140.0),
    "n_indian": (10.0, 70.0),
    "s_indian": (-20.0, 80.0),
    "red_sea": (20.0, 38.0),
    "med_east": (34.0, 25.0),
    "caribbean": (15.0, -75.0),
    "english_ch": (50.0, -2.0),
    "north_sea": (56.0, 3.0),
    "south_china": (15.0, 115.0),
    "coral_sea": (-15.0, 155.0),
}


def log(*args):
    if DEBUG:
        print(*args)


def km(a, b):
    return haversine_distance_meters(a, b) / 1000.0


def nearest(loc, points):
    if not points:
        return None
    return min(points, key=lambda p: km(loc, p))


def near_enough(a, b, meters=_INFRA_M):
    return b is not None and haversine_distance_meters(a, b) <= meters


def get_cfg(vehicle_type):
    return vehicle_type.value


def vehicle_mode(vehicle_type):
    if vehicle_type == VehicleType.CargoShip:
        return "ocean"
    if vehicle_type in (VehicleType.SemiTruck, VehicleType.Train):
        return "land"
    return "air"


def rebuild_clusters(boxes):
    clusters = {}
    for box in boxes.values():
        if box["delivered"]:
            continue
        if box["vehicle_id"] is not None:
            continue
        key = (box["location"], box["destination"])
        clusters.setdefault(key, []).append(box["id"])
    return clusters


def update_box_memory(boxes, tick):
    seen = set()

    for bid, box in boxes.items():
        seen.add(bid)

        if box["delivered"]:
            _box_memory.pop(bid, None)
            continue

        signature = (
            box["location"],
            box["vehicle_id"],
        )

        if bid not in _box_memory:
            _box_memory[bid] = {
                "last_signature": signature,
                "last_moved_tick": tick,
            }
            continue

        if _box_memory[bid]["last_signature"] != signature:
            _box_memory[bid]["last_signature"] = signature
            _box_memory[bid]["last_moved_tick"] = tick

    for bid in list(_box_memory.keys()):
        if bid not in seen:
            _box_memory.pop(bid, None)


def cluster_is_stuck(box_ids, tick):
    if not box_ids:
        return False
    for bid in box_ids:
        mem = _box_memory.get(bid)
        if mem is None:
            return False
        if tick - mem["last_moved_tick"] < _STUCK_TICKS:
            return False
    return True


def path_km(path):
    total = 0.0
    for i in range(len(path) - 1):
        total += km(path[i], path[i + 1])
    return total


def chain_waypoints(origin_port, dest_port):
    path = [origin_port]
    current = origin_port
    used = set()

    while True:
        direct = km(current, dest_port)
        best_name = None
        best_total = direct

        for name, wp in OCEAN_WAYPOINTS.items():
            if name in used:
                continue
            if km(wp, dest_port) >= direct:
                continue
            total = km(current, wp) + km(wp, dest_port)
            if total < best_total:
                best_total = total
                best_name = name

        if best_name is None:
            break

        used.add(best_name)
        current = OCEAN_WAYPOINTS[best_name]
        path.append(current)

    path.append(dest_port)
    return path


def build_ship_path(origin, dest, ocean_ports):
    if not ocean_ports:
        return None

    origin_port = nearest(origin, ocean_ports)
    dest_port = nearest(dest, ocean_ports)

    if origin_port is None or dest_port is None:
        return None
    if not near_enough(origin, origin_port):
        return None
    if not near_enough(dest, dest_port):
        return None

    ship_path = chain_waypoints(origin_port, dest_port)

    return {
        "origin_port": origin_port,
        "dest_port": dest_port,
        "path": ship_path,
        "path_km": path_km(ship_path),
    }


def estimate_terrain_penalty(vehicle_type, origin, dest, ocean_ports, shipping_hubs):
    cfg = get_cfg(vehicle_type)
    rate = getattr(cfg, "terrain_penalty_per_km", 0.0)
    d = km(origin, dest)

    if vehicle_type in (VehicleType.Airplane, VehicleType.Drone):
        return 0.0

    if vehicle_type == VehicleType.CargoShip:
        ship_info = build_ship_path(origin, dest, ocean_ports)
        if ship_info is None:
            return float("inf")
        return 0.0

    if vehicle_type in (VehicleType.SemiTruck, VehicleType.Train):
        if not shipping_hubs:
            return float("inf")
        oh = nearest(origin, shipping_hubs)
        dh = nearest(dest, shipping_hubs)
        if oh is None or dh is None:
            return float("inf")
        if not near_enough(origin, oh) or not near_enough(dest, dh):
            return float("inf")

        origin_near_port = False
        dest_near_port = False
        if ocean_ports:
            op = nearest(origin, ocean_ports)
            dp = nearest(dest, ocean_ports)
            origin_near_port = near_enough(origin, op)
            dest_near_port = near_enough(dest, dp)

        water_guess_km = 0.0
        if d > 2500 and origin_near_port and dest_near_port:
            water_guess_km = d * 0.65
        elif d > 1500 and origin_near_port and dest_near_port:
            water_guess_km = d * 0.35
        elif d > 3000:
            water_guess_km = d * 0.15

        return water_guess_km * rate

    return float("inf")

def can_service_route(vehicle_type, origin, dest, ocean_ports, airports, shipping_hubs):
    d = km(origin, dest)

    if vehicle_type == VehicleType.CargoShip:
        ship_info = build_ship_path(origin, dest, ocean_ports)
        return ship_info is not None

    if vehicle_type in (VehicleType.SemiTruck, VehicleType.Train):
        if not shipping_hubs:
            return False

        oh = nearest(origin, shipping_hubs)
        dh = nearest(dest, shipping_hubs)

        if oh is None or dh is None:
            return False

        if not near_enough(origin, oh):
            return False

        if not near_enough(dest, dh):
            return False

        if d >= _LONG_ROUTE_KM:
            return False

        origin_port = nearest(origin, ocean_ports) if ocean_ports else None
        dest_port = nearest(dest, ocean_ports) if ocean_ports else None

        origin_near_port = near_enough(origin, origin_port) if origin_port is not None else False
        dest_near_port = near_enough(dest, dest_port) if dest_port is not None else False

        if origin_near_port and dest_near_port and d >= 1200:
            return False

        return True

    if vehicle_type == VehicleType.Airplane:
        if not airports:
            return False

        oa = nearest(origin, airports)
        da = nearest(dest, airports)

        if oa is None or da is None:
            return False

        return near_enough(origin, oa) and near_enough(dest, da)

    if vehicle_type == VehicleType.Drone:
        if not airports:
            return False

        oa = nearest(origin, airports)
        da = nearest(dest, airports)

        if oa is None or da is None:
            return False

        if not (near_enough(origin, oa) and near_enough(dest, da)):
            return False

        return d < _LONG_ROUTE_KM

    return False

def spawn_point_for(vehicle_type, origin, ocean_ports, airports, shipping_hubs):
    if vehicle_type == VehicleType.CargoShip:
        ship_info = build_ship_path(origin, origin, ocean_ports)
        if ship_info is not None:
            return ship_info["origin_port"]
        p = nearest(origin, ocean_ports) if ocean_ports else None
        return p if near_enough(origin, p) else None

    if vehicle_type in (VehicleType.SemiTruck, VehicleType.Train):
        h = nearest(origin, shipping_hubs) if shipping_hubs else None
        return h if near_enough(origin, h) else None

    if vehicle_type in (VehicleType.Airplane, VehicleType.Drone):
        a = nearest(origin, airports) if airports else None
        return a if near_enough(origin, a) else None

    return None


def route_score(vehicle_type, origin, dest, n_boxes, ocean_ports, airports, shipping_hubs):
    if not can_service_route(vehicle_type, origin, dest, ocean_ports, airports, shipping_hubs):
        return None

    cfg = get_cfg(vehicle_type)
    load = min(n_boxes, cfg.capacity)
    if load <= 0:
        return None

    route_path = [dest]
    distance_km = km(origin, dest)

    if vehicle_type == VehicleType.CargoShip:
        ship_info = build_ship_path(origin, dest, ocean_ports)
        if ship_info is None:
            return None
        route_path = ship_info["path"][1:]
        distance_km = ship_info["path_km"]

    terrain_penalty = estimate_terrain_penalty(vehicle_type, origin, dest, ocean_ports, shipping_hubs)
    if terrain_penalty == float("inf"):
        return None

    cost = cfg.base_cost + cfg.per_km_cost * distance_km + terrain_penalty
    value = load * _BOX_VALUE
    margin = value - cost
    cost_per_box_km = cfg.per_km_cost / load

    return {
        "vehicle_type": vehicle_type,
        "distance_km": distance_km,
        "load": load,
        "cost": cost,
        "value": value,
        "margin": margin,
        "terrain_penalty": terrain_penalty,
        "cost_per_box_km": cost_per_box_km,
        "route_path": route_path,
        "route_kind": "port_to_port" if vehicle_type == VehicleType.CargoShip else "direct",
    }




def choose_best_vehicle(origin, dest, n_boxes, ocean_ports, airports, shipping_hubs, force=False):
    candidates = []
    direct_distance = km(origin, dest)

    if direct_distance >= _LONG_ROUTE_KM:
        vehicle_pool = [VehicleType.Airplane, VehicleType.Drone]
    else:
        vehicle_pool = [
            VehicleType.CargoShip,
            VehicleType.Train,
            VehicleType.SemiTruck,
            VehicleType.Airplane,
            VehicleType.Drone,
        ]

    for vt in vehicle_pool:
        scored = route_score(vt, origin, dest, n_boxes, ocean_ports, airports, shipping_hubs)
        if scored is None:
            continue
        if not force and scored["margin"] <= 0:
            continue
        candidates.append(scored)

    if not candidates:
        return None

    candidates.sort(
        key=lambda x: (
            x["cost"],
            x["cost_per_box_km"],
            -x["load"],
        )
    )
    return candidates[0]

def find_idle_vehicle(vehicle_type, vehicles, origin, ocean_ports, airports, shipping_hubs):
    spawn = spawn_point_for(vehicle_type, origin, ocean_ports, airports, shipping_hubs)
    if spawn is None:
        return None

    best_vid = None
    best_dist = float("inf")

    for vid, job in _vehicle_jobs.items():
        if job["vehicle_type"] != vehicle_type.name:
            continue
        if job["state"] != "IDLE":
            continue
        if vid not in vehicles:
            continue
        if vehicles[vid]["destination"] is not None:
            continue
        if vehicles[vid]["cargo"]:
            continue

        d = km(vehicles[vid]["location"], spawn)
        if d < best_dist:
            best_dist = d
            best_vid = vid

    return best_vid


def create_or_get_vehicle(sim_state, vehicle_type, origin, vehicles, ocean_ports, airports, shipping_hubs, force_new=False):
    if not force_new:
        vid = find_idle_vehicle(vehicle_type, vehicles, origin, ocean_ports, airports, shipping_hubs)
        if vid is not None:
            return vid

    spawn = spawn_point_for(vehicle_type, origin, ocean_ports, airports, shipping_hubs)
    if spawn is None:
        return None

    try:
        vid = sim_state.create_vehicle(vehicle_type, spawn)
    except ValueError:
        return None

    _vehicle_jobs[vid] = {
        "vehicle_type": vehicle_type.name,
        "state": "IDLE",
        "origin": None,
        "dest": None,
        "box_ids": [],
        "cluster_key": None,
        "force_job": False,
        "route_path": [],
        "route_kind": "direct",
    }
    return vid


def refresh_jobs_with_missing_vehicles(vehicles):
    for vid in list(_vehicle_jobs.keys()):
        if vid not in vehicles:
            _vehicle_jobs.pop(vid, None)


def get_active_clusters():
    active_clusters = set()
    for job in _vehicle_jobs.values():
        if job["state"] != "IDLE" and job["cluster_key"] is not None:
            active_clusters.add(job["cluster_key"])
    return active_clusters


def dispatch(sim_state, boxes, vehicles, ocean_ports, airports, shipping_hubs):
    active_clusters = get_active_clusters()
    clusters = rebuild_clusters(boxes)
    ranked = sorted(clusters.items(), key=lambda item: -len(item[1]))

    for (origin, dest), bids in ranked:
        key = (origin, dest)
        if key in active_clusters:
            continue

        is_stuck = cluster_is_stuck(bids, sim_state.tick)
        best = choose_best_vehicle(
            origin,
            dest,
            len(bids),
            ocean_ports,
            airports,
            shipping_hubs,
            force=is_stuck,
        )
        if best is None:
            continue

        vid = create_or_get_vehicle(
            sim_state,
            best["vehicle_type"],
            origin,
            vehicles,
            ocean_ports,
            airports,
            shipping_hubs,
            force_new=is_stuck,
        )
        if vid is None:
            continue

        cfg = get_cfg(best["vehicle_type"])
        chosen_boxes = bids[:cfg.capacity]

        _vehicle_jobs[vid] = {
            "vehicle_type": best["vehicle_type"].name,
            "state": "LOADING",
            "origin": origin,
            "dest": dest,
            "box_ids": chosen_boxes,
            "cluster_key": key,
            "force_job": is_stuck,
            "route_path": list(best["route_path"]),
            "route_kind": best["route_kind"],
        }

        reason = "forced-stuck" if is_stuck else "normal"
        log(
            f"[dispatch] {reason} {vid} {best['vehicle_type'].name} "
            f"{origin} -> {dest} boxes={len(chosen_boxes)} "
            f"cost={best['cost']:.1f} terrain={best['terrain_penalty']:.1f} "
            f"cpbk={best['cost_per_box_km']:.6f} route={best['route_kind']}"
        )


def execute(sim_state, boxes, vehicles):
    for vid, job in list(_vehicle_jobs.items()):
        if vid not in vehicles:
            continue

        vehicle = vehicles[vid]
        loc = vehicle["location"]

        if job["state"] == "IDLE":
            continue

        if job["state"] == "LOADING":
            if vehicle["destination"] is not None:
                continue

            cap = get_cfg(VehicleType[job["vehicle_type"]]).capacity
            cap_left = cap - len(vehicle["cargo"])
            if cap_left <= 0:
                job["state"] = "EN_ROUTE"
                continue

            loadable = []
            remaining = []

            for bid in job["box_ids"]:
                if bid not in boxes:
                    continue

                box = boxes[bid]

                if box["delivered"]:
                    continue

                if box["destination"] != job["dest"]:
                    remaining.append(bid)
                    continue

                if box["vehicle_id"] is not None:
                    remaining.append(bid)
                    continue

                if haversine_distance_meters(loc, box["location"]) > _PROXIMITY_M:
                    remaining.append(bid)
                    continue

                if len(loadable) < cap_left:
                    loadable.append(bid)
                else:
                    remaining.append(bid)

            if loadable:
                try:
                    sim_state.load_vehicle(vid, loadable)
                    job["box_ids"] = remaining
                    log(f"[load] {vid} loaded={len(loadable)}")
                except ValueError as e:
                    log(f"[load-failed] {vid}: {e}")
                    continue

            current_vehicle = sim_state.get_vehicles().get(vid)
            if current_vehicle and current_vehicle["cargo"]:
                job["state"] = "EN_ROUTE"

        elif job["state"] == "EN_ROUTE":
            if vehicle["destination"] is not None:
                continue

            while job["route_path"] and haversine_distance_meters(loc, job["route_path"][0]) <= _ARRIVAL_M:
                job["route_path"].pop(0)

            if not job["route_path"]:
                job["state"] = "UNLOADING"
                continue

            try:
                sim_state.move_vehicle(vid, job["route_path"][0])
                log(f"[move] {vid} -> {job['route_path'][0]}")
            except (ValueError, KeyError) as e:
                log(f"[move-failed] {vid}: {e}")

        elif job["state"] == "UNLOADING":
            if vehicle["destination"] is not None:
                continue

            if vehicle["cargo"]:
                try:
                    cargo_ids = list(vehicle["cargo"])
                    sim_state.unload_vehicle(vid, cargo_ids)
                    log(f"[unload] {vid} unloaded={len(cargo_ids)}")
                except ValueError as e:
                    log(f"[unload-failed] {vid}: {e}")
                    continue

            current_vehicle = sim_state.get_vehicles().get(vid)
            if current_vehicle is not None and not current_vehicle["cargo"]:
                log(f"[done] {vid} finished {job['cluster_key']}")
                _vehicle_jobs[vid] = {
                    "vehicle_type": job["vehicle_type"],
                    "state": "IDLE",
                    "origin": None,
                    "dest": None,
                    "box_ids": [],
                    "cluster_key": None,
                    "force_job": False,
                    "route_path": [],
                    "route_kind": "direct",
                }


def step(sim_state):
    try:
        ocean_ports = list(sim_state.get_ocean_ports())
    except Exception:
        ocean_ports = []

    try:
        airports = list(sim_state.get_airports())
    except Exception:
        airports = []

    try:
        shipping_hubs = list(sim_state.get_shipping_hubs())
    except Exception:
        shipping_hubs = []

    boxes = sim_state.get_boxes()
    vehicles = sim_state.get_vehicles()

    refresh_jobs_with_missing_vehicles(vehicles)
    update_box_memory(boxes, sim_state.tick)

    dispatch(sim_state, boxes, vehicles, ocean_ports, airports, shipping_hubs)

    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    execute(sim_state, boxes, vehicles)

    if sim_state.tick % 10 == 0:
        waiting = 0
        stuck = 0
        for b in boxes.values():
            if b["delivered"]:
                continue
            if b["vehicle_id"] is None:
                waiting += 1
                mem = _box_memory.get(b["id"])
                if mem is not None and sim_state.tick - mem["last_moved_tick"] >= _STUCK_TICKS:
                    stuck += 1

        active = sum(1 for j in _vehicle_jobs.values() if j["state"] != "IDLE")
        log(
            f"[tick {sim_state.tick}] waiting={waiting} stuck={stuck} "
            f"vehicles={len(vehicles)} active={active} "
            f"cost={sim_state.total_cost:.1f} terrain={sim_state.terrain_penalty:.1f}"
        )
