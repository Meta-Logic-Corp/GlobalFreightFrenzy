from simulator import VehicleType, haversine_distance_meters
from math import ceil

_PROXIMITY_M = 50.0
_HUB_CLUSTER_RADIUS_M = 5000.0
_DEST_CLUSTER_RADIUS_M = 80000.0

_TRAIN_MIN_BOXES = 80
_SHIP_MIN_BOXES = 120
_TRUCK_MAX_PREFERRED_BOXES = 70
_DRONE_MAX_DIST_M = 30000.0
_AIR_MIN_DIST_M = 1200000.0
_AIR_MIN_BOXES = 40
_LONG_HAUL_M = 300000.0


def _distance_m(a, b):
    return haversine_distance_meters(a, b)


def _distance_km(a, b):
    return _distance_m(a, b) / 1000.0


def _avg_point(points):
    if not points:
        return (0.0, 0.0)
    return (
        sum(p[0] for p in points) / len(points),
        sum(p[1] for p in points) / len(points),
    )


def _cfg(vtype):
    return vtype.value


def _cfg_base_cost(vtype):
    return getattr(vtype.value, "base_cost", 0.0)


def _cfg_per_km_cost(vtype):
    return getattr(vtype.value, "per_km_cost", 0.0)


def _cfg_speed(vtype):
    cfg = vtype.value
    return getattr(cfg, "speed_km_h", getattr(cfg, "speed", 0.0))


def _cfg_capacity_from_type(vtype):
    return getattr(vtype.value, "capacity", 0)


def _cfg_capacity_from_name(vehicle_type_name):
    return getattr(VehicleType[vehicle_type_name].value, "capacity", 0)


def _get_active_events(sim_state):
    if hasattr(sim_state, "get_active_events"):
        try:
            return sim_state.get_active_events() or []
        except Exception:
            return []
    return []


def _events_block(vehicle_type_name, events):
    for event in events:
        etype = event.get("type")
        if etype == "ground_stop_flights" and vehicle_type_name in ("Airplane", "Drone"):
            return True
        if etype == "oceanic_weather" and vehicle_type_name == "CargoShip":
            return True
    return False


def _boxes_waiting(boxes):
    out = {}
    for bid, box in boxes.items():
        if box["delivered"]:
            continue
        if box["vehicle_id"] is not None:
            continue
        out[bid] = box
    return out


def _cluster_hubs(boxes):
    hubs = []

    for bid, box in boxes.items():
        loc = box["location"]
        placed = False

        for hub in hubs:
            if _distance_m(loc, hub["center"]) <= _HUB_CLUSTER_RADIUS_M:
                hub["box_ids"].append(bid)
                hub["points"].append(loc)
                hub["center"] = _avg_point(hub["points"])
                placed = True
                break

        if not placed:
            hubs.append({
                "center": loc,
                "points": [loc],
                "box_ids": [bid],
            })

    return hubs


def _boxes_at_hub(boxes, hub_center):
    out = []
    for bid, box in boxes.items():
        if box["delivered"]:
            continue
        if box["vehicle_id"] is not None:
            continue
        if _distance_m(box["location"], hub_center) <= _PROXIMITY_M:
            out.append(bid)
    return out


def _deliverable_here(vehicle, boxes):
    loc = vehicle["location"]
    out = []
    for bid in vehicle["cargo"]:
        if bid not in boxes:
            continue
        box = boxes[bid]
        if box["delivered"]:
            continue
        if _distance_m(loc, box["destination"]) <= _PROXIMITY_M:
            out.append(bid)
    return out


def _group_by_exact_destination(box_ids, boxes):
    groups = {}
    for bid in box_ids:
        dest = boxes[bid]["destination"]
        key = (round(dest[0], 5), round(dest[1], 5))
        if key not in groups:
            groups[key] = {
                "destination": dest,
                "box_ids": [],
            }
        groups[key]["box_ids"].append(bid)
    return sorted(groups.values(), key=lambda g: len(g["box_ids"]), reverse=True)


def _group_by_destination_region(box_ids, boxes):
    clusters = []

    for bid in box_ids:
        dest = boxes[bid]["destination"]
        placed = False

        for cluster in clusters:
            if _distance_m(dest, cluster["center"]) <= _DEST_CLUSTER_RADIUS_M:
                cluster["box_ids"].append(bid)
                pts = [boxes[x]["destination"] for x in cluster["box_ids"]]
                cluster["center"] = _avg_point(pts)
                placed = True
                break

        if not placed:
            clusters.append({
                "center": dest,
                "box_ids": [bid],
            })

    clusters.sort(key=lambda c: len(c["box_ids"]), reverse=True)
    return clusters


def _find_nearest_hub(point, hub_centers, exclude=None):
    best = None
    best_dist = float("inf")

    for hub in hub_centers:
        if exclude is not None and _distance_m(hub, exclude) <= _PROXIMITY_M:
            continue
        d = _distance_m(point, hub)
        if d < best_dist:
            best_dist = d
            best = hub

    return best, best_dist


def _safe_create_vehicle(sim_state, vehicle_type, location):
    try:
        vid = sim_state.create_vehicle(vehicle_type, location)
        print(f"created {vid} type={vehicle_type.name} at {location}")
        return vid
    except Exception as e:
        print(f"failed create {vehicle_type.name} at {location}: {e}")
        return None


def _cost_per_box_for_leg(vtype, origin, target, box_count):
    capacity = max(_cfg_capacity_from_type(vtype), 1)
    dist_km = _distance_km(origin, target)
    trips = ceil(box_count / capacity)

    total_cost = _cfg_base_cost(vtype) + (_cfg_per_km_cost(vtype) * dist_km * trips)
    return total_cost / max(box_count, 1)


def _score_vehicle_for_leg(vtype, origin, target, box_count, leg_kind):
    dist_m = _distance_m(origin, target)
    dist_km = dist_m / 1000.0
    speed = max(_cfg_speed(vtype), 1.0)
    capacity = max(_cfg_capacity_from_type(vtype), 1)
    cost_per_box = _cost_per_box_for_leg(vtype, origin, target, box_count)

    score = 1.0 / max(cost_per_box, 1e-6)

    utilization = min(box_count / capacity, 1.0)
    score *= (0.5 + utilization)

    time_factor = speed / max(dist_km, 1.0)
    score *= (1.0 + min(time_factor, 5.0) * 0.05)

    if leg_kind == "land_final":
        if vtype == VehicleType.Train:
            if box_count >= _TRAIN_MIN_BOXES:
                score *= 2.5
            else:
                score *= 0.35
        elif vtype == VehicleType.SemiTruck:
            if box_count <= _TRUCK_MAX_PREFERRED_BOXES:
                score *= 1.8
            else:
                score *= 0.9
        elif vtype == VehicleType.Airplane:
            if dist_m >= _AIR_MIN_DIST_M and box_count >= _AIR_MIN_BOXES:
                score *= 0.9
            else:
                score *= 0.08
        elif vtype == VehicleType.Drone:
            if box_count <= 5 and dist_m <= _DRONE_MAX_DIST_M:
                score *= 1.1
            else:
                score *= 0.05
        elif vtype == VehicleType.CargoShip:
            score *= 0.01

    elif leg_kind == "hub_transfer_ocean":
        if vtype == VehicleType.CargoShip:
            if box_count >= _SHIP_MIN_BOXES:
                score *= 3.5
            else:
                score *= 0.4
        elif vtype == VehicleType.Airplane:
            if dist_m >= _AIR_MIN_DIST_M and box_count >= _AIR_MIN_BOXES:
                score *= 0.7
            else:
                score *= 0.1
        elif vtype == VehicleType.Train:
            score *= 0.2
        elif vtype == VehicleType.SemiTruck:
            score *= 0.05
        elif vtype == VehicleType.Drone:
            score *= 0.01

    elif leg_kind == "hub_transfer_land":
        if vtype == VehicleType.Train:
            if box_count >= _TRAIN_MIN_BOXES:
                score *= 3.0
            else:
                score *= 0.5
        elif vtype == VehicleType.SemiTruck:
            score *= 1.3
        elif vtype == VehicleType.Airplane:
            if dist_m >= _AIR_MIN_DIST_M and box_count >= _AIR_MIN_BOXES:
                score *= 0.8
            else:
                score *= 0.1
        elif vtype == VehicleType.CargoShip:
            score *= 0.02
        elif vtype == VehicleType.Drone:
            score *= 0.03

    return score


def _choose_vehicle_for_leg(sim_state, origin, target, box_count, leg_kind):
    events = _get_active_events(sim_state)

    if leg_kind == "hub_transfer_ocean":
        candidates = [
            VehicleType.CargoShip,
            VehicleType.Airplane,
        ]
    elif leg_kind == "hub_transfer_land":
        candidates = [
            VehicleType.Train,
            VehicleType.SemiTruck,
            VehicleType.Airplane,
        ]
    else:
        candidates = [
            VehicleType.Train,
            VehicleType.SemiTruck,
            VehicleType.Airplane,
            VehicleType.Drone,
        ]

    ranked = []
    for vtype in candidates:
        if _events_block(vtype.name, events):
            continue
        score = _score_vehicle_for_leg(vtype, origin, target, box_count, leg_kind)
        ranked.append((score, vtype))

    ranked.sort(key=lambda x: x[0], reverse=True)

    for score, vtype in ranked:
        vid = _safe_create_vehicle(sim_state, vtype, origin)
        if vid is not None:
            print(
                f"selected {vtype.name} "
                f"leg_kind={leg_kind} "
                f"boxes={box_count} "
                f"score={score:.6f} "
                f"cost_per_box={_cost_per_box_for_leg(vtype, origin, target, box_count):.6f}"
            )
            return vid

    return None


def _load_up_to_capacity(sim_state, vid, vehicle, candidate_box_ids):
    remaining = max(0, _cfg_capacity_from_name(vehicle["vehicle_type"]) - len(vehicle["cargo"]))
    if remaining <= 0:
        return 0

    chosen = candidate_box_ids[:remaining]
    if not chosen:
        return 0

    try:
        sim_state.load_vehicle(vid, chosen)
        print(f"loaded {len(chosen)} onto {vid}")
        return len(chosen)
    except Exception as e:
        print(f"failed load {vid}: {e}")
        return 0


def _pick_loaded_vehicle_next_exact_destination(vehicle, boxes):
    if not vehicle["cargo"]:
        return None

    groups = _group_by_exact_destination(vehicle["cargo"], boxes)
    if not groups:
        return None

    return groups[0]["destination"]


def _classify_transfer_leg(origin_hub, region_center, nearest_dest_hub):
    if nearest_dest_hub is None:
        return "land_final", region_center

    direct_dist = _distance_m(origin_hub, region_center)
    hub_to_hub_dist = _distance_m(origin_hub, nearest_dest_hub)

    if direct_dist >= _LONG_HAUL_M and hub_to_hub_dist >= 100000.0:
        return "hub_transfer_ocean", nearest_dest_hub

    if hub_to_hub_dist >= 100000.0:
        return "hub_transfer_land", nearest_dest_hub

    return "land_final", region_center


def _spawn_initial(sim_state):
    boxes = _boxes_waiting(sim_state.get_boxes())
    hubs = _cluster_hubs(boxes)
    hub_centers = [h["center"] for h in hubs]

    print(f"tick0 hubs={len(hubs)}")

    for hub in hubs:
        hub_center = hub["center"]
        waiting_ids = _boxes_at_hub(sim_state.get_boxes(), hub_center)
        if not waiting_ids:
            continue

        region_groups = _group_by_destination_region(waiting_ids, sim_state.get_boxes())
        if not region_groups:
            continue

        best_region = region_groups[0]
        region_center = best_region["center"]
        region_box_count = len(best_region["box_ids"])

        nearest_dest_hub, _ = _find_nearest_hub(region_center, hub_centers, exclude=hub_center)
        leg_kind, planned_target = _classify_transfer_leg(hub_center, region_center, nearest_dest_hub)

        vid = _choose_vehicle_for_leg(
            sim_state,
            hub_center,
            planned_target,
            region_box_count,
            leg_kind,
        )
        if vid is None:
            continue

        vehicles = sim_state.get_vehicles()
        vehicle = vehicles.get(vid)
        if vehicle is None:
            continue

        if leg_kind == "land_final":
            exact_groups = _group_by_exact_destination(waiting_ids, sim_state.get_boxes())
            candidate_ids = exact_groups[0]["box_ids"] if exact_groups else waiting_ids
        else:
            candidate_ids = best_region["box_ids"]

        loaded = _load_up_to_capacity(sim_state, vid, vehicle, candidate_ids)
        if loaded <= 0:
            continue

        try:
            sim_state.move_vehicle(vid, planned_target)
            print(f"moving {vid} to {planned_target} leg_kind={leg_kind}")
        except Exception as e:
            print(f"failed move {vid}: {e}")


def _manage_idle_vehicle(sim_state, vid, vehicle, all_hub_centers):
    boxes = sim_state.get_boxes()
    loc = vehicle["location"]

    deliverable = _deliverable_here(vehicle, boxes)
    if deliverable:
        try:
            sim_state.unload_vehicle(vid, deliverable)
            print(f"unloaded {len(deliverable)} from {vid}")
        except Exception as e:
            print(f"failed unload {vid}: {e}")

        vehicles = sim_state.get_vehicles()
        boxes = sim_state.get_boxes()
        vehicle = vehicles.get(vid)
        if vehicle is None:
            return

    if vehicle["cargo"]:
        next_dest = _pick_loaded_vehicle_next_exact_destination(vehicle, boxes)
        if next_dest is not None:
            try:
                sim_state.move_vehicle(vid, next_dest)
                print(f"moving loaded {vid} to exact destination {next_dest}")
            except Exception as e:
                print(f"failed move loaded {vid}: {e}")
        return

    waiting_here = _boxes_at_hub(boxes, loc)
    if not waiting_here:
        return

    region_groups = _group_by_destination_region(waiting_here, boxes)
    if not region_groups:
        return

    best_region = region_groups[0]
    region_center = best_region["center"]
    region_box_count = len(best_region["box_ids"])

    vehicle_type = vehicle["vehicle_type"]

    if vehicle_type == "CargoShip":
        nearest_dest_hub, _ = _find_nearest_hub(region_center, all_hub_centers, exclude=loc)
        if nearest_dest_hub is None:
            return

        loaded = _load_up_to_capacity(sim_state, vid, vehicle, best_region["box_ids"])
        if loaded <= 0:
            return

        try:
            sim_state.move_vehicle(vid, nearest_dest_hub)
            print(f"ship {vid} moving hub-to-hub {nearest_dest_hub}")
        except Exception as e:
            print(f"failed ship move {vid}: {e}")
        return

    if vehicle_type in ("Train", "SemiTruck", "Airplane", "Drone"):
        exact_groups = _group_by_exact_destination(waiting_here, boxes)
        if not exact_groups:
            return

        loaded = _load_up_to_capacity(sim_state, vid, vehicle, exact_groups[0]["box_ids"])
        if loaded <= 0:
            return

        vehicles = sim_state.get_vehicles()
        boxes = sim_state.get_boxes()
        vehicle = vehicles.get(vid)
        if vehicle is None or not vehicle["cargo"]:
            return

        next_dest = _pick_loaded_vehicle_next_exact_destination(vehicle, boxes)
        if next_dest is None:
            return

        try:
            sim_state.move_vehicle(vid, next_dest)
            print(f"{vehicle_type} {vid} moving to {next_dest}")
        except Exception as e:
            print(f"failed move {vehicle_type} {vid}: {e}")


def _spawn_for_waiting_hubs(sim_state):
    vehicles = sim_state.get_vehicles()
    boxes = _boxes_waiting(sim_state.get_boxes())
    hubs = _cluster_hubs(boxes)
    hub_centers = [h["center"] for h in hubs]

    idle_locations = []
    for vehicle in vehicles.values():
        if vehicle["destination"] is None:
            idle_locations.append(vehicle["location"])

    for hub in hubs:
        hub_center = hub["center"]
        waiting_ids = _boxes_at_hub(sim_state.get_boxes(), hub_center)
        if not waiting_ids:
            continue

        already_served = False
        for loc in idle_locations:
            if _distance_m(loc, hub_center) <= _PROXIMITY_M:
                already_served = True
                break
        if already_served:
            continue

        region_groups = _group_by_destination_region(waiting_ids, sim_state.get_boxes())
        if not region_groups:
            continue

        best_region = region_groups[0]
        region_center = best_region["center"]
        region_box_count = len(best_region["box_ids"])

        nearest_dest_hub, _ = _find_nearest_hub(region_center, hub_centers, exclude=hub_center)
        leg_kind, planned_target = _classify_transfer_leg(hub_center, region_center, nearest_dest_hub)

        vid = _choose_vehicle_for_leg(
            sim_state,
            hub_center,
            planned_target,
            region_box_count,
            leg_kind,
        )
        if vid is None:
            continue

        vehicles = sim_state.get_vehicles()
        vehicle = vehicles.get(vid)
        if vehicle is None:
            continue

        if leg_kind == "land_final":
            exact_groups = _group_by_exact_destination(waiting_ids, sim_state.get_boxes())
            candidate_ids = exact_groups[0]["box_ids"] if exact_groups else waiting_ids
        else:
            candidate_ids = best_region["box_ids"]

        loaded = _load_up_to_capacity(sim_state, vid, vehicle, candidate_ids)
        if loaded <= 0:
            continue

        try:
            sim_state.move_vehicle(vid, planned_target)
            print(f"new vehicle {vid} moving to {planned_target} leg_kind={leg_kind}")
        except Exception as e:
            print(f"failed move new vehicle {vid}: {e}")


def step(sim_state):
    print(
        f"tick={sim_state.tick} "
        f"total_cost={sim_state.total_cost} "
        f"terrain_penalty={getattr(sim_state, 'terrain_penalty', 'n/a')}"
    )

    if sim_state.tick == 0:
        _spawn_initial(sim_state)
        return

    boxes = _boxes_waiting(sim_state.get_boxes())
    hubs = _cluster_hubs(boxes)
    all_hub_centers = [h["center"] for h in hubs]

    vehicles = sim_state.get_vehicles()
    for vid, vehicle in vehicles.items():
        if vehicle["destination"] is not None:
            continue
        _manage_idle_vehicle(sim_state, vid, vehicle, all_hub_centers)

    _spawn_for_waiting_hubs(sim_state)
