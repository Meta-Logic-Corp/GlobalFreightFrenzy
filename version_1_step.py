from simulator import VehicleType, haversine_distance_meters
import math
import random

SEED = 42
random.seed(SEED)

_PROXIMITY_M = 50.0
_CLUSTER_RADIUS_KM = 150.0
_ENROUTE_CORRIDOR_KM = 80.0
_ENROUTE_PROGRESS_MIN = 0.15
_MAX_DIRECT_LAND_KM = 1400.0
_MAX_DIRECT_DRONE_KM = 250.0

DEBUG = True

VEHICLE_ENUM = {
    "SemiTruck": VehicleType.SemiTruck,
    "Train": VehicleType.Train,
    "Airplane": VehicleType.Airplane,
    "CargoShip": VehicleType.CargoShip,
    "Drone": VehicleType.Drone,
}

VEHICLE_STATS = {
    "SemiTruck": {
        "base_cost": 100.0,
        "per_km_cost": 0.05,
        "speed_kmh": 80.0,
        "capacity": 50,
        "mode": "land",
    },
    "Train": {
        "base_cost": 500.0,
        "per_km_cost": 0.02,
        "speed_kmh": 120.0,
        "capacity": 500,
        "mode": "land",
    },
    "Airplane": {
        "base_cost": 2000.0,
        "per_km_cost": 0.50,
        "speed_kmh": 800.0,
        "capacity": 100,
        "mode": "air",
    },
    "CargoShip": {
        "base_cost": 1000.0,
        "per_km_cost": 0.01,
        "speed_kmh": 30.0,
        "capacity": 1000,
        "mode": "ocean",
    },
    "Drone": {
        "base_cost": 50.0,
        "per_km_cost": 0.30,
        "speed_kmh": 50.0,
        "capacity": 5,
        "mode": "air",
    },
}

OCEAN_PORTS = set()
LAND_DIRECT_EDGES = set()
OCEAN_DIRECT_EDGES = set()

DISPATCH_STATE = {
    "vehicle_plans": {},
    "hub_vehicles": {},
    "last_undelivered_count": None,
    "stalled_ticks": 0,
}


def log(*args):
    if DEBUG:
        print(*args)


class Hub:
    def __init__(self, location):
        self.location = location
        self.cargo_count = 0
        self.destination_counts = {}
        self.destination_distances_km = {}
        self.destination_boxes = {}
        self.boxes = []
        self.score = float("-inf")
        self.score_details = {}

    def add_box(self, box):
        destination = box["destination"]
        self.cargo_count += 1
        self.destination_counts[destination] = self.destination_counts.get(destination, 0) + 1
        self.destination_distances_km[destination] = distance_km(self.location, destination)
        self.destination_boxes.setdefault(destination, []).append(box)
        self.boxes.append(box)

    def __repr__(self):
        return (
            f"Hub(location={self.location}, "
            f"cargo_count={self.cargo_count}, "
            f"destination_counts={self.destination_counts}, "
            f"destination_distances_km={self.destination_distances_km}, "
            f"score={self.score}, "
            f"score_details={self.score_details})"
        )



def distance_km(a, b):
    return haversine_distance_meters(a, b) / 1000.0


def distance_m(a, b):
    return haversine_distance_meters(a, b)


def normalized_edge(a, b):
    return tuple(sorted([a, b]))


def movement_cost_km(distance_km_value, vehicle_name):
    return VEHICLE_STATS[vehicle_name]["per_km_cost"] * distance_km_value


def spawn_plus_move_cost(start, end, vehicle_name, include_base_cost=True):
    cost = movement_cost_km(distance_km(start, end), vehicle_name)
    if include_base_cost:
        cost += VEHICLE_STATS[vehicle_name]["base_cost"]
    return cost


def delivered_value_for_box_count(box_count):
    return 120.0 * float(box_count)
def soft_air_penalty(vehicle_name, box_count, route_distance_total_km, cheaper_surface_candidates, emergency_mode=False):
    penalty = 0.0
    reasons = []

    if vehicle_name == "Drone":
        if cheaper_surface_candidates and not emergency_mode:
            penalty += 6000.0
            reasons.append("surface_available_for_drone")
        if box_count > 3 and not emergency_mode:
            penalty += 4000.0
            reasons.append("drone_box_count_high")
        if route_distance_total_km > 100.0:
            penalty += 8000.0
            reasons.append("drone_route_too_long")

    elif vehicle_name == "Airplane":
        if cheaper_surface_candidates and not emergency_mode:
            penalty += 5000.0
            reasons.append("surface_available_for_plane")
        if route_distance_total_km < 1800.0 and not emergency_mode:
            penalty += 4000.0
            reasons.append("plane_route_too_short")
        if box_count < 25 and not emergency_mode:
            penalty += 3500.0
            reasons.append("plane_box_count_too_low")

    return penalty, reasons

def update_stall_state(boxes):
    undelivered_count = sum(1 for b in boxes.values() if not b["delivered"])

    last = DISPATCH_STATE.get("last_undelivered_count")
    if last is not None and undelivered_count >= last:
        DISPATCH_STATE["stalled_ticks"] += 1
    else:
        DISPATCH_STATE["stalled_ticks"] = 0

    DISPATCH_STATE["last_undelivered_count"] = undelivered_count
    return DISPATCH_STATE["stalled_ticks"] >= 8


def rebuild_plan_from_current_cargo(sim_state, vehicle_id, vehicle, boxes):
    cargo_ids = list(vehicle["cargo"])
    if not cargo_ids:
        return False

    vehicle_name = normalize_vehicle_name(vehicle["vehicle_type"])
    current_location = vehicle["location"]

    destination_to_box_ids = {}

    for box_id in cargo_ids:
        box = boxes.get(box_id)
        if box is None or box["delivered"]:
            continue
        destination_to_box_ids.setdefault(box["destination"], []).append(box_id)

    if not destination_to_box_ids:
        DISPATCH_STATE["vehicle_plans"][vehicle_id] = {
            "phase": "idle",
            "hub_location": current_location,
            "pickup_location": current_location,
            "vehicle_name": vehicle_name,
            "stops": [],
            "current_stop_index": 0,
        }
        return False

    reachable = []
    for dest, box_ids in destination_to_box_ids.items():
        if is_route_allowed(current_location, dest, vehicle_name):
            reachable.append({
                "location": dest,
                "box_ids": box_ids,
                "distance": distance_km(current_location, dest),
            })

    if not reachable:
        log(
            "CARGO_REPLAN_FAIL",
            "vehicle_id=", vehicle_id,
            "vehicle=", vehicle_name,
            "location=", current_location,
            "reason=no_reachable_cargo_destination",
            "destinations=", list(destination_to_box_ids.keys()),
        )
        return False

    reachable.sort(key=lambda x: x["distance"])

    DISPATCH_STATE["vehicle_plans"][vehicle_id] = {
        "phase": "to_dropoff",
        "hub_location": current_location,
        "pickup_location": current_location,
        "vehicle_name": vehicle_name,
        "stops": [{"location": r["location"], "box_ids": list(r["box_ids"])} for r in reachable],
        "current_stop_index": 0,
    }

    first_stop = reachable[0]["location"]
    try:
        sim_state.move_vehicle(vehicle_id, first_stop)
        log(
            "CARGO_REPLAN_OK",
            "vehicle_id=", vehicle_id,
            "vehicle=", vehicle_name,
            "from=", current_location,
            "to=", first_stop,
            "remaining_stops=", [r["location"] for r in reachable],
        )
        return True
    except ValueError as e:
        log(
            "CARGO_REPLAN_MOVE_FAIL",
            "vehicle_id=", vehicle_id,
            "vehicle=", vehicle_name,
            "target=", first_stop,
            "error=", e,
        )
        return False

def estimate_event_penalty(vehicle_name, active_events):
    mode = VEHICLE_STATS[vehicle_name]["mode"]
    penalty = 0.0

    for event in active_events:
        event_type = event.get("type")

        if event_type == "ground_stop_flights" and mode == "air":
            penalty += 1_000_000.0
        elif event_type == "traffic" and mode == "land":
            speed_multiplier = event.get("speed_multiplier", 0.25)
            slowdown = max(0.0, 1.0 - speed_multiplier)
            penalty += 1500.0 * slowdown
        elif event_type == "oceanic_weather" and mode == "ocean":
            penalty += 1_000_000.0

    return penalty


def project_point_to_segment_km(point, seg_start, seg_end):
    lat_scale = 111.0
    lon_scale = 111.0 * math.cos(math.radians((seg_start[0] + seg_end[0]) * 0.5))

    ax = seg_start[1] * lon_scale
    ay = seg_start[0] * lat_scale
    bx = seg_end[1] * lon_scale
    by = seg_end[0] * lat_scale
    px = point[1] * lon_scale
    py = point[0] * lat_scale

    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    ab2 = (abx * abx) + (aby * aby)
    if ab2 <= 1e-9:
        dx = px - ax
        dy = py - ay
        return math.sqrt(dx * dx + dy * dy), 0.0

    t = ((apx * abx) + (apy * aby)) / ab2
    t_clamped = max(0.0, min(1.0, t))

    qx = ax + (t_clamped * abx)
    qy = ay + (t_clamped * aby)

    dx = px - qx
    dy = py - qy
    d = math.sqrt(dx * dx + dy * dy)

    return d, t_clamped


def is_land_route_allowed(start, end):
    edge = normalized_edge(start, end)

    if LAND_DIRECT_EDGES:
        return edge in LAND_DIRECT_EDGES

    return distance_km(start, end) <= _MAX_DIRECT_LAND_KM


def is_ocean_route_allowed(start, end):
    edge = normalized_edge(start, end)

    if OCEAN_DIRECT_EDGES:
        return edge in OCEAN_DIRECT_EDGES

    return start in OCEAN_PORTS and end in OCEAN_PORTS


def is_route_allowed(start, end, vehicle_name):
    mode = VEHICLE_STATS[vehicle_name]["mode"]

    if mode == "air":
        if vehicle_name == "Drone" and distance_km(start, end) > _MAX_DIRECT_DRONE_KM:
            return False
        return True

    if mode == "land":
        return is_land_route_allowed(start, end)

    if mode == "ocean":
        return is_ocean_route_allowed(start, end)

    return False


def route_distance_km(stops, start_location):
    if not stops:
        return 0.0

    total = 0.0
    current = start_location
    for stop in stops:
        total += distance_km(current, stop)
        current = stop
    return total


def route_is_fully_allowed(start_location, stops, vehicle_name):
    current = start_location
    for stop in stops:
        if not is_route_allowed(current, stop, vehicle_name):
            return False
        current = stop
    return True


def compute_center(points):
    lat = sum(p[0] for p in points) / max(1, len(points))
    lon = sum(p[1] for p in points) / max(1, len(points))
    return (lat, lon)


def cluster_destinations(destination_to_boxes):
    destinations = list(destination_to_boxes.keys())
    clusters = []

    for dest in destinations:
        placed = False
        for cluster in clusters:
            center = cluster["center"]
            if distance_km(dest, center) <= _CLUSTER_RADIUS_KM:
                cluster["destinations"].append(dest)
                cluster["boxes"].extend(destination_to_boxes[dest])
                cluster["center"] = compute_center(cluster["destinations"])
                placed = True
                break

        if not placed:
            clusters.append({
                "center": dest,
                "destinations": [dest],
                "boxes": list(destination_to_boxes[dest]),
            })

    return clusters


def choose_anchor_destination(hub_location, cluster_destinations_list):
    best_dest = None
    best_dist = -1.0

    for dest in cluster_destinations_list:
        d = distance_km(hub_location, dest)
        if d > best_dist:
            best_dist = d
            best_dest = dest

    return best_dest


def sort_stops_from_hub(hub_location, destinations):
    return sorted(destinations, key=lambda d: distance_km(hub_location, d))


def pick_cluster_boxes_for_vehicle(hub_location, cluster, vehicle_name):
    capacity = VEHICLE_STATS[vehicle_name]["capacity"]
    destinations = cluster["destinations"]
    anchor = choose_anchor_destination(hub_location, destinations)
    if anchor is None:
        return [], []

    selected_destinations = []

    for dest in destinations:
        corridor_distance_km, progress = project_point_to_segment_km(dest, hub_location, anchor)

        if dest == anchor:
            selected_destinations.append(dest)
            continue

        if corridor_distance_km <= _ENROUTE_CORRIDOR_KM and progress >= _ENROUTE_PROGRESS_MIN:
            selected_destinations.append(dest)

    if anchor not in selected_destinations:
        selected_destinations.append(anchor)

    selected_destinations = sort_stops_from_hub(hub_location, list(set(selected_destinations)))

    selected_boxes = []
    stop_box_ids = {dest: [] for dest in selected_destinations}

    for dest in selected_destinations:
        boxes_for_dest = sorted(
            [
                b for b in cluster["boxes"]
                if b["destination"] == dest and not b["delivered"] and b["vehicle_id"] is None
            ],
            key=lambda b: distance_km(hub_location, b["destination"]),
        )

        for box in boxes_for_dest:
            if len(selected_boxes) >= capacity:
                break
            selected_boxes.append(box)
            stop_box_ids[dest].append(box["id"])

        if len(selected_boxes) >= capacity:
            break

    final_stops = [dest for dest in selected_destinations if stop_box_ids.get(dest)]
    return selected_boxes, [{"location": stop, "box_ids": stop_box_ids[stop]} for stop in final_stops]


def estimate_capacity_penalty(vehicle_name, box_count):
    capacity = VEHICLE_STATS[vehicle_name]["capacity"]
    if box_count <= capacity:
        return 0.0
    overflow = box_count - capacity
    return 10_000.0 + (1000.0 * overflow)


def estimate_scale_penalty(vehicle_name, total_available_boxes, route_distance_total_km):
    capacity = VEHICLE_STATS[vehicle_name]["capacity"]
    trips_needed = math.ceil(total_available_boxes / max(1, capacity))
    penalty = (trips_needed - 1) * 150.0

    if vehicle_name == "Drone":
        penalty += (trips_needed - 1) * 2500.0
        if total_available_boxes > 5:
            penalty += 10_000.0
        if route_distance_total_km > 120.0:
            penalty += 9_000.0

    if vehicle_name == "Airplane":
        if total_available_boxes < 20:
            penalty += 3_500.0
        if route_distance_total_km < 1500.0:
            penalty += 4_000.0
        if total_available_boxes < 10:
            penalty += 6_000.0

    if vehicle_name == "Train" and total_available_boxes < 15:
        penalty += 400.0

    if vehicle_name == "CargoShip" and total_available_boxes < 30:
        penalty += 1_000.0

    return penalty


def estimate_surface_bias_adjustment(vehicle_name, route_distance_total_km, box_count):
    adjustment = 0.0
    reasons = []

    if vehicle_name == "SemiTruck":
        if 10 <= box_count <= 50:
            adjustment -= 120.0
            reasons.append("truck_good_scale")
        if route_distance_total_km <= 800.0:
            adjustment -= 100.0
            reasons.append("truck_good_distance")

    if vehicle_name == "Train":
        if box_count >= 20:
            adjustment -= 180.0
            reasons.append("train_bulk_reward")
        if route_distance_total_km >= 300.0:
            adjustment -= 120.0
            reasons.append("train_medium_long_reward")

    if vehicle_name == "CargoShip":
        if box_count >= 40:
            adjustment -= 250.0
            reasons.append("ship_bulk_reward")
        if route_distance_total_km >= 500.0:
            adjustment -= 200.0
            reasons.append("ship_long_ocean_reward")

    return adjustment, reasons


def airplane_hard_gate(vehicle_name, box_count, route_distance_total_km, cheaper_surface_candidates):
    if vehicle_name not in ("Airplane", "Drone"):
        return True, []

    reasons = []

    if vehicle_name == "Drone":
        if box_count > 3:
            reasons.append("drone_box_count_too_high")
        if route_distance_total_km > 100.0:
            reasons.append("drone_route_too_long")
        if cheaper_surface_candidates:
            reasons.append("surface_available_for_drone")
        return len(reasons) == 0, reasons

    if cheaper_surface_candidates and box_count < 35:
        reasons.append("surface_available_and_box_count_not_high_enough")
    if route_distance_total_km < 1800.0:
        reasons.append("plane_route_too_short")
    if box_count < 25:
        reasons.append("plane_box_count_too_low")

    return len(reasons) == 0, reasons


def compute_candidate_costs(
    vehicle_start,
    pickup_location,
    stops,
    vehicle_name,
    box_count,
    total_available_boxes,
    active_events,
    include_base_cost=True,
):
    route_stops_only = [s["location"] for s in stops]
    total_route_distance_km = route_distance_km(route_stops_only, pickup_location)

    base_visit_cost = spawn_plus_move_cost(
        vehicle_start,
        pickup_location,
        vehicle_name,
        include_base_cost=include_base_cost,
    )

    delivery_cost = 0.0
    current = pickup_location
    for stop in stops:
        delivery_cost += movement_cost_km(distance_km(current, stop["location"]), vehicle_name)
        current = stop["location"]

    handling_cost = float(box_count)
    event_penalty = estimate_event_penalty(vehicle_name, active_events)
    capacity_penalty = estimate_capacity_penalty(vehicle_name, box_count)
    scale_penalty = estimate_scale_penalty(vehicle_name, total_available_boxes, total_route_distance_km)
    surface_adjustment, surface_adjustment_reasons = estimate_surface_bias_adjustment(
        vehicle_name,
        total_route_distance_km,
        box_count,
    )

    total_cost = (
        base_visit_cost
        + delivery_cost
        + handling_cost
        + event_penalty
        + capacity_penalty
        + scale_penalty
        + surface_adjustment
    )

    return {
        "base_visit_cost": base_visit_cost,
        "delivery_cost": delivery_cost,
        "handling_cost": handling_cost,
        "event_penalty": event_penalty,
        "capacity_penalty": capacity_penalty,
        "scale_penalty": scale_penalty,
        "surface_adjustment": surface_adjustment,
        "surface_adjustment_reasons": surface_adjustment_reasons,
        "total_cost": total_cost,
        "total_route_distance_km": total_route_distance_km,
    }



def score_cluster_route_details(
    vehicle_start,
    pickup_location,
    cluster,
    vehicle_name,
    active_events=None,
    include_base_cost=True,
    emergency_mode=False,
):
    active_events = active_events or []

    selected_boxes, stops = pick_cluster_boxes_for_vehicle(pickup_location, cluster, vehicle_name)
    if not selected_boxes or not stops:
        return None

    if not is_route_allowed(vehicle_start, pickup_location, vehicle_name):
        log(
            "CANDIDATE_FAIL",
            "hub=", pickup_location,
            "vehicle=", vehicle_name,
            "reason=start_to_pickup_not_allowed",
        )
        return None

    current = pickup_location
    for stop in stops:
        if not is_route_allowed(current, stop["location"], vehicle_name):
            log(
                "CANDIDATE_FAIL",
                "hub=", pickup_location,
                "vehicle=", vehicle_name,
                "reason=route_leg_not_allowed",
                "from=", current,
                "to=", stop["location"],
            )
            return None
        current = stop["location"]

    box_count = len(selected_boxes)
    route_stops_only = [s["location"] for s in stops]

    cheaper_surface_candidates = []
    for surface_vehicle in ("SemiTruck", "Train", "CargoShip"):
        if surface_vehicle == vehicle_name:
            continue
        if route_is_fully_allowed(pickup_location, route_stops_only, surface_vehicle):
            cheaper_surface_candidates.append(surface_vehicle)

    cost_details = compute_candidate_costs(
        vehicle_start=vehicle_start,
        pickup_location=pickup_location,
        stops=stops,
        vehicle_name=vehicle_name,
        box_count=box_count,
        total_available_boxes=len(cluster["boxes"]),
        active_events=active_events,
        include_base_cost=include_base_cost,
    )

    air_penalty, air_penalty_reasons = soft_air_penalty(
        vehicle_name=vehicle_name,
        box_count=box_count,
        route_distance_total_km=cost_details["total_route_distance_km"],
        cheaper_surface_candidates=cheaper_surface_candidates,
        emergency_mode=emergency_mode,
    )

    total_cost = cost_details["total_cost"] + air_penalty
    delivered_value = delivered_value_for_box_count(box_count)
    net_value = delivered_value - total_cost
    cost_per_box = total_cost / max(1, box_count)

    if emergency_mode:
        score = net_value - (2.0 * cost_per_box)
    else:
        score = net_value - (4.0 * cost_per_box)

    return {
        "vehicle": vehicle_name,
        "pickup_location": pickup_location,
        "cluster_center": cluster["center"],
        "stop_count": len(stops),
        "stops": stops,
        "box_count": box_count,
        "box_ids": [box["id"] for box in selected_boxes],
        "delivered_value": delivered_value,
        "net_value": net_value,
        "cost_per_box": cost_per_box,
        "cheaper_surface_candidates": cheaper_surface_candidates,
        "rejected": False,
        "reject_reasons": [],
        "air_penalty": air_penalty,
        "air_penalty_reasons": air_penalty_reasons,
        "emergency_mode": emergency_mode,
        "score": score,
        **cost_details,
        "total_cost": total_cost,
    }

def best_route_for_hub(hub, active_events=None, vehicle_start=None, allowed_vehicle_names=None, emergency_mode=False):
    active_events = active_events or []
    clusters = cluster_destinations(hub.destination_boxes)
    start_location = hub.location if vehicle_start is None else vehicle_start

    vehicle_names = list(VEHICLE_STATS.keys())
    if allowed_vehicle_names is not None:
        vehicle_names = [v for v in vehicle_names if v in allowed_vehicle_names]

    best = None

    for cluster in clusters:
        candidates = []

        for vehicle_name in vehicle_names:
            details = score_cluster_route_details(
                vehicle_start=start_location,
                pickup_location=hub.location,
                cluster=cluster,
                vehicle_name=vehicle_name,
                active_events=active_events,
                include_base_cost=True,
                emergency_mode=emergency_mode,
            )

            if details is None:
                continue

            candidates.append(details)

        if not candidates:
            log(
                "HUB_CLUSTER_NO_ROUTE",
                "hub=", hub.location,
                "cluster_center=", cluster["center"],
                "emergency_mode=", emergency_mode,
                "cluster_box_count=", len(cluster["boxes"]),
            )
            continue

        candidates.sort(key=lambda d: (d["score"], -d["total_cost"]), reverse=True)
        cluster_best = candidates[0]

        if best is None or cluster_best["score"] > best["score"]:
            best = cluster_best

    if best is None and not emergency_mode:
        return best_route_for_hub(
            hub,
            active_events=active_events,
            vehicle_start=vehicle_start,
            allowed_vehicle_names=allowed_vehicle_names,
            emergency_mode=True,
        )

    return best

def build_hubs(boxes):
    hubs = {}

    for box in boxes.values():
        if box["delivered"]:
            continue
        if box["vehicle_id"] is not None:
            continue

        location = box["location"]
        if location not in hubs:
            hubs[location] = Hub(location)
        hubs[location].add_box(box)

    return hubs


def refresh_hub_scores(hubs, active_events, emergency_mode=False):
    for hub in hubs.values():
        best = best_route_for_hub(hub, active_events=active_events, emergency_mode=emergency_mode)
        if best is None:
            hub.score = float("-inf")
            hub.score_details = {}
            log(
                "HUB_NO_ROUTE",
                "hub=", hub.location,
                "cargo_count=", hub.cargo_count,
                "emergency_mode=", emergency_mode,
            )
        else:
            hub.score = best["score"]
            hub.score_details = best

def at_location(a, b, threshold_m=_PROXIMITY_M):
    return distance_m(a, b) <= threshold_m


def box_ids_still_available(boxes, box_ids, location=None):
    valid = []
    for box_id in box_ids:
        if box_id not in boxes:
            continue
        box = boxes[box_id]
        if box["delivered"]:
            continue
        if box["vehicle_id"] is not None:
            continue
        if location is not None and box["location"] != location:
            continue
        valid.append(box_id)
    return valid


def normalize_vehicle_name(vehicle_type_value):
    if isinstance(vehicle_type_value, str):
        return vehicle_type_value

    for name, enum_value in VEHICLE_ENUM.items():
        if vehicle_type_value == enum_value:
            return name

    text = str(vehicle_type_value)
    for name in VEHICLE_STATS:
        if name in text:
            return name

    return text


def build_plan_from_best(best, hub_location, vehicle_name):
    return {
        "phase": "to_pickup",
        "hub_location": hub_location,
        "pickup_location": hub_location,
        "vehicle_name": vehicle_name,
        "stops": [{"location": s["location"], "box_ids": list(s["box_ids"])} for s in best["stops"]],
        "current_stop_index": 0,
    }


def clear_vehicle_hub_claim(vehicle_id):
    to_remove = []
    for hub_location, claimed_vehicle_id in DISPATCH_STATE["hub_vehicles"].items():
        if claimed_vehicle_id == vehicle_id:
            to_remove.append(hub_location)
    for hub_location in to_remove:
        DISPATCH_STATE["hub_vehicles"].pop(hub_location, None)


def claim_hub_for_vehicle(hub_location, vehicle_id):
    clear_vehicle_hub_claim(vehicle_id)
    DISPATCH_STATE["hub_vehicles"][hub_location] = vehicle_id


def try_create_best_vehicle(sim_state, hub, active_events, emergency_mode=False):
    best = best_route_for_hub(hub, active_events=active_events, emergency_mode=emergency_mode)
    if best is None:
        log("CREATE_SKIP", "hub=", hub.location, "reason=no_best_route", "emergency_mode=", emergency_mode)
        return None

    vehicle_name = best["vehicle"]

    try:
        vehicle_id = sim_state.create_vehicle(VEHICLE_ENUM[vehicle_name], hub.location)
        DISPATCH_STATE["vehicle_plans"][vehicle_id] = build_plan_from_best(best, hub.location, vehicle_name)
        claim_hub_for_vehicle(hub.location, vehicle_id)
        sim_state.move_vehicle(vehicle_id, hub.location)
        log(
            "CREATE_VEHICLE",
            "vehicle_id=", vehicle_id,
            "vehicle=", vehicle_name,
            "hub=", hub.location,
            "score=", best["score"],
            "box_count=", best["box_count"],
            "cost=", best["total_cost"],
            "cost_per_box=", best["cost_per_box"],
            "net_value=", best["net_value"],
            "route_km=", best["total_route_distance_km"],
            "air_penalty=", best.get("air_penalty"),
            "air_penalty_reasons=", best.get("air_penalty_reasons"),
            "surface_candidates=", best.get("cheaper_surface_candidates"),
            "emergency_mode=", best.get("emergency_mode"),
        )
        return vehicle_id
    except ValueError as e:
        log("CREATE_FAIL", "hub=", hub.location, "vehicle=", vehicle_name, "error=", e)
        return None


def assign_plan_to_vehicle_from_current_location(sim_state, vehicle_id, vehicle, hubs, active_events, emergency_mode=False):
    vehicle_location = vehicle["location"]
    vehicle_name = normalize_vehicle_name(vehicle["vehicle_type"])
    best_overall = None
    best_hub = None

    for hub in hubs.values():
        if hub.cargo_count <= 0:
            continue

        best = best_route_for_hub(
            hub,
            active_events=active_events,
            vehicle_start=vehicle_location,
            allowed_vehicle_names=[vehicle_name],
            emergency_mode=emergency_mode,
        )

        if best is None:
            continue

        if best_overall is None or best["score"] > best_overall["score"]:
            best_overall = best
            best_hub = hub

    if best_overall is None or best_hub is None:
        log(
            "REASSIGN_SKIP",
            "vehicle_id=", vehicle_id,
            "vehicle=", vehicle_name,
            "location=", vehicle_location,
            "reason=no_reachable_hub",
            "emergency_mode=", emergency_mode,
        )
        return False

    DISPATCH_STATE["vehicle_plans"][vehicle_id] = build_plan_from_best(best_overall, best_hub.location, vehicle_name)
    claim_hub_for_vehicle(best_hub.location, vehicle_id)

    log(
        "REASSIGN",
        "vehicle_id=", vehicle_id,
        "vehicle=", vehicle_name,
        "from=", vehicle_location,
        "to_hub=", best_hub.location,
        "score=", best_overall["score"],
        "cost=", best_overall["total_cost"],
        "cost_per_box=", best_overall["cost_per_box"],
        "boxes=", best_overall["box_count"],
        "stops=", [s["location"] for s in best_overall["stops"]],
        "emergency_mode=", best_overall.get("emergency_mode"),
    )

    if not at_location(vehicle_location, best_hub.location):
        try:
            sim_state.move_vehicle(vehicle_id, best_hub.location)
            log("MOVE_TO_PICKUP", "vehicle_id=", vehicle_id, "target=", best_hub.location)
        except ValueError as e:
            log("MOVE_TO_PICKUP_FAIL", "vehicle_id=", vehicle_id, "target=", best_hub.location, "error=", e)

    return True



def process_vehicle_plan(sim_state, vehicle_id, vehicle, boxes):
    plan = DISPATCH_STATE["vehicle_plans"].get(vehicle_id)
    if plan is None:
        log("PLAN_NONE", "vehicle_id=", vehicle_id)
        return

    current_location = vehicle["location"]
    pickup_location = plan["pickup_location"]
    vehicle_name = plan["vehicle_name"]
    stops = plan["stops"]

    log(
        "PLAN_STATUS",
        "vehicle_id=", vehicle_id,
        "vehicle=", vehicle_name,
        "phase=", plan["phase"],
        "location=", current_location,
        "pickup=", pickup_location,
        "stop_index=", plan.get("current_stop_index"),
        "cargo=", list(vehicle["cargo"]),
    )

    if plan["phase"] == "to_pickup":
        if at_location(current_location, pickup_location):
            all_box_ids = []
            for stop in stops:
                all_box_ids.extend(stop["box_ids"])

            available_box_ids = box_ids_still_available(boxes, all_box_ids, location=pickup_location)

            log(
                "AT_PICKUP",
                "vehicle_id=", vehicle_id,
                "pickup=", pickup_location,
                "requested_box_ids=", all_box_ids,
                "available_box_ids=", available_box_ids,
            )

            if available_box_ids:
                try:
                    sim_state.load_vehicle(vehicle_id, available_box_ids)
                    log("LOAD_OK", "vehicle_id=", vehicle_id, "box_ids=", available_box_ids)
                except ValueError as e:
                    reduced = available_box_ids[:max(1, min(len(available_box_ids), VEHICLE_STATS[vehicle_name]["capacity"]))]
                    log("LOAD_FAIL", "vehicle_id=", vehicle_id, "error=", e, "retry_box_ids=", reduced)
                    try:
                        sim_state.load_vehicle(vehicle_id, reduced)
                        available_box_ids = reduced
                        log("LOAD_RETRY_OK", "vehicle_id=", vehicle_id, "box_ids=", available_box_ids)
                    except ValueError as e2:
                        available_box_ids = []
                        log("LOAD_RETRY_FAIL", "vehicle_id=", vehicle_id, "error=", e2)

            filtered_stops = []
            for stop in stops:
                kept_ids = [box_id for box_id in stop["box_ids"] if box_id in available_box_ids]
                if kept_ids:
                    filtered_stops.append({
                        "location": stop["location"],
                        "box_ids": kept_ids,
                    })

            if not filtered_stops:
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("PLAN_IDLE_NO_BOXES", "vehicle_id=", vehicle_id)
                return

            plan["stops"] = filtered_stops
            plan["current_stop_index"] = 0
            first_stop = plan["stops"][0]["location"]

            if not is_route_allowed(pickup_location, first_stop, vehicle_name):
                if list(vehicle["cargo"]):
                    if rebuild_plan_from_current_cargo(sim_state, vehicle_id, vehicle, boxes):
                        return
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("PLAN_IDLE_BAD_ROUTE_FIRST_STOP", "vehicle_id=", vehicle_id, "from=", pickup_location, "to=", first_stop)
                return

            plan["phase"] = "to_dropoff"
            try:
                sim_state.move_vehicle(vehicle_id, first_stop)
                log("MOVE_TO_FIRST_STOP", "vehicle_id=", vehicle_id, "target=", first_stop)
            except ValueError as e:
                if list(vehicle["cargo"]):
                    if rebuild_plan_from_current_cargo(sim_state, vehicle_id, vehicle, boxes):
                        return
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("MOVE_TO_FIRST_STOP_FAIL", "vehicle_id=", vehicle_id, "target=", first_stop, "error=", e)
        else:
            try:
                sim_state.move_vehicle(vehicle_id, pickup_location)
                log("MOVE_TO_PICKUP_CONTINUE", "vehicle_id=", vehicle_id, "target=", pickup_location)
            except ValueError as e:
                if list(vehicle["cargo"]):
                    if rebuild_plan_from_current_cargo(sim_state, vehicle_id, vehicle, boxes):
                        return
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("MOVE_TO_PICKUP_CONTINUE_FAIL", "vehicle_id=", vehicle_id, "target=", pickup_location, "error=", e)
        return

    if plan["phase"] == "to_dropoff":
        idx = plan["current_stop_index"]
        if idx >= len(plan["stops"]):
            plan["phase"] = "idle"
            clear_vehicle_hub_claim(vehicle_id)
            log("PLAN_IDLE_DONE_INDEX", "vehicle_id=", vehicle_id)
            return

        stop = plan["stops"][idx]
        stop_location = stop["location"]

        prev_location = pickup_location if idx == 0 else plan["stops"][idx - 1]["location"]
        if not is_route_allowed(prev_location, stop_location, vehicle_name):
            if list(vehicle["cargo"]):
                if rebuild_plan_from_current_cargo(sim_state, vehicle_id, vehicle, boxes):
                    return
            plan["phase"] = "idle"
            clear_vehicle_hub_claim(vehicle_id)
            log("PLAN_IDLE_BAD_ROUTE_STOP", "vehicle_id=", vehicle_id, "from=", prev_location, "to=", stop_location)
            return

        if at_location(current_location, stop_location):
            cargo_ids = set(vehicle["cargo"])
            unload_ids = [box_id for box_id in stop["box_ids"] if box_id in cargo_ids]

            log(
                "AT_DROPOFF",
                "vehicle_id=", vehicle_id,
                "stop=", stop_location,
                "planned_box_ids=", stop["box_ids"],
                "cargo_box_ids=", list(cargo_ids),
                "unload_ids=", unload_ids,
            )

            if unload_ids:
                try:
                    sim_state.unload_vehicle(vehicle_id, unload_ids)
                    log("UNLOAD_OK", "vehicle_id=", vehicle_id, "box_ids=", unload_ids)
                except ValueError as e:
                    log("UNLOAD_FAIL", "vehicle_id=", vehicle_id, "box_ids=", unload_ids, "error=", e)

            plan["current_stop_index"] += 1

            if plan["current_stop_index"] >= len(plan["stops"]):
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("PLAN_IDLE_ROUTE_COMPLETE", "vehicle_id=", vehicle_id)
                return

            next_stop = plan["stops"][plan["current_stop_index"]]["location"]
            if not is_route_allowed(stop_location, next_stop, vehicle_name):
                if list(vehicle["cargo"]):
                    if rebuild_plan_from_current_cargo(sim_state, vehicle_id, vehicle, boxes):
                        return
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("PLAN_IDLE_BAD_ROUTE_NEXT", "vehicle_id=", vehicle_id, "from=", stop_location, "to=", next_stop)
                return

            try:
                sim_state.move_vehicle(vehicle_id, next_stop)
                log("MOVE_TO_NEXT_STOP", "vehicle_id=", vehicle_id, "target=", next_stop)
            except ValueError as e:
                if list(vehicle["cargo"]):
                    if rebuild_plan_from_current_cargo(sim_state, vehicle_id, vehicle, boxes):
                        return
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("MOVE_TO_NEXT_STOP_FAIL", "vehicle_id=", vehicle_id, "target=", next_stop, "error=", e)
        else:
            try:
                sim_state.move_vehicle(vehicle_id, stop_location)
                log("MOVE_TO_DROPOFF_CONTINUE", "vehicle_id=", vehicle_id, "target=", stop_location)
            except ValueError as e:
                if list(vehicle["cargo"]):
                    if rebuild_plan_from_current_cargo(sim_state, vehicle_id, vehicle, boxes):
                        return
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("MOVE_TO_DROPOFF_CONTINUE_FAIL", "vehicle_id=", vehicle_id, "target=", stop_location, "error=", e)
        return

    if plan["phase"] == "idle":
        log("PLAN_IDLE", "vehicle_id=", vehicle_id, "location=", current_location)

def step(sim_state):
    tick = sim_state.tick
    boxes = sim_state.get_boxes()
    vehicles = sim_state.get_vehicles()

    log("\n==============================")
    log("TICK", tick)
    log("BOX_COUNT", len(boxes))
    log("VEHICLE_COUNT", len(vehicles))

    try:
        active_events = sim_state.get_active_events()
        if not active_events:
            log("get_active_events exists but returned empty list")
        else:
            log("ACTIVE_EVENTS", active_events)
    except AttributeError:
        active_events = []
        log("get_active_events endpoint does not exist")

    emergency_mode = update_stall_state(boxes)
    log("STALLED_TICKS", DISPATCH_STATE["stalled_ticks"])
    log("EMERGENCY_MODE", emergency_mode)

    hubs = build_hubs(boxes)
    refresh_hub_scores(hubs, active_events, emergency_mode=emergency_mode)

    log("HUB_COUNT", len(hubs))
    for location, hub in hubs.items():
        log(
            "HUB",
            "location=", location,
            "cargo_count=", hub.cargo_count,
            "score=", hub.score,
            "best_vehicle=", hub.score_details.get("vehicle"),
            "best_boxes=", hub.score_details.get("box_count"),
            "best_cost=", hub.score_details.get("total_cost"),
            "best_cost_per_box=", hub.score_details.get("cost_per_box"),
            "best_air_penalty=", hub.score_details.get("air_penalty"),
            "best_stops=", [s["location"] for s in hub.score_details.get("stops", [])],
        )

    for vehicle_id, vehicle in vehicles.items():
        log(
            "VEHICLE_BEFORE",
            "vehicle_id=", vehicle_id,
            "type=", normalize_vehicle_name(vehicle["vehicle_type"]),
            "location=", vehicle["location"],
            "cargo=", list(vehicle["cargo"]),
            "plan=", DISPATCH_STATE["vehicle_plans"].get(vehicle_id),
        )
        process_vehicle_plan(sim_state, vehicle_id, vehicle, boxes)

    vehicles = sim_state.get_vehicles()

    for vehicle_id, vehicle in vehicles.items():
        plan = DISPATCH_STATE["vehicle_plans"].get(vehicle_id)
        if plan is None or plan.get("phase") == "idle":
            assign_plan_to_vehicle_from_current_location(
                sim_state,
                vehicle_id,
                vehicle,
                hubs,
                active_events,
                emergency_mode=emergency_mode,
            )

    vehicles = sim_state.get_vehicles()
    sorted_hubs = sorted(hubs.values(), key=lambda h: h.score, reverse=True)

    for hub in sorted_hubs:
        if hub.cargo_count <= 0:
            continue

        assigned_vehicle_id = DISPATCH_STATE["hub_vehicles"].get(hub.location)

        if assigned_vehicle_id is not None and assigned_vehicle_id not in vehicles:
            log("HUB_CLAIM_STALE", "hub=", hub.location, "vehicle_id=", assigned_vehicle_id)
            DISPATCH_STATE["hub_vehicles"].pop(hub.location, None)
            DISPATCH_STATE["vehicle_plans"].pop(assigned_vehicle_id, None)
            assigned_vehicle_id = None

        if assigned_vehicle_id is None:
            log(
                "HUB_UNCLAIMED",
                "hub=", hub.location,
                "cargo_count=", hub.cargo_count,
                "score=", hub.score,
                "emergency_mode=", emergency_mode,
            )
            try_create_best_vehicle(sim_state, hub, active_events, emergency_mode=emergency_mode)
            continue

        plan = DISPATCH_STATE["vehicle_plans"].get(assigned_vehicle_id)
        if plan is None:
            log("HUB_PLAN_MISSING", "hub=", hub.location, "vehicle_id=", assigned_vehicle_id)
            DISPATCH_STATE["hub_vehicles"].pop(hub.location, None)
            try_create_best_vehicle(sim_state, hub, active_events, emergency_mode=emergency_mode)
            continue

        log(
            "HUB_CLAIMED",
            "hub=", hub.location,
            "vehicle_id=", assigned_vehicle_id,
            "phase=", plan.get("phase"),
        )

    if tick == 0:
        log("=== INITIAL HUB SNAPSHOT ===")
        for location, hub in hubs.items():
            log(location, hub)
