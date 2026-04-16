
from simulator import VehicleType, haversine_distance_meters

AIRPORTS = [
    {'id': 'los_angeles_international_airport',         'lat':  33.9425, 'lon': -118.4081},
    {'id': 'john_f_kennedy_international_airport',      'lat':  40.6413, 'lon':  -73.7781},
    {'id': 'ohare_international_airport',               'lat':  41.9742, 'lon':  -87.9073},
    {'id': 'dallas_fort_worth_international_airport',   'lat':  32.8998, 'lon':  -97.0403},
    {'id': 'miami_international_airport',               'lat':  25.7959, 'lon':  -80.2870},
    {'id': 'seattle_tacoma_international_airport',      'lat':  47.4502, 'lon': -122.3088},
    {'id': 'heathrow_airport',                          'lat':  51.4700, 'lon':   -0.4543},
    {'id': 'frankfurt_airport',                         'lat':  50.0379, 'lon':    8.5622},
    {'id': 'dubai_international_airport',               'lat':  25.2532, 'lon':   55.3657},
    {'id': 'chhatrapati_shivaji_maharaj_international', 'lat':  19.0896, 'lon':   72.8656},
    {'id': 'singapore_changi_airport',                  'lat':   1.3644, 'lon':  103.9915},
    {'id': 'haneda_airport',                            'lat':  35.5494, 'lon':  139.7798},
    {'id': 'sydney_kingsford_smith_airport',            'lat': -33.9461, 'lon':  151.1772},
    {'id': 'guarulhos_international_airport',           'lat': -23.4356, 'lon':  -46.4731},
    {'id': 'or_tambo_international_airport',            'lat': -26.1367, 'lon':   28.2411},
    {'id': 'jomo_kenyatta_international_airport',       'lat':  -1.3192, 'lon':   36.9275},
    {'id': 'mexico_city_international_airport',         'lat':  19.4361, 'lon':  -99.0719},
    {'id': 'toronto_pearson_international_airport',     'lat':  43.6777, 'lon':  -79.6248},
]

OCEAN_PORTS = [
    {'id': 'port_of_los_angeles',     'lat':  33.7361, 'lon': -118.2639},
    {'id': 'port_of_new_york_and_nj', 'lat':  40.6681, 'lon':  -74.0455},
    {'id': 'port_of_chicago',         'lat':  41.8800, 'lon':  -87.6200},
    {'id': 'portmiami',               'lat':  25.7781, 'lon':  -80.1794},
    {'id': 'port_of_seattle',         'lat':  47.6026, 'lon': -122.3382},
    {'id': 'port_of_london',          'lat':  51.5074, 'lon':   -0.0174},
    {'id': 'port_of_hamburg',         'lat':  53.5461, 'lon':    9.9661},
    {'id': 'jebel_ali_port',          'lat':  25.0108, 'lon':   55.0617},
    {'id': 'jawaharlal_nehru_port',   'lat':  18.9498, 'lon':   72.9483},
    {'id': 'port_of_singapore',       'lat':   1.2644, 'lon':  103.8200},
    {'id': 'port_of_tokyo',           'lat':  35.6296, 'lon':  139.7773},
    {'id': 'port_botany',             'lat': -33.9656, 'lon':  151.2010},
    {'id': 'port_of_santos',          'lat': -23.9608, 'lon':  -46.3288},
    {'id': 'port_of_durban',          'lat': -29.8713, 'lon':   31.0262},
    {'id': 'port_of_mombasa',         'lat':  -4.0435, 'lon':   39.6682},
    {'id': 'port_of_toronto',         'lat':  43.6407, 'lon':  -79.3590},
    {'id': 'port_of_veracruz',        'lat':  19.2010, 'lon':  -96.1342},
]

SHIPPING_HUBS = [
    {'id': 'los_angeles_distribution_center', 'lat':  33.9425, 'lon': -118.4081},
    {'id': 'new_york_distribution_center',    'lat':  40.6413, 'lon':  -73.7781},
    {'id': 'chicago_distribution_center',     'lat':  41.9742, 'lon':  -87.9073},
    {'id': 'dallas_distribution_center',      'lat':  32.8481, 'lon':  -97.0403},
    {'id': 'miami_distribution_center',       'lat':  25.7959, 'lon':  -80.2870},
    {'id': 'seattle_distribution_center',     'lat':  47.4502, 'lon': -122.3088},
    {'id': 'london_hub',                      'lat':  51.5074, 'lon':   -0.1278},
    {'id': 'frankfurt_distribution_center',   'lat':  50.1109, 'lon':    8.6821},
    {'id': 'dubai_hub',                       'lat':  25.2048, 'lon':   55.2708},
    {'id': 'mumbai_distribution_center',      'lat':  19.0760, 'lon':   72.8777},
    {'id': 'singapore_hub',                   'lat':   1.3521, 'lon':  103.8198},
    {'id': 'tokyo_distribution_center',       'lat':  35.6762, 'lon':  139.6503},
    {'id': 'sydney_hub',                      'lat': -33.8688, 'lon':  151.2093},
    {'id': 'sao_paulo_distribution_center',   'lat': -23.5505, 'lon':  -46.6333},
    {'id': 'johannesburg_hub',                'lat': -26.2041, 'lon':   28.0473},
    {'id': 'nairobi_distribution_center',     'lat':  -1.2921, 'lon':   36.8219},
    {'id': 'mexico_city_hub',                 'lat':  19.4326, 'lon':  -99.1332},
    {'id': 'toronto_hub',                     'lat':  43.6532, 'lon':  -79.3832},
]

# Proximity thresholds (metres)
LOAD_R          = 50.0
ARRIVE_R        = 100.0
INFRA_R         = 5_000.0

# How often to run the dispatch pass, and vehicle selection thresholds
ASSIGN_INTERVAL = 10
TRAIN_MIN_BOXES = 30
SHIP_MIN_BOXES  = 20

# Canal waypoints for ships (Rarely Used)
PANAMA      = (8.99,    -79.57)
SUEZ        = (30.58,    32.26)

# Land waypoints to keep trucks off water
FLORIDA_GW  = (30.33,   -81.65)   # Jacksonville - Florida peninsula
UPPER_CHINA = (43.8627,  118.754) # Inner Mongolia - Singapore <-> Tokyo
CENT_AFRICA = (4.371,    33.366)  # South Sudan - Johannesburg <-> north
EGYPT_GW    = (29.06,    40.00)   # Cairo - Nairobi <-> Dubai/Mumbai
SAO_GW1     = (15.861,  -87.80)  # Guatemala - Central American land bridge
SAO_GW2     = (30.50,  -113.50)   # SE New Mexico - Sao Paulo <-> North America

# Tracks active vehicle plans and prevents double-dispatch
itineraries: dict = {}
claimed: set = set()


def step(sim_state):
    """Main entry point called every tick."""
    _run_vehicles(sim_state)
    if sim_state.tick % ASSIGN_INTERVAL == 0:
        _assign(sim_state)


def _run_vehicles(sim_state):
    """Advance each vehicle with an itinerary that has finished its last move."""
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue

        itin = itineraries.get(vid)
        if itin is None:
            continue

        loc = v["location"]

        if itin["state"] == "TO_PICKUP":
            dist = haversine_distance_meters(loc, itin["pickup_loc"])
            if dist <= LOAD_R:
                loadable = [
                    b for b in itin["box_ids"]
                    if b in boxes
                       and not boxes[b]["delivered"]
                       and boxes[b]["vehicle_id"] is None
                ]
                if loadable:
                    try:
                        sim_state.load_vehicle(vid, loadable)
                        itin["box_ids"] = loadable
                    except Exception:
                        pass
                itin["state"] = "TRANSIT"
                _advance(sim_state, vid, itin)
            else:
                sim_state.move_vehicle(vid, itin["pickup_loc"])

        elif itin["state"] == "TRANSIT":
            if not itin["waypoints"]:
                cargo = list(v["cargo"])
                if cargo:
                    try:
                        sim_state.unload_vehicle(vid, cargo)
                    except Exception:
                        pass
                # Unclaim boxes not yet delivered so the next leg can pick them up
                for b in itin["box_ids"]:
                    if b in boxes and not boxes[b]["delivered"]:
                        claimed.discard(b)
                del itineraries[vid]
            else:
                wp = itin["waypoints"][0]
                dist = haversine_distance_meters(loc, wp)
                if dist <= ARRIVE_R:
                    itin["waypoints"].pop(0)
                    _advance(sim_state, vid, itin)
                else:
                    sim_state.move_vehicle(vid, wp)


def _advance(sim_state, vid, itin):
    """Issue the next move_vehicle call if waypoints remain."""
    if itin["waypoints"]:
        sim_state.move_vehicle(vid, itin["waypoints"][0])


def _assign(sim_state):
    """Group unassigned boxes by (location, destination) and dispatch largest clusters first."""
    boxes = sim_state.get_boxes()

    clusters: dict = {}
    for b_id, box in boxes.items():
        if box["delivered"] or box["vehicle_id"] is not None or b_id in claimed:
            continue
        key = (box["location"], box["destination"])
        if key not in clusters:
            clusters[key] = {"origin": box["location"], "dest": box["destination"], "box_ids": []}
        clusters[key]["box_ids"].append(b_id)

    for c in sorted(clusters.values(), key=lambda c: -len(c["box_ids"])):
        _dispatch(sim_state, c)


def _dispatch(sim_state, cluster):
    """
    Assign the best vehicle for a cluster based on where the boxes are and where they're going.
    Three cases:
      A — same region: direct ground transport
      B — cross-region, at a port: ship (large batch) or plane (small batch)
      C — cross-region, not at a port: drive to nearest same-region port first
    """
    origin = cluster["origin"]
    dest = cluster["dest"]
    box_ids = cluster["box_ids"]
    n = len(box_ids)

    if n == 0:
        return False

    src_region = _region(origin)
    dst_region = _region(dest)
    at_port = _near_any(origin, OCEAN_PORTS)

    if src_region == dst_region:
        vtype = VehicleType.Train if n >= TRAIN_MIN_BOXES else VehicleType.SemiTruck
        selected = box_ids[:vtype.value.capacity]
        vid = _find_idle_ground(sim_state, vtype, origin)
        if not vid:
            vid = _spawn(sim_state, vtype, _nearest(origin, SHIPPING_HUBS))
        if not vid:
            return False
        claimed.update(selected)
        itineraries[vid] = {
            "state": "TO_PICKUP", "pickup_loc": origin,
            "box_ids": selected, "waypoints": _land_wps(origin, dest) + [dest],
        }
        return True

    elif at_port:
        dst_port = _nearest(dest, OCEAN_PORTS)
        if n >= SHIP_MIN_BOXES:
            vtype = VehicleType.CargoShip
            selected = box_ids[:vtype.value.capacity]
            wps = _ocean_wps(origin, dst_port) + [dst_port]
            spawn = origin
        else:
            vtype = VehicleType.Airplane
            selected = box_ids[:vtype.value.capacity]
            wps = [_nearest(dest, AIRPORTS)]
            spawn = _nearest(origin, AIRPORTS)
        vid = _spawn(sim_state, vtype, spawn)
        if not vid:
            return False
        claimed.update(selected)
        itineraries[vid] = {
            "state": "TO_PICKUP", "pickup_loc": origin,
            "box_ids": selected, "waypoints": wps,
        }
        return True

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
        "reject_reasons": reject_reasons,
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


def refresh_hub_scores(hubs, active_events):
    for hub in hubs.values():
        best = best_route_for_hub(hub, active_events=active_events)
        if best is None:
            hub.score = float("-inf")
            hub.score_details = {}
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


def try_create_best_vehicle(sim_state, hub, active_events):
    best = best_route_for_hub(hub, active_events=active_events)
    if best is None:
        log("CREATE_SKIP", "hub=", hub.location, "reason=no_best_route")
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
            "surface_candidates=", best.get("cheaper_surface_candidates"),
        )
        return vehicle_id
    except ValueError as e:
        log("CREATE_FAIL", "hub=", hub.location, "vehicle=", vehicle_name, "error=", e)
        return None


def assign_plan_to_vehicle_from_current_location(sim_state, vehicle_id, vehicle, hubs, active_events):
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
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("PLAN_IDLE_BAD_ROUTE_FIRST_STOP", "vehicle_id=", vehicle_id, "from=", pickup_location, "to=", first_stop)
                return

            plan["phase"] = "to_dropoff"
            try:
                sim_state.move_vehicle(vehicle_id, first_stop)
                log("MOVE_TO_FIRST_STOP", "vehicle_id=", vehicle_id, "target=", first_stop)
            except ValueError as e:
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("MOVE_TO_FIRST_STOP_FAIL", "vehicle_id=", vehicle_id, "target=", first_stop, "error=", e)
        else:
            try:
                sim_state.move_vehicle(vehicle_id, pickup_location)
                log("MOVE_TO_PICKUP_CONTINUE", "vehicle_id=", vehicle_id, "target=", pickup_location)
            except ValueError as e:
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
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("PLAN_IDLE_BAD_ROUTE_NEXT", "vehicle_id=", vehicle_id, "from=", stop_location, "to=", next_stop)
                return

            try:
                sim_state.move_vehicle(vehicle_id, next_stop)
                log("MOVE_TO_NEXT_STOP", "vehicle_id=", vehicle_id, "target=", next_stop)
            except ValueError as e:
                plan["phase"] = "idle"
                clear_vehicle_hub_claim(vehicle_id)
                log("MOVE_TO_NEXT_STOP_FAIL", "vehicle_id=", vehicle_id, "target=", next_stop, "error=", e)
        else:
            try:
                sim_state.move_vehicle(vehicle_id, stop_location)
                log("MOVE_TO_DROPOFF_CONTINUE", "vehicle_id=", vehicle_id, "target=", stop_location)
            except ValueError as e:
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

    hubs = build_hubs(boxes)
    refresh_hub_scores(hubs, active_events)

    log("STALL_TICKS", DISPATCH_STATE["stalled_ticks"])
    log("EMERGENCY_MODE", emergency_mode)
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
            assign_plan_to_vehicle_from_current_location(sim_state, vehicle_id, vehicle, hubs, active_events)

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
            log("HUB_UNCLAIMED", "hub=", hub.location, "cargo_count=", hub.cargo_count, "score=", hub.score)
            try_create_best_vehicle(sim_state, hub, active_events)
            continue

        plan = DISPATCH_STATE["vehicle_plans"].get(assigned_vehicle_id)
        if plan is None:
            log("HUB_PLAN_MISSING", "hub=", hub.location, "vehicle_id=", assigned_vehicle_id)
            DISPATCH_STATE["hub_vehicles"].pop(hub.location, None)
            try_create_best_vehicle(sim_state, hub, active_events)
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
=======

AIRPORTS = [
    {'id': 'los_angeles_international_airport',         'lat':  33.9425, 'lon': -118.4081},
    {'id': 'john_f_kennedy_international_airport',      'lat':  40.6413, 'lon':  -73.7781},
    {'id': 'ohare_international_airport',               'lat':  41.9742, 'lon':  -87.9073},
    {'id': 'dallas_fort_worth_international_airport',   'lat':  32.8998, 'lon':  -97.0403},
    {'id': 'miami_international_airport',               'lat':  25.7959, 'lon':  -80.2870},
    {'id': 'seattle_tacoma_international_airport',      'lat':  47.4502, 'lon': -122.3088},
    {'id': 'heathrow_airport',                          'lat':  51.4700, 'lon':   -0.4543},
    {'id': 'frankfurt_airport',                         'lat':  50.0379, 'lon':    8.5622},
    {'id': 'dubai_international_airport',               'lat':  25.2532, 'lon':   55.3657},
    {'id': 'chhatrapati_shivaji_maharaj_international', 'lat':  19.0896, 'lon':   72.8656},
    {'id': 'singapore_changi_airport',                  'lat':   1.3644, 'lon':  103.9915},
    {'id': 'haneda_airport',                            'lat':  35.5494, 'lon':  139.7798},
    {'id': 'sydney_kingsford_smith_airport',            'lat': -33.9461, 'lon':  151.1772},
    {'id': 'guarulhos_international_airport',           'lat': -23.4356, 'lon':  -46.4731},
    {'id': 'or_tambo_international_airport',            'lat': -26.1367, 'lon':   28.2411},
    {'id': 'jomo_kenyatta_international_airport',       'lat':  -1.3192, 'lon':   36.9275},
    {'id': 'mexico_city_international_airport',         'lat':  19.4361, 'lon':  -99.0719},
    {'id': 'toronto_pearson_international_airport',     'lat':  43.6777, 'lon':  -79.6248},
]

OCEAN_PORTS = [
    {'id': 'port_of_los_angeles',     'lat':  33.7361, 'lon': -118.2639},
    {'id': 'port_of_new_york_and_nj', 'lat':  40.6681, 'lon':  -74.0455},
    {'id': 'port_of_chicago',         'lat':  41.8800, 'lon':  -87.6200},
    {'id': 'portmiami',               'lat':  25.7781, 'lon':  -80.1794},
    {'id': 'port_of_seattle',         'lat':  47.6026, 'lon': -122.3382},
    {'id': 'port_of_london',          'lat':  51.5074, 'lon':   -0.0174},
    {'id': 'port_of_hamburg',         'lat':  53.5461, 'lon':    9.9661},
    {'id': 'jebel_ali_port',          'lat':  25.0108, 'lon':   55.0617},
    {'id': 'jawaharlal_nehru_port',   'lat':  18.9498, 'lon':   72.9483},
    {'id': 'port_of_singapore',       'lat':   1.2644, 'lon':  103.8200},
    {'id': 'port_of_tokyo',           'lat':  35.6296, 'lon':  139.7773},
    {'id': 'port_botany',             'lat': -33.9656, 'lon':  151.2010},
    {'id': 'port_of_santos',          'lat': -23.9608, 'lon':  -46.3288},
    {'id': 'port_of_durban',          'lat': -29.8713, 'lon':   31.0262},
    {'id': 'port_of_mombasa',         'lat':  -4.0435, 'lon':   39.6682},
    {'id': 'port_of_toronto',         'lat':  43.6407, 'lon':  -79.3590},
    {'id': 'port_of_veracruz',        'lat':  19.2010, 'lon':  -96.1342},
]

SHIPPING_HUBS = [
    {'id': 'los_angeles_distribution_center', 'lat':  33.9425, 'lon': -118.4081},
    {'id': 'new_york_distribution_center',    'lat':  40.6413, 'lon':  -73.7781},
    {'id': 'chicago_distribution_center',     'lat':  41.9742, 'lon':  -87.9073},
    {'id': 'dallas_distribution_center',      'lat':  32.8481, 'lon':  -97.0403},
    {'id': 'miami_distribution_center',       'lat':  25.7959, 'lon':  -80.2870},
    {'id': 'seattle_distribution_center',     'lat':  47.4502, 'lon': -122.3088},
    {'id': 'london_hub',                      'lat':  51.5074, 'lon':   -0.1278},
    {'id': 'frankfurt_distribution_center',   'lat':  50.1109, 'lon':    8.6821},
    {'id': 'dubai_hub',                       'lat':  25.2048, 'lon':   55.2708},
    {'id': 'mumbai_distribution_center',      'lat':  19.0760, 'lon':   72.8777},
    {'id': 'singapore_hub',                   'lat':   1.3521, 'lon':  103.8198},
    {'id': 'tokyo_distribution_center',       'lat':  35.6762, 'lon':  139.6503},
    {'id': 'sydney_hub',                      'lat': -33.8688, 'lon':  151.2093},
    {'id': 'sao_paulo_distribution_center',   'lat': -23.5505, 'lon':  -46.6333},
    {'id': 'johannesburg_hub',                'lat': -26.2041, 'lon':   28.0473},
    {'id': 'nairobi_distribution_center',     'lat':  -1.2921, 'lon':   36.8219},
    {'id': 'mexico_city_hub',                 'lat':  19.4326, 'lon':  -99.1332},
    {'id': 'toronto_hub',                     'lat':  43.6532, 'lon':  -79.3832},
]

# Proximity thresholds (metres)
LOAD_R          = 50.0
ARRIVE_R        = 100.0
INFRA_R         = 5_000.0

# How often to run the dispatch pass, and vehicle selection thresholds
ASSIGN_INTERVAL = 10
TRAIN_MIN_BOXES = 30
SHIP_MIN_BOXES  = 20

# Canal waypoints for ships (Rarely Used)
PANAMA      = (8.99,    -79.57)
SUEZ        = (30.58,    32.26)

# Land waypoints to keep trucks off water
FLORIDA_GW  = (30.33,   -81.65)   # Jacksonville - Florida peninsula
UPPER_CHINA = (43.8627,  118.754) # Inner Mongolia - Singapore <-> Tokyo
CENT_AFRICA = (4.371,    33.366)  # South Sudan - Johannesburg <-> north
EGYPT_GW    = (29.06,    40.00)   # Cairo - Nairobi <-> Dubai/Mumbai
SAO_GW1     = (15.861,  -87.80)  # Guatemala - Central American land bridge
SAO_GW2     = (30.50,  -113.50)   # SE New Mexico - Sao Paulo <-> North America

# Tracks active vehicle plans and prevents double-dispatch
itineraries: dict = {}
claimed: set = set()


def step(sim_state):
    """Main entry point called every tick."""
    _run_vehicles(sim_state)
    if sim_state.tick % ASSIGN_INTERVAL == 0:
        _assign(sim_state)


def _run_vehicles(sim_state):
    """Advance each vehicle with an itinerary that has finished its last move."""
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue

        itin = itineraries.get(vid)
        if itin is None:
            continue

        loc = v["location"]

        if itin["state"] == "TO_PICKUP":
            dist = haversine_distance_meters(loc, itin["pickup_loc"])
            if dist <= LOAD_R:
                loadable = [
                    b for b in itin["box_ids"]
                    if b in boxes
                       and not boxes[b]["delivered"]
                       and boxes[b]["vehicle_id"] is None
                ]
                if loadable:
                    try:
                        sim_state.load_vehicle(vid, loadable)
                        itin["box_ids"] = loadable
                    except Exception:
                        pass
                itin["state"] = "TRANSIT"
                _advance(sim_state, vid, itin)
            else:
                sim_state.move_vehicle(vid, itin["pickup_loc"])

        elif itin["state"] == "TRANSIT":
            if not itin["waypoints"]:
                cargo = list(v["cargo"])
                if cargo:
                    try:
                        sim_state.unload_vehicle(vid, cargo)
                    except Exception:
                        pass
                # Unclaim boxes not yet delivered so the next leg can pick them up
                for b in itin["box_ids"]:
                    if b in boxes and not boxes[b]["delivered"]:
                        claimed.discard(b)
                del itineraries[vid]
            else:
                wp = itin["waypoints"][0]
                dist = haversine_distance_meters(loc, wp)
                if dist <= ARRIVE_R:
                    itin["waypoints"].pop(0)
                    _advance(sim_state, vid, itin)
                else:
                    sim_state.move_vehicle(vid, wp)


def _advance(sim_state, vid, itin):
    """Issue the next move_vehicle call if waypoints remain."""
    if itin["waypoints"]:
        sim_state.move_vehicle(vid, itin["waypoints"][0])


def _assign(sim_state):
    """Group unassigned boxes by (location, destination) and dispatch largest clusters first."""
    boxes = sim_state.get_boxes()

    clusters: dict = {}
    for b_id, box in boxes.items():
        if box["delivered"] or box["vehicle_id"] is not None or b_id in claimed:
            continue
        key = (box["location"], box["destination"])
        if key not in clusters:
            clusters[key] = {"origin": box["location"], "dest": box["destination"], "box_ids": []}
        clusters[key]["box_ids"].append(b_id)

    for c in sorted(clusters.values(), key=lambda c: -len(c["box_ids"])):
        _dispatch(sim_state, c)


def _dispatch(sim_state, cluster):
    """
    Assign the best vehicle for a cluster based on where the boxes are and where they're going.
    Three cases:
      A — same region: direct ground transport
      B — cross-region, at a port: ship (large batch) or plane (small batch)
      C — cross-region, not at a port: drive to nearest same-region port first
    """
    origin = cluster["origin"]
    dest = cluster["dest"]
    box_ids = cluster["box_ids"]
    n = len(box_ids)

    if n == 0:
        return False

    src_region = _region(origin)
    dst_region = _region(dest)
    at_port = _near_any(origin, OCEAN_PORTS)

    if src_region == dst_region:
        vtype = VehicleType.Train if n >= TRAIN_MIN_BOXES else VehicleType.SemiTruck
        selected = box_ids[:vtype.value.capacity]
        vid = _find_idle_ground(sim_state, vtype, origin)
        if not vid:
            vid = _spawn(sim_state, vtype, _nearest(origin, SHIPPING_HUBS))
        if not vid:
            return False
        claimed.update(selected)
        itineraries[vid] = {
            "state": "TO_PICKUP", "pickup_loc": origin,
            "box_ids": selected, "waypoints": _land_wps(origin, dest) + [dest],
        }
        return True

    elif at_port:
        dst_port = _nearest(dest, OCEAN_PORTS)
        if n >= SHIP_MIN_BOXES:
            vtype = VehicleType.CargoShip
            selected = box_ids[:vtype.value.capacity]
            wps = _ocean_wps(origin, dst_port) + [dst_port]
            spawn = origin
        else:
            vtype = VehicleType.Airplane
            selected = box_ids[:vtype.value.capacity]
            wps = [_nearest(dest, AIRPORTS)]
            spawn = _nearest(origin, AIRPORTS)
        vid = _spawn(sim_state, vtype, spawn)
        if not vid:
            return False
        claimed.update(selected)
        itineraries[vid] = {
            "state": "TO_PICKUP", "pickup_loc": origin,
            "box_ids": selected, "waypoints": wps,
        }
        return True

    else:
        src_port = _nearest_same_region(origin, OCEAN_PORTS) or _nearest(origin, OCEAN_PORTS)
        vtype = VehicleType.Train if n >= TRAIN_MIN_BOXES else VehicleType.SemiTruck
        selected = box_ids[:vtype.value.capacity]
        vid = _find_idle_ground(sim_state, vtype, origin)
        if not vid:
            vid = _spawn(sim_state, vtype, _nearest(origin, SHIPPING_HUBS))
        if not vid:
            return False
        claimed.update(selected)
        itineraries[vid] = {
            "state": "TO_PICKUP", "pickup_loc": origin,
            "box_ids": selected, "waypoints": _land_wps(origin, src_port) + [src_port],
        }
        return True


def _find_idle_ground(sim_state, vtype, pickup_loc):
    """Return the nearest idle vehicle of the given type if reusing it is cheaper than spawning a new one."""
    best_vid, best_cost = None, float("inf")
    for vid, v in sim_state.get_vehicles().items():
        if v["vehicle_type"] != vtype.name or v["destination"] is not None:
            continue
        itin = itineraries.get(vid)
        if itin is not None and itin.get("waypoints"):
            continue
        cost = haversine_distance_meters(v["location"], pickup_loc) / 1000.0 * vtype.value.per_km_cost
        if cost < (vtype.value.base_cost * 2) and cost < best_cost:
            best_cost, best_vid = cost, vid
    return best_vid


def _region(loc):
    """Return the broad continental region for a (lat, lon) coordinate."""
    lat, lon = loc
    if lon < -30:              return "AMERICAS"
    if lat > 0 and lon >= 100: return "EAST_ASIA"
    if lat <= 0 and lon >= 100: return "OCEANIA"
    return "EURASIA_AFRICA"


def _nearest(loc, infra):
    """Return the (lat, lon) of the closest item in an infrastructure list."""
    best_d, best_c = float("inf"), None
    for item in infra:
        d = haversine_distance_meters(loc, (item["lat"], item["lon"]))
        if d < best_d:
            best_d, best_c = d, (item["lat"], item["lon"])
    return best_c


def _nearest_same_region(loc, infra):
    """Return the closest infra item in the same region as loc, or None."""
    loc_region = _region(loc)
    best_d, best_c = float("inf"), None
    for item in infra:
        coord = (item["lat"], item["lon"])
        if _region(coord) != loc_region:
            continue
        d = haversine_distance_meters(loc, coord)
        if d < best_d:
            best_d, best_c = d, coord
    return best_c


def _near_any(loc, infra):
    """Return True if loc is within INFRA_R metres of any item in the list."""
    return any(haversine_distance_meters(loc, (i["lat"], i["lon"])) <= INFRA_R for i in infra)


def _ocean_wps(src, dst):
    """Return a Panama or Suez waypoint if needed to keep a ship in navigable water."""
    sx, dx = src[1], dst[1]
    if sx < -30 and -30 <= dx < 100:  return [PANAMA]
    if -30 <= sx < 100 and dx < -30:  return [PANAMA]
    if -30 <= sx < 100 and dx >= 100: return [SUEZ]
    if sx >= 100 and -30 <= dx < 100: return [SUEZ]
    return []


def _land_wps(src, dst):
    """Return waypoints to keep a ground vehicle on solid ground between src and dst."""
    wps = []
    src_r = _region(src)
    dst_r = _region(dst)

    if src_r == "AMERICAS" and dst_r == "AMERICAS":
        def _south_fl(loc):
            return loc[0] < 30.5 and loc[1] > -82.5

        def _south_am(loc):
            return loc[0] < 9.0

        def _north_am(loc):
            return loc[0] > 25.0

        if _south_am(src) and _north_am(dst):
            wps.extend([SAO_GW1, SAO_GW2])
            if _south_fl(dst): wps.append(FLORIDA_GW)
            return wps
        if _north_am(src) and _south_am(dst):
            if _south_fl(src): wps.append(FLORIDA_GW)
            wps.extend([SAO_GW2, SAO_GW1])
            return wps
        if _south_fl(src) != _south_fl(dst):
            wps.append(FLORIDA_GW)
        return wps

    if src_r == "EURASIA_AFRICA" and dst_r == "EURASIA_AFRICA":
        def _s_africa(loc):
            return loc[0] < -15.0 and 15.0 < loc[1] < 45.0

        def _e_africa(loc):
            return -5.0 < loc[0] < 15.0 and 30.0 < loc[1] < 50.0

        def _mideast(loc):
            return loc[0] > 10.0 and 45.0 < loc[1] < 85.0

        if _s_africa(src) != _s_africa(dst):
            wps.append(CENT_AFRICA)
        if _e_africa(src) and _mideast(dst) or _mideast(src) and _e_africa(dst):
            wps.append(EGYPT_GW)
        return wps

    if src_r == "EAST_ASIA" and dst_r == "EAST_ASIA":
        def _se_asia(loc):
            return loc[0] < 15.0 and loc[1] >= 100.0

        def _japan(loc):
            return loc[0] > 30.0 and loc[1] > 125.0

        if _se_asia(src) and _japan(dst) or _japan(src) and _se_asia(dst):
            wps.append(UPPER_CHINA)
        return wps

    if {src_r, dst_r} == {"EAST_ASIA", "EURASIA_AFRICA"}:
        wps.append(EGYPT_GW)

    return wps


def _spawn(sim_state, vtype, preferred_loc):
    """Try to spawn a vehicle at preferred_loc, falling back through the appropriate infra list."""
    if vtype in (VehicleType.SemiTruck, VehicleType.Train):
        fallback = [h for h in SHIPPING_HUBS if _region((h["lat"], h["lon"])) != "OCEANIA"]
    elif vtype == VehicleType.CargoShip:
        fallback = OCEAN_PORTS
    else:
        fallback = AIRPORTS

    for loc in [preferred_loc] + [(i["lat"], i["lon"]) for i in fallback]:
        try:
            return sim_state.create_vehicle(vtype, loc)
        except (ValueError, TypeError):
            continue
    return None
>>>>>>> 028c5ea41f1b2d0b55e092c54f28d7f926970e14
