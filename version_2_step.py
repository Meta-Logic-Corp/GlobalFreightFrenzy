from simulator import VehicleType, haversine_distance_meters

_PROXIMITY_M = 50.0


def _get_active_events(sim_state):
    if hasattr(sim_state, "get_active_events"):
        try:
            return sim_state.get_active_events() or []
        except Exception:
            return []
    return []


def _distance_m(a, b):
    return haversine_distance_meters(a, b)


def _get_vehicle_capacity(vehicle_type_name):
    cfg = VehicleType[vehicle_type_name].value
    return getattr(cfg, "capacity", 0)


def _spawn_best_available_vehicle(sim_state, location):
    preferred_order = [
        VehicleType.SemiTruck,
        VehicleType.Train,
        VehicleType.Airplane,
        VehicleType.Drone,
        VehicleType.CargoShip,
    ]

    for vtype in preferred_order:
        try:
            vid = sim_state.create_vehicle(vtype, location)
            print(f"created {vid} type={vtype.name} at {location}")
            return vid
        except Exception as e:
            print(f"failed to create {vtype.name} at {location}: {e}")

    return None


def _boxes_at_location(boxes, location):
    out = []
    for bid, box in boxes.items():
        if box["delivered"]:
            continue
        if box["vehicle_id"] is not None:
            continue
        if _distance_m(location, box["location"]) <= _PROXIMITY_M:
            out.append(bid)
    return out


def _deliverable_for_vehicle(vehicle, boxes):
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


def _group_boxes_by_destination(box_ids, boxes):
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


def _pick_best_load_for_vehicle(vehicle, boxes):
    loc = vehicle["location"]
    available = _boxes_at_location(boxes, loc)
    if not available:
        return []

    capacity = _get_vehicle_capacity(vehicle["vehicle_type"])
    remaining = max(0, capacity - len(vehicle["cargo"]))
    if remaining <= 0:
        return []

    groups = _group_boxes_by_destination(available, boxes)
    if not groups:
        return []

    best = groups[0]["box_ids"][:remaining]
    return best


def _pick_next_destination(vehicle, boxes):
    if not vehicle["cargo"]:
        return None

    loc = vehicle["location"]
    groups = {}

    for bid in vehicle["cargo"]:
        if bid not in boxes:
            continue
        box = boxes[bid]
        if box["delivered"]:
            continue

        dest = box["destination"]
        key = (round(dest[0], 5), round(dest[1], 5))
        if key not in groups:
            groups[key] = {
                "destination": dest,
                "count": 0,
                "distance": _distance_m(loc, dest),
            }
        groups[key]["count"] += 1

    if not groups:
        return None

    ranked = sorted(
        groups.values(),
        key=lambda g: (-g["count"], g["distance"])
    )
    return ranked[0]["destination"]


def _spawn_initial_vehicles(sim_state):
    boxes = sim_state.get_boxes()
    hubs = {}

    for bid, box in boxes.items():
        if box["delivered"]:
            continue
        hubs.setdefault(box["location"], []).append(bid)

    print(f"tick 0: found {len(hubs)} hubs")

    for hub_loc, hub_box_ids in hubs.items():
        print(f"hub {hub_loc} has {len(hub_box_ids)} boxes")
        vid = _spawn_best_available_vehicle(sim_state, hub_loc)
        if vid is None:
            print(f"no valid vehicle could be created at hub {hub_loc}")
            continue

        vehicles = sim_state.get_vehicles()
        if vid not in vehicles:
            print(f"vehicle {vid} missing right after creation")
            continue

        vehicle = vehicles[vid]
        boxes = sim_state.get_boxes()

        to_load = _pick_best_load_for_vehicle(vehicle, boxes)
        if to_load:
            try:
                sim_state.load_vehicle(vid, to_load)
                print(f"loaded {len(to_load)} boxes onto {vid}")
            except Exception as e:
                print(f"failed to load {vid}: {e}")
        else:
            print(f"nothing loadable for {vid} at spawn hub")

        vehicles = sim_state.get_vehicles()
        boxes = sim_state.get_boxes()
        vehicle = vehicles.get(vid)

        if vehicle and vehicle["cargo"]:
            next_dest = _pick_next_destination(vehicle, boxes)
            if next_dest is not None:
                try:
                    sim_state.move_vehicle(vid, next_dest)
                    print(f"moving {vid} to {next_dest}")
                except Exception as e:
                    print(f"failed to move {vid}: {e}")


def _manage_vehicles(sim_state):
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    for vid, vehicle in vehicles.items():
        if vehicle["destination"] is not None:
            continue

        deliverable = _deliverable_for_vehicle(vehicle, boxes)
        if deliverable:
            try:
                sim_state.unload_vehicle(vid, deliverable)
                print(f"unloaded {len(deliverable)} boxes from {vid}")
            except Exception as e:
                print(f"failed to unload {vid}: {e}")

            vehicles = sim_state.get_vehicles()
            boxes = sim_state.get_boxes()
            vehicle = vehicles.get(vid)
            if vehicle is None:
                continue

        if not vehicle["cargo"]:
            to_load = _pick_best_load_for_vehicle(vehicle, boxes)
            if to_load:
                try:
                    sim_state.load_vehicle(vid, to_load)
                    print(f"loaded {len(to_load)} boxes onto idle {vid}")
                except Exception as e:
                    print(f"failed to load idle {vid}: {e}")

                vehicles = sim_state.get_vehicles()
                boxes = sim_state.get_boxes()
                vehicle = vehicles.get(vid)
                if vehicle is None:
                    continue

        if vehicle["cargo"]:
            next_dest = _pick_next_destination(vehicle, boxes)
            if next_dest is not None:
                try:
                    sim_state.move_vehicle(vid, next_dest)
                    print(f"moving {vid} to {next_dest}")
                except Exception as e:
                    print(f"failed to move {vid}: {e}")


def _spawn_for_unserved_hubs(sim_state):
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    idle_vehicle_locations = set()
    for vehicle in vehicles.values():
        if vehicle["destination"] is None:
            idle_vehicle_locations.add(vehicle["location"])

    hubs = {}
    for bid, box in boxes.items():
        if box["delivered"]:
            continue
        if box["vehicle_id"] is not None:
            continue
        hubs.setdefault(box["location"], []).append(bid)

    for hub_loc, hub_box_ids in hubs.items():
        if hub_loc in idle_vehicle_locations:
            continue

        if len(hub_box_ids) == 0:
            continue

        print(f"trying extra spawn at hub {hub_loc} with {len(hub_box_ids)} waiting boxes")
        vid = _spawn_best_available_vehicle(sim_state, hub_loc)
        if vid is None:
            continue

        vehicles = sim_state.get_vehicles()
        boxes = sim_state.get_boxes()
        vehicle = vehicles.get(vid)
        if vehicle is None:
            continue

        to_load = _pick_best_load_for_vehicle(vehicle, boxes)
        if to_load:
            try:
                sim_state.load_vehicle(vid, to_load)
                print(f"loaded {len(to_load)} boxes onto new vehicle {vid}")
            except Exception as e:
                print(f"failed loading new vehicle {vid}: {e}")

        vehicles = sim_state.get_vehicles()
        boxes = sim_state.get_boxes()
        vehicle = vehicles.get(vid)
        if vehicle and vehicle["cargo"]:
            next_dest = _pick_next_destination(vehicle, boxes)
            if next_dest is not None:
                try:
                    sim_state.move_vehicle(vid, next_dest)
                    print(f"moving new vehicle {vid} to {next_dest}")
                except Exception as e:
                    print(f"failed moving new vehicle {vid}: {e}")


def step(sim_state):
    print(f"tick={sim_state.tick} total_cost={sim_state.total_cost}")

    if sim_state.tick == 0:
        boxes = sim_state.get_boxes()
        print(f"boxes at start: {len(boxes)}")
        print("has get_active_events:", hasattr(sim_state, "get_active_events"))
        events = _get_active_events(sim_state)
        if events:
            print("active events:", events)
        _spawn_initial_vehicles(sim_state)
        return

    _manage_vehicles(sim_state)
    _spawn_for_unserved_hubs(sim_state)
