from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Vehicle capabilities
VEHICLE_RESTRICTIONS = {
    VehicleType.SemiTruck: {"terrain": "land", "spawn": "hub", "load_unload": "hub", "max_distance": None},
    VehicleType.Train: {"terrain": "land", "spawn": "hub", "load_unload": "hub", "max_distance": None},
    VehicleType.Airplane: {"terrain": "any", "spawn": "airport", "load_unload": "airport", "max_distance": None},
    VehicleType.CargoShip: {"terrain": "water", "spawn": "ocean_port", "load_unload": "ocean_port", "max_distance": None},
    VehicleType.Drone: {"terrain": "any", "spawn": "airport", "load_unload": "airport", "max_distance": 20000}
}

def is_water_crossing(from_loc, to_loc):
    """Detect if a route requires crossing an ocean (overseas)."""
    dist = haversine_distance_meters(from_loc, to_loc)
    if dist > 500000:  # 500 km
        return True
    from_lat, from_lon = from_loc
    to_lat, to_lon = to_loc
    # Atlantic crossing
    if (from_lon < -60 and to_lon > -10) or (from_lon > -10 and to_lon < -60):
        if abs(from_lat - to_lat) < 50:
            return True
    # Pacific crossing
    if (from_lon < -120 and to_lon > 120) or (from_lon > 120 and to_lon < -120):
        return True
    return False

def can_vehicle_handle_route(vtype, from_loc, to_loc, num_boxes):
    """Check if a vehicle can legally travel this route (capacity, distance)."""
    if num_boxes > vtype.value.capacity:
        return False
    if VEHICLE_RESTRICTIONS[vtype]["max_distance"]:
        dist = haversine_distance_meters(from_loc, to_loc)
        if dist > VEHICLE_RESTRICTIONS[vtype]["max_distance"]:
            return False
    return True

def nearest_facility(loc, facilities):
    """Return the closest facility (airport/port/hub) to a location."""
    if not facilities:
        return None
    return min(facilities, key=lambda f: haversine_distance_meters(loc, f))

def is_valid_operation_location(vtype, loc, sim_state):
    """Check if a vehicle can load/unload at this location."""
    if vtype in [VehicleType.SemiTruck, VehicleType.Train]:
        hubs = sim_state.get_shipping_hubs()
        return any(haversine_distance_meters(loc, h) <= 5000 for h in hubs)
    elif vtype == VehicleType.Airplane:
        airports = sim_state.get_airports()
        if not airports:
            airports = sim_state.get_shipping_hubs()
        return any(haversine_distance_meters(loc, a) <= 5000 for a in airports)
    elif vtype == VehicleType.CargoShip:
        ports = sim_state.get_ocean_ports()
        return any(haversine_distance_meters(loc, p) <= 5000 for p in ports)
    return True

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    # Get real facility locations from the API
    hubs = sim_state.get_shipping_hubs()
    airports = sim_state.get_airports()
    ocean_ports = sim_state.get_ocean_ports()
    if not airports:
        airports = hubs  # fallback

    # ---------- 1. UNLOAD (with facility check) ----------
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            vtype = VehicleType[v["vehicle_type"]]
            loc = v["location"]
            if is_valid_operation_location(vtype, loc, sim_state):
                to_unload = [bid for bid in v["cargo"] if haversine_distance_meters(loc, boxes[bid]["destination"]) <= _PROXIMITY_M]
                if to_unload:
                    sim_state.unload_vehicle(vid, to_unload)
                    boxes = sim_state.get_boxes()
            else:
                # Not at valid facility – go to the nearest one
                if vtype == VehicleType.Airplane:
                    target = nearest_facility(loc, airports)
                elif vtype == VehicleType.CargoShip:
                    target = nearest_facility(loc, ocean_ports)
                else:
                    target = nearest_facility(loc, hubs)
                if target:
                    sim_state.move_vehicle(vid, target)

    # ---------- 2. MANAGE VEHICLES (move, transfer) ----------
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue
        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        capacity_left = vtype.value.capacity - len(v["cargo"])

        # Load boxes at current location (if any)
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

            # --- Case 1: Vehicle can handle the whole route directly ---
            if can_vehicle_handle_route(vtype, loc, dest, num_boxes):
                sim_state.move_vehicle(vid, dest)

            # --- Case 2: Land vehicle trying to go overseas -> go to port/airport ---
            elif vtype in [VehicleType.SemiTruck, VehicleType.Train] and is_water_crossing(loc, dest):
                # Find the nearest port or airport as a waypoint
                port_dist = float('inf')
                airport_dist = float('inf')
                nearest_port = nearest_facility(loc, ocean_ports)
                nearest_airport = nearest_facility(loc, airports)
                if nearest_port:
                    port_dist = haversine_distance_meters(loc, nearest_port)
                if nearest_airport:
                    airport_dist = haversine_distance_meters(loc, nearest_airport)

                if port_dist < airport_dist and port_dist < 500000:
                    waypoint = nearest_port
                else:
                    waypoint = nearest_airport if airport_dist < 500000 else None

                if waypoint:
                    sim_state.move_vehicle(vid, waypoint)
                else:
                    # fallback: go to any hub
                    hub = nearest_facility(loc, hubs)
                    if hub:
                        sim_state.move_vehicle(vid, hub)

            # --- Case 3: Water/air vehicle that has reached land -> transfer to land vehicle ---
            elif vtype in [VehicleType.CargoShip, VehicleType.Airplane] and not is_water_crossing(loc, dest):
                cargo_copy = list(v["cargo"])
                dest_copy = dest
                sim_state.unload_vehicle(vid, cargo_copy)
                boxes = sim_state.get_boxes()
                # Choose train for bulk, otherwise truck
                land_type = VehicleType.Train if len(cargo_copy) >= 20 else VehicleType.SemiTruck
                try:
                    new_vid = sim_state.create_vehicle(land_type, loc)
                    sim_state.load_vehicle(new_vid, cargo_copy)
                    sim_state.move_vehicle(new_vid, dest_copy)
                except ValueError:
                    # fallback to truck
                    try:
                        new_vid = sim_state.create_vehicle(VehicleType.SemiTruck, loc)
                        sim_state.load_vehicle(new_vid, cargo_copy)
                        sim_state.move_vehicle(new_vid, dest_copy)
                    except ValueError:
                        sim_state.move_vehicle(vid, dest_copy)

            # --- Case 4: Other mismatches – go to nearest hub ---
            else:
                hub = nearest_facility(loc, hubs)
                if hub:
                    sim_state.move_vehicle(vid, hub)
                else:
                    sim_state.move_vehicle(vid, dest)

        # Empty vehicle: go to nearest undelivered box or hub
        elif not v["cargo"]:
            nearest_box = None
            min_dist = float('inf')
            for bid, box in boxes.items():
                if not box["delivered"] and box["vehicle_id"] is None:
                    dist = haversine_distance_meters(loc, box["location"])
                    if dist < min_dist:
                        min_dist = dist
                        nearest_box = box["location"]
            if nearest_box:
                sim_state.move_vehicle(vid, nearest_box)
            elif hubs:
                sim_state.move_vehicle(vid, nearest_facility(loc, hubs))

    # ---------- 3. SPAWN NEW VEHICLES ----------
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
                    # OVERSEAS: Only spawn a truck to go to a port/airport (waypoint), NOT directly to destination
                    # Find the nearest port/airport
                    port = nearest_facility(origin, ocean_ports)
                    airport = nearest_facility(origin, airports)
                    port_dist = haversine_distance_meters(origin, port) if port else float('inf')
                    airport_dist = haversine_distance_meters(origin, airport) if airport else float('inf')

                    if port_dist < airport_dist and port_dist < 500000:
                        waypoint = port
                    elif airport_dist < 500000:
                        waypoint = airport
                    else:
                        waypoint = None

                    if waypoint:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                            to_load = box_ids[:VehicleType.SemiTruck.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, waypoint)
                            break
                        except ValueError:
                            pass
                    # If no waypoint found, try to spawn ship/plane directly at origin (if allowed)
                    if num_boxes <= VehicleType.CargoShip.value.capacity:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                            to_load = box_ids[:VehicleType.CargoShip.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    if num_boxes <= VehicleType.Airplane.value.capacity:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Airplane, origin)
                            to_load = box_ids[:VehicleType.Airplane.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    # If everything fails, skip this origin (no truck spawned)
                    continue

                else:
                    # LAND ROUTE
                    if num_boxes >= 20 and num_boxes <= VehicleType.Train.value.capacity:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Train, origin)
                            to_load = box_ids[:VehicleType.Train.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    if num_boxes <= VehicleType.SemiTruck.value.capacity:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                            to_load = box_ids[:VehicleType.SemiTruck.value.capacity]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            continue
