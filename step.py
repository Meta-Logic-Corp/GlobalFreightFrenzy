from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

CAPACITY = {
    VehicleType.SemiTruck: 50,
    VehicleType.Train: 500,
    VehicleType.Airplane: 100,
    VehicleType.CargoShip: 1000,
    VehicleType.Drone: 5,
}

def is_overseas(origin, dest):
    """Detect if route requires water crossing (overseas)."""
    dist = haversine_distance_meters(origin, dest)
    if dist > 3000000:  # 3000 km
        return True
    o_lat, o_lon = origin
    d_lat, d_lon = dest
    # Atlantic crossing
    if (o_lon < -60 and d_lon > -10) or (o_lon > -10 and d_lon < -60):
        return abs(o_lat - d_lat) < 50
    # Pacific crossing
    if (o_lon < -120 and d_lon > 120) or (o_lon > 120 and d_lon < -120):
        return True
    # Middle East ↔ North America
    if (40 < o_lon < 80 and -130 < d_lon < -60) or (40 < d_lon < 80 and -130 < o_lon < -60):
        return True
    # South Asia ↔ Europe
    if (70 < o_lon < 90 and -10 < d_lon < 40) or (70 < d_lon < 90 and -10 < o_lon < 40):
        return True
    return False

def nearest_facility(loc, facilities):
    """Return closest facility from a list of coordinates."""
    if not facilities:
        return None
    return min(facilities, key=lambda f: haversine_distance_meters(loc, f))

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    # Get actual facilities from the simulator
    hubs = sim_state.get_shipping_hubs()
    airports = sim_state.get_airports()
    ocean_ports = sim_state.get_ocean_ports()

    # 1. UNLOAD boxes at destination
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [
                bid for bid in v["cargo"]
                if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M
            ]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()  # refresh

    # 2. MANAGE existing vehicles (move, transfer)
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue

        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        capacity_left = CAPACITY[vtype] - len(v["cargo"])

        # Load boxes at current location if possible
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

            # Check if at a transfer facility
            at_airport = any(haversine_distance_meters(loc, ap) <= 5000 for ap in airports)
            at_port = any(haversine_distance_meters(loc, op) <= 5000 for op in ocean_ports)

            # Land vehicle that needs to go overseas
            if vtype in [VehicleType.SemiTruck, VehicleType.Train] and is_overseas(loc, dest):
                if at_airport or at_port:
                    # Transfer to ship or plane
                    cargo_copy = list(v["cargo"])
                    dest_copy = dest
                    sim_state.unload_vehicle(vid, cargo_copy)
                    boxes = sim_state.get_boxes()
                    # Prefer ship if at port and cargo fits
                    if at_port and len(cargo_copy) <= CAPACITY[VehicleType.CargoShip]:
                        try:
                            new_vid = sim_state.create_vehicle(VehicleType.CargoShip, loc)
                            sim_state.load_vehicle(new_vid, cargo_copy)
                            sim_state.move_vehicle(new_vid, dest_copy)
                        except ValueError:
                            pass
                    elif at_airport and len(cargo_copy) <= CAPACITY[VehicleType.Airplane]:
                        try:
                            new_vid = sim_state.create_vehicle(VehicleType.Airplane, loc)
                            sim_state.load_vehicle(new_vid, cargo_copy)
                            sim_state.move_vehicle(new_vid, dest_copy)
                        except ValueError:
                            pass
                else:
                    # Go to nearest airport or port
                    target = nearest_facility(loc, airports) or nearest_facility(loc, ocean_ports)
                    if target:
                        sim_state.move_vehicle(vid, target)
                    else:
                        sim_state.move_vehicle(vid, dest)

            # Water/air vehicle that has reached land (destination no longer overseas)
            elif vtype in [VehicleType.CargoShip, VehicleType.Airplane] and not is_overseas(loc, dest):
                cargo_copy = list(v["cargo"])
                dest_copy = dest
                sim_state.unload_vehicle(vid, cargo_copy)
                boxes = sim_state.get_boxes()
                # Transfer to train or truck
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

            # Normal land or short‑distance movement
            else:
                sim_state.move_vehicle(vid, dest)

        elif not v["cargo"]:
            # Empty vehicle: go to nearest undelivered box or hub
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

    # 3. SPAWN new vehicles for undelivered boxes (only at tick 0 or occasionally)
    if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        if undelivered:
            # Group by origin
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)

            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                needs_overseas = any(is_overseas(origin, boxes[bid]["destination"]) for bid in box_ids[:5])

                if needs_overseas:
                    # Spawn a truck that will go to the nearest airport or port
                    nearest_airport = nearest_facility(origin, airports)
                    nearest_port = nearest_facility(origin, ocean_ports)
                    # Choose the closer facility within 500 km
                    airport_dist = haversine_distance_meters(origin, nearest_airport) if nearest_airport else float('inf')
                    port_dist = haversine_distance_meters(origin, nearest_port) if nearest_port else float('inf')
                    if port_dist < airport_dist and port_dist < 500000:
                        transfer_point = nearest_port
                    elif airport_dist < 500000:
                        transfer_point = nearest_airport
                    else:
                        transfer_point = None

                    if transfer_point:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                            to_load = box_ids[:CAPACITY[VehicleType.SemiTruck]]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, transfer_point)
                            break
                        except ValueError:
                            pass
                    # Fallback: try to spawn ship/plane directly at origin (if allowed)
                    if num_boxes <= CAPACITY[VehicleType.CargoShip]:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                            to_load = box_ids[:CAPACITY[VehicleType.CargoShip]]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    if num_boxes <= CAPACITY[VehicleType.Airplane]:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Airplane, origin)
                            to_load = box_ids[:CAPACITY[VehicleType.Airplane]]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass

                else:  # Land route
                    # Prefer train for bulk/long distance
                    if num_boxes >= 20 and num_boxes <= CAPACITY[VehicleType.Train]:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Train, origin)
                            to_load = box_ids[:CAPACITY[VehicleType.Train]]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            pass
                    # Otherwise use truck
                    if num_boxes <= CAPACITY[VehicleType.SemiTruck]:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                            to_load = box_ids[:CAPACITY[VehicleType.SemiTruck]]
                            sim_state.load_vehicle(vid, to_load)
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                            break
                        except ValueError:
                            continue
