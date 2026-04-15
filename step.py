_PROXIMITY_M = 50.0
_LOAD_UNLOAD_FACILITY_RANGE_M = 5000.0

def is_water_crossing(from_loc, to_loc) -> bool:
    """Improved water crossing detection"""
    dist = haversine_distance_meters(from_loc, to_loc)
    if dist > 500_000:
        return True
    
    lat1, lon1 = from_loc
    lat2, lon2 = to_loc
    
    # Atlantic crossing
    if ((lon1 < -50 and lon2 > -20) or (lon1 > -20 and lon2 < -50)) and abs(lat1 - lat2) < 60:
        return True
    # Pacific crossing
    if abs(lon1 - lon2) > 180:
        return True
    return False


def get_nearest_location(current: tuple, candidates: tuple) -> tuple | None:
    """Find nearest location from a list of hubs/airports/ports"""
    if not candidates:
        return None
    nearest = min(candidates, key=lambda p: haversine_distance_meters(current, p))
    return nearest


def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    # Get facility locations (best practice)
    hubs = sim_state.get_shipping_hubs()
    airports = sim_state.get_airports() or hubs  # fallback as per API
    ocean_ports = sim_state.get_ocean_ports()

    all_transfer_points = set(hubs) | set(airports) | set(ocean_ports)

    # ====================== UNLOAD DELIVERED BOXES ======================
    for vid, v in list(vehicles.items()):
        if not v["cargo"]:
            continue
            
        to_unload = [
            bid for bid in v["cargo"]
            if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M
        ]
        if to_unload:
            try:
                sim_state.unload_vehicle(vid, to_unload)
            except ValueError:
                pass  # rare race condition

    # Refresh boxes after unloading
    boxes = sim_state.get_boxes()

    # ====================== VEHICLE MANAGEMENT ======================
    for vid, v in list(vehicles.items()):
        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        cargo = v["cargo"]

        # --- Load available boxes ---
        if len(cargo) < vtype.value.capacity:
            loadable = [
                bid for bid, b in boxes.items()
                if b["vehicle_id"] is None and not b["delivered"]
                and haversine_distance_meters(loc, b["location"]) <= _PROXIMITY_M
            ]
            if loadable:
                can_load = vtype.value.capacity - len(cargo)
                to_load = loadable[:can_load]
                try:
                    sim_state.load_vehicle(vid, to_load)
                except ValueError:
                    pass
                boxes = sim_state.get_boxes()

        # --- Route cargo if loaded ---
        if cargo:
            first_box = boxes[cargo[0]]
            dest = first_box["destination"]
            num_boxes = len(cargo)

            # Try direct move first (simulator will penalize invalid terrain)
            try:
                sim_state.move_vehicle(vid, dest)
                continue
            except (ValueError, KeyError):
                pass  # terrain violation or other issue → need transload

            # Need to transload
            nearest_hub = get_nearest_location(loc, tuple(all_transfer_points))
            if not nearest_hub:
                continue

            # Unload cargo for transfer
            try:
                sim_state.unload_vehicle(vid, list(cargo))
                boxes = sim_state.get_boxes()
            except ValueError:
                continue

            transferred = False

            # Water crossing → prefer CargoShip, fallback to Airplane
            if is_water_crossing(loc, dest):
                if num_boxes >= 8 and ocean_ports:
                    port = get_nearest_location(nearest_hub, ocean_ports)
                    if port:
                        try:
                            new_vid = sim_state.create_vehicle(VehicleType.CargoShip, port)
                            sim_state.load_vehicle(new_vid, cargo[:1000])
                            sim_state.move_vehicle(new_vid, dest)
                            transferred = True
                        except ValueError:
                            pass

                if not transferred and num_boxes <= 90:
                    airport = get_nearest_location(nearest_hub, airports)
                    if airport:
                        try:
                            new_vid = sim_state.create_vehicle(VehicleType.Airplane, airport)
                            sim_state.load_vehicle(new_vid, cargo[:90])
                            sim_state.move_vehicle(new_vid, dest)
                            transferred = True
                        except ValueError:
                            pass

            # Land bulk → Train
            elif num_boxes >= 30 and hubs:
                hub = get_nearest_location(nearest_hub, hubs)
                if hub:
                    try:
                        new_vid = sim_state.create_vehicle(VehicleType.Train, hub)
                        sim_state.load_vehicle(new_vid, cargo[:500])
                        sim_state.move_vehicle(new_vid, dest)
                        transferred = True
                    except ValueError:
                        pass

            # Fallback: original vehicle or truck
            if not transferred:
                try:
                    sim_state.move_vehicle(vid, dest)
                except ValueError:
                    # At least move toward a hub
                    if nearest_hub:
                        sim_state.move_vehicle(vid, nearest_hub)

        # --- Empty vehicle: go find work ---
        else:
            nearest_box_loc = None
            min_dist = float('inf')
            for b in boxes.values():
                if b["vehicle_id"] is None and not b["delivered"]:
                    d = haversine_distance_meters(loc, b["location"])
                    if d < min_dist:
                        min_dist = d
                        nearest_box_loc = b["location"]
            if nearest_box_loc:
                try:
                    sim_state.move_vehicle(vid, nearest_box_loc)
                except ValueError:
                    pass

    # ====================== SPAWN NEW VEHICLES ======================
    if tick == 0 or (len(vehicles) < 30 and tick % 35 == 0):
        undelivered = [bid for bid, b in boxes.items() 
                      if not b["delivered"] and b["vehicle_id"] is None]
        
        if not undelivered:
            return

        # Group by origin
        origins = defaultdict(list)
        for bid in undelivered:
            origins[boxes[bid]["location"]].append(bid)

        for origin, box_ids in origins.items():
            if not box_ids:
                continue
                
            sample_dest = boxes[box_ids[0]]["destination"]
            needs_water = is_water_crossing(origin, sample_dest)
            n_boxes = len(box_ids)

            spawned = False

            # 1. Water crossing priority
            if needs_water:
                if n_boxes >= 15 and ocean_ports:
                    port = get_nearest_location(origin, ocean_ports)
                    if port:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.CargoShip, port)
                            sim_state.load_vehicle(vid, box_ids[:1000])
                            sim_state.move_vehicle(vid, sample_dest)
                            spawned = True
                        except ValueError:
                            pass

                if not spawned and n_boxes <= 80:
                    airport = get_nearest_location(origin, airports)
                    if airport:
                        try:
                            vid = sim_state.create_vehicle(VehicleType.Airplane, airport)
                            sim_state.load_vehicle(vid, box_ids[:80])
                            sim_state.move_vehicle(vid, sample_dest)
                            spawned = True
                        except ValueError:
                            pass

            # 2. Bulk land - Train
            if not spawned and n_boxes >= 30 and hubs:
                hub = get_nearest_location(origin, hubs)
                if hub:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.Train, hub)
                        sim_state.load_vehicle(vid, box_ids[:500])
                        sim_state.move_vehicle(vid, sample_dest)
                        spawned = True
                    except ValueError:
                        pass

            # 3. Default - SemiTruck
            if not spawned and n_boxes <= 50 and hubs:
                hub = get_nearest_location(origin, hubs)
                if hub:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, hub)
                        sim_state.load_vehicle(vid, box_ids[:50])
                        sim_state.move_vehicle(vid, sample_dest)
                        spawned = True
                    except ValueError:
                        pass

            # 4. Drone for very small/short range (optional optimization)
            if not spawned and n_boxes <= 5:
                airport = get_nearest_location(origin, airports)
                if airport and haversine_distance_meters(origin, sample_dest) <= 18000:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.Drone, airport)
                        sim_state.load_vehicle(vid, box_ids[:5])
                        sim_state.move_vehicle(vid, sample_dest)
                    except ValueError:
                        pass
