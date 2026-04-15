# -------------------------
# SPAWN LOGIC (FIXED)
# -------------------------
if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):

    undelivered = [
        bid for bid, box in boxes.items()
        if not box["delivered"] and box["vehicle_id"] is None
    ]

    for bid in undelivered[:200]:  # limit scan for performance
        box = boxes[bid]
        origin = box["location"]
        dest = box["destination"]

        # try each box independently (IMPORTANT FIX)
        cargo = [bid]

        try:
            # WATER ROUTE
            if is_water_crossing(origin, dest):
                vid = sim_state.create_vehicle(VehicleType.CargoShip, origin)
                sim_state.load_vehicle(vid, cargo)
                sim_state.move_vehicle(vid, dest)
                continue

            # BULK LAND
            if len(cargo) >= 20:
                vid = sim_state.create_vehicle(VehicleType.Train, origin)
                sim_state.load_vehicle(vid, cargo)
                sim_state.move_vehicle(vid, dest)
                continue

            # DEFAULT TRUCK
            vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
            sim_state.load_vehicle(vid, cargo)
            sim_state.move_vehicle(vid, dest)

        except ValueError:
            continue
