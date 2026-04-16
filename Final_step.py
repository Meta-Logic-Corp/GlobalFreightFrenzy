from simulator import VehicleType, haversine_distance_meters
import math

_PROXIMITY_M = 50.0
_GATEWAY_THRESHOLD_M = 5000.0  # 5km rule for airport/port access

NATIONAL_RAIL_GROUPS = {
    "NA_RAIL": [
        (47.4502, -122.3088), (33.9425, -118.4081), (32.8481, -97.0403), 
        (41.9742, -87.9073), (43.651, -79.347), (40.6413, -73.7781)
        ],
    "AF_EU_RAIL": [
        (51.47, -0.4543), (50.0379, 8.5622), (-1.3192, 36.9278), (-26.1337, 28.242)
    ],
    "ME_AS_RAIL": [
        (25.2532, 55.3657), (19.0896, 72.8656)
    ]
}

# Global Flight Groups: Used to optimize vehicle counts for shared regions
GLOBAL_GROUPS = {
    "NA": [(47.4502, -122.3088), (33.9425, -118.4081), (32.8481, -97.0403), (19.4326, -99.1332), (25.7959, -80.287), (41.9742, -87.9073), (43.651, -79.347), (40.6413, -73.7781)],
    "EU": [(51.47, -0.4543), (50.0379, 8.5622)],
    "ASIA": [(35.6762, 139.6503), (25.2532, 55.3657), (19.0896, 72.8656), (1.3644, 103.9915)],
    "SOUTH": [(-23.4356, -46.4731), (-1.3192, 36.9278), (-26.1337, 28.242)],
    "SYDNEY": [(-33.9399, 151.1753)]
}

def get_group(gps_tuple, group_dict):
    for group_name, coords in group_dict.items():
        if gps_tuple in coords:
            return group_name
    return None

def step(sim_state):
    tick = sim_state.tick
    
    if tick == 0:
        boxes = sim_state.get_boxes()
        airports = sim_state.get_airports()
        ports = sim_state.get_ocean_ports()
        
        regional_demand = {}
        global_demand = {}

        for box in boxes.values():
            o, d = box["location"], box["destination"] #
            o_rail = get_group(o, NATIONAL_RAIL_GROUPS)
            d_rail = get_group(d, NATIONAL_RAIL_GROUPS)
            d_global = get_group(d, GLOBAL_GROUPS)

            if o_rail and o_rail == d_rail:
                # Intra-regional demand for Trains
                if o not in regional_demand: regional_demand[o] = {}
                regional_demand[o][d_rail] = regional_demand[o].get(d_rail, 0) + 1
            else:
                # Global/Intercontinental demand for Planes/Ships
                if o not in global_demand: global_demand[o] = {}
                global_demand[o][d_global] = global_demand[o].get(d_global, 0) + 1
            
        for origin, target_regions in {**regional_demand, **global_demand}.items():

            for group_name, count in target_regions.items():
                spawned = False

                # CASE A: Same Rail Group
                if group_name in NATIONAL_RAIL_GROUPS:
                    try:
                        for _ in range(math.ceil(count / 500)):
                            sim_state.create_vehicle(VehicleType.Train, origin)
                        spawned = True
                    except ValueError: pass

                # CASE B: Global
                else:
                    try:
                        for _ in range(math.ceil(count / 100)):
                            sim_state.create_vehicle(VehicleType.Airplane, origin)
                        spawned = True
                    except ValueError: pass
                    
                    if not spawned:
                        try:
                            for _ in range(math.ceil(count / 1000)):
                                sim_state.create_vehicle(VehicleType.CargoShip, origin)
                            spawned = True
                        except ValueError: pass


# ── Every tick: manage each vehicle ──────────────────────────────────
    # CRITICAL: Always refresh snapshots at the start of every tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Track boxes being loaded this tick to prevent double-loading ValueErrors
    currently_loading = set()
    
    # Get facility lists for Airplane/Ship validation
    airports = sim_state.get_airports()
    ports = sim_state.get_ocean_ports()

    for vid, vehicle in vehicles.items():
        # Skip vehicles that are still moving toward a destination
        if vehicle["destination"] is not None:
            continue

        loc = vehicle["location"]
        v_type_str = vehicle["vehicle_type"]
        
        # 1. UNLOAD logic: Check if we are at a box's destination hub
        deliverable = [
            bid for bid in vehicle["cargo"]
            if haversine_distance_meters(loc, boxes[bid]["destination"]) <= _PROXIMITY_M
        ]
        
        if deliverable:
            try:
                # API REQ: Airplanes/Drones must be near an airport to unload
                if v_type_str in ["Airplane", "Drone"]:
                    if any(haversine_distance_meters(loc, a["location"] if isinstance(a, dict) else a) <= _GATEWAY_THRESHOLD_M for a in airports):
                        sim_state.unload_vehicle(vid, deliverable)
                # API REQ: CargoShips must be near a port to unload
                elif v_type_str == "CargoShip":
                    if any(haversine_distance_meters(loc, p["location"] if isinstance(p, dict) else p) <= _GATEWAY_THRESHOLD_M for p in ports):
                        sim_state.unload_vehicle(vid, deliverable)
                else:
                    sim_state.unload_vehicle(vid, deliverable)
            except ValueError:
                pass 
            continue # End tick for this vehicle after unloading

        # 2. LOAD logic: Pick up any undelivered boxes at this current hub
        caps = {"SemiTruck": 50, "Train": 500, "Airplane": 100, "CargoShip": 1000, "Drone": 5}
        max_cap = caps.get(v_type_str, 50)
        space_left = max_cap - len(vehicle["cargo"])
        
        if space_left > 0:
            loadable_here = [
                bid for bid, box in boxes.items()
                if not box["delivered"] and box["vehicle_id"] is None
                and bid not in currently_loading
                and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M
            ]
            
            if loadable_here:
                try:
                    # Validate facility access before attempting to load
                    can_load = True
                    if v_type_str in ["Airplane", "Drone"]:
                        can_load = any(haversine_distance_meters(loc, a["location"] if isinstance(a, dict) else a) <= _GATEWAY_THRESHOLD_M for a in airports)
                    elif v_type_str == "CargoShip":
                        can_load = any(haversine_distance_meters(loc, p["location"] if isinstance(p, dict) else p) <= _GATEWAY_THRESHOLD_M for p in ports)
                    
                    if can_load:
                        to_load = loadable_here[:space_left]
                        sim_state.load_vehicle(vid, to_load)
                        for bid in to_load:
                            currently_loading.add(bid)
                        continue # Wait until next tick to move after loading
                except ValueError:
                    pass
            
        # 3. MOVE logic: Decide where to go next
        if vehicle["cargo"]:
            # Destination-based movement: head toward the first box's target
            target = boxes[vehicle["cargo"][0]]["destination"]
            sim_state.move_vehicle(vid, target)
        else:
            # Empty vehicle relocation: Find the nearest undelivered box
            waiting = [b for b in boxes.values() if not b["delivered"] and b["vehicle_id"] is None]
            if waiting:
                # Logic optimization: Airplanes only "chase" boxes located at airports
                if v_type_str in ["Airplane", "Drone"]:
                    boxes_at_airports = [
                        b for b in waiting 
                        if any(haversine_distance_meters(b["location"], a["location"] if isinstance(a, dict) else a) <= _GATEWAY_THRESHOLD_M for a in airports)
                    ]
                    if boxes_at_airports:
                        nearest_box = min(boxes_at_airports, key=lambda b: haversine_distance_meters(loc, b["location"]))
                        sim_state.move_vehicle(vid, nearest_box["location"])
                else:
                    nearest_box = min(waiting, key=lambda b: haversine_distance_meters(loc, b["location"]))
                    sim_state.move_vehicle(vid, nearest_box["location"])