"""Example step function for the LogicTransportationSimulator.

Strategy: Greedy one-vehicle-per-hub delivery using random valid vehicle types.

* **Tick 0** – one randomly chosen spawn-valid vehicle type is created at
  each shipping hub and immediately loaded with all boxes located there.
* **Every tick** – each stationary truck:
    1. Unloads any cargo that has reached its destination (within 50 m).
    2. Loads any unloaded boxes at its current location.
    3. Drives toward the first remaining box's destination.

Trucks with multiple boxes destined for different locations complete one
delivery at a time, then reposition for the next.
"""

from simulator import VehicleType, haversine_distance_meters, sim_state
import random

_PROXIMITY_M = 50.0


def _print_facilities(sim_state):
    """Print configured facilities once for quick scenario inspection."""
    airports = sim_state.get_airports()
    ocean_ports = sim_state.get_ocean_ports()
    hubs = sim_state.get_shipping_hubs()

    def _print_group(title, coords):
        print(f"{title} ({len(coords)}):")
        if not coords:
            print("  - none")
            return
        for idx, (lat, lon) in enumerate(coords, start=1):
            print(f"  {idx:>2}. ({lat:.6f}, {lon:.6f})")

    print("\nFacilities")
    print("==========")
    _print_group("Shipping hubs", hubs)
    _print_group("Airports", airports)
    _print_group("Ocean ports", ocean_ports)
    print("")


def step(sim_state):
    """Called by the simulator each tick.

    Args:
        sim_state: A :class:`~simulator.SimulationState` instance.  State may
                   only be modified through its public command methods.
    """
    tick = sim_state.tick

    # Optionally check for active events
    for event in sim_state.get_active_events():
        print(f"{event['type']} in effect — {event['remaining_ticks']} ticks left")


    # ── Tick 0: spawn one random valid vehicle at each unique hub ───────
    if tick == 0:
        _print_facilities(sim_state)
        boxes = sim_state.get_boxes()
        print(boxes)
        seen_hubs = set()
        vehicle_types = list(VehicleType)
        for box in boxes.values():
            loc = box["location"]
            if loc not in seen_hubs:
                seen_hubs.add(loc)

                # Spawn only vehicle types allowed at this location.
                shuffled_types = vehicle_types[:]
                random.shuffle(shuffled_types)
                for vehicle_type in shuffled_types:
                    try:
                        sim_state.create_vehicle(vehicle_type, loc)
                        break
                    except ValueError:
                        continue

    # ── Every tick: manage each vehicle ──────────────────────────────────
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    print(f"Undelivered box penalty: ${sim_state.undelivered_box_penalty}")

    for vid, vehicle in vehicles.items():
        loc = vehicle["location"]
        config = VehicleType[vehicle["vehicle_type"]].value
        remaining_capacity = config.capacity - len(vehicle["cargo"])
        has_capacity = remaining_capacity > 0

        # Skip vehicles that are still en route.
        if vehicle["destination"] is not None:
            continue

        # 1. Unload boxes whose destination is at (or within 50 m of) here.
        deliverable = [
            bid
            for bid in vehicle["cargo"]
            if haversine_distance_meters(loc, boxes[bid]["destination"]) <= _PROXIMITY_M
        ]
        if deliverable:
            try:
                sim_state.unload_vehicle(vid, deliverable)
                boxes = sim_state.get_boxes()  # refresh after unload
            except ValueError:
                # Some vehicle classes can only load/unload at specific facilities.
                pass

        # 2. Load any unloaded, undelivered boxes at this location.
        loadable = [
            bid
            for bid, box in boxes.items()
            if not box["delivered"]
            and box["vehicle_id"] is None
            and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M
        ]
        if loadable and has_capacity:
            try:
                sim_state.load_vehicle(vid, loadable[:remaining_capacity])
                boxes = sim_state.get_boxes()  # refresh after load
            except ValueError:
                # Ignore invalid facility/location operations for this strategy.
                pass

        # 3. Head toward the first cargo box's destination (if any remain).
        vehicles = sim_state.get_vehicles()  # refresh cargo list
        vehicle = vehicles[vid]
        if vehicle["cargo"]:
            next_dest = boxes[vehicle["cargo"][0]]["destination"]
            sim_state.move_vehicle(vid, next_dest)
