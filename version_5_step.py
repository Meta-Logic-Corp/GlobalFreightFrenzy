from simulator import VehicleType, haversine_distance_meters

DEBUG = True

_UNDELIVERED_PENALTY = 1000.0
_AT_PORT_M = 5000.0
_PROXIMITY_M = 50.0
_WAYPOINT_ARRIVAL_M = 20000.0

VEHICLE_MODE = {
    "SemiTruck": "land",
    "Train":     "land",
    "Airplane":  "air",
    "CargoShip": "ocean",
    "Drone":     "air",
}

# Ocean waypoints — chokepoints + open-ocean pivots.
OCEAN_WAYPOINTS = {
    "panama":       (9.08, -79.68),
    "suez":         (30.0, 32.55),
    "gibraltar":    (35.95, -5.6),
    "malacca":      (2.5, 101.5),
    "good_hope":    (-34.4, 18.5),
    "horn":         (-55.98, -67.27),
    "bab_el":       (12.58, 43.33),
    "n_atlantic_w": (35.0, -60.0),
    "n_atlantic_e": (45.0, -20.0),
    "s_atlantic":   (-20.0, -20.0),
    "n_pacific_w":  (35.0, 150.0),
    "n_pacific_e":  (35.0, -150.0),
    "s_pacific":    (-20.0, -140.0),
    "n_indian":     (10.0, 70.0),
    "s_indian":     (-20.0, 80.0),
    "red_sea":      (20.0, 38.0),
    "med_east":     (34.0, 25.0),
    "caribbean":    (15.0, -75.0),
    "english_ch":   (50.0, -2.0),
    "north_sea":    (56.0, 3.0),
    "south_china":  (15.0, 115.0),
    "coral_sea":    (-15.0, 155.0),
}


def log(*args):
    if DEBUG:
        print(*args)


def km(a, b):
    return haversine_distance_meters(a, b) / 1000.0


def _nearest(loc, points):
    return min(points, key=lambda p: km(loc, p)) if points else None


# ─────────────────────────────────────────────────────────────────────────
# Waypoint chaining — greedy: insert the waypoint that most reduces
# remaining distance to dest, while also being closer to dest than current.
# Repeat until no waypoint helps.
# ─────────────────────────────────────────────────────────────────────────
def chain_waypoints(origin_port, dest_port):
    """Return [origin_port, wp1, wp2, ..., dest_port].

    At each step, from current position, find the waypoint W such that
    dist(current -> W -> dest) < dist(current -> dest), picking the W that
    minimizes total remaining path. Stop when no waypoint helps.
    """
    path = [origin_port]
    current = origin_port
    used = set()

    while True:
        direct = km(current, dest_port)
        best_wp = None
        best_total = direct

        for name, wp in OCEAN_WAYPOINTS.items():
            if name in used:
                continue
            # W must be closer to dest than current is (no going backwards).
            if km(wp, dest_port) >= direct:
                continue
            # Total path via W: current -> W + W -> dest.
            total = km(current, wp) + km(wp, dest_port)
            if total < best_total:
                best_total = total
                best_wp = name

        if best_wp is None:
            break
        used.add(best_wp)
        path.append(OCEAN_WAYPOINTS[best_wp])
        current = OCEAN_WAYPOINTS[best_wp]

    path.append(dest_port)
    return path


# ─────────────────────────────────────────────────────────────────────────
# Route planning per box
# ─────────────────────────────────────────────────────────────────────────
def _leg_cost(vehicle_type, distance_km, n_boxes, terrain_penalty_est=0.0):
    """Cost for one leg: base + per_km + load + estimated terrain penalty.

    terrain_penalty_est is an extra cost added per km for expected terrain
    violations (e.g., truck crossing water). For land legs >1500 km it's
    almost guaranteed that some portion crosses ocean.
    """
    cfg = vehicle_type.value
    n_veh = (n_boxes + cfg.capacity - 1) // cfg.capacity
    return (cfg.base_cost * n_veh
            + cfg.per_km_cost * distance_km * n_veh
            + terrain_penalty_est * n_veh
            + n_boxes * 1)


def _estimated_terrain_penalty(vehicle_type, origin, dest):
    """Rough estimate of terrain violation cost for a vehicle crossing a
    straight line between two coords.

    Land vehicles (trucks, trains) crossing ocean get charged
    terrain_penalty_per_km on the water portion. Without full geography we
    can't know exactly, but for long legs we assume a significant fraction
    crosses water.
    """
    cfg = vehicle_type.value
    # Get terrain penalty rate from the vehicle config (API says this exists).
    rate = getattr(cfg, "terrain_penalty_per_km", 0.0)
    if rate == 0:
        return 0.0

    mode = VEHICLE_MODE[vehicle_type.name]
    d_km = km(origin, dest)
    if mode == "land":
        # Heuristic: legs >1500 km almost certainly cross water.
        # Assume 50% of the excess distance is water.
        if d_km > 1500:
            water_km = (d_km - 1500) * 0.5 + 500  # some water even in shorter
            return rate * water_km
        elif d_km > 500:
            # Moderate legs might cross a bay or gulf; assume 10%
            return rate * d_km * 0.1
        return 0.0
    if mode == "ocean":
        # Ships crossing land — shouldn't happen if we use waypoints properly.
        return 0.0
    return 0.0


def _path_km(path):
    return sum(km(path[i], path[i + 1]) for i in range(len(path) - 1))


def plan_box_route(origin, dest, n_boxes, ocean_ports, airports):
    """Return cheapest route plan for a cluster of n_boxes going origin->dest.

    n_boxes matters because base costs amortize across the cluster. Pass the
    full cluster size, not 1 per box.
    """
    candidates = []

    # ── Option: port -> port with chained waypoints ────────────────────
    if ocean_ports:
        origin_port = _nearest(origin, ocean_ports)
        dest_port   = _nearest(dest,   ocean_ports)
        origin_at_port = km(origin, origin_port) * 1000 <= _AT_PORT_M
        dest_at_port   = km(dest,   dest_port)   * 1000 <= _AT_PORT_M

        ship_path = chain_waypoints(origin_port, dest_port)
        ship_km = _path_km(ship_path)
        ship_cost = _leg_cost(VehicleType.CargoShip, ship_km, n_boxes)

        total_cost = ship_cost
        legs = []

        if not origin_at_port:
            t_km = km(origin, origin_port)
            total_cost += _leg_cost(VehicleType.SemiTruck, t_km, n_boxes)
            legs.append({
                "mode": "land", "vehicle_type": "SemiTruck",
                "from": origin, "to": origin_port, "path": [origin, origin_port],
            })

        legs.append({
            "mode": "ocean", "vehicle_type": "CargoShip",
            "from": origin_port, "to": dest_port, "path": ship_path,
        })

        if not dest_at_port:
            t_km = km(dest_port, dest)
            total_cost += _leg_cost(VehicleType.SemiTruck, t_km, n_boxes)
            legs.append({
                "mode": "land", "vehicle_type": "SemiTruck",
                "from": dest_port, "to": dest, "path": [dest_port, dest],
            })

        candidates.append({
            "legs": legs,
            "cost": total_cost,
            "origin_port": origin_port,
            "dest_port": dest_port,
            "summary": f"port {origin_port} -> {dest_port} "
                       f"({len(ship_path)-2} waypoints, {ship_km:.0f} km sea)",
        })

    # ── Option: truck direct ────────────────────────────────────────────
    d_km = km(origin, dest)
    truck_terrain = _estimated_terrain_penalty(VehicleType.SemiTruck, origin, dest)
    truck_cost = _leg_cost(VehicleType.SemiTruck, d_km, n_boxes, truck_terrain)
    candidates.append({
        "legs": [{
            "mode": "land", "vehicle_type": "SemiTruck",
            "from": origin, "to": dest, "path": [origin, dest],
        }],
        "cost": truck_cost,
        "origin_port": None,
        "dest_port": None,
        "summary": f"truck direct {d_km:.0f} km (terrain est ${truck_terrain:.0f})",
    })

    # ── Option: plane direct (with truck hops if needed) ───────────────
    if airports:
        origin_ap = _nearest(origin, airports)
        dest_ap   = _nearest(dest,   airports)
        air_km = km(origin_ap, dest_ap)
        pre_truck  = km(origin, origin_ap)
        post_truck = km(dest_ap, dest)
        total = (_leg_cost(VehicleType.Airplane, air_km, n_boxes)
                 + _leg_cost(VehicleType.SemiTruck, pre_truck, n_boxes)
                 + _leg_cost(VehicleType.SemiTruck, post_truck, n_boxes))
        legs = []
        if pre_truck * 1000 > _AT_PORT_M:
            legs.append({"mode": "land", "vehicle_type": "SemiTruck",
                         "from": origin, "to": origin_ap,
                         "path": [origin, origin_ap]})
        legs.append({"mode": "air", "vehicle_type": "Airplane",
                     "from": origin_ap, "to": dest_ap,
                     "path": [origin_ap, dest_ap]})
        if post_truck * 1000 > _AT_PORT_M:
            legs.append({"mode": "land", "vehicle_type": "SemiTruck",
                         "from": dest_ap, "to": dest,
                         "path": [dest_ap, dest]})
        candidates.append({
            "legs": legs,
            "cost": total,
            "origin_port": None,
            "dest_port": None,
            "summary": f"plane {origin_ap} -> {dest_ap} ({air_km:.0f} km air)",
        })

    return min(candidates, key=lambda c: c["cost"])


# ─────────────────────────────────────────────────────────────────────────
# Hub class
# ─────────────────────────────────────────────────────────────────────────
class Hub:
    def __init__(self, location):
        self.location = location
        self.boxes_by_dest = {}   # dest -> [box_id, ...]
        self.box_routes = {}      # box_id -> route plan dict

    def add_box(self, box):
        self.boxes_by_dest.setdefault(box["destination"], []).append(box["id"])

    @property
    def total_boxes(self):
        return sum(len(b) for b in self.boxes_by_dest.values())

    @property
    def n_destinations(self):
        return len(self.boxes_by_dest)

    def compute_routes(self, ocean_ports, airports):
        """Compute per-destination route plans. n_boxes = cluster size so that
        base costs amortize across all boxes sharing the leg."""
        self.box_routes.clear()
        for dest, bids in self.boxes_by_dest.items():
            plan = plan_box_route(self.location, dest, len(bids),
                                  ocean_ports, airports)
            for bid in bids:
                self.box_routes[bid] = plan

    def summary(self):
        lines = [f"HUB {self.location}  ({self.total_boxes} boxes, "
                 f"{self.n_destinations} destinations)"]
        for dest, bids in self.boxes_by_dest.items():
            if not bids:
                continue
            plan = self.box_routes.get(bids[0])
            if plan is None:
                lines.append(f"  -> {dest}: {len(bids)} boxes  [no plan]")
                continue
            per_box = plan["cost"] / len(bids) if bids else plan["cost"]
            lines.append(f"  -> {dest}: {len(bids)} boxes  "
                         f"${per_box:.0f}/box (${plan['cost']:.0f} total)  "
                         f"{plan['summary']}")
        return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────────
# VehicleRoute class — state machine for one sim vehicle
# ─────────────────────────────────────────────────────────────────────────
class VehicleRoute:
    IDLE, LOADING, EN_ROUTE, UNLOADING, DONE = (
        "IDLE", "LOADING", "EN_ROUTE", "UNLOADING", "DONE"
    )

    def __init__(self, vid, vehicle_type_name):
        self.vid = vid
        self.vehicle_type_name = vehicle_type_name
        self.waypoints = []
        self.cargo_plan = []
        self.state = self.IDLE
        self.notes = ""

    def assign_route(self, waypoints, cargo_plan=None, notes=""):
        self.waypoints = list(waypoints)
        self.cargo_plan = list(cargo_plan) if cargo_plan else []
        self.notes = notes
        self.state = self.LOADING if self.cargo_plan else self.EN_ROUTE

    def update_state(self, vehicle_snapshot):
        loc = vehicle_snapshot["location"]
        moving = vehicle_snapshot["destination"] is not None

        if self.state == self.DONE:
            return self.state

        if self.state == self.LOADING:
            if not self.cargo_plan or vehicle_snapshot["cargo"]:
                self.state = self.EN_ROUTE if self.waypoints else self.UNLOADING
            return self.state

        if self.state == self.EN_ROUTE:
            if moving:
                return self.state
            while len(self.waypoints) > 1 and \
                    haversine_distance_meters(loc, self.waypoints[0]) <= _WAYPOINT_ARRIVAL_M:
                self.waypoints.pop(0)
            while self.waypoints and \
                    haversine_distance_meters(loc, self.waypoints[0]) <= _PROXIMITY_M:
                self.waypoints.pop(0)
            if not self.waypoints:
                self.state = self.UNLOADING
            return self.state

        if self.state == self.UNLOADING:
            if not vehicle_snapshot["cargo"]:
                self.state = self.DONE
            return self.state

        return self.state

    def next_move_target(self):
        if self.state == self.EN_ROUTE and self.waypoints:
            return self.waypoints[0]
        return None

    def __repr__(self):
        return (f"VehicleRoute(vid={self.vid}, type={self.vehicle_type_name}, "
                f"state={self.state}, wp_left={len(self.waypoints)}, "
                f"cargo={len(self.cargo_plan)}, notes='{self.notes}')")


# ─────────────────────────────────────────────────────────────────────────
# Module state
# ─────────────────────────────────────────────────────────────────────────
_hubs = {}
_vehicle_routes = {}

# Dispatcher bookkeeping.
_claimed_boxes = set()
_active_ship_clusters = {}   # (origin_port, dest_port) -> vid
_active_truck_legs = {}      # (origin, dest) -> vid
_active_plane_legs = {}      # (origin, dest) -> vid


def _rebuild_hubs(boxes):
    _hubs.clear()
    for box in boxes.values():
        if box["delivered"]:
            continue
        # Track a box as "at a hub" only if it's not on a vehicle.
        if box["vehicle_id"] is not None:
            continue
        loc = box["location"]
        if loc not in _hubs:
            _hubs[loc] = Hub(loc)
        _hubs[loc].add_box(box)


# ─────────────────────────────────────────────────────────────────────────
# Dispatcher helpers
# ─────────────────────────────────────────────────────────────────────────
def _find_idle_vehicle(vehicle_type_name, near_location, vehicles,
                       max_km=float("inf")):
    """Find the closest idle (DONE state) vehicle of this type."""
    best_vid = None
    best_route = None
    best_km = max_km
    for vid, route in _vehicle_routes.items():
        if route.vehicle_type_name != vehicle_type_name:
            continue
        if route.state != VehicleRoute.DONE:
            continue
        if vid not in vehicles:
            continue
        d = km(vehicles[vid]["location"], near_location)
        if d < best_km:
            best_km = d
            best_vid = vid
            best_route = route
    return best_vid, best_route


def _dispatch_ocean_clusters(sim_state, clusters, vehicles):
    """One ship per (origin_port, dest_port) cluster."""
    for (origin_port, dest_port), box_ids in clusters.items():
        if (origin_port, dest_port) in _active_ship_clusters:
            continue
        vid, route = _find_idle_vehicle("CargoShip", origin_port, vehicles)
        if vid is None:
            log(f"  [dispatch] no idle ship for {origin_port}->{dest_port}")
            continue
        ship_path = chain_waypoints(origin_port, dest_port)
        ship_loc = vehicles[vid]["location"]
        waypoints = []
        if km(ship_loc, origin_port) * 1000 > _AT_PORT_M:
            waypoints.append(origin_port)
        waypoints.extend(ship_path[1:])
        route.assign_route(
            waypoints,
            cargo_plan=[],   # ship will pick up via transfer at origin_port
            notes=f"ocean cluster {origin_port}->{dest_port} ({len(box_ids)} boxes)"
        )
        _active_ship_clusters[(origin_port, dest_port)] = vid
        log(f"  [dispatch] ship {vid}: {origin_port}->{dest_port}, "
            f"{len(ship_path)-1} legs, {len(box_ids)} boxes")


def _dispatch_hub_trucks(sim_state, vehicles):
    """One truck/plane per (origin, first-leg-destination) group."""
    for hub in _hubs.values():
        origin = hub.location
        first_leg_groups = {}
        for bid, plan in hub.box_routes.items():
            if bid in _claimed_boxes:
                continue
            if not plan["legs"]:
                continue
            first_leg = plan["legs"][0]
            key = (first_leg["to"], first_leg["vehicle_type"])
            first_leg_groups.setdefault(key, []).append(bid)

        for (target, vtype_name), bids in first_leg_groups.items():
            leg_key = (origin, target)
            if leg_key in _active_truck_legs or leg_key in _active_plane_legs:
                continue

            if vtype_name == "SemiTruck":
                vid, route = _find_idle_vehicle("SemiTruck", origin, vehicles)
                if vid is None:
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                        route = VehicleRoute(vid, "SemiTruck")
                        _vehicle_routes[vid] = route
                        log(f"  [dispatch] spawned truck {vid} at hub {origin}")
                    except ValueError as e:
                        log(f"  [dispatch] can't spawn truck at {origin}: {e}")
                        continue
                route.assign_route(
                    [target], cargo_plan=bids,
                    notes=f"truck {origin}->{target} ({len(bids)} boxes)"
                )
                _active_truck_legs[leg_key] = vid
                _claimed_boxes.update(bids)
                log(f"  [dispatch] truck {vid}: {origin}->{target}, {len(bids)} boxes")

            elif vtype_name == "Airplane":
                vid, route = _find_idle_vehicle("Airplane", origin, vehicles)
                if vid is None:
                    log(f"  [dispatch] no idle plane near {origin}")
                    continue
                route.assign_route(
                    [target], cargo_plan=bids,
                    notes=f"plane {origin}->{target} ({len(bids)} boxes)"
                )
                _active_plane_legs[leg_key] = vid
                _claimed_boxes.update(bids)
                log(f"  [dispatch] plane {vid}: {origin}->{target}, {len(bids)} boxes")


def _execute_vehicle_actions(sim_state, vehicles, boxes):
    """Drive each VehicleRoute's state machine by calling sim methods."""
    for vid, route in list(_vehicle_routes.items()):
        if vid not in vehicles:
            continue
        vehicle = vehicles[vid]
        loc = vehicle["location"]
        route.update_state(vehicle)

        # LOADING: pick up planned cargo if at location.
        if route.state == VehicleRoute.LOADING and route.cargo_plan:
            cfg = VehicleType[route.vehicle_type_name].value
            cap_left = cfg.capacity - len(vehicle["cargo"])
            loadable = []
            for bid in route.cargo_plan:
                if bid not in boxes: continue
                b = boxes[bid]
                if b["delivered"] or b["vehicle_id"] is not None: continue
                if haversine_distance_meters(loc, b["location"]) > _PROXIMITY_M: continue
                loadable.append(bid)
                if len(loadable) >= cap_left: break
            if loadable:
                try:
                    sim_state.load_vehicle(vid, loadable)
                except ValueError as e:
                    log(f"  [exec] load {vid} failed: {e}")
            route.state = VehicleRoute.EN_ROUTE if route.waypoints else VehicleRoute.UNLOADING

        # EN_ROUTE: move toward next waypoint.
        if route.state == VehicleRoute.EN_ROUTE:
            target = route.next_move_target()
            if target is not None and vehicle["destination"] is None:
                try:
                    sim_state.move_vehicle(vid, target)
                except (ValueError, KeyError) as e:
                    log(f"  [exec] move {vid} failed: {e}")

        # UNLOADING: drop everything. Transfer points (ports) naturally hand
        # cargo to the next vehicle via the dispatcher in future ticks.
        if route.state == VehicleRoute.UNLOADING and vehicle["cargo"]:
            try:
                sim_state.unload_vehicle(vid, list(vehicle["cargo"]))
            except ValueError as e:
                log(f"  [exec] unload {vid} failed: {e}")


# ─────────────────────────────────────────────────────────────────────────
# Main step
# ─────────────────────────────────────────────────────────────────────────
def step(sim_state):
    tick = sim_state.tick

    try:
        ocean_ports = list(sim_state.get_ocean_ports()) or None
    except Exception:
        ocean_ports = None
    try:
        airports = list(sim_state.get_airports()) or None
    except Exception:
        airports = None
    try:
        shipping_hubs = list(sim_state.get_shipping_hubs()) or None
    except Exception:
        shipping_hubs = None

    boxes = sim_state.get_boxes()

    # Tick 0: position vehicles (no dispatching yet).
    if tick == 0:
        log("=== POSITIONING VEHICLES ===")
        log(f"  ocean ports: {len(ocean_ports) if ocean_ports else 0}")
        log(f"  airports:    {len(airports) if airports else 0}")
        log(f"  hubs:        {len(shipping_hubs) if shipping_hubs else 0}")

        # Ships at waypoints (via nearest port).
        if ocean_ports:
            for name, coord in OCEAN_WAYPOINTS.items():
                nearest_port = _nearest(coord, ocean_ports)
                try:
                    vid = sim_state.create_vehicle(VehicleType.CargoShip, nearest_port)
                    route = VehicleRoute(vid, "CargoShip")
                    route.assign_route([coord], notes=f"position at {name}")
                    _vehicle_routes[vid] = route
                    sim_state.move_vehicle(vid, coord)
                    log(f"  {vid} -> {name}")
                except (ValueError, KeyError) as e:
                    log(f"  waypoint-ship for {name} failed: {e}")
        else:
            for name, coord in OCEAN_WAYPOINTS.items():
                try:
                    vid = sim_state.create_vehicle(VehicleType.CargoShip, coord)
                    route = VehicleRoute(vid, "CargoShip")
                    route.state = VehicleRoute.DONE
                    _vehicle_routes[vid] = route
                    log(f"  waypoint-ship {vid} at {name}")
                except ValueError as e:
                    log(f"  waypoint-ship at {name} failed: {e}")

        # Extra ship at each port.
        if ocean_ports:
            for port in ocean_ports:
                try:
                    vid = sim_state.create_vehicle(VehicleType.CargoShip, port)
                    route = VehicleRoute(vid, "CargoShip")
                    route.state = VehicleRoute.DONE
                    _vehicle_routes[vid] = route
                    log(f"  port-ship {vid} at {port}")
                except ValueError as e:
                    log(f"  port-ship at {port} failed: {e}")

        # Drone at each airport.
        if airports:
            for ap in airports:
                try:
                    vid = sim_state.create_vehicle(VehicleType.Drone, ap)
                    route = VehicleRoute(vid, "Drone")
                    route.state = VehicleRoute.DONE
                    _vehicle_routes[vid] = route
                    log(f"  drone {vid} at {ap}")
                except ValueError as e:
                    log(f"  drone at {ap} failed: {e}")

        # Truck at each hub.
        if shipping_hubs:
            for hub in shipping_hubs:
                try:
                    vid = sim_state.create_vehicle(VehicleType.SemiTruck, hub)
                    route = VehicleRoute(vid, "SemiTruck")
                    route.state = VehicleRoute.DONE
                    _vehicle_routes[vid] = route
                    log(f"  truck {vid} at {hub}")
                except ValueError as e:
                    log(f"  truck at {hub} failed: {e}")

    # Every tick: rebuild hubs, plan box routes.
    _rebuild_hubs(boxes)
    for hub in _hubs.values():
        hub.compute_routes(ocean_ports, airports)

    # Advance vehicle state machines (sync with sim snapshot).
    vehicles = sim_state.get_vehicles()
    for vid, route in _vehicle_routes.items():
        if vid not in vehicles:
            continue
        route.update_state(vehicles[vid])

    # ── DISPATCH ─────────────────────────────────────────────────────────
    # Build current ocean clusters from box routes.
    clusters = {}
    for hub in _hubs.values():
        for bid, plan in hub.box_routes.items():
            if bid in _claimed_boxes:
                continue
            op = plan.get("origin_port")
            dp = plan.get("dest_port")
            if op is None or dp is None:
                continue
            clusters.setdefault((op, dp), []).append(bid)

    _dispatch_ocean_clusters(sim_state, clusters, vehicles)
    _dispatch_hub_trucks(sim_state, vehicles)

    # ── EXECUTE ──────────────────────────────────────────────────────────
    _execute_vehicle_actions(sim_state, vehicles, boxes)

    # ── LOG ──────────────────────────────────────────────────────────────
    if tick == 0 or tick % 10 == 0:
        log(f"\n=== TICK {tick} — {len(_hubs)} hubs, {len(clusters)} ocean clusters ===")
        for hub in _hubs.values():
            log(hub.summary())
        if clusters:
            ranked = sorted(clusters.items(), key=lambda x: -len(x[1]))
            log(f"\n--- OCEAN CLUSTERS ---")
            for (op, dp), bids in ranked[:10]:
                log(f"  {op} -> {dp}: {len(bids)} boxes")
        active = [r for r in _vehicle_routes.values() if r.state != VehicleRoute.DONE]
        log(f"\n--- {len(active)} active vehicles ---")
        for r in active[:15]:
            log(f"  {r}")
        log(f"  cost so far: ${sim_state.total_cost:.0f} "
            f"(terrain: ${sim_state.terrain_penalty:.0f})")

