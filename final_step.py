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

# Fixed hub airports. All cross-region cargo from a given region funnels to ONE
# departure airport, and lands at ONE arrival airport. This maximises batch size
# and minimises the number of planes dispatched.
#
# ROUTE_HUBS[(src_region, dst_region)] = (departure_airport, arrival_airport)
JFK       = (40.6413,  -73.7781)
LAX       = (33.9425, -118.4081)
HEATHROW  = (51.4700,   -0.4543)
HANEDA    = (35.5494,  139.7798)
SYDNEY_AP = (-33.9461, 151.1772)
GUARULHOS = (-23.4356, -46.4731)

ROUTE_HUBS = {
    ("AMERICAS",       "EURASIA_AFRICA"): (JFK,       HEATHROW),
    ("EURASIA_AFRICA", "AMERICAS"):       (HEATHROW,  JFK),
    ("AMERICAS",       "EAST_ASIA"):      (LAX,       HANEDA),
    ("EAST_ASIA",      "AMERICAS"):       (HANEDA,    JFK),
    ("EURASIA_AFRICA", "EAST_ASIA"):      (HEATHROW,  HANEDA),
    ("EAST_ASIA",      "EURASIA_AFRICA"): (HANEDA,    HEATHROW),
    ("AMERICAS",       "OCEANIA"):        (LAX,       SYDNEY_AP),
    ("OCEANIA",        "AMERICAS"):       (SYDNEY_AP, JFK),
    ("EAST_ASIA",      "OCEANIA"):        (HANEDA,    SYDNEY_AP),
    ("OCEANIA",        "EAST_ASIA"):      (SYDNEY_AP, HANEDA),
    ("EURASIA_AFRICA", "OCEANIA"):        (HEATHROW,  SYDNEY_AP),
    ("OCEANIA",        "EURASIA_AFRICA"): (SYDNEY_AP, HEATHROW),
    ("AMERICAS",       "AMERICAS"):       (JFK,       GUARULHOS),  # north↔south within Americas
    ("OCEANIA",        "OCEANIA"):        (SYDNEY_AP, SYDNEY_AP),
}

LOAD_R          = 50.0
ARRIVE_R        = 100.0
INFRA_R         = 5_000.0
ASSIGN_INTERVAL = 10
# Flush staging after this many ticks with no new arrivals (greedy: wait for more cargo)
IDLE_FLUSH_TICKS = 40
PLANE_CAPACITY   = 100
DRONE_CAPACITY   = 5
TRAIN_MIN_BOXES  = 30
DRONE_MAX_KM     = 500.0  # drone last-mile up to 500km from arrival airport

# Land waypoints
FLORIDA_GW  = (30.33,   -81.65)
UPPER_CHINA = (43.8627,  118.754)
CENT_AFRICA = (4.371,    33.366)
EGYPT_GW    = (29.06,    40.00)
SAO_GW1     = (15.861,  -87.80)
SAO_GW2     = (30.50,  -113.50)

itineraries: dict = {}
claimed:     set  = set()
# staging[(src_airport, dst_region)] = {"box_ids": [], "last_tick": int}
staging:     dict = {}


def step(sim_state):
    """Main entry point called every tick."""
    _run_vehicles(sim_state)
    if sim_state.tick % ASSIGN_INTERVAL == 0:
        _assign(sim_state)
        _flush_staging(sim_state)


def _run_vehicles(sim_state):
    """Advance each idle vehicle that has an active itinerary."""
    vehicles = sim_state.get_vehicles()
    boxes    = sim_state.get_boxes()

    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue
        itin = itineraries.get(vid)
        if itin is None:
            continue

        loc = v["location"]

        if itin["state"] == "TO_PICKUP":
            if haversine_distance_meters(loc, itin["pickup_loc"]) <= LOAD_R:
                loadable = [
                    b for b in itin["box_ids"]
                    if b in boxes and not boxes[b]["delivered"] and boxes[b]["vehicle_id"] is None
                ]
                if loadable:
                    try:
                        sim_state.load_vehicle(vid, loadable)
                        itin["box_ids"] = loadable
                    except Exception:
                        pass

                fresh = sim_state.get_boxes()
                actually_loaded = [b for b in itin["box_ids"] if b in fresh and fresh[b]["vehicle_id"] == vid]
                if not actually_loaded and not loadable:
                    for b in itin["box_ids"]:
                        claimed.discard(b)
                    del itineraries[vid]
                    continue

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
                for b in itin["box_ids"]:
                    if b in boxes and not boxes[b]["delivered"]:
                        claimed.discard(b)
                del itineraries[vid]
            else:
                wp = itin["waypoints"][0]
                if haversine_distance_meters(loc, wp) <= ARRIVE_R:
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
    Greedy hub-and-spoke routing:

    Cross-region:
      All boxes from a region funnel to one fixed departure airport via truck/train.
      They stage there until a plane is full (100) or idle for IDLE_FLUSH_TICKS.
      Plane flies to the fixed arrival airport for that route pair.
      Last-mile from arrival: drone (≤5 boxes, ≤500km), train (≥30), truck otherwise.

    Same-region:
      Train (≥30 boxes) or truck, with idle vehicle reuse prioritised.
    """
    origin  = cluster["origin"]
    dest    = cluster["dest"]
    box_ids = cluster["box_ids"]
    n       = len(box_ids)
    if n == 0:
        return False

    src_region = _region(origin)
    dst_region = _region(dest)
    at_airport = _near_any(origin, AIRPORTS)

    # ── Boxes at an airport, cross-region: stage for batch flight ────────────
    if at_airport and src_region != dst_region:
        route = ROUTE_HUBS.get((src_region, dst_region))
        if not route:
            return False
        departure_ap, _ = route

        # Only accept boxes physically present at the correct departure hub airport
        if not _is_near(origin, departure_ap):
            # Boxes are at wrong airport — treat as ground leg to correct hub
            vtype    = VehicleType.Train if n >= TRAIN_MIN_BOXES else VehicleType.SemiTruck
            selected = box_ids[:vtype.value.capacity]
            vid = _find_idle_ground(sim_state, vtype, origin)
            if not vid:
                vid = _spawn(sim_state, vtype, _nearest(origin, SHIPPING_HUBS))
            if not vid:
                return False
            claimed.update(selected)
            itineraries[vid] = {
                "state": "TO_PICKUP", "pickup_loc": origin,
                "box_ids": selected, "waypoints": _land_wps(origin, departure_ap) + [departure_ap],
            }
            return True

        boxes = sim_state.get_boxes()
        ready = [
            b for b in box_ids
            if b not in claimed and b in boxes
            and not boxes[b]["delivered"] and boxes[b]["vehicle_id"] is None
        ]
        if not ready:
            return False
        key = (departure_ap, dst_region)
        if key not in staging:
            staging[key] = {"box_ids": [], "last_tick": sim_state.tick}
        staging[key]["box_ids"].extend(ready)
        staging[key]["last_tick"] = sim_state.tick
        claimed.update(ready)
        return True

    # ── Boxes at arrival airport, same-region as dest: last-mile dispatch ─────
    if at_airport and src_region == dst_region:
        dist_km = haversine_distance_meters(origin, dest) / 1000.0
        if n <= DRONE_CAPACITY and dist_km <= DRONE_MAX_KM:
            vid = _spawn(sim_state, VehicleType.Drone, origin)
            if vid:
                selected = box_ids[:DRONE_CAPACITY]
                claimed.update(selected)
                itineraries[vid] = {
                    "state": "TO_PICKUP", "pickup_loc": origin,
                    "box_ids": selected, "waypoints": [dest],
                }
                return True

        vtype    = VehicleType.Train if n >= TRAIN_MIN_BOXES else VehicleType.SemiTruck
        selected = box_ids[:vtype.value.capacity]
        vid = _find_idle_ground(sim_state, vtype, origin)
        if not vid:
            vid = _spawn(sim_state, vtype, origin)
        if not vid:
            return False
        claimed.update(selected)
        itineraries[vid] = {
            "state": "TO_PICKUP", "pickup_loc": origin,
            "box_ids": selected, "waypoints": _land_wps(origin, dest) + [dest],
        }
        return True

    # ── Same-region, not at airport: direct ground transport ─────────────────
    if src_region == dst_region:
        vtype    = VehicleType.Train if n >= TRAIN_MIN_BOXES else VehicleType.SemiTruck
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

    # ── Cross-region, not at airport: truck/train to the departure hub airport ─
    route = ROUTE_HUBS.get((src_region, dst_region))
    if not route:
        return False
    departure_ap, _ = route

    vtype    = VehicleType.Train if n >= TRAIN_MIN_BOXES else VehicleType.SemiTruck
    selected = box_ids[:vtype.value.capacity]
    vid = _find_idle_ground(sim_state, vtype, origin)
    if not vid:
        vid = _spawn(sim_state, vtype, _nearest(origin, SHIPPING_HUBS))
    if not vid:
        return False
    claimed.update(selected)
    wps = _land_wps(origin, departure_ap) + [departure_ap]
    itineraries[vid] = {
        "state": "TO_PICKUP", "pickup_loc": origin,
        "box_ids": selected, "waypoints": wps,
    }
    return True


def _flush_staging(sim_state):
    """Dispatch batch planes when a bucket is full or has been idle for IDLE_FLUSH_TICKS."""
    to_delete = []
    boxes = sim_state.get_boxes()

    for key, bucket in staging.items():
        departure_ap, dst_region = key

        # Verify boxes are still physically present and undelivered
        box_ids = [
            b for b in bucket["box_ids"]
            if b in boxes and not boxes[b]["delivered"] and boxes[b]["vehicle_id"] is None
        ]
        bucket["box_ids"] = box_ids

        if not box_ids:
            to_delete.append(key)
            continue

        idle_ticks = sim_state.tick - bucket["last_tick"]
        full       = len(box_ids) >= PLANE_CAPACITY
        idle       = idle_ticks >= IDLE_FLUSH_TICKS

        if not full and not idle:
            continue

        route = ROUTE_HUBS.get((None, dst_region))  # arrival only lookup
        # Find arrival airport from any route that lands in dst_region
        arrival_ap = None
        for (src_r, dst_r), (dep, arr) in ROUTE_HUBS.items():
            if dst_r == dst_region and dep == departure_ap:
                arrival_ap = arr
                break
        if not arrival_ap:
            continue

        remaining = list(box_ids)
        while remaining:
            batch = remaining[:PLANE_CAPACITY]
            if not batch:
                break
            vid = _spawn(sim_state, VehicleType.Airplane, departure_ap)
            if not vid:
                break
            itineraries[vid] = {
                "state": "TO_PICKUP", "pickup_loc": departure_ap,
                "box_ids": batch, "waypoints": [arrival_ap],
            }
            remaining = remaining[PLANE_CAPACITY:]

        bucket["box_ids"] = remaining
        if not remaining:
            to_delete.append(key)

    for key in to_delete:
        del staging[key]


def _find_idle_ground(sim_state, vtype, pickup_loc):
    """Return the nearest idle ground vehicle if reusing it is cheaper than spawning new."""
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
    if lon < -30:               return "AMERICAS"
    if lat > 0 and lon >= 100:  return "EAST_ASIA"
    if lat <= 0 and lon >= 100: return "OCEANIA"
    return "EURASIA_AFRICA"


def _is_near(loc, target, radius=INFRA_R):
    """Return True if loc is within radius metres of target."""
    return haversine_distance_meters(loc, target) <= radius


def _nearest(loc, infra):
    """Return the (lat, lon) of the closest item in an infrastructure list."""
    best_d, best_c = float("inf"), None
    for item in infra:
        d = haversine_distance_meters(loc, (item["lat"], item["lon"]))
        if d < best_d:
            best_d, best_c = d, (item["lat"], item["lon"])
    return best_c


def _near_any(loc, infra):
    """Return True if loc is within INFRA_R metres of any item in the list."""
    return any(haversine_distance_meters(loc, (i["lat"], i["lon"])) <= INFRA_R for i in infra)


def _land_wps(src, dst):
    """Return waypoints to keep a ground vehicle on solid ground between src and dst."""
    wps   = []
    src_r = _region(src)
    dst_r = _region(dst)

    if src_r == "AMERICAS" and dst_r == "AMERICAS":
        def _south_fl(loc): return loc[0] < 30.5 and loc[1] > -90.0
        def _south_am(loc): return loc[0] < 9.0
        def _north_am(loc): return loc[0] > 25.0
        def _mexico(loc):   return 9.0 <= loc[0] <= 25.0 and loc[1] < -80.0

        # Hardcoded: Mexico City / Central America zone → Sao Paulo
        # These latitudes (9–25°N) fall between _north_am and _south_am so they need
        # explicit handling — go through Guatemala (SAO_GW1) to stay on land.
        if _mexico(src) and _south_am(dst):
            wps.append(SAO_GW1)
            return wps
        if _south_am(src) and _mexico(dst):
            wps.append(SAO_GW1)
            return wps

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
        def _s_africa(loc): return loc[0] < -15.0 and 15.0 < loc[1] < 45.0
        def _e_africa(loc): return -5.0 < loc[0] < 15.0 and 30.0 < loc[1] < 50.0
        def _mideast(loc):  return loc[0] > 10.0 and 45.0 < loc[1] < 85.0

        if _s_africa(src) != _s_africa(dst):
            wps.append(CENT_AFRICA)
        if (_e_africa(src) and _mideast(dst)) or (_mideast(src) and _e_africa(dst)):
            wps.append(EGYPT_GW)
        return wps

    if src_r == "EAST_ASIA" and dst_r == "EAST_ASIA":
        def _se_asia(loc): return loc[0] < 15.0 and loc[1] >= 100.0
        def _japan(loc):   return loc[0] > 30.0 and loc[1] > 125.0

        if (_se_asia(src) and _japan(dst)) or (_japan(src) and _se_asia(dst)):
            wps.append(UPPER_CHINA)
        return wps

    if {src_r, dst_r} == {"EAST_ASIA", "EURASIA_AFRICA"}:
        wps.append(EGYPT_GW)

    return wps


def _spawn(sim_state, vtype, preferred_loc):
    """Try to spawn a vehicle at preferred_loc, falling back through the appropriate infra list."""
    if vtype in (VehicleType.SemiTruck, VehicleType.Train):
        fallback = [h for h in SHIPPING_HUBS if _region((h["lat"], h["lon"])) != "OCEANIA"]
    else:
        fallback = AIRPORTS

    for loc in [preferred_loc] + [(i["lat"], i["lon"]) for i in fallback]:
        try:
            return sim_state.create_vehicle(vtype, loc)
        except (ValueError, TypeError):
            continue
    return None