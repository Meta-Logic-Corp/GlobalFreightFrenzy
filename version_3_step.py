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

        # Hardcoded intercept: any North American or Florida-area origin heading to
        # South America must go through SAO_GW1 (Guatemala) and SAO_GW2 (New Mexico)
        # to follow the Central American land bridge. FLORIDA_GW is prepended if the
        # truck is starting in the South Florida / Gulf coast zone.
        if _north_am(src) and _south_am(dst):
            if _south_fl(src):
                wps.append(FLORIDA_GW)
            wps.extend([SAO_GW2, SAO_GW1])
            return wps
        if _south_am(src) and _north_am(dst):
            wps.extend([SAO_GW1, SAO_GW2])
            if _south_fl(dst): wps.append(FLORIDA_GW)
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