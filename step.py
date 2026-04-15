from simulator import VehicleType
import math

# Helper to calculate distance
def get_dist(c1, c2):
    return math.sqrt((c1[0]-c2[0])**2 + (c1[1]-c2[1])**2)

# We use a global-style dictionary to track which boxes are already assigned
# to prevent creating multiple vehicles for the same box.
assigned_boxes = set()

def step(sim_state):
    global assigned_boxes
    
    boxes = sim_state.get_boxes()
    vehicles = sim_state.get_vehicles()

    # 1. UNLOAD LOGIC (Check all vehicles to see if they reached their destination)
    for vid, v in vehicles.items():
        if v["cargo"] and v["destination"] is not None:
            # Check if we are at the destination
            # (Using a small threshold for GPS coordinates)
            for bid in v["cargo"]:
                box_dest = boxes[bid]["destination"]
                if get_dist(v["location"], box_dest) < 0.01:
                    # Unload the box
                    sim_state.unload_vehicle(vid, [bid])
                    # Once unloaded, the box is marked delivered by the API

    # 2. DISPATCH LOGIC (Find new boxes to deliver)
    for bid, box in boxes.items():
        if not box["delivered"] and bid not in assigned_boxes:
            # Check if the box is already on a vehicle
            if box["vehicle_id"] is not None:
                assigned_boxes.add(bid)
                continue

            dist = get_dist(box["location"], box["destination"])
            
            # Determine the best vehicle type
            # If far away, Airplane is safest to avoid ocean penalties
            if dist > 1000:
                preferred_types = [VehicleType.Airplane, VehicleType.SemiTruck]
            else:
                preferred_types = [VehicleType.SemiTruck, VehicleType.Airplane]

            # Attempt to spawn a vehicle
            spawned = False
            for vtype in preferred_types:
                try:
                    # Try to spawn at the box's current location
                    vid = sim_state.create_vehicle(vtype, box["location"])
                    
                    # If successful, load and move
                    sim_state.load_vehicle(vid, [bid])
                    sim_state.move_vehicle(vid, box["destination"])
                    
                    assigned_boxes.add(bid)
                    spawned = True
                    break 
                except ValueError:
                    # This happens if the vehicle type is not allowed at this location
                    # (e.g. trying to spawn a Truck at an Airport)
                    continue
            
            if not spawned:
                # If we couldn't spawn anything, we'll try again next tick
                pass
