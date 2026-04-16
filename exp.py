from simulator import VehicleType, haversine_distance_meters
import random


_PROXIMITY_M = 50.0    


def cost_analysys(distance,can_get_via_port,can_get_via_plane,can_get_via_land):
    plane_rate = 0.5
    boat_rate = 0.01
    train_rate = 0.02
    semi_rate = 0.05
    drone_rate = 0.3
    costs=[-1,-1,-1]
    min_num = -1
    index = 0
    if can_get_via_plane:
        costs[0] = distance*plane_rate
        min_num = costs[0]
    if can_get_via_port:
        costs[1] = boat_rate*distance
        if min_num == -1:
            min_costs = costs[1]
            index == 1
        else:
            min_costs = min(costs[1],min_costs)
            if min_costs == costs[1]:
                index = 1
    if can_get_via_land:
        costs[2] = min(train_rate*distance,semi_rate*distance,drone_rate*distance)
        if min_costs == -1:
            min_costs = costs[2]
            index = 2
        else:
            min_costs = min(costs[2],min_costs)
            if(min_costs) == costs[2]:
                index = 2
    return {costs:index,index:index}




def hub_hasAirport(hub):
    
    for i in Airports:
        if haversine_distance(i,hub) <= _PROXIMITY_M:
            return True
    return False

def hub_hasPort(hub):
    for i in Ports:
        if haversine_distance(i,hub) <= _PROXIMITY_M:
            return True
    return False

vehic = None


N_A_Hubs = []
N_A_Matrix = []
Airports = None
Ports = None
boxes = []
path_mat = None

linker_truck = None

curr_dest = None

def step(sim_state):
    """Called by the simulator each tick.

    Args:
        sim_state: A :class:`~simulator.SimulationState` instance.  State may
                   only be modified through its public command methods.
    """
    tick = sim_state.tick
    boxes = sim_state.get_boxes()
    hubs = sim_state.get_shipping_hub_details()
    global path_mat
    global N_A_Matrix
    global linker_truck
    global curr_dest
    
    #gets location and number of boxes in shipping hubs
    if(tick == 0):
        for hub in hubs:
            #all hubs in north america are to the left of this coordinate
            if hub["location"]["lon"] < -46.6333:
                tup = (hub["location"]["lat"],hub["location"]["lon"],len(hub["boxes"]))
                N_A_Hubs.append(tup)
        sorted_NA = sorted(N_A_Hubs,key=lambda x:x[1])
        print(sorted_NA)

        for box in boxes.items():
            pass
        #sorts in ascending order
        for i in range(len(sorted_NA)):
            row = []
            for j in range(len(sorted_NA)):
                if( i == j):
                    row.append(0)
                    continue
                row.append(haversine_distance_meters((sorted_NA[i][0],sorted_NA[i][1]),(sorted_NA[j][0],sorted_NA[j][1])))
            N_A_Matrix.append(row)

        #zeroes out hubs that do not have any boxes
        for i in range(len(sorted_NA)):
            if sorted_NA[i][2] == 0:
                for j in range(len(N_A_Matrix)):
                    N_A_Matrix[i][j] = 0
                    pass
                for j in range(len(N_A_Matrix)):
                    N_A_Matrix[j][i] = 0
                    pass
        #prints out boxes
        for i in range(len(N_A_Matrix)):
            print(N_A_Matrix[i])
        path_mat = [[0 for _ in range(len(N_A_Matrix))] for _ in range(len(N_A_Matrix))]
        visited_hub = {0}
        
        for i in range(len(sorted_NA)):
            if(i in visited_hub):
                continue
            val = None
            index = 0
            has_reached_value = False
            for j in range(len(N_A_Matrix)):
                if N_A_Matrix[i][j] is not 0 and not has_reached_value and j not in visited_hub:
                    has_reached_value = True
                    val = N_A_Matrix[i][j]
                    index = j
                if has_reached_value and j not in visited_hub and N_A_Matrix[i][j] is not 0 and N_A_Matrix[i][j] <= val:
                    val = N_A_Matrix[i][j]
                    index = j
                    path_mat[i][j] = 1
                    path_mat[j][i] = 1
            visited_hub.add(index)
        
        
        for i in range(len(path_mat)):
            for j in range(len(path_mat)):
                if(path_mat[i][j] == 1):
                    linker_truck = sim_state.create_vehicle(VehicleType.SemiTruck,(sorted_NA[i][0],sorted_NA[i][1]))
                    curr_dest = (sorted_NA[j][0],sorted_NA[j][1])
                    break
                    
            
        for i in range(len(path_mat)):
            print(path_mat[i])
    if(tick is not 0):
        sim_state.move_vehicle(linker_truck,curr_dest)
                
        

                
            


    if(vehic is not None):
        print(vehic)
