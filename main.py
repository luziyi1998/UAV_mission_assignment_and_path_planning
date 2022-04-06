import argparse
import graph.generate_graph
import task_allocation_layer.task_allocation
import MAPF.single_drone_FP
import numpy as np
import networkx as nx
from math import radians, cos, sin, asin, sqrt
import MAPF.Astar

def haversine(lon1, lat1, lon2, lat2):  # 经度1，纬度1，经度2，纬度2 （十进制度数）
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # 将十进制度数转化为弧度
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine公式
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))
    r = 6371  # 地球平均半径，单位为公里
    return c * r * 1000


def UAV_task_allocation_path_planning(N_depots, N_transit_edges, N_packages, N_drones, Max_flight):
    # get a graph that randomly generate depot nodes, transit nodes and package nodes and cost_matrix
    TG, depots_node, transit_node, packages_node, transit_edges, cost_matrix = graph.generate_graph.return_Digraph(N_depots,
                                                                                                N_transit_edges, N_packages)

    def dist(a, b, TG=TG):
        lat1 = TG.nodes[a]['lat']
        lon1 = TG.nodes[a]['lon']
        lat2 = TG.nodes[b]['lat']
        lon2 = TG.nodes[b]['lon']
        #     print("node",a,lon1,lat1,"to node",b,lon2,lat2," liner distance:",haversine(lon1, lat1, lon2, lat2))
        #     print(haversine(lon1, lat1, lon2, lat2))
        #     print(lat1, lon1, lat2, lon2)
        return haversine(lon1, lat1, lon2, lat2)


    # update other parameters
    total_nodes = len(TG.nodes)
    N_transits = len(transit_node)

    # we get transit node, package node and depot node, now we store the rest nodes are normal node
    normal_node = np.array([])
    for i in range(len(TG.nodes())):
        if i not in depots_node and i not in transit_node and i not in packages_node:
            normal_node = np.append(normal_node, i)

    print("do task allocation")
    drone_tours = task_allocation_layer.task_allocation.task_allocation(total_nodes, N_depots, N_packages, N_drones,
                                                                        cost_matrix, depots_node, transit_node,
                                                                        packages_node, normal_node)


    # find each drones route
    drone_routes = []
    for i in range(len(drone_tours)):
        cur_route = [drone_tours[i][0]]
        #     print("drone", i, "route:")
        for (node1, node2) in zip(drone_tours[i][:len(drone_tours[i]) - 1], drone_tours[i][1:]):
            cur_route.extend(nx.astar_path(TG, node1, node2, dist)[1:])
        #         print(nx.astar_path(TG, node1, node2, dist))
        drone_routes.append(cur_route)

    pos_location = {}
    for i in TG.nodes:
        pos_location[i] = (TG.nodes[i]['lon'], TG.nodes[i]['lat'])

    node_size = 1000
    colo_map = {}
    nod_size_map = {}
    for i in transit_node:
        colo_map[i] = 'green'
        nod_size_map[i] = node_size
    for i in packages_node:
        colo_map[i] = 'red'
        nod_size_map[i] = node_size
    for i in depots_node:
        colo_map[i] = 'black'
        nod_size_map[i] = node_size
    for i in normal_node:
        colo_map[i] = 'grey'
        nod_size_map[i] = 0
    len(colo_map)
    color_map = []
    node_size_map = []
    for i in range(len(colo_map)):
        color_map.append(colo_map[i])
        node_size_map.append(nod_size_map[i])

    import matplotlib.pyplot as plt
    plt.rcParams['figure.figsize'] = (40, 40)  # 单位是inches
    # green transit_node
    # black depot_node
    # red package node
    # grey normal_node
    nx.draw(TG,
            pos=pos_location,
            node_size=node_size_map,
            cmap=plt.get_cmap('viridis'),
            node_color=color_map,
            width=1,
            with_labels=True,
            font_color='white')
    fig_name = 'bicycle_map.jpg'
    plt.savefig(fig_name, dpi=200)
    # plt.show()

    drone_one_dilivery_route = []
    drone_one_dilivery_time_stamp = []
    drone_one_dilivery_time = []
    fail_packages = []
    # now we compute each drone routes(我们应该为每个无人机先计算出一个交付任务——从仓库出发再到回到仓库，然后计算这些交付序列的冲突节点,解决所有冲突后，得到所有无人机该交付任务的最终路径)
    # 我们现在只算每个无人机第一个任务路径
    for i in range(len(drone_tours)):
        start_depot, package = drone_tours[i][0:2]
        print()
        success, departure_route, departure_time_stamp, departure_time = one_drone_dilivery_departure_route(TG, transit_node, transit_edges, start_depot, package, Max_flight)
        if success:
            drone_one_dilivery_route.append(departure_route)
            drone_one_dilivery_time_stamp.append(departure_time_stamp)
            drone_one_dilivery_time.append(departure_time)
        else:
            if package in packages_node:
                fail_packages.append(package)
            drone_one_dilivery_route.append([])
            drone_one_dilivery_time_stamp.append([0])
            drone_one_dilivery_time.append(-1) # denote fail


    return drone_one_dilivery_route, drone_one_dilivery_time_stamp, drone_one_dilivery_time, fail_packages







def one_drone_dilivery_departure_route(TG, transit_node, transit_edges, depot_id, package_id, Max_flight):
    # now we get a specific transit network only contain one depot and one package and transit node
    transit_network = MAPF.single_drone_FP.generate_transit_network(TG, transit_node, transit_edges, depot_id, package_id)

    success, x_edges, sub_mission_way = MAPF.single_drone_FP.departure_route_plan(transit_network, depot_id, package_id, Max_flight)
    if success == False:
        return False, [], [0], -1
    # path finding by Linear Planning
    route_plan_distance_cost = 0
    cur_time = 0
    for way in sub_mission_way:
        route_plan_distance_cost += transit_network.edges[way[0], way[1]]['weight']
    print(depot_id, " to", package_id, "route_plan_distance_cost", route_plan_distance_cost)

    # get the entire route of this sub_mission
    direct_edges = {}
    for (i, j) in sub_mission_way:
        direct_edges[i] = j
    sub_mission_route_by_LP = [depot_id]
    cur_node = depot_id
    while len(sub_mission_route_by_LP) <= len(sub_mission_way):
        sub_mission_route_by_LP.append(direct_edges[cur_node])
        cur_node = direct_edges[cur_node]
    print("route:", sub_mission_route_by_LP)


    route_plan_time_cost = 0
    time_stamp = [0.0]
    for (i, j) in zip(sub_mission_route_by_LP[:len(sub_mission_route_by_LP) - 1], sub_mission_route_by_LP[1:]):
        route_plan_time_cost += transit_network.edges[i, j]['time']
        time_stamp.append(route_plan_time_cost)

        # if transit_network.edges[i, j]['type'] == 'flight':
        #     route_plan_time_cost += transit_network.edges[i, j]['weight']
        # else:
        #     route_plan_time_cost += haversine(TG.nodes[i]['lon'], TG.nodes[i]['lat'], TG.nodes[j]['lon'],
        #                                             TG.nodes[j]['lat'])

    print("use time", route_plan_time_cost)

    return True, sub_mission_route_by_LP, time_stamp, route_plan_time_cost


def update_transit_constrain(transit_constrain, transit_node, arrive_time, wait_time):
    left, right = arrive_time, arrive_time + wait_time
    placed = False
    ans = []
    for li, ri in transit_constrain[transit_node]:
        if li >right:
            if not placed:
                ans.append([left, right])
                placed = True
            ans.append([li, ri])
        elif ri < left:
            ans.append([li, ri])
        else:
            left = min(left, li)
            right = max(right, ri)

    if not placed:
        ans.append([left, right])

    transit_constrain[transit_node] = ans
    return transit_constrain


def one_drone_dilivery_departure_route_by_astar(TG, transit_constrain, transit_node, transit_edges, depot_id, package_id, Max_flight, start_time=0):
    # now we get a specific transit network only contain one depot and one package and transit node
    """
    :param TG:
    :param transit_constrain:
    :param transit_node:
    :param transit_edges:
    :param depot_id:
    :param package_id:
    :param Max_flight:
    :return:
    """
    transit_network = MAPF.single_drone_FP.generate_transit_network(TG, transit_node, transit_edges, depot_id, package_id)

    success, sub_mission_way, transit_wait = MAPF.Astar.astar_with_conflict_avoid(transit_network, transit_constrain, transit_edges, depot_id, package_id, Max_flight, start_time)
    if success == False:
        return False, [], [0], -1
    # path finding by astar
    route_plan_distance_cost = 0
    route_plan_time_cost = start_time
    time_stamp = [start_time]
    for (i, j) in zip(sub_mission_way[:len(sub_mission_way)-1], sub_mission_way[1:]):
        route_plan_distance_cost += transit_network.edges[i, j]['weight']
        if transit_network.edges[i, j]['type'] == 'transit':
            update_transit_constrain(transit_constrain, i, route_plan_time_cost, transit_wait[(i, j)])
            route_plan_time_cost += transit_wait[(i, j)]
        route_plan_time_cost += transit_network.edges[i, j]['time']

        time_stamp.append(route_plan_time_cost)
    # print(depot_id, " to", package_id, "route_plan_distance_cost", route_plan_distance_cost)
    # print("route:", sub_mission_way)
    # print("use time", route_plan_time_cost)

    return True, sub_mission_way, time_stamp, route_plan_time_cost


def UAV_task_allocation_path_planning_by_astar(N_depots, N_transit_edges, N_packages, N_drones, Max_flight):
    # get a graph that randomly generate depot nodes, transit nodes and package nodes and cost_matrix
    TG, depots_node, transit_node, packages_node, transit_edges, cost_matrix = graph.generate_graph.return_Digraph(N_depots,
                                                                                                N_transit_edges, N_packages)

    def dist(a, b, TG=TG):
        lat1 = TG.nodes[a]['lat']
        lon1 = TG.nodes[a]['lon']
        lat2 = TG.nodes[b]['lat']
        lon2 = TG.nodes[b]['lon']
        #     print("node",a,lon1,lat1,"to node",b,lon2,lat2," liner distance:",haversine(lon1, lat1, lon2, lat2))
        #     print(haversine(lon1, lat1, lon2, lat2))
        #     print(lat1, lon1, lat2, lon2)
        return haversine(lon1, lat1, lon2, lat2)


    # update other parameters
    total_nodes = len(TG.nodes)
    N_transits = len(transit_node)

    # we get transit node, package node and depot node, now we store the rest nodes are normal node
    normal_node = np.array([])
    for i in range(len(TG.nodes())):
        if i not in depots_node and i not in transit_node and i not in packages_node:
            normal_node = np.append(normal_node, i)

    print("do task allocation")
    drone_tours = task_allocation_layer.task_allocation.task_allocation(total_nodes, N_depots, N_packages, N_drones,
                                                                        cost_matrix, depots_node, transit_node,
                                                                        packages_node, normal_node)


    # find each drones route
    drone_routes = []
    for i in range(len(drone_tours)):
        cur_route = [drone_tours[i][0]]
        #     print("drone", i, "route:")
        for (node1, node2) in zip(drone_tours[i][:len(drone_tours[i]) - 1], drone_tours[i][1:]):
            cur_route.extend(nx.astar_path(TG, node1, node2, dist)[1:])
        #         print(nx.astar_path(TG, node1, node2, dist))
        drone_routes.append(cur_route)

    pos_location = {}
    for i in TG.nodes:
        pos_location[i] = (TG.nodes[i]['lon'], TG.nodes[i]['lat'])

    node_size = 1000
    colo_map = {}
    nod_size_map = {}
    for i in transit_node:
        colo_map[i] = 'green'
        nod_size_map[i] = node_size
    for i in packages_node:
        colo_map[i] = 'red'
        nod_size_map[i] = node_size
    for i in depots_node:
        colo_map[i] = 'black'
        nod_size_map[i] = node_size
    for i in normal_node:
        colo_map[i] = 'grey'
        nod_size_map[i] = 0
    len(colo_map)
    color_map = []
    node_size_map = []
    for i in range(len(colo_map)):
        color_map.append(colo_map[i])
        node_size_map.append(nod_size_map[i])

    import matplotlib.pyplot as plt
    plt.rcParams['figure.figsize'] = (40, 40)  # 单位是inches
    # green transit_node
    # black depot_node
    # red package node
    # grey normal_node
    nx.draw(TG,
            pos=pos_location,
            node_size=node_size_map,
            cmap=plt.get_cmap('viridis'),
            node_color=color_map,
            width=1,
            with_labels=True,
            font_color='white')
    fig_name = 'bicycle_map.jpg'
    plt.savefig(fig_name, dpi=200)
    # plt.show()

    # drone_one_dilivery_route = []
    # drone_one_dilivery_time_stamp = []
    # drone_one_dilivery_time = []
    # fail_packages = []
    drone_start_time = [0 for _ in range(len(drone_tours))]
    # now we compute each drone routes(我们应该为每个无人机先计算出一个交付任务——从仓库出发再到回到仓库，然后计算这些交付序列的冲突节点,解决所有冲突后，得到所有无人机该交付任务的最终路径)

    # 生成初始传输点限制
    transit_constrain = {}
    for transit in transit_node:
        transit_constrain[transit] = []
    # 我们现在只算每个无人机第一个任务路径
    # for i in range(len(drone_tours)):
    #     start_depot, package = drone_tours[i][0:2]
    #     success, departure_route, departure_time_stamp, departure_time = one_drone_dilivery_departure_route_by_astar(TG, transit_constrain, transit_node, transit_edges, start_depot, package, Max_flight, drone_start_time[i])
    #     if success:
    #         drone_one_dilivery_route.append(departure_route)
    #         drone_one_dilivery_time_stamp.append(departure_time_stamp)
    #         drone_one_dilivery_time.append(departure_time)
    #         drone_start_time[i] = drone_one_dilivery_time_stamp[-1]
    #     else:
    #         if package in packages_node:
    #             fail_packages.append(package)
    #         drone_one_dilivery_route.append([])
    #         drone_one_dilivery_time_stamp.append([0])
    #         drone_one_dilivery_time.append(-1) # denote fail

    drone_route_plans, drone_route_time_stamp = UAV_find_all(TG, drone_tours, transit_constrain, transit_node, transit_edges, Max_flight)
    return drone_route_plans, drone_route_time_stamp


    # return drone_one_dilivery_route, drone_one_dilivery_time_stamp, drone_one_dilivery_time, fail_packages


def UAV_find_all(TG, drone_tours, transit_constrain, transit_node, transit_edges, Max_flight=400):
    """
    :param TG:
    :param drone_tours:
    :param transit_constrain:
    :param transit_node:
    :param transit_edges:
    :return:
    """
    total_task = 0
    for tour in drone_tours:
        total_task += len(tour) - 1
    task_plan = 0
    index = 0
    drone_route_plans = [[] for _ in range(len(drone_tours))]
    drone_route_time_stamp = [[] for _ in range(len(drone_tours))]
    drone_start_time = [0 for _ in range(len(drone_tours))]
    while task_plan < total_task:
        for i in range(len(drone_tours)):
            if index + 1 >= len(drone_tours[i]):
                continue
            start, end = drone_tours[i][index], drone_tours[i][index + 1]
            success, departure_route, departure_time_stamp, departure_time = one_drone_dilivery_departure_route_by_astar(
                TG, transit_constrain, transit_node, transit_edges, start, end, Max_flight, drone_start_time[i])
            #         print(drone_tours[i][index], drone_tours[i][index+1])
            if success:
                drone_route_plans[i].append(departure_route)
                drone_route_time_stamp[i].append(departure_time_stamp)
                drone_start_time[i] = departure_time_stamp[-1]
            else:
                drone_route_plans[i].append([])
                drone_route_time_stamp[i].append([0])
            task_plan += 1
        index += 1
    return drone_route_plans, drone_route_time_stamp

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--N_depots", type=int, default=3, help="number of depots")
    parser.add_argument("--N_drones", type=int, default=20, help="number of drones")
    parser.add_argument("--N_packages", type=int, default=60, help="number of packages")
    parser.add_argument("--N_transit_edges", type=int, default=20, help="number of transit edges")
    parser.add_argument("--Max_flight", type=int, default=400, help="drone's max flight distance")
    parser.add_argument("--Algorithm", type=str, default='astar', help="drone's max flight distance")
    # 接受基础参数
    opt = parser.parse_args()
    N_depots = opt.N_depots
    N_drones = opt.N_drones
    N_packages = opt.N_packages
    N_transit_edges = opt.N_transit_edges
    Max_flight = opt.Max_flight
    Algorithm = opt.Algorithm
    # print(N_depots, N_drones, N_packages, N_transit_edges)

    if Algorithm == 'astar':
        print("use astar")
        drone_dilivery_route, drone_delivery_time_stamp = UAV_task_allocation_path_planning_by_astar(N_depots, N_transit_edges, N_packages, N_drones, Max_flight)
        print(drone_dilivery_route)
        print(drone_delivery_time_stamp)
    else:
        drone_one_dilivery_route, drone_one_delivery_time_stamp, drone_one_dilivery_time, fail_packages = UAV_task_allocation_path_planning(N_depots, N_transit_edges, N_packages, N_drones, Max_flight)


        for (route, time_stamp, time) in zip(drone_one_dilivery_route, drone_one_delivery_time_stamp, drone_one_dilivery_time):
            print("route:", route)
            print("time_stamp", time_stamp)
            print("time", time)
        print("fail deliveries", fail_packages)




