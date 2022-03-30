import networkx as nx
import numpy as np
import scipy.sparse
from scipy.optimize import linprog
from scipy import sparse
"""
对于一个无人机去送包裹的问题，只有两种情况，
1：无人机到包裹的直线距离小于无人机最大的飞行距离的二分之一，这时无人机直接飞过去即可
2：无人机到包裹的直线距离大于无人机最大的飞行距离的二分之一，此时不能直接飞过去，因为无人机有可能无法飞回其中一个仓库充电，此时，无人机想要送达包裹，
只有借助换乘点来接近转换到不同的换乘点，期望该换乘点离包裹的直线距离更近，在此过程中，无人机直线距离飞到换乘点的消耗，以及最终所在换乘点离包裹的直线距离总和，
也不能超过无人机最大飞行距离的一半。

solution1:所以，建立新的图，只包括仓库、包裹和传输点，边只有(所有仓库到所有包裹,不可以，因为直接结束)，所有仓库到所有传输点，所有包裹到所有传输点，传输点到传输点(考虑换乘),
图的权值为直线距离，但特殊的是换乘点之间的边权值为0，表示无人机在换乘边之间移动不消耗最大距离，这样，我们找到无人机到达包裹的最小消耗（只是消耗最小，不一定是时间最短）
找到时间最短，可以从无人机到达包裹所有可行路径中，算出时间最短的？复杂度？
solution2:为每个dp问题或者pd(因为是无向图，所有dp和pd路程是一样的)，建立子图(矩阵)，一个矩阵包含了一个仓库到所有换乘点，仓库到包裹(一个)，一个包裹到所有换乘点的直线距离消耗(换乘点消耗为0)，
另一个矩阵是这样的花费时间(直线距离/无人机速度 直线距离/货车速度 + 等待时间)
"""
from math import radians, cos, sin, asin, sqrt


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


def drone_find_path(TG, dpd, transit_node, packages_node, depots_node, flight_matrix, transit_edges):
    max_fligth = 200
    depot1 = dpd[0]
    packge = dpd[1]
    depot2 = dpd[2] # maybe are depot1
    # if can fly directly by consuming no more than half of max_flight_distance, just fly


def generate_transit_network(TG, transit_node, transit_edges, depot_id, package_id):
    # we generate a new type graph
    transit_network = nx.Graph()
    # firstly, add transit nodes
    for i in transit_node:
        #     print(i)
        transit_network.add_node(i
                                 , lon=TG.nodes[i]['lon']
                                 , lat=TG.nodes[i]['lat']
                                 , type='transit'
                                 )
    # then we add transit to another transit edges
    for i in transit_node:
        for j in transit_node:
            if i != j:
                transit_network.add_edge(i, j
                                         , type='flght'
                                         , weight=haversine(TG.nodes[i]['lon'], TG.nodes[i]['lat'], TG.nodes[j]['lon'],
                                                            TG.nodes[j]['lat'])
                                         )
    # reset transit edges weight to 0
    for edge in transit_edges:
        transit_network.edges[edge[0], edge[1]]['weight'] = 0.0
        transit_network.edges[edge[0], edge[1]]['type'] = 'transit'

    print(len(transit_network.edges))

    # we add depots and packages

    transit_network.add_node(depot_id
                             , lon=TG.nodes[depot_id]['lon']
                             , lat=TG.nodes[depot_id]['lat']
                             , type='depot'
                             )

    transit_network.add_node(package_id
                             , lon=TG.nodes[package_id]['lon']
                             , lat=TG.nodes[package_id]['lat']
                             , type='package'
                             )
    transit_network.add_edge(depot_id, package_id
                             , type='flight'
                             , weight=haversine(TG.nodes[depot_id]['lon'], TG.nodes[depot_id]['lat'],
                                                TG.nodes[package_id]['lon'], TG.nodes[package_id]['lat'])
                             )

    print(len(transit_network.edges))

    # finally we add edge ?depot to depot?, depot to package and depot to transit and transit to package

    for j in transit_node:  # depot to transit
        transit_network.add_edge(depot_id, j
                                 , type='flight'
                                 , weight=haversine(TG.nodes[depot_id]['lon'], TG.nodes[depot_id]['lat'],
                                                    TG.nodes[j]['lon'], TG.nodes[j]['lat'])  # directly distance
                                 )
    print(len(transit_network.edges))
    for t in transit_node:
        transit_network.add_edge(t, package_id
                                 , type='flight'
                                 , weight=haversine(TG.nodes[t]['lon'], TG.nodes[t]['lat'], TG.nodes[package_id]['lon'],
                                                    TG.nodes[package_id]['lat'])  # directly distance
                                 )
    print(len(transit_network.edges))

    return transit_network


"""
线性规划
我们有  边的消耗飞行距离向量 consume_vector    边的消耗时间向量 time_vector，这两个下标同一对应。 优化向量x，目的是 min time_vector * x  
限制1 consume_vector * x <= max_flight_distance/2
为了确保优化向量x选中的边形成一条可通行的路径(中间没有跳跃断层), 限制2 每个节点的出度等于入度，这样就保证了我们到达一个节点，也要从这个节点出去，不存在跳跃的情况 
限制3 仓库的出度是1，入度是0  
限制4 包裹的出度是0，入度是1
"""
def route_plan(TG, depot_id, package_id, max_flight_distance):
    distance_cost_vector = np.array([])
    time_cost_vector = np.array([])
    edge_to_vector_idx = {}
    vector_idx_to_edge = {}
    out_nbrs = {}
    in_nbrs = {}

    for i in TG.nodes:
        for j in TG.nodes:
            if i != j:
                distance_cost = TG.edges[i, j]['weight']
                time_cost = haversine(TG.nodes[i]['lon'], TG.nodes[i]['lat'], TG.nodes[j]['lon'], TG.nodes[j]['lat'])

                # append to vector
                distance_cost_vector = np.append(distance_cost_vector, distance_cost)
                time_cost_vector = np.append(time_cost_vector, time_cost)
                edge_to_vector_idx[(i, j)] = len(distance_cost_vector)-1
                vector_idx_to_edge[len(distance_cost_vector)-1] = (i, j)

                if i not in out_nbrs:
                    out_nbrs[i] = []
                out_nbrs[i].append(j)

                if j not in in_nbrs:
                    in_nbrs[j] = []
                in_nbrs[j].append(i)
    #
    # for (i, j) in TG.edges:
    #     distance_cost = TG.edges[i, j]['weight']
    #     time_cost = haversine(TG.nodes[i]['lon'], TG.nodes[i]['lat'], TG.nodes[j]['lon'], TG.nodes[j]['lat'])




    # do MIP: we nned to get vector x make x * time_cost_vector is minimum

    n_idxs = len(time_cost_vector)
    print("n_idx", n_idxs)
    print("distance_cost_vector_len", len(distance_cost_vector))

    # constrain1
    # make sure consume_vector * x <= max_flight_distance/2
    A_ub_constraint1 = distance_cost_vector
    b_ub_constraint1 = np.array([max_flight_distance])

    # constrain2
    # make sure every node in_degree equals to out_degree   but ignore depot and package
    node_out_edge_mask = np.zeros((len(TG.nodes)-2, n_idxs))
    node_in_edge_mask = np.zeros((len(TG.nodes)-2, n_idxs))
    # print("out shape", node_out_edge_mask.shape)
    # print("in shape", node_in_edge_mask.shape)

    node_nbr_idx = -1
    for i in TG.nodes:
        if i != depot_id and i != package_id:
            node_nbr_idx += 1
            for onbr in out_nbrs[i]:
                index = edge_to_vector_idx[i, onbr]
                node_out_edge_mask[node_nbr_idx, index] = 1

            for inbr in in_nbrs[i]:
                index = edge_to_vector_idx[inbr, i]
                node_in_edge_mask[node_nbr_idx, index] = 1

    for i in range(len(TG.nodes)-2):
        for j in range(n_idxs):
            node_out_edge_mask[i, j] = node_out_edge_mask[i, j] - node_in_edge_mask[i, j]

    A_eq_constraint1 = node_out_edge_mask
    b_eq_constraint1 = np.zeros(len(TG.nodes)-2).reshape(-1, 1)


    # constrain3
    # out-flow and in-flow from depot is 1 and 0
    depot_out_edge_mask = np.zeros(n_idxs)
    depot_in_edge_mask = np.zeros(n_idxs)
    # print("out shape", depot_out_edge_mask.shape)
    # print("in shape", depot_in_edge_mask.shape)

    for onbr in out_nbrs[depot_id]:
        index = edge_to_vector_idx[depot_id, onbr]
        depot_out_edge_mask[index] = 1
    for inbr in in_nbrs[depot_id]:
        index = edge_to_vector_idx[inbr, depot_id]
        depot_in_edge_mask[index] = 1

    A_eq_constraint2 = np.vstack([depot_out_edge_mask, depot_in_edge_mask])
    b_eq_constraint2 = np.array([1, 0]).reshape(-1 ,1)
    # print("out shape", depot_out_edge_mask.shape)
    # print("in shape", depot_in_edge_mask.shape)

    # constrain3
    # out-flow and in-flow from package is 0 and 1
    package_out_edge_mask = np.zeros(n_idxs)
    package_in_edge_mask = np.zeros(n_idxs)

    for onbr in out_nbrs[package_id]:
        index = edge_to_vector_idx[package_id, onbr]
        package_out_edge_mask[index] = 1
    for inbr in in_nbrs[package_id]:
        index = edge_to_vector_idx[inbr, package_id]
        package_in_edge_mask[index] = 1

    A_eq_constraint3 = np.vstack([package_out_edge_mask, package_in_edge_mask])
    b_eq_constraint3 = np.array([0, 1]).reshape(-1 ,1)
    # add all constraints
    A_ub = A_ub_constraint1.reshape(1, -1)
    b_ub = b_ub_constraint1.reshape(1 ,-1)
    # print("eq_con1 shape", A_eq_constraint1.shape)
    # print("eq_con2 shape", A_eq_constraint2.shape)
    A_eq = np.vstack([A_eq_constraint1, A_eq_constraint2])
    # print('1.5')
    # print("A_eq shape", A_eq.shape)
    # print("eq con3 shape", A_eq_constraint3.shape)
    A_eq = np.vstack([A_eq, A_eq_constraint3])

    # print("b con1 shape", b_eq_constraint1.shape)
    # print("b con2 shape", b_eq_constraint2.shape)
    b_eq = np.vstack([b_eq_constraint1, b_eq_constraint2]).reshape(-1, 1)
    # print("b_eq", b_eq.shape)
    b_eq = np.vstack([b_eq, b_eq_constraint3]).reshape(-1, 1)

    x_bound = [(0, 1) for _ in range(n_idxs)]
    print("start potimizer")
    print("A_ub", A_ub.shape)
    print("b_ub", b_ub.shape)
    print("A_eq", A_eq.shape)
    print("b_uq", b_eq.shape)

    res = linprog(c=time_cost_vector, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq,
                  bounds=x_bound, )
    print(res)
    x_edges = res.x
    # covert float to int
    x_edges = [round(x) for x in x_edges]

    # finally we get the optimal edges that less than x_edges
    edges = [vector_idx_to_edge[i] for (i, val) in enumerate(x_edges) if val > 0]

    return x_edges, edges