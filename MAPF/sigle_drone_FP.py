import networkx as nx

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


def depot_to_package_path(TG, transit_node, transit_edges, depot_id, package_id):
    # we generate a new type graph
    transit_network = nx.Graph()
    # firstly, add transit nodes
    for i in transit_node:
        transit_network.add_node(i
                                 , lon=TG.nodes[i]['lon']
                                 , lat=TG.nodes[i]['lat']
                                 , type='transit'
                                 )
    # then we add transit edges
    for edge in transit_edges:
        transit_network.add_edge(edge[0], edge[1]
                                 , type='transit'
                                 , weight=0
                                 )
        for t in transit_node:  # transit to transtit (not a pair of transit node)
            if t not in edge:
                transit_network.add_edge(edge[0], t
                                         , type='flight'
                                         , weight=haversine(TG.nodes[edge[0]]['lon'], TG.nodes[edge[0]]['lat'],
                                                            TG.nodes[t]['lon'], TG.nodes[t]['lat'])
                                         )
                transit_network.add_edge(edge[1], t
                                         , type='flight'
                                         , weight=haversine(TG.nodes[edge[0]]['lon'], TG.nodes[edge[1]]['lat'],
                                                            TG.nodes[t]['lon'], TG.nodes[t]['lat'])
                                         )

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