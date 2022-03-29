import networkx as nx
import pandas as pd
import matplotlib.pyplot as plt
import copy
import numpy as np
import xml.dom.minidom
# 构建有/无向图对象
Map = nx.DiGraph()



dom = xml.dom.minidom.parse('/Users/luzy6/PycharmProjects/UAV_mission_assignment_and_path_planning/map0.osm')

root = dom.documentElement

from math import radians, cos, sin, asin, sqrt

# get distance between two points
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

# return depot_drone_transit_graph, depots_node, transit_node, packages_node, cost_matrix
def return_Digraph(N_depots, N_transit_edges, N_packages):
    node = root.getElementsByTagName('node')
    print(node[0].nodeName)

    pos_location = {}  # position of all nodes in the graph" to draw the figure
    loc = []

    for i in range(len(node)):
        ID = node[i].getAttribute('id')
        lat = float(node[i].getAttribute('lat'))
        lon = float(node[i].getAttribute('lon'))
        #     print(ID, lat, lon)

        Map.add_node(ID
                     , ID=ID
                     , lat=lat
                     , lon=lon
                     )
        pos_location[ID] = (lon, lat)
        loc.append([lon, lat])


    LOC = pd.DataFrame(loc)
    LOC.describe()

    way_set = root.getElementsByTagName('way')
    # print(way_set[0].nodeName)

    for way in way_set:
        previous_node = start_node_id = way.getElementsByTagName('nd')[0].getAttribute('ref')
        end_node_id = way.getElementsByTagName('nd')[-1].getAttribute('ref')
        lon1 = pos_location[previous_node][0]
        lat1 = pos_location[previous_node][1]
        #     print(previous_node, end_node_id)
        # we pick some node in one way not all

        for sub_node in way.getElementsByTagName('nd'):
            current_node_id = sub_node.getAttribute('ref')
            lon2 = pos_location[current_node_id][0]
            lat2 = pos_location[current_node_id][1]
            if (current_node_id != start_node_id):
                Map.add_edge(previous_node, current_node_id
                             , way_type=0, weight=haversine(lon1, lat1, lon2, lat2)
                             )
                previous_node = current_node_id
        # print(sub_node.getAttribute('ref'))

    G2 = copy.deepcopy(Map)
    # Digraph
    subgraphs = max(nx.strongly_connected_components(Map), key=len)

    # no digraph
    # subgraphs = max(nx.connected_components(Map), key=len)


    for node in Map.nodes:
        if node not in subgraphs:
            G2.remove_node(node)
    # print(len(G2.nodes))
    # for node in Map.nodes:
    #     if ((Map.nodes[node]['lat'] >= 22.3989 or Map.nodes[node]['lat'] <= 22.3879) and node in G2.nodes):
    #         G2.remove_node(node)
    #     if ((Map.nodes[node]['lon'] <= 113.5436 or Map.nodes[node]['lon'] >= 113.5573) and node in G2.nodes):
    #         G2.remove_node(node)

    G3 = nx.to_undirected(G2)

    # create transit graph
    node_name_idx = []
    idx_node_name = {}
    idx = -1
    for node in G3.nodes:
        idx += 1
        node_name_idx.append(node)
        idx_node_name[node] = idx

    TG = nx.Graph()
    for i in range(len(node_name_idx)):
        TG.add_node(i, lon=G3.nodes[node_name_idx[i]]['lon'], lat=G3.nodes[node_name_idx[i]]['lat'])
    # add edges

    for edge in G3.edges:
        TG.add_edge(idx_node_name[edge[0]], idx_node_name[edge[1]], weight=G3.edges[edge[0], edge[1]]['weight'])

    # plt.rcParams['figure.figsize'] = (20, 20)  # 单位是inches
    # nx.draw(G3
    #         , pos=pos_location
    #         #         , with_labels = True
    #         , node_size=0.0001
    #         , node_color='grey'
    #         # '#FF8000' #'#6DCAF2' # '#FF8000' # '#6DCAF2' #  '#B9F1E5'   #'grey'   # '#FFBFBF' #  'k' #nodes_col.values()   #'y'
    #         , width=0.5  # default = 1.0 , Line width of edges
    #         #         , font_size = 4
    #         #         , font_family = 'arial'
    #         , edge_color='grey'  # b, k, m, g,
    #         )
    # fig_name = 'zhuhai_keji6lu.jpg'
    #
    # plt.savefig(fig_name, dpi=200)
    # plt.show()


    # get shortest path of each nodes
    len_path = dict(nx.all_pairs_dijkstra(TG))

    # we get top-N_transit_edges longest point to point then set the nodes are transit stop
    edges_distance = {}
    transit_node = np.array([])
    for (i, j) in TG.edges:
        edges_distance[TG.edges[i, j]['weight']] = [i, j]

    for i, j in zip(range(N_transit_edges), sorted(edges_distance, reverse=True)):
        print("distance ", j)
        transit_node = np.append(transit_node, edges_distance[j])
        #     np.append(transit_node, edges_distance[i][1])
        # print(edges_distance[j])

    transit_node = np.unique(transit_node)  # del the Duplicate Nodes

    # cost_matrix
    cost_matrix = np.zeros((TG.number_of_nodes(), TG.number_of_nodes()))
    for i in range(0, TG.number_of_nodes()):
        #     sub_row_cost = [0 for _ in range(TG.number_of_nodes())]
        for key, val in zip(len_path[i][0].keys(), len_path[i][0].values()):
            cost_matrix[i][key] = val





    # generate random depots
    depots_node = np.array([])
    while len(depots_node) < N_depots:
        rand_node = np.random.randint(0, len(TG.nodes), 1)
        if rand_node not in depots_node and rand_node not in transit_node:
            depots_node = np.append(depots_node, rand_node)

    # packages are randomly picked from the rest node
    packages_node = np.array([])
    while len(packages_node) < N_packages:
        rand_node = np.random.randint(0, len(TG.nodes), 1)
        if rand_node not in depots_node and rand_node not in transit_node:
            packages_node = np.append(packages_node, rand_node)

    # the rest nodes are package node
    # for i in range(len(TG.nodes)):
    #     if i not in depots_node and i not in transit_node:
    #         packages_node = np.append(packages_node, i)


    for i in range(len(TG.nodes)):
        if i in depots_node:
            TG.nodes[i]['type'] = 'depot'
        elif i in transit_node:
            TG.nodes[i]['type'] = 'transit'
        elif i in packages_node:
            TG.nodes[i]['type'] = 'package'
        else:
            TG.nodes[i]['type'] = 'normal'



    return TG, depots_node, transit_node, packages_node, cost_matrix


