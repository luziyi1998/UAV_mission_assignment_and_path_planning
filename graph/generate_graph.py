import networkx as nx
import pandas as pd
import matplotlib.pyplot as plt
import copy
import numpy as np
import xml.dom.minidom
# 构建有向图对象
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
def return_Digraph(N_depots):
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
    subgraphs = max(nx.strongly_connected_components(Map), key=len)

    for node in Map.nodes:
        if node not in subgraphs:
            G2.remove_node(node)

    for node in Map.nodes:
        if ((Map.nodes[node]['lat'] >= 22.3989 or Map.nodes[node]['lat'] <= 22.3879) and node in G2.nodes):
            G2.remove_node(node)
        if ((Map.nodes[node]['lon'] <= 113.5436 or Map.nodes[node]['lon'] >= 113.5573) and node in G2.nodes):
            G2.remove_node(node)

    G3 = nx.to_undirected(G2)

    plt.rcParams['figure.figsize'] = (20, 20)  # 单位是inches
    nx.draw(G3
            , pos=pos_location
            #         , with_labels = True
            , node_size=0.0001
            , node_color='grey'
            # '#FF8000' #'#6DCAF2' # '#FF8000' # '#6DCAF2' #  '#B9F1E5'   #'grey'   # '#FFBFBF' #  'k' #nodes_col.values()   #'y'
            , width=0.5  # default = 1.0 , Line width of edges
            #         , font_size = 4
            #         , font_family = 'arial'
            , edge_color='grey'  # b, k, m, g,
            )
    fig_name = 'zhuhai_keji6lu.jpg'

    plt.savefig(fig_name, dpi=200)
    plt.show()

    # G2 is weighted Digraph
    matrix_G2 = nx.to_numpy_array(G2)
    # we turn to weighted noDi graph
    G4 = nx.from_numpy_array(matrix_G2)

    # get shortest path of each nodes
    len_path = dict(nx.all_pairs_dijkstra(G4))

    # we get top-10 longest point to point then set the nodes are transit stop
    edges_distance = {}
    transit_node = np.array([])
    for (i, j) in G4.edges:
        edges_distance[G4.edges[i, j]['weight']] = [i, j]


    for i, j in zip(range(10), sorted(edges_distance, reverse=True)):
        transit_node = np.append(transit_node, edges_distance[j])
        #     np.append(transit_node, edges_distance[i][1])
        # print(edges_distance[j])

    transit_node = np.unique(transit_node)  # del the Duplicate Nodes

    # cost_matrix
    cost_matrix = np.zeros((G4.number_of_nodes(), G4.number_of_nodes()))
    for i in range(0, G4.number_of_nodes()):
        #     sub_row_cost = [0 for _ in range(G4.number_of_nodes())]
        for key, val in zip(len_path[i][0].keys(), len_path[i][0].values()):
            cost_matrix[i][key] = val

    # final graph weighted Digraph
    depot_drone_transit_graph = nx.from_numpy_array(cost_matrix, create_using=nx.DiGraph)



    # generate random depots
    depots_node = np.array([])
    while len(depots_node) < N_depots:
        rand_node = np.random.randint(0, len(G4.nodes), 1)
        if rand_node not in depots_node and rand_node not in transit_node:
            depots_node = np.append(depots_node, rand_node)

    # the rest node are package sites
    packages_node = np.array([])
    for i in range(len(G4.nodes)):
        if i not in depots_node and i not in transit_node:
            packages_node = np.append(packages_node, i)






    return depot_drone_transit_graph, depots_node, transit_node, packages_node, cost_matrix


