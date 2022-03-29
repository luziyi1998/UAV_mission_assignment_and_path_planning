import numpy as np
import networkx as nx
import scipy.sparse
from scipy.optimize import linprog
from scipy import sparse
import copy

def MCT(total_node, n_depots, n_packages, cost_matrix, depots_node, transit_node, packages_node, normal_node):
    print("MCT")
    cost_vector = np.array([])
    edge_to_vector_idx = {} # format:[i,j]->idx
    vector_idx_to_edge = {} # format:idx->[i,j]
    out_nbrs = {}
    in_nbrs = {}
    # packages_node = np.array([])
    # for i in range(total_node):
    #     if i not in transit_node and i not in depots_node:
    #         packages_node = np.append(packages_node, i)

    for i in range(total_node):
        if i not in transit_node and i not in normal_node: #exclude transit node and normal_node
            for j in range(total_node):
                if j not in transit_node and i not in normal_node:

                    #exclude self edge and package to package edges
                    if i!=j and (i in depots_node or j in depots_node):
                        edge_cost = cost_matrix[i][j]

                        cost_vector = np.append(cost_vector, edge_cost)
                        edge_to_vector_idx[(i,j)] = len(cost_vector) - 1
                        vector_idx_to_edge[len(cost_vector) - 1] = (i,j)

                        if i not in out_nbrs:
                            out_nbrs[i] = []
                        out_nbrs[i].append(j)

                        if j not in in_nbrs:
                            in_nbrs[j] = []
                        in_nbrs[j].append(i)

    #do  MIP: we need to get vector x that make x * cost_vector is minimum
    
    n_idxs = len(cost_vector)

    # add constraint on package->depot and depot->package edges
    depot_package_mask = sparse.lil_matrix((n_depots*n_packages*2, n_idxs))
    index = -1
    for i in depots_node:
        for j in packages_node:
            index += 1

            if (i, j) in edge_to_vector_idx:
                depot_package_mask[index, edge_to_vector_idx[i, j]] = 1.0

            if (j, i) in edge_to_vector_idx:
                depot_package_mask[index + n_depots*n_packages, edge_to_vector_idx[j, i]] = 1.0


    x_bound = [(0, None) for _ in range(n_idxs)]
    A_ub_constraint1 = depot_package_mask
    b_ub_constraint1 = np.ones(n_depots*n_packages*2) # make sure every depot to package or reverse only ues once

    print("constraints for out-edges and in-edges for package")
    # constraints for out-edges and in-edges for package
    package_out_edge_mask = sparse.lil_matrix((n_packages, n_idxs))
    package_in_edge_mask = sparse.lil_matrix((n_packages, n_idxs))

    nbr_index = -1
    for i in packages_node:
        nbr_index += 1
        for onbr in out_nbrs[i]:
            index = edge_to_vector_idx[i, onbr]
            package_out_edge_mask[nbr_index, index] = 1.

        for inbr in in_nbrs[i]:
            index = edge_to_vector_idx[inbr, i]
            package_in_edge_mask[nbr_index, index] = 1.

    A_eq_constraint1 = package_out_edge_mask
    b_eq_constraint1 = np.ones(n_packages)  # make sure every packages are visited
    A_eq_constraint2 = package_in_edge_mask
    b_eq_constraint2 = np.ones(n_packages)  # make sure every packages are visited

    print("constranints for in-flow and out-flow from depots")
    # constranints for in-flow and out-flow from depots
    depot_out_edge_mask = np.zeros((n_depots, n_idxs))
    depot_in_edge_mask = np.zeros((n_depots, n_idxs))
    print("1")
    depot_nbr_index = -1
    for i in depots_node:
        depot_nbr_index += 1
        for onbr in out_nbrs[i]:
            index = edge_to_vector_idx[i, onbr]
            depot_out_edge_mask[depot_nbr_index, index] = 1.

        for inbr in in_nbrs[i]:
            index = edge_to_vector_idx[inbr, i]
            depot_in_edge_mask[depot_nbr_index, index] = 1.
    print("make sure every depots in and out degree equal")
    # make sure every depots in and out degree equal
    for i in range(n_depots):
        for j in range(n_idxs):
            depot_out_edge_mask[i, j] = depot_out_edge_mask[i, j] - depot_in_edge_mask[i, j]

    A_eq_constraint3 = depot_out_edge_mask
    b_eq_constraint3 = np.zeros(n_depots).reshape(-1, 1)
    print("all all constarints")

    # print("last eq: make sure every package is picked, so the number of edges is 2 * n_packages")
    # #last eq: make sure every package is picked, so the number of edges is 2*n_packages
    # A_eq_constraint4 = np.ones(n_idxs)
    # b_eq_constraint4 = n_packages * 2

    # add all constraints
    A_ub = A_ub_constraint1
    b_ub = b_ub_constraint1

    A_eq = scipy.sparse.vstack([A_eq_constraint1, A_eq_constraint2])
    A_eq = scipy.sparse.vstack([A_eq, A_eq_constraint3])
    # A_eq = scipy.sparse.vstack([A_eq, A_eq_constraint4])
    print("A_eq.shape ", A_eq.shape)
    b_eq = np.hstack([b_eq_constraint1, b_eq_constraint2]).reshape(-1, 1)
    print("b_eq.shape ", b_eq.shape, "b_eq_constraint3.shape ", b_eq_constraint3.shape)
    b_eq = np.vstack([b_eq, b_eq_constraint3]).reshape(-1, 1)
    # b_eq = np.append(b_eq, b_eq_constraint4).reshape(-1, 1)
    print("b_eq.shape ", b_eq.shape)
    print("start optimizer")
    res = linprog(c=cost_vector, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq,
                  bounds=x_bound,)
    print(res)

    x_edges = res.x
    # covert flaot to int
    x_edges = [round(x) for x in x_edges]

    # finally we get the optimal edges that less than x_edges
    edges = [vector_idx_to_edge[i] for (i, val) in enumerate(x_edges) if val>0]


    return (edges, x_edges, vector_idx_to_edge, edge_to_vector_idx, cost_vector)


# 得到MCT结果的强连通分量，
def get_strong_connected_components(total_node, n_depots, n_packages, edges, depot_nodes, packages_node):

    adj_matrix = np.zeros([total_node, total_node])

    for e in edges:
        adj_matrix[e[0], e[1]] = 1

    #we actually dont consider the transit node but we still can get the components of depots and packages
    graph = nx.from_numpy_array(adj_matrix, create_using=nx.DiGraph)

    components = nx.strongly_connected_components(graph)
    depot_components = []
    count = 0
    for comp in components:
        if len(comp) > 1:
            cur_comp_depot = np.array([])
            for idx in comp:
                if idx in depot_nodes:
                    count += 1
                    print("component ", count)
                    cur_comp_depot = np.append(cur_comp_depot, idx)

            depot_components.append(cur_comp_depot)


    return depot_components


# we change the x_edges and merge each components util one
def do_merge_component(x_edges, depot_components, edge_to_vector_idx, n_depots, n_packages, cost_vector):
    merged_components = depot_components

    # stop when there are only one comp
    while len(merged_components) > 1:
        min_to_merge = (0, 0) # i -> j
        min_depots_in_merged_comps = (0, 0)
        cmin = float("inf")

        for comp1_idx in range(len(merged_components) - 1):
            for comp2_idx in range(comp1_idx + 1,len(merged_components)):
                # print("comp1_idx", comp1_idx, "comp2_idx", comp2_idx)
                cmin_for_dep_pair = float("inf")
                depots_to_merge = (0, 0)

                for dep1 in merged_components[comp1_idx]:
                    for dep2 in merged_components[comp2_idx]:
                        # print("dep1", dep1, "dep2", dep2)
                        dep_cost = cost_vector[edge_to_vector_idx[dep1, dep2]] + cost_vector[edge_to_vector_idx[dep2, dep1]]
                        # print("dep_cost", dep_cost)
                        if dep_cost < cmin_for_dep_pair:
                            cmin_for_dep_pair = dep_cost
                            depots_to_merge = (dep1, dep2)

                if cmin_for_dep_pair < cmin:
                    cmin = cmin_for_dep_pair
                    min_to_merge = (comp1_idx, comp2_idx)
                    min_depots_in_merged_comps = depots_to_merge

        # add edges for merged deps
        dep1to2 = edge_to_vector_idx[min_depots_in_merged_comps[0], min_depots_in_merged_comps[1]]
        x_edges[dep1to2] = 1.0
        dep2to1 = edge_to_vector_idx[min_depots_in_merged_comps[1], min_depots_in_merged_comps[0]]
        x_edges[dep2to1] = 1.0

        # create new merged depot components
        new_comps = len(merged_components) - 1
        new_merged_depot_comps = []

        new_merged_depot_comps.append(np.hstack([merged_components[min_to_merge[0]], merged_components[min_to_merge[1]]]))

        for (i, deps) in enumerate(merged_components):
            if i not in min_to_merge:
                new_merged_depot_comps.append(deps)

        merged_components = copy.deepcopy(new_merged_depot_comps)
        print("new_components: ", merged_components)

    return merged_components, x_edges


# get the tour that start from one depot then visit all the package finally end at a depot
def get_multiedge_tour(total_node, depots_node, packages_node, x_edges, vector_idx_to_edges, n_depots, n_packages):
    edge_count = np.zeros(total_node)
    adj_list = [[] for i in range(total_node)]

    for (i, val) in enumerate(x_edges):

        if val > 0:
            idx_pred, idx_succ = vector_idx_to_edges[i]

            # add to the number of out_edges
            edge_count[idx_pred] += val

            # append the neighbor val number of times
            for _ in range(int(val)):
                adj_list[idx_pred].append(idx_succ)

    # now we get the edge_count and adj_list

    curr_path = [] #stack
    circuit = np.array([])

    # start from the first vtx with degree > 1
    first_idx = 0
    for i in depots_node:
        if edge_count[int(i)] > 0:
            first_idx = i

    # push now because will also end at it
    curr_path.append(first_idx)
    curr_v = first_idx

    while len(curr_path) > 0:
        if edge_count[int(curr_v)] > 0:

            curr_path.append(curr_v)
            next_v = adj_list[int(curr_v)].pop()
            edge_count[int(curr_v)] -= 1
            curr_v = next_v

        else:
            circuit = np.append(circuit, curr_v)
            curr_v = curr_path.pop()


    return circuit

# Cut off the extra nodes at both ends, out of the first node connecting the package and the last node connecting the package
def trim_circuit(circuit, depots_node, packages_node):

    first_site_idx = -1
    index = -1
    for i in circuit:
        index += 1
        # print("node ", i, "index: ", index)
        if  i in packages_node:
            first_site_idx = index
            break

    if first_site_idx == -1:
        print("nothing")
        return

    # cut the extra head nodes
    circuit = circuit[first_site_idx-1:]

    last_site_idx = -1
    index = -1
    #Inverted traversal
    for j in range(len(circuit)-1, -1, -1):
        # print("j", j, "node ", circuit[j])
        if circuit[j] in packages_node:
            last_site_idx = j
            break
    #cut the extra rear node: last package_idx is j, we remain the j and j+1 index and delete j+2 and following
    circuit = circuit[:j+2]

    # print("first_package_idx ", first_site_idx, "last_package_idx ", last_site_idx)

    return circuit

# now we distribute the tour over m drones: circuit -> m tours, each tours denote a certain drone's route
def cut_circuit(circuit, n_depots, n_drones, edges_to_vector_idx, cost_vector, packages_node):
    arc_costs = [cost_vector[edges_to_vector_idx[i, j]] for (i, j) in zip(circuit[:len(circuit)-1], circuit[1:])]
    total_cost = sum(arc_costs)
    print("total_cost", total_cost)

    drone_tours = [[] for _ in range(n_drones)]
    idx = 0 #over the circuit
    # print("len circuit", len(circuit))
    for i in range(n_drones):
        this_drone_tour = [circuit[idx]]
        tour_cost = 0

        while tour_cost <= total_cost/n_drones and idx <len(circuit)-1:
            # print("while idx", idx)
            tour_cost += arc_costs[idx] # cost of idx th edge
            idx += 1
            this_drone_tour.append(circuit[idx])

        # every time we get a done's route we make sure the route ends at a depot
        if idx < len(circuit)-1:
            if circuit[idx] in packages_node:
                tour_cost += arc_costs[idx]
                idx += 1
                this_drone_tour.append(circuit[idx])

        drone_tours[i] = this_drone_tour

        if idx == len(circuit)-1:
            break

    return drone_tours


def task_allocation(total_nodes, N_depots, N_packages, N_drones, cost_matrix, depots_node, transit_node, packages_node, normal_node):

    edges, x_edges, vector_idx_to_edges, edges_to_vector_idx, cost_vector = MCT(total_nodes, N_depots, N_packages,
                                                                                cost_matrix, depots_node, transit_node, packages_node, normal_node)

    depot_components = get_strong_connected_components(total_nodes, N_depots, N_packages,
                                                       edges, depots_node, packages_node)

    merged_components, x_edges = do_merge_component(x_edges, depot_components, edges_to_vector_idx,
                                           N_depots, N_packages, cost_vector)

    circuit = get_multiedge_tour(total_nodes, depots_node, packages_node, x_edges,
                                 vector_idx_to_edges, N_depots, N_packages)

    circuit = trim_circuit(circuit, depots_node, packages_node)

    drone_tours = cut_circuit(circuit, N_depots, N_drones, edges_to_vector_idx,
                              cost_vector, packages_node)

    # make sure the sub_tour has no extra depot at the end and start
    empty_tours = []
    for (i, tours) in enumerate(drone_tours):
        if len(tours) > 0:
            drone_tours[i] = trim_circuit(tours, depots_node, packages_node)
        if len(drone_tours[i]) == 0:
            empty_tours.append(i)

    # delete empty drone tours
    final_drone_tours = []
    for (i, tours) in enumerate(drone_tours):
        if i not in empty_tours:
            final_drone_tours.append(tours)

    return final_drone_tours
