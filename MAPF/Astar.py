import copy
import heapq
import numpy as np

class My_PriorityQueue(object):
    def __init__(self):
        self._queue = []
        self._index = 0

    def push(self, item, priority):
        """
        队列由 (priority, index, item) 形式组成
        priority 增加 "-" 号是因为 heappush 默认是最小堆
        index 是为了当两个对象的优先级一致时，按照插入顺序排列
        """
        heapq.heappush(self._queue, (priority, self._index, item))
        self._index += 1

    def pop(self):
        """
        弹出优先级最高的对象
        """
        return heapq.heappop(self._queue)[-1]

    def qsize(self):
        return len(self._queue)

    def empty(self):
        return True if not self._queue else False


class A(object):
    def __init__(self, node_id, time_cost, flight_cost, path):
        self.node_id = node_id
        self.time_cost = time_cost
        self.flight_cost = flight_cost
        self.path = path
    def __repr__(self):
        return "{0} -- {1}".format(self.node_id, self.time_cost)




def astar(transit_network, depot_id, package_id, Max_flight):
    start_node = A(depot_id, 0, 0, [depot_id])
    pq = My_PriorityQueue()
    pq.push(start_node, start_node.time_cost + transit_network.edges[depot_id, package_id]['time'])
#     max_count = 0
    while not pq.empty():
#         if pq.qsize() > max_count:
#             max_count = pq.qsize()
#             print(max_count)
        cur = pq.pop()

        if(cur.node_id == package_id):
            # print("path", cur.path)
            # print("flight_cost",cur.flight_cost)
            return True, cur.path

        for neighbor, w in transit_network[cur.node_id].items():
            if cur.flight_cost + transit_network.edges[cur.node_id, neighbor]['weight'] < Max_flight:

                path = copy.deepcopy(cur.path)
                path.append(neighbor)

                if neighbor == package_id:
                    add_node = A(neighbor, cur.time_cost + transit_network.edges[cur.node_id, neighbor]['time'], cur.flight_cost + transit_network.edges[cur.node_id, neighbor]['weight'], path)
                    pq.push(add_node, add_node.time_cost)
                else:
                    add_node = A(neighbor, cur.time_cost + transit_network.edges[cur.node_id, neighbor]['time'], cur.flight_cost + transit_network.edges[cur.node_id, neighbor]['weight'], path)
                    pq.push(add_node, add_node.time_cost + transit_network.edges[neighbor, package_id]['time'])
    return False, [], [0], -1


def conflict_exit(T_interval, arrive_time, wait_time):
    """
    理论上，如果无人机在车来的前到达即可，也就是到达时间加上等待时间，是无人机最迟到换乘点的限制
    :param T_interval:
    :param arrive_time:
    :param wait_time:
    :return: extra wait time
    """
    if len(T_interval) == 0 or T_interval[0][0] > arrive_time + wait_time or T_interval[-1][1] < arrive_time + wait_time: # 该换乘点无占用  或在占用发生前或发生后该无人机已经可以离开，则不存在冲突
        return -1
    # print(T_interval, arrive_time + wait_time)
    # print(1)

    if len(T_interval) == 1:
        return T_interval[-1][1] - arrive_time - wait_time
    for i in range(len(T_interval)-1):
        if T_interval[i][1] < arrive_time and T_interval[i+1][0] > arrive_time + wait_time: # 该无人机可以夹在换乘点占用中间的空闲进入且离开
            return -1
    # print(T_interval[-1][1], arrive_time, wait_time)
    return T_interval[-1][1] - arrive_time - wait_time


# astar with coflicts avoid
def astar_with_conflict_avoid(transit_network, transit_constrain, transit_edges, depot_id, package_id, Max_flight, start_time=0):
    """
    为了避免冲突，我们为每个无人机规划路径时，会将之前无人机在每个换乘点等待的时间戳保存下来，维护一个关于换乘点限制的字典，表示该换乘点都在哪一个时间戳区间内被占用，
    后续无人机在做路径规划时，需要将换乘点约束考虑进来，即当路径规划到某一个换乘点的时候，应该将时间消耗的权重加上前面无人机所占用的时间，重新进行路径规划
    :param transit_network:
    :param transit_constarin:
    :param depot_id:
    :param package_id:
    :param Max_flight:
    :return: bool, route
    """
    wait_times = 0.2
    max_wait = 20 # max waitting time
    transit_wait = {}
    #更新当前无人机对每个传输点可能的等待时机
    for edge in transit_edges:
        transit_wait[(edge[0], edge[1])] = min(wait_times * np.random.rand() * transit_network.edges[edge]['time'], max_wait)
        transit_wait[(edge[1], edge[0])] = transit_wait[(edge[0], edge[1])]
        # transit_network.edges[(edge[0], edge[1])]['time'] += transit_wait[(edge[0], edge[1])]
    # print(transit_edges)
    start_node = A(depot_id, start_time, 0, [depot_id])
    pq = My_PriorityQueue()
    close_list = set([])
    pq.push(start_node, start_node.time_cost + transit_network.edges[depot_id, package_id]['time'])
    while not pq.empty():
        cur = pq.pop()
        close_list.add(cur.node_id)
        if(cur.node_id == package_id):
            return True, cur.path, transit_wait

        for neighbor, w in transit_network[cur.node_id].items():
            if neighbor not in close_list and cur.flight_cost + transit_network.edges[cur.node_id, neighbor]['weight'] < Max_flight:
                total_wait = 0
                if transit_network.edges[cur.node_id, neighbor]['type'] == 'transit':
                    # 当前无人机到达该站点的时间加上无人机在该站点等待的时间后得到的时间戳，存在冲突，那么，无人机仍然想走原路，则到达时间延长，则等待时间加长
                    extra_wait = conflict_exit(transit_constrain[cur.node_id], cur.time_cost, transit_wait[(cur.node_id, neighbor)])
                    # if cur.node_id == 95.0:
                    #     print("95", cur.time_cost, "neibo", neighbor, "wait", transit_wait[(95, neighbor)], transit_network.edges[95, neighbor]['time'], transit_constrain[cur.node_id])
                    if extra_wait != -1: # exit conflict
                        # print("exit conflict", cur.node_id, neighbor, "extra wait", extra_wait)
                        transit_wait[cur.node_id, neighbor] += extra_wait
                        total_wait += transit_wait[(cur.node_id, neighbor)]

                path = copy.deepcopy(cur.path)
                path.append(neighbor)

                if neighbor == package_id:
                    add_node = A(neighbor, cur.time_cost + transit_network.edges[cur.node_id, neighbor]['time'] + total_wait, cur.flight_cost + transit_network.edges[cur.node_id, neighbor]['weight'], path)
                    pq.push(add_node, add_node.time_cost)
                else:
                    add_node = A(neighbor, cur.time_cost + transit_network.edges[cur.node_id, neighbor]['time'] + total_wait, cur.flight_cost + transit_network.edges[cur.node_id, neighbor]['weight'], path)
                    pq.push(add_node, add_node.time_cost + transit_network.edges[neighbor, package_id]['time'])
    print("false plan by astar")
    return False, [], [0]
