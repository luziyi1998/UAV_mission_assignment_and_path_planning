Efficient Large-Scale Multi-Drone Delivery Using Transit Networks

Shushman Choudhury, Kiril Solovey, Mykel J. Kochenderfer, and Marco Pavone

**DEPOT**

**DELIVERY**

**TRANSFER**

**RIDE**

**TRANSIT**

**FLIGHT**

*Abstract*—We consider the problem of controlling a large fleet of drones to deliver packages simultaneously across broad urban areas. To conserve energy, drones hop between public transit vehicles (e.g., buses and trams). We design a com- prehensive algorithmic framework that strives to minimize the maximum time to complete any delivery. We address the multifaceted complexity of the problem through a two-layer ap- proach. First, the upper layer assigns drones to package delivery sequences with a near-optimal polynomial-time task allocation algorithm. Then, the lower layer executes the allocation by periodically routing the fleet over the transit network while employing efficient bounded-suboptimal multi-agent pathfind- ing techniques tailored to our setting. Experiments demonstrate the efficiency of our approach on settings with up to 200 drones, 5000 packages, and transit networks with up to 8000 stops in San Francisco and Washington DC. Our results show that the framework computes solutions typically within a few seconds on commodity hardware, and that drones travel up to 360% of their flight range with public transit.

I. INTRODUCTION

Rapidly growing e-commerce demands have greatly strained dense urban communities by increasing delivery truck traffic and slowing operations and impacting travel times for public and private vehicles [24, 26]. Further congestion is being induced by newer services relying on ride-sharing vehicles. There is a clear need to redesign the current method of package distribution in cities [29]. The agility and aerial reach of drones, the flexibility and ease of establishing drone networks, and recent advances in drone capabilities make them highly promising for logistics networks [28]. However, drones have limited travel range and carrying capacity [14, 45]. On the other hand, ground-based transit networks have less flexibility but greater coverage and throughput. By combining the strengths of both, we can achieve significant commercial benefits and social impact (e.g., reducing ground congestion and delivering essentials).

We address the problem of operating a large number of drones to deliver multiple packages simultaneously in an area. The drones can use one or more vehicles in a public- transit network as modes of transportation, thereby saving their limited battery energy stored onboard and increasing their effective travel range. We are required to decide which deliveries each drone should make and in what order, which modes of transit to use, and for what duration (Figure 1).

Our approach must contend with the multiple significant challenges of our problem. It must plan over large time- dependent transit networks, while accounting for energy constraints that limit the drones’ flight ranges. It must avoid inter-drone conflicts, such as where more than one drone attempts to board the same vehicle at the same time, or when the maximum carrying capacity of a vehicle is exceeded.

The authors are with Stanford University, CA, USA.

Fig. 1: Our multi-drone delivery framework plans for drones to piggyback on public transit vehicles while delivering packages from depots to the requested locations. Our framework is scalable and efficient, and minimizes the amount of time for any individual delivery.

We seek not just feasible multi-agent plans but high-quality solutions in terms of a cumulative objective over all drones, the makespan, i.e., the maximum individual delivery time for any drone. Additionally, our approach must also solve the task allocation problem of determining which drones deliver which packages, and from which distribution centers.

*A. Related work*

Some individual aspects of our problem have already been studied. Choudhury et al. [10] investigated the single-agent setting of controlling a drone to use multiple modes of transit en route to its destination. Recent work has considered pairing a drone with a delivery truck, which does not exploit public transit [2, 18, 36]. The multi-agent issues of task allocation and inter-agent conflicts were not addressed either. Our problem is closely related to routing a fleet of autonomous vehicles providing mobility-on-demand ser- vices [27, 44, 47]. Specifically, the task is to compute routes for the vehicles (both customer-carrying and empty) so that travel demand is fulfilled and operational cost is minimized. In particular, recent works study the combination of such service with public transit, where passengers can use several modes of transportation in the same trip [40, 50]. However, such works abstract away inter-agent constraints or dynamics and are not suited for autonomous pathfinding. The task- allocation setting we consider in our problem can be viewed as an instance of the vehicle routing problem [8, 37, 46], variants of which are typically solved by mixed integer linear programming (MILP) formulations that scale poorly, or by heuristics without optimality guarantees.

We must contend with the challenges of planning for mul- tiple agents. Accordingly, the second layer of our approach is a multi-agent path finding (MAPF) problem [16, 48]. Since the drones are on the same team, we have a cen- tralized or cooperative pathfinding setting [42]. The MAPF problem is NP-hard to solve optimally [49]. A number

arXiv:1909.11840v5 [cs.RO] 5 Jan 2021

of efficient solvers have been developed that work well in practice [17]. The MAPF formulation and algorithms have been extended to several relevant scenarios such as lifelong pickup-and-delivery [33] and joint task assignment and pathfinding [25, 32], though for different task settings and constraints than ours. Also, a MAPF formulation was applied for UAV traffic management in cities [23]. However, none of the approaches considered pathfinding over large time-dependent transit networks. We use models, algorithms and techniques from transportation planning [6, 13, 39].

*B. Statement of contributions*

We present a comprehensive algorithmic framework for large-scale multi-drone delivery in synergy with a ground transit network. Our approach strives to minimize the max- imum time to complete any delivery. We decompose the highly challenging problem and solve it stage-wise with a two-layer approach. First, the upper layer assigns drones to package-delivery sequences with a task allocation algorithm. Then, the lower layer executes the allocation by periodically routing the fleet over the transit network.

Algorithmically, we develop a new delivery sequence allo- cation method for the upper layer that obtains a near-optimal solution in polynomial runtime. For the lower layer, we ex- tend techniques for multi-agent path finding that account for time-dependent transit networks and agent energy constraints to perform multi-drone routing. Experimentally, we present results supporting the efficiency of our approach on settings with up to 200 drones, 5000 packages, and transit networks of up to 8000 stops in San Francisco and the Washington DC area. Our framework can compute solutions within a few seconds (up to 2 minutes for the largest settings) on commodity hardware, and in our problem scenarios, drones can travel up to 450% of their flight range using transit.

The following is the paper structure. We present an overall description of the two-layer approach in Section II, and then elaborate on each layer in Sections III and IV. We present experimental results on simulations in Section V, and conclude the paper with Section VI.

II. METHODOLOGY

We provide a high-level description of our formulation and approach to illustrate the various interacting components.

*A. Problem Formulation*

We are operating a centralized homogeneous fleet of m drones within a city-scale domain. There are l prod- uct *depots* with known geographic locations, denoted by VD := {d1,...,dl} ⊂ R2. The depots are both product dispatch centers and drone-charging stations. At the start of a large time interval (e.g., a day), a batch of delivery request locations for k different *packages*, denoted VP := {p1,...,pk} ⊂ R2, is received (we assume that k ≫ m). We assume that any package can be dispatched from any depot; our framework exploits this property to optimize the solution quality in terms of *makespan*, i.e., the maximum execution time for any delivery. In Section III, we mention how our approach can accommodate dispatch constraints.

The drones carry packages from depots to delivery loca- tions. They can extend their effective travel range by using public transit vehicles in the area, which remain unaffected by the drones’ actions. Our problem is to route drones to deliver all packages while minimizing makespan. A drone route consists of its current location and the sequence of depot and package locations to visit with a combination of flying and riding on transit. We characterize the drones’ limited energy as a maximum flight distance constraint. A feasible solution must satisfy inter-drone constraints such as collision avoidance and transit vehicle capacity limits.

Finally, we make some assumptions for our setting: a drone carries one package at a time, which is reason- able given state-of-the-art drone payloads [14]; drones are recharged upon visiting a depot in negligible time (e.g., a bat- tery replacement); depots have unlimited drone capacity; the transit network is deterministic with respect to locations and vehicle travel times (we mention uncertainty in Section VI). We do account for the time-varying nature of the transit.

*B. Approach overview*

In principle, we could frame the entire problem as a mixed integer linear program (MILP). However, for real-world problems (hundreds of drones; thousands of packages; large transit networks), even state-of-the-art MILP approaches are unlikely to scale. Moreover, even a simpler problem that ignores the interaction constraints is an instance of the noto- riously challenging multi-depot vehicle routing problem [37]. Thus, we decouple the problem into two distinct subproblems that we solve stage-wise in layers.

The upper layer performs *task allocation* to determine which packages are delivered by which drone and in what order. It takes as input the known depot and package lo- cations, and an estimate of the drone travel time between every pair of locations. It then solves a threefold allocation to minimize delivery makespan and assigns to each package (i) the dispatch depot and (ii) the delivery drone, and to each drone (iii) the order of package deliveries. To this end, we develop an efficient polynomial-time task-allocation algorithm that achieves a near-optimal makespan.

The lower layer performs *route planning* for the drone fleet to execute the allocated delivery tasks. It generates detailed routes of drone locations in time and space and the transit vehicles used, while accounting for the time- varying transit network. It also ensures that (i) simultaneous transit boarding by multiple drones is avoided, (ii) no transit vehicle exceeds its drone-carrying capacity, and (iii) drone (battery) energy constraints are respected. We efficiently handle individual and inter-drone constraints by framing the routing problem as an extension of multi-agent path finding (MAPF) to transit networks. We adapt a scalable, bounded sub-optimal variant of a highly effective MAPF solver called Conflict-Based Search (CBS) [41] to solve the one-delivery- per-drone problem. Finally, we obtain routes for the sequence of deliveries in a receding-horizon fashion by replanning for the next task once a drone completes its current one.

Decomposition-based stage-wise optimization approaches typically have an approximation gap compared to the optimal solution of the full problem. For us, this gap manifests in

the surrogate cost estimate we use for the drone’s travel time in the task-allocation layer (instead of jointly solving for allocation and multi-agent routing over transit networks, which is not feasible at scale). The better the surrogate, the more *coupled* the layers are, i.e., the better is the solution of the first stage for the second one. Such surrogates have a tradeoff between efficiency and approximation quality. An easy-to-compute travel time surrogate, for instance, is the drone’s direct flight time between two locations (ignoring transit). However, that can be poor-quality when the drone requires transit for an out-of-range target. We use a surrogate that actually accounts for the transit network, at the expense of some modest preprocessing. We defer details to Appendix III, but the idea is to precompute the pairwise shortest travel times between locations spread around the city, over a representative snapshot of the transit network.

III. TASK ASSIGNMENT AND PACKAGE ALLOCATION

We leverage our problem’s structure to design a new algorithm called MERGESPLITTOURS for the task-allocation layer, which guarantees a near-optimal solution in polyno- mial time. The goal of this layer is to (i) distribute the set of packages VP among m agents, (ii) assign each package destinationp∈VP toadepotd∈VD,and(iii)assigndrones to a sequence of depot pickups and package deliveries. The objective is to minimize the maximum travel time among all agents over all three of the above components.

Our problem can be cast as a special version of the m traveling salesman problem [7], which we call the m *minimal visiting paths problem* (m-MVP). We seek a set of m paths such that the makespan, i.e., the maximum travel time for any path, is minimized. We only need *paths* that start and end at (the same or different) depots, not tours. Our formulation is a special case of the the *asymmetric* variant, for a *directed* underlying graph, which is NP-hard even for m = 1 on general graphs [4] (although it is not known whether the specific instance of our problem is NP-hard as well). Moreover, the current best polynomial- time approximation [4] yields the fairly large approximation factor O(logn/loglogn), for a graph with n vertices. An additional challenge is the inability to assume the triangle inequality on our objective of travel times.

A key element of m-MVP is the *allocation graph* GA = (VA, EA), with vertex set VA = VD ∪ VP . Each directed edge (u,v) ∈ EA is weighted according to an estimated travel time cuv from the location of u to that of v in the city. For every d ∈ VD,p ∈ VP we exclude the edge (d,p) from EA if it is impossible to reach p from d while using at most 1/2 of the flight range allowed (similarly for (p, d) edges). As we flagged in Section II-B, any dispatch constraints are modeled by excluding edges from the corresponding depot. We are now ready for the full definition of m-MVP:

Definition 1. Given allocation graph GA, the m minimal

visiting paths problem (m-MVP) consists of finding m paths

P∗ on G , such that (1) each path P∗ starts at some depot 1:mA i′

d ∈ VD and terminates at the same or different d ∈ VD, (2) exactly one path visits each package p ∈ VP , and (3) the maximum travel time of any of the paths is minimized.

Algorithm 1: MERGESPLITTOURS(GA)

Solve MCT(GA) to get t tours T := {T1,...,Tt}; while |T| > 1 do

Pick distinct tours T , T ′ ∈ T and depots d∈T,d′ ∈T′ that minimize cdd′ +cd′d;

MergeT,T′ byadding(d,d′),(d′,d)edges; Split final tour T into m paths P1,...,Pm, where

LENGTH(Pi) is proportional to LENGTH(T)/m for

each i (similar to [19]);
 Extend each Pi to ensure it begins and ends at a

depot;
 return P1,...,Pm;

TABLE I: An integer programming formulation of the MCT problem.

Given allocation graph GA = (VA, EA), with VA = VD ∪ VP , minimize 􏰊 xuv · cuv (1)

(u,v)∈EA

subject to
 xuv ∈{0,1}, ∀(u,v)∈EA,u∈VP ∨v∈VP, (2)

xuv ∈ N􏰒0, ∀(d,d′) ∈ EA,d,d′ ∈ VD, (3)

􏰊 xdp = d∈N+ (p)

􏰊 xpd =1, ∀p∈VP, (4) d∈N− (p)

􏰊 xvd− 􏰊 xdv=0, ∀d∈VD. (5) v∈N+ (d) v∈N− (d)

where N+(v), N−(v) denote the in and out going neighbors of v ∈ VA.

Let O P T
 maxi∈[m] LENGTH(Pi∗), where LENGTH(·) denotes the total travel time along a given path or tour. We make three obser- vations. First, if a path contains the sub-path (d, p), (p, d′ ), for some d, d′ ∈ VD, p ∈ VP , then p should be dispatched from depot d and the drone delivering p will return to d′ after delivery. Second, a package p being found in Pi∗ indicates that drone i ∈ [m] should deliver it. Third, Pi∗ fully characterizes the order of packages delivered by drone i.

*A. Algorithm Overview*

be the

optimal

makespan,

We present our MERGESPLITTOURS algorithm for solv- ing m-MVP (Algorithm 1); see a detailed description in Appendix I. A key step is generating an initial set of tours T by solving the minimal-connecting tours (MCT) problem (see Table I), which attempts to connect packages to depots within tours to minimize the total edge weight in eq. (1). The constraint in eq. (4) is that each package is connected to precisely one incoming and one outgoing edge from and to depots respectively. The final constraint in eq. (5) enforces inflow and outflow equality for every depot. Edges connecting packages can be used at most once, whereas edges connecting depots can be used multiple times. The solution to MCT is the assignment {xuv }(u,v)∈EA , i.e., which edges of GA are used and how many times. This assignment implicitly represents the desired collection of the tours T1, . . . , Tt; see Appendix I.

*B. Theoretical Guarantees*

All proofs from this secion are in Appendix I. The following theorem states that MERGESPLITTOURS is correct

i.e., O P T :=

| **CAPACITY =2** 							 					 					 						 							**BOARDING CONFLICT** | **CAPACITY =2** 							 					 					 						 							**CONFLICT RESOLVED** | **CAPACITY CONFLICT** 								**CAPACITY =1** | **CAPACITY =1** 							 					 					 						 							**CONFLICT RESOLVED** |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
|                                                              |                                                              |                                                              |                                                              |

(a) (b) (c) (d)

Fig. 2: In our formulation of multi-agent path finding with transit networks, conflicts arise from the violation of shared inter-drone constraints: (a) boarding conflicts between two or more drones and (c) capacity conflicts between more drones than the transit vehicle can accommodate. The modified paths after resolving the corresponding conflicts are depicted in (b) and (d), respectively.

and that its makespan is close to optimal.

Theorem 1. *Suppose* GA *is strongly connected and the subgraph* GA(VD) *induced by the vertices* VD *is a directed clique. Let* P1, . . . , Pm *be the output of* MERGESPLIT- TOURS*.Then,everypackage*p∈VP *iscontainedinexactly one path* Pi*, and every* Pi *starts and ends at a depot. Moreover,* maxi∈[m] LENGTH(Pi) 􏰑 OPT + α + β *holds,*

where α := max cdd′ +cd′d , β := max cdp+cpd′ . d,d′ ∈VD d,d′ ∈VD ,p∈VP

The key idea is that the total cost of the tours induced

by the solution to MCT cannot exceed the total length of

{P∗,...,P∗ }. The MCT solution is then adapted to m paths 1m

with an additional overhead of α + β per path. When m ≪ |VP | (typically the case), α and β are small compared to OPT, making the bound tight. For instance, in our randomly- generated scenarios in Section V-A, for m = 5 and k = 200, the approximation ratio maxi∈[m] LENGTH(Pi)/OPT = 1.09, and for m = 10, k = 500, the factor is 1.06.

The computational bottleneck of the algorithm is MCT, while the other components can clearly be implemented polynomially in the input size. However, it suffices to solve a relaxed version of MCT to obtain the same integral solution.

Lemma 1. *The optimal solution to the fractional relaxation ofMCT,inwhich*xuv ∈[0,1]*forall*u∈VP ∨v∈VP*,and* xuv ∈ R+ *otherwise, yields the integer optimal solution.*

The lemma follows from casting MCT as the minimum- cost circulation problem, for which the constraint matrix is totally unimodular [3]. Therefore, MERGESPLITTOURS can be implemented in polynomial time.

IV. MULTI-AGENT PATH FINDING

For each drone i ∈ [m], the allocation layer yields a sequence of delivery tasks d1 p1 . . . pl dl+1 . Each delivery sequence has one or more subsequences of dpd′. The route- planning layer treats each dpd′ subsequence *as an individual drone task*, i.e., leaving with the package from depot d, carrying it to package location p and returning to the (same or different) depot d′, without exceeding the energy capacity. We seek an efficient and scalable method to obtain high- quality (with respect to travel time) feasible paths, while using transit options to extend range, for m different drone dpd′ tasks simultaneously. The full set of delivery sequences can be satisfied by replanning when a drone finishes its current task and begins a new one; we discuss and com- pare two replanning strategies in Appendix IV. Thus, we

formulate the problem of multi-drone routing to satisfy a set of delivery sequences as receding-horizon multi-agent path finding (MAPF) over transit networks. In this section, we describe the graph representation of our problem and present an efficient bounded sub-optimal algorithm.

*A. MAPF with Transit Networks (MAPF-TN)*

The problem of Multi-Agent Path Finding with Transit Networks (MAPF-TN) is the extension of standard MAPF to where agents can use one or more modes of transit in addition to moving. The incorporation of transit networks introduces additional challenges and underlying structure. The input to MAPF-TN is the set of m tasks (di , pi , d′i )i=1:m and the directed operation graph GO = (VO , EO ). In Sec- tion III, the allocation graph GA only considered depots and packages, and edges between them. Here, GO also includes transit vertices, VT N = 􏰓τ ∈T Rτ , where T is the set of trips, and each trip Rτ = {(s1, t1) . . .} is a sequence of time- stamped stop locations (a given stop location may appear as several different nodes with distinct time-stamps). Similarly, we also use time-expanded versions of VD and VP [39].

The edges are defined as follows: An edge e = (u, v) ∈ E is a *transit edge* if u,v ∈ VTN and are consecutive stops on the same trip Rt. Any other edge is a *flight edge*. An edge is *time-constrained* if v ∈ VT N and *time-unconstrained* otherwise. Every edge has three attributes: traversal time T , energy expended N, and capacity C. Since each vertex is associated with a location, ∥v − u∥ denotes the distance between them for a suitable metric. MAPF typically abstracts away agent dynamics; we have a simple model where drones move at constant speed σ, and distance flown represents energy expended. Due to high graph density (drones can fly point-to-point between many stops), we do not explicitly enumerate edges but generate them on-the-fly during search.

We now define the three attributes for EO. For time- constrained edges, T(e) = v.t−u.t is the difference between corresponding time-stamps (if u ∈ VD ∪ VP , u.t is the chosen departure time), and for time-unconstrained edges, T (e) = ∥v − u∥/σ is the time of direct flight. For flight edges, N(e) = ∥v−u∥ (flight distance), and for transit edges, N(e) = 0. For transit edges, C(e) is bounded by the capacity of the vehicle, while for flight edges, C(e) = ∞. Here, we assume that time-unconstrained flight in open space can be accommodated (thorougly examined in [23]).

We now describe the remaining relevant MAPF-TN de- tails. An individual path πi for drone i from di through pi

to d′ is feasible if the energy constraint 􏰉 N(e) 􏰑 N ̄ is i  ̄ e∈πi

satisfied, where N is the drone’s maximum flight distance. In addition, the drone should be able to traverse the distance of a time-constrained flight edge in time, i.e., σ × (v.t − u.t) > ∥v − u∥. For simplicity, we abstract away energy expenditure due to hovering in place by flying the drone at reduced speed to reach the transit just in time. Thus, the constraint N ̄ is only on the traversed distance. The cost of an individual path

TABLE II: The mean computation time for MERGESPLITTOURS in seconds, over 100 different trials for each setting. MERGESPLITTOURS is polynomial in input size and highly scalable. Here, k = |VP | is the number of package deliveries and l = |VD| is the number of depots. For all instances that took longer than 60 s, only one trial was used.

is the total traversal time, T (π ) = 􏰉 T (e). A feasible i

k l=2 50 0.004

100 0.012 200 0.038 500 0.201 1000 0.781 5000 23.97

l=5 l = 10 0.016 0.057

0.050 0.195 0.173 0.699 1.025 4.384 4.109 24.30 238.9 1031

l=20 l = 30 0.248 0.658

0.807 2.117 2.968 8.409 25.04 85.01 122.3 322.5

2192 5275

􏰓 e∈πi
 i=1:m πi is a set of m individually feasible

solution Π =
 paths that does not violate any of the following two *shared constraints* (see Figure 2): (i) *Boarding constraint*, i.e., no two drones may board the same vehicle at the same stop; (ii) *Capacity constraint*, i.e., a transit edge e may not be used by more than C(e) drones. As with the allocation layer, the global objective for MAPF-TN is to minimize the solution makespan, argminΠ maxπ∈Π T(π), i.e., minimize the worst individual completion time.

*B. Conflict-Based Search for MAPF-TN*

To tackle MAPF-TN, we modify the Conflict-Based Search (CBS) algorithm [41]. The multi-agent level of CBS identifies shared constraints and imposes corresponding path constraints on the single-agent level. The single-agent level computes optimal individual paths that respect all constraints. If individual paths conflict (i.e., violate a shared constraint), the multi-agent level adds further constraints to resolve the conflict, and invokes the single-agent level again, for the con- flicting agents. In MAPF-TN, conflicts arise from boarding and capacity constraints. CBS obtains optimal multi-agent solutions without having to run (potentially significantly expensive) multi-agent searches. However, its performance can degrade heavily with many conflicts in which constraints are violated. Figure 2 illustrates the generation and resolution of conflicts in our MAPF-TN problem.

For scalability, we use a bounded sub-optimal variant of CBS called *Enhanced CBS* (ECBS), which achieves orders of magnitude speedups over CBS [5]. ECBS uses bounded sub-optimal *Focal Search* [38] at both levels, instead of best- first A* [22]. Focal search allows using an inadmissible heuristic that prioritizes efficiency. We now describe a crucial modification to ECBS required for MAPF-TN.

Focal Weight-constrained Search: Unlike typical MAPF, the low-level graph search in MAPF-TN has a path-wide constraint (traversal distance) *in addition to the objective function* of traversal time. For the shortest path problems on graphs, adding a path-wide constraint makes it NP-hard [20]. Several algorithms for constrained search require an explicit enumeration of the edges [9, 15]. We extend the A* for MultiConstraint Shortest Path (A*-MCSP) algorithm [31] (suitable for our implicit graph) to focal search (called Focal- MCSP). Focal-MCSP uses admissible heuristics on both objective and constraint and maintains only non-dominated paths to intermediate nodes. This extensive book-keeping requires a careful implementation for efficiency.

Focal-MCSP inherits the properties of A*-MCSP and Fo- cal Search; therefore, it yields a bounded-suboptimal feasible path to the target. Accordingly, ECBS with Focal-MCSP yields a bounded sub-optimal solution to MAPF-TN. The

result follows from the analysis of ECBS [5]. Also, note that a dpd′ path requires a bounded sub-optimal path from d to p and another from p to d′, such that their concatenation is feasible. Since this is even more complicated, in practice, we run Focal-MCSP twice (from d to p and p to d′) with half the energy constraint each time and concatenate the paths, guaranteeing feasibility. In Appendix II-B we discuss other required modifications to standard MAPF and important speedup techniques that nonetheless retain the bounded sub- optimality of Enhanced CBS for our MAPF-TN formulation.

V. EXPERIMENTS AND RESULTS

We implemented our approach using the Julia language and tested it on a machine with a 6-core 3.7GHz 16GiB RAM CPU.1 For very large combinatorial optimization prob- lems, solution quality and algorithm efficiency are of interest. We have already shown that the upper and lower layers are near-optimal and bounded-suboptimal respectively in terms of solution quality, i.e., makespan. Therefore, for evaluation we focus on their efficiency and scalability to large real- world settings. We do not attempt to baseline against a MILP approach for the full problem; we estimate that a typical setting of interest will have on the order of 107 variables in a MILP formulation, besides exponential constraints.

We ran simulations with two large-scale public transit networks in San Francisco (SFMTA) and the Washington Metropolitan Area (WMATA). We used the open-source General Transit Feed Specification data [1] for each network. We considered only the bus network (by far the most exten- sive), but our formulation can accommodate multiple modes. We defined a geographical bounding box in each case, of area 150 km2 for SFMTA and 400 km2 for WMATA (illus- trated in Appendix IV), within which depots and package locations were randomly generated. For the transit network, we considered all bus trips that operate within the bounding box. The *size* of the time-expanded network, |VTN|, is the total number of stops made by all trips; |VT N | = 4192 for SFMTA and |VT N | = 7608 for WMATA (recall that edges are implicit, so |ETN| varies between queries, but the full graph GO can be dense). The drone’s flight range constraint is set (conservatively) to 7 km and average speed to 25 kph, based on the DJI Mavic 2 specifications [14]. In this section, we evaluate the two main components — the task allocation and multi-agent path finding layers. In Appendix IV we compare the performance of two replanning strategies for

1The code for our work is available at https://github.com/sisl/ MultiAgentAllocationTransit.jl.

TABLE III: (All times are in seconds) An extensive analysis of the MAPF-TN layer, on 100 trials for each setting of depots and agents (and 30 trials for 5 depots and 50 agents). Each trial uses different randomly generated depots and delivery locations. The integer carrying capacity of any transit edge C(e) was randomly chosen from {3, 4, 5} (single and double-buses). The sub-optimality factor for ECBS was 1.1. For settings with m/l = 10, a number of trials timed out (over 180 s) and were discarded.

San Francisco 􏰀|VT N | = 4192 ; Area 150 km2 􏰁

Washington DC 􏰀|VT N | = 7608; Area 400 km2 􏰁

{Depots, Agents} {l, m}

{5, 10} {5, 20} {5, 50} {10, 20} {10, 50} {10, 100} {20, 50} {20, 100} {20, 200}

{Median, Avg} Plan Time

{0.61, 1.17} {1.39, 2.13} {2.13, 3.89} {0.41, 1.02} {0.73, 1.46} {2.09, 7.29} {0.17, 0.46} {0.49, 1.05} {0.89, 2.10}

{Avg, Max} Range Ext.

{1.53, 3.41} {1.61, 2.66} {1.64, 2.48} {1.24, 2.35} {1.38, 3.58} {1.43, 2.16} {0.98, 1.69} {1.06, 1.79} {1.13, 2.31}

{Avg, Max} Transit Used

{2.93, 6} {3.48, 6} {4.2, 6} {2.31, 6} {2.94, 5} {3.67, 8} {1.09, 7} {1.61, 9} {2.23, 6}

Avg Soln. Makespan

2554.7 2886.8 3380.9 2091.6 2504.7 2971.8 1273.6 1642.4 1898.5

{Median, Avg} Plan Time

{3.91, 5.65} {9.01, 13.1} {19.1, 28.9} {1.61, 4.67} {4.77, 15.8} {18.1, 26.2} {0.73, 1.92} {2.45, 5.24} {4.68, 10.5}

{Avg, Max} Range Ext.

{1.66, 3.08} {1.79, 3.21} {2.07, 3.21} {1.37, 3.12} {1.72, 3.03} {1.86, 3.18} {1.29, 2.88} {1.48, 2.67} {1.61, 2.87}

{Avg, Max} Transit Used

{3.18, 7}

{3.57, 8} {4.44, 7} {2.57, 7} {3.53, 7} {4.25, 8} {2.23, 7} {3.19, 6} {3.58, 7}

Avg Soln. Makespan

5167.3 5384.5 6140.2 4017.2 5312.3 5623.9 3571.8 4304.5 5085.6

when a drone finishes its current delivery, and two surrogate travel time estimates for coupling the layers.

*A. Task Allocation*

The scale of the allocation problem is determined by the number of depots and packages, i.e., l + k. The runtimes for MERGESPLITTOURS with varying l,k over SFMTA are displayed in Table II. The roughly quadratic increase in runtimes along a specific row or column demonstrate that our provably near-optimal MERGESPLITTOURS algorithm is indeed polynomial in the size of the input. Even for up to 5000 deliveries, the absolute runtimes are quite reasonable. We do not compare with naive MILP even for allocation, as the number of variables would exceed (l · k)2, in addition to the expensive subtour elimination constraints [34].

*B. MAPF with Transit Networks (MAPF-TN)*

Solving multi-agent path finding optimally is NP- hard [49]. Previous research has benchmarked CBS variants and shown that Enhanced CBS is most effective [5, 11]. Therefore, we focus on extensively evaluating our own approach rather than redundant baselining. Table III quan- tifies several aspects of the MAPF-TN layer with varying numbers of depots (l) and agents (m), the two most tunable parameters. Before each trial, we run the allocation layer and collect m dpd′ tasks, one for each agent. We then run the MAPF-TN solver on this set of tasks to compute a solution.

We discuss broad observations here and provide a detailed analysis in Appendix IV. The results are very promising; our approach scales to large numbers of agents (200) and large transit networks (nearly 8000 vertices); the highest average makespan for the true delivery time is less than an hour (3380.9 s) for SFMTA and 2 hours (6140.2 s) for WMATA; drones are using up to 9 transit options per route to extend their range by up to 3.6x. As we anticipated, conflict reso- lution is a major bottleneck of MAPF-TN. A *higher ratio of agents to depots* increases conflicts due to shared transit, thereby increasing plan time (compare {5, 20} to {10, 20}). *A higher number of depots* puts more deliveries within flight range of a depot, reducing conflicts, makespan, and the need for transit usage and range extension (compare {10, 50} to {20, 50}). Plan times are much higher for WMATA due to a larger area and a larger and less uniformly distributed bus network, leading to higher single-agent search times and

more multi-agent conflicts. Trials taking more than 3 minutes were discarded; two pathological cases with SFMATA and WMATA (each with {l = 10,m = 100}) took nearly 4 and 8 minutes, due to 30 and 10 conflicts respectively. In any case, a deployed system would have better compute and parallelized implementations. Finally, note that the running times reported here are actually pessimistic, because we con- sider cases where drones are released simultaneously from the depots, which increases conflicts. However, a gradual release by executing the MAPF solver over a longer horizon (as we discuss in Appendix IV-B) results in fewer conflicts, allowing us to cope with an even larger drone fleet.

VI. CONCLUSION AND FUTURE WORK

We designed a comprehensive algorithmic framework for solving the highly challenging problem of multi-drone pack- age delivery with routing over transit networks. Our two- layer approach is efficient and highly scalable to large prob- lem settings and obtains high-quality solutions that satisfy the many system constraints. We ran extensive simulations with two real-world transit networks that demonstrated the widespread applicability of our framework and how using ground transit judiciously allows drones to significantly extend their effective range.

A key future direction is to perform case studies that estimate the operational cost of our framework, evaluate its impact on road congestion, and consider potential exter- nalities like noise pollution and disparate impact on urban communities. Another direction is to extend our model to overcome its limitations: delays and uncertainty in the travel pattern of transit vehicles [35] and delivery time windows [43]; jointly routing ground vehicles and drones; optimizing for the placements of depots, whose locations are currently randomly generated and given as input.

ACKNOWLEDGMENTS

This work was supported in part by NSF, Award Number: 1830554, the Toyota Research Institute (TRI), and the Ford Motor Company. The authors thank Sarah Laaminach, Nico- las Lanzetti, Mauro Salazar, and Gioele Zardini for fruitful discussions on transit networks.

REFERENCES

1. [1]  General Transit Feed Specification. URL https://developers. google.com/transit/gtfs/. Accessed: August 30, 2019.

2. [2]  Niels Agatz, Paul Bouman, and Marie Schmidt. Optimization Ap-

   proaches for the Traveling Salesman Problem with Drone. *Trans-*

   *portation Science*, 52(4):965–981, 2018.

3. [3]  Ravindra K. Ahuja, Thomas L. Magnanti, and James B. Orlin. *Network*

   *Flows: Theory, Algorithms, and Applications*. Pearson, 1993.

4. [4]  Arash Asadpour, Michel X. Goemans, Aleksander Madry, Shayan Oveis Gharan, and Amin Saberi. An O(log n/ log log n)- Approximation Algorithm for the Asymmetric Traveling Salesman

   Problem. *Operations Research*, 65(4):1043–1061, 2017.

5. [5]  Max Barer, Guni Sharon, Roni Stern, and Ariel Felner. Suboptimal Variants of the Conflict-based Search Algorithm for the Multi-agent Pathfinding Problem. In *Symposium on Combinatorial Search*, 2014.

6. [6]  Hannah Bast, Daniel Delling, Andrew Goldberg, Matthias Mu ̈ller- Hannemann, Thomas Pajor, Peter Sanders, Dorothea Wagner, and Renato F Werneck. Route Planning in Transportation Networks. In

   *Algorithm Engineering*, pages 19–80. Springer, 2016.

7. [7]  Tolga Bektas. The Multiple Traveling Salesman Problem: an Overview of Formulations and Solution Procedures. *Omega*, 34(3):209–219,

   2006.

8. [8]  Jose Caceres-Cruz, Pol Arias, Daniel Guimarans, Daniel Riera, and

   Angel A. Juan. Rich Vehicle Routing Problem: Survey. *ACM Comput.*

   *Surv.*, 47(2):32:1–32:28, 2014.

9. [9]  W Matthew Carlyle, Johannes O Royset, and R Kevin Wood.

   Lagrangian Relaxation and Enumeration for Solving Constrained

   Shortest-Path Problems. *Networks*, 52(4):256–270, 2008.

10. [10]  Shushman Choudhury, Jacob P. Knickerbocker, and Mykel J. Kochen- derfer. Dynamic Real-time Multimodal Routing with Hierarchical Hybrid Planning. In *IEEE Intelligent Vehicles Symposium (IV)*, pages

    2397–2404, 2019.

11. [11]  Liron Cohen, Tansel Uras, TK Satish Kumar, Hong Xu, Nora Ayanian,

    and Sven Koenig. Improved Solvers for Bounded-Suboptimal Multi- Agent Path Finding. In *International Joint Conference on Artificial Intelligence (IJCAI)*, pages 3067–3074, 2016.

12. [12]  Thomas H. Cormen, Charles E. Leiserson, Ronald L. Rivest, and Clifford Stein. *Introduction to Algorithms*. MIT Press, 2009.

13. [13]  Daniel Delling, Peter Sanders, Dominik Schultes, and Dorothea Wag- ner. Engineering Route Planning Algorithms. In *Algorithmics of Large and Complex Networks*, pages 117–139. Springer-Verlag, 2009.

14. [14]  DJI. DJI Mavic 2 Specifications Sheet. URL http://bit.ly/ 2mfCAvz.

15. [15]  Irina Dumitrescu and Natashia Boland. Improved Preprocessing, Labeling and Scaling Algorithms for the Weight-Constrained Shortest Path Problem. *Networks: An International Journal*, 42(3):135–153, 2003.

16. [16]  Michael Erdmann and Tomas Lozano-Perez. On Multiple Moving Objects. *Algorithmica*, 2(1-4):477, 1987.

17. [17]  Ariel Felner, Roni Stern, Solomon Eyal Shimony, Eli Boyarski, Meir Goldenberg, Guni Sharon, Nathan Sturtevant, Glenn Wagner, and Pavel Surynek. Search-based Optimal Solvers for the Multi-Agent Pathfinding Problem: Summary and Challenges. In *Symposium on Combinatorial Search*, 2017.

18. [18]  Sergio Mourelo Ferrandez, Timothy Harbison, Troy Weber, Robert Sturges, and Robert Rich. Optimization of a Truck-Drone in Tandem Delivery Network using k-means and Genetic Algorithm. *Journal of Industrial Engineering and Management*, 9(2):374–388, 2016.

19. [19]  Greg N. Frederickson, Matthew S. Hecht, and Chul E. Kim. Ap- proximation Algorithms for some Routing Problems. In *17th Annual Symposium on Foundations of Computer Science, Houston, Texas, USA, 25-27 October 1976*, pages 216–227, 1976.

20. [20]  Michael R Garey and David S Johnson. *Computers and Intractability; A Guide to the Theory of NP-Completeness*. WH Freeman & Co., 1990.

21. [21]  John H Halton. On the Efficiency of certain Quasi-Random Sequences of Points in Evaluating Multi-Dimensional Integrals. *Numerische Mathematik*, 2(1):84–90, 1960.

22. [22]  Peter Hart, Nils Nilsson, and Bertram Raphael. A Formal Basis for the Heuristic Determination of Minimum Cost Paths. *IEEE Transactions on Systems Science and Cybernetics*, 2(4):100–107, 1968.

23. [23]  Florence Ho, Ana Salta, Ruben Geraldes, Artur Goncalves, Marc Cavazza, and Helmut Prendinger. Multi-agent Path Finding for UAV Traffic Management. In *International Conference on Autonomous Agents and Multiagent Systems (AAMAS)*, pages 131–139, 2019.

24. [24]  Jose Holguin-Veras, Johanna Amaya Leal, Ivan Sanchez-Diaz, Michael Browne, and Jeffrey Wojtowicz. State of the art and Practice

of Urban Freight Management: Part I: Infrastructure, Vehicle-Related, and Traffic Operations. *Transportation Research Part A: Policy and Practice*, 2018.

[25] Wolfgang Ho ̈nig, Scott Kiesel, Andrew Tinka, Joseph W Durham, and Nora Ayanian. Conflict-based Search with Optimal Task Assignment. In *International Conference on Autonomous Agents and Multiagent Systems (AAMAS)*, pages 757–765, 2018.

[26] Edward Humes. Online Shopping Was Supposed to Keep People Out of Traffic. It Only Made Things Worse, 2018. URL http: //bit.ly/2HCkAmQ. Accessed: August 30, 2019.

[27] Ramo ́n Iglesias, Federico Rossi, Rick Zhang, and Marco Pavone. A BCMP network approach to modeling and controlling autonomous mobility-on-demand systems. *I. J. Robotics Res.*, 38(2-3), 2019.

[28] Martin Joerss, Florian Neuhaus, and Jurgen Schroder. How Customer Demands are Reshaping Last-Mile Delivery, 2016. URL https: //mck.co/2NIRdmE. Accessed: August 30, 2019.

[29] Nabin Kafle, Bo Zou, and Jane Lin. Design and Modeling of a Crowdsource-Enabled System for Urban Parcel Relay and Delivery. *Transportation Research Part B: Methodological*, 99:62 – 82, 2017. ISSN 0191-2615.

[30] Hsiang-Tsung Kung, Fabrizio Luccio, and Franco P Preparata. On Finding the Maxima of a Set of Vectors. *Journal of the ACM (JACM)*, 22(4):469–476, 1975.

[31] Yuxi Li, Janelee Harms, and Robert Holte. Fast Exact Multiconstraint Shortest Path Algorithms. In *IEEE International Conference on Communications*, pages 123–130, 2007.

[32] Minghua Liu, Hang Ma, Jiaoyang Li, and Sven Koenig. Task and Path Planning for Multi-Agent Pickup and Delivery. In *International Conference on Autonomous Agents and Multiagent Systems (AAMAS)*, pages 1152–1160, 2019.

[33] Hang Ma, Jiaoyang Li, TK Kumar, and Sven Koenig. Lifelong Multi-agent Path Finding for Online Pickup and Delivery Tasks. In *International Conference on Autonomous Agents and Multiagent Systems (AAMAS)*, pages 837–845, 2017.

[34] Clair E Miller, Albert W Tucker, and Richard A Zemlin. Integer Programming Formulation of Traveling Salesman Problems. *Journal of the ACM (JACM)*, 7(4):326–329, 1960.

[35] Matthias Mu ̈ller-Hannemann, Frank Schulz, Dorothea Wagner, and Christos Zaroliagis. Timetable Information: Models and Algorithms. In *Algorithmic Methods for Railway Optimization*, pages 67–90. Springer, 2007.

[36] Chase C. Murray and Amanda G. Chu. The Flying Sidekick Traveling Salesman Problem: Optimization of Drone-Assisted Parcel Delivery. *Transportation Research Part C: Emerging Technologies*, 54:86 – 109, 2015.

[37] Alena Otto, Niels Agatz, James Campbell, Bruce Golden, and Erwin Pesch. Optimization Approaches for Civil Applications of Unmanned Aerial Vehicles (uavs) or Aerial Drones: A Survey. *Networks*, 72(4): 411–458, 2018.

[38] Judea Pearl and Jin H Kim. Studies in Semi-Admissible Heuristics. *IEEE Transactions on Pattern Analysis and Machine Intelligence*, (4): 392–399, 1982.

[39] Evangelia Pyrga, Frank Schulz, Dorothea Wagner, and Christos Zaro- liagis. Efficient Models for Timetable Information in Public Trans- portation Systems. *Journal of Experimental Algorithmics (JEA)*, 12: 2–4, 2008.

[40] Mauro Salazar, Federico Rossi, Maximilian Schiffer, Christopher H. Onder, and Marco Pavone. On the interaction between autonomous mobility-on-demand and public transportation systems. In *Interna- tional Conference on Intelligent Transportation Systems*, pages 2262– 2269, 2018.

[41] Guni Sharon, Roni Stern, Ariel Felner, and Nathan Sturtevant. Conflict-based Search for Optimal Multi-Agent Path Finding. In *AAAI Conference on Artificial Intelligence (AAAI)*, 2012.

[42] David Silver. Cooperative Pathfinding. In *AAAI Conference on Artificial Intelligence (AAAI)*, pages 117–122, 2005.

[43] Marius M Solomon. Algorithms for the Vehicle Routing and Schedul- ing Problems with Time Window Constraints. *Operations Research*, 35(2):254–265, 1987.

[44] Kiril Solovey, Mauro Salazar, and Marco Pavone. Scalable and Congestion-Aware Routing for Autonomous Mobility-On-Demand via Frank-Wolfe Optimization. In *Proceedings of Robotics: Science and Systems*, 2019.

[45] Adrienne Welch Sudbury and E Bruce Hutchinson. A Cost Analysis of Amazon Prime Air (Drone Delivery). *Journal for Economic Educators*, 16(1):1–12, 2016.

[46] P. Toth and D. Vigo. *Vehicle Routing – Problems, Methods, and*

*Applications*. SIAM, 2 edition, 2014.

1. [47]  Alex Wallar, Menno Van Der Zee, Javier Alonso-Mora, and Daniela

   Rus. Vehicle rebalancing for mobility-on-demand systems with ride- sharing. In *IEEE/RSJ International Conference on Intelligent Robots and Systems*, pages 4539–4546, 2018.

2. [48]  J.YuandS.M.LaValle.OptimalMultirobotPathPlanningonGraphs: Complete Algorithms and Effective Heuristics. *IEEE Transactions on Robotics*, 32(5):1163–1177, 2016.

3. [49]  Jingjin Yu and Steven M LaValle. Structure and Intractability of Optimal Multi-robot Path Planning on Graphs. In *AAAI Conference on Artificial Intelligence (AAAI)*, 2013.

4. [50]  J. Zgraggen, M. Tsao, M. Salazar, M. Schiffer, and M. Pavone. A Model Predictive Control Scheme for Intermodal Autonomous Mobility-on-Demand. In *IEEE International Conference on Intelligent Transportation Systems*, 2019.

   APPENDIX I

TASK ALLOCATION: ADDITIONAL DETAILS AND PROOFS

We present a full and extended version of the MERGES- PLITTOURS algorithm (Algorithm 2) for the task allocation layer. Figure 3 illustrates the behaviour of MCT, which provides an approximate solution for the m-MVP problem (Definition 1). The algorithm consists of three main steps:

Step1(lines1): GenerateacollectionofttoursT1,...,Tt, forsome1􏰑t􏰑k,suchthateverypackagep∈VP is covered by exactly one tour, and the total distance of the tours is minimized. This step is achieved by solving the minimal-connecting tours (MCT) problem (see Table I). The solution to MCT is given by an assignment {xuv}(u,v)∈E, which indicates which edges of G are used and for how many times. This assignment implicitly represents the desired collection of tours T1 , . . . , Tt , as described above. The reason behind why such an assignment breaks into a collection of tours is discussed in Lemma 2 below.

Step 2 (lines 2-10): The T1, . . . , Tt tours are merged in an iterative fashion, until a single tour T is generated. We first identify t 􏰒 1 connected depot sets D = {D1, . . . , Dt}, which are induced by the MCT solution (line 2). That is, every Di consists of all the depots that belong to one specific tour Ti encoded by x. We then perform a merging routine which merges the tours and consequently merges the con- nected depot sets. This routing iterates over all combinations of D,D′ ∈ D,d ∈ D,d′ ∈ D′ (lines 5-8), and chooses (d, d′), (d′, d), such that cdd′ +cd′d is minimized. Then x and D are updated accordingly (lines 9, 10). For a given D and d ∈ VD, the notation D(d) represents the depot component D ∈ D that d belongs to.

Step 3 (lines 6-14): The tour T is partitioned into m paths {P1, . . . , Pm} such that the length of every path is proportional to the length of T divided by m. Additionally, every path Pi starts and ends in a depot, but not necessarily the same one. This step is reminiscent to an algorithm presented in [19] for m-TSP in undirected graphs.

*A. Completeness and optimality*

In preparation to the proof Theorem 1, we have the follow- ing lemma, which states that MCT produces a collection of pairwise-disjoint tours. Henceforth we assume that that GA is strongly connected and that GA (VD ) is a directed clique.

Lemma 2. *Let* x *be the output of* MCT(GA,VP)*. Then there exists a collection of vertex-disjoint tours* T1 , . . . , Tm′ *,*

Algorithm 2: MERGESPLITTOURS-FULL Input: Allocation graph G = (V , E ), with

AAA
 VA =VD ∪· VP, number of agents m􏰒1;

Output: Paths {P1, . . . , Pm}, such that every package is visited exactly once;

x:={xuv}(u,v)∈EA ←MCT(GA,VP);
 D := {D1 . . . , Dt} ← CONNECTEDDEPOTS(GA, x); while |D| > 1 do

cmin ← ∞,dmin ← ∅,d′min ← ∅;
 forD,D′ ∈D,D̸=D′,d∈D,d′ ∈D′ do

ifc′+c′ <c then dd dd min

cmin ← cdd′ + cd′d, dmin ← d, d′min ← d′;

xdmind′min ← 1,xd′mindmin ← 1;

D ← D\{D(dmin), D(d′min)}∪{D(dmin)∪D(d′min)}; T := (d1, p1, . . . , pl−1, dl) ← GETTOUR(GA, x); i←1; j ←1;
 for i = 0 to m do

Pi ←∅,Li ←0;
 while Li 􏰑 LENGTH(T)/m and l < t do

Li←Li+cdjpj +cpjdj+1;
 Pi ←Pi ∪{(dj,pj),(pj,dj+1)}; j ← j + 1;

return {P1,...,Pm};
 *such that for every* (u,v) ∈ EA *such that* xuv > 0*, there*

*exists* Ti *in which* (u, v) *appears exactly* xuv *times.*

*Proof.* By definition of MCT, for every p ∈ Vp there exists precisely one incoming edge (d,p) and one outgoing edge (p,d′) such that xdp = xpd′ = 1. Also, note that by Equa- tion (5), the in-degree and out-degree of every d ∈ VD are equal to each other. Thus, an Eulerian tour can be formed, which traverses every edge (u, v) exactly xuv times.

We are ready for the main proof.

*Proof of Theorem* *1**.* First, note that after every iteration of the “while” loop, the updated assignment x still represents a collection of tours. Second, this loop is repeated at most m− 1 times; t, which represents the initial number of connected depots (line 2), is at most m, since every tour induced by MCT must contain at least one depot.

Next, let OPT be the optimal solution to m-MVP. That is, there exists m paths {P1∗ , . . . , Pm∗ } which represent the solution to m-MVP, and for every i ∈ [m], |Pi∗| 􏰑 OPT. Observe that

m
 􏰊 |Pi| 􏰑 m · max |Pi∗| = m · OPT,

i=1 i∈[m]

where {P1,...,Pm} is the result of MERGESPLITTOURS. Next, by definition of α, we have that |T | 􏰑 m · OPT + mα. Lastly, by definition of β we have that

Ti 􏰑|T|/m+β􏰑OPT+α+β. *B. Computational complexity*

We conclude this section with an analysis of the com- putational complexity of MERGESPLITTOURS. Recall that

| d1 							 						 							d3 							 					 					 						 							p2  d2 							 						 							p3  p1 							 					 					 						 							p4 | d1 							 						 							d3 							 					 					 						 							p2  d2 							 						 							p3  p1 							 					 					 					 					 					 					 						 							p4 | d1 							 						 							d3 							 					 					 						 							p2  d2 							 						 							p3  p1 							 					 					 						 							p4 |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
|                                                              |                                                              |                                                              |

(a) (b) (c)

Fig. 3: Three key steps in the MERGESPLITTOURS algorithm for delivery sequence allocation: (a) The allocation graph is defined for the depots d1:3 and packages p1:4. (b) The MCT step yields a solution that connects each package delivery with the depot from which the drone is dispatched and the depot to which it returns. (c) A tour merging steps merges the depots d2 and d3 into a single cluster.

we identified the main bottleneck of MERGESPLITTOURS to be the solution computation for MCT. We proceed to prove Lemma 1, which states such a solution to MCT can be obtained via a linear relaxation.

*Proof.* The main observation to make is that MCT can be transformed into a minimum-cost circulation (MCC) prob- lem. If all edge capacities are integral, the linear relaxation of MCC enjoys a totally unimodular constraint matrix form [3]. Hence, the linear relaxation will necessarily have an integer optimal solution, which will be a fortiori an optimal solution to the original MCF problem.

In the current representation of MCT, eq. (4) does not cor- respond to a circulation problem. However, we can introduce a small modification to GA which would allow us to recast it as circulation. Consider the graph G′ = (V ′, E′) such that

V′ :=VD ∪VP ∪{p′|p∈VP}, E′:={(d,p)|d∈V ,p∈V ,(d,p)∈E}

constraints represents one subset of (p − c) agents being restricted from using the transit edge in question.

As we pointed out in Section V-B, conflict resolution is a significant bottleneck for solving large MAPF-TN instances. In our experiments, we generated all constraint subsets of a capacity conflict, however, pathological scenarios may arise where this significantly degrades performance in practice (our ultimate yardstick). Whether there exists a principled way to analyse constraint set enumeration and suboptimality and how this can be efficiently implemented in practice are both important questions for future research.

*B. Speedup Techniques*

The NP-hardness of multi-agent path finding [49] and the additional computational challenges of MAPF-TN (path energy constraint; large and dense graphs) make empirical performance paramount, given our real-world scenarios and emphasis on scalability. We now discuss some speedup techniques that improve the efficiency of the low-level search while maintaining its bounded sub-optimality (which in turn ensures bounded sub-optimality of the overall solution, as per Enhanced CBS). Certainly, these techniques are not exhaus- tive; there is an entire body of work in transportation plan- ning devoted to speeding up algorithm running times [13]. We devised and implemented two simple methods.

*1) Preprocessing Public Transit Networks:* Focal-MCSP can become a bottleneck when it has to be run multiple times (at least twice for each agent’s current dpd′ task and more in case of conflicts). Its performance depends significantly on the availability and quality of admissible heuristics, i.e., heuristics that *underestimate* the cost to the goal, for the objective (elapsed time) and constraint (distance traversed, a surrogate for the energy expended). The public-transit network for a given area is usually known in advance and follows a pre-determined timetable. We can analyze and preprocess such a network to obtain admissible heuristics. These can then be used for multiple instances of MAPF-TN throughout the day, while searching for paths to a specific package delivery location.

For the objective function, i.e., the elapsed time, a lower bound is typically the time to fly directly to the goal, without

DP
 ∪ {(p′, d)|d ∈ VD, p ∈ VP , (p, d) ∈ E}

∪ {(p, p′)|p ∈ VP }
 and we require that for every such (p, p′ ), xpp′ = 1.

APPENDIX II MAPF-TN: ADDITIONAL DETAILS

We now elaborate on two aspects of multi-agent pathfind- ing with transit networks (MAPF-TN) that we alluded to in Section IV. First, we discuss how we extend the notion of conflict handling in Conflict-Based Search to the capacity conflicts of MAPF-TN, where more than one agent can use a transit edge. Second, we discuss two important speedup techniques that improve the empirical performance of our MAPF-TN solver, without sacrificing bounded suboptimality.

*A. Capacity Conflicts in (E)CBS*

In the classical MAPF formulation, at most one agent can occupy a particular vertex or traverse a particular edge at a given time. Therefore, conflicts between p > 1 agents yield p new nodes in the multi-agent level search tree of Conflict- Based Search (CBS) and any of its modified variants. In MAPF-TN, however, transit edges in general have capacity C(e) 􏰒 1. Consider a solution generated during a run of Enhanced CBS that has assigned to some transit edge

p > C(e) > 1 drones. In order to guarantee bounded sub- optimality of the solution, we must generate all 􏰀 p 􏰁 sets

p−c
 of constraints, where c = C(e). Each such set of (p − c)

deviating and waiting to board public transit (of course, taking such a route in practice is usually infeasible due to the distance constraint). Therefore, we define the heuristic simply as:

We can now define the goal-directed heuristic function hN for the distance traversed. Let the goal node for a query to Focal-MCSP be vg ∈ VD ∪· VP . We want the heuristic value for the operation graph node v ∈ VO ≡ (VD ∪· VP ∪VTN) that is expanded during Focal-MCSP. If v ∈ VD ∪· VP is a depot or package, we set hN (v, vg) = N ̄T (v, vg). Otherwise, v ∈ VT N is a transit vertex. Recall that each transit vertex is a stop that is associated with a corresponding transit trip. Let the trip associated with v ∈ VTN be Rτ. We then set hN(v,vg) = N ̄T (vτ,vg), where vτ ∈ VT is the trip metagraph vertex corresponding to the trip Rτ . The heuristic hN as defined above is admissible, i.e. is a lower bound on the drone’s flight distance from the expanded operation graph node to the target depot/package location.

In practice, we will solve several instances of MAPF-TN throughout a day, with traffic delays and other disruptions to the timetable. However, the handling of dynamic networks and timetable delays is a separate subfield of research in transportation planning [6, 13] and out of the scope of this work. We make the reasonable assumption (made often in transit planning work) that travel times between locations do not vary greatly throughout the day, and we ignore the effect of delays and disruptions to the pre-determined timetable while using our heuristics.

*2) Pruning Focal-MCSP search space:* As we mentioned in Section IV-A, the edges of the operation graph are not explicitly enumerated but rather implicitly encoded and gen- erated just-in-time during the node expansion stage of Focal- MCSP. An implicit edge set makes the Focal-MCSP search highly memory-efficient by only having to store the vertices of the operation graph. This memory-efficiency comes at the cost of computation time as the outgoing edges of a vertex must be computed during the search. A careful observation of the transit vertices allows us to prune the set of out- neighbors of a vertex expanded during Focal-MCSP, *while still guaranteeing bounded sub-optimality*.

Let u ∈ VO be an operation graph vertex that is expanded during Focal-MCSP. Consider all the transit vertices of a transit trip Rτ (if u ∈ VTN is itself a transit vertex, then consider a trip different from the trip that u lies on). Those transit vertices are candidate out-neighbors for the expanded node u (candidate target vertices of a *time-constrained flight edge* emanating from u and making a connection to the trip Rτ ). It may appear that all trip stops in Rτ that the drone can reach in time, i.e., for which σ×(v.t−u.t) 􏰒 ∥v−u∥ (a required condition, as we mentioned in Section IV-A) should be added as out-neighbors.

However, while considering connections to a trip Rτ , we actually need to only add the transit vertices on Rτ that are *non-dominated* in terms of the tuple of time difference and flight distance (v.t − u.t, ∥v − u∥). Doing so will continue to ensure bounded sub-optimality of Focal-MCSP. We for- malize this observation through a lemma.

Lemma 3. *Let* u ∈ VO *be an operation graph ver- tex expanded during Focal-MCSP. While considering time- constrained flight connections to trip* Rτ*, let* v1 *and* v2 *be two consecutive transit vertices on trip* Rτ *such that* (v1.t − u.t, ∥v1 − u∥) ≼ (v2.t − u.t, ∥v2 − u∥)*. Then, prun-*

h (v,vg)= ∥vg−v∥ Tσ

(6)

where σ is the average drone speed, vg is the goal node and v is the node being expanded. The above heuristic will be admissible, i.e., be a lower bound on elapsed time if the average drone speed is higher than average transit speed. This assumption is typically true for the transit vehicles we consider, given that they are required to wait at stops for people to get on. A more data-driven estimate can be obtained by analyzing actual flight times, but that is out of the scope of this work.

For the constraint function, i.e., the distance traversed, we use a heuristic based on extensive network preprocessing. For a given transit network in the area of operation, we consider the minimal time window such that every instance of a transit vehicle trip in that network can start and finish (as per the timetable). We then create the so-called *trip metagraph*, whose set of vertices is VD ∪· VP ∪· VT , where, from our earlier notation, VD and VP are the sets of depot and package verticesrespectively.Eachvertexvτ ∈VT representsasingle transit vehicle trip Rτ , and encodes its sequence of time- stamped stops (we will discuss what this means in practice shortly). The trip metagraph is complete, i.e., there is an edge between every pair of vertices.

We now define the cost of energy expenditure, i.e., dis- tance traversed for each edge e = (u, v), hereafter denoted as

e = (u → v), in the trip metagraph. If u, v ∈ VT to trips Rτ and Rτ ′ respectively,

correspond

where, as before, v.t refers to the time-stamp of the stop v for that particular trip. The edge cost here is thus *the shortest distance between stops that can be traversed by the drone in the difference between time stamps*. If u, v ∈ VD ∪· VP , we simply set N(e) = ∥v − u∥, the direct flight distance between the locations. For all other edges, i.e., where one of u or v corresponds to a trip Rτ and the other to a depot or package location (in either direction), we set

N(e)= min∥v−u∥ u∈Rτ

and in such cases, the cost for edge (v → u) is equal to that of (u → v). This concludes the assignment of edge costs.

Given the complete specification of the edge cost function, we now run Floyd-Warshall’s algorithm [12] on the trip metagraph to get a cost matrix N ̄T , where N ̄T (u, v) is the cost of the shortest-path from vτ to vτ′ on the trip metagraph. Intuitively, this cost matrix encodes the *least flight distance* required to switch from one trip to another, from a trip to a depot/package and vice versa, and between two depots/package locations, either using the transit network or flying directly, whichever is shorter.

N (e) =
 σ × (v.t − u.t) 􏰒 ∥v − u∥,

min ∥v − u∥, such that u∈Rτ ,v∈Rτ ′

(a) (b)
 Fig. 4: The geographical bounding boxes for (a) San Francisco (roughly 150km2) and the (b) Washington DC Metropolitan area (roughly 400km2).

*ing* v2 *as an out-neighbor has no effect on the solution of Focal-MCSP.*

The following proof relies heavily on the analysis of A*- MCSP (see, e.g., [31, Section V]), upon which Focal-MCSP is based.

*Proof.* We assume that both v1 and v2 are physically reach- able by the drone, i.e., σ×(v.t−u.t) 􏰒 ∥v−u∥ for v = v1, v2 (otherwise they would be discarded anyway).

Note that (v1 → v2) is a transit edge, so the flight distance N(v1 → v2) = 0 by definition. The Focal-MCSP algorithm tracks the *objective* (traversal time) and *constraint* (flight distance) values of partial paths to nodes. It discards a partial path that is dominated by another on both metrics. Consider the only two possible partial paths to v2 from u, u → v2 and u→v1 →v2.

Let the weight constraint accumulated on the path thus far to the expanded node u be W u. The traversal time cost at v2 for both partial paths is v2.t (since v2 is time-stamped). The accumulated traversal distance weight at v2 for u → v2 is Wu+N(u → v2) = Wu+∥v2 −u∥. On the other hand, for u → v1 → v2, the corresponding accumulated weight at node v2 is Wu+N(u → v1)+N(v1 → v2) = Wu+∥v1 −u∥ < W u+∥v2 −u∥, by the original assumption. Focal-MCSP will always discard the partial path u → v2 and instead prefer the alternative, u → v1 → v2. Therefore, pruning v2 as an out- neighbor will have no effect on the solution of Focal-MCSP, which thus continues to be bounded sub-optimal.

The intuition is that if a transit connection is useful to make, then a stop that is both earlier and closer in distance than another will always be preferred. The above result was for *consecutive* vertices on a transit trip; we can extend it to the full sequence of vertices on the trip Rτ by induction.

We use Kung’s algorithm [30] to find the non-dominated elements of the set of transit trip vertices. For two criteria functions, as in our case, Kung’s algorithm yields a solution in O(nlogn) time, where n is the size of the set and the bottleneck is due to sorting the set as per one of the criteria.

In our specific case, since the transit trip vertices are already sorted in increasing order of the traversal time criterion, we can add out-neighbors for a transit trip in O(n) time, which is as fast as we could have done anyway.

APPENDIX III SURROGATE TRAVEL TIME ESTIMATE

At the end of Section II, we mentioned the role of the sur- rogate estimate for travel time between two depots/packages used by MERGESPLITTOURS for the task allocation. We also briefly discussed the actual surrogate estimate we use in our approach. We now provide some more details about how the estimate is actually computed in a preprocessing step and then used during runtime. In Appendix IV, we quantitatively compare the surrogate estimate that we use to the direct flight time between two locations, in terms of the computation time and solution quality of the MAPF-TN layer.

Consider the given geographical area of operation, en- coded as a bounding box of coordinates (Figure 4 illustrates both areas). During preprocessing, we generate a representa- tive set of locations across the area. To ensure good coverage, we use a quasi-random low dispersion sampling scheme [21] to compute the locations. This set of locations induces a Voronoi decomposition [12] of the geographical area where the locations are the sites. Every point in the bounding box is associated with the nearest element (by the appropriate distance metric) in the set of locations. We then choose a representative time window of transit for the area. Between every pair of locations in the set, we compute and store the travel time using the transit network (with the same Focal- MCSP parameters we use for MAPF-TN).

During runtime, at the task allocation layer, we need the estimated travel time between two depot/package locations v, v′ ∈ VD ∪· VP . Each of v and v′ has a corresponding nearest representative location (the site of its Voronoi cell). We then look up the precomputed travel time estimate between the corresponding sites and use that value in MERGES- PLITTOURS. The implicit assumption is that the travel time

between the representative sites is the dominating factor compared to the last-mile travel between each site and its corresponding depot/package. If v and v′ are in the same cell, i.e., their nearest representative location is the same, we use the direct flight time between v and v′, i.e., ∥v′ −v∥/σ. The assumption here is that v and v′ are more likely to share a cell if they are close together, and in that case, the drone is more likely to be able to fly directly between them anyway.

The number of representative locations for a given area is an engineering parameter. For our results, we use 100 points in San Francisco and 150 points in Washington DC. For the quasi-random sampling scheme we use, the higher the number of sampled points, the lower the dispersion, i.e., the better the coverage of the area, and typically, the better is the quality of the surrogate estimate. Domain knowledge about the transit network and travel time distribution in a given urban area may yield a higher quality surrogate than our domain-agnostic approach.

APPENDIX IV FURTHER RESULTS

We now elaborate on three additional aspects of our results, as we alluded to in Section V. First, we provide a more extensive analysis of the behavior of our layer for multi-agent path finding with transit networks (MAPF-TN). Second, we compare two different replanning strategies to solve for a sequence of drone delivery tasks. Third, we quantitatively compare the effect of two different surrogate travel time estimates.

*A. Further Insights of MAPF-TN Results*

We will now supplement our discussion in Section V-B on prominent observations of the behavior of the MAPF- TN layer, based on the numbers in Table III. With regards to scalability, recall that each low-level search is actually *two* Focal-MCSP searches (from d → p and p → d′) that are concatenated, so the effective number of agents (from a typical MAPF perspective) is actually 2m and not m. This observation only serves to strengthen our scalability claim. Since our MAPF-TN solver is built upon Conflict-Based Search, the key factor affecting plan time is the generation and resolution of conflicts, which we have discussed in detail already. We also discussed how the number of depots and the ratio of depots to agents affects the likelihood of conflicts. Depots or warehouses are highly expensive to construct in practice. Thus, in a given area, the placement of depots (that we generate randomly for our benchmarks) can have a significant impact on computation time and scalability; indeed, that is a key question for future work.

The order of magnitude higher runtimes for Washington DC is worth commenting on a bit more. Note that we are using the same drone parameters and transit capacity settings for Washington DC, which has an area nearly three times that of SF, and a transit network nearly twice as big. Consequently, the need for using transit to satisfy deliveries is greatly increased (notice how the average transit usage is reliably higher than for SF). Additionally, the bus network for Washington DC is more sparse in the outskirts and suburban areas. Thus, the bus network becomes more of a bottleneck

TABLE IV: (All times are in seconds) A comparison of replanning strategies for a subset of the {l, m} scenarios from Table III for the San Francisco

network. We run 20 different trials for values in each case.

each setting and depict the average Replan-m

Replan-1

{l, m} {5, 10}

{5, 20} {20, 50} {20, 100}

Replan Time

0.271 0.034 0.006 0.009

Soln. Mksp.

2943.1 3092.2 1463.5 1952.2

Replan Time

0.645 1.599 0.278 0.399

Soln. Mksp.

2880.1 3092.2 1463.5 1952.2

than for San Francisco, leading to more conflicts. Even when there are no conflicts, the average Focal-MCSP search times increase because more of the larger transit graph is being explored by the search algorithm.

With regards to solution quality (makespan), we briefly commented on the real-world significance that even for a large metropolitan area of 400km2, the longest delivery in a set of m tasks is under 2 hours. We used a representative transit window that is largely replicated throughout the rest of the day; therefore, for a given business day of, say, 12 hours, we can expect any drone to make *at least* 6 deliveries (and typically many more).

*B. Replanning Strategies*

We have previously discussed how our MAPF-TN solver based on Enhanced Conflict-Based Search (ECBS) computes paths for a single dpd′ task for each drone. However, drones will typically be assigned to a sequence of deliveries by the task allocation layer. Rather than computing paths for the entire sequence for each drone ahead of time, we use a re- ceding horizon approach where we replan for a drone after it completes its current task. Our computation time is negligible compared to the actual solution execution time (compare the ‘Plan Time’ and ‘Makespan’ columns in Table II); therefore, a receding horizon strategy appears to be quite reasonable.

Two natural replanning strategies emerge in such a con- text: replanning *only* for the finished drone, while main- taining the paths of all the other drones, which we call Replan-1, and replanning for all drones, from each of their current states, which we call Replan-m. In terms of the tradeoff between computation time and solution quality, these two approaches are at the opposite ends of a spectrum. The Replan-m strategy will be optimal among replanning strategies, while being the most computationally expensive as it recomputes m paths; on the other hand, Replan−1 requires only the computation of a single path with the remaining m − 1 paths imposing boarding and capacity constraints.

To evaluate the two replanning strategies, we use the same setup that we did for evaluating MAPF-TN in Section V. For each MAPF-TN solution (one path for each drone), we consider the drone that finishes first among the m drones (since we use a continuous time representation, ties are highly unlikely in practice). In the case of Replan-1, we run Focal-MCSP for the drone with the various constraints induced by the remaining paths of the other agents. We update the (m-agent) solution with the new path (updating makespan if need be). In the case of Replan-m, we run

TABLE V: We compare our MAPF-TN results from Table III (Average Plan Time and Makespan) against those where the framework uses the direct flight time as a surrogate estimate for MERGESPLITTOURS instead of our preprocessed surrogate using representative locations. For clarity of viewing, we split out the results by city/network into two separate tables. The values for the Preprocessed sub-table are copied over from Table III.

{l, m} {5, 10}

{5, 20} {5, 50} {10, 20} {10, 50} {10, 100} {20, 50} {20, 100} {20, 200}

San Francisco

Preprocessed

Plan Soln. Time Mksp.

1.17 2554.7 2.13 2886.8 3.89 3380.9 1.02 2091.6 1.46 2504.7 7.29 2971.8 0.46 1273.6 1.05 1642.4 2.10 1898.5

Direct Flight

Plan Soln. Time Mksp.

1.51 2624.8 2.69 3092.9 5.08 3412.4 0.83 1868.9 1.25 2247.3 3.78 2649.6 0.27 1079.1 0.64 1371.1 1.43 1426.2

{l, m} {5, 10}

{5, 20} {5, 50} {10, 20} {10, 50} {10, 100} {20, 50} {20, 100} {20, 200}

Washington DC

Preprocessed

Plan Soln. Time Mksp.

5.65 5167.3 13.1 5384.5 28.9 6140.2 4.67 4017.2 15.8 5312.3 26.2 5623.9 1.92 3571.8 5.24 4304.5 10.5 5085.6

Direct Flight

Plan Soln. Time Mksp.

13.6 4654.7 35.2 5339.6 51.1 6323.4 11.9 4527.3 28.6 5509.6 53.8 5774.1 8.49 4058.1 22.8 4613.9 17.6 5216.1

Enhanced CBS for the m agents with their current states (at that time) as their initial state; this yields another (m-agent) solution.

In Table IV, we compare the average makespan and computation times of the m-agent solutions resulting from the two strategies. We use a representative subset of the {Depots, Agents} scenarios that we used in Table III; few depots with a lower agent/depot ratio ({5,10}); few de- pots with a higher ratio ({5,20}); and similarly for many depots ({20, 50} and {20, 100}). It is clear that Replan-1 achieves similar quality solutions as Replan-m does, at fairly lower computational cost. This motivates our decision to use Replan-1 in practice.

In principle, we can design scenarios where Replan-1 has a much greater solution quality gap against Replan-m than what we see in Table IV. However, the Replan-1 strategy is sub-optimal only when (i) the (m − 1) unfinished drone paths actually conflict with the new Focal-MCSP path of drone i, that has just finished, and (ii) resolving the conflict(s) would have prioritized the path of drone i over the others. In practice, it is not very likely that both of these conditions will hold together, especially when there are many depots and some drones can fly directly to their next target; in our trials for l = 20, the sub-optimality condition for Replan-1 never holds, which is why the makespans for those two rows are exactly the same for both strategies.

*C. Comparison of Surrogate Estimates*

We now compare the effect of two different surrogate travel time estimates — the approximate travel time between representative locations in the city using the transit (as described in Appendix III) and the direct flight time between two locations, ignoring the transit. For the results in Table III, recall that we ran MAPF-TN on the first dpd′ task for each drone obtained from the result of MERGESPLITTOURS; for those results, MERGESPLITTOURS used the preprocessed surrogate for the allocation graph edge costs. As a compar- ison, we rerun the exact same scenarios as in Table III, but this time, we use the direct flight time (ignoring the transit) as the edge cost for MERGESPLITTOURS. We compare the two primary performance factors, plan time and solution makespan, for both surrogates in Table V.

We expect the direct flight time surrogate to be a poor es- timate in scenarios where transit is used frequently, because the allocation step does not account for it. Accordingly, we do observe a difference in plan time and solution quality between Preprocessed and Direct Flight for the settings with fewer depots and higher agent-to-depot ratios. For the settings with 5 depots in San Francisco, and for almost all settings in Washington (except the first), both computation time and the makespan are lower for Preprocessed, i.e., it is strictly better than Direct Flight. However, for the settings in San Francisco with 10 or more depots, in most cases the drones are close enough to their deliveries to fly directly (recall the lower average transit usage of those cases from Table III). Here the Direct flight surrogate is more accurate, leading to lower makespan solutions. The key takeaway is how the choice of surrogate plays a role on real-world settings for our two-stage approach.