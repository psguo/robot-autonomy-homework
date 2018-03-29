
class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        start_node_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        visited_nodes_cost = {}
        visited_nodes_parents = {}

        # visited_nodes_cost = [-1] * self.planning_env.discrete_env.NodesNumber()
        # visited_nodes_parents = [0] * self.planning_env.discrete_env.NodesNumber()

        visited_nodes_cost[start_node_id] = 0  # stands for starting node's cost
        visited_nodes_parents[start_node_id] = -1 # stands for starting node's parent is none

        traversed_nodes = [start_node_id]

        # 1. Traverse
        while True:
            # 1.1 Find all reachable node's cost
            all_successors_cost = {}
            all_successors_parents = {}
            for current_node in traversed_nodes:
                successors = self.planning_env.GetSuccessors(current_node)
                for successor in successors:
                    if successor not in visited_nodes_parents: # not visited nodes
                        successor_cost = visited_nodes_cost[current_node] + 1 + self.planning_env.ComputeHeuristicCost(successor, goal_node_id)
                        if successor in all_successors_cost:
                            if successor_cost < all_successors_cost[successor]:
                                all_successors_cost[successor] = successor_cost
                                all_successors_parents[successor] = current_node
                        else:
                            all_successors_cost[successor] = successor_cost
                            all_successors_parents[successor] = current_node


            # 1.2 Determine the lowest cost successor
            next_node = min(all_successors_cost, key=all_successors_cost.get)

            # 1.3 Update parent & cost
            traversed_nodes.append(next_node)
            parent_node = all_successors_parents[next_node]
            visited_nodes_cost[next_node] = visited_nodes_cost[parent_node] + 1
            visited_nodes_parents[next_node] = parent_node


            # print(self.planning_env.discrete_env.NodeIdToGridCoord(next_node))
            # 1.4 check if reached goal
            if next_node == goal_node_id:
                break
            if self.visualize:
                self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(parent_node),
                                           self.planning_env.discrete_env.NodeIdToConfiguration(next_node))

        # 2. Find the path
        plan_nodes = [goal_node_id]
        current_node = goal_node_id
        while current_node != start_node_id:
            current_node = visited_nodes_parents[current_node]
            plan_nodes.append(current_node)

        # 3. Add configuration to plan
        for node_id in plan_nodes[::-1]:
            plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(node_id))

        return plan
