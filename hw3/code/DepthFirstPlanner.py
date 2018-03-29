class DepthFirstPlanner(object):
    
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

        queue = [start_node_id]
        visited_nodes = [0] * self.planning_env.discrete_env.NodesNumber()

        visited_nodes[start_node_id] = -1  # stands for starting node's parent is none

        # 1. Traverse
        while len(queue) > 0:
            current_node = queue.pop(-1)
            if current_node == goal_node_id:
                break
            successors = self.planning_env.GetSuccessors(current_node)
            for successor in successors:
                if visited_nodes[successor] == 0:
                    visited_nodes[successor] = current_node
                    queue.append(successor)

                    if self.visualize:
                        self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(current_node),
                                                    self.planning_env.discrete_env.NodeIdToConfiguration(successor))

        # 2. Find the path
        plan_nodes = [goal_node_id]
        current_node = goal_node_id
        while current_node != start_node_id:
            current_node = visited_nodes[current_node]
            plan_nodes.append(current_node)

        # 3. Add configuration to plan
        for node_id in plan_nodes[::-1]:
            plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(node_id))

        return plan
