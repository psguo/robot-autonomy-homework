import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        self.env = self.robot.GetEnv()
        self.resolution = resolution

    def CheckConfigCollision(self, config):

        t = self.robot.GetTransform()
        t[:2,3] = numpy.array(config)
        self.robot.SetTransform(t)
        return self.env.CheckCollision(self.robot)

    def GetSuccessors(self, node_id):

        successors = []

        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        current_config = self.discrete_env.NodeIdToConfiguration(node_id)
        for idx in range(self.discrete_env.dimension):
            if coord[idx] - 1 >= 0:
                neighbor_coord = coord[:]
                neighbor_coord[idx] -= 1
                neighbor_config = self.discrete_env.GridCoordToConfiguration(neighbor_coord)
                if not self.CheckConfigCollision(neighbor_config):
                    successors.append(self.discrete_env.GridCoordToNodeId(neighbor_coord))
            if coord[idx] + 1 <= self.discrete_env.num_cells[idx] - 1:
                neighbor_coord = coord[:]
                neighbor_coord[idx] += 1
                neighbor_config = self.discrete_env.GridCoordToConfiguration(neighbor_coord)
                if not self.CheckConfigCollision(neighbor_config):
                    successors.append(self.discrete_env.GridCoordToNodeId(neighbor_coord))

        return successors

    def ComputeDistance(self, start_id, end_id):

        # computes the distance between the configurations given
        # by the two node ids

        start_config = numpy.array(self.discrete_env.NodeIdToGridCoord(start_id))
        end_config = numpy.array(self.discrete_env.NodeIdToGridCoord(end_id))


        dist = numpy.linalg.norm(end_config-start_config)
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):

        # TODO: Here you will implement a function that
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id, goal_id)

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

        
