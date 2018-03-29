import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)


    def ConfigurationToNodeId(self, config):

        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return node_id

    def NodeIdToConfiguration(self, nid):

        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(coord)
        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for idx in range(self.dimension):
            lower_limit = self.lower_limits[idx]
            coord[idx] = int(numpy.floor((config[idx] - lower_limit)/self.resolution))
        return coord

    def GridCoordToConfiguration(self, coord):
        
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for idx in range(self.dimension):
            lower_limit = self.lower_limits[idx]
            config[idx] = round(lower_limit + coord[idx] * self.resolution + self.resolution/2,2)
        return config

    def GridCoordToNodeId(self,coord):
        
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0
        accum_dim = 1
        for idx in range(self.dimension):
            node_id += coord[idx] * accum_dim
            accum_dim *= int(self.num_cells[idx])
        return node_id

    def NodeIdToGridCoord(self, node_id):

        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        accum_dim = 1
        for idx in range(self.dimension-1):
            accum_dim *= int(self.num_cells[idx])

        for idx in range(self.dimension-1, -1, -1):
            coord[idx] = node_id / accum_dim
            node_id %= accum_dim
            if idx > 0:
                accum_dim /= int(self.num_cells[idx-1])
        return coord
        
        
        
