import rospy
from nav_msgs.msg import OccupancyGrid,MapMetaData
import numpy as np
import matplotlib.pyplot as plt

class MapViz:
    def __init__(self):
        rospy.init_node('map_viz', log_level=rospy.DEBUG)
        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None
        self.occupancy_updated = False
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_metadata_callback)

    def map_metadata_callback(self, msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        self.map_probs = msg.data
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            print("Got call back map")
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  4,
                                                  self.map_probs)
            self.occupancy_updated = True
    
    def show_map(self):
        print("plotting...")

        fig = plt.figure()
        self.occupancy.plot(fig.number)
        plt.show()
    
    def run(self):
        self.show_map()


class StochOccupancyGrid2D(object):
    def __init__(self, resolution, width, height, origin_x, origin_y,
                window_size, probs, thresh=0.5):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.probs = probs
        self.window_size = window_size
        self.thresh = thresh

    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))


    def occupancy_index(self, grid_x, grid_y):
       """Given x, y into a grid, return index into self.occupancy list."""
       return grid_y * self.width + grid_x


    def grid_index(self, index):
       """Given index in self.occupancy list, return x, y indices in grid."""
       x = index % self.width
       y = index / self.width
       return x, y

    def add_occupancy(self, window = 5):
        occupied_indices = self.probs > self.thresh
        num_positions = len(self.probs)

        for index, prob in enumerate(occupied_indices):

            lower_bound = index % self.width
            upper_bound = self.width - lower_bound
            for j in range(window):
                for i in range(window):

                    if i < lower_bound:
                        index_down_left = index - j*self.width - i
                        if index_down_left > 0:
                            self.probs[index_down_left] += prob

                        index_up_left = index + j*self.width - i
                        if index_up_left > 0:
                            self.probs[index_up_left] += prob

                    if i < upper_bound:
                        index_down_right = index - j*self.width + i
                        if index_down_right > 0:
                            self.probs[index_down_right] += prob

                        index_up_right = index + j*self.width + i
                        if index_up_right > 0:
                            self.probs[index_up_right] += prob
                            
                        # self.probs[index + i] += prob

                    # index_up = index + j*self.width
                    # if index_up < num_positions:
                    #     self.probs[index_up] += prob

                    # index_down = index - j*self.width
                    # if index_down > 0:
                    #     self.probs[index_down] += prob

    def is_free(self, state):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        p_total = 1.0
        lower = -int(round((self.window_size-1)/2))
        upper = int(round((self.window_size-1)/2))
        for dx in range(lower,upper+1):
            for dy in range(lower,upper+1):
                x, y = self.snap_to_grid([state[0] + dx * self.resolution, state[1] + dy * self.resolution])
                grid_x = int((x - self.origin_x) / self.resolution)
                grid_y = int((y - self.origin_y) / self.resolution)
                if grid_y>0 and grid_x>0 and grid_x<self.width and grid_y<self.height:
                    p_total *= (1.0-max(0.0,float(self.probs[grid_y * self.width + grid_x])/100.0))
        return (1.0-p_total) < self.thresh

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        pts = []
        for i in range(len(self.probs)):
            # convert i to (x,y)
            gy = int(i/self.width)
            gx = i % self.width
            x = gx * self.resolution + self.origin_x
            y = gy * self.resolution + self.origin_y
            if not self.is_free((x,y)):
                pts.append((x,y))
        pts_array = np.array(pts)
        plt.scatter(pts_array[:,0],pts_array[:,1],color="red",zorder=15,label='planning resolution')

if __name__ == '__main__':
    mapViz = MapViz()
    #rospy.spin()
    rate = rospy.Rate(1)  # 10 Hz
    while not rospy.is_shutdown():
    # # don't start until we received
        if mapViz.occupancy_updated:
            mapViz.show_map()
            exit(1)
        rate.sleep()

### TESTING

# # A simple example
# width = 10
# height = 10
# x_init = (0,0)
# x_goal = (8,8)
# obstacles = [((6,6),(8,7)),((2,1),(4,2)),((2,4),(4,6)),((6,2),(8,4))]
# occupancy = DetOccupancyGrid2D(width, height, obstacles)

# A large random example
# width = 101
# heig
