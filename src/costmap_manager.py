#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
from itertools import product


"""
Class to deal with OccupancyGrid in Python
as in local / global costmaps.
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au> (it was all wrong)
Modified: Alessandro Leonardi 
"""


class OccupancyGridManager(object):
    def __init__(self, topic, subscribe_to_updates=False):
        # OccupancyGrid starts on lower left corner
        # and width / X is from bottom to top
        # and height / Y is from left to right
        # This makes no sense to me, but it is what it is
        self._grid_data = None
        self._occ_grid_metadata = None
        self._reference_frame = None
        self._sub = rospy.Subscriber(topic, OccupancyGrid,
                                     self._occ_grid_cb,
                                     queue_size=1)
        if subscribe_to_updates:
            self._updates_sub = rospy.Subscriber(topic + '_updates',
                                                 OccupancyGridUpdate,
                                                 self._occ_grid_update_cb,
                                                 queue_size=1)
        rospy.loginfo("Waiting for '" +
                      str(self._sub.resolved_name) + "'...")
        while self._occ_grid_metadata is None and \
                self._grid_data is None:
            rospy.sleep(0.1)
        rospy.loginfo("OccupancyGridManager for '" +
                      str(self._sub.resolved_name) +
                      "'initialized!")
        rospy.loginfo("Height (y): " + str(self.height) +
                      " Width (x): " + str(self.width) +
                      " reference_frame: " + str(self.reference_frame) +
                      " origin: " + str(self.origin))

    @property
    def resolution(self):
        return self._occ_grid_metadata.resolution

    @property
    def width(self):
        return self._occ_grid_metadata.width

    @property
    def height(self):
        return self._occ_grid_metadata.height

    @property
    def origin(self):
        return self._occ_grid_metadata.origin

    @property
    def reference_frame(self):
        return self._reference_frame

    def _occ_grid_cb(self, data):
        rospy.logdebug("Got a full OccupancyGrid update")
        self._occ_grid_metadata = data.info
        # Contains resolution, width & height
        # np.set_printoptions(threshold=99999999999, linewidth=200)
        self._grid_data = np.array(data.data,
                                   dtype=np.int8).reshape(data.info.height,
                                                          data.info.width)
        self._grid_data = self._grid_data.T
        self._reference_frame = data.header.frame_id
        # self._grid_data = np.zeros((data.info.height,
        #                             data.info.width),
        #                            dtype=np.int8)

    def _occ_grid_update_cb(self, data):
        rospy.logdebug("Got a partial OccupancyGrid update")
        # x, y origin point of the update
        # width and height of the update
        # data, the udpdate
        data_np = np.array(data.data,
                           dtype=np.int8).reshape(data.height, data.width)
        self._grid_data[data.y:data.y +
                        data.height, data.x:data.x + data.width] = data_np
        self._grid_data = self._grid_data.T
        # print("grid update:")
        # print(self._grid_data)

    def get_world_x_y(self, costmap_x, costmap_y):
        world_x = costmap_x * self.resolution + self.origin.position.x
        world_y = costmap_y * self.resolution + self.origin.position.y
        return world_x, world_y

    def get_costmap_x_y(self, world_x, world_y):
        costmap_x = int(
            round((world_x - self.origin.position.x) / self.resolution))
        costmap_y = int(
            round((world_y - self.origin.position.y) / self.resolution))
        return costmap_x, costmap_y

    def get_cost_from_world_x_y(self, x, y):
        cx, cy = self.get_costmap_x_y(x, y)
        try:
            return self.get_cost_from_costmap_x_y(cx, cy)
        except IndexError as e:
            raise IndexError("Coordinates out of grid (in frame: {}) x: {}, y: {} must be in between: [{}, {}], [{}, {}]. Internal error: {}".format(
                self.reference_frame, x, y,
                self.origin.position.x,
                self.origin.position.x + self.width * self.resolution,
                self.origin.position.y,
                self.origin.position.y + self.height * self.resolution,
                e))

    def get_cost_from_costmap_x_y(self, x, y):
        if self.is_in_gridmap(x, y):
            return self._grid_data[x][y]
        else:
            raise IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, self.width, self.height))

    def is_in_gridmap(self, x, y):
        if -1 < x < self.width and -1 < y < self.height:
            return True
        else:
            return False

    def get_closest_cell_under_cost(self, x, y, cost_threshold, max_radius):
        """
        Looks from closest to furthest in a circular way for the first cell
        with a cost under cost_threshold up until a distance of max_radius,
        useful to find closest free cell.
        returns -1, -1 , -1 if it was not found.
        :param x int: x coordinate to look from
        :param y int: y coordinate to look from
        :param cost_threshold int: maximum threshold to look for
        :param max_radius int: maximum number of cells around to check
        """
        return self._get_closest_cell_arbitrary_cost(
            x, y, cost_threshold, max_radius, bigger_than=False)

    def get_closest_cell_over_cost(self, x, y, cost_threshold, max_radius):
        """
        Looks from closest to furthest in a circular way for the first cell
        with a cost over cost_threshold up until a distance of max_radius,
        useful to find closest obstacle.
        returns -1, -1, -1 if it was not found.
        :param x int: x coordinate to look from
        :param y int: y coordinate to look from
        :param cost_threshold int: minimum threshold to look for
        :param max_radius int: maximum number of cells around to check
        """
        return self._get_closest_cell_arbitrary_cost(
            x, y, cost_threshold, max_radius, bigger_than=True)

    def _get_closest_cell_arbitrary_cost(self, x, y,
                                         cost_threshold, max_radius,
                                         bigger_than=False):

        # Check the actual goal cell
        try:
            cost = self.get_cost_from_costmap_x_y(x, y)
        except IndexError:
            return None

        if bigger_than:
            if cost > cost_threshold:
                return x, y, cost
        else:
            if cost < cost_threshold:
                return x, y, cost

        def create_radial_offsets_coords(radius):
            """
            Creates an ordered by radius (without repetition)
            generator of coordinates to explore around an initial point 0, 0
            For example, radius 2 looks like:
            [(-1, -1), (-1, 0), (-1, 1), (0, -1),  # from radius 1
            (0, 1), (1, -1), (1, 0), (1, 1),  # from radius 1
            (-2, -2), (-2, -1), (-2, 0), (-2, 1),
            (-2, 2), (-1, -2), (-1, 2), (0, -2),
            (0, 2), (1, -2), (1, 2), (2, -2),
            (2, -1), (2, 0), (2, 1), (2, 2)]
            """
            # We store the previously given coordinates to not repeat them
            # we use a Dict as to take advantage of its hash table to make it more efficient
            coords = {}
            # iterate increasing over every radius value...
            for r in range(1, radius + 1):
                # for this radius value... (both product and range are generators too)
                tmp_coords = product(range(-r, r + 1), repeat=2)
                # only yield new coordinates
                for i, j in tmp_coords:
                    if (i, j) != (0, 0) and not coords.get((i, j), False):
                        coords[(i, j)] = True
                        yield (i, j)

        coords_to_explore = create_radial_offsets_coords(max_radius)

        for idx, radius_coords in enumerate(coords_to_explore):
            # for coords in radius_coords:
            tmp_x, tmp_y = radius_coords
            # print("Checking coords: " +
            #       str((x + tmp_x, y + tmp_y)) +
            #       " (" + str(idx) + " / " + str(len(coords_to_explore)) + ")")
            try:
                cost = self.get_cost_from_costmap_x_y(x + tmp_x, y + tmp_y)
            # If accessing out of grid, just ignore
            except IndexError:
                pass
            if bigger_than:
                if cost > cost_threshold:
                    return x + tmp_x, y + tmp_y, cost

            else:
                if cost < cost_threshold:
                    return x + tmp_x, y + tmp_y, cost

        return -1, -1, -1

    def get_area_of_obst(self, x, y,cost_threshold, max_radius,bigger_than=False):
        count = 0
        # Check the actual goal cell
        try:
            cost = self.get_cost_from_costmap_x_y(x, y)
        except IndexError:
            return None

        if bigger_than:
            if cost > cost_threshold:
                count += 1
        else:
            if cost < cost_threshold:
                count += 1

        def create_radial_offsets_coords(radius):
            """
            Creates an ordered by radius (without repetition)
            generator of coordinates to explore around an initial point 0, 0
            For example, radius 2 looks like:
            [(-1, -1), (-1, 0), (-1, 1), (0, -1),  # from radius 1
            (0, 1), (1, -1), (1, 0), (1, 1),  # from radius 1
            (-2, -2), (-2, -1), (-2, 0), (-2, 1),
            (-2, 2), (-1, -2), (-1, 2), (0, -2),
            (0, 2), (1, -2), (1, 2), (2, -2),
            (2, -1), (2, 0), (2, 1), (2, 2)]
            """
            # We store the previously given coordinates to not repeat them
            # we use a Dict as to take advantage of its hash table to make it more efficient
            coords = {}
            # iterate increasing over every radius value...
            for r in range(1, radius + 1):
                # for this radius value... (both product and range are generators too)
                tmp_coords = product(range(-r, r + 1), repeat=2)
                # only yield new coordinates
                for i, j in tmp_coords:
                    if (i, j) != (0, 0) and not coords.get((i, j), False):
                        coords[(i, j)] = True
                        yield (i, j)

        coords_to_explore = create_radial_offsets_coords(max_radius)

        for idx, radius_coords in enumerate(coords_to_explore):
            # for coords in radius_coords:
            tmp_x, tmp_y = radius_coords
            # print("Checking coords: " +
            #       str((x + tmp_x, y + tmp_y)) +
            #       " (" + str(idx) + " / " + str(len(coords_to_explore)) + ")")
            try:
                cost = self.get_cost_from_costmap_x_y(x + tmp_x, y + tmp_y)
            # If accessing out of grid, just ignore
            except IndexError:
                pass
            if bigger_than:
                if cost > cost_threshold:
                    count += 1

            else:
                if cost < cost_threshold:
                    count += 1

        return count

    def get_center_obst(self, x, y, cost_threshold,radius_obst,radius_tollerance):
        coordinate = [[x,y]]
        center_probability_prec = 0
        # Check the actual goal cell
        try:
            cost = self.get_cost_from_costmap_x_y(x, y)
        except IndexError:
            return None

        def create_radial_offsets_coords(radius):
            """
            Creates an ordered by radius (without repetition)
            generator of coordinates to explore around an initial point 0, 0
            For example, radius 2 looks like:
            [(-1, -1), (-1, 0), (-1, 1), (0, -1),  # from radius 1
            (0, 1), (1, -1), (1, 0), (1, 1),  # from radius 1
            (-2, -2), (-2, -1), (-2, 0), (-2, 1),
            (-2, 2), (-1, -2), (-1, 2), (0, -2),
            (0, 2), (1, -2), (1, 2), (2, -2),
            (2, -1), (2, 0), (2, 1), (2, 2)]
            """
            # We store the previously given coordinates to not repeat them
            # we use a Dict as to take advantage of its hash table to make it more efficient
            coords = {}
            # iterate increasing over every radius value...
            for r in range(1, radius + 1):
                # for this radius value... (both product and range are generators too)
                tmp_coords = product(range(-r, r + 1), repeat=2)
                # only yield new coordinates
                for i, j in tmp_coords:
                    if (i, j) != (0, 0) and not coords.get((i, j), False):
                        coords[(i, j)] = True
                        yield (i, j)

        coords_to_explore = create_radial_offsets_coords(radius_obst)
        for idx, radius_coords in reversed(list(enumerate(coords_to_explore))):
            # for coords in radius_coords:
            tmp_x, tmp_y = radius_coords
            try:
                    cost= self.get_cost_from_costmap_x_y(x + tmp_x, y + tmp_y)
                # If accessing out of grid, just ignore
            except IndexError:
                    pass  
            if cost >= cost_threshold:
                #verify that the second circle is all of cost_treshold
                center_probability = 0
                coords_to_explore_center = create_radial_offsets_coords(radius_tollerance)
                for idx_c, radius_coords_c in reversed(list(enumerate(coords_to_explore_center))):
                    # for coords in radius_coords:
                    #print(center_probability)
                    tmp_x_c, tmp_y_c = radius_coords_c
                    try:
                        cost_c = self.get_cost_from_costmap_x_y(x + tmp_x_c, y + tmp_y_c)
                        if(cost_c >=  cost_threshold ):
                            center_probability +=1
                    # If accessing out of grid, just ignore
                    except IndexError:
                        pass

                if center_probability >= center_probability_prec:
                    #print(center_probability)
                    #print(center_probability_prec)
                    coordinate.append([x + tmp_x, y + tmp_y])
                    center_probability_prec = center_probability
        x_mean = 0 
        y_mean = 0 
        for i in range (len(coordinate)):
            x_mean += coordinate[i][0]
            y_mean += coordinate[i][1]
        x_mean /= (len(coordinate))
        y_mean /= (len(coordinate))
        return x_mean,y_mean