#!/usr/bin/env python3

# This assignment lets you both define a strategy for picking the next point to explore and determine how you
#  want to chop up a full path into way points. You'll need path_planning.py as well (for calculating the paths)
#
# Note that there isn't a "right" answer for either of these. This is (mostly) a light-weight way to check
#  your code for obvious problems before trying it in ROS. It's set up to make it easy to download a map and
#  try some robot starting/ending points
#
# Given to you:
#   Image handling
#   plotting
#   Some structure for keeping/changing waypoints and converting to/from the map to the robot's coordinate space
#
# Slides

# The ever-present numpy
import numpy as np

# Your path planning code
import lab3.path_planning as path_planning
from collections import deque



# -------------- Showing start and end and path ---------------
def plot_with_explore_points(im_threshhold, zoom=1.0, robot_loc=None, explore_points=None, best_pt=None):
    """Show the map plus, optionally, the robot location and points marked as ones to explore/use as end-points
    @param im - the image of the SLAM map
    @param im_threshhold - the image of the SLAM map
    @param robot_loc - the location of the robot in pixel coordinates
    @param best_pt - The best explore point (tuple, i,j)
    @param explore_points - the proposed places to explore, as a list"""

    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 2)
    axs[0].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[0].set_title("original image")
    axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[1].set_title("threshold image")
    """
    # Used to double check that the is_xxx routines work correctly
    for i in range(0, im_threshhold.shape[1]-1, 10):
        for j in range(0, im_threshhold.shape[0]-1, 2):
            if is_reachable(im_thresh, (i, j)):
                axs[1].plot(i, j, '.b')
    """

    # Show original and thresholded image
    if explore_points is not None:
        for p in explore_points:
            axs[1].plot(p[0], p[1], '.b', markersize=2)

    for i in range(0, 2):
        if robot_loc is not None:
            axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
        if best_pt is not None:
            axs[i].plot(best_pt[0], best_pt[1], '*y', markersize=10)
        axs[i].axis('equal')

    for i in range(0, 2):
        # Implements a zoom - set zoom to 1.0 if no zoom
        width = im_threshhold.shape[1]
        height = im_threshhold.shape[0]

        axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
        axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)


# -------------- For converting to the map and back ---------------
def convert_pix_to_x_y(im_size, pix, size_pix):
    """Convert a pixel location [0..W-1, 0..H-1] to a map location (see slides)
    Note: Checks if pix is valid (in map)
    @param im_size - width, height of image
    @param pix - tuple with i, j in [0..W-1, 0..H-1]
    @param size_pix - size of pixel in meters
    @return x,y """
    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Pixel {pix} not in image, image size {im_size}")

    return [size_pix * pix[i] / im_size[1-i] for i in range(0, 2)]


def convert_x_y_to_pix(im_size, x_y, size_pix):
    """Convert a map location to a pixel location [0..W-1, 0..H-1] in the image/map
    Note: Checks if x_y is valid (in map)
    @param im_size - width, height of image
    @param x_y - tuple with x,y in meters
    @param size_pix - size of pixel in meters
    @return i, j (integers) """
    pix = [int(x_y[i] * im_size[1-i] / size_pix) for i in range(0, 2)]

    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Loc {x_y} not in image, image size {im_size}")
    return pix


def is_reachable(im, pix):
    """ Is the pixel reachable, i.e., has a neighbor that is free?
    Used for
    @param im - the image
    @param pix - the pixel i,j"""

    # GUIDE: Returns True (the pixel is adjacent to a pixel that is free)
    #  False otherwise
    # You can use four or eight connected - eight will return more points
    # YOUR CODE HERE
    i, j = pix
    height, width = im.shape

    # Bounds check
    if not (0 <= i < width and 0 <= j < height):
        return False

    # This frontier definition: the pixel itself must be UNSEEN
    if not path_planning.is_unseen(im, pix):
        return False

    # 8-connected neighbours around (i, j)
    neighbour_offsets = [
        (-1, -1), (0, -1), (1, -1),
        (-1,  0),          (1,  0),
        (-1,  1), (0,  1), (1,  1),
    ]

    # At least one neighbour should be FREE
    for di, dj in neighbour_offsets:
        ni, nj = i + di, j + dj
        if 0 <= ni < width and 0 <= nj < height:
            if path_planning.is_free(im, (ni, nj)):
                return True

    return False

def find_all_possible_goals(im):
    """ Find all of the places where you have a pixel that is unseen next to a pixel that is free
    It is probably easier to do this, THEN cull it down to some reasonable places to try
    This is because of noise in the map - there may be some isolated pixels
    @param im - thresholded image
    @return dictionary or list or binary image of possible pixels"""

    # YOUR CODE HERE
    height, width = im.shape
    frontier_points = []

    for i in range(width):
        for j in range(height):
            if is_reachable(im, (i, j)):
                frontier_points.append((i, j))

    return frontier_points

def find_best_point(im, possible_points, robot_loc):
    """ Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from
    @param robot_loc - location of the robot (in case you want to factor that in)
    """
    # YOUR CODE HERE
    height, width = im.shape

    # Normalize possible_points into a list of (i, j) unseen frontier pixels
    if isinstance(possible_points, dict):
        frontier_pixels = list(possible_points.keys())
    elif isinstance(possible_points, np.ndarray):
        ys, xs = np.nonzero(possible_points)
        frontier_pixels = list(zip(xs, ys))
    else:
        frontier_pixels = list(possible_points)

    # Quick sanity check: if nothing to explore, stay where you are
    if not frontier_pixels:
        return robot_loc

    # -----------------------------
    # 1) BFS from robot to find reachable FREE cells
    # -----------------------------
    reachable = np.zeros((height, width), dtype=bool)
    q = deque()

    # Only start BFS if the robot is on free space
    if path_planning.is_free(im, robot_loc):
        rx, ry = robot_loc
        reachable[ry, rx] = True
        q.append(robot_loc)

    while q:
        x, y = q.popleft()
        for nx, ny in path_planning.eight_connected((x, y)):
            if 0 <= nx < width and 0 <= ny < height:
                if not reachable[ny, nx] and path_planning.is_free(im, (nx, ny)):
                    reachable[ny, nx] = True
                    q.append((nx, ny))

    # -----------------------------
    # 2) For each frontier pixel, look for reachable FREE neighbours
    # -----------------------------
    candidate_goals = []

    neighbour_offsets = [
        (-1, -1), (0, -1), (1, -1),
        (-1,  0),          (1,  0),
        (-1,  1), (0,  1), (1,  1),
    ]

    for fx, fy in frontier_pixels:
        # Only consider valid frontier pixels
        if not (0 <= fx < width and 0 <= fy < height):
            continue
        if not path_planning.is_unseen(im, (fx, fy)):
            continue

        # Find free, reachable neighbours around this unseen frontier pixel
        for dx, dy in neighbour_offsets:
            gx, gy = fx + dx, fy + dy
            if 0 <= gx < width and 0 <= gy < height:
                if path_planning.is_free(im, (gx, gy)) and reachable[gy, gx]:
                    candidate_goals.append((gx, gy))

    # If we somehow didn't find any reachable neighbour goals, fall back
    if not candidate_goals:
        return robot_loc

    # -----------------------------
    # 3) Pick the reachable goal closest to the robot
    # -----------------------------
    rx, ry = robot_loc
    best_pt = None
    best_dist2 = None

    for gx, gy in candidate_goals:
        dx = gx - rx
        dy = gy - ry
        dist2 = dx * dx + dy * dy
        if best_dist2 is None or dist2 < best_dist2:
            best_dist2 = dist2
            best_pt = (gx, gy)

    return best_pt

def find_waypoints(im, path):
    """ Place waypoints along the path
    @param im - the thresholded image
    @param path - the initial path
    @ return - a new path"""

    # Again, no right answer here
    # YOUR CODE HERE
    if path is None:
        return None

    n = len(path)
    if n <= 2:
        # Already just start and goal.
        return path

    # Aim for roughly 8 waypoints (including start and goal).
    desired_waypoints = 8
    step = max(1, n // desired_waypoints)

    waypoints = []
    for idx in range(0, n, step):
        waypoints.append(path[idx])

    # Ensure the goal is included as the final waypoint.
    if waypoints[-1] != path[-1]:
        waypoints.append(path[-1])

    return waypoints

if __name__ == '__main__':
    im, im_thresh = path_planning.open_image("map.pgm")

    robot_start_loc = (1940, 1953)

    all_unseen = find_all_possible_goals(im_thresh)
    best_unseen = find_best_point(im_thresh, all_unseen, robot_loc=robot_start_loc)

    plot_with_explore_points(im_thresh, zoom=0.1, robot_loc=robot_start_loc, explore_points=all_unseen, best_pt=best_unseen)

    path = path_planning.dijkstra(im_thresh, robot_start_loc, best_unseen)
    waypoints = find_waypoints(im_thresh, path)
    path_planning.plot_with_path(im, im_thresh, zoom=0.1, robot_loc=robot_start_loc, goal_loc=best_unseen, path=waypoints)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt
    plt.show()

    print("Done")
