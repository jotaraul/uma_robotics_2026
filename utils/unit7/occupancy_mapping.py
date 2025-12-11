import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
from IPython.display import display, clear_output

import sys
sys.path.append("..")
from utils.laser.laser2D import Laser2D
from utils.DrawRobot import DrawRobot
from utils.tcomp import tcomp
from matplotlib.colors import ListedColormap



class OccupancyGridMap:
    
    def __init__(self, width, height, resolution):
        """Class constructor

        Parameters:
        width (int): Map width (in meters).
        height (int): Map height (in meters).
        resolution (float): Map resolution (in meters). For example if resolution=0.1 
            this means that the side of each cell will be 10 centimiters.
        
       """
        self.width = width
        self.height = height
        self.resolution = resolution
        
        # Grid map initialization as a numpy array (notice that the height defines the 
        # number of rows, and the width the number of columns)
        self.map = np.ones((int(height/resolution),
                                int(width/resolution)))
        
        # Log odds initialization (start with zero log-odds
        self.log_odds = np.zeros((int(height/resolution),
                                 int(width/resolution))) 

        # Probabilities initialization (start with 0.5)
        self.probabilities = np.ones((int(height/resolution),
                                int(width/resolution)))*0.5
        


def beams_endpoints(robot_pose, z, grid_map, max_attempts=10, d_after_obstacle=1):
    """
    Computes the endpoints of sensor beams in the world frame after extending beyond an obstacle,
    adjusting endpoints that fall outside the grid_map boundaries by reducing d_after_obstacle.

    Parameters:
    - robot_pose: The position and orientation of the robot [x, y, theta].
    - z: Array of observations from the sensor (each element contains distance and angle).
    - grid_map: The map containing width and height attributes that define boundaries.
    - max_attempts: Maximum number of attempts to adjust each endpoint.
    - d_after_obstacle: Initial extension distance in meters.

    Returns:
    - np.array: An array of adjusted endpoints in the world frame, with shape (2, n) for n beams.
    """
    theta_robot = robot_pose[2][0]
    x_robot, y_robot = robot_pose[0][0], robot_pose[1][0]
    
    # Map boundaries
    x_min, x_max = 0, grid_map.width - 1*grid_map.resolution
    y_min, y_max = 0, grid_map.height - 1*grid_map.resolution
    
    endpoints = []

    # Iterate over each beam in z
    for beam_distance, beam_angle in zip(z[0], z[1]):
        theta_i = theta_robot + beam_angle
        adjusted_d = d_after_obstacle  # Start with the initial d_after_obstacle for each beam

        for _ in range(max_attempts+1):
            # Calculate endpoint with the current adjusted_d
            x = x_robot + (beam_distance + adjusted_d) * np.cos(theta_i)
            y = y_robot + (beam_distance + adjusted_d) * np.sin(theta_i)

            # Check if the endpoint is within map boundaries
            if x_min <= x <= x_max and y_min <= y <= y_max:
                endpoints.append([x, y])
                break  # Found a valid endpoint, so exit the adjustment loop

            # Reduce d_after_obstacle if out of bounds
            adjusted_d -= d_after_obstacle/max_attempts 

        else: # Is executed only if the for is completed without a break (Corner case)
            # If all attempts fail, use the robot's position as a fallback endpoint
            endpoints.append([x_robot, y_robot])

    return np.array(endpoints).T  # Return in shape (2, n)
    

def ray_tracing(x0, y0, x1, y1):
    """
    Bresenham's Line Algorithm in 2D.
    Returns a generator of coordinate tuples along the line from (x0, y0) to (x1, y1).
    """

    # Round coordinates to the nearest integers to work within a discrete grid
    x0 = int(round(x0))
    y0 = int(round(y0))
    x1 = int(round(x1))
    y1 = int(round(y1))
    
    # Calculate differences in x and y between the start and end points
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    
    # Initialize starting point
    x, y = x0, y0

    # Determine the step direction in each axis (positive or negative)
    sx = 1 if x1 >= x0 else -1  # Step in x direction
    sy = 1 if y1 >= y0 else -1  # Step in y direction

    # Check if the line is more horizontal or vertical by comparing dx and dy
    if dy <= dx:
        # If dx is greater, we will iterate primarily in the x direction
        # Initialize error term to half of dx
        err = dx / 2.0
        while x != x1:  # Loop until we reach the end x-coordinate
            yield x, y  # Yield the current point in the line
            err -= dy  # Update error term by subtracting dy
            if err < 0:  # If error goes negative, adjust the y-coordinate
                y += sy  # Step in y direction
                err += dx  # Update error term by adding dx
            x += sx  # Step in x direction
        yield x, y  # Yield the final point (x1, y1)
    else:
        # If dy is greater, we will iterate primarily in the y direction
        # Initialize error term to half of dy
        err = dy / 2.0
        while y != y1:  # Loop until we reach the end y-coordinate
            yield x, y  # Yield the current point in the line
            err -= dx  # Update error term by subtracting dx
            if err < 0:  # If error goes negative, adjust the x-coordinate
                x += sx  # Step in x direction
                err += dy  # Update error term by adding dy
            y += sy  # Step in y direction
        yield x, y  # Yield the final point (x1, y1)



def get_cells_to_update(robot_pose, z_endpoints, resolution, grid_shape):
    """
    Determines which grid cells are intersected by sensor beams based on the robot's position, 
    beam endpoints, and map resolution. Only cells within the bounds of the grid map are returned.

    Parameters:
    - robot_pose (numpy array): The robot's pose in the environment as a column vector 
      with shape (3, 1), where robot_pose[0][0] is the x-coordinate and robot_pose[1][0] 
      is the y-coordinate.
    - z_endpoints (numpy array): Array of shape (2, n), where each column represents the x and y
      coordinates of the endpoint of a sensor beam, relative to the environment.
    - resolution (float): The size of each grid cell, used to convert real-world coordinates 
      into grid cell indices.
    - grid_shape (tuple): The shape of the grid map (height, width), used to validate cell indices.

    Returns:
    - cells_to_update (list): A list of lists, where each inner list contains the indices of 
      grid cells that each beam intersects and are within the grid boundaries.
    """
    
    # Initialize an empty list to store the cells to be updated for each beam
    cells_to_update = []

    # Convert the robot's position from world coordinates to grid cell indices
    robot_cell_indices = [int(robot_pose[0][0] / resolution), int(robot_pose[1][0] / resolution)]

    # Iterate over each beam endpoint in z_endpoints
    for beam_endpoint in z_endpoints.T:
        # Convert the beam endpoint from world coordinates to grid cell indices
        beam_endpoint_cell_indices = [int(beam_endpoint[0] / resolution), int(beam_endpoint[1] / resolution)]

        # Trace the line from the robot's cell to the beam endpoint cell, getting all cells along the path
        cells_indices = list(ray_tracing(robot_cell_indices[0], 
                                         robot_cell_indices[1], 
                                         beam_endpoint_cell_indices[0], 
                                         beam_endpoint_cell_indices[1]))

        # Filter cells that are within the bounds of the grid map
        valid_cells_indices = [
            (int(x), int(y)) for x, y in cells_indices
            if 0 <= int(x) < grid_shape[1] and 0 <= int(y) < grid_shape[0]
        ]

        # Append the list of valid cells for this beam path to the main list
        cells_to_update.append(valid_cells_indices)

    # Return the list of cells that need to be updated and are within bounds
    return cells_to_update


def inverse_beam_sensor_model(robot_pose, z, cells_to_update, resolution):
    """
    Computes the occupancy probabilities for cells traversed by each beam 
    in a laser observation, based on an inverse sensor model. 

    Parameters:   
    - robot_pose: np.array
        The pose of the robot as a 2D array, where the first row contains 
        the x-coordinate and the second row contains the y-coordinate of 
        the robot.
    - z: np.array
        A 2D array with the observation taken.
    - cells_to_update: list of lists
        A list where each element is a list of (i, j) cell indices that 
        correspond to the cells visited by a specific beam.
     - resoultion: resolution of the grid map being built

    Returns:
    - p_cell_occupied: list of lists
        A list with the same structure as `cells_to_update`, containing 
        the computed occupancy probabilities for each cell along each beam.
    """
    
    sigma = 0.4 # Standard deviation modeling sensor uncertainty       
    l_th = 0.2  # Lower probability threshold
    
    # Define robot position in terms of cells
    x = robot_pose[0][0] / resolution
    y = robot_pose[1][0] / resolution    
    
    # Number of beams in the observation
    n_beams = z.shape[1]

    # Initialize output list to store occupancy probabilities for each beam
    p_cell_occupied = [[] for _ in range(n_beams)]
    
    # Process each beam observation
    for beam_index in np.arange(n_beams):
        
        # The measured distance for the current beam in terms of cells
        z_i = z[0][beam_index] / resolution

        # Calculate occupancy probability for each cell traversed by the beam
        for cell in cells_to_update[beam_index]:
            
            # Compute Euclidean distance between the robot and the cell
            d = np.sqrt(np.power(x - cell[0], 2) + np.power(y - cell[1], 2))
            
            # Compute the value for the gaussian probability distribution
            f = (1 / (np.sqrt(2 * np.pi) * sigma)) * np.exp(-0.5 * np.power((d - z_i) / sigma, 2))            

            # Determine final occupancy probability based on conditions
            if (d < z_i) and (f < l_th):
                p_cell_occupied[beam_index].append(l_th)
            elif (d > z_i) and (f < 0.5):
                p_cell_occupied[beam_index].append(0.5)
            else:
                if f > 0.8:
                    f = 0.8
                p_cell_occupied[beam_index].append(f)

                    
    return p_cell_occupied


def update_log_odds(grid_map, cells_to_update, p_cells_occupied, l_0):
    """
    Updates the log-odds values of cells in the occupancy grid map based on new sensor observations.

    Parameters:
    - grid_map: object
        The occupancy grid map object that has an attribute `log_odds`, a 2D array representing
        the log-odds values of occupancy for each cell in the map.
    - cells_to_update: list of lists
        A list where each element is a list of (i, j) tuples representing the indices of cells 
        traversed by a specific beam. Each sublist corresponds to one beam in the observation.
    - p_cells_occupied: list of lists
        A list of lists containing the occupancy probabilities for each cell in each beam path.
        The structure matches `cells_to_update`, where each sublist corresponds to one beam.
    - l_0: float
        The prior log-odds value used to initialize cells. This value is added to each cell's 
        log-odds during updates to incorporate prior knowledge.

    This function updates `grid_map.log_odds` in-place, incorporating the latest observation data 
    and progressively refining the map’s understanding of which cells are likely occupied or free.
    """

    # Iterate over observation beams
    for beam_index in range(len(cells_to_update)):
        
        # Iterate over the cells traversed by this beam
        for traversed_cell, p_occupied in zip(cells_to_update[beam_index], p_cells_occupied[beam_index]):

            # Compute the sensor model log odd
            tau_t = np.log(p_occupied/(1-p_occupied))

            # Update cell log odds!
            grid_map.log_odds[traversed_cell[1]][traversed_cell[0]] += tau_t +  l_0


def update_probabilities(grid_map): 
    """
    Updates the occupancy probabilities in the grid map based on the current log-odds values.

    This function converts log-odds values stored in `grid_map.log_odds` to occupancy probabilities 
    and stores the result in `grid_map.probabilities`. It uses the sigmoid function to map log-odds 
    to probability values, where each cell's probability represents the likelihood of being occupied.

    Parameters:
    - grid_map: An object containing:
        - log_odds: A 2D numpy array with log-odds values for each cell in the occupancy grid.
        - probabilities: A 2D numpy array that will be updated with the computed occupancy probabilities.

    Note:
    This function modifies `grid_map.probabilities` in place.
    """
    grid_map.probabilities = 1 - 1 / (1 + np.exp(grid_map.log_odds))


def plot_state(robot_pose, z_world, virtual_map, grid_map, cells_to_update=None):
    """
    Plots the robot pose, map, beam endpoints, and optionally grid cells to be updated
    alongside the occupancy grid map.

    Parameters:
    - robot_pose: The position of the robot as a numpy array or list with shape (2, 1).
    - z_world: Array of beam endpoints in world coordinates.
    - virtual_map: The map to be plotted (e.g., walls or environment boundaries).
    - grid_map: Object containing occupancy probabilities for each cell in the grid.
    - cells_to_update (optional): List of lists of grid cell coordinates to update.
    - resolution: Resolution for converting cell coordinates to world coordinates (default is 1).
    """
    # with output:  # Direct output to the Output widget
    clear_output(wait=True)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Plot the virtual map (environment boundaries or walls) in the first subplot
    ax1.plot(virtual_map[0, :], virtual_map[1, :], 'k-')
    
    # Set grid and axis limits
    ax1.grid()
    ax1.set_xlim(np.nanmin(virtual_map[0])-2,np.nanmax(virtual_map[0])+2) # nanmin ignores nan numbers
    ax1.set_ylim(np.nanmin(virtual_map[1])-2,np.nanmax(virtual_map[1])+2)  
    
    # Title and axis labels
    ax1.set_title('Living-room Map')
    ax1.set_xlabel('X position (m)')
    ax1.set_ylabel('Y position (m)')   
            
    # Plot the beam endpoints in world coordinates
    ax1.plot(z_world[0], z_world[1], 'x')
    
    # If cells_to_update is provided, plot the grid cells to be updated
    if cells_to_update is not None:
        for cell_path in cells_to_update:
            ax1.plot([cell[0] * grid_map.resolution for cell in cell_path], 
                        [cell[1] * grid_map.resolution for cell in cell_path], 
                        's', markersize=5, color='blue', alpha=0.2)

        
    # Draw robot (the usual DrawRobot function doesn't work because of the subplots) 
    ax1.plot(robot_pose[0, 0], robot_pose[1, 0], 'o', color='red', markersize=10, alpha=0.6)
    orientation_length = 0.5  # Length of the orientation line
    x_start, y_start = robot_pose[0, 0], robot_pose[1, 0]
    x_end = x_start + orientation_length * np.cos(robot_pose[2, 0])
    y_end = y_start + orientation_length * np.sin(robot_pose[2, 0])
    ax1.plot([x_start, x_end], [y_start, y_end], color='red', linewidth=2, alpha=0.7)

    # Plot the occupancy grid map with probabilities in the second subplot
    cax = ax2.imshow(grid_map.probabilities, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    ax2.set_title('Occupancy Grid Map')
    ax2.set_xlabel('X (cells)')
    ax2.set_ylabel('Y (cells)')
    fig.colorbar(cax, ax=ax2, label='Occupancy Probability')

    plt.tight_layout()
    plt.show()
    plt.close(fig)  # Ensure the figure is closed after display to avoid lingering

def process_new_observation(grid_map, robot_pose, z):
    """
    Processes a new sensor observation to update the occupancy grid map.
    
    Parameters:
    - grid_map: An object representing the occupancy grid map. It should contain:
        - map.shape: Shape of the map grid to check boundary limits.
        - resolution: Resolution of each cell in the grid map.
        - log_odds: 2D numpy array of log-odds values for each cell.
    - robot_pose: The robot's current pose, represented as a numpy array of shape (3, 1)
      containing [x, y, theta].
    - z: Sensor observations,  representing distances and angles of detected obstacles.

    """
    z_endpoints = beams_endpoints(robot_pose, z, grid_map)  
    cells_to_update = get_cells_to_update(robot_pose, z_endpoints, grid_map.resolution, grid_map.map.shape)    
    p_cells_occupied = inverse_beam_sensor_model(robot_pose, z, cells_to_update, grid_map.resolution)
    l_0 = 0 # assume prior p_occupied == p_free
    update_log_odds(grid_map, cells_to_update, p_cells_occupied, l_0)
    update_probabilities(grid_map)    

    # Display control buttons and plot the updated map and robot state
    # plot_state(robot_pose, z_endpoints, virtual_map, grid_map, cells_to_update)


def plot_grid_map(grid_map):
    # Define thresholds
    lower_threshold = 0.3  # Probability below this will be set to 0
    upper_threshold = 0.7  # Probability above this will be set to 1

    # Update the map based on thresholds
    grid_map.map = np.full_like(grid_map.probabilities, 0.5)
    grid_map.map[grid_map.probabilities < lower_threshold] = 0
    grid_map.map[grid_map.probabilities > upper_threshold] = 1

    # Assume grid_map.map is already defined with values 0, 0.5, and 1
    # Define a simple color map for the three values, assigning to 0 the white color,
    # to 0.5 gray, and to 1 the black one
    cmap = ListedColormap(['white', 'gray', 'black'])  

    # Plot the occupancy grid map
    plt.imshow(grid_map.map, cmap=cmap, origin='lower', vmin=0, vmax=1)
    plt.title('Occupancy Grid Map')
    plt.xlabel('X (cells)')
    plt.ylabel('Y (cells)')

    # Add grid overlay
    plt.gca().set_xticks(np.arange(-0.5, grid_map.map.shape[1], 1), minor=True)
    plt.gca().set_yticks(np.arange(-0.5, grid_map.map.shape[0], 1), minor=True)
    plt.gca().grid(which='minor', color='black', linestyle='-', linewidth=0.1)

    # Add a simple legend to represent the three values
    labels = ['Unoccupied (0)', 'Unknown (0.5)', 'Occupied (1)']
    colors = cmap.colors
    for color, label in zip(colors, labels):
        plt.plot([], [], 's', color=color, label=label)

    plt.legend(loc='upper left')
    plt.tight_layout()
    plt.show()
