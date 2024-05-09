#Import all the necessary dependencies and libraries
import numpy as np
import cv2
import heapq
# import cv2.imshow
import time

"""**Step 1: Define the action cost set**"""

# Define the cost of each action
Action_set = {(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0,-1, 1), (1, 1, 1.4), (-1, 1, 1.4), (1,-1, 1.4), (-1,-1, 1.4)}

#efining the start time of the algorithem
start_time = time.time()

# Define the canvas height and canvas width
canvas_height = 500
canvas_width = 1200

# Make an empty canvas
canvas = 255 * np.ones((canvas_height, canvas_width, 3), dtype=np.uint8)

# Clearance threshold and color
clearance_threshold = 5
color = (230, 230, 250)  # Light color

# Draw rectangles along the edges for clearance
# Top edge
cv2.rectangle(canvas, (0, 0), (canvas_width, clearance_threshold), color, -1)
# Bottom edge
cv2.rectangle(canvas, (0, canvas_height - clearance_threshold), (canvas_width, canvas_height), color, -1)
# Left edge
cv2.rectangle(canvas, (0, 0), (clearance_threshold, canvas_height), color, -1)
# Right edge
cv2.rectangle(canvas, (canvas_width - clearance_threshold, 0), (canvas_width, canvas_height), color, -1)

#Defining the shape and formation of the top rectangle i.e. obstacle 1
# Initialize the obstacle's boundary with a margin for clearance
boundary_margin = np.array([[100 - clearance_threshold, 0],
                            [100 - clearance_threshold, 400 + clearance_threshold],
                            [175 + clearance_threshold, 400 + clearance_threshold],
                            [175 + clearance_threshold, 0]])
boundary_margin = boundary_margin.reshape(-1, 1, 2)
# Fill the area on the canvas for the obstacle with a clearance buffer
cv2.fillPoly(canvas, [boundary_margin], color)

# Configure the exact boundary points of the defined obstacle
obstacle_boundary = np.array([[100, 0], [100, 400], [175, 400], [175, 0]])
obstacle_boundary = obstacle_boundary.reshape(-1, 1, 2)
# Apply color fill within the obstacle's exact boundaries on the canvas
cv2.fillPoly(canvas, [obstacle_boundary], (0, 0, 0))

#Defining the shape and formation of the bottom rectangle i.e. obstacle 2
# Adjust the coordinates for the obstacle's boundary with clearance
obstacle_with_clearance = np.array([
    [275 - clearance_threshold, 500],
    [275 - clearance_threshold, 100 - clearance_threshold],
    [350 + clearance_threshold, 100 - clearance_threshold],
    [350 + clearance_threshold, 500]
])
# Convert the adjusted coordinates for use with fillPoly
obstacle_with_clearance = obstacle_with_clearance.reshape(-1, 1, 2)
# Fill the obstacle's boundary including clearance on the canvas
cv2.fillPoly(canvas, [obstacle_with_clearance], color)

# Original obstacle vertices
original_vertices = np.array([
    [275, 500],
    [275, 100],
    [350, 100],
    [350, 500]
])
# Reshape for fillPoly function
original_vertices = original_vertices.reshape(-1, 1, 2)
# Fill the original obstacle shape on the canvas with black color
cv2.fillPoly(canvas, [original_vertices], (0, 0, 0))

#Defining the shape and formation of the hexagon i.e. obstacle 3
# Define the center and side length of the hexagon with clearance
center = (650, canvas_height/2)
side_length = 150 + clearance_threshold

# Calculate the coordinates of the vertices for the rotated hexagon
vertices = []
for i in range(6):
    angle_deg = 60 * i + 330  # Adjusting angle for 30 degrees clockwise rotation
    angle_rad = np.deg2rad(angle_deg)
    x = int(center[0] + side_length * np.cos(angle_rad))
    y = int(center[1] + side_length * np.sin(angle_rad))
    vertices.append((x, y))

# Draw the hexagon by connecting the vertices
for i in range(6):
    cv2.line(canvas, vertices[i], vertices[(i + 1) % 6], (255, 255, 255), 2)

# Fill up the inside of the hexagon with a color
pts = np.array(vertices, np.int32)
pts = pts.reshape((-1, 1, 2))
cv2.fillConvexPoly(canvas, pts, color)

# Define the center and side length of the hexagon without clearance
center = (650, canvas_height/2)
side_length = 150

# Calculate the coordinates of the vertices for the rotated hexagon without clearance
vertices = []
for i in range(6):
    angle_deg = 60 * i + 330  # Adjusting angle for 30 degrees clockwise rotation
    angle_rad = np.deg2rad(angle_deg)
    x = int(center[0] + side_length * np.cos(angle_rad))
    y = int(center[1] + side_length * np.sin(angle_rad))
    vertices.append((x, y))

# Draw the hexagon by connecting the vertices
for i in range(6):
    cv2.line(canvas, vertices[i], vertices[(i + 1) % 6], (255, 255, 255), 2)

# Fill up the inside of the hexagon with a color
pts = np.array(vertices, np.int32)
pts = pts.reshape((-1, 1, 2))
cv2.fillConvexPoly(canvas, pts, (0, 0, 0))

# Define the points for OBSTACLE 4 with clearance adjustments
points_with_clearance = [
    [canvas_width - 300 - clearance_threshold, 50 - clearance_threshold],
    [canvas_width - 100 + clearance_threshold, 50 - clearance_threshold],
    [canvas_width - 100 + clearance_threshold, 450 + clearance_threshold],
    [canvas_width - 300 - clearance_threshold, 450 + clearance_threshold],
    [canvas_width - 300 - clearance_threshold, 375 - clearance_threshold], # 450 - 75
    [canvas_width - 180 -clearance_threshold, 375 - clearance_threshold], # 300 + 120
    [canvas_width - 180 - clearance_threshold, 125 + clearance_threshold],
    [canvas_width - 300 - clearance_threshold, 125 + clearance_threshold] # 180 - 120
]

# Adjust the shape for usage in cv2.fillPoly
adjusted_points = np.array(points_with_clearance, np.int32).reshape((-1, 1, 2))
# Fill the polygon on canvas with specified color
cv2.fillPoly(canvas, [adjusted_points], color)

# Define the points for OBSTACLE 4 without clearance adjustments
points_without_clearance = [
    [canvas_width - 300, 50],
    [canvas_width - 100, 50],
    [canvas_width - 100, 450],
    [canvas_width - 300, 450],
    [canvas_width - 300, 375], # 450 - 75
    [canvas_width - 180, 375], # 300 + 120
    [canvas_width - 180, 125],
    [canvas_width - 300, 125] # 180 - 120
]

# Adjust the shape for usage in cv2.fillPoly
adjusted_points_without_clearance = np.array(points_without_clearance, np.int32).reshape((-1, 1, 2))
# Fill the polygon on canvas with black color to delineate the obstacle without clearance
cv2.fillPoly(canvas, [adjusted_points_without_clearance], (0, 0, 0))

"""**STEP 3: GENERATE THE GRAPH AND CHECK FOR GOAL NODE IN EACH ITERATION**"""

# Loop until a valid start position is provided
while True:
    x_start = int(input('Input starting position of x - coordinate (between 0 and 1199): '))
    y_start = int(input('Input starting position of y - coordinate  (between 0 and 499): '))
    # Verify start position is not an obstacle
    if canvas[y_start, x_start, 0] != 255:
        print('Invalid starting postion. The coordinate points given are located inside the obstacle.')
    else:
        break  # Valid start found

# Loop until a valid goal position is provided
while True:
    x_goal = int(input('Input goal x - coordinate (between 0 and 1199): '))
    y_goal = int(input('Input goal y - coordinate(between 0 and 499): '))
    # Verify goal position is not an obstacle
    if canvas[y_goal, x_goal, 0] != 255:
        print('Invalid goal position. The coordinate points given are located inside the obstacle.')
    else:
        break  # Valid goal found

print("Starting postion and the goal position are valid. Commencing the node_pathfinding...")

# Initialize search
search_queue = []
heapq.heappush(search_queue, (0, x_start, y_start))

# Track visited nodes, their parents, and cost
visited_nodes = {(x_start, y_start): True}
node_parents = {(x_start, y_start): None}
node_path_cost = {(x_start, y_start): 0}

goal_reached = False

# Execute search until goal reached or queue is empty
while search_queue:
    current_cost, current_x, current_y = heapq.heappop(search_queue)

    # Check if current node is the goal
    if current_x == x_goal and current_y == y_goal:
        print('node_path to goal discovered.')
        goal_reached = True
        break

    # Explore neighboring nodes
    for dx, dy, move_cost in Action_set:
        next_x, next_y = current_x + dx, current_y + dy

        # Ensure move is within bounds and not into an obstacle
        if 0 <= next_x < canvas_width and 0 <= next_y < canvas_height and canvas[next_y, next_x, 0] == 255:
            # Proceed if new node or found a cheaper node_path to this node
            if (next_x, next_y) not in visited_nodes or node_path_cost[(next_x, next_y)] > current_cost + move_cost:
                heapq.heappush(search_queue, (current_cost + move_cost, next_x, next_y))
                visited_nodes[(next_x, next_y)] = True
                node_parents[(next_x, next_y)] = (current_x, current_y)
                node_path_cost[(next_x, next_y)] = current_cost + move_cost

# Handle unsuccessful search
if not goal_reached:
    print('Unable to reach goal. Terminating.')

"""**STEP 4: OPTIMAL node_path formation**"""

# Initialize the node_path list
node_path = []

# Starting point for the node_path tracing
x, y = x_goal, y_goal

# Use a for loop to iterate indefinitely until the break condition is met
for _ in iter(int, 1):
    # Append current node to the node_path
    node_path.append((x, y))

    # Break the loop if the start node is reached
    if (x, y) == (x_start, y_start):
        break

    # Move to the parent node
    x, y = node_parents[(x, y)]

# Since the loop does not inherently reverse the node_path, do it after the loop
node_path.reverse()

"""**STEP 5: REPRESENT THE OPTIMAL node_path formation**"""

# Illustrate start and goal positions with distinctive colors
cv2.circle(canvas, (x_start, y_start), radius=10, color=(0,139,139), thickness=-1)
cv2.circle(canvas, (x_goal, y_goal), radius=10, color=(139,69,19), thickness=-1)

# Initialize video recording in .mp4 format with specific parameters
video_output = cv2.VideoWriter('dijkstra.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps=50, frameSize=(canvas_width, canvas_height))

# Define a variable to regulate frame updates
update_frame_rate = 200
frame_count = 0

# Mark each visited node in red and update the video at specified intervals
for node in visited_nodes:
    frame_count += 1
    canvas[node[1], node[0]] = [240,230,140]  # Color visited nodes light cyan color
    if frame_count == update_frame_rate:
        cv2.imshow("canvas", canvas)
        video_output.write(canvas)
        cv2.waitKey(1)
        frame_count = 0  # Reset counter after update

# Reset counter for drawing the optimal node_path
update_frame_rate = 5
frame_count = 0

# Reiterate drawing start and goal positions
cv2.circle(canvas, (x_start, y_start), 10, (0,139,139), -1)
cv2.circle(canvas, (x_goal, y_goal), 10, (139,69,19), -1)

# Iterate over the node_path coordinates, drawing lines between consecutive nodes
for index in range(len(node_path) - 1):
    cv2.line(canvas, node_path[index], node_path[index + 1], (128,0,0), 2)  # Draw node_path in maroon color
    frame_count += 1
    if frame_count == update_frame_rate:
        cv2.imshow("canvas", canvas)
        video_output.write(canvas)
        cv2.waitKey(1)
        frame_count = 0  # Reset counter after update


# Finalize video recording and cleanup
video_output.release()
cv2.waitKey(0)
cv2.destroyAllWindows()

"""**Printing the total time taken to complete the algorithm**"""

# Record the end time
end_time = time.time()

# Calculate the runtime
runtime = end_time - start_time

print(f"Runtime: {runtime} seconds")
