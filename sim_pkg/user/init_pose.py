'''
Firefly

Function to initialize agent's poses.
Input:
    swarmsize --  swarmsize.
    x -- An array to store  agents' x positions, the length of this array is the same as swarmsize
    y -- An array to store agents' y positions, the length of this array is the same as swarmsize
    theta -- An array to store agents' orientations, the length of this is the same as swarmsize

Usage:
    Usr can configure an agent's initial x, y, theta by modifying the value of the corresponding element in array x, y, and theta.
    For example, initialize agent 0's pose to x = 0, y = 1, theta = 2:
    x[0] = 0
    y[0] = 1
    theta[0] = 2

Constraints to be considered:
    x -- the value should range between -2.5 to 2.5.
    y -- the value should range between -1.5 to 1.5.
    theta -- the value should range between -pi to pi.

    The minimal pairwise inter-agent distance should be greater than 0.12

def init(swarmsize, x, y, theta, a_ids):
    import math
    import random
    for i in range(swarmsize):
        x[i] = (i % 16 ) * 0.11-1
        y[i] = (i / 16 ) * 0.11-1
        a_ids[i] = 0
        theta[i] = 0
        if i==0:
            a_ids[i]=1
        elif i==15:
            a_ids[i]=2
    pass
'''
def init(swarmsize, x, y, theta, a_ids):
    import math
    import random
    import csv
    
    overlap_radius = 0.26

    def check_overlap(current_x, current_y, index):
        # Check for overlap with previously generated positions
        for i in range(index):
            distance = (current_x - x[i])**2 + (current_y - y[i])**2
            if distance < overlap_radius:
                return True  # Overlapping positions found
            # if current_x == x[i] and current_y == y[i]:
            #     return True  # Overlapping positions found
        return False  # No overlap
    
    for i in range(swarmsize):
        # Generate random positions until no overlap is found
        while True:
            x[i] = random.uniform(-4, 4)
            y[i] = random.uniform(-4, 4)
            if not check_overlap(x[i], y[i], i):
                break  # No overlap, exit the loop

        a_ids[i] = i
        theta[i] = random.uniform(-math.pi, math.pi)
    
    # # Write to csv file
    # rows = zip(a_ids, x, y, theta)
    # with open('user/init_pose.csv', 'w', newline='') as csv_file:
    #     writer = csv.writer(csv_file)

    #     # Write the rows
    #     writer.writerows(rows)

    return x, y, theta, a_ids