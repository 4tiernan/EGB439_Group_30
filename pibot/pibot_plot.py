import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import numpy as np
from navigation import generate_bernoulli

class Bot_Plotter():
    def __init__(self, bot): 
        plt.ion()  # ← turn on interactive mode — opens window without blocking

        self.plt = plt
        self.bot = bot
        pose = self.bot.getLocalizerPose(group_number=30)
        # Figure / axes
        self.fig, _ = plt.subplots(1, 1)
        self.axes = self.fig.axes[0]

        
        self.path = [[pose[0], pose[1]]]
        self.arena_size = 2.0 # meters

        self.axes.set_xlim(-0.1, self.arena_size + 0.1)
        self.axes.set_ylim(-0.1, self.arena_size + 0.1)
        self.axes.set_aspect('equal')
        self.axes.set_title("PiBot Simulator")
        self.axes.set_xlabel("X (m)")
        self.axes.set_ylabel("Y (m)")

        # Draw arena once — never needs to change
        border = plt.Polygon(
            [[0,0],[self.arena_size,0],[self.arena_size,self.arena_size],[0,self.arena_size]],
            closed=True, fill=False, edgecolor='black', linewidth=2
        )
        self.axes.add_patch(border)

        self.desired_arrow = FancyArrowPatch(
            (0, 0), (0, 0),
            arrowstyle='->',
            color='green',
            linewidth=2,
            mutation_scale=15,
            label='Desired Heading'
        )

        self.axes.add_patch(self.desired_arrow)

        # Create artists that will be updated each frame
        self.path_line,    = self.axes.plot([], [], 'b-', linewidth=1, label='Path')
        self.robot_fill    = self.axes.fill([], [], color='red', alpha=0.7)[0]
        self.robot_outline,= self.axes.plot([], [], 'r-')
        self.localiser_dot,= self.axes.plot([], [], 'k+', markersize=8, label='Localiser')

        path = generate_bernoulli()
        x = path[0]
        y = path[1]

        self.figure_eight_path,         = self.axes.plot(x, y)
                
        # Mark the start position with a green dot
        self.axes.plot(pose[0], pose[1], 'go', markersize=6, label='Start')
        # Legend for start position
        self.axes.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=3, fontsize=8)

    def update(self, desired_heading=None):
        # Update path
                
        #self.path_line.set_data(self.path_x, self.path_y)

        # Update robot triangle
        #pose = self.bot.getLocalizerPose(group_number=30)
        pose = self.bot.pose
        x, y, theta = pose
        self.path.append([pose[0], pose[1]])
        L = 0.08
        triangle = np.array([[L, 0], [-L/2, L/2], [-L/2, -L/2], [L, 0]])
        R = np.array([[np.cos(theta), -np.sin(theta)],
                    [np.sin(theta),  np.cos(theta)]])
        tri = (R @ triangle.T).T + np.array([x, y])
        self.robot_fill.set_xy(tri[:-1])
        self.robot_outline.set_data(tri[:, 0], tri[:, 1])

        if desired_heading is not None:
            arrow_length = 0.3

            dx = arrow_length * np.cos(desired_heading)
            dy = arrow_length * np.sin(desired_heading)

            self.desired_arrow.set_positions(
                (x, y),
                (x + dx, y + dy)
            )

        # Update localiser
        lx, ly, _ = pose
        self.localiser_dot.set_data([lx], [ly])
        self.path_line.set_data([point[0] for point in self.path], [point[1] for point in self.path])

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()