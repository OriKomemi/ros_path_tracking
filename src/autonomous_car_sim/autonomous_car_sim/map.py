import numpy as np
import matplotlib.pyplot as plt
from fsd_path_planning import ConeTypes
from sub_modules.fsds.types import Point2D
from typing import List, Dict

class Map:
    """
    simple map container
    """

    def __init__(self, initial_position: Point2D, cones: List):

        self.initial_position = initial_position
        self.initial_position.x /= 100.0
        self.initial_position.y /= 100.0    
        self.cones = cones
        self.cones_by_colors = {0: [], 1: [], 2: [], 3: [], 4: []}
        self.color_map = {0: "k", 1: "gold", 2: "royalblue", 3: "peru", 4: "darkorange"}
        self.label_map = {
        0: "Unknown",
        1: "Right (yellow)",
        2: "Left (blue)",
        3: "Orange small",
        4: "Orange big",  
        }
        self.organize_cones_by_color()
        self.cones_by_type = self.prepare_cones_for_planner()
    
    def organize_cones_by_color(self):
        for cone in self.cones:
            # Support both dict-like and attr-like cone objects
            color = cone["color"] # a numner from 0 to 4 probably
            x = cone["x"] / 100.0
            y = cone["y"] / 100.0

            # Make position relative to initial_position and convert cm -> m
            px = (x - self.initial_position.x) 
            py = (y - self.initial_position.y) 
            self.cones_by_colors[color].append((px, py))


    def prepare_cones_for_planner(self):
        # Prepare cones by type for path planning
        # cones_by_colors - the cones organized by color(5 colors = 5 lists)
        # cones_by_type - the cones organized by ConeTypes (5 types = 5 lists)
        cones_by_type = [np.zeros((0, 2)) for _ in range(len(ConeTypes))] # the outer list has 5 elements, one for each ConeTypes, then each element is an array of shape (N, 2)
        cones_by_type[ConeTypes.LEFT] = np.array(self.cones_by_colors[ConeTypes.BLUE])
        cones_by_type[ConeTypes.RIGHT] = np.array(self.cones_by_colors[ConeTypes.YELLOW])
        cones_by_type[ConeTypes.START_FINISH_LINE] = np.array(self.cones_by_colors[ConeTypes.ORANGE_BIG]) 
        cones_by_type[ConeTypes.START_FINISH_AREA] = np.array(self.cones_by_colors[ConeTypes.ORANGE_SMALL])
        return cones_by_type   

    def get_initial_position(self):
        return (self.initial_position.x , self.initial_position.y)
    

    def get_cones_by_colors(self):
        return self.cones_by_colors
    
    def plot_cones_by_color(self):
        fig, ax = plt.subplots()
        for color_number, points in self.cones_by_colors.items():
            if not points:
                continue
            points = np.array(points)
            ax.scatter(points[:, 0], points[:, 1], s=20, c=self.color_map[color_number], label=self.label_map[color_number])

        ax.scatter(self.initial_position.x, self.initial_position.y, s=30, c="deeppink", label="speedy meCwin", marker='^')    
        ax.set_xlabel("X [m] (relative to start)")
        ax.set_ylabel("Y [m] (relative to start)")
        ax.set_aspect("equal", adjustable="box")
        ax.legend()
        ax.grid(True)
        ax.set_title("Referee cones")
        plt.show()