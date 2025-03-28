


import math
import pygame
import tkinter as tk

import tkinter as tk
from tkinter import filedialog





min_angles = []

max_angles = []


def load_constraints(filename):
    min_angles.clear()
    max_angles.clear()
    with open(filename) as f:
        lines = f.readlines()
        for line in lines:
            max_angle, min_angle = line.split(',')
            min_angles.append(float(min_angle))
            max_angles.append(float(max_angle))
            print("Loaded: ", min_angle, max_angle)

            

class Elevator:
    h_p = 0.5
    height = 0
    height_velocity = 0
    a_p = 0.5
    angle = 0
    angle_velocity = 0

    def __init__(self, h_p, height, a_p, angle):
        self.h_p = h_p
        self.height = height
        self.a_p = a_p
        self.angle = angle
    
    def setSetpoint(self, target_height, target_angle):

        # Get the min and max angles for the given height
        min_angle, max_angle = get_constraints(self.height)

        print("Min angle: ", min_angle, "Max angle: ", max_angle)
        
        # Clamp the angle setpoint to the min and max angles
        if target_angle < min_angle:
            target_angle = min_angle
        if target_angle > max_angle:
            target_angle = max_angle


        height_error = target_height - self.height
        self.height_velocity = height_error * self.h_p

        angle_error = target_angle - self.angle
        self.angle_velocity = angle_error * self.a_p

    def update(self, dt):
        self.height += self.height_velocity * dt
        self.angle += self.angle_velocity * dt

        
    def getHeight(self):
        return self.height
    
    def getAngle(self):
        return self.angle

        
        

# UI for visualizing the elevator

def get_constraints(normalized_height):
    # Get the min and max angles for the given height
    min_angle = 0
    max_angle = 0

    floored_index = int(normalized_height * (len(min_angles) - 1))
    ceiled_index = int(math.ceil(normalized_height * (len(min_angles) - 1)))


    tweening_factor = normalized_height * (len(min_angles) - 1) - floored_index

    # Linearly interpolate the min and max angles
    min_angle = min_angles[floored_index] + (min_angles[ceiled_index] - min_angles[floored_index]) * tweening_factor
    max_angle = max_angles[floored_index] + (max_angles[ceiled_index] - max_angles[floored_index]) * tweening_factor

    return min_angle, max_angle


def main():

    filename = filedialog.askopenfilename(title="Select constraints file", filetypes=[("Text files", "*.txt")])

    load_constraints(filename)


    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Elevator Motion Planning Simulation")

    elevator = Elevator(5, 0, 10, 0)

    EE_LENGTH_NORMALIED = 0.2


    running = True
    
    getTicksLastFrame = pygame.time.get_ticks()

    
    height_setpoint = 0.5
    angle_setpoint = 30

    while running:

        t = pygame.time.get_ticks()
        # deltaTime in seconds.
        dt = (t - getTicksLastFrame) / 1000.0
        getTicksLastFrame = t

        print(dt)

        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False



        if pygame.key.get_pressed()[pygame.K_w]:
            height_setpoint += 0.5 * dt
        if pygame.key.get_pressed()[pygame.K_s]:
            height_setpoint -= 0.5 * dt
        if pygame.key.get_pressed()[pygame.K_a]:
            angle_setpoint -= 90 * dt
        if pygame.key.get_pressed()[pygame.K_d]:
            angle_setpoint += 90 * dt
            
                                    
        # Clamp the setpoints to the min and max angles
        if height_setpoint > 1:
            height_setpoint = 1
        if height_setpoint < 0:
            height_setpoint = 0
        if angle_setpoint > 90:
            angle_setpoint = 90
        if angle_setpoint < -90:
            angle_setpoint = -90


        # Set the setpoints
        elevator.setSetpoint(height_setpoint, angle_setpoint)
                  
        screen.fill((0, 0, 0))

        
        elevator.update(dt)

        # Draw the elevator

        pygame.draw.rect(screen, (200, 200, 200), (395, 100, 10, 450))


        ee_len = EE_LENGTH_NORMALIED * 450

        # Draw the end effector target
        
        target_height = height_setpoint * 450
        target_angle = angle_setpoint * math.pi / 180
        target_start_point = (400, 100 + 450 - target_height)
        target_end_point = (target_start_point[0] + ee_len * math.sin(target_angle), target_start_point[1] - ee_len * math.cos(target_angle))
        pygame.draw.lines(screen, (0, 245, 0), False, [target_start_point, target_end_point], 12)


        # Draw the end effector
        
        height = elevator.getHeight() * 450
        angle = elevator.getAngle() * math.pi / 180



        start_point = (400, 100 + 450 - height)
        end_point = (start_point[0] + ee_len * math.sin(angle), start_point[1] - ee_len * math.cos(angle)) 

        pygame.draw.lines(screen, (255, 255, 255), False, [start_point, end_point], 12)

        


        pygame.display.flip()
        
    pygame.quit()


if __name__ == "__main__":
    main()

        




