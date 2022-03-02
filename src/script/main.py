#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import pygame

from Classes.World import World, HUD

import data_logger
from scenarios import SCENARIO


#%% Globals
screen = None

#%% Constants
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

#%% Setup PyGame
def init():
    global screen
    # intiazlize pygame
    pygame.init()
    # create the window in the middle of the screen
    info = pygame.display.Info()
    center_x = info.current_w / 2
    center_y = info.current_h / 2
    x_pos = center_x - (WINDOW_WIDTH / 2)
    y_pos = center_y - (WINDOW_HEIGHT / 2)
    # set the position of the window
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x_pos, y_pos)
    # create pygame window
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

#%% Setup the World and the containing objects
def setup_world():
    tool = SCENARIO[0]
    marks = SCENARIO[1:]
    tool.start()
    hud = HUD(10, WINDOW_HEIGHT - 22)
    world = World(marks, tool, hud)
    return world

#%% Game Over
def end_training(world):
    """Continue to allow movement of tool but pause HUD"""
    world.tool.setColorBlack()
    while True:
        # check for events
        for event in pygame.event.get():    
            if event.type == pygame.QUIT:
                print("Bye Bye")
                data_logger.disable_logging()
                world.tool.disconnect()
                world.tool.stop()
                pygame.quit()
                return
        
        # update graphics
        world.draw(screen)
        # switch graphics buffer (show the new changes we just draw)
        pygame.display.flip()
    

#%% Game Loop
def game_loop():
    world = setup_world()
    
    while not world.isGameOver():
        # check for events
        for event in pygame.event.get():    
            if event.type == pygame.QUIT:
                print("Bye Bye")
                data_logger.disable_logging()
                world.tool.disconnect()
                world.tool.stop()
                pygame.quit()
                return
                
        # update World state
        world.update()
        
        # update graphics
        world.draw(screen)
        
        # switch graphics buffer (show the new changes we just draw)
        pygame.display.flip()
        
    end_training(world)
    
    
    
#%% Main
if __name__ == "__main__":
    init()
    game_loop()
