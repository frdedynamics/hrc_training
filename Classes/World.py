#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pygame
import os
from data_logger import enable_logging, disable_logging


class World:
    __RES_DIR = "res"
    BG = pygame.image.load(os.path.join(__RES_DIR, "background.png"))
    BG_COLOR = (181, 190, 204)  # RGB
    STATE_START = 0
    
    def __init__(self, marks, tool, hud, logging=True):
        """
        @param marks: An ordered list of Circle objects that the tool should reach (in the same order)
        @param tool: The tool object that is being controlled
        @param hud: A HUD object to update and display on screen
        """
        self.marks = marks
        self.tool = tool
        self.tool.setTarget(self.marks[0])
        self.hud = hud
        self.state = World.STATE_START
        self.stateEnd = len(self.marks)
        self.marks[0].setMarked()
        self.elapsed_time = 0

        if logging:
            enable_logging()
            self.tool.setTimingFunc(self.getElapsedTime)
            self.tool.enableLogging()

    def getElapsedTime(self):
        return self.elapsed_time
        
    def update(self):
        """Update the world"""
        self.updateState()
        self.updateHud()
    
    def updateState(self):
        """Updates the current state of the world"""
        if self.state < self.stateEnd:
            if self.tool.collidesWith(self.marks[self.state]):
                self.marks[self.state].setCollided()
                self.state += 1
                if self.state < self.stateEnd:
                    self.marks[self.state].setMarked()
                    self.tool.setTarget(self.marks[self.state])
                else:
                    disable_logging()
                    self.tool.disableLogging()
    
    def updateHud(self):
        """Updates the HUD when appropriate"""
        # update elapsed time
        if not self.isGameOver() and self.tool.isConnected():
            self.elapsed_time = self.tool.getElapsedTime()
        # text for the next mark
        if self.state < self.stateEnd:
            state = self.state + 1
        else:
            state = self.stateEnd
        # update HUD's text
        self.hud.setText(self.elapsed_time, self.tool.getLastCalculatedDistance(), state)
                
    def isGameOver(self):
        """Returns True when the game is over (endoscope has collided with mark 2)"""
        return self.state == self.stateEnd
    
    def draw(self, surface):
        # surface.fill(World.BG_COLOR)  # fill screen with blank color
        surface.blit(World.BG, (0,0))
        for mark in self.marks:
            mark.draw(surface)
        self.tool.draw(surface)
        self.hud.draw(surface)
        

class HUD:
    COLOR = [0, 0, 0]

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.color = HUD.COLOR
        self.font = pygame.font.SysFont('Arial', 12)
        self.text = None
        self.setText(0, 0, 0)
        
    def setText(self, time, distance, mark_number):
        text = "Time: %.03fs             Distance to [mark %d]: %d" % (time, mark_number, distance)
        self.text = self.font.render(text, False, self.color)
        
    def draw(self, surface):
        surface.blit(self.text, (self.x, self.y))

