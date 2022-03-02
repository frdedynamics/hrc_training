# -*- coding: utf-8 -*-
import pygame
import math
import random
import os


class Circle(object):
    __RES_DIR = "res"
    COLOR_UNSET = (255, 0, 0)
    COLOR_SET = (0, 255, 0)
    # the maximum distance in pixels between the center of two circles 
    # for them to be considered 'collided'
    COLLISION_DISTANCE_THRESHOLD = 6
    # the maximum differen in pixels between the radius two circles 
    # for them to be considered 'collided'
    COLLISION_RADIUS_THRESHOLD = 4
    CIRCLE_IMG_RED = pygame.image.load(os.path.join(__RES_DIR, "Red_icon.png"))
    CIRCLE_IMG_YELLOW = pygame.image.load(os.path.join(__RES_DIR, "Yellow_icon.png"))
    CIRCLE_IMG_GREEN = pygame.image.load(os.path.join(__RES_DIR, "Green_icon.png"))
    CIRCLE_IMG_GREY = pygame.image.load(os.path.join(__RES_DIR, "Grey_icon.png"))
    CIRCLE_IMG_BLACK = pygame.image.load(os.path.join(__RES_DIR, "Black_icon.png"))
    
    def __init__(self, idx, x, y, radius):
        """
        @param position: A list in the shape [x,y]
        @param radius: The radius of the circle
        """
        self.idx = idx
        self.x = x
        self.y = y
        self.r = radius
        self.img = Circle.CIRCLE_IMG_RED
        self.prevCalculatedDistance = 0

    def distanceTo(self, other):
        """Returns the Euclidian distance between the center of this Cirlce and the given Circle"""
        # we save the result in prevCalculatedDistance so we can use it again
        # without having to redo the calculations (used by the HUD)
        self.prevCalculatedDistance = math.sqrt((self.y - other.y)**2 + (self.x - other.x)**2)
        return self.prevCalculatedDistance
    
    def getLastCalculatedDistance(self):
        return self.prevCalculatedDistance

    def collidesWith(self, other):
        """
        Checks if the given Circle object collides with this Circle
        @param other: The other circle object to compare with
        @returns True if the given Circle collides with this Circle
        @returns False if the given Circle doesn't collide with this Circle
        """
        return (
                self.distanceTo(other) <= Circle.COLLISION_DISTANCE_THRESHOLD 
                and abs(self.r - other.r) <= Circle.COLLISION_RADIUS_THRESHOLD
        )

    def setCollided(self):
        """
        Set the current Circle as collided;
        Change its color to show that it has been reached
        """
        self.img = Circle.CIRCLE_IMG_GREEN
        
    def setMarked(self):
        """
        Set the current Circle as a Mark;
        Changes its color to show that it is the current goal that the
        endoscope tool should reach
        """
        self.img = Circle.CIRCLE_IMG_YELLOW
        
    def __str__(self):
        return "[Circle %d] X: %d, Y: %d, r: %d" % (self.idx, self.x, self.y, self.r)
    
    def draw(self, surface):
        width = self.r * 2
        origin = (self.x - self.r, self.y - self.r)
        img = pygame.transform.scale(self.img, (width, width))
        surface.blit(img, origin)
        
