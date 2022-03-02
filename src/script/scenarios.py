# -*- coding: utf-8 -*-

from Classes.Circle import Circle
from Classes.Tool import Tool



scenario1 = [
            Tool(500, 470, 85),
            Circle(0, 500, 170, 50),
            Circle(1, 900, 170, 15),
            Circle(2, 900, 470, 85)
    ]

scenario2 = [
            Tool(500, 100, 15),
            Circle(0, 610, 400, 85),
            Circle(1, 720, 201, 100),
            Circle(2, 900, 616, 50)
    ]


scenario3 = [
            Tool(600, 100, 80),
            Circle(0, 388, 312, 60),
            Circle(1, 670, 594, 100),
            Circle(2, 882, 381, 15)
    ]


scenario4 = [
            Tool(612, 600, 15),
            Circle(0, 437, 80, 35),
            Circle(1, 650, 110, 100),
            Circle(2, 470, 237, 60)            
    ]


scenario5 = [
            Tool(620, 210, 60),
            Circle(0, 416, 550, 15),
            Circle(1, 387, 525, 25),
            Circle(2, 500, 426, 90)
    ]


SCENARIO = scenario5