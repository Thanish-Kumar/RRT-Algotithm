import pygame
from RRTbase import RRTGraph
from RRTbase import RRTMap

def main():
     dimensions = (600, 1000)
     start = (50, 50)
     goal = (510, 510)
     obsdim = 30
     obsnum = 50
     iteration = 0

     pygame.init()
     map_1 = RRTMap(start, goal, dimensions, obsdim, obsnum)
     graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)

     obstacles = graph.makeObs()
     map_1.drawMap(obstacles)

     while not graph.extract_path():
          
          if iteration%10 == 0:
               x, y, parent=graph.bias(goal)
               pygame.draw.circle(map_1.map, map_1.grey, (x[-1], y[-1]),
                                  map_1.nodeRad+2, 0)
               pygame.draw.line(map_1.map, map_1.blue, (x[-1], y[-1]),
                                (x[parent[-1]], y[parent[-1]]))
               
          else:
               x, y, parent = graph.expand()
               pygame.draw.circle(map_1.map, map_1.grey, (x[-1], y[-1]),
                                  map_1.nodeRad+2, 0)
               pygame.draw.line(map_1.map, map_1.blue, (x[-1], y[-1]),
                                (x[parent[-1]], y[parent[-1]]))

          if iteration%5 == 0:
               pygame.display.update()

          iteration += 1
          
     map_1.drawPath(graph.path_to_goal())
     pygame.display.update()
     pygame.event.clear(0)
     pygame.event.wait(0)

if __name__ == '__main__':
     main()
 
     
     
