import random
import math
import pygame

class RRTMap:
     def __init__(self, start, goal, MapDim, obsdim, obsnum):
          self.start = start
          self.goal = goal

          self.MapDim = MapDim
          self.Maph, self.Mapw = self.MapDim

          #window setting
          self.MapWindowName = 'RRT path planning'
          pygame.display.set_caption(self.MapWindowName)
          self.map = pygame.display.set_mode((self.Mapw, self.Maph))
          self.map.fill((255,255,255))
          self.nodeRad = 2
          self.nodeThickness = 0
          self.edgeThickness = 1
          self.obstacle = []
          self.obsdim = obsdim
          self.obsnum = obsnum

          #color
          self.grey = (70,70,70)
          self.blue = (0,0,255)
          self.green = (0,255,0)
          self.red = (255,0,0)
          self.white = (255,255,255)
          
          

     def drawMap(self, obstacles):
          pygame.draw.circle(self.map, self.green, self.start, self.nodeRad+5, 0)
          pygame.draw.circle(self.map, self.green, self.goal, self.nodeRad+20, 1)
          self.drawObs(obstacles)

     def drawPath(self, path):
          for i in path:
               pygame.draw.circle(self.map, self.red, i, self.nodeRad+3, 0)

     def drawObs(self, obstacles):
          obstaclesList = obstacles.copy()

          while(len(obstaclesList)>0):
               obstacle = obstaclesList.pop(0)
               pygame.draw.rect(self.map, self.grey, obstacle)

class RRTGraph:
     def __init__(self, start, goal, MapDim, obsdim, obsnum):
          (x,y) = start
          self.start = start
          self.goal = goal
          self.goalFlag = False # true when goal is reached
          self.maph, self.mapw = MapDim

          #variables to represent tree
          self.x = []
          self.y = []
          self.parent = []

          #initialising the start 
          self.x.append(x)
          self.y.append(y)
          self.parent.append(0)

          #obstacle
          obstackes = []
          self.obsdim = obsdim
          self.obsnum = obsnum

          #path
          self.goalState = None # flag to show if destination reached
          self.path = [] # list of calculation path
          

     def makeRandomRect(self):
          uppercornerx = int(random.uniform(0,self.mapw-self.obsdim))
          uppercornery = int(random.uniform(0,self.maph-self.obsdim))

          return (uppercornerx, uppercornery)

     def makeObs(self):
          obs = []

          for i in range (0, self.obsnum):
               rectang = None # holds variable before storing it in list
               startgoalcol = True # indicates start or goal pos are insided newly created obstacle
               while startgoalcol:
                    upper = self.makeRandomRect()
                    rectang = pygame.Rect(upper,(self.obsdim,self.obsdim))
                    if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                         startgoalcol = True
                    else:
                         startgoalcol = False
               obs.append(rectang)
          self.obstacles = obs.copy()
          return obs

     def add_node(self, n, x, y):
          self.x.insert(n, x)
          self.y.insert(n, y)
          
     def remove_node(self, n):
          self.x.pop(n)
          self.y.pop(n)

     def add_edge(self, parent, child):
          self.parent.insert(child, parent)

     def remove_edge(self, n):
          self.parent.pop(n)
          pass

     def number_of_nodes(self):
          return len(self.x)

     def distance(self, n1, n2):
          dist_x = float(self.x[n1]) - float(self.x[n2])
          dist_y = float(self.y[n1]) - float(self.y[n2])
          dist = ((dist_x)**2+(dist_y)**2)**(1/2)
          return dist

     def sample_envir(self):
          x = int(random.uniform(0, self.mapw))
          y = int(random.uniform(0, self.maph))
          return x,y
     
     def nearest(self, n):
          dmin = self.distance(0,n)
          nnear = 0

          for i in range (0,n):
               if self.distance(i, n) < dmin:
                    dmin = self.distance(i, n)
                    nnear = i
          return nnear
          
     def isFree(self):
          n = self.number_of_nodes() - 1
          x = self.x[n]
          y = self.y[n]
          
          obs = self.obstacles.copy()
          while len(obs) > 0:
               rectang = obs.pop(0)
               if rectang.collidepoint(x,y):
                    self.remove_node(n)
                    return False
          return True

     def crossObstacle(self, x1, y1, x2, y2):
          obs = self.obstacles.copy()
          while len(obs) > 0:
               rectang = obs.pop(0)
               for i in range (0,101):
                    u = i/100
                    x = x1*u + x2*(1-u)
                    y = y1*u + y2*(1-u)
                    if rectang.collidepoint(x, y):
                         return True
                    
          return False

     def connect(self, n1, n2):
          (x1, y1) = (self.x[n1], self.y[n1])
          (x2, y2) = (self.x[n2], self.y[n2])

          if self.crossObstacle(x1, y1, x2, y2):
               self.remove_node(n2)
               return False
          else:
               self.add_edge(n1, n2)
               return True

     def step(self, nnear, nrand, dmax=35):
          d=self.distance(nnear, nrand)
          if(d>dmax):
               (xnear, ynear) = (self.x[nnear], self.y[nnear])
               (xrand, yrand) = (self.x[nrand], self.y[nrand])

               (px, py) = ((xrand-xnear), (yrand-ynear))
               theta = math.atan2(py, px)
               (x, y) = (int(xnear+math.cos(theta)*dmax),
                         int(ynear+math.sin(theta)*dmax))
               self.remove_node(nrand)

               if abs(x-self.goal[0])<dmax and abs(y-self.goal[1])<dmax: #could be stated more clearly as distance between two points
                    if not self.crossObstacle(self.goal[0], self.goal[1], xnear, ynear):# correction 1 - to check there is path btw 
                         self.add_node(nrand, self.goal[0], self.goal[1])
                         print('Connected')
                         self.goalState = nrand
                         self.goalFlag = True
                    else:
                         self.add_node(nrand, x, y)

               else:
                    self.add_node(nrand, x, y)

     def extract_path(self):
          if self.goalFlag:
               self.path = []
               element = self.goalState
               while element != 0:
                    self.path.append(element)
                    element = self.parent[element]
               self.path.append(0)
          return self.goalFlag
     
     def path_to_goal(self):
          path_cords = []
          for i in self.path:
               path_cords.append((self.x[i], self.y[i]))
          return path_cords

     def getPathCords(self, new_node_x, new_node_y, x, y):
          path_to_nearest_x = []
          path_to_nearest_y = []

          path_to_nearest_x.append(x)
          path_to_nearest_y.append(y)
          
          for i in range (mod(x-new_node_x)):
               path_to_nearest_x.append(x+i)
               path_to_nearest_y.append(y+i)

          return path_to_nearest_x, path_to_nearest_y
          

     def bias(self, ngoal):
          n = self.number_of_nodes()
          self.add_node(n, ngoal[0], ngoal[1])
          nearest = self.nearest(n)
          self.step(nearest, n)
          self.connect(nearest, n)
          return self.x, self.y, self.parent
          
     def expand(self):
          n = self.number_of_nodes()
          x, y = self.sample_envir()
          self.add_node(n, x, y)
          if self.isFree():
               nnear = self.nearest(n)
               self.step(nnear, n)
               self.connect(nnear, n)
          return self.x, self.y, self.parent

     def cost(self):
          pass

     

     
