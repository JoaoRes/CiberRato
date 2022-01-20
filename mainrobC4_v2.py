
from os import write
import sys
from typing import DefaultDict
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from astar import *
from itertools import permutations
import math

CELLROWS=7
CELLCOLS=14



class MyRob(CRobLinkAngs):
    calculate = True
    visited = set()
    notTaken = set()
    walls = set()
    posinitial = ()
    target = ()
    prevTarget = ()
    mypos= ()
    myorient = ()
    nextorient = ()
    d = {(28,14): '0'}
    neighbors = [(0,2),(0,-2),(2,0),(-2,0)]
    path = list()
    dictionary_noTaken= dict()
    havepath = False
    goals = {}
    out = (0,0)
    out_1 = (0,0)
    gps = (0,0)
    prev_gps = (0,0)
    gps_robo = (0,0)
    teta = 0
    prev_teta=0
    in_left=0
    in_right=0

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'end'

        self.readSensors()
        while True:
            self.readSensors()
            if self.measures.endLed:
                self.mapWriting()
                print(self.robName + " exiting")
                (x, y) = (round(self.mypos[0]),round(self.mypos[1]))
                if self.measures.ground != -1 and (x,y)!=(0,0): 
                    self.goals[(x,y)] = self.measures.ground
                g = list(self.goals.keys())
                aux = []
                for i in range(0,len(g)):
                    aux.append(i)
                perm = list(permutations(aux))
                finPath= []
                firstP = []
                for i in perm:
                    aux1 = astar((0,0),g[i[0]],self.visited,self.walls)
                    aux2 = aux1.copy()
                    old = g[i[0]]
                    for j in range(1,len(i)):
                        aux1 = astar(old,g[i[j]],self.visited,self.walls) + aux1
                        old = g[i[j]]
                    aux1 = astar(old,(0,0),self.visited,self.walls) + aux1
                    if len(finPath)==0 or len(finPath) > len(aux1):
                            firstP = aux2.copy()
                            finPath = aux1.copy()
                    elif len(finPath)==len(aux1) and (len(firstP)==0 or len(firstP)>=len(aux2)):
                            firstP = aux2.copy()
                            finPath = aux1.copy()
                
                file= open("path.txt", 'w')
                file.write('0 0')
                file.write('\n')
                for i in reversed(finPath):
                    file.write(str(i[0]) + " "+ str(i[1]))
                    if (i[0], i[1]) in list(self.goals.keys()):
                        num = self.goals[(i[0], i[1])]
                        file.write(' #'+str(num))
                    file.write('\n')
                
                quit()
            if state == 'stop' and self.measures.start:
                self.mypos = (0,0)
                self.target = (0,0)
                self.prevTarget = (0,0)
                self.posinitial = (self.measures.x, self.measures.y)
                self.gps_robo = (0,0)
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'go':
                self.myorient = self.correctCompass()
                self.gps_robo = (self.measures.x - self.posinitial[0], self.measures.y- self.posinitial[1])
                # print('POSICAO INICIAL', self.posinitial)
                # print('MINHA POSICAO', self.mypos)
                # print('TARGET',self.target)
                # print('\n')

                #print(self.target)
                if self.measures.ground==0:
                    self.setVisitingLed(True)

                if self.reached(self.mypos,self.target):
                    self.prevTarget = self.target
                    self.visited.add(self.prevTarget)
                    # print("VISITED NODES", self.visited)
                    # print("NOT VISITED NODES", self.notTaken)
                    state= 'end'
                else:
                    # print('BUSSULA',self.measures.compass)
                    # print('CORRECAO DA BUSSOLA',self.correctCompass())
                    if self.correctCompass() == 180:
                        if self.measures.compass < 0:
                            self.straight(0.10,self.measures.compass,0.01,self.correctCompass())
                        else: 
                            self.straight(0.10,self.measures.compass,0.01,self.correctCompass())
                    else:
                        self.straight(0.10,self.measures.compass,0.01,self.correctCompass())
            if state== 'rotate right':
                # print("ESTOU A RODAR")
                if self.nextorient == ():
                    if self.correctCompass() == 180 or self.correctCompass() == -180: 
                        self.nextorient = 90
                    else:
                        self.nextorient = self.myorient-90
                    # print("OBJETIVO", self.nextorient)
                elif abs(self.measures.compass - self.nextorient) <= 10:
                    self.myorient= self.nextorient
                    self.nextorient = ()
                    state = 'end'
                else:
                    # print(self.measures.compass)
                    self.driveMotors(0.05, -0.05)
                    self.in_right=0
                    self.in_left=0
                    #self.out = ( (0 + self.out_1[0]) / 2 , (0 + self.out_1[1]) / 2 )
                    
            if state == 'rotate left':
                if self.nextorient == ():
                    if self.correctCompass() == 180 or self.correctCompass() == -180: 
                        self.nextorient = -90
                    else:
                        self.nextorient = self.myorient+90
                    # print("OBJETIVO", self.nextorient)
                elif abs(self.measures.compass - self.nextorient) <=10 :
                    self.myorient= self.nextorient
                    self.nextorient = ()
                    state = 'end'
                else:
                    # print(self.measures.compass)
                    self.driveMotors(-0.05,0.05)     
                    self.in_right=0
                    self.in_left=0
                    #self.out = ( (0 + self.out_1[0]) / 2 , (0 + self.out_1[1]) / 2 )        
                    
            if state == 'rotate mazespin':
                # print("ESTOU A RODAR")
                if self.nextorient == ():
                    if self.correctCompass() == 90 or self.correctCompass() == -90: 
                        self.nextorient = - self.correctCompass()
                        # print("OBJETIVO", self.nextorient)
                    elif self.correctCompass == -180:
                        self.nextorient = 0
                    elif self.correctCompass() == 0:
                        self.nextorient = 180
                    else: 
                        self.nextorient=0
                    
                elif abs(self.measures.compass - self.nextorient) <= 10:
                    self.myorient= self.nextorient
                    self.nextorient = ()
                    state = 'end'
                else:
                    # print(self.measures.compass)
                    self.driveMotors(-0.05,0.05)
                    self.in_right=0
                    self.in_left=0
                    #self.out = ( (0 + self.out_1[0]) / 2 , (0 + self.out_1[1]) / 2 )
            if state == 'end':
                print("ANTES DA CORREÇÃO ->" , self.mypos)
                self.gps_Correction(self.checkwalls())
                print("DEPOIS DA CORREÇÃO -> ", self.mypos)
                if self.calculate== True:
                    #print("Print Walls", self.checkwalls())
                    
                    if self.havepath == True:
                        # print("PATH -> " ,self.path)
                        self.target = self.path.pop()
                        # print("TARGET LA DE CIMA-> ",self.target)
                        #print("TARGET", self.target)
                        if len(self.path)== 0:
                            self.havepath= False
                    else:
                        self.compass_orientation(self.checkwalls())

                state = self.next_move(self.prevTarget, self.target)

            self.mypos = self.my_gps(self.prev_gps[0],self.prev_gps[1]) 

            if self.measures.time == 5995: 
                self.finish()

            # print("PATH-> ", self.path)
            #print("MY POS CALCULADO-> ", round(self.mypos[0],3) , round(self.mypos[1],3))
            #print("MY POS GPS ->" , (round(self.gps_robo[0],3) ,round(self.gps_robo[1],3)))
            #print("TEMPO ->", self.measures.time)
            # print("TARGET -> ", self.target)
            #print("ORIENTACAO -> ", self.correctCompass())

            
                
        


    def next_move(self, position, target):
            # calcular next state com base no target                self.driveMotors(0,0)
            diff = position[0] - target[0], position[1] - target[1]
            #print("DIFF -> ", diff)
            #print("ORIENTACAO -> ", self.correctCompass())

            if self.correctCompass() == 0:  # direita
                if diff[0] == -2 and diff[1] == 0:
                    self.calculate = False
                    return "go"

                elif diff[0] == 0 and diff[1] == -2:
                    self.calculate = False
                    return "rotate left"

                elif diff[0] == 0 and diff[1] == 2:
                    self.calculate = False
                    return "rotate right"
                else:
                    self.calculate = False
                    return "rotate mazespin"

            elif self.correctCompass() == 90:  # cima
                if diff[0] == -2 and diff[1] == 0:
                    self.calculate = False
                    return "rotate right"

                elif diff[0] == 0 and diff[1] == -2:
                    self.calculate = False
                    return "go"

                elif diff[0] == 2 and diff[1] == 0:
                    self.calculate = False
                    return "rotate left"
                else:
                    self.calculate = False
                    return "rotate mazespin"

            elif self.correctCompass() == -90:  # baixo
                if diff[0] == 0 and diff[1] == 2:
                    self.calculate = False
                    return "go"

                elif diff[0] == 2 and diff[1] == 0:
                    self.calculate = False
                    return "rotate right"

                elif diff[0] == -2 and diff[1] == 0:
                    self.calculate = False
                    return "rotate left"
                else: 
                    self.calculate = False
                    return "rotate mazespin"

            elif self.correctCompass() == 180 or self.correctCompass() == -180:  # esquerda
                if diff[0] == 2 and diff[1] == 0:
                    self.calculate = False
                    #print("DEBUG 2")
                    return "go"

                elif diff[0] == 0 and diff[1] == 2:
                    self.calculate = False
                    return "rotate left"

                elif diff[0] == 0 and diff[1] == -2:
                    self.calculate = False
                    return "rotate right"
                else:
                    self.calculate = False
                    return "rotate mazespin"

            return None
    
    def distSensor(self, wall):
        return 1/self.measures.irSensor[wall]

    def gps_Correction(self, walls):
        center_id = 0
        right_id = 1
        left_id = 2
        back_id = 3

        (x,y) = (round(self.mypos[0]), round(self.mypos[1]))
        if ((x,y) == (0,0)):
            return

        if self.correctCompass()==0:
            if walls[0]==1:
                wall_position = (x+1, y)
                self.mypos = (wall_position[0]-self.distSensor(center_id)-0.5, wall_position[1])

            if walls[1] == 1:
                wall_position = (x, y-1)
                print("WALL POSITION", wall_position)
                print(self.distSensor(right_id))
                self.mypos = (wall_position[0], wall_position[1] + self.distSensor(right_id) + 0.5)
                print("HERE")
            elif walls[2]==1:
                wall_position = (x, y+1)
                self.mypos = (wall_position[0], wall_position[1] - self.distSensor(left_id) - 0.5)
                print("THERE")

        if self.correctCompass()==90:
            if walls[0]==1:
                wall_position = (x, y+1)
                self.mypos = (wall_position[0], -self.distSensor(center_id)-0.5 + wall_position[1])

            if walls[1] == 1:
                wall_position = (x-1, y)
                self.mypos = (wall_position[0] + self.distSensor(right_id) + 0.5, wall_position[1])
            elif walls[2]==1:
                wall_position = (x+1, y)
                self.mypos = (wall_position[0] - self.distSensor(left_id) - 0.5,  wall_position[1])
            # if walls[0]==1:
            #     wall_position = (self.prevTarget[0], self. prevTarget[1]+1)
            #     self.mypos = (self.prevTarget[0], wall_position[0]-self.distSensor(center_id)-0.5)
            
            # if walls[2]==1:
            #     wall_position = (self.prevTarget[0]-1, self.prevTarget[1])
            #     self.mypos = (wall_position[0] + self.distSensor(left_id) + 0.5, self.prevTarget[1])
            # elif walls[1] == 1:
            #     wall_position = (self.prevTarget[0]+1, self.prevTarget[1])
            #     self.mypos = (wall_position[0] - self.distSensor(right_id) - 0.5, self.prevTarget[1])

        if self.correctCompass()==-90:
            if walls[0]==1:
                wall_position = (x, y-1)
                self.mypos = (wall_position[0], self.distSensor(center_id) + 0.5 + wall_position[1])

            if walls[1] == 1:
                wall_position = (x+ 1, y)
                self.mypos = (wall_position[0] - self.distSensor(right_id) - 0.5, wall_position[1])
            elif walls[2]==1:
                wall_position = (x-1, y)
                self.mypos = (wall_position[0] + self.distSensor(left_id) + 0.5,  wall_position[1])
            # if walls[0]==1:
            #     wall_position = (self.prevTarget[0], self.prevTarget[1]-1)
            #     self.mypos = (self.prevTarget[0], wall_position[0]+self.distSensor(center_id)+0.5)

            # if walls[2]==1:
            #     wall_position = (self.prevTarget[0]+1, self.prevTarget[1])
            #     self.mypos = (wall_position[0] - self.distSensor(right_id) - 0.5, self.prevTarget[1])
            # elif walls[1] == 1:
            #     wall_position = (self.prevTarget[0]-1, self.prevTarget[1])
            #     self.mypos = (wall_position[0] + self.distSensor(left_id) + 0.5, self.prevTarget[1])

        if self.correctCompass()==-180 or 180:
            if walls[0]==1:
                wall_position = (x-1, y)
                self.mypos = (wall_position[0]+self.distSensor(center_id)+0.5, wall_position[1])

            if walls[1] == 1:
                wall_position = (x, y+1)
                self.mypos = (wall_position[0], wall_position[1] - self.distSensor(right_id) - 0.5)
            elif walls[2]==1:
                wall_position = (x, y-1)
                self.mypos = (wall_position[0], wall_position[1] + self.distSensor(left_id) + 0.5)
            # if walls[0]==1:
            #     wall_position = (self.prevTarget[0]-1, self.prevTarget[1])
            #     self.mypos = (wall_position[0]+self.distSensor(center_id)+0.5, self.prevTarget[1])
            # if walls[2]==1:
            #     wall_position = (self.prevTarget[0], self.prevTarget[1]-1)
            #     self.mypos = (self.prevTarget[0], wall_position[1] + self.distSensor(left_id) + 0.5)
            # elif walls[1] == 1:
            #     wall_position = (self.prevTarget[0], self.prevTarget[1]+1)
            #     self.mypos = (self.prevTarget[0], wall_position[1] - self.distSensor(left_id)-0.5)

            

    def my_gps(self, prev_x, prev_y):

        #print("GPS ROBO ->" , round(self.gps_robo[0],2) , round(self.gps_robo[1],2))
        self.out_1 = self.out
        self.out = ( (self.in_left + self.out_1[0]) / 2 , (self.in_right + self.out_1[1]) / 2 )


        self.prev_teta = self.teta
        if self.mypos == (0,0):
            lin = (self.out[0]+self.out[1])/2 /2
        else: 
            lin = (self.out[0]+self.out[1])/2   

        print("LIN", lin)
        self.gps = ( prev_x + lin *cos(radians(self.correctCompass())) , prev_y + lin *sin(radians(self.correctCompass())))
        
        
        self.prev_gps = self.gps
        #print ("MY GPS->", round(self.gps[0],2), round(self.gps[1],2))

        return(self.gps)

    def straight(self, linear, m, k, ref):
        
        rot = k * (m-ref)

        ##print("ROT ->", rot)
        right_wheel = linear - (rot/2)
        left_wheel = linear + (rot/2)

        self.in_right = right_wheel 
        self.in_left =  left_wheel


        self.driveMotors(left_wheel,right_wheel)
         

    def reached(self, mypos, target):
        array= []
        if self.myorient== 0 :
            if abs(mypos[0] -target[0]) <= 0.28 and round(mypos[0])%2==0 and round(mypos[1])%2==0:
                self.calculate = True
                return 1
        elif self.myorient== 90 :
            if abs(mypos[1] - target[1]) <= 0.28 and round(mypos[1])%2==0 and round(mypos[0])%2==0:
                self.calculate = True
                return 1
        elif self.myorient== -90 :
            if abs(mypos[1] -target[1]) <= 0.28 and round(mypos[1])%2==0 and round(mypos[0])%2==0:
                self.calculate = True
                return 1
        elif self.myorient== 180  or self.myorient==-180:
            if abs(mypos[0] -target[0]) <= 0.28 and round(mypos[0])%2==0 and round(mypos[1])%2==0:
                self.calculate = True
                return 1        

    
    def correctCompass(self):
        if -15 < self.measures.compass < 15:
            return 0
        elif 75 < self.measures.compass< 105:
            return 90
        elif -105 < self.measures.compass <-75:
            return -90
        elif self.measures.compass <= -170 or self.measures.compass >= 170:  
            return 180 * self.measures.compass / abs(self.measures.compass)

        return self.measures.compass
    #add to dictionary if not present
    def add_dict(self, key, str):
        if key not in self.d:
            self.d[key] = str
        if self.d[key] == '|' or self.d[key] == '-':
            self.walls.add((key[0]-28,14-key[1]))
        
        ##print("PAREDES" , self.walls)

    #checks compass orientation 
    def compass_orientation(self,walls):
        compass = self.correctCompass()
        (x, y) = (self.prevTarget)
        tmp= [0,0,0] #[front, right, left]
        print("MYPOS-> ", (round(self.mypos[0]),round(self.mypos[1])))
        print("Target -> ", self.target)

        if compass==0:
            if walls[0] == 1 :
                print('| :', (x+1,y))
                self.add_dict((28+x+1,14-y), '|')
            else: 
                self.add_dict((28+x+1,14-y), 'X')
                print('X :', (x+1,y))
                tmp[0] = (self.prevTarget[0]+2,self.prevTarget[1])
                #tmp[0]=(x+2,y)
                if (x+2,y) not in self.visited:
                    self.add_dict((28+x+2,14-y), 'X')
            if walls[1] == 1 :
                self.add_dict((28+x,14-y+1), '-')
                print('- :', (x,y+1))
            else: 
                self.add_dict((28+x,14-y+1), 'X')
                print('X :', (x,y+1))
                tmp[1] = (self.prevTarget[0],self.prevTarget[1]-2)
                #tmp[1]=(x,y-2)
                if (x,y-2) not in self.visited:
                    self.add_dict((28+x,14-y+2), 'X')
            if walls[2] == 1 :
                self.add_dict((28+x,14-y-1), '-')
                print('- :', (x,y-1))
            else: 
                self.add_dict((28+x,14-y-1), 'X')
                print('X :', (x,y-1))
                tmp[2] = (self.prevTarget[0],self.prevTarget[1]+2)
                #tmp[2]=(x,y+2)
                if (x,y+2) not in self.visited:
                    self.notTaken.add((x,y+2))
                    self.add_dict((28+x,14-y-2), 'X')
        elif compass==90:
            if walls[0] == 1 :
                self.add_dict((28+x,14-y-1), '-')
                print('- :', (x,y-1))
            else: 
                self.add_dict((28+x,14-y-1), 'X')
                print('X :', (x,y-1))
                tmp[0] = (self.prevTarget[0],self.prevTarget[1]+2)
                #tmp[0]=(x,y+2)
                if (x,y+2) not in self.visited:
                    self.add_dict((28+x,14-y+2), 'X')
            if walls[1] == 1 :
                self.add_dict((28+x+1,14-y), '|')
                print('| :', (x+1,y))
            else: 
                self.add_dict((28+x+1,14-y), 'X')
                print('X :', (x+1,y))
                tmp[1] = (self.prevTarget[0]+2,self.prevTarget[1])
                #tmp[1]=(x+2,y)
                if (x+2,y) not in self.visited:
                    self.add_dict((28+x+2,14-y), 'X')
            if walls[2] == 1 :
                self.add_dict((28+x-1,14-y), '|')
                print('| :', (x-1,y))
            else: 
                self.add_dict((28+x-1,14-y), 'X')
                print('X :', (x-1,y))
                tmp[2] = (self.prevTarget[0]-2,self.prevTarget[1])
                #tmp[2]=(x-2,y)
                if (x-2,y) not in self.visited:
                    self.notTaken.add((x-2,y))
                    self.add_dict((28+x-2,14-y), 'X')
        elif compass==180 or compass == -180:
            if walls[0] == 1 :
                self.add_dict((28+x-1,14-y), '|')
                print('| :', (x-1,y))
            else: 
                self.add_dict((28+x-1,14-y), 'X')
                print('X :', (x-1,y))
                tmp[0] = (self.prevTarget[0]-2,self.prevTarget[1])
                #tmp[0] = (x-2,y)self.mapWriting()
                if (x+2,y) not in self.visited:
                    self.add_dict((28+x-2,14-y), 'X')
            if walls[1] == 1 :
                self.add_dict((28+x,14-y-1), '-')
                print('- :', (x,y-1))
            else: 
                self.add_dict((28+x,14-y-1), 'X')
                print('X :', (x,y-1))
                tmp[1] = (self.prevTarget[0],self.prevTarget[1]+2)
                #tmp[1]= (x,y+2)
                if (x,y+2) not in self.visited:
                    self.add_dict((28+x,14-y-2), 'X')
            if walls[2] == 1 :
                self.add_dict((28+x,14-y+1), '-')
                print('- :', (x,y+1))
            else: 
                self.add_dict((28+x,14-y+1), 'X')
                print('X :', (x,y+1))
                tmp[2] = (self.prevTarget[0],self.prevTarget[1]-2)
                #tmp[2]= (x,y-2)
                if (x,y-2) not in self.visited:
                    self.notTaken.add((x,y-2))
                    self.add_dict((28+x,14-y+2), 'X')
        elif compass==-90:
            if walls[0] == 1 :
                self.add_dict((28+x,14-y+1), '-')
                print('- :', (x,y+1))
            else: 
                self.add_dict((28+x,14-y+1), 'X')
                print('X :', (x,y+1))
                tmp[0] = (self.prevTarget[0],self.prevTarget[1]-2)
                #tmp[0]=(x,y-2)
                if (x,y-2) not in self.visited:
                    self.add_dict((28+x,14-y-2), 'X')
            if walls[1] == 1 :
                self.add_dict((28+x-1,14-y), '|')
                print('| :', (x-1,y))
            else: 
                self.add_dict((28+x-1,14-y), 'X')
                print('X :', (x-1,y))
                tmp[1] = (self.prevTarget[0]-2,self.prevTarget[1])
                #tmp[1]=(x-2,y)
                if (x-2,y) not in self.visited:
                    self.add_dict((28+x-2,14-y), 'X')
            if walls[2] == 1 :
                self.add_dict((28+x+1,14-y), '|')
                print('| :', (x+1,y))
            else: 
                self.add_dict((28+x+1,14-y), 'X')
                print('X :', (x+1,y))
                tmp[2] = (self.prevTarget[0]+2,self.prevTarget[1])
                #tmp[2]=(x+2,y)
                if (x+2,y) not in self.visited:
                    self.add_dict((28+x+2,14-y), 'X')

        self.dictionary_noTaken[self.prevTarget] = set()
        array= []

        if tmp[0] != 0 and tmp[0] not in self.visited:
            self.target = tmp[0]
            self.visited.add(tmp[0])
            if tmp[1] != 0 and tmp[1] not in self.visited:
                self.dictionary_noTaken[self.prevTarget].add(tmp[1])
            if tmp[2] != 0 and tmp[2] not in self.visited:
                self.dictionary_noTaken[self.prevTarget].add(tmp[2])
        elif tmp[1] != 0 and tmp[1] not in self.visited:
            self.target = tmp[1]
            self.visited.add(tmp[1])
            if tmp[2] != 0 and tmp[2] not in self.visited:
                self.dictionary_noTaken[self.prevTarget].add(tmp[2])
        elif tmp[2]!= 0 and tmp[2] not in self.visited:
            self.target = tmp[2]
            self.visited.add(tmp[2])
        else:
            array= []
            for i in self.dictionary_noTaken.keys():
                if len(self.dictionary_noTaken[i]) == 0:
                    array.append(i)
                if self.prevTarget in self.dictionary_noTaken[i]:
                    self.dictionary_noTaken[i].discard(self.prevTarget)
            for i in array:
                self.dictionary_noTaken.pop(i)

            
            if len(self.dictionary_noTaken.keys()) == 0:
                print("DEBUG MYPOS : " ,(round(self.mypos[0]), round(self.mypos[1])))
                if round(self.mypos[0])==0 and round(self.mypos[1])==0:
                    self.finish()
                else:
                    self.path = astar(self.prevTarget,(0,0),self.visited,self.walls)
                    self.target= self.path.pop()
                    self.calculate == False
                    if len(self.path)!= 0:
                        self.havepath==True
                return

            #print("LENGTH DICIONARIO -> " , len(self.dictionary_noTaken)   )

            neigh = min(self.dictionary_noTaken,key=lambda point : hypot(self.prevTarget[1]-point[1], self.prevTarget[0]-point[0]))
            
            #print("SOU O NEIGHBOR", neigh)
            #print("sou o target anterior", self.prevTarget)
            #print("VIsited -> "+str(self.visited))
            #print("Walls -> "+str(self.walls))
            print("NEIGH -> ", neigh)
            self.path= astar(self.prevTarget,neigh,self.visited,self.walls)
            self.target = self.path.pop()
            self.calculate== False
            if len(self.path)!= 0:
                 self.havepath=True

                
            #print("CAMINHO", self.path)
        
        #print("not taken", self.dictionary_noTaken)
        if self.measures.ground != -1 and (x,y)!=(0,0): 
            self.d[(x+28,14-y)] = str(self.measures.ground)
            self.goals[(x,y)] = self.measures.ground
        else:
            self.add_dict((28+x,14-y), 'X')

        if (x,y) in self.notTaken:
            self.notTaken.remove((x,y))
            
        
        #print("NOT TAKEN: ", self.notTaken)
        
    def checkwalls(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        
        walls = [0,0,0,0]       # walls =[front, right, left, back]
    

        if (self.measures.irSensor[center_id] + self.measures.irSensor[back_id] ) /2 >= 0.95: 
            walls[0] = 1
        if self.measures.irSensor[right_id] >= 1.3: 
            #print("wall right")
            walls[1] = 1
        if self.measures.irSensor[left_id] >= 1.3: 
            #print("wall left")
            walls[2]=1
        if self.measures.irSensor[back_id] >= 1.3: 
            #print("wall back")
            walls[3]=1
        
        #print(walls)
        return walls

    def mapWriting(self):
        file= open("mappingC4.txt", 'w')    
        for i in range(1,28):
            for j in range(1,56):
                if (j,i) in self.d:
                    file.write(self.d.get((j,i)))
                else:
                    file.write(' ')
            if i != 27:
                file.write('\n')
        
        file.close()


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
ficheiro = ""

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        ficheiro = (sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
