
from os import write
import sys
from typing import DefaultDict
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14



class MyRob(CRobLinkAngs):

    posinitial = ()
    target = ()
    prevTarget = ()
    mypos= ()
    myorient = ()
    nextorient = ()

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
        stopped_state = 'go'

        self.readSensors()
        while True:
            self.readSensors()
            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                self.posinitial = (self.measures.x, self.measures.y)
                self.target = (0,0)
                self.prevTarget = (0,0)
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'go':
                self.mypos = (round(self.measures.x - self.posinitial[0],2) , round(self.measures.y-self.posinitial[1],2))
                self.myorient = self.correctCompass()
                print('POSICAO INICIAL', self.posinitial)
                print('MINHA POSICAO', self.mypos)
                print('TARGET',self.target)
                print('\n')
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                    
                if self.target == ():
                    self.calculateTarget()
                elif self.reached(self.mypos,self.target):
                    self.prevTarget = self.target
                    state= 'end'
                else:
                    print('BUSSULA',self.measures.compass)
                    print('CORRECAO DA BUSSOLA',self.correctCompass())
                    if self.correctCompass() == 180:
                        if self.measures.compass < 0:
                            self.straight(0.15,self.measures.compass,0.05,self.correctCompass())
                        else: 
                            self.straight(0.15,self.measures.compass,0.05,self.correctCompass())
                    else:
                        self.straight(0.15,self.measures.compass,0.05,self.correctCompass())
            if state== 'rotate right':
                print("ESTOU A RODAR")
                if self.nextorient == ():
                    if self.correctCompass() == 180 or self.correctCompass() == -180: 
                        self.nextorient = 90
                    else:
                        self.nextorient = self.myorient-90
                    print("OBJETIVO", self.nextorient)
                elif abs(self.measures.compass - self.nextorient) <= 5:
                    self.myorient= self.nextorient
                    self.nextorient = ()
                    state = 'end'
                else:
                    print(self.measures.compass)
                    self.driveMotors(0.03,-0.03)
            if state == 'rotate left':
                if self.nextorient == ():
                    if self.correctCompass() == 180 or self.correctCompass() == -180: 
                        self.nextorient = -90
                    else:
                        self.nextorient = self.myorient+90
                    print("OBJETIVO", self.nextorient)
                elif abs(self.measures.compass - self.nextorient) <=5:
                    self.myorient= self.nextorient
                    self.nextorient = ()
                    state = 'end'
                else:
                    print(self.measures.compass)
                    self.driveMotors(-0.03,0.03)                
            if state == 'end':
                self.target= ()
                self.driveMotors(0,0)
                print('PAREDES ', self.checkwalls())
                if self.checkwalls()[0]== 1:
                    print('IM HERE CHECK WALL 1')
                    if self.checkwalls()[1]== 0:
                        print('IM HERE CHECK WALL 2')
                        state = 'rotate right'
                    elif self.checkwalls()[2]== 0:
                        print('IM HERE CHECK WALL 3')
                        state = 'rotate left'
                    elif self.checkwalls()[0]== 1 and self.checkwalls()[1]== 1 and self.checkwalls()[2]== 1:
                         print(self.checkwalls())
                         state = 'rotate right' 
                else:
                    state = 'go'
                

    def calculateTarget(self):
        print('ESTOU A CALCULAR')
        if self.correctCompass()== 0:
            print('PREVIOUS TARGET',self.prevTarget)
            self.target = (self.prevTarget[0]+2, self.prevTarget[1])
        elif self.correctCompass()== 90:
            print('PREVIOUS TARGET',self.prevTarget)
            self.target = (self.prevTarget[0], self.prevTarget[1]+2)
        elif self.correctCompass()== -90:
            print('PREVIOUS TARGET',self.prevTarget)
            self.target = (self.prevTarget[0], self.prevTarget[1]-2)
        elif self.correctCompass()== 180 or self.correctCompass()== -180:
            print('PREVIOUS TARGET',self.prevTarget)
            self.target = (self.prevTarget[0]-2, self.prevTarget[1])  
        
        return self.target

    def straight(self, linear, m, k, ref):
        rot = k * (m-ref)

        right_wheel = linear - (rot/2)
        left_wheel = linear + (rot/2)

        self.driveMotors(left_wheel,right_wheel)

    def reached(self, mypos, target):
        if self.myorient== 0 :
            if abs(mypos[0] -target[0]) <= 0.3:
                return 1
        elif self.myorient== 90 :
            if abs(mypos[1] - target[1]) <= 0.3:
                return 1
        elif self.myorient== -90 :
            if abs(mypos[1] -target[1]) <= 0.3:
                return 1
        elif self.myorient== 180  or self.myorient==-180:
            if abs(mypos[0] -target[0]) <= 0.3:
                return 1        

    
    def correctCompass(self):
        if -10 < self.measures.compass < 10:
            return 0
        elif 80 < self.measures.compass< 100:
            return 90
        elif -100 < self.measures.compass <-80:
            return -90
        elif self.measures.compass <= -170 or self.measures.compass >= 170:  
            return 180 * self.measures.compass / abs(self.measures.compass)






    def checkwalls(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        
        walls = [0,0,0,0]       # walls =[front, right, left, back]


        if self.measures.irSensor[center_id] >= 1.2 : 
            #print("wall front")
            walls[0] = 1
        if self.measures.irSensor[right_id] >= 1.2 : 
            #print("wall right")
            walls[1] = 1
        if self.measures.irSensor[left_id] >= 1.2 : 
            #print("wall left")
            walls[2]=1
        if self.measures.irSensor[back_id] >= 1.2 : 
            #print("wall back")
            walls[3]=1
        
        #print(walls)
        return walls

    def mapWriting(self):
        file= open("mapping.txt", 'w')    
        for i in range(0,26):
            for j in range(0,55):
                if i == 14 and j == 28:
                    file.write('I')
                file.write(' ')
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

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
