import sys

from tkinter import *

import numpy

from heapq import *

import time



def heuristic(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    return abs(dx) + abs(dy)

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
                
    return False

'''Here is an example of using my algo with a numpy array,
   astar(array, start, destination)
   astar function returns a list of points (shortest path)'''
##nmap1 = numpy.fromfile(file=open("MAP 1.txt"); dtype=int).reshape((100,100,100))
nmap1 = numpy.array([
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1],
    [1,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,1,0,1],
    [1,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,1,0,1,1,1,1,0,1,1,1,0,0,0,1,0,0],
    [1,0,0,1,0,0,0,1,0,0,0,0,1,0,1,0,1,0,1],
    [1,0,0,1,0,0,0,1,0,0,0,0,1,1,1,0,0,0,1],
    [1,1,0,1,0,1,0,1,0,1,1,0,1,0,0,0,0,0,1],
    [1,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,1,1,1],
    [1,0,0,1,0,1,1,0,0,0,1,0,0,0,1,1,1,0,1],
    [1,0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1],
    [1,0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1]])
nmap2 = numpy.array([
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,0,1,0,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
    [1,0,1,0,1,0,1,0,1,1,1,1,0,1,1,0,0,0,1],
    [1,0,1,0,0,0,1,0,0,0,0,1,0,0,1,0,0,0,1],
    [1,0,1,0,0,0,1,0,0,0,0,1,0,0,1,0,1,0,1],
    [0,0,1,1,1,0,1,1,1,1,0,1,1,0,1,0,1,0,0],
    [1,1,1,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,1],
    [1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,0,1],
    [1,0,1,1,1,0,0,0,0,0,0,1,0,0,1,0,0,0,1],
    [1,0,1,0,0,0,1,0,0,0,0,1,1,0,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0,0,1],
    [1,0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1]])
nmap3 = numpy.array([
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1],
    [1,0,0,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,1,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,1,1,1,1,1,0,1,1,1,0,1,1,1],
    [1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0,1],
    [1,1,1,0,1,0,0,0,0,0,1,0,0,1,1,0,0,0,1],
    [1,0,0,0,1,1,0,1,1,1,1,1,0,0,1,1,1,1,1],
    [1,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,1],
    [1,0,1,1,1,0,0,0,0,1,0,1,1,0,0,0,0,0,1],
    [1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,1],
    [1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,1],
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1]])
nmap4 = numpy.array([
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
    [1,0,0,0,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1],
    [1,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1,1,1],
    [1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,0],
    [1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,1],
    [1,0,0,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,1],
    [1,0,0,0,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])
nmap5 = numpy.array([
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1],
    [1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1],
    [0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0],
    [1,0,1,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1],
    [1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])
nmap6 = numpy.array([
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
    [1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,1],
    [1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,1],
    [0,0,1,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,1],
    [1,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,1],
    [1,0,0,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1],
    [1,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1],
    [1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])

class userInterface():

    def __init__ (self, window):
        # Initializes the first window
         
        self.window = window
        self.menuCanvas = Canvas(self.window, width=300, height=300, bg='white')
        self.menuCanvas.pack(fill=BOTH, expand=1)

        self.buttonFrame = Frame(self.menuCanvas)
        self.buttonFrame.pack(side="top", pady=25)

        self.startButton = Button(self.menuCanvas, text="Play game", command = lambda: self.loadGameArena())
        self.startButton.pack()

        self.settingsButton = Button(self.menuCanvas, text="Settings")
        self.settingsButton.pack()

        self.exitButton = Button(self.menuCanvas, text="Exit", command = lambda: self.window.destroy())
        self.exitButton.pack()

    def settings(self):
        # Might need to edit this a little. 
        self.SettingsWindow = Tk()
        self.label = Label(self.SettingsWindow, text="There should be settings here. ")
        self.label.pack()
        self.SettingsWindow.mainloop()

    def loadGameArena(self):
        # Eventually this will have a new game/load game option prior to loading the actual game
        self.menuCanvas.pack_forget()
        theGame = theRobotGame(self.window)

class theRobotGame():
    def __init__(self, window):
        self.window = window
        self.window.geometry("1000x580+200+60")
        self.robotLocation = 1
        self.robotCoords = (65, 65)
        self.zone1 = Canvas(self.window, width=380, height = 580, bg="white")
        self.zone1.pack(side=RIGHT, fill=BOTH, expand=1)
        self.labelFrame = Frame(self.zone1)
        self.labelFrame.pack()
        self.ListOfItems = Label(self.labelFrame, text="Items Collected:", font="bold")
        self.ListOfItems.pack()
        self.alist = []
        self.SortButton = Button(self.zone1, text="Sort", command = lambda: self.SortAlgorithm(self.alist))
        self.SortButton.pack()
        self.createMap(self.robotCoords)
        self.LabelList = Label(self.zone1, text=self.alist)
    def SortAlgorithm(self, alist):
        def swap(i, j):                    
            alist[i], alist[j] = alist[j], alist[i] 

        def heap(end,i):   
            l=2 * i + 1  
            r=2 * (i + 1)   
            max=i   
            if l < end and alist[i] < alist[l]:   
                max = l
            if r < end and alist[max] < alist[r]:   
                max = r
            if max != i:   
                swap(i, max)   
                heap(end, max)

        def heap_sort():     
            end = len(alist)   
            start = end // 2 - 1 
            for i in range(start, -1, -1):   
                heap(end, i)
            for i in range(end-1, 0, -1):   
                swap(i, 0)   
                heap(i, 0)
                k=k+1
        self.LabelList.pack()
        self.LabelList.config(text=self.alist)
        
        
    def createMap(self, crds):

        self.coords = crds
        # split each if into it's own seperate function: zone1(), zone2() etc...
        if self.robotLocation == 1:
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 1")
            self.zone.pack(side=LEFT,fill=BOTH)
            self.teleport1 = self.zone.create_line(290, 560, 320, 560, fill="green", width=2)
            self.teleport2 = self.zone.create_line(590, 290, 590, 320, fill ="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas1(20,20,50,50)
            self.initiateGameplay(nmap1,1,1,7,6)

        if self.robotLocation == 2:
            self.zone.pack_forget()
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 2")
            self.zone.pack(fill=BOTH)
            self.teleport2 = self.zone.create_line(20, 290, 20, 320, fill ="green", width=2)
            self.teleport3 = self.zone.create_line(290, 560, 320, 560, fill="green", width=2)
            self.teleport7 = self.zone.create_line(590, 290, 590, 320, fill="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas2(20,20,50,50)

        if self.robotLocation == 6:
            self.zone.pack_forget()
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 6")
            self.zone.pack(fill=BOTH)
            self.teleport1 = self.zone.create_line(290,20,320,20, fill="green", width=2)
            self.teleport6 = self.zone.create_line(20,290,20,320, fill ="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas6(20,20,50,50)

        if self.robotLocation == 3:
            self.zone.pack_forget()
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 3")
            self.zone.pack(fill=BOTH)
            self.teleport3 = self.zone.create_line(20,290,20,320, fill="green", width=2)
            self.teleport4 = self.zone.create_line(290,560,320,560, fill="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas3(20,20,50,50)

        if self.robotLocation == 4:
            self.zone.pack_forget()
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 4")
            self.zone.pack(fill=BOTH)
            self.teleport4 = self.zone.create_line(290, 20, 320, 20, fill="green", width=2)
            self.teleport5 = self.zone.create_line(590, 290, 590, 320, fill ="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas4(20,20,50,50)

        if self.robotLocation == 5:
            self.zone.pack_forget()
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 5")
            self.zone.pack(fill=BOTH)
            self.teleport5 = self.zone.create_line(20,290,20,320, fill ="green", width=2)
            self.teleport6 = self.zone.create_line(290,20,320,20, fill ="green", width=2)
            self.teleport7 = self.zone.create_line(590,290,590,320, fill="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas5(20,20,50,50)
    def canvas1(self, topx,topy,botx,boty):
            room = open("MAP 1.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=30
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo1 = PhotoImage(file="FUEL.png")
                        self.fuel = self.zone.create_image(topx+15, topy+15, image=self.photo1)
                        botx += 30
                        topx = botx - 30
    def canvas2(self, topx,topy,botx,boty):
            room = open("MAP 2.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=20
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo2 = PhotoImage(file="Antenna.png")
                        self.antenna = self.zone.create_image(topx+15, topy+15, image=self.photo2)
                        botx += 30
                        topx = botx - 30
    def canvas3(self, topx,topy,botx,boty):
            room = open("MAP 3.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=20
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo3 = PhotoImage(file="BATTERIES.png")
                        self.batteries = self.zone.create_image(topx+15, topy+15, image=self.photo3)
                        botx += 30
                        topx = botx - 30
    def canvas4(self, topx,topy,botx,boty):
            room = open("MAP 4.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=20
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo4 = PhotoImage(file="DOOR.png")
                        self.door = self.zone.create_image(topx+15, topy+15, image=self.photo4)
                        botx += 30
                        topx = botx - 30
    def canvas5(self, topx,topy,botx,boty):
            room = open("MAP 5.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=20
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo5 = PhotoImage(file="Propulsion.png")
                        self.propulsion = self.zone.create_image(topx+15, topy+15, image=self.photo5)
                        botx += 30
                        topx = botx - 30
    def canvas6(self, topx,topy,botx,boty):
            room = open("MAP 6.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=20
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo6 = PhotoImage(file="CPU.png")
                        self.cpu = self.zone.create_image(topx+15, topy+15, image=self.photo6)
                        botx += 30
                        topx = botx - 30

        
        
    def initiateGameplay(self, xmap, x1,x2,y1,y2):
        self.zone.focus_set()
        
        pa,pb = x1,x2
        l = len(astar(xmap, (x1,x2), (y1,y2))) - 1
        while l >= 0:
            self.zone.update()
            a,b = astar(xmap, (x1,x2), (y1,y2)).pop(l)
            print("step %d" %l)
            print(a - pa)
            print(b - pb)
            ##down
            if a - pa == 1 and b - pb == 0:
                self.zone.move(self.robot, 0, 30)
                time.sleep(0.1)
                if self.robotLocation == 6:
                    self.ItemTuple6 = self.zone.find_overlapping(500,500,530,530)
                    if self.robot in self.ItemTuple6:
                        self.zone.delete(self.cpu)
                        self.CPUlabel = Label(self.labelFrame, text="CPU")
                        self.CPUlabel.pack()
                        self.alist.append("CPU")
                        self.initiateGameplay(nmap6,16,16,9,0)
                    else:
                        pass
                if self.robotLocation == 1:
                    self.ItemTuple1 = self.zone.find_overlapping(200,230,230,260)
                    if self.robot in self.ItemTuple1:
                        self.zone.delete(self.fuel)
                        self.FuelLabel = Label(self.labelFrame, text="Fuel")
                        self.FuelLabel.pack()
                        self.alist.append("Fuel")
                        self.initiateGameplay(nmap1,8,7,9,18)
                    else:
                        pass

                    self.teleportTuple1 = self.zone.find_overlapping(290,550,320,560)
                    
                    if self.robot in self.teleportTuple1:
                        self.robotLocation = 4
                        self.robotCoords = (305,65)
                        self.createMap(self.robotCoords)
                    else:
                        pass
                if self.robotLocation == 3:
                    self.ItemTuple3 = self.zone.find_overlapping(500,260,530,290)
                    if self.robot in self.ItemTuple3:
                        self.zone.delete(self.batteries)
                        self.BattLabel = Label(self.labelFrame, text="Battery")
                        self.BattLabel.pack()
                        self.alist.append("Battery")
                        self.initiateGameplay(nmap3,8,16,17,9)
                    else:
                        pass

                    self.teleportTuple4 = self.zone.find_overlapping(290,550,320,560)
                    
                    if self.robot in self.teleportTuple4:
                        self.robotLocation = 6
                        self.robotCoords = (305,65)
                        self.createMap(self.robotCoords)
                        self.initiateGameplay(nmap6,1,9,16,16)
                    else:
                        pass

                if self.robotLocation == 2:
                    
                    self.ItemTuple2 = self.zone.find_overlapping(440,80,470,110)
                    if self.robot in self.ItemTuple2:
                        self.zone.delete(self.antenna)
                        self.AntennaLabel = Label(self.labelFrame, text="Antenna")
                        self.AntennaLabel.pack()
                        self.alist.append("Antenna")
                        self.initiateGameplay(nmap2,2,14,9,18)
                    else:
                        pass

                    self.teleportTuple7 = self.zone.find_overlapping(290,550,320,560)

                    if self.robot in self.teleportTuple7:
                        self.robotLocation = 5
                        self.robotCoords= (305,65)
                        self.createMap(self.robotCoords)
                    else:
                        pass
                if self.robotLocation == 4:
                    self.ItemTuple4 = self.zone.find_overlapping(380,500,410,530)
                    if self.robot in self.ItemTuple4:
                        self.zone.delete(self.door)
                        self.DoorLabel = Label(self.labelFrame, text="Door")
                        self.DoorLabel.pack()
                        self.alist.append("Door")
                        #time.sleep(5)
                    else:
                        pass
                if self.robotLocation == 5:
                    self.ItemTuple5 = self.zone.find_overlapping(530,500,560,530)
                    if self.robot in self.ItemTuple5:
                        self.zone.delete(self.propulsion)
                        self.PropulsionLabel = Label(self.labelFrame, text="Propulsion")
                        self.PropulsionLabel.pack()
                        self.alist.append("Propulsion")
                        self.initiateGameplay(nmap5,16,17,9,0)
                    else:
                        pass
            ##up
            if a - pa == -1 and b - pb == 0:
                self.zone.move(self.robot, 0, -30)
                time.sleep(0.1)
                if self.robotLocation == 1:
                    self.ItemTuple1 = self.zone.find_overlapping(200,230,230,260)
                    if self.robot in self.ItemTuple1:
                        self.zone.delete(self.fuel)
                        self.FuelLabel = Label(self.labelFrame, text="Fuel")
                        self.FuelLabel.pack()
                        self.alist.append("Fuel")
                        self.initiateGameplay(nmap1,8,7,9,18)
                    else:
                        pass

                if self.robotLocation == 6:
                    self.ItemTuple6 = self.zone.find_overlapping(500,500,530,530)
                    if self.robot in self.ItemTuple6:
                        self.zone.delete(self.cpu)
                        self.CPUlabel = Label(self.labelFrame, text="CPU")
                        self.CPUlabel.pack()
                        self.alist.append("CPU")
                        self.initiateGameplay(nmap6,16,16,9,0)
                    else:
                        pass

                    self.teleportTuple1 = self.zone.find_overlapping(290,30,320,20)

                    if self.robot in self.teleportTuple1:
                        self.robotLocation = 3
                        self.zone.pack_forget()
                        self.robotCoords = (305,515)
                        self.createMap(self.robotCoords)
                    else:
                        pass

                if self.robotLocation == 4:

                    self.teleportTuple4 = self.zone.find_overlapping(290,30,320,20)

                    if self.robot in self.teleportTuple4:
                        self.robotLocation = 1
                        self.zone.pack_forget()
                        self.robotCoords = (305,515)
                        self.createMap(self.robotCoords)
                    else:
                        pass

                if self.robotLocation == 5:
                    self.ItemTuple5 = self.zone.find_overlapping(530,500,560,530)
                    if self.robot in self.ItemTuple5:
                        self.zone.delete(self.propulsion)
                        self.PropulsionLabel = Label(self.labelFrame, text="Propulsion")
                        self.PropulsionLabel.pack()
                        self.alist.append("Propulsion")
                        self.initiateGameplay(nmap5,16,17,9,0)

                    self.teleportTuple7 = self.zone.find_overlapping(290,30,320,20)

                    if self.robot in self.teleportTuple7:
                        self.robotLocation = 2
                        self.zone.pack_forget()
                        self.robotCoords = (305,515)
                        self.createMap(self.robotCoords)
                    else:
                        pass
                if self.robotLocation == 2:
                    
                    self.ItemTuple2 = self.zone.find_overlapping(440,80,470,110)
                    if self.robot in self.ItemTuple2:
                        self.zone.delete(self.antenna)
                        self.AntennaLabel = Label(self.labelFrame, text="Antenna")
                        self.AntennaLabel.pack()
                        self.alist.append("Antenna")
                        self.initiateGameplay(nmap2,2,14,9,18)
                    else:
                        pass
                if self.robotLocation == 3:
                    self.ItemTuple3 = self.zone.find_overlapping(500,260,530,290)
                    if self.robot in self.ItemTuple3:
                        self.zone.delete(self.batteries)
                        self.BattLabel = Label(self.labelFrame, text="Battery")
                        self.BattLabel.pack()
                        self.alist.append("Battery")
                        self.initiateGameplay(nmap3,8,16,17,9)
                    else:
                        pass
            ##right
            if a - pa == 0 and b - pb == 1:
                self.zone.move(self.robot, 30, 0)
                time.sleep(0.1)
                if self.robotLocation == 6:
                    self.ItemTuple6 = self.zone.find_overlapping(500,500,530,530)
                    if self.robot in self.ItemTuple6:
                        self.zone.delete(self.cpu)
                        self.CPUlabel = Label(self.labelFrame, text="CPU")
                        self.CPUlabel.pack()
                        self.alist.append("CPU")
                        self.initiateGameplay(nmap6,16,16,9,0)
                        
                    else:
                        pass

                if self.robotLocation == 1:
                    self.ItemTuple1 = self.zone.find_overlapping(200,230,230,260)
                    if self.robot in self.ItemTuple1:
                        self.zone.delete(self.fuel)
                        self.FuelLabel = Label(self.labelFrame, text="Fuel")
                        self.FuelLabel.pack()
                        self.alist.append("Fuel")
                        self.initiateGameplay(nmap1,8,7,9,18)
                    else:
                        pass
                    self.teleportTuple2 = self.zone.find_overlapping(580,260,590,320)
                    
                    if self.robot in self.teleportTuple2:
                        self.robotLocation = 2
                        self.zone.pack_forget()
                        self.robotCoords = (65,305)
                        self.createMap(self.robotCoords)
                        self.initiateGameplay(nmap2,9,1,2,14)
                    else:
                        pass

                if self.robotLocation == 2:
                    
                    self.ItemTuple2 = self.zone.find_overlapping(440,80,470,110)
                    if self.robot in self.ItemTuple2:
                        self.zone.delete(self.antenna)
                        self.AntennaLabel = Label(self.labelFrame, text="Antenna")
                        self.AntennaLabel.pack()
                        self.alist.append("Antenna")
                        self.initiateGameplay(nmap2,2,14,9,18)
                    else:
                        pass

                    self.teleportTuple3 = self.zone.find_overlapping(580,260,590,320)

                    if self.robot in self.teleportTuple3:
                        self.robotLocation = 3
                        self.robotCoords = (65,305)
                        self.createMap(self.robotCoords)
                        self.initiateGameplay(nmap3,9,1,8,16)
                    else:
                        pass

                if self.robotLocation == 5:
                    self.ItemTuple5 = self.zone.find_overlapping(530,500,560,530)
                    if self.robot in self.ItemTuple5:
                        self.zone.delete(self.propulsion)
                        self.PropulsionLabel = Label(self.labelFrame, text="Propulsion")
                        self.PropulsionLabel.pack()
                        self.alist.append("Propulsion")
                        self.initiateGameplay(nmap5,16,17,9,0)
                    else:
                        pass

                    self.teleportTuple5 = self.zone.find_overlapping(580,260,590,320)
                    
                    if self.robot in self.teleportTuple5:
                        self.robotLocation = 6
                        self.zone.pack_forget()
                        self.robotCoords = (65,305)
                        self.createMap(self.robotCoords)
                    else:
                        pass

                if self.robotLocation == 4:

                    self.teleportTuple6 = self.zone.find_overlapping(580, 260, 590, 320)

                    if self.robot in self.teleportTuple6:
                        self.robotLocation = 5
                        self.zone.pack_forget()
                        self.robotCoords = (65,305)
                        self.createMap(self.robotCoords)
                    else:
                        pass
                if self.robotLocation == 3:
                    self.ItemTuple3 = self.zone.find_overlapping(500,260,530,290)
                    if self.robot in self.ItemTuple3:
                        self.zone.delete(self.batteries)
                        self.BattLabel = Label(self.labelFrame, text="Battery")
                        self.BattLabel.pack()
                        self.alist.append("Battery")
                        self.initiateGameplay(nmap3,8,16,17,9)
                    else:
                        pass
            ##left
            if a - pa == 0 and b - pb == -1:
                self.zone.move(self.robot, -30, 0)
                time.sleep(0.1)
                if self.robotLocation == 1:
                    self.ItemTuple1 = self.zone.find_overlapping(200,230,230,260)
                    if self.robot in self.ItemTuple1:
                        self.zone.delete(self.fuel)
                        self.FuelLabel = Label(self.labelFrame, text="Fuel")
                        self.FuelLabel.pack()
                        self.alist.append("Fuel")
                        self.initiateGameplay(nmap1,7,6,9,18)
                    else:
                        pass

                if self.robotLocation == 2:
                    
                    self.ItemTuple2 = self.zone.find_overlapping(440,80,470,110)
                    if self.robot in self.ItemTuple2:
                        self.zone.delete(self.antenna)
                        self.AntennaLabel = Label(self.labelFrame, text="Antenna")
                        self.AntennaLabel.pack()
                        self.alist.append("Antenna")
                        self.initiateGameplay(nmap2,2,14,9,18)
                    else:
                        pass

                    self.teleportTuple1 = self.zone.find_overlapping(20,290,30,320)

                    if self.robot in self.teleportTuple1:
                        self.robotLocation = 1
                        self.zone.pack_forget()
                        self.robotCoords = (545,305)
                        self.createMap(self.robotCoords)

                if self.robotLocation == 3:
                    self.ItemTuple3 = self.zone.find_overlapping(500,260,530,290)
                    if self.robot in self.ItemTuple3:
                        self.zone.delete(self.batteries)
                        self.BattLabel = Label(self.labelFrame, text="Battery")
                        self.BattLabel.pack()
                        self.alist.append("Battery")
                        self.initiateGameplay(nmap3,8,16,17,9)
                    else:
                        pass

                    self.teleportTuple3 = self.zone.find_overlapping(20,290,30,320)

                    if self.robot in self.teleportTuple3:
                        self.robotLocation = 2
                        self.zone.pack_forget()
                        self.robotCoords = (545,305)
                        self.createMap(self.robotCoords)
                    else:
                        pass

                if self.robotLocation == 6:
                    self.ItemTuple6 = self.zone.find_overlapping(500,500,530,530)
                    if self.robot in self.ItemTuple6:
                        self.zone.delete(self.cpu)
                        self.CPUlabel = Label(self.labelFrame, text="CPU")
                        self.CPUlabel.pack()
                        self.alist.append("CPU")
                        self.initiateGameplay(nmap6,16,16,9,0)
                    else:
                        pass

                    self.teleportTuple5 = self.zone.find_overlapping(20,290,30,320)

                    if self.robot in self.teleportTuple5:
                        self.robotLocation = 5
                        self.zone.pack_forget()
                        self.robotCoords = (545,305)
                        self.createMap(self.robotCoords)
                        self.initiateGameplay(nmap5,9,17,16,17)
                    else:
                        pass

                if self.robotLocation == 5:
                    self.ItemTuple5 = self.zone.find_overlapping(530,500,560,530)
                    if self.robot in self.ItemTuple5:
                        self.zone.delete(self.propulsion)
                        self.PropulsionLabel = Label(self.labelFrame, text="Propulsion")
                        self.PropulsionLabel.pack()
                        self.alist.append("Propulsion")
                        self.initiateGameplay(nmap5,16,17,9,0)

                    self.teleportTuple6 = self.zone.find_overlapping(20,290,30,320)

                    if self.robot in self.teleportTuple6:
                        self.robotLocation = 4
                        self.zone.pack_forget()
                        self.robotCoords = (545,305)
                        self.createMap(self.robotCoords)
                        self.initiateGameplay(nmap4,9,17,16,12)
                    else:
                        pass
            l = l - 1
            pa,pb = a,b
            
        
def main():
    window = Tk()
    window.geometry("400x200+450+300") # Window needs to be centered on each PC screen 
    game = userInterface(window)
    window.mainloop()

if __name__=='__main__':
    sys.exit(main())
