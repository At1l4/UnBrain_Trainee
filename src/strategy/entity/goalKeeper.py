from ..entity import Entity
from strategy.field.UVF import UVF
from strategy.field.DirectionalField import DirectionalField
from strategy.field.goalKeeper import GoalKeeperField
from strategy.field.attractive import AttractiveField
from strategy.movements import goalkeep, spinGoalKeeper
from tools import angError, howFrontBall, howPerpBall, ang, norml, norm, angl
from tools.interval import Interval
from control.goalKeeper import GoalKeeperControl
from control.UFC import UFC_Simple
import numpy as np
import math
import time


class GoalKeeper(Entity):
    def __init__(self, world, robot, side=1):
        super().__init__(world, robot)

        self._control = GoalKeeperControl(world)
        self.lastChat = 0
        self.state = "Stable"

    @property
    def control(self):
        return self._control

    def equalsTo(self, otherGoalKeeper):
        return True

    def onExit(self):
        pass

    def setGoalKeeperControl(self):
        self._control = GoalKeeperControl(self.world)
    
    def setAttackerControl(self):
        self._control = UFC_Simple(self.world)
        
    def directionDecider(self):
       if self.robot.field is not None:
            ref_th = self.robot.field.F(self.robot.pose)
            rob_th = self.robot.th
            
            if abs(angError(ref_th, rob_th)) > 90 * np.pi / 180: 
                self.robot.direction *= -1
                self.lastChat = time.time()
            
            # Inverter a direção se o robô ficar preso em algo
            elif not self.robot.isAlive() and self.robot.spin == 0:
                if time.time()-self.lastChat > .3:
                    self.lastChat = time.time()
                    self.robot.direction *= -1

    def maintainPosition(self): #Mantem o robo no gol
        
        dist_lim = 0.01 #Minima distancia do gol

        positive_correction = -1
        negative_correction = 1

        rr = np.array(self.robot.pos) #Posica e velocidade do robo
        vr = np.array(self.robot.v)

        rg = np.array(self.world.field.goalPos) #Posicao do gol

        dist_robot_goal = rr[0] - rg[0]

        if abs(dist_robot_goal )> dist_lim : #Caso extrapole o limite
            if dist_robot_goal>0 :

                self.robot.vref[0] += positive_correction #Corrije

                #alternativamente self.robot.direction *= -1

            else:

                self.robot.vref[0] += negative_correction

                #alternativamente self.robot.direction *= -1

    def trackBall(self):

        max_error = 0.01 #Maxima distancia desejada da bola
        positive_correction = -1
        negative_correction = 1

        rr = np.array(self.robot.pos)
        vr = np.array(self.robot.v)

        rb = np.array(self.world.ball.pos)
        vb = np.array(self.world.ball.v)

        rb += vb*1/30 #Posicao a cada frame (30fps)

        dist_robot_ball = rr[1] - rb[1] 

        if abs(dist_robot_ball) < max_error: 

            if dist_robot_ball>0 :

                self.robot.vref[1] += positive_correction

                #alternativamente self.robot.direction *= -1

            else:

                self.robot.vref[1] += negative_correction
        
                #alternativamente self.robot.direction *= -1


    def defendBall(self):

        critical_distance = 0.01 #Maxima distancia da bola ao gol

        rr = np.array(self.robot.pos)
        vr = np.array(self.robot.v)

        rb = np.array(self.world.ball.pos)
        vb = np.array(self.world.ball.v)

        rg = np.array(self.world.field.goalPos) #Posicao do gol

        rb += vb*1/30 #Posicao a cada frame (30fps)

        dist_goal_ball = rr - rg

        if math.sqrt(dist_goal_ball[0]**2 + dist_goal_ball[1]**2) < critical_distance:

            















   '''NAO UTILIZAR def fieldDecider(self):
        #Define os vetores posicao e velocidade
        #Do robo, bola e gol
        rr = np.array(self.robot.pos)
        vr = np.array(self.robot.v)

        rb = np.array(self.world.ball.pos)
        vb = np.array(self.world.ball.v)
        rg = -np.array(self.world.field.goalPos)
        rg[0] += 0.06
    
         # Aplica o movimento
        self.robot.vref = 0.2

        #Define Angulacao
        self.robot.setSpin(spinGoalKeeper(rb, rr, rg), timeOut = 0.13)

        #Estado de defesa
        Pb = goalkeep(rb, vb, rr, rg)

        if self.state == "Stable":
            """if self.robot.field is not None:
                ref_th = self.robot.field.F(self.robot.pose)
                rob_th = self.robot.th
                if abs(angError(ref_th, rob_th)) > 45 * np.pi / 180:
                    print('unstable ANG')
                    self.robot.setSpin(1, timeOut = 0.13)"""
            
            #Caso se afaste muito do gol
            if np.abs(rr[0]-rg[0]) > 0.03: #or abs(angError(self.robot.th, ang(rr, rg))) > 40 * np.pi / 180:
                print(np.abs(rr[0]-rg[0]) > 0.03)
                print('unstable')
                self.state = "Unstable"
                """if self.robot.field is not None:
                    ref_th = self.robot.field.F(self.robot.pose)
                    rob_th = self.robot.th
                    if abs(angError(ref_th, rob_th)) > 45 * np.pi / 180:
                        print('unstable ANG')
                        self.robot.setSpin(1, timeOut = 0.05)"""
        elif self.state == "Unstable":
            #Se estiver fora da linha
            if rr[0] > 0:
                print("Attacker Control acessado")
                self.setAttackerControl()
                self.state = "Far"
            #Se nao estiver longe
            elif np.abs(rr[0]-rg[0]) < 0.015:
                self.state = "Stable"
        else:
            print('else')
            if np.abs(rr[0]-rg[0]) < 0.015:
                print("IF")
                self.state = "Stable"
                self.setGoalKeeperControl()

        #self.robot.field = UVF(Pb, spiral=0.01)
        #self.robot.field = DirectionalField(Pb[2], Pb=Pb) if np.abs(rr[0]-Pb[0]) < 0.07 else UVF(Pb, spiral=0.01)

        if self.state == "Stable":
            #print("entrou no stable")
            self.robot.field = DirectionalField(Pb[2], Pb=(rr[0], Pb[1], Pb[2]))
        elif self.state == "Unstable":
            #print("entrou no unstable")
            #self.robot.field = UVF(Pb, radius=0.02)
            #self.robot.field = GoalKeeperField(Pb, rg[0]-0.02)
            self.robot.field = AttractiveField((rg[0]-0.02, Pb[1], Pb[2]))
        elif self.state == "Far":
            #print("entrou no far")
            self.robot.field = UVF(Pb, radius=0.04)
        #self.robot.field = DirectionalField(Pb[2], Pb=Pb) '''
