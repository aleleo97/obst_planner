#! /usr/bin/env python
import cvxpy as cp
import numpy as np
from sympy import symbols, IndexedBase, Idx

#let's define the variables of the class (u inputs and x states)
u = IndexedBase('u')
n_in = symbols('n_in ', integer=True)
u[n_in]
#you can change the number of input but not the name
n_in = Idx('n_in', 2)
x = IndexedBase('x')
n_states = symbols('n_states', integer=True)
x[n_states]
#You can change the number of states not the name
n_states = Idx('n_states', 3)
class ConvexOpt():
  def __init__(self,N,x_init,x_fin,u_in,A_list,B_list,C_list,xante,uante,p = 0,n_ost = 0,H = 0):
    #init the variables of the class
    self.N = N
    self.x_init = x_init
    self.x_fin = x_fin
    self.u_in = u_in
    self.Ad_list = A_list
    self.Bd_list = B_list
    self.Cd_list = C_list
    self.xante = xante
    self.uante = uante
    self.p = p
    self.H = H
    self.n_ost = n_ost
    
  def CVXOPT(self,opt_power = False,opt_velocity = False):
    #save the number of states and inputs
    x_len =(int) (x[n_states].shape[0])
    u_len = (int) (u[n_in].shape[0])
    #define the variables to be evaluate 
    xv = cp.Variable(shape=(x_len, self.N))
    uv = cp.Variable((u_len, self.N-1))
    tau = cp.Variable(shape=(self.N))
    tau_vel = cp.Variable(shape=(self.N))
    tau_u = cp.Variable(shape=(self.N-1))
    hogb = cp.Variable(self.N-1)
    hog = cp.Variable(shape = (x_len,self.N-1))
    nu = cp.Variable(1,)
    #define the objective of the convex optimization 
    obj = cp.sum_squares(np.ones(shape=(1,self.N))*tau +np.ones(shape=(1,self.N))*tau_vel + 10*np.ones(shape=(1,self.N-1))*tau_u  + np.ones(shape=(1,self.N-1))*10**3*hogb+ 10**7*nu)
    obj = cp.Minimize(obj)
    #define all constrains to be take into account but they have to be convex 
    constr = []
    #initial condition for x-y position and angular position
    constr += [xv[:,0] == self.x_init]
    #initial condition related to inputs 
    constr += [uv[:,0] == self.u_in]
    #final position constrain 
    constr += [cp.norm(xv[:,self.N-1] - self.x_fin) <= 10e-9]
    #trajectory limitation 
    for t in range(0,self.N-1):
      #discrete trajectory with virtual control 
        constr += [ xv[:,t+1] == self.Ad_list[t]@xv[:,t] + self.Bd_list[t] @ uv[:,t] + self.Cd_list[t]]
        #norm(hog(:,k)) <= hogb(k)
        constr += [cp.norm(hog[:,t]) <= hogb[t]]

    #take into account only the shortest trajectory 
        #constr += [cp.norm(xv[:,t-1] - xv[:,t]) <= tau[t]]

        #I tried to code linear obstacle but  working only in rectangular case  
        #constr += [xv[1,t] <= 6]
        #constr += [xv[1,t] >= 0]
        #constr += [xv[0,t] <= 5] 
        #constr += [cp.norm2(xv[0,t] - 1) >= 1]

        #contrainte qui definit la presence d'un obstacle
        #position
        for i in range(self.n_ost):
            #H = self.H [i] #geometrie
            if np.any(self.xante) and np.any(self.uante) and len(self.p)>0: #contrainte qui vient de la thèse de Miki
                A = self.H[i]
                b = np.dot(self.H[i],self.p[i])
                v = np.dot(self.H[i],self.xante[:2,t]) - b
                f = cp.norm2(v)
                constr += [nu >= 0]
                constr += [f+np.transpose(A@v)@(xv[:2,t]-self.xante[:2,t])/f >= 1 - nu]
                constr += [cp.norm(self.xante[:2,t]-xv[:2,t]) <= tau[t]]  #contrainte de distance entre deux points de deux iterations successives
    #limit the final velocity    
    constr += [cp.norm(uv[:,self.N-2]) <= 10e-9]
    #contrain of the velocity of convergence to the final point % ||target - x_k||_2 <= taui_k
    if(opt_velocity):
      for t in range (0,self.N-1):
        constr += [cp.norm2(xv[:,t] - self.x_fin)<= tau_vel[t]]
    #constrain to optimize the power efficency related to the norm of u
    if(opt_power):
      for t in range (0,self.N-2):
        constr += [cp.norm(uv[:,t]) <= tau_u[t]]

    #resolve the problem    
    prob = cp.Problem(obj , constr)
    prob.solve(solver = cp.SCS,verbose=True)
    xv = np.array(xv.value)
    uv = np.array(uv.value)
    return xv,uv