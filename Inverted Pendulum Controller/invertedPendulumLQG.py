import numpy as np
import control as ct
import scipy
import control.matlab as cm
import matplotlib.pyplot as plt
import slycot

dt = 0.01

m = 0.5     # mass of pend
M = 1      # mass of cart
g = 9.8     # gravitational force
l = 0.5     # length of pend

A = np.array([[0, 1, 0, 0],              # System Creation (Continuous and Discrete)
            [0, -0.1, m*g/M, 0],
            [0, 0, 0, 1],
          [0, 0, (m*g)/(M*l)+(g/l), -0.25]])
B = np.array([[0, ],
            [1/M , ],
             [0, ],
            [1/(M*l),]])
C = np.array([[1., 0., 0., 0.],
              [0, 0, 1, 0]])
D = 0

sys_Cont = ct.StateSpace(A, B, C, D)
sys_Dis = cm.c2d(sys_Cont, dt)

Ad = sys_Dis.A
Bd = sys_Dis.B
Cd = sys_Dis.C

# Controlability test
Controlable = ct.ctrb(Ad, Bd)     
# print(np.linalg.det(Controlable))     # full rank - therefore controllable

# Observability Test:
Observable = ct.obsv(Ad,Cd)   
# print(np.linalg.det(Observable))    # full rank - therefore observable



def pendControl(sys, sysC, x0, x0_est):

  A = sys.A
  B = sys.B
  C = sys.C
  dt = sys.dt
  sim_Time = 8
  t = np.linspace(0, sim_Time, sim_Time/dt)

  Overshoot_LQR = performance_specs[0]
  settle_LQR = performance_specs[1]
  Overshoot_KF = performance_specs[2]
  settle_KF = performance_specs[3]
  
  #LQR Development:

  Q_LQR = np.array([[1, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 10, 0],
                  [0, 0, 0, 0]])
  R_LQR = 0.001

  X,L,K = ct.dare(A,B,Q_LQR,R_LQR)  

  # Kalman Filter Development:

  Q_KF = np.array([[1, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 10, 0],
                  [0, 0, 0, 0]])
  R_KF = np.array([[0.001, 0],
                   [0, 0.001]])

  X_KF, Eigs_KF, L = ct.dare(np.transpose(A),np.transpose(C),Q_KF,R_KF)

  L = np.transpose(L)

  # Simulation

  state_act = x0
  state_est = x0_est
  u = -K@state_est
  y = C@state_act

  for n in range(0,t.size-1):

    state_act_Current = np.reshape(np.array(state_act[:,n]), (4,1))
    state_est_Current = np.reshape(np.array(state_est[:,n]), (4,1))
    y_Current = np.array(y[:,n])
    u_Current = np.array(u[n])

    state_act_New = A@state_act_Current + B@u_Current
    state_est_New = (A-B@K-L@C)@state_est_Current + L@y_Current
    y_New = C@state_act_New
    u_New = -K@state_est_New

    state_act = np.insert(state_act,[n+1],state_act_New,axis=1)
    state_est = np.insert(state_est,[n+1],state_est_New,axis=1)
    y = np.insert(y, [n+1], y_New, axis=1)
    u = np.insert(u, [n+1], u_New, axis=0)

  return state_act, state_est, y, u, np.array(t)


x0 = np.array([[1], [0], [-0.2], [0]])
x0_est = np.array([[1.5],[0], [0.2], [0]])

state_act, state_est, y, u, t = pendControl(sys_Dis, sys_Cont, x0, x0_est)

x_act = state_act[0,:]
xDot_act = state_act[1,:]
theta_act = state_act[2,:]
thetaDot_act = state_act[3,:]

output_Position = np.transpose(y[0,:])
output_theta = np.transpose(y[1,:])

x_est = state_est[0,:]
xDot_est = state_est[1,:]
theta_est = state_est[2,:]
thetaDot_est = state_est[3,:]

fig0 = plt.figure()
plt.plot(t,x_act, linewidth=3)
plt.plot(t,x_est)
plt.legend(("Actual", "Estimated"))
plt.title("Cart Position")
plt.xlabel("time (s)")
plt.ylabel("position (m)")
plt.grid()
fig0.set_size_inches(8,8)

fig1 = plt.figure()
plt.plot(t,xDot_act, linewidth=3)
plt.plot(t,xDot_est)
plt.legend(("Actual", "Estimated"))
plt.title("xDot")
plt.xlabel("time (s)")
plt.ylabel("Cart Velocity (m/s)")
plt.grid()
fig1.set_size_inches(8,8)

fig2 = plt.figure()
plt.plot(t,theta_act, linewidth=3)
plt.plot(t,theta_est)
plt.legend(("Actual", "Estimated"))
plt.title("theta")
plt.xlabel("time (s)")
plt.ylabel("Angular Position (rad)")
plt.grid()
fig2.set_size_inches(8,8)


fig3 = plt.figure()
plt.plot(t,thetaDot_act, linewidth=3)
plt.plot(t,thetaDot_est)
plt.legend(("Actual", "Estimated"))
plt.title("thetaDot")
plt.xlabel("time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.grid()
fig3.set_size_inches(8,8)

fig4 = plt.figure()
plt.plot(t, output_Position)
plt.title("Output (Position)")
plt.xlabel('Time (s)')
plt.ylabel('Measured Position (m)')
plt.grid()
fig4.set_size_inches(8,8)

fig4 = plt.figure()
plt.plot(t, output_theta)
plt.title("Output (theta)")
plt.xlabel('Time (s)')
plt.ylabel('Measured theta (m)')
plt.grid()
fig4.set_size_inches(8,8)

plt.show()
