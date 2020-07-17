"""

Path tracking simulation with SMC steering control and PID speed control.

author Atsushi Sakai (@Atsushi_twi)

"""
import scipy.linalg as la
import matplotlib.pyplot as plt
import math
import numpy as np
import sys
sys.path.append("../../PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise



c = 3
rho = 0.01
k = 0.5
wheel_base = 2.6

# switch function parameter
eps = 0.05
Delta = 0.1


Kp = 1.0  # speed proportional gain

# steering control parameter
KTH = 1.0
KE = 1.0

# LQR parameter
Q = np.eye(4)
R = 2*np.eye(1)

# 车辆参数
M   = 1573.0 #(kg) 总质量 mass
I_z = 2873.0 # (kg/m^2) 绕Z轴的转动惯量 
l_f = 1.10  # (m)
l_r = 1.58  # (m)
C_alpha_f = 20000.0 #(N/rad) 前轮总侧偏刚度
C_alpha_r = 20000.0 #(N/rad) 后轮总侧偏刚度
#V_x = 30.0 # (m/s)

# parameters
dt = 0.1  # time tick[s]
L = l_f+l_r  # Wheel base of the vehicle [m]
max_steer = np.deg2rad(45.0)  # maximum steering angle[rad]

show_animation = True
#  show_animation = False
vehicle_yaw = []

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt
    state.yaw = pi_2_pi(state.yaw)
    return state


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = A.T @ X @ A - A.T @ X @ B @ \
            la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    eigVals, eigVecs = la.eig(A - B @ K)

    return K, X, eigVals

def Sigmoid(x):
    return x/(np.fabs(x) + eps)

def Sat(x):
    val = 0;
    if x >= Delta:
        val = 1
    elif x <= -Delta:
        val = -1
    else:
        val = x/Delta
    return val

#
### state:vehicle state (x,y,yaw,v)
### cx: CubicSpline reference x point 
### cy: CubicSpline reference y point
### cyaw: CubicSpline reference yaw angle
### ck: CubicSpline Curvature
#
yaw_delta = 0.2
def smc_steering_control(state, cx, cy, cyaw, ck):
    
#    rote_state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)
    ind, e = calc_nearest_index(state, cx, cy, cyaw)
    
    # rote 180 deg ,make the 
    if state.yaw > (0.5*np.pi + yaw_delta) or state.yaw < (-0.5*np.pi - yaw_delta): 
        rote_yaw = state.yaw - 0
        line_yaw = cyaw[ind] - 0
        x1 = (cy[ind] - state.y)
        x2 = -(np.tan(line_yaw) - np.tan(rote_yaw))
        s = c*x1 + x2
        delta_c = np.arctan(L*np.power(np.cos(rote_yaw),3)*((L*ck[ind])/(L*np.power(np.cos(line_yaw),3)) + c*x2 + rho*Sigmoid(s) + k*s))
        print(">delta:",delta_c,"yaw:",state.yaw,"x1:",x1,"x2:",x2)
    elif state.yaw < (0.5*np.pi - yaw_delta) and state.yaw > (-0.5*np.pi + yaw_delta):
        x1 = cy[ind] - state.y
        x2 = np.tan(cyaw[ind]) - np.tan(state.yaw)
        s = c*x1 + x2
        delta_c = np.arctan(L*np.power(np.cos(state.yaw),3)*((L*ck[ind])/(L*np.power(np.cos(cyaw[ind]),3)) + c*x2 + rho*Sigmoid(s) + k*s)) 
        print("<delta:",delta_c,"yaw:",state.yaw,"x1:",x1,"x2:",x2)
    else:
        x1 = cy[ind] - state.y
        if(state.yaw <= (np.pi*0.5-0.01) or state.yaw >= (-np.pi*0.5+0.01)):
            x2 = -(cyaw[ind] - state.yaw)
        elif(state.yaw >= (np.pi*0.5+0.01) or state.yaw <= (-np.pi*0.5-0.01)):
            x2 = (cyaw[ind] - state.yaw)
        else:
            x2 = 0
        s = c*x1 + x2
        
        delta_r = np.arctan2(L*ck[ind],1)
        delta_c = delta_r + np.arctan( rho*Sigmoid(s) + k*s)
        print("=delta:",delta_c,"yaw:",state.yaw)
    return delta_c,ind
   
    
def smc_steering_control_rote(state, cx, cy, cyaw, ck):
    
    ind, e = calc_nearest_index(state, cx, cy, cyaw)
    
    if state.yaw >= -np.pi and state.yaw < -0.75*np.pi:
        rote_angle = 0.75*np.pi
    elif state.yaw >= -0.75*np.pi and state.yaw < -0.5*np.pi:
        rote_angle = 0.5*np.pi
    elif state.yaw >= -0.5*np.pi and state.yaw < -0.25*np.pi:
        rote_angle = 0.25*np.pi
    elif state.yaw >= -0.25*np.pi and state.yaw < 0.0:  
        rote_angle = 0.0
    elif state.yaw >= 0.0 and state.yaw < 0.25*np.pi:
        rote_angle = 0.0
    elif state.yaw >= 0.25*np.pi and state.yaw < 0.5*np.pi:
        rote_angle = -0.25*np.pi
    elif state.yaw >= 0.5*np.pi and state.yaw < 0.75*np.pi:    
        rote_angle = -0.5*np.pi
    elif state.yaw >= 0.75*np.pi and state.yaw <= np.pi: 
        rote_angle = -0.75*np.pi
    else:
        print("over the horh")
    
    cos_r = np.cos(rote_angle)
    sin_r = np.sin(rote_angle) 
    
    ROTE = np.zeros((2,2))
    
    ROTE[0,0] =   cos_r
    ROTE[0,1] =  -sin_r
    ROTE[1,0] =   sin_r
    ROTE[1,1] =   cos_r
    
    actual_yaw = state.yaw + rote_angle
    target_yaw = cyaw[ind] + rote_angle
    
    actual_f = np.zeros((2,1))
    actual_r = np.zeros((2,1))
    actual_f[0,0] = state.x
    actual_f[1,0] = state.y
    actual_r = ROTE @ actual_f  
    
    target_f = np.zeros((2,1))
    target_r = np.zeros((2,1))
    target_f[0,0] = cx[ind] 
    target_f[1,0] = cy[ind]   
    target_r = ROTE @ target_f
     
    x1 = target_r[1,0] - actual_r[1,0]
    x2 = np.tan(target_yaw) - np.tan(actual_yaw)
    s = c*x1 + x2
    delta_c = np.arctan(L*np.power(np.cos(actual_yaw),3)*((L*ck[ind])/(L*np.power(np.cos(target_yaw),3)) + c*x2 + rho*Sigmoid(s) + k*s)) 
    print("delta:",delta_c,"x1:",x1,"x2:",x2)

    return delta_c,ind

def stanley_control(state, cx, cy, cyaw, ck, preind):
    
    front_x = state.x + L*np.cos(state.yaw)
    front_y = state.y + L*np.sin(state.yaw)
    
    
    front_state = State(x=front_x, y= front_y, yaw= state.yaw, v=state.v)
    ind, e = calc_nearest_index(front_state, cx, cy, cyaw)
    
    v = state.v
    th_e = -pi_2_pi(state.yaw - cyaw[ind])
    
    if v ==0 :
        return 0,ind
    delta =  th_e + np.arctan(-0.2*e/v)

    return delta, ind

def rear_wheel_feedback_control(state, cx, cy, cyaw, ck, preind):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    omega = v * k * math.cos(th_e) / (1.0 - k * e) - \
        KTH * abs(v) * th_e - KE * v * math.sin(th_e) * e / th_e

    if th_e == 0.0 or omega == 0.0:
        return 0.0, ind

    delta = math.atan2(L * omega / v, 1.0)
    #  print(k, v, e, th_e, omega, delta)

    return delta, ind


def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    A[3, 0] = -v * k**2
    # print(A)

    B = np.zeros((4, 1))
    B[3, 0] = v / L

    K, _, _ = dlqr(A, B, Q, R)
    x = np.zeros((4, 1))

    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt

    ff = math.atan2(L * k, 1)
    fb = pi_2_pi((-K @ x)[0, 0])

    delta = ff + fb

    return delta, ind, e, th_e

def lqr_steering_control_Kinematic(state, cx, cy, cyaw, ck, pe, pth_e, pv):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A = np.zeros((4, 4))
    Ad = np.zeros((4, 4))
    
#    A[0, 1] = 1.0
#    A[1, 2] = v
#    A[2, 3] = 1.0
#    A[3, 0] = -v * k**2
    # print(A)
#    Ad = la.inv(np.identity(4) - dt * 0.5 * A) @ (np.identity(4) + dt * 0.5 * A)
#    Ad = np.identity(4) + A*dt
    Ad[0,0] = 1.0
    Ad[0,1] = dt
    Ad[1,2] = v
    Ad[2,2] = 1.0
    Ad[2,3] = dt
    Ad[3,0] = -v * k**2

    B = np.zeros((4, 1))
    Bd = np.zeros((4, 1))
    
    B[3, 0] = v / L
    
    K, _, _ = dlqr(Ad, B, Q, R)
    x = np.zeros((4, 1))

    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt

    ff = L * k
    fb = pi_2_pi((-K @ x)[0, 0])

    delta = ff + fb

    return delta, ind, e, th_e, v


def lqr_steering_control_test(state, cx, cy, cyaw, ck, pe, pth_e):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A = np.zeros((2, 2))
#    A[0, 0] = 1.0
    A[0, 1] = v
    A[1, 0] = -v * k**2
#    A[1, 1] = 1.0

#    A[3, 0] = -v * k**2
    # print(A)

    Ad = la.inv(np.identity(2) - dt * 0.5 * A) @ (np.identity(2) + dt * 0.5 * A)

    B = np.zeros((2, 1))
    B[1, 0] = dt*v / L

    K, _, _ = dlqr(Ad, B, Q, R)
    x = np.zeros((2, 1))

    x[0, 0] = e
#    x[1, 0] = (e - pe) / dt
    x[1, 0] = th_e
#    x[3, 0] = (th_e - pth_e) / dt

    ff = math.atan2(L * k, 1)
    fb = pi_2_pi((-K @ x)[0, 0])

    delta = ff + fb

    return delta, ind, e, th_e


def lqr_steering_control_dynamic(state, cx, cy, cyaw, ck, pe, pth_e):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)
    
#    crs_e = np.cos(cyaw[ind])*(state.y - cy[ind]) - np.sin(cyaw[ind])*(state.x - cx[ind])

    k = ck[ind]
    
    if state.v == 0:
        V_x = 0.01
    else:
        V_x = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A  = np.zeros((4, 4))
    Ad = np.zeros((4, 4))
    
    A[0, 1] = 1.0
    A[1, 1] = -2.*(C_alpha_f + C_alpha_r)/(M*V_x)
    A[1, 2] =  2.*(C_alpha_f + C_alpha_r)/M
    A[1, 3] = -2.*(C_alpha_f*l_f - C_alpha_r*l_r)/(M*V_x)
    A[2, 3] = 1.0
    A[3, 1] = -2.*(C_alpha_f*l_f - C_alpha_r*l_r)/(I_z*V_x)
    A[3, 2] =  2.*(C_alpha_f*l_f - C_alpha_r*l_r)/I_z
    A[3, 3] = -2.*(C_alpha_f*l_f**2 + C_alpha_r*l_r**2)/(I_z*V_x)
    
#    print(A)
    
    Ad = la.inv(np.identity(4) - dt * 0.5 * A) @ (np.identity(4) + dt * 0.5 * A)
#    Ad = np.identity(4) + A*dt
#    print(Ad)

    B  = np.zeros((4, 1))
    Bd = np.zeros((4, 1))
    
    B[1, 0] = 1.*C_alpha_f/M
    B[3, 0] = 1.*C_alpha_f*l_f/I_z
    
    Bd = B * dt

    K, _, _ = dlqr(Ad, Bd, Q, R)
    x = np.zeros((4, 1))

    x[0, 0] =  e
    x[1, 0] = (e - pe) / dt
    x[1, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt

    ff = math.atan2(L * k, 1)
    fb = pi_2_pi((-K @ x)[0, 0])

    delta = ff + fb

    return delta, ind, e, th_e


def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal):
    T = 500.0  # max simulation time
    goal_dis = 0.2
    stop_speed = 0.05

    state = State(x=-0.0, y= 0.0, yaw=0.0, v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    delta = [0.0]
    
    e, e_th ,e_v  = 0.0, 0.0, 0.0

    while T >= time:
        
        dl, target_ind, e, e_th, e_v = lqr_steering_control_Kinematic(state, cx, cy, cyaw, ck, e, e_th, e_v)
#        dl, target_ind, e, e_th = lqr_steering_control_dynamic(state, cx, cy, cyaw, ck, e, e_th)
#        dl, target_ind, e, e_th = lqr_steering_control(state, cx, cy, cyaw, ck, e, e_th)
#        dl, target_ind, e, e_th = lqr_steering_control_test(state, cx, cy, cyaw, ck, e, e_th)
        
#        dl, target_ind = smc_steering_control_rote(state, cx, cy, cyaw, ck)
#        dl, target_ind = smc_steering_control(state, cx, cy, cyaw, ck)

#        dl, target_ind = rear_wheel_feedback_control(state, cx, cy, cyaw, ck,0)
        
#        dl, target_ind = stanley_control(state, cx, cy, cyaw, ck,0)
        
        
        ai = PIDControl(speed_profile[target_ind], state.v)
        state = update(state, ai, dl)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        delta.append(dl*16.0*57.3)
        
        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
                      + ",target index:" + str(target_ind))
            plt.pause(0.0001)

    return t, x, y, yaw, v, delta


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)

    direction = 1.0

    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    speed_profile[-1] = 0.0

    return speed_profile


def main():
    print("LQR steering control tracking start!!")
#    ax = [0.0, 6.0, 12.5, 28.0, 37.5, 46.0, 58.0]
#    ay = [0.0, -5.0, 6.0, -3.5, 8.0, -3.0, 5.0]
    
#    ax = [0.0,  6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
#    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    
#    ax = [0.0,  -6.0, -12.5, -10.0, -7.5, -3.0, 1.0]
#    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    
#    ax = [0.0, 12.0, 5.0, 0.0, -5.0, -12.0, 0.0]
#    ay = [0.0, 15.0, 20.0, 15.0, 20.0, 15.0, 0.0]
    
    ax = [0.0, 10.0, 15.0, 20.0, 60.0]
    ay = [0.0,  2.0, 10.0, 18.0, 20.0]
    
    
#    ax = [0.0,  1.0, 1.5,  2.0, 3.0]
#    ay = [0.0, 0.05, 0.1, 0.15, 0.2]
    
    goal = [ax[-1], ay[-1]]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    target_speed = 7  # simulation parameter km/h -> m/s

    sp = calc_speed_profile(cx, cy, cyaw, target_speed)

    t, x, y, yaw, v, delta = closed_loop_prediction(cx, cy, cyaw, ck, sp, goal)

    if show_animation:  # pragma: no cover
        plt.close()
        plt.subplots(1)
        plt.plot(ax, ay, "xb", label="input")
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots(1)
        plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("yaw angle[deg]")

        plt.subplots(1)
        plt.plot(s, ck, "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("curvature [1/m]")

        plt.subplots(1)
        plt.plot(x, delta, "-g", label="delta")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("curvature [1/m]")
        
        plt.show()


if __name__ == '__main__':
    main()
