import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt

filename = "csv/3d_coor_from_kalman/kalman4.csv"

GRAVITY = 9800 * np.array([0,0,1])
FRICTION = 0.5*0.445*1.225*(40/2)**2*np.pi/10**6/2.7
POS_COEF1 = 0.6
VEL_COEF1 = 0.6
POS_COEF2 = 0.5
VEL_COEF2 = 0.5
TIME_INTERVAL = 1.0/100.0
mode_switch = 1 # 1-filter1 2-filter2
mode_count = 0
mode_thres = 10

def kalman_filter1(old_state, new_state, input_valid = True):
    """
        Fitting with the model for the target flying in the air
    """
    state = np.zeros(6)
    if input_valid:
        state[0:3] = POS_COEF1 * new_state \
            + (1 - POS_COEF1) * (old_state[0:3] + old_state[3:6] * TIME_INTERVAL)
        # print new_state, old_state[0:3]
        friction = -FRICTION*np.linalg.norm(old_state[3:6])*old_state[3:6]
        state[3:6] = VEL_COEF1 * (new_state - old_state[0:3]) / TIME_INTERVAL \
            + (1 - VEL_COEF1) * (old_state[3:6] - (GRAVITY - friction) * TIME_INTERVAL)
    else:
        state[0:3] = old_state[0:3] + old_state[3:6] * TIME_INTERVAL
        friction = -FRICTION*np.linalg.norm(old_state[3:6])*old_state[3:6]
        state[3:6] = old_state[3:6] - (GRAVITY - friction) * TIME_INTERVAL
    return state

def kalman_filter2(old_state, new_state, input_valid = True):
    """
        Fitting with the model for the target with no accerlation
    """
    state = np.zeros(6)
    if input_valid:
        state[0:3] = POS_COEF2 * new_state \
            + (1 - POS_COEF2) * (old_state[0:3] + old_state[3:6] * TIME_INTERVAL)
        state[3:6] = VEL_COEF2 * (new_state - old_state[0:3]) / TIME_INTERVAL \
            + (1 - VEL_COEF2) * old_state[3:6]
    else:
        state[0:3] = old_state[0:3] + old_state[3:6] * TIME_INTERVAL
        state[3:6] = old_state[3:6]
    return state

if __name__ == '__main__':
    data = genfromtxt(filename, delimiter=',')
    # print data
    
    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    # ax.set_xlim([np.amin(data),np.amax(data)])
    # ax.set_ylim([np.amin(data),np.amax(data)])
    # ax.set_ylim([-500, 1000])
    ax.set_zlim([-200, 1000])
    ax.plot(data[:,0], data[:,1], data[:,2], label='filtered')
    ax.plot(data[:,6], data[:,7], data[:,8], label='origin')

    # ax.plot(raw_data[:,0], raw_data[:,1], raw_data[:,2], label='raw')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend()

    plt.show()