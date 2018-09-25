
import numpy as np
import math
import rospy
import roslib
import tf
import intera_interface
import cv2 
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from SawyerClass import Sawyer
from keras.models import load_model


plt.style.use('ggplot')
plt.rcParams['text.latex.preamble']=[r"\usepackage{lmodern}"]
#Options
params = {'text.usetex' : True,
          'font.size' : 11,
          'font.family' : 'lmodern',
          'text.latex.unicode': True,
          }
plt.rcParams.update(params) 



def psiF(h, c, s, i):
    return np.exp(-h[i]*(s-c[i])**2)

def plotGaussians(sv, gv, parameters, w_all, title="Gaussians"):
    colrs = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    if w_all is None: w_all = np.ones(parameters[0][gv].shape)
    plt.figure()
    #plt.suptitle("%s" % title, fontsize= 18)
    for g in range(len(parameters)):
        plt.subplot(4, 1, g + 1)
        for i in range(len(parameters[g][gv])):
            plt.plot(parameters[g][sv], parameters[g][gv][i]*w_all[g][i], color = colrs[i % len(colrs)])
            plt.ylabel("q%s: $w_i \psi_i$" % str(g + 1), fontsize=16)
        if g != len(parameters) - 1: 
            plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
        else:
            plt.xlabel("time (s)", fontsize=16)
    plt.grid()
    plt.savefig('%s%s_%s.png' % (fig_dir, fname.split('.')[0], title.replace(" ", "_").lower()))
    plt.show()

def DynamicMotionPrimitive(x, time, convert):
    time = time / convert  # Scale down time
    time = time - time[0]  # Make sure t[0] = 0
    dtx = np.diff(time)[0]  # TODO: should be dt
    dx = np.concatenate([[0], np.diff(x)]) / dtx  # Generate velocities
    ddx = np.concatenate([[0], np.diff(dx)]) / dtx  # Generate accelerations
    x0 = x[0]
    gx = x[-1]
    h = 1

    par = {}
    par['x']=x
    par['dx']=dx
    par['ddx']=ddx
    par['time']=time
    par['dt']=dtx
    par['x0']=x0
    par['gx']=gx
    par['ng'] = 30
    par['h'] = np.concatenate(np.ones((1, par['ng'])) * h)
    par['s'] = 1
    par['as'] = 6 
    par['tau'] = time[-1]
    par['K'] = 4444
    par['D'] = par['K']/4
    par['length'] = len(time)

    stime = []
    sE_x = np.array([])
    ftarget = np.array([])

    for i in range(0, len(time)):  # TODO: check end
        t = time[i]
        s = np.exp((-1 * par['as'] * t) / par['tau'])
        stime.append(s)
        ftar_tmp = (-1 * par['K'] * (gx - x[i]) + par['D'] * dx[i] + par['tau'] * ddx[i]) / (gx - x0)
        ftarget = np.append(ftarget, ftar_tmp)
        sE_x = np.append(sE_x, s * (gx - x0))

    # Calculate gaussian parameters
    # TODO: check if stime is in order, or err, smallest first largest last
    incr = (max(stime) - min(stime)) / (par['ng'] - 1)  # TODO: replace w/ index based min/max
    c = np.arange(min(stime), max(stime) + incr, incr)  # TODO: compare outputs
    lrc = c[::-1]
    ctime = (-1 * par['tau'] * np.log(lrc)) / par['as']
    d = np.diff(c)  # TODO: lc
    c = c / d[0]  # TODO: lc
    par['c'] = c
    w_x = []

    # Regression
    psV = []

    for i in range(0, par['ng']):  # TODO: possibly add one, needs to go to 100

        psV_x = []
        MypsV_x = np.zeros((par['ng'],len(time)))

        for j in range(0, len(time)):  # TODO: may not go to end
            psV_x.append(psiF(par['h'], par['c'], stime[j] / d[0], i))
            MypsV_x[i][j]=(psiF(par['h'], par['c'], stime[j] / d[0], i))

        psV_x = np.array(psV_x)#
        psV.append(psV_x)

        w_x.append((np.dot(sE_x[np.newaxis], np.dot(np.diag(psV_x),(ftarget[np.newaxis]).T)))/ np.dot((sE_x[np.newaxis]), np.dot(np.diag(psV_x), sE_x[np.newaxis].T)))
    
    par['psV']= np.array(psV)
    par['MypsV_x']=MypsV_x
    par['psV_x']=psV_x
    par['c'] = c
    par['stime'] = stime
    par['ftarget'] = ftarget
    par['w_x'] = np.concatenate(np.array(w_x))
    par['x0'] = x0
    par['d0'] = d[0]
    par['ctime'] = ctime

    return par

def DMP_Generate(par, target):

    f_replay_x = []
    fr_x_zeros = []

    # TODO: check usage
    ydd_x_r = 0
    yd_x_r = 0
    y_x_r = par['x0']

    ydd_xr = []
    yd_xr = []
    y_xr = []

    for j in range(0, len(par['time'])):  # TODO check if reaches end
        psum_x = 0
        pdiv_x = 0
        for i in range(0, par['ng']):  # TODO check what i is doing in psiF below
            psum_x += psiF(par['h'], par['c'], par['stime'][j] / par['d0'], i) * par['w_x'][i]
            pdiv_x += psiF(par['h'], par['c'], par['stime'][j] / par['d0'], i)

        # Generate new trajectories according to new control input
        f_replay_x.append((psum_x / pdiv_x) * par['stime'][j] * (target - par['x0']))

        if j > 0:  # TODO: check
            if np.sign(f_replay_x[j - 1]) != np.sign(f_replay_x[j]):
                fr_x_zeros.append(j - 1)  # TODO: lc

        ydd_x_r = (par['K'] * (target - y_x_r) - (par['D'] * yd_x_r) + (target - par['x0']) * f_replay_x[j]) / par['tau']
        yd_x_r = yd_x_r + (ydd_x_r * par['dt']) / par['tau']
        y_x_r = y_x_r + (yd_x_r * par['dt']) / par['tau']

        ydd_xr.append(ydd_x_r[0])
        yd_xr.append(yd_x_r[0])
        y_xr.append(y_x_r[0])

    results = {}
    results['ydd_xr'] = ydd_xr
    results['yd_xr'] = yd_xr
    results['y_xr'] = y_xr
    results['fr_x_zeros'] = fr_x_zeros
    results['f_replay_x'] = f_replay_x

    return results

# Models location
forward_model_file = 'models/forwardmodel_6M.h5'
inverse_model_file = 'models/MLP_2.h5'
fig_dir = '/home/cloud/Desktop/mit_urtc_19/'
fname = 'demo-wave.txt'
# Load the models
forwardModel= load_model(forward_model_file)
inverseModel= load_model(inverse_model_file)

# Review of the Models
#forwardModel.summary()
#inverseModel.summary()


# Declare the joint position history and time history

## DO FILE LOADING STUFF HERE

data = np.loadtxt(fname, skiprows = 1, delimiter = ',')

# skip: 2, 4, 6
q = np.array(data[:,1:8])
target = q[-1]
time = data[:-1, 0]
q = q[:-1]

q1=q[:,0]
q2=q[:,1]
q3=q[:,3]
q4=q[:,5]

print(data.shape)
print(time.shape)
print(target.shape)
print(q1.shape)

#plt.figure(1)
#plt.suptitle('Joint History Response', fontsize=18)

#plt.subplot(4,1,1)
#plt.plot(time,q1,color='blue')
##plt.plot(time,r1['y_xr'],color='red')
#plt.ylabel('q1 (rad)', fontsize = 16)

#plt.subplot(4, 1, 2)
#plt.plot(time,q2,color='blue')
##plt.plot(time,r2['y_xr'],color='red')
#plt.ylabel('q2 (rad)', fontsize = 16)

#plt.subplot(4, 1, 3)
#plt.plot(time,q3,color='blue')
#plt.plot(time,r3['y_xr'],color='red')
#plt.ylabel('q3 (rad)', fontsize = 16)

#plt.subplot(4, 1, 4)
#plt.plot(time,q4,color='blue')
#plt.plot(time,r4['y_xr'],color='red')
#plt.ylabel('q4 (rad)', fontsize = 16)
#plt.xlabel('time (s)', fontsize = 16)

#plt.show()

#Learn the DMP parameters
scale=6.9 #TODO: Joe changed
p1=DynamicMotionPrimitive(q1, time, scale)
p2=DynamicMotionPrimitive(q2, time, scale)
p3=DynamicMotionPrimitive(q3, time, scale)
p4=DynamicMotionPrimitive(q4, time, scale)

print("STIME FINAL: %d" % p1['stime'][-1])

#Get the DMP results
r1=DMP_Generate(p1, p1['x'][-1])
r2=DMP_Generate(p2, p2['x'][-1])
r3=DMP_Generate(p3, p3['x'][-1])
r4=DMP_Generate(p4, p4['x'][-1])

plt.figure(1)
#plt.suptitle('Joint History Response', fontsize=18)

plt.subplot(4,1,1)
plt.plot(time,q1,color='blue', label='q')
plt.plot(time,r1['y_xr'],color='red', label='$\dot{q}$')
plt.legend(loc="upper right")
plt.ylabel('q1 (rad)', fontsize = 16)
plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)

plt.subplot(4, 1, 2)
plt.plot(time,q2,color='blue', label='q')
plt.plot(time,r2['y_xr'],color='red', label='$\dot{q}$')
plt.ylabel('q2 (rad)', fontsize = 16)
plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)

plt.subplot(4, 1, 3)
plt.plot(time,q3,color='blue', label='q')
plt.plot(time,r3['y_xr'],color='red', label='$\dot{q}$')
plt.ylabel('q3 (rad)', fontsize = 16)
plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)

plt.subplot(4, 1, 4)
plt.plot(time,q4,color='blue', label='q')
plt.plot(time,r4['y_xr'],color='red', label='$\dot{q}$')
plt.ylabel('q4 (rad)', fontsize = 16)
plt.xlabel('time (s)', fontsize = 16)

plt.savefig('%s%s_joint_history.png' % (fig_dir, fname.split('.')[0]))
plt.show()


parameters=[p1,p2,p3,p4]

plotGaussians('stime', 'psV', parameters, None, "Initial Policies")

plotGaussians('stime', 'psV', parameters, np.array([p1['w_x'], p2['w_x'], p3['w_x'], p4['w_x']]), "Learned Policies")

tp = forwardModel.predict(np.array([q1, q2, q3, q4]).T)/100
fp = forwardModel.predict(np.array([r1['y_xr'], r2['y_xr'], r3['y_xr'], r4['y_xr']]).T)/100

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(tp[:,0], tp[:,1], tp[:,2], 'b')
ax.scatter(fp[:,0], fp[:,1], fp[:,2], 'r')
plt.show()

y1 = np.array(r1['y_xr'])[:, np.newaxis]
y2 = np.array(r2['y_xr'])[:, np.newaxis]
y3 = np.array(r3['y_xr'])[:, np.newaxis]
y4 = np.array(r4['y_xr'])[:, np.newaxis]


jnots = np.zeros((len(q1), 1))
traj_final = np.concatenate(((y1, y2, jnots, y3, jnots, y4, jnots)), axis=1)
traj_final = np.concatenate((traj_final,np.multiply(np.ones((len(q1),1)),0.0402075604203)),axis=1)

# Create the trajectory file 
#time=np.linspace(0, time[-1],len(q1)).reshape((len(q1),1))

traj_final=np.concatenate((time[:, np.newaxis],traj_final),axis=1)

# Save trajectory
np.savetxt('traj_final.txt', traj_final, delimiter=',',header='time,right_j0,right_j1,right_j2,right_j3,right_j4,right_j5,right_j6,right_gripper',comments='',fmt="%1.12f") 
