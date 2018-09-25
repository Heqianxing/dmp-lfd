#!/usr/bin/env python2

import scipy.io
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from keras.models import load_model

mat = scipy.io.loadmat('DMP_3.mat')
#q = mat['DMP'][0][0][0]
#p = mat['DMP'][0][0][1]
target = np.concatenate(mat['DMP'][0][0][2])
#time = np.concatenate(mat['DMP'][0][0][3])

data = np.loadtxt('data_collector/demo1.txt', skiprows = 1, delimiter = ',')

q = np.array(data[:,1:8])
target = q[-1]
q = q[:-1]
time = data[:-1, 0]

def psiF(h, c, s, i):
    return np.exp(-h[i]*(s-c[i])**2)


def DynamicMotionPrimitive(x, time, convert, target):
    time = time / convert  # Scale down time
    time = time - time[0]  # Make sure t[0] = 0
    dtx = np.diff(time)[0]  # TODO: should be dt
    dx = np.concatenate([[0], np.diff(x)]) / dtx  # Generate velocities
    ddx = np.concatenate([[0], np.diff(dx)]) / dtx  # Generate accelerations
    x0 = x[0]
    gx = x[-1]
    h = 1

    par = {}
    par['ng'] = 100
    par['h'] = np.concatenate(np.ones((1, par['ng'])) * h)
    par['s'] = 1
    par['as'] = 5
    par['tau'] = time[-1]
    par['K'] = 10000
    par['D'] = 100

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
    for i in range(0, par['ng']):  # TODO: possibly add one, needs to go to 100

        psV_x = []
        for j in range(0, len(time)):  # TODO: may not go to end
            psV_x.append(psiF(par['h'], par['c'], stime[j] / d[0], i))
        psV_x = np.array(psV_x)

        w_x.append((np.dot(sE_x[np.newaxis], np.dot(np.diag(psV_x),(ftarget[np.newaxis]).T)))
                   / np.dot((sE_x[np.newaxis]), np.dot(np.diag(psV_x), sE_x[np.newaxis].T)))

    # store results
    r = par
    r['length'] = len(time)
    r['time'] = time
    r['stime'] = stime
    r['ftarget'] = ftarget
    r['w_x'] = w_x
    r['x0'] = x0
    r['gx'] = gx
    r['d0'] = d[0]
    r['dt'] = dtx
    r['ctime'] = ctime

    # generate est. DMP

    f_replay_x = []
    fr_x_zeros = []

    # TODO: check usage
    ydd_x_r = 0
    yd_x_r = 0
    y_x_r = r['x0']

    ydd_xr = []
    yd_xr = []
    y_xr = []

    for j in range(0, len(r['time'])):  # TODO check if reaches end
        psum_x = 0
        pdiv_x = 0
        for i in range(0, r['ng']):  # TODO check what i is doing in psiF below
            psum_x += psiF(r['h'], r['c'], r['stime'][j] / r['d0'], i) * r['w_x'][i]
            pdiv_x += psiF(r['h'], r['c'], r['stime'][j] / r['d0'], i)

        # Generate new trajectories according to new control input
        f_replay_x.append((psum_x / pdiv_x) * r['stime'][j] * (target - r['x0']))

        if j > 0:  # TODO: check
            if np.sign(f_replay_x[j - 1]) != np.sign(f_replay_x[j]):
                fr_x_zeros.append(j - 1)  # TODO: lc

        ydd_x_r = (r['K'] * (target - y_x_r) - (r['D'] * yd_x_r) + (target - r['x0']) * f_replay_x[j]) / r['tau']
        yd_x_r = yd_x_r + (ydd_x_r * dtx) / r['tau']
        y_x_r = y_x_r + (yd_x_r * dtx) / r['tau']

        ydd_xr.append(ydd_x_r)
        yd_xr.append(yd_x_r)
        y_xr.append(y_x_r)

    results = {}
    results['ydd_xr'] = ydd_xr
    results['yd_xr'] = yd_xr
    results['y_xr'] = y_xr
    results['fr_x_zeros'] = fr_x_zeros
    results['f_replay_x'] = f_replay_x

    return results


fm = load_model('models/fm.h5')


p = fm.predict(q)

print(q.shape)
print(p.shape)
print(target.shape)
print(time.shape)
DMP_res = []
for i in range(0, 1):
    temp = []
    for j in range(0, len(q[0])):
        targ = target[j]
        result = DynamicMotionPrimitive(q[:, j], time, 1000, targ)
        # TODO: missing line temp(:,j) = results['y_xr
        DMP_res.append(result['y_xr'])  # changed bc missing above line

DMP_res = np.array(DMP_res)
#print(DMP_res.shape)
    
    
cart = fm.predict(np.squeeze(DMP_res).T)       

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(p[:,0], p[:,1], p[:,2], 'b')
ax.scatter(cart[0], cart[1], cart[2], 'r')
plt.show()


DMP_res = []

for i in range(0, 1):
    temp = []
    for j in range(0, 3):
        targ = target[j]
        result = DynamicMotionPrimitive(p[:, j], time, 1000, targ)
        # TODO: missing line temp(:,j) = results['y_xr
        DMP_res.append(result['y_xr'])  # changed bc missing above line


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(p[:,0], p[:,1], p[:,2], 'b')
ax.scatter(DMP_res[0], DMP_res[1], DMP_res[2], 'r')
plt.show()
