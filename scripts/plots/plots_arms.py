# -*- coding: utf-8 -*-

#
# Plots for NAO
#

from __future__ import unicode_literals

import numpy as np
import matplotlib.pyplot as plt


folder = '../../data/'
prefix = 'motion_arms_'

# Read the files
path = folder + prefix
q = np.loadtxt(path+'q.txt')
xl     = np.loadtxt(path+"LHand.txt")
xldes  = np.loadtxt(path+"LHand_des.txt")
xr     = np.loadtxt(path+"RHand.txt")
xrdes  = np.loadtxt(path+"RHand_des.txt")
stime = np.loadtxt(path+"time.txt")

# Set the plotting times
tf = 600
q = q[:tf,:];
time = stime[:tf]
xl = xl[:tf,:]; xldes = xldes[:tf,:]; xr = xr[:tf,:]; xrdes = xrdes[:tf,:]; 

# Plot the temporal joint configuration
plt.plot(q[:,0], q[:,8:])
plt.xlabel('tiempo [s]')
plt.ylabel('Valor articular [rad]')
plt.title('Evolución articular en el tiempo')
# plt.legend((r'$q_1$', r'$q_2$', r'$q_3$', r'$q_4$', r'$q_5$', r'$q_6$', r'$q_7$'), loc='best')
plt.grid()
plt.show()

# ---------------------
# Right Hand
# ---------------------
plt.subplot(121)
plt.plot(xr[:,0], xr[:,1], linewidth=2)
plt.plot(xr[:,0], xr[:,2], linewidth=2)
plt.plot(xr[:,0], xr[:,3], linewidth=2)
plt.plot(xrdes[:,0], xrdes[:,1:4],'k--')
plt.xlabel('tiempo [s]')
plt.ylabel('posición [m]')
plt.title("Posición de la mano derecha")
plt.legend(('x', 'y', 'z'), loc='best')
#plt.grid()
# Plot orientation
plt.subplot(122)
plt.plot(xr[:,0], xr[:,4], linewidth=2)
plt.plot(xr[:,0], xr[:,5], linewidth=2)
plt.plot(xr[:,0], xr[:,6], linewidth=2)
plt.plot(xr[:,0], xr[:,7], linewidth=2)
plt.plot(xrdes[:,0], xrdes[:,4:], 'k--')
plt.xlabel('tiempo [s]')
plt.ylabel('cuaternión')
plt.title('Orientación de la mano derecha')
plt.legend((r'$\varepsilon_w$', r'$\varepsilon_x$', r'$\varepsilon_y$',
            r'$\varepsilon_z$'), loc="best")
#plt.axis('equal')
#plt.grid()
plt.show()


# ---------------------
# Left Hand
# ---------------------
plt.subplot(121)
plt.plot(xl[:,0], xl[:,1], linewidth=2)
plt.plot(xl[:,0], xl[:,2], linewidth=2)
plt.plot(xl[:,0], xl[:,3], linewidth=2)
plt.plot(xldes[:,0], xldes[:,1:4],'k--')
plt.xlabel('tiempo [s]')
plt.ylabel('posición [m]')
plt.title("Posición de la mano izquierda")
plt.legend(('x', 'y', 'z'), loc='best')
#plt.grid()
# Plot orientation
plt.subplot(122)
plt.plot(xl[:,0], xl[:,4], linewidth=2)
plt.plot(xl[:,0], xl[:,5], linewidth=2)
plt.plot(xl[:,0], xl[:,6], linewidth=2)
plt.plot(xl[:,0], xl[:,7], linewidth=2)
plt.plot(xldes[:,0], xldes[:,4:], 'k--')
plt.xlabel('tiempo [s]')
plt.ylabel('cuaternión')
plt.title('Orientación de la mano izquierda')
plt.legend((r'$\varepsilon_w$', r'$\varepsilon_x$', r'$\varepsilon_y$',
            r'$\varepsilon_z$'), loc="best")
#plt.axis('equal')
#plt.grid()
plt.show()


# ---------------------
# Base
# ---------------------
plt.subplot(121)
plt.plot(q[:,0], q[:,1], linewidth=2)
plt.plot(q[:,0], q[:,2], linewidth=2)
plt.plot(q[:,0], q[:,3], linewidth=2)
plt.xlabel('tiempo [s]')
plt.ylabel('posición [m]')
plt.title("Posición de la base flotante")
plt.legend(('x', 'y', 'z'), loc='best')
plt.grid()
# plt.show()
# Plot orientation
plt.subplot(122)
plt.plot(q[:,0], q[:,4], linewidth=2)
plt.plot(q[:,0], q[:,5], linewidth=2)
plt.plot(q[:,0], q[:,6], linewidth=2)
plt.plot(q[:,0], q[:,7], linewidth=2)
plt.xlabel('tiempo [s]')
plt.ylabel('cuaternión')
plt.title('Orientación de la base flotante')
plt.legend((r'$\varepsilon_w$', r'$\varepsilon_x$', r'$\varepsilon_y$',
            r'$\varepsilon_z$'), loc="best")
plt.grid()
plt.show()


# Plot computation time
plt.plot(time)
plt.xlabel('iteración')
plt.ylabel('tiempo [ms]')
plt.title('Tiempo de cálculo')
plt.grid()
plt.show()
