import serial
from pyquaternion import Quaternion
import numpy as np
import time
import glob
from matplotlib import pyplot as plt
import matplotlib.animation as animation

ports = glob.glob('/dev/tty[A-Za-z]*')
for port in ports:
    if "ACM" in port:
        s= serial.Serial(
            port=port, baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
        )
        print(f'connected to {port}')

GYRO_SCALE=1/52.3

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1,projection='3d')

def progress_bar(current, total,gyro, bar_length=20):
    fraction = current / total

    arrow = int(fraction * bar_length - 1) * '-' + '>'
    padding = int(bar_length - len(arrow)) * ' '

    ending = '\n' if current == total else '\r'

    print(f'Progress: [{arrow}{padding}] {int(fraction*100)}% | gyro: {round(gyro[0],2)} {round(gyro[1],2)} {round(gyro[2],2)}',end=ending)

vZ = np.array([0,0,1])
vX = np.array([1,0,0])
vY = np.array([0,1,0])

Agndapp = Quaternion(1,0,0,0)

prev_time=0

VVX=[]
VVY=[]
VVZ=[]

LEN=600
SAMPLING_LEN=100

gyro_bias=np.array([0,0,0],dtype=float)
for i in range(SAMPLING_LEN):
    serialString = s.readline()
    data=serialString.decode("utf-8")
    if len(data)>10:
        splt=data.split(";")
        splt[0]=splt[0][-4:]
        splt[-1]=splt[-1][:-1] #remove \n symbol
        try:
            accgyro=list(map(float,splt))
            acc=accgyro[:2]
            gyro=list(map(lambda x:x*GYRO_SCALE,accgyro[3:]))
            progress_bar(i,SAMPLING_LEN,gyro)
            gyro_bias+=np.array(gyro)
        except Exception as e:
            print(e)

gyro_bias/=SAMPLING_LEN
print("\n")
print(f"set gyro biases {gyro_bias} from {SAMPLING_LEN} samples")
print("\n")

for i in range(LEN):
    serialString = s.readline()
    data=serialString.decode("utf-8")
    if len(data)>10:
        splt=data.split(";")
        splt[0]=splt[0][-4:]
        splt[-1]=splt[-1][:-1] #remove \n symbol
        try:
            accgyro=list(map(float,splt))
            acc=accgyro[:2]
            gyro=list(map(lambda x:x*GYRO_SCALE,accgyro[3:]))
            
            #integration 
            if len(gyro) == 3:
                omega=np.array(gyro) - gyro_bias
            else:
                omega=np.array([0,0,0])
            #printing
            progress_bar(i,LEN,omega)

            dt=time.time()-prev_time
            prev_time=time.time()

            Mapp=Quaternion(scalar=1,vector=0.5 * dt * omega )

            Agndapp = Agndapp * Mapp

            Agndapp = Agndapp.normalised

            vz=Agndapp.rotate(vZ)
            vy=Agndapp.rotate(vY)
            vx=Agndapp.rotate(vX)

            x,y,z=vz    
            
            VVX.append(x)
            VVY.append(y)
            VVZ.append(z)
        except Exception as e:
            print(e)

def animate(frame):
    progress_bar(frame,LEN,[0,0,0])
    ax.clear()
    ax.set_xlim([-1,1])
    ax.set_ylim([-1,1])
    ax.set_zlim([-1,1])
    ax.plot(VVX[:frame],VVY[:frame],VVZ[:frame])
    ax.scatter3D(VVX[frame], VVY[frame],VVZ[frame])
    ax.set_xlabel(f'frame {frame}')
    
print('\n')           
print('creating video')
ani = animation.FuncAnimation(fig, animate, frames=len(VVX),interval=50)
ani.save('vid.mp4')