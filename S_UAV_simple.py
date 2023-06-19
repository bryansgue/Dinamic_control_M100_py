#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose2D, Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
import time
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy



#pkg> status ## Es para check los paquetes instalados
# No es necesario en Python, ya que los paquetes se importan directamente

maxloops = 1000
rosloops = 0

hdp_vision = [0,0,0.0,0]
pos = [0,0,0.1,0]
vel = [0,0,0,0]
axes = [0,0,0,0,0,0]
axes2 = [0,0,0,0,0,0]

def quaternion_to_euler(qw, qx, qy, qz):
    # Calcula los ángulos de Euler en la convención ZYX
    # yaw (Z), pitch (Y), roll (X)
    
    # Normaliza el cuaternio
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw /= norm
    qx /= norm
    qy /= norm
    qz /= norm
    
    # Calcula los ángulos de Euler
    yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    pitch = np.arcsin(2*(qw*qy - qx*qz))
    roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
    
    # Devuelve los ángulos de Euler en radianes
    return [roll, pitch, yaw]

def odo_callback(msg):
    global pos
    global vel

    x_real = msg.pose.pose.position.x 
    y_real = msg.pose.pose.position.y
    z_real = msg.pose.pose.position.z
    qx_real = msg.pose.pose.orientation.x
    qy_real = msg.pose.pose.orientation.y
    qz_real = msg.pose.pose.orientation.z
    qw_real = msg.pose.pose.orientation.w

    vx_real = msg.twist.twist.linear.x
    vy_real = msg.twist.twist.linear.y
    vz_real = msg.twist.twist.linear.z
    wx_real = msg.twist.twist.angular.x
    wy_real = msg.twist.twist.angular.y
    wz_real = msg.twist.twist.angular.z

    euler_angles = quaternion_to_euler(qw_real, qx_real, qy_real, qz_real)
    #print(euler_angles)
    
    pos = [x_real, y_real, z_real, euler_angles[2]]
    vel = [vx_real, vy_real, vz_real, wz_real]

def visual_callback(msg):

    global hdp_vision 

    vx_visual = msg.linear.x
    vy_visual = msg.linear.y
    vz_visual = msg.linear.z
    wx_visual = msg.angular.x
    wy_visual = msg.angular.y
    wz_visual = msg.angular.z

    hdp_vision = [vx_visual, vy_visual, vz_visual, wz_visual]
    #print(euler_angles)


    
def rc_callback(data):
    # Extraer los datos individuales del mensaje
    global axes
    axes_aux = data.axes
    psi = -np.pi / 2

    R = np.array([[np.cos(psi), -np.sin(psi), 0, 0, 0, 0],
                [np.sin(psi), np.cos(psi), 0, 0, 0, 0],
                [0, 0, -1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]])
    axes = R@axes_aux
    #print(axes[5])

def joystick_callback(data):
    # Extraer los datos individuales del mensaje
    global axes2
    axes_aux= data.axes
    axes2 = [axes_aux[1],axes_aux[0],axes_aux[2],axes_aux[3],axes_aux[4],axes_aux[5]]
  

def sensor1_callback(msg):
    sensor1 = [msg.x, msg.y, msg.z]

def sendvalues(pub_obj, vc):
    msg = Twist()
    #vc = [float(valor) for valor in vc_no[:, 0]]
    msg.linear.x = vc[0]
    msg.linear.y = vc[1]
    msg.linear.z = vc[2]
    msg.angular.z = vc[3]
    pub_obj.publish(msg)


    

def main(pub_control):

    print("OK, controller is running!!!")
    
    # Espera 1 segundo para comenzar la ejecución
    

    
    hz = 30  # Frecuencia de actualización
    ts = 1 / hz
    
    
    # Inicialización de matrices
    
    J = np.zeros((4, 4))

    #Gains = np.array([1.5000 ,   1.0011,    1.5000,    1.5000,   1.5000,    1.0576,    1.5000,    1.5000,    1.5000,    1.0001,    1.5000,    1.4999,  1.5000,    1.0000 ,   1.5000  ,  1.4999])
    Gains = 0.5*np.array([1,   1,    1,    1,   1,    1,    1,    1,    1,    1,    1,    1,  1,    1,   1 ,  1])

    K1 = np.diag([Gains[0], Gains[1], Gains[2], Gains[3]])  # Distribuir los primeros 4 elementos de Gains en la matriz K1
    K2 = np.diag([Gains[4], Gains[5], Gains[6], Gains[7]])  # Distribuir los elementos 5 al 8 de Gains en la matriz K2
    K3 = np.diag([Gains[8], Gains[9], Gains[10], Gains[11]])  # Distribuir los elementos 9 al 12 de Gains en la matriz K3
    K4 = np.diag([Gains[12], Gains[13], Gains[14], Gains[15]])  # Distribuir los elementos 13 al 16 de Gains en la matriz K4

    

    a = 0
    b = 0
    time_init = time.time()

    # Inicialización de vc_anterior
    vc = np.array([0,0,0,0]) 
    vc_anterior = vc.copy()

    ros_rate = 30  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate
    
    
    while True:
        tic = time.time()

        condicion = axes[5]
        if condicion  == -4545.0:
            hdp = [axes[0], axes[1], axes[3], axes[2]]
            #hdp[:, k] = [axes2[0], axes2[1], axes2[5], axes2[2]/2]
            print("Kinematic: ", " ".join("{:.2f}".format(value) for value in np.round(hdp, decimals=2)), end='\r')


        elif condicion == -10000.0:        
            hdp = [hdp_vision[0], hdp_vision[1], axes[3]/4, hdp_vision[3]]
            print("Servo-Visual:", " ".join("{:.2f}".format(value) for value in np.round(hdp, decimals=2)), end='\r')

        else:
            hdp = [0.0, 0, 0.0, 0]    
            hdp = [hdp_vision[0], hdp_vision[1], axes[3]/4, hdp_vision[3]]
            print("Manual", " ".join("{:.2f}".format(value) for value in np.round(hdp, decimals=2)), end='\r')
   

        h = pos
        v = vel

        psi = h[3]
        J[0, 0] = np.cos(psi)
        J[0, 1] = -np.sin(psi)
        J[1, 0] = np.sin(psi)
        J[1, 1] = np.cos(psi)
        J[2, 2] = 1
        J[3, 3] = 1

        
        
        vc = np.array(hdp) 

        
        vcp = (vc - vc_anterior) / ts

        
        x = np.array([ 0.3259, 0,0.3787, 0,0.4144, 0, 0, 0.2295, 0.7623, 0, 0.8279,0,0.8437, 0, 0, 0, 0,1.0390])
        w = v[3]

        # INERTIAL MATRIX
        M11 = x[0]
        M12 = 0
        M13 = 0
        M14 = a * w * x[1]
        M21 = 0
        M22 = x[2]
        M23 = 0
        M24 = b * w * x[3]
        M31 = 0
        M32 = 0
        M33 = x[4]
        M34 = 0
        M41 = a * w * x[5]
        M42 = b * w * x[6]
        M43 = 0
        M44 = x[7]

        M = np.array([[M11, M12, M13, M14],
                    [M21, M22, M23, M24],
                    [M31, M32, M33, M34],
                    [M41, M42, M43, M44]])

        # CENTRIOLIS MATRIX
        C11 = x[8]
        C12 = 0
        C13 = 0
        C14 = a * w * x[9]
        C21 = 0
        C22 = x[10]
        C23 = 0
        C24 = b * w * x[11]
        C31 = 0
        C32 = 0
        C33 = x[12]
        C34 = 0
        C41 = b * (w ** 2) * x[13]
        C42 = a * (w ** 2) * x[14]
        C43 = 0
        C44 = (a ** 2) * (w ** 2) * x[15] + (b ** 2) * (w ** 2) * x[16] + x[17]

        C = np.array([[C11, C12, C13, C14],
                    [C21, C22, C23, C24],
                    [C31, C32, C33, C34],
                    [C41, C42, C43, C44]])

        # GRAVITATIONAL MATRIX
        G11 = 0
        G21 = 0
        G31 = 0
        G41 = 0

        G = np.array([[G11],
                    [G21],
                    [G31],
                    [G41]])

        ve = vc - v
        control = vcp + K3 @ np.tanh(np.linalg.inv(K3) @ K4 @ ve)
        
        vref = np.squeeze(M @ control + C @ vc + np.ravel(G))
        sendvalues(pub_control, vref )

        rate.sleep() 
        toc = time.time() - tic 

        vc_anterior = vc.copy()
        #print("FPS: {:.2f}".format(1/toc), end='\r')
    
    
    

if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("Acados_controller",disable_signals=True, anonymous=True)

        pub = rospy.Publisher("/m100/velocityControl", Twist, queue_size=10)
        
    
        sub = rospy.Subscriber("/dji_sdk/odometry", Odometry, odo_callback, queue_size=10)
        RC_sub = rospy.Subscriber("/dji_sdk/rc", Joy, rc_callback, queue_size=10)
        Joystick_sub = rospy.Subscriber("/joy", Joy, joystick_callback, queue_size=10)
        vision_sub = rospy.Subscriber("/dji_sdk/visual_servoing/vel/drone", Twist, visual_callback, queue_size=10)

        main(pub)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Interrupción del usuario. Saliendo del bucle...")
        pass
    else:
        print("Complete Execution")
        pass
