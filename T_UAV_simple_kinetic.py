#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose2D, Twist, Pose, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Joy
from scipy.io import savemat
import os


from Functions_SimpleModel import odometry_call_back, get_odometry_simple, send_velocity_control
from Functions_DinamicControl import calc_G, calc_C, calc_M, calc_J, limitar_angulo
from fancy_plots import plot_pose, plot_error, plot_time
import P_UAV_simple

def main(vel_pub, vel_msg ):

    print("OK, controller is running!!!")
    
    timerun = 60 # Segundos
    hz = 30  # Frecuencia de actualización
    ts = 1 / hz
    samples = timerun * hz  # datos de muestreo totales
    
    # Inicialización de matrices
    t = np.arange(0, samples * ts, ts)
    x = np.zeros((8, samples))
    uc = np.zeros((4, samples))
    u = np.zeros((4, samples))
    he = np.zeros((4, samples))
    ue = np.zeros((4, samples))
    psidp = np.zeros(samples)

    #GANANCIAS DEL CONTROLADOR
    K1 = np.diag([1.0167, 1.4292, 2.0432, 2.4993])
    K2 = np.diag([2.7479, 3.1665, 3.7262, 2.6893])


    # GANANCIAS DEL CONTROLADOR


    
    #K2 = np.diag([1,1,1,1])
    #K3 = np.diag([1,1,1,1])
   
    
    #TAREA DESEADA
    value = 8
    xd = lambda t: 4 * np.sin(value*0.04*t) + 3
    yd = lambda t: 4 * np.sin(value*0.08*t)
    zd = lambda t: 2 * np.sin(value*0.08*t) + 6
    xdp = lambda t: 4 * value * 0.04 * np.cos(value*0.04*t)
    ydp = lambda t: 4 * value * 0.08 * np.cos(value*0.08*t)
    zdp = lambda t: 2 * value * 0.08 * np.cos(value*0.08*t)

    hxd = xd(t)
    hyd = yd(t)
    hzd = zd(t)
    hxdp = xdp(t)
    hydp = ydp(t)
    hzdp = zdp(t)

    psid = np.arctan2(hydp, hxdp)
    psidp = np.gradient(psid, ts)

    # Vector Initial conditions
    a = 0
    b = 0
    
    # Reference Signal of the system
    ref = np.zeros((12, t.shape[0]), dtype = np.double)
    ref[0,:] = hxd
    ref[1,:] = hyd
    ref[2,:] = hzd
    ref[3,:] = 0*psid
    ref[4,:] = hxdp
    ref[5,:] = hydp
    ref[6,:] = hzdp
    ref[7,:] = 0*psidp

    # Simulation System
    ros_rate = 30  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate


    P_UAV_simple.main(vel_pub, vel_msg )

    #INICIALIZA LECTURA DE ODOMETRIA
    for k in range(0, 10):
        # Read Real data
        x[:, 0] = get_odometry_simple()
        # Loop_rate.sleep()
        rate.sleep() 
        print("Init System Tratyectory")
    
    for k in range(samples-1):
        #INICIO DEL TIEMPO DE BUCLE
        tic = time.time()

        # MODELO CINEMATICO Y DINAMICO
        chi = [0.6756,    1.0000,    0.6344,    1.0000,    0.4080,    1.0000,    1.0000,    1.0000,    0.2953,    0.5941,   -0.8109,    1.0000,    0.3984,    0.7040,    1.0000,    0.9365,    1.0000, 1.0000,    0.9752]# Position
        
        J = calc_J(x[:, k])
        M = calc_M(chi, a, b)
        C = calc_C(chi, a, b, x[:,k])
        G = calc_G()
      
        # CONTROLADOR CINEMATICO
        he[:, k] = ref[0:4, k] - x[0:4, k]
        he[3, k] =  limitar_angulo(he[3, k])
        uc[:, k] = np.linalg.pinv(J) @ (ref[4:8, k] + K2 @ np.tanh(np.linalg.inv(K2) @ K1 @ he[:, k]))
        #uc[:, k] = np.linalg.pinv(J) @ (K1 @ np.tanh(K2 @ he[:, k]))
          

        #ENVIO DE VELOCIDADES AL DRON
        send_velocity_control(uc[:, k], vel_pub, vel_msg )

        #LECTURA DE ODOMETRIA MODELO SIMPLIFICADO
        x[:, k+1] = get_odometry_simple()

        rate.sleep() 
        toc = time.time() - tic 
                
        print("Error:", " ".join("{:.2f}".format(value) for value in np.round(he[:, k], decimals=2)), end='\r')

    send_velocity_control([0, 0, 0, 0], velocity_publisher, velocity_message)
 
    fig1 = plot_pose(x, ref, t)
    fig1.savefig("1_pose.png")
    fig2 = plot_error(he, t)
    fig2.savefig("2_error_pose.png")

    #For MODEL TESTS
    # Ruta que deseas verificar
    pwd = "/home/bryansgue/Doctoral_Research/Matlab/Graficas_Metologia"

    # Verificar si la ruta no existe
    if not os.path.exists(pwd) or not os.path.isdir(pwd):
        print(f"La ruta {pwd} no existe. Estableciendo la ruta local como pwd.")
        pwd = os.getcwd()  # Establece la ruta local como pwd
    
    #SELECCION DEL EXPERIMENTO
    Test = "Real"

    if Test == "MiL":
        name_file = "Kinetic_MiL.mat"
    elif Test == "HiL":
        name_file = "Kinetic_HiL.mat"
    elif Test == "Real":
        name_file = "Kinetic_Real.mat"
    
    save = True
    if save==True:
        savemat(os.path.join(pwd, name_file), {
            'x_states': x,
            'ref': ref,
            'uc_input': uc,
            't_time': t })


if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("Acados_controller",disable_signals=True, anonymous=True)

        # SUCRIBER
        velocity_subscriber = rospy.Subscriber("/dji_sdk/odometry", Odometry, odometry_call_back)
        
        # PUBLISHER
        velocity_message = TwistStamped()
        velocity_publisher = rospy.Publisher("/m100/velocityControl", TwistStamped, queue_size=10)

        main(velocity_publisher, velocity_message)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        
        send_velocity_control([0, 0, 0, 0], velocity_publisher, velocity_message)
        pass
    else:
        print("Complete Execution")
        pass
