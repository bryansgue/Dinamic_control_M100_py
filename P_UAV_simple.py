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

pos = [0,0,0.1,0]
vel = [0,0,0,0]

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
    
    return None

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



def set_config(run, pub_config):
    msg = Int32MultiArray()
    msg.data = [run, run-run]
    pub_config.publish(msg)
    

def main(pub_control,pub_config):

    print("OK, controller is running!!!")
    set_config(1, pub_config)
    # Espera 1 segundo para comenzar la ejecución
    

    timerun = 30  # Segundos
    hz = 30  # Frecuencia de actualización
    ts = 1 / hz
    samples = timerun * hz  # datos de muestreo totales
    
    # Inicialización de matrices
    h = np.zeros((4, samples))
    vc = np.zeros((4, samples))
    vref = np.zeros((4, samples))
    v = np.zeros((4, samples))
    hd = np.zeros((4, samples))
    hdp = np.zeros((4, samples))
    he = np.zeros((4, samples))
    ve = np.zeros((4, samples))
    vcpp = np.zeros((4, samples))
    psidp = np.zeros(samples)

    J = np.zeros((4, 4))

    Gains = np.array([1.5000 ,   1.0011,    1.5000,    1.5000,   1.5000,    1.0576,    1.5000,    1.5000,    1.5000,    1.0001,    1.5000,    1.4999,  1.5000,    1.0000 ,   1.5000  ,  1.4999])

    K1 = np.diag([Gains[0], Gains[1], Gains[2], Gains[3]])  # Distribuir los primeros 4 elementos de Gains en la matriz K1
    K2 = np.diag([Gains[4], Gains[5], Gains[6], Gains[7]])  # Distribuir los elementos 5 al 8 de Gains en la matriz K2
    K3 = np.diag([Gains[8], Gains[9], Gains[10], Gains[11]])  # Distribuir los elementos 9 al 12 de Gains en la matriz K3
    K4 = np.diag([Gains[12], Gains[13], Gains[14], Gains[15]])  # Distribuir los elementos 13 al 16 de Gains en la matriz K4

    t = np.arange(0, samples * ts, ts)
    
    #xd = lambda t: 5 * np.sin(0.04 * t) + 0.1
    #yd = lambda t: 5 * np.sin(0.08 * t) + 0.1
    #zd = lambda t: 1 * np.sin(0.08 * t) + 2

    xd = lambda t: np.full_like(t, -0.188)
    yd = lambda t: np.full_like(t, -0.1254)
    zd = lambda t: np.full_like(t, 3)

    xdp = lambda t: 5 * 0.04 * np.cos(0.04 * t)
    ydp = lambda t: 5 * 0.08 * np.cos(0.08 * t)
    zdp = lambda t: 0.08 * np.cos(0.08 * t)

    xdpp = lambda t: -5 * 0.04 * 0.04 * np.sin(0.04 * t)
    ydpp = lambda t: -5 * 0.08 * 0.08 * np.sin(0.08 * t)

    hxd = xd(t)
    hyd = yd(t)
    hzd = zd(t)

    hxdp = xdp(t)
    hydp = ydp(t)
    hzdp = zdp(t)

    hxdpp = xdpp(t)
    hydpp = ydpp(t)

    #psid = np.arctan2(hydp, hxdp)
    psid_f = lambda t: np.full_like(t, 0)
    psid = psid_f(t)

    for k in range(samples):
        if k > 0:
            psidp[k] = (psid[k] - psid[k - 1]) / ts
        else:
            psidp[k] = psid[k] / ts

    psidp[0] = 0

    

    a = 0
    b = 0
    time_init = time.time()
    
    for k in range(samples):
        tic = time.time()

        hd[:, k] = [hxd[k], hyd[k], hzd[k], psid[k]]
        hdp[:, k] = [hxdp[k], hydp[k], hzdp[k], psidp[k]]

        h[:, k] = pos
        v[:, k] = vel

        psi = h[3, k]
        J[0, 0] = np.cos(psi)
        J[0, 1] = -np.sin(psi)
        J[1, 0] = np.sin(psi)
        J[1, 1] = np.cos(psi)
        J[2, 2] = 1
        J[3, 3] = 1

        he[:, k] = hd[:, k] - h[:, k]
        vc[:, k] = np.linalg.pinv(J) @ (hdp[:, k] + K1 @ np.tanh(K2 @ he[:, k]))
        #vc[:, k] = np.linalg.pinv(J) @ (K1 @ np.tanh(K2 @ he[:, k]))

        if k > 0:
            vcp = (vc[:, k] - vc[:, k - 1]) / ts
        else:
            vcp = vc[:, k] / ts

        x = np.array([ 0.3259, 0,0.3787, 0,0.4144, 0, 0, 0.2295, 0.7623, 0, 0.8279,0,0.8437, 0, 0, 0, 0,1.0390])
        w = v[3, k]

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


        ve[:, k] = vc[:, k] - v[:, k]
        control = vcp + K3 @ np.tanh(np.linalg.inv(K3) @ K4 @ ve[:, k])
        #control = control.reshape(4, 1)
        vref[:, k] = np.squeeze(M @ control + C @ vc[:, k] + np.ravel(G))

        
        sendvalues(pub_control, vref[:, k] )
        while (time.time() - tic <= ts):
                None
        toc = time.time() - tic 
        print("Error:", " ".join("{:.2f}".format(value) for value in np.round(he[:, k], decimals=2)), end='\r')

    v_end = [0, 0.0, 0.0, 0]
    sendvalues(pub_control, v_end)
    set_config(0, pub_config)

    x = np.arange(1, samples + 1)
    hxe = he[0, :samples]
    hye = he[1, :samples]
    hze = he[2, :samples]
    hpsie = he[3, :samples]

    plt.plot(x, hxe, label="hxe", lw=1)
    plt.plot(x, hye, label="hye", lw=1)
    plt.plot(x, hze, label="hze", lw=1)
    plt.plot(x, hpsie, label="hpsie", lw=1)

    plt.title("Error de control")
    plt.xlabel("Eje x")
    plt.ylabel("Error")
    plt.legend()
    plt.savefig("error_posicion.pdf")
    #plt.show()




if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("Acados_controller",disable_signals=True, anonymous=True)

        pub = rospy.Publisher("/m100/velocityControl", Twist, queue_size=10)
        pub_config = rospy.Publisher("/UAV/Config", Int32MultiArray, queue_size=10)
    
        sub = rospy.Subscriber("/dji_sdk/odometry", Odometry, odo_callback, queue_size=10)

        main(pub,pub_config)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass
