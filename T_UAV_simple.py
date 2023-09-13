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



#pkg> status ## Es para check los paquetes instalados
# No es necesario en Python, ya que los paquetes se importan directamente

maxloops = 1000
rosloops = 0

## Global variables system
xd = 3.0
yd = -4.6
zd = 5.16
vxd = 0.0
vyd = 0.0
vzd = 0.0

qx = 0.0005
qy = 0.0
qz = 0.0
qw = 1.0
wxd = 0.0
wyd = 0.0
wzd = 0.0

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

def odometry_call_back(odom_msg):
    global xd, yd, zd, qx, qy, qz, qw, vxd, vyd, vzd, wxd, wyd, wzd, time_message

    # Read desired linear velocities from node
    time_message = odom_msg.header.stamp
    xd = odom_msg.pose.pose.position.x 
    yd = odom_msg.pose.pose.position.y
    zd = odom_msg.pose.pose.position.z
    vxd = odom_msg.twist.twist.linear.x
    vyd = odom_msg.twist.twist.linear.y
    vzd = odom_msg.twist.twist.linear.z


    qx = odom_msg.pose.pose.orientation.x
    qy = odom_msg.pose.pose.orientation.y
    qz = odom_msg.pose.pose.orientation.z
    qw = odom_msg.pose.pose.orientation.w

    wxd = odom_msg.twist.twist.angular.x
    wyd = odom_msg.twist.twist.angular.y
    wzd = odom_msg.twist.twist.angular.z
    return None

def get_body_vel():
    quat = np.array([qx, qy, qz, qw], dtype=np.double)
    rot = R.from_quat(quat)
    rot = rot.as_matrix()
    rot_inv = np.linalg.inv(rot)
    vel_w = np.array([vxd, vyd, vzd], dtype=np.double)
    #vel_w = vel_w.reshape(3,1)
    vel = rot_inv@vel_w 

    #u = np.array([vel[0,0], vel[1,0], vel[2,0]], dtype=np.double)
    return vel

def get_pos():
    h = np.array([xd, yd, zd], dtype=np.double)
    return h

def get_euler():
    quat = np.array([qx, qy, qz, qw], dtype=np.double)
    rot = R.from_quat(quat)
    eul = rot.as_euler('xyz', degrees=False)

    euler = np.array([eul[0], eul[1], eul[2]], dtype=np.double)
    return euler

def get_omega():
    omega = np.array([wxd, wyd, wzd], dtype=np.double)
    return omega

def get_euler_p(omega, euler):
    W = np.array([[1, np.sin(euler[0])*np.tan(euler[1]), np.cos(euler[0])*np.tan(euler[1])],
                  [0, np.cos(euler[0]), np.sin(euler[0])],
                  [0, np.sin(euler[0])/np.cos(euler[1]), np.cos(euler[0])/np.cos(euler[1])]])

    euler_p = np.dot(W, omega)
    return euler_p

def sensor1_callback(msg):
    sensor1 = [msg.x, msg.y, msg.z]

def sendvalues(pub_obj, vc):
    msg = TwistStamped()
    #vc = [float(valor) for valor in vc_no[:, 0]]
    msg.twist.linear.x = vc[0]
    msg.twist.linear.y = vc[1]
    msg.twist.linear.z = vc[2]
    msg.twist.angular.z = vc[3]
    pub_obj.publish(msg)

def limitar_angulo(ErrAng):

    if ErrAng >= 1 * np.pi:
        while ErrAng >= 1 * np.pi:
            ErrAng -= 2 * np.pi
        return ErrAng

    if ErrAng <= -1 * np.pi:
        while ErrAng <= -1 * np.pi:
            ErrAng += 2 * np.pi
        return ErrAng
    
    return ErrAng

def set_config(run, pub_config):
    msg = Int32MultiArray()
    msg.data = [run, run-run]
    pub_config.publish(msg)
    

def main(pub_control,pub_config):

    print("OK, controller is running!!!")
    set_config(1, pub_config)
    # Espera 1 segundo para comenzar la ejecución
    

    timerun = 60  # Segundos
    hz = 30  # Frecuencia de actualización
    ts = 1 / hz
    samples = timerun * hz  # datos de muestreo totales
    
    # Inicialización de matrices
    h = np.zeros((4, samples))
    uc = np.zeros((4, samples))
    uref = np.zeros((4, samples))
    u = np.zeros((4, samples))
    hd = np.zeros((4, samples))
    hdp = np.zeros((4, samples))
    he = np.zeros((4, samples))
    ue = np.zeros((4, samples))
    vcpp = np.zeros((4, samples))
    psidp = np.zeros(samples)

    J = np.zeros((4, 4))

    Gains = 0.9*np.array([1.5000 ,   1.0011,    1.5000,    1.5000,   1.5000,    1.0576,    1.5000,    1.5000,    1.5000,    1.0001,    1.5000,    1.4999,  1.5000,    1.0000 ,   1.5000  ,  1.4999])

    K1 = np.diag([1,1,1,1])  # Distribuir los primeros 4 elementos de Gains en la matriz K1
    K2 = np.diag([1,1,1,1])  # Distribuir los elementos 5 al 8 de Gains en la matriz K2
    K3 = np.diag([1,1,1,1])  # Distribuir los elementos 9 al 12 de Gains en la matriz K3
    K4 = np.diag([1,1,1,1])  # Distribuir los elementos 13 al 16 de Gains en la matriz K4
    t = np.arange(0, samples * ts, ts)
    
    #xd = lambda t: np.full_like(t, -12.6)
    #yd = lambda t: np.full_like(t, 22.2222222222)
    #zd = lambda t: np.full_like(t, 10)
    
    #xd = lambda t: 5 * np.cos(0.95*t) + 5
    #yd = lambda t: 5 * np.sin (0.95 * t)
    #zd = lambda t: 0.01 * np.sin (0.3 * t) +10  

     
    #xdp = lambda t: -5 * 0.95 * np.sin(0.95 * t)
    #ydp = lambda t: 5 * 0.95* np.cos(0.95* t)
    #zdp = lambda t: 0.01*0.3* np.cos(0.3 * t)
    num = 6
    xd = lambda t: 5 * np.sin(num *0.04 * t) + 0.1
    yd = lambda t: 5 * np.sin(num *0.08 * t) + 0.1
    zd = lambda t: 1 * np.sin(0.08 * t) + 2

    xdp = lambda t: 5 *num * 0.04 * np.cos(num *0.04 * t)
    ydp = lambda t: 5 *num * 0.08 * np.cos(num *0.08 * t)
    zdp = lambda t: 0.08 * np.cos(0.08 * t)

    xdpp = lambda t: -5 *num * 0.04 * 0.04 * num * np.sin(num *0.04 * t)
    ydpp = lambda t: -5 *num * 0.08 * 0.08 * num * np.sin(num *0.08 * t)

    hxd = xd(t)
    hyd = yd(t)
    hzd = zd(t)

    hxdp = xdp(t)
    hydp = ydp(t)
    hzdp = zdp(t)

    hxdpp = xdpp(t)
    hydpp = ydpp(t)

    psid = np.arctan2(hydp, hxdp)
    #psid_f = lambda t: np.full_like(t, 0)
    #psid = psid_f(t)

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

        pos = get_pos()
        euler = get_euler()

        vel_body = get_body_vel()
        omega = get_omega()
        euler_p = get_euler_p(omega,euler)


        h[:, k] = np.array([pos[0] , pos[1], pos[2], (euler[2]) ])
        u[:, k] = np.array([vel_body[0] ,vel_body[1], vel_body[2], euler_p[2] ])

        psi = h[3, k]
        J[0, 0] = np.cos(psi)
        J[0, 1] = -np.sin(psi)
        J[1, 0] = np.sin(psi)
        J[1, 1] = np.cos(psi)
        J[2, 2] = 1
        J[3, 3] = 1

        he[:, k] = hd[:, k] - h[:, k]
        he[3, k] =  limitar_angulo(he[3, k])
        uc[:, k] = np.linalg.pinv(J) @ (hdp[:, k] + K1 @ np.tanh(K2 @ he[:, k]))
        #vc[:, k] = np.linalg.pinv(J) @ (K1 @ np.tanh(K2 @ he[:, k]))

        if k > 0:
            vcp = (uc[:, k] - uc[:, k - 1]) / ts
        else:
            vcp = uc[:, k] / ts

        chi = [0.6756,    1.0000,    0.6344,    1.0000,    0.4080,    1.0000,    1.0000,    1.0000,    0.2953,    0.5941,   -0.8109,    1.0000,    0.3984,    0.7040,    1.0000,    0.9365,    1.0000, 1.0000,    0.9752]# Position
        w = u[3, k]

        # INERTIAL MATRIchi
        M11 = chi[0]
        M12 = 0
        M13 = 0
        M14 = b * chi[1]
        M21 = 0
        M22 = chi[2]
        M23 = 0
        M24 = a* chi[3]
        M31 = 0
        M32 = 0
        M33 = chi[4]
        M34 = 0
        M41 = b*chi[5]
        M42 = a* chi[6]
        M43 = 0
        M44 = chi[7]*(a**2+b**2) + chi[8]

        M = np.array([[M11, M12, M13, M14],
                    [M21, M22, M23, M24],
                    [M31, M32, M33, M34],
                    [M41, M42, M43, M44]])

        # CENTRIOLIS MATRIchi
        C11 = chi[9]
        C12 = w*chi[10]
        C13 = 0
        C14 = a * w * chi[11]
        C21 = w*chi[12]
        C22 = chi[13]
        C23 = 0
        C24 = b * w * chi[14]
        C31 = 0
        C32 = 0
        C33 = chi[15]
        C34 = 0
        C41 = a *w* chi[16]
        C42 = b * w * chi[17]
        C43 = 0
        C44 = chi[18]

        C = np.array([[C11, C12, C13, C14],
                    [C21, C22, C23, C24],
                    [C31, C32, C33, C34],
                    [C41, C42, C43, C44]])

        # GRAVITATIONAL MATRIchi
        G11 = 0
        G21 = 0
        G31 = 0
        G41 = 0

        G = np.array([[G11],
                    [G21],
                    [G31],
                    [G41]])

    
        ue[:, k] = uc[:, k] - u[:, k]
        control = 0*vcp + K3 @ np.tanh(np.linalg.inv(K3) @ K4 @ ue[:, k])
        #control = control.reshape(4, 1)
        uref[:, k] = np.squeeze(M @ control + C @ uc[:, k] + np.ravel(G))

        
        sendvalues(pub_control, uref[:, k] )
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
    plt.savefig("error_trayectoria.pdf")
    #plt.show()




if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("Acados_controller",disable_signals=True, anonymous=True)

        pub = rospy.Publisher("/m100/velocityControl", TwistStamped, queue_size=10)
        pub_config = rospy.Publisher("/UAV/Config", Int32MultiArray, queue_size=10)
    
        sub = rospy.Subscriber("/dji_sdk/odometry", Odometry, odometry_call_back, queue_size=10)

        main(pub,pub_config)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass
