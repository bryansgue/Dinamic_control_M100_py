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

# Global variables Odometry Drone
x_real = 0.0
y_real = 0.0
z_real = 0.0
vx_real = 0.0
vy_real = 0.0
vz_real = 0.0

# Angular velocities
qx_real = 0.0005
qy_real = 0.0
qz_real = 0.0
qw_real = 1.0
wx_real = 0.0
wy_real = 0.0
wz_real = 0.0

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
    global x_real, y_real, z_real, qx_real, qy_real, qz_real, qw_real, vx_real, vy_real, vz_real, wx_real, wy_real, wz_real
    # Read desired linear velocities from node
    x_real = odom_msg.pose.pose.position.x 
    y_real = odom_msg.pose.pose.position.y
    z_real = odom_msg.pose.pose.position.z
    vx_real = odom_msg.twist.twist.linear.x
    vy_real = odom_msg.twist.twist.linear.y
    vz_real = odom_msg.twist.twist.linear.z


    qx_real = odom_msg.pose.pose.orientation.x
    qy_real = odom_msg.pose.pose.orientation.y
    qz_real = odom_msg.pose.pose.orientation.z
    qw_real = odom_msg.pose.pose.orientation.w

    wx_real = odom_msg.twist.twist.angular.x
    wy_real = odom_msg.twist.twist.angular.y
    wz_real = odom_msg.twist.twist.angular.z
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
    

def get_odometry_simple():

    global x_real, y_real, z_real, qx_real, qy_real, qz_real, qw_real, vx_real, vy_real, vz_real, wx_real, wy_real, wz_real

    quaternion = [qx_real, qy_real, qz_real, qw_real ]
    r_quat = R.from_quat(quaternion)
    euler =  r_quat.as_euler('zyx', degrees = False)
    psi = euler[0]

    J = np.zeros((3, 3))
    J[0, 0] = np.cos(psi)
    J[0, 1] = -np.sin(psi)
    J[0, 2] = 0
    J[1, 0] = np.sin(psi)
    J[1, 1] = np.cos(psi)
    J[1, 2] = 0
    J[2, 0] = 0
    J[2, 1] = 0
    J[2, 2] = 1

    J_inv = np.linalg.inv(J)
    v = np.dot(J_inv, [vx_real, vy_real, vz_real])
 
    ul_real = v[0]
    um_real = v[1]
    un_real = v[2]


    x_state = [x_real,y_real,z_real,psi,ul_real,um_real,un_real, wz_real]

    return x_state

def calc_M(chi, a, b):
    

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
    
    return M

def calc_C(chi, a, b, x):
    w = x[7]

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

    return C

def calc_G():
    # GRAVITATIONAL MATRIchi
    G11 = 0
    G21 = 0
    G31 = 0
    G41 = 0

    G = np.array([[G11],
                [G21],
                [G31],
                [G41]])


def calc_J(x):
    
    psi = x[3]

    J = np.zeros((4, 4))

    J[0, 0] = np.cos(psi)
    J[0, 1] = -np.sin(psi)
    J[1, 0] = np.sin(psi)
    J[1, 1] = np.cos(psi)
    J[2, 2] = 1
    J[3, 3] = 1

    return J

def main(pub_control,pub_config):

    print("OK, controller is running!!!")
    
    timerun = 60*5  # Segundos
    hz = 30  # Frecuencia de actualización
    ts = 1 / hz
    samples = timerun * hz  # datos de muestreo totales
    
    # Inicialización de matrices
    t = np.arange(0, samples * ts, ts)
    x = np.zeros((8, samples))
    uc = np.zeros((4, samples))
    uref = np.zeros((4, samples))
    he = np.zeros((4, samples))
    ue = np.zeros((4, samples))
    psidp = np.zeros(samples)

    #GANANCIAS DEL CONTROLADOR
    K1 = np.diag([1,1,1,1])  
    K2 = np.diag([1,1,1,1])  
    K3 = np.diag([1,1,1,1])  
    K4 = np.diag([1,1,1,1])  
    
    #TAREA DESEADA
    num = 4
    xd = lambda t: 5 * np.sin(num *0.04 * t) + 0.1
    yd = lambda t: 2 * np.sin(num *0.08 * t) + 0.1
    zd = lambda t: 1 * np.sin(0.08 * t) + 5
    xdp = lambda t: 5 *num * 0.04 * np.cos(num *0.04 * t)
    ydp = lambda t: 2 *num * 0.08 * np.cos(num *0.08 * t)
    zdp = lambda t: 0.08 * np.cos(0.08 * t)

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
    xref = np.zeros((12, t.shape[0]), dtype = np.double)
    xref[0,:] = hxd
    xref[1,:] = hyd
    xref[2,:] = hzd
    xref[3,:] = psid
    xref[4,:] = hxdp
    xref[5,:] = hydp
    xref[6,:] = hzdp
    xref[7,:] = psidp

    # Simulation System
    ros_rate = 30  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate

    #INICIALIZA LECTURA DE ODOMETRIA
    for k in range(0, 100):
        # Read Real data
        x[:, 0] = get_odometry_simple()
        # Loop_rate.sleep()
        rate.sleep() 
        print("Init System")
    
    for k in range(samples):
        #INICIO DEL TIEMPO DE BUCLE
        tic = time.time()

        # MODELO CINEMATICO Y DINAMICO
        chi = [0.6756,    1.0000,    0.6344,    1.0000,    0.4080,    1.0000,    1.0000,    1.0000,    0.2953,    0.5941,   -0.8109,    1.0000,    0.3984,    0.7040,    1.0000,    0.9365,    1.0000, 1.0000,    0.9752]# Position
        
        J = calc_J(x[:, k])
        M = calc_M(chi, a, b)
        C = calc_C(chi, a, b, x[:,k])
        G = calc_G()
      
        # CONTROLADOR CINEMATICO
        he[:, k] = xref[0:4, k] - x[0:4, k]
        he[3, k] =  limitar_angulo(he[3, k])
        #uc[:, k] = np.linalg.pinv(J) @ (xref[4:8, k] + K1 @ np.tanh(K2 @ he[:, k]))
        uc[:, k] = np.linalg.pinv(J) @ (K1 @ np.tanh(K2 @ he[:, k]))
  
        if k > 0:
            vcp = (uc[:, k] - uc[:, k - 1]) / ts
        else:
            vcp = uc[:, k] / ts
     
        #COMPENSADOR DINAMICO
        ue[:, k] = uc[:, k] - x[4:8, k]
        control = 0*vcp + K3 @ np.tanh(np.linalg.inv(K3) @ K4 @ ue[:, k])
        uref[:, k] = M @ control + C @ uc[:, k]

        #ENVIO DE VELOCIDADES AL DRON
        sendvalues(pub_control, uc[:, k] )

        #LECTURA DE ODOMETRIA MODELO SIMPLIFICADO
        x[:, k+1] = get_odometry_simple()

        rate.sleep() 
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

        velocity_publisher = rospy.Publisher("/m100/velocityControl", TwistStamped, queue_size=10)
        pub_config = rospy.Publisher("/UAV/Config", Int32MultiArray, queue_size=10)
    
        sub = rospy.Subscriber("/dji_sdk/odometry", Odometry, odometry_call_back, queue_size=10)

        main(velocity_publisher,pub_config)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        
        sendvalues(velocity_publisher, [0, 0, 0, 0], )
        pass
    else:
        print("Complete Execution")
        pass
