from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from acados_template import AcadosModel
import scipy.linalg
import numpy as np
import time
import matplotlib.pyplot as plt
from casadi import Function
from casadi import MX
from casadi import reshape
from casadi import vertcat
from casadi import horzcat
from casadi import cos
from casadi import sin
from casadi import solve
from casadi import inv
from casadi import mtimes

import rospy
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
#from c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython
from geometry_msgs.msg import TwistStamped
import math

# CARGA FUNCIONES DEL PROGRAMA
from fancy_plots import plot_pose, plot_error, plot_time
from Functions_SimpleModel import f_system_simple_model_external
from Functions_SimpleModel import f_d, odometry_call_back, get_odometry_simple, send_velocity_control, pub_odometry_sim

u_referencia = [0,0,0,0]

def u_ref_callback(msg):

    global u_referencia

    ul_ref = msg.twist.linear.x
    um_ref = msg.twist.linear.y
    un_ref = msg.twist.linear.z
    r_ref = msg.twist.angular.z

    u_referencia = [ul_ref, um_ref, un_ref, r_ref]

def get_u_ref():
    
    u_ref = np.array(u_referencia , dtype=np.double)
    return u_ref

def main(vel_pub, vel_msg, odom_sim_pub, odom_sim_msg):
    # Initial Values System
    # Simulation Time
    t_final = 60*2
    # Sample time
    frec= 30
    t_s = 1/frec
    
    # Time simulation
    t = np.arange(0, t_final + t_s, t_s)

    # Sample time vector
    delta_t = np.zeros((1, t.shape[0] ), dtype=np.double)
 

    # Vector Initial conditions
    x = np.zeros((8, t.shape[0]+1), dtype = np.double)


    # Read Real data
    x[:, 0] = [0,0,1,0,0,0,0,0]

    # Initial Control values
    u_control = np.zeros((4, t.shape[0]), dtype = np.double)
    #u_control = np.zeros((4, t.shape[0]), dtype = np.double)

    # Simulation System
    ros_rate = 30  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate

    # Create Optimal problem
    model, f = f_system_simple_model_external()

    for k in range(0, t.shape[0]):
        tic = time.time()
        
        u_control[:, k] = get_u_ref()

        x[:, k+1] = f_d(x[:, k], u_control[:, k], t_s, f)
        pub_odometry_sim(x[:, k+1], odom_sim_pub, odom_sim_msg)
        
        
        print("x:", " ".join("{:.2f}".format(value) for value in np.round(x[0:12, k], decimals=2)))
        
        rate.sleep() 
        toc = time.time() - tic 
        
        
    send_velocity_control([0, 0, 0, 0], vel_pub, vel_msg)

 

    print(f'Mean iteration time with MLP Model: {1000*np.mean(delta_t):.1f}ms -- {1/np.mean(delta_t):.0f}Hz)')



if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("Acados_controller",disable_signals=True, anonymous=True)

        # SUCRIBER
        velocity_subscriber = rospy.Subscriber("/dji_sdk/odometry", Odometry, odometry_call_back)
        u_ref_sub =  rospy.Subscriber("/m100/velocityControl", TwistStamped, u_ref_callback, queue_size=10)
        
        # PUBLISHER
        velocity_message = TwistStamped()
        velocity_publisher = rospy.Publisher("/m100/velocityControl", TwistStamped, queue_size=10)

        odometry_sim_msg = Odometry()
        odom_sim_pub = rospy.Publisher('/dji_sdk/odometry', Odometry, queue_size=10)
    
        
    

        main(velocity_publisher, velocity_message, odom_sim_pub, odometry_sim_msg)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("\nError System")
        send_velocity_control([0, 0, 0, 0], velocity_publisher, velocity_message)
        pass
    else:
        print("Complete Execution")
        pass
