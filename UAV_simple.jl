using RobotOS
using LinearAlgebra
using Plots


@rosimport geometry_msgs.msg: Point, Pose2D, Twist, Pose
@rosimport nav_msgs.msg: Odometry
@rosimport std_msgs.msg: Int32MultiArray

rostypegen()
using .geometry_msgs.msg
using .nav_msgs.msg
using .std_msgs.msg

function M_function(chi,a,b) 
    # INERCIAL MATRIchi
    M11=chi[1];
    M12=0;
    M13=0;
    M14=chi[2];
    M21=0;
    M22=chi[3] ;
    M23=0;
    M24=chi[4];
    M31=0;
    M32=0;
    M33=chi[5];
    M34=0;
    M41=b*chi[6];
    M42=a*chi[7];
    M43=0;
    M44=chi[8]*(a^2 + b^2)+chi[9];
    M=[M11 M12 M13 M14; M21 M22 M23 M24; M31 M32 M33 M34; M41 M42 M43 M44];
    return M
end

function C_function(chi,a,b,v,k) 
    w = v[4]
    C11=chi[10];
    C12=w*chi[11];
    C13=0;
    C14=a*w*chi[12];
    C21=w*chi[13];
    C22=chi[14];
    C23=0;
    C24=b*w*chi[15];
    C31=0;
    C32=0;
    C33=chi[16];
    C34=0;
    C41=a*w*chi[17];
    C42=b*w*chi[18];
    C43=0;
    C44=chi[19];
    C=[C11 C12 C13 C14; C21 C22 C23 C24; C31 C32 C33 C34; C41 C42 C43 C44];
    return C
end

function G_function()    
    # GRAVITATIONAL MATRIchi
    G11=0;
    G21=0;
    G31=0;
    G41=0;
    G=[G11;G21;G31;G41];
    return G
end

function dinamic_system(M,C,G,vref,v_actual,ts)    
    # GRAVITATIONAL MATRIchi
    vp = inv(M)*(vref-C*v_actual-G);
    return vp
end

function Load_chi()    
    chi = [1.4189 ;0.0213;1.2633;-0.2427;1.9647;0.1000;0.1000;0.1000;1.9293;0.7411;-0.0313;0.1000;0.0289;0.8760;0.1000;0.8701;0.1000;0.1000;0.9200]
    return chi
end

function vc_callback(pose::Twist) 
    global v_ref =[pose.linear.x, pose.linear.y, pose.linear.z, pose.angular.z]
end

function euler_to_quaternion(roll, pitch, yaw)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qw, qx, qy, qz]
end

function set_odo(pub_odo,h,hp,frame_id)  
    msg = Odometry()
    
    quat = euler_to_quaternion(0, 0, h[4])

    msg.header.frame_id = frame_id
    msg.pose.pose.position.x = h[1]
    msg.pose.pose.position.y = h[2]
    msg.pose.pose.position.z = h[3]
    msg.pose.pose.orientation.w = quat[1]
    msg.pose.pose.orientation.x = quat[2]
    msg.pose.pose.orientation.y = quat[3]
    msg.pose.pose.orientation.z = quat[4]
    msg.twist.twist.linear.x = hp[1]
    msg.twist.twist.linear.y = hp[2]
    msg.twist.twist.linear.z = hp[3]
    msg.twist.twist.angular.x = 0
    msg.twist.twist.angular.y = 0
    msg.twist.twist.angular.z = hp[4]

    publish(pub_odo, msg) 
end

function main()

    # Espera arranque
    println("Run the controller, please!")
    
    t = 1200;
    hz = 30 # Frecuencia de actualizacion
    samples = t*hz; # datos de muestreo totales
    loop_rate = Rate(hz); # Tiempo de muestre espera
 

    # Inicializacion de matrices
    h = zeros(4, samples+1)
    hp = zeros(4, samples)
    v = zeros(4, samples+1)
    vref = zeros(4, samples+1)
        
    # Estados iniciales del robot
    h[:,1] = [0 0 1.5 0.000000000000001]
    hp[:,1] = [0 0 0 0]

    println("OK, controller is runnig!!!")

    #Begins the controller
    for k in 1:(t*hz) 
        
        tic = time()
        ts = (1/hz);
        a=0;
        b=0;
        
        chi = Load_chi();

        M= M_function(chi,a,b) 
        C= C_function(chi,a,b,v[:,k],k) 
        G= G_function() 

        try
            vref[:,k] = v_ref
        catch
            vref[:,k] = [0, 0.0, 0.0, 0]  
        end 

        #vp = inv(M)*(vref[:,k]-C*v[:,k]-G);


        vp = dinamic_system(M,C,G,vref[:,k],v[:,k],ts);
        v[:,k+1] = v[:,k] +vp*ts;

        # Cinematica del robot
        psi = h[4,k]
        rot = [cos(psi) -sin(psi) 0 0; sin(psi) cos(psi) 0 0; 0 0 1 0; 0 0 0 1]
        hp[:,k] = rot*v[:,k] # CAmbio aqui      

        # Integracion numerica
        h[:,k+1] = hp[:,k]*(1/hz) + h[:,k] 
        
        # Enviar valores de posicion y velocidad (Odometria)
        set_odo(pub_pos, h[:,k+1], v[:,k+1],"input")
        
        println(h[:,k])
        rossleep(loop_rate) 
        toc = time()
        dt = toc-tic
    end    
end

if ! isinteractive()
    init_node("Robot")  
    sub_vc  = Subscriber{Twist}("/m100/velocityControl", vc_callback,  queue_size=10)
    #sub_run = Subscriber{Int32MultiArray}("/UAV/Config", config_callback,  queue_size=10)
    
    # Lanza los publisher del sistema
    pub_config = Publisher{Int32MultiArray}("/UAV/Config", queue_size=10)
    pub_pos = Publisher{Odometry}("/dji_sdk/odometry", queue_size=10)
    pub = Publisher{Twist}("/m100/velocityControl", queue_size=10)
    main()
end