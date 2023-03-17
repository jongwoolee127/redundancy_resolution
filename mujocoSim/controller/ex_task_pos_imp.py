import sys
import numpy as np
import mujoco
import mujoco_viewer

sys.path += [ "../modules" ]

from utils  import min_jerk_traj

# Call the xml model file + data for MuJoCo
dir_name   = '../robot_model/'
robot_name = 'double_pendulum.xml'
model = mujoco.MjModel.from_xml_path( dir_name + robot_name )
data  = mujoco.MjData( model )

# Create the viewer object
viewer = mujoco_viewer.MujocoViewer( model, data, hide_menus = True )

# Set numpy print options
np.set_printoptions( precision = 4, threshold = 9, suppress = True )

# The number of degrees of freedom (nq) and active joints (n_act) of the robot 
nq    = model.nq
n_act = model.nu

# Parameters for the simulation
T        = 8.                       # Total Simulation Time
dt       = model.opt.timestep       # Time-step for the simulation (set in xml file)
fps      = 30                       # Frames per second
n_frames = 0                        # The current frame of the simulation
speed    = 1.0                      # The speed of the simulator
t_update = 1./fps * speed           # Time for update 

# The time-step defined in the xml file should be smaller than update
assert( dt <= t_update )

# Set the initial condition of the robot
data.qpos[ 0:nq ] = np.array( [-1.0, 1.0 ] )
mujoco.mj_forward( model, data )

# Save the references for the joint position q and joint velocity dq 
q  = data.qpos[ 0:nq ]
dq = data.qvel[ 0:nq ]

# Save the references for the end-effector position p, Jacobian, which is needed for computing 
idx_EE = mujoco.mj_name2id( model, mujoco.mjtObj.mjOBJ_GEOM, "geom_EE" )
p = data.geom_xpos[ idx_EE, : ]

# The task-space impedances of the robot, position control
# Since the robot moves in the xz plane, set y as 0 
Kp = np.diag( [ 400, 0, 400 ] )
Bp = 0.5 * Kp

# The parameters of the minimum-jerk trajectory.
t0 = 1.0
D  = 2.0
p0i = np.copy( p )
p0f = p + np.array( [ 2.0, 0.0, 0.0 ] )     # Again, y is zero

# Empty Jacobian Matrix needed for the computation 
jacp_EE = np.zeros( ( 3, n_act ) )
jacr_EE = np.zeros( ( 3, n_act ) )      # Not needed for this script.

# Initialize the reference end-effector position
p0  = np.zeros( 3 )
dp0 = np.zeros( 3 )


while data.time <= T:

    mujoco.mj_step( model, data )

    # First-order Task-space Impedance Controller
    # Calculate the minimum-jerk trajectory 
    for i in range( 3 ):
        p0[ i ], dp0[ i ], _ = min_jerk_traj( data.time, t0, t0 + D, p0i[ i ], p0f[ i ], D )

    # Get the Jacobian of the end-effector
    mujoco.mj_jacGeom( model, data, jacp_EE, jacr_EE, idx_EE )

    # Get the Coriolis + Gravity of the end-effector
    # Not needed for this script, just for demonstration
    sum_forces = np.copy( data.qfrc_bias[ : ] )

    # Get the Mass Matrix of the end-effector
    # Not needed for this script, just for demonstration
    Mtmp = np.zeros( ( nq, nq ) )
    mujoco.mj_fullM( model, Mtmp, data.qM )

    # Calculate the end-effector veloicty, dq = J(q)dq
    dp = jacp_EE @ dq

    # First-order Task-space impedance controller, position control only.
    tau_imp = jacp_EE.T @ ( Kp @ ( p0 - p ) + Bp @ ( dp0 - dp ) )

    # Torque input
    data.ctrl[ :n_act ] = tau_imp

    # Update Visualization
    if( n_frames != ( data.time // t_update ) ):
        n_frames += 1
        viewer.render( )
        print( "[Time] %6.3f" % data.time )

# close
viewer.close()