#!/usr/bin/env python3
import sys
import rospy
import actionlib
import math
import cvxpy as cp
import numpy as np
import geometry_msgs.msg as geometry_msgs
import PyKDL as kdl

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trac_ik_python.trac_ik import IK
from kdl_parser_py.urdf import treeFromParam

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

class TrajectoryClient:
    def __init__(self):
        rospy.init_node("cbf_trajectory")
        timeout = rospy.Duration(5)
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        self.joint_trajectory_controller = "pos_joint_traj_controller"
        self.current_joint_angles = [0.0] * len(JOINT_NAMES)
        self.ik_solver = IK("base_link", "tool0")
        self.cluster_centroid = None
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/centroid", Float32MultiArray, self.centroid_callback)

        success, tree = treeFromParam("/robot_description")
        if not success:
            rospy.logerr("Failed to load KDL tree from URDF")
            sys.exit(1)

        self.chain = tree.getChain("base_link", "tool0")
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain) 
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

    def joint_state_callback(self, msg):
        joint_indices = {name: i for i, name in enumerate(msg.name)}
        filtered_joint_names = [
            "elbow_joint",
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        for i, name in enumerate(filtered_joint_names):
            if name in joint_indices:
                self.current_joint_angles[i] = msg.position[joint_indices[name]]
                
    def centroid_callback(self, msg):
        data = np.array(msg.data)
        self.cluster_centroid = data

    def manipulator_cbf(self, p_obs, q, p, J, u_des, gamma):
        R = 0.15
        R_w = 0.2
        p_obs = np.array(p_obs)
        
        q = np.array(q)
        p = np.array(p)
        J = np.array(J)

        u_des = np.array(u_des)

        fx = np.zeros((6,1))
        gx = np.eye(6)

        dhdx = 2*(p - p_obs).transpose() @ J[0:3, :]
        hx = np.linalg.norm(p - p_obs)**2 - (R + R_w)**2
       
        print(f"h(x): {hx}")

        P = np.eye(6)

        u = cp.Variable(6)
        cost = cp.quad_form(u - u_des, P)
        constraint = [(dhdx @ fx).flatten() + dhdx @ gx @ u + (hx*gamma).flatten() >=0 ]
        prob = cp.Problem(cp.Minimize(cost), constraint)
        prob.solve()

        if prob.status == "optimal":
             return u.value, hx
        else:
             print("Status: ", prob.status)
             return None       
               
    def calculate_jacobian(self, joint_angles):
        kdl_angles = kdl.JntArray(len(joint_angles))
        for i, angle in enumerate(joint_angles):
            kdl_angles[i] = angle

        jacobian = kdl.Jacobian(len(joint_angles))
        self.jac_solver.JntToJac(kdl_angles, jacobian)
        jacobian_array = np.zeros((6, len(joint_angles)))
        for i in range(6):
            for j in range(len(joint_angles)):
                jacobian_array[i, j] = jacobian[i, j]
        
        return jacobian_array
    
    def swap_jacobian_blocks(self, jacobian_array):
        if jacobian_array.shape != (6, 6):
            raise ValueError("The Jacobian matrix must be a 6x6 matrix.")
        swapped_jacobian = jacobian_array.copy()
        swapped_jacobian[:3, :] = jacobian_array[3:, :]
        swapped_jacobian[3:, :] = jacobian_array[:3, :]
        return swapped_jacobian
    
    def get_cartesian_positions(self, joint_angles):
        cartesian_positions = []
        for angles in joint_angles:
            kdl_angles = kdl.JntArray(len(angles))
            for i, angle in enumerate(angles):
                kdl_angles[i] = angle
           
            end_effector_frame = kdl.Frame()
            self.fk_solver.JntToCart(kdl_angles, end_effector_frame)

            x, y, z = end_effector_frame.p[0], end_effector_frame.p[1], end_effector_frame.p[2]
            cartesian_positions.append([x, y, z])
        
        return np.array(cartesian_positions)
        
    def send_joint_trajectory(self):
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        num_points = 101
        time_duration = 4
        dt = time_duration / (num_points - 1)
        
        # r+rw = 0.35, obs = (0,1,0.15) line trajectory
        position_list = [[0.0, -1.57079633, 1.57079633, -1.57079633, -1.57079633, 0.0], [0.01130777, -1.58604596, 1.59137538, -1.57612575, -1.57079633, 0.01130777], [0.02295525, -1.60095539, 1.61134923, -1.58119017, -1.57079633, 0.02295525], [0.0349519, -1.61551939, 1.63073097, -1.58600791, -1.57079633, 0.0349519], [0.04730716, -1.62973143, 1.64953238, -1.59059728, -1.57079633, 0.04730716], [0.06003034, -1.64358372, 1.6677641, -1.59497671, -1.57079633, 0.06003034], [0.07313058, -1.65706731, 1.68543575, -1.59916476, -1.57079633, 0.07313058], [0.0866168, -1.67017214, 1.70255599, -1.60318018, -1.57079633, 0.0866168], [0.10049762, -1.68288707, 1.7191327, -1.60704195, -1.57079633, 0.10049762], [0.11478124, -1.6952, 1.73517298, -1.61076931, -1.57079633, 0.11478124], [0.12947542, -1.70709786, 1.75068328, -1.61438174, -1.57079633, 0.12947542], [0.14458734, -1.71856677, 1.76566943, -1.61789899, -1.57079633, 0.14458734], [0.16012352, -1.72959204, 1.78013672, -1.621341, -1.57079633, 0.16012352], [0.1760897, -1.74015831, 1.7940899, -1.62472792, -1.57079633, 0.1760897], [0.19249074, -1.75024961, 1.8075333, -1.62808002, -1.57079633, 0.19249074], [0.20933052, -1.75984947, 1.82047077, -1.63141763, -1.57079633, 0.20933052], [0.22661178, -1.76894104, 1.83290577, -1.63476105, -1.57079633, 0.22661178], [0.24433607, -1.77750719, 1.84484135, -1.63813049, -1.57079633, 0.24433607], [0.26250356, -1.78553064, 1.8562802, -1.64154589, -1.57079633, 0.26250356], [0.281113, -1.79299411, 1.86722464, -1.64502685, -1.57079633, 0.281113], [0.30016154, -1.79988043, 1.8776766, -1.64859249, -1.57079633, 0.30016154], [0.31964468, -1.80617272, 1.88763767, -1.65226128, -1.57079633, 0.31964468], [0.33955618, -1.81185451, 1.89710908, -1.6560509, -1.57079633, 0.33955618], [0.35988794, -1.81690992, 1.90609165, -1.65997806, -1.57079633, 0.35988794], [0.38063, -1.82132382, 1.91458587, -1.66405838, -1.57079633, 0.38063], [0.40177044, -1.82508196, 1.92259181, -1.66830617, -1.57079633, 0.40177044], [0.42329538, -1.82817117, 1.93010917, -1.67273433, -1.57079633, 0.42329538], [0.44518901, -1.83057946, 1.93713725, -1.67735411, -1.57079633, 0.44518901], [0.46743357, -1.83229621, 1.94367494, -1.68217506, -1.57079633, 0.46743357], [0.49000942, -1.83331226, 1.94972075, -1.68720481, -1.57079633, 0.49000942], [0.51289512, -1.8336201, 1.95527278, -1.692449, -1.57079633, 0.51289512], [0.53606752, -1.8332139, 1.96032874, -1.69791116, -1.57079633, 0.53606752], [0.55950191, -1.83208967, 1.96488598, -1.70359264, -1.57079633, 0.55950191], [0.58317216, -1.83024529, 1.96894148, -1.70949252, -1.57079633, 0.58317216], [0.60705088, -1.82768058, 1.97249187, -1.71560761, -1.57079633, 0.60705088], [0.63110962, -1.82439738, 1.97553349, -1.72193244, -1.57079633, 0.63110962], [0.65531911, -1.82039947, 1.97806241, -1.72845926, -1.57079633, 0.65531911], [0.67964942, -1.81569266, 1.98007443, -1.73517811, -1.57079633, 0.67964942], [0.70407022, -1.81028468, 1.9815652, -1.74207685, -1.57079633, 0.70407022], [0.72855102, -1.8041852, 1.98253019, -1.74914132, -1.57079633, 0.72855102], [0.75306138, -1.79740569, 1.98296478, -1.75635542, -1.57079633, 0.75306138], [0.77757112, -1.78995938, 1.9828643, -1.76370125, -1.57079633, 0.77757112], [0.80205056, -1.78186112, 1.98222408, -1.77115929, -1.57079633, 0.80205056], [0.82647072, -1.77312727, 1.98103951, -1.77870856, -1.57079633, 0.82647072], [0.85080347, -1.76377557, 1.97930607, -1.78632683, -1.57079633, 0.85080347], [0.87502173, -1.75382493, 1.9770194, -1.7939908, -1.57079633, 0.87502173], [0.89909962, -1.74329537, 1.97417533, -1.80167629, -1.57079633, 0.89909962], [0.92301257, -1.73220775, 1.97076993, -1.8093585, -1.57079633, 0.92301257], [0.94673739, -1.72058369, 1.96679951, -1.81701214, -1.57079633, 0.94673739], [0.97025245, -1.70844532, 1.96226069, -1.8246117, -1.57079633, 0.97025245], [0.99353761, -1.69581517, 1.95715042, -1.83213158, -1.57079633, 0.99353761], [1.01657438, -1.68271595, 1.95146596, -1.83954634, -1.57079633, 1.01657438], [1.03934585, -1.66917044, 1.9452049, -1.84683079, -1.57079633, 1.03934585], [1.06183675, -1.65520129, 1.93836519, -1.85396023, -1.57079633, 1.06183675], [1.08403338, -1.6408309, 1.9309451, -1.86091053, -1.57079633, 1.08403338], [1.10592365, -1.62608126, 1.9229432, -1.86765827, -1.57079633, 1.10592365], [1.12749697, -1.61097384, 1.91435837, -1.87418086, -1.57079633, 1.12749697], [1.14874425, -1.59552946, 1.90518977, -1.88045664, -1.57079633, 1.14874425], [1.16965779, -1.57976822, 1.89543678, -1.88646489, -1.57079633, 1.16965779], [1.18940452, -1.56434407, 1.88550261, -1.89195486, -1.57079633, 1.18940452], [1.20967949, -1.54799187, 1.8745871, -1.89739156, -1.57079633, 1.20967949], [1.22960322, -1.53137711, 1.8630851, -1.90250432, -1.57079633, 1.22960322], [1.249173, -1.51451623, 1.85099657, -1.90727666, -1.57079633, 1.249173], [1.2683872, -1.49742455, 1.83832145, -1.91169323, -1.57079633, 1.2683872], [1.28724514, -1.48011622, 1.82505961, -1.91573972, -1.57079633, 1.28724514], [1.30574702, -1.46260427, 1.81121081, -1.91940287, -1.57079633, 1.30574702], [1.32389384, -1.4449005, 1.79677456, -1.92267038, -1.57079633, 1.32389384], [1.34168734, -1.42701554, 1.78175013, -1.92553091, -1.57079633, 1.34168734], [1.35912992, -1.40895877, 1.76613639, -1.92797395, -1.57079633, 1.35912992], [1.37622457, -1.39073839, 1.74993184, -1.92998977, -1.57079633, 1.37622457], [1.39297478, -1.37236138, 1.73313441, -1.93156936, -1.57079633, 1.39297478], [1.40938453, -1.35383349, 1.71574146, -1.9327043, -1.57079633, 1.40938453], [1.4254582, -1.33515927, 1.69774966, -1.93338672, -1.57079633, 1.4254582], [1.44120049, -1.31634208, 1.6791549, -1.93360914, -1.57079633, 1.44120049], [1.45661643, -1.29738406, 1.65995215, -1.93336442, -1.57079633, 1.45661643], [1.47171131, -1.27828613, 1.64013543, -1.93264563, -1.57079633, 1.47171131], [1.48649061, -1.25904802, 1.61969761, -1.93144592, -1.57079633, 1.48649061], [1.50095999, -1.23966821, 1.59863031, -1.92975843, -1.57079633, 1.50095999], [1.51512526, -1.22014396, 1.57692379, -1.92757616, -1.57079633, 1.51512526], [1.52899233, -1.20047124, 1.55456672, -1.92489182, -1.57079633, 1.52899233], [1.54256717, -1.1806447, 1.53154607, -1.9216977, -1.57079633, 1.54256717], [1.55585583, -1.16065764, 1.50784685, -1.91798554, -1.57079633, 1.55585583], [1.56886437, -1.14050193, 1.48345192, -1.91374631, -1.57079633, 1.56886437], [1.58159887, -1.12016792, 1.4583417, -1.9089701, -1.57079633, 1.58159887], [1.5940654, -1.09964433, 1.43249386, -1.90364586, -1.57079633, 1.5940654], [1.60626998, -1.07891812, 1.40588298, -1.89776119, -1.57079633, 1.60626998], [1.61821861, -1.05797435, 1.37848012, -1.8913021, -1.57079633, 1.61821861], [1.62991724, -1.03679591, 1.35025227, -1.88425268, -1.57079633, 1.62991724], [1.64137174, -1.01536335, 1.32116182, -1.8765948, -1.57079633, 1.64137174], [1.65258792, -0.99365449, 1.29116579, -1.86830762, -1.57079633, 1.65258792], [1.6635715, -0.97164407, 1.26021492, -1.85936718, -1.57079633, 1.6635715], [1.67432812, -0.94930325, 1.22825264, -1.84974572, -1.57079633, 1.67432812], [1.68486331, -0.92659898, 1.19521366, -1.83941101, -1.57079633, 1.68486331], [1.69518252, -0.90349327, 1.16102229, -1.82832535, -1.57079633, 1.69518252], [1.70529109, -0.87994212, 1.12559028, -1.81644448, -1.57079633, 1.70529109], [1.71519427, -0.85589429, 1.08881399, -1.80371603, -1.57079633, 1.71519427], [1.72489718, -0.83128957, 1.0505708, -1.79007755, -1.57079633, 1.72489718], [1.73440485, -0.80605652, 1.01071416, -1.77545397, -1.57079633, 1.73440485], [1.74372221, -0.7801094, 0.96906708, -1.75975401, -1.57079633, 1.74372221], [1.75285408, -0.75334386, 0.9254129, -1.74286536, -1.57079633, 1.75285408], [1.76180518, -0.72563092, 0.87948226, -1.72464767, -1.57079633, 1.76180518]]
        """
        position_list = []

        for t in np.linspace(0, 1, num_points):
            intermediated_jnt_pose = [
                start_joint_positions[i] * (1 - t) + goal_joint_positions[i] * t
                for i in range(len(start_joint_positions))
            ]
            position_list.append(intermediated_jnt_pose)
        """
        cartesian_positions = self.get_cartesian_positions(position_list)
        
        print("Cartesian Positions")
        np.set_printoptions(precision=4, suppress=True)
        print(cartesian_positions.shape)
        
        print("Joint Positions")
        position_list_array = np.array(position_list)
        print(position_list_array.shape)
        
        velocity_list = [[0.28269424, -0.38124072, 0.51447636, -0.13323564, -0.0, 0.28269424], [0.29118689, -0.37273584, 0.49934633, -0.12661048, -0.0, 0.29118689], [0.29991641, -0.36410006, 0.48454342, -0.12044335, -0.0, 0.29991641], [0.30888148, -0.35530085, 0.47003527, -0.11473442, -0.0, 0.30888148], [0.31807942, -0.34630726, 0.45579306, -0.1094858, -0.0, 0.31807942], [0.32750607, -0.33708986, 0.44179103, -0.10470117, -0.0, 0.32750607], [0.33715559, -0.32762071, 0.42800612, -0.10038541, -0.0, 0.33715559], [0.34702031, -0.31787339, 0.41441766, -0.09654427, -0.0, 0.34702031], [0.35709058, -0.30782308, 0.40100705, -0.09318397, -0.0, 0.35709058], [0.36735456, -0.29744662, 0.3877575, -0.09031088, -0.0, 0.36735456], [0.37779807, -0.28672265, 0.37465381, -0.08793115, -0.0, 0.37779807], [0.38840447, -0.27563177, 0.36168211, -0.08605034, -0.0, 0.38840447], [0.39915447, -0.2641567, 0.34882972, -0.08467302, -0.0, 0.39915447], [0.41002606, -0.25228244, 0.3360849, -0.08380246, -0.0, 0.41002606], [0.42099436, -0.23999655, 0.32343673, -0.08344018, -0.0, 0.42099436], [0.43203162, -0.22728926, 0.31087489, -0.08358563, -0.0, 0.43203162], [0.44310715, -0.21415376, 0.29838958, -0.08423582, -0.0, 0.44310715], [0.45418736, -0.20058636, 0.28597132, -0.08538496, -0.0, 0.45418736], [0.46523584, -0.18658673, 0.27361087, -0.08702414, -0.0, 0.46523584], [0.47621346, -0.17215803, 0.26129909, -0.08914106, -0.0, 0.47621346], [0.48707861, -0.15730714, 0.24902688, -0.09171974, -0.0, 0.48707861], [0.49778743, -0.14204471, 0.23678508, -0.09474037, -0.0, 0.49778743], [0.50829414, -0.12638531, 0.22456439, -0.09817908, -0.0, 0.50829414], [0.51855143, -0.11034747, 0.21235541, -0.10200794, -0.0, 0.51855143], [0.52851091, -0.09395364, 0.20014852, -0.10619488, -0.0, 0.52851091], [0.53812366, -0.07723019, 0.18793398, -0.11070379, -0.0, 0.53812366], [0.54734077, -0.06020726, 0.1757019, -0.11549463, -0.0, 0.54734077], [0.55611396, -0.04291864, 0.1634423, -0.12052367, -0.0, 0.55611396], [0.56439621, -0.02540144, 0.15114522, -0.12574378, -0.0, 0.56439621], [0.57214242, -0.00769592, 0.13880073, -0.13110482, -0.0, 0.57214242], [0.57931007, 0.01015495, 0.12639914, -0.13655409, -0.0, 0.57931007], [0.58585983, 0.02810586, 0.11393101, -0.14203687, -0.0, 0.58585983], [0.59175618, 0.04610959, 0.10138736, -0.14749696, -0.0, 0.59175618], [0.59696792, 0.06411753, 0.0887598, -0.15287732, -0.0, 0.59696792], [0.60146865, 0.08208017, 0.07604059, -0.15812076, -0.0, 0.60146865], [0.60523715, 0.09994767, 0.06322287, -0.16317053, -0.0, 0.60523715], [0.60825766, 0.11767037, 0.0503007, -0.16797107, -0.0, 0.60825766], [0.6105201, 0.13519938, 0.03726923, -0.17246861, -0.0, 0.6105201], [0.61202007, 0.1524871, 0.02412474, -0.17661184, -0.0, 0.61202007], [0.61275892, 0.16948773, 0.01086473, -0.18035246, -0.0, 0.61275892], [0.61274352, 0.18615776, -0.00251203, -0.18364573, -0.0, 0.61274352], [0.61198609, 0.20245644, -0.01600548, -0.18645096, -0.0, 0.61198609], [0.61050387, 0.21834614, -0.02961432, -0.18873182, -0.0, 0.61050387], [0.60831871, 0.23379272, -0.04333597, -0.19045675, -0.0, 0.60831871], [0.60545663, 0.24876583, -0.05716671, -0.19159913, -0.0, 0.60545663], [0.6019473, 0.2632391, -0.07110171, -0.19213739, -0.0, 0.6019473], [0.59782353, 0.27719033, -0.08513519, -0.19205515, -0.0, 0.59782353], [0.59312072, 0.29060159, -0.0992605, -0.19134109, -0.0, 0.59312072], [0.58787629, 0.30345923, -0.11347031, -0.18998891, -0.0, 0.58787629], [0.58212915, 0.31575389, -0.12775675, -0.18799714, -0.0, 0.58212915], [0.5759192, 0.32748043, -0.14211159, -0.18536884, -0.0, 0.5759192], [0.5692868, 0.33863777, -0.15652641, -0.18211136, -0.0, 0.5692868], [0.56227236, 0.34922872, -0.17099277, -0.17823595, -0.0, 0.56227236], [0.5549159, 0.35925981, -0.18550244, -0.17375738, -0.0, 0.5549159], [0.54725672, 0.36874102, -0.20004749, -0.16869353, -0.0, 0.54725672], [0.53933306, 0.37768551, -0.21462057, -0.16306494, -0.0, 0.53933306], [0.53118185, 0.38610933, -0.22921496, -0.15689437, -0.0, 0.53118185], [0.5228385, 0.39403116, -0.24382481, -0.15020635, -0.0, 0.5228385], [0.49366836, 0.38560357, -0.2483544, -0.13724917, -0.0, 0.49366836], [0.5068743, 0.40880514, -0.27288755, -0.13591759, -0.0, 0.5068743], [0.49809307, 0.41536883, -0.28755001, -0.12781882, -0.0, 0.49809307], [0.48924461, 0.42152198, -0.30221341, -0.11930857, -0.0, 0.48924461], [0.48035504, 0.42729222, -0.31687805, -0.11041416, -0.0, 0.48035504], [0.47144844, 0.43270808, -0.33154579, -0.10116228, -0.0, 0.47144844], [0.46254689, 0.43779881, -0.3462201, -0.0915787, -0.0, 0.46254689], [0.45367053, 0.44259415, -0.36090617, -0.08168798, -0.0, 0.45367053], [0.44483761, 0.44712417, -0.37561095, -0.07151323, -0.0, 0.44483761], [0.43606457, 0.45141916, -0.39034328, -0.06107588, -0.0, 0.43606457], [0.42736612, 0.45550945, -0.40511394, -0.05039551, -0.0, 0.42736612], [0.41875535, 0.45942539, -0.41993574, -0.03948964, -0.0, 0.41875535], [0.41024378, 0.46319728, -0.43482367, -0.02837361, -0.0, 0.41024378], [0.40184151, 0.46685534, -0.44979494, -0.0170604, -0.0, 0.40184151], [0.3935573, 0.47042975, -0.46486918, -0.00556057, -0.0, 0.3935573], [0.38539866, 0.47395064, -0.48006858, 0.00611794, -0.0, 0.38539866], [0.37737195, 0.47744821, -0.49541807, 0.01796986, -0.0, 0.37737195], [0.36948248, 0.48095282, -0.51094559, 0.02999277, -0.0, 0.36948248], [0.36173458, 0.48449513, -0.52668233, 0.04218719, -0.0, 0.36173458], [0.35413173, 0.48810629, -0.54266308, 0.05455679, -0.0, 0.35413173], [0.34667657, 0.49181813, -0.55892665, 0.06710852, -0.0, 0.34667657], [0.33937107, 0.49566347, -0.57551633, 0.07985286, -0.0, 0.33937107], [0.3322165, 0.49967641, -0.59248049, 0.09280408, -0.0, 0.3322165], [0.32521359, 0.50389272, -0.60987329, 0.10598057, -0.0, 0.32521359], [0.31836254, 0.50835033, -0.62775559, 0.11940526, 0.0, 0.31836254], [0.31166307, 0.51308981, -0.64619592, 0.13310611, 0.0, 0.31166307], [0.30511452, 0.51815513, -0.66527188, 0.14711676, 0.0, 0.30511452], [0.29871585, 0.52359438, -0.68507168, 0.16147731, 0.0, 0.29871585], [0.29246574, 0.5294608, -0.70569613, 0.17623533, 0.0, 0.29246574], [0.28636257, 0.53581401, -0.72726116, 0.19144714, 0.0, 0.28636257], [0.2804045, 0.5427215, -0.74990088, 0.20717938, 0.0, 0.2804045], [0.2745895, 0.55026053, -0.77377162, 0.2235111, 0.0, 0.2745895], [0.26891537, 0.55852061, -0.799057, 0.24053639, 0.0, 0.26891537], [0.26337977, 0.5676066, -0.8259745, 0.2583679, 0.0, 0.26337977], [0.25798026, 0.57764287, -0.85478423, 0.27714136, 0.0, 0.25798026], [0.25271431, 0.5887787, -0.88580042, 0.29702172, 0.0, 0.25271431], [0.24757934, 0.60119573, -0.91940716, 0.31821143, 0.0, 0.24757934], [0.24257274, 0.615118, -0.95607991, 0.34096191, 0.0, 0.24257274], [0.23769185, 0.63082623, -0.99641578, 0.36558955, 0.0, 0.23769185], [0.23293406, 0.64867812, -1.04117699, 0.39249886, 0.0, 0.23293406], [0.22829678, 0.66913832, -1.09135472, 0.4222164, 0.0, 0.22829678], [0.22377749, 0.69282369, -1.14826601, 0.45544232, 0.0, 0.22377749], [0.21937377, 0.72057437, -1.21370544, 0.49313107, 0.0, 0.21937377]]
       
        """
        velocity_list = []
        for j in range(len(position_list) - 1):
            velocity = [
                (position_list[j+1][i] - position_list[j][i]) / dt
                for i in range(6)
            ]
            velocity_list.append(velocity)
        velocity_list.append([0] * 6)
        """
        print("Joint Velocitiy")
        velocity_list_array = np.array(velocity_list)
        print(velocity_list_array.shape)
        
        duration_list = np.linspace(dt, time_duration, num_points).tolist()
        duartion_array = np.array(duration_list)
        print(f"duration shape:{duartion_array.shape}")
        
        gamma = 1
        u_act_list = []
        h_list=[]
        for i, position in enumerate(position_list):
            print()
            print(f"INDEX {i+1}")
            p_obs = np.array([0, 0.45, 0.15])
            q = position_list[i]
            print(f"position:{position_list[i]}")
            p = cartesian_positions[i]
            print(f"cartesian:{cartesian_positions[i]}")
            u_des = velocity_list[i]
            print(f"velocity:{velocity_list[i]}")
            J = self.calculate_jacobian(position)
            #print(f"J:{J[0:3,:]}")
            S_J = self.swap_jacobian_blocks(J)
            print("Jacobian")
            print(S_J)
            u_act, hx = self.manipulator_cbf(p_obs, q, p, J, u_des, gamma)
            if u_act is not None:
                u_act_list.append(u_act)
                h_list.append(hx)
            else:
                u_act_list.append(u_des)  
        
        for i, h in enumerate(h_list):
            print(f"{[i]} hx: {h}")

        print("u_act_array:")
        u_act_array = np.array(u_act_list)
        print(u_act_array.shape)

        for i , updated_position in enumerate(u_act_list):
            print(f"[{i+1}] u_act_list: {updated_position}")

        updated_position_list = [position_list[0]]    
        for i in range(1, len(u_act_list)):
            previous_position = updated_position_list[-1]
            velocity = u_act_list[i - 1]
            new_position = [previous_position[j] + velocity[j] * dt for j in range(len(previous_position))]
            updated_position_list.append(new_position)

        for i, position in enumerate(updated_position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.velocities = u_act_list[i]
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)
       
        updated_position_list_array = np.array(updated_position_list)
        for i, safety_position in enumerate(updated_position_list):
            print(f"{i+1} safety_q:{updated_position_list_array[i]}")
            
        last_position = self.get_cartesian_positions([updated_position_list[-1]])
        print(f"last position: {last_position}")
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()
 
        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))
            
if __name__ == "__main__":
    client = TrajectoryClient()
    client.send_joint_trajectory()


