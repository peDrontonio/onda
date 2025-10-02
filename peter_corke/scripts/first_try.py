import roboticstoolbox as rtb
import numpy as np
import time
from spatialmath import SE3

class RRRPRManipulator():
    def __init__(self, l1: float, l2: float, l3: float, l4: float, l5: float) -> None:
        """
            ## RRRPR Manipulator Class
            This class defines the RRRPR manipulator, integrating the simulation functions from Peter Corke's toolbox.
            
            - **Input:** 
                - `l1`: float
                - `l2`: float
                - `l3`: float
                - `l4`: float
                - `l5`: float
        """
        
        # robot dimensions
        self.l1_ = l1
        self.l2_ = l2
        self.l3_ = l3
        self.l4_ = l4
        self.l5_ = l5
        
        # Definindo as juntas do robô
        self.robot = rtb.DHRobot([
            rtb.RevoluteDH(d=0, a=self.l1_, alpha=-np.pi/2.0),  # 1ª junta rotacional
            rtb.RevoluteDH(d=0, a=self.l2_, alpha=0),  # 2ª junta rotacional
            rtb.RevoluteDH(d=0, a=self.l3_, alpha=0),  # 3ª junta rotacional
            rtb.PrismaticDH(theta=0, a=0, alpha=0, qlim=[0, self.l4_]),  # 4ª junta prismática
            rtb.RevoluteDH(d=0, a=self.l5_, alpha=0),  # 5ª junta rotacional
        ], name="RRRPR")
        
        
    def ik_calculator(self, end_effector: SE3) -> np.ndarray:
        '''
            ### Inverse Kinematics Calculator:
            Placeholder for inverse kinematics. For now, returns zeros.
            - **Input:** `end_effector: SE3`
            
            - **Output:** `q: np.ndarray (1, 5)`  
        '''
        
        q = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Placeholder
        
        # TODO: Implement proper IK for RRRPR
        
        return q

    def static_display(self) -> None:
        '''
            ### Static Display
            Function for static plot of the manipulator.
            - Display pose: `q0 = [ pi/4, pi/4, pi/4, 5.0, -pi/4 ]`
        '''
        
        q0 = [
            np.pi/4,    # Theta1
            np.pi/4,    # Theta2
            np.pi/4,    # Theta3
            5.0,        # D4
            -np.pi/4,   # Theta5
        ]
        
        self.robot.plot(q0)
        
    def dynamic_display(self) -> None:
        '''
            ### Dynamic display
            Function for dynamic plot of the manipulator.
        '''
        
        # First position
        q_0 = [
            np.pi/4,
            np.pi/4,
            np.pi/4,
            5.0,  
            -np.pi/4
        ]
        
        # Second position
        q_1 = [
            0,
            np.pi/6,
            np.pi/6,
            5.0,
            -np.pi/4
        ]
        
        # Third position
        q_2 = [
            0,
            np.pi/6,
            np.pi/6,
            7.0,
            -np.pi/2
        ]
        
        # Combining the joint positions
        joint_positions = np.vstack((q_0, q_1, q_2))
        # Calculating trajectories
        traj = self.plan_trajectory_by_joints(joint_positions=joint_positions)
        
        # Plot trajectory
        self.robot.plot(q=traj)  
    
    def plan_trajectory_by_joints(self, joint_positions: np.ndarray, steps: int = 1000) -> np.ndarray:
        '''
            ### Plan Trajectory
            This function receives a sequence of joint values and calculates and returns the trajectory passing through all joint positions.
            - input: `joint_positions: np.ndarray (n, 5)`
            - output: `traj: np.ndarray (steps*(n-1), 5)`
        '''
        if joint_positions.shape[1] != 5:
            raise TypeError("The parameter must be a numpy.ndarray in format (n, 5).")

        # Calculating the trajectory
        traj = []
        for i in range(joint_positions.shape[0] - 1):
            traj_segment = rtb.jtraj(joint_positions[i], joint_positions[i + 1], steps)
            traj.append(traj_segment.q)

        return np.vstack(traj) if traj else np.array([])
