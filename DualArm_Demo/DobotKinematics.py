import numpy as np

class DobotKinematics:
    def __init__(self):
        # Dobot Magician dimensions (mm)
        self.L_BASE = 138.0  # Base height (approx)
        self.L_REAR = 135.0  # Rear Arm
        self.L_FORE = 147.0  # Fore Arm
        self.L_END = 60.0    # End Effector (approx)

    def get_links(self, j1, j2, j3, j4):
        """
        Input: Joint angles in degrees
        Output: List of 3D points [P0, P1, P2, P3, P_End]
        """
        # Convert to radians
        j1_rad = np.radians(j1)
        j2_rad = np.radians(j2)
        j3_rad = np.radians(j3)  # Dobot J3 is relative to horizontal
        
        # P0: Base bottom
        p0 = np.array([0, 0, 0])
        
        # P1: Shoulder (Base top)
        p1 = np.array([0, 0, self.L_BASE])
        
        # P2: Elbow (End of Rear Arm)
        # J2 is angle relative to horizontal. 
        # Note: In some conventions J2=0 is vertical. 
        # For Dobot, let's assume J2 is relative to horizontal plane.
        dx2 = self.L_REAR * np.cos(j2_rad)
        dz2 = self.L_REAR * np.sin(j2_rad)
        
        p2 = np.array([
            dx2 * np.cos(j1_rad),
            dx2 * np.sin(j1_rad),
            self.L_BASE + dz2
        ])
        
        # P3: Wrist (End of Fore Arm)
        # J3 in Dobot is usually relative to horizontal (due to parallel linkage)
        # So we calculate its vector independently and add to P2
        dx3 = self.L_FORE * np.cos(j3_rad)
        dz3 = self.L_FORE * np.sin(j3_rad)
        
        p3 = p2 + np.array([
            dx3 * np.cos(j1_rad),
            dx3 * np.sin(j1_rad),
            dz3
        ])
        
        # P_End: End Effector
        # Assuming pointing down or normal to flange? 
        # Let's just visualize the flange point P3 for simplicity, 
        # or add a small vertical drop for the suction cup
        p_end = p3 + np.array([0, 0, -self.L_END])
        
        return np.array([p0, p1, p2, p3, p_end])

