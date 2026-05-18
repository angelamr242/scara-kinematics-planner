import numpy as np

class ScaraKinematics:
    def __init__(self, a1=0.5, a2=0.4):
        # Link lengths in meters
        self.a1 = a1  
        self.a2 = a2  

    def forward_kinematics(self, theta1, theta2, d3):
        """Calculates the [X, Y, Z] end-effector position given joint angles."""
        x = self.a1 * np.cos(theta1) + self.a2 * np.cos(theta1 + theta2)
        y = self.a1 * np.sin(theta1) + self.a2 * np.sin(theta1 + theta2)
        z = -d3  # Prismatic joint moves downward
        return np.array([x, y, z])

    def inverse_kinematics(self, x, y, z):
        """Calculates the [theta1, theta2, d3] joint states given an [X, Y, Z] target."""
        # Calculate theta2 using the law of cosines
        cos_theta2 = (x**2 + y**2 - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
        
        # Clamp value to prevent domain errors due to floating point math
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0) 
        
        # We choose the elbow-down configuration (negative root)
        sin_theta2 = -np.sqrt(1 - cos_theta2**2) 
        theta2 = np.arctan2(sin_theta2, cos_theta2)
        
        # Calculate theta1
        k1 = self.a1 + self.a2 * cos_theta2
        k2 = self.a2 * sin_theta2
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        
        # Calculate d3
        d3 = -z
        
        return np.array([theta1, theta2, d3])

# Quick verification test
if __name__ == "__main__":
    robot = ScaraKinematics()
    target = [0.4, 0.3, -0.1]
    joints = robot.inverse_kinematics(*target)
    print(f"Target XYZ: {target}")
    print(f"Calculated Joints (th1, th2, d3): {np.round(joints, 3)}")
