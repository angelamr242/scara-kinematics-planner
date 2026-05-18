import numpy as np

def generate_trapezoidal_profile(q_start, q_goal, max_vel, max_accel, dt=0.01):
    """
    Generates a smooth position array from start to goal over time dt.
    """
    dist = q_goal - q_start
    sign = np.sign(dist)
    dist = abs(dist)
    
    if dist == 0:
        return np.array([0]), np.array([q_start])
    
    # Calculate critical profile times and distances
    t_accel = max_vel / max_accel
    d_accel = 0.5 * max_accel * (t_accel ** 2)
    
    # Check if the distance is too short to reach max velocity (Triangle Profile)
    if 2 * d_accel > dist: 
        t_accel = np.sqrt(dist / max_accel)
        t_cruise = 0
        v_cruise = max_accel * t_accel
    else:
        t_cruise = (dist - (2 * d_accel)) / max_vel
        v_cruise = max_vel
        
    t_total = (2 * t_accel) + t_cruise
    time_steps = np.arange(0, t_total, dt)
    positions = []
    
    for t in time_steps:
        if t <= t_accel: 
            # Phase 1: Acceleration
            p = 0.5 * max_accel * (t ** 2)
        elif t <= (t_accel + t_cruise): 
            # Phase 2: Constant Velocity Cruise
            p = d_accel + v_cruise * (t - t_accel)
        else: 
            # Phase 3: Deceleration
            t_dec = t - t_accel - t_cruise
            p = dist - d_accel + (v_cruise * t_dec) - (0.5 * max_accel * (t_dec ** 2))
            
        positions.append(q_start + (sign * p))
        
    return time_steps, np.array(positions)

# Quick verification test
if __name__ == "__main__":
    t, pos = generate_trapezoidal_profile(0.0, 1.5, max_vel=1.0, max_accel=2.0)
    print(f"Generated {len(pos)} waypoints. Final position: {round(pos[-1], 3)}")
