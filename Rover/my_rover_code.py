import numpy as np 

def generate_rover_code(u1, u2):
    # Wheel radius and separation distance
    r = 0.05
    L = 0.2

    # Define the state-space matrices for differential drive kinematics
    # A is the state transition matrix
    # B is the control input matrix
    # C is the observation matrix
    # D is the feedthrough matrix
    A = np.array([[0, 0, -r/2 * (u1 + u2) * np.sin(0)],  # θ is set to 0 here
                  [0, 0, r/2 * (u1 + u2) * np.cos(0)],
                  [0, 0, 0]])
    B = np.array([[r/2, r/2], [0, 0], [r/L, -r/L]])
    C = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])
    D = np.zeros((3, 2))  # Change the shape of D matrix

    # Create the state-space system
    sys = (A, B, C, D)

    # Time values for simulation
    dt = 0.01  # Time step
    num_samples = 100  # Number of samples

    # Input signal (u1 and u2) for simulation
    U = np.column_stack((u1 * np.ones(num_samples), u2 * np.ones(num_samples)))

    # Calculate the system response using discrete-time simulation
    t, y = simulate_discrete_system(sys, U, dt)

    # Extract the target speed and yaw from the output vector
    target_speed_m = y[:, 0]
    target_yaw_rad = y[:, 2]

    # Convert target_yaw_rad to target_yaw_deg
    target_yaw_deg = np.degrees(target_yaw_rad)

    # Print calculated values
   
    return target_speed_m[-1],target_yaw_deg[-1]
    

def simulate_discrete_system(sys, U, dt):
    A, B, C, D = sys
    num_samples = U.shape[0]
    num_states = A.shape[0]
   
    X = np.zeros((num_samples, num_states))
    Y = np.zeros((num_samples, num_states))

    for k in range(num_samples):
        if k == 0:
            X[k, :] = np.zeros(num_states)  # Initial state
        else:
            X[k, :] = np.dot(A, X[k - 1, :]) + np.dot(B, U[k - 1, :])
       
        Y[k, :] = np.dot(C, X[k, :]) + np.dot(D, U[k, :])

    return np.arange(0, num_samples) * dt, Y  # Return time vector and system response
