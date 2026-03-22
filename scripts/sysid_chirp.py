import numpy as np
import time
import csv
import os
import motor_driver # This is the compiled .so from encos-driver

def generate_chirp(t, f_start, f_end, T, amplitude):
    """
    Generate a linear chirp signal.
    f(t) = f_start + (f_end - f_start) * t / T
    phase(t) = 2 * pi * integrate(f(t) dt) = 2 * pi * (f_start * t + 0.5 * (f_end - f_start) * t^2 / T)
    """
    k = (f_end - f_start) / T
    phase = 2 * np.pi * (f_start * t + 0.5 * k * t**2)
    return amplitude * np.sin(phase)

def run_sysid(motor_id=0, mode='5x'):
    # Parameters from user's image for 4310 motor
    params = {
        '2x': {'f_end': 23.36, 'amp_deg': 13.91},
        '3x': {'f_end': 35.04, 'amp_deg': 9.27},
        '5x': {'f_end': 58.40, 'amp_deg': 5.56}
    }
    
    if mode not in params:
        print(f"Unknown mode {mode}, defaulting to 5x")
        mode = '5x'
        
    f_start = 0.1
    f_end = params[mode]['f_end']
    amplitude = np.deg2rad(params[mode]['amp_deg'])
    T = 20.0  # Duration in seconds
    fs = 500.0 # Frequency of control loop (Hz)
    
    print(f"Starting SysID Chirp Sweep ({mode}):")
    print(f"  f_start: {f_start} Hz")
    print(f"  f_end:   {f_end} Hz")
    print(f"  Amp:     {params[mode]['amp_deg']} deg ({amplitude:.4f} rad)")
    print(f"  Duration: {T} s")

    # Initialize driver
    motor_driver.initialize()
    time.sleep(1) # Wait for scan
    
    # Pre-generate trajectory
    t_arr = np.linspace(0, T, int(T * fs))
    pos_targets = generate_chirp(t_arr, f_start, f_end, T, amplitude)
    
    data_log = []
    
    start_time = time.time()
    try:
        for i, target in enumerate(pos_targets):
            # Control loop timing
            loop_start = time.time()
            
            # Send command
            # The driver takes an array of MOTOR_COUNT. We only control one for SysID.
            qpos_cmd = np.zeros(20) # Assuming 20 motors max as per constant.h
            qpos_cmd[motor_id] = target
            motor_driver.send_qpos(qpos_cmd)
            
            # Pull feedback
            motor_driver.pull_msg()
            fb_pos = motor_driver.get_positions()
            fb_vel = motor_driver.get_velocities()
            fb_tor = motor_driver.get_torques()
            
            # Record
            data_log.append([
                time.time() - start_time,
                target,
                fb_pos[motor_id],
                fb_vel[motor_id],
                fb_tor[motor_id]
            ])
            
            # Wait for next step
            elapsed = time.time() - loop_start
            wait_time = (1.0 / fs) - elapsed
            if wait_time > 0:
                time.sleep(wait_time)
                
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        motor_driver.uninitialize()

    # Save to CSV
    filename = f"sysid_data_{mode}_{int(time.time())}.csv"
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'target_pos', 'actual_pos', 'actual_vel', 'actual_tor'])
        writer.writerows(data_log)
    
    print(f"Done. Data saved to {filename}")

if __name__ == "__main__":
    # Example usage: python sysid_chirp.py
    run_sysid(motor_id=0, mode='5x')
