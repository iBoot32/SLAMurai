import serial
import time

def read_odom(ser):
    line = ser.readline()
    if not line:
        return None
    return line.decode(errors='ignore').strip()

if __name__ == "__main__":
    port = '/dev/ttyACM0'
    baud = 115200
    N = 500

    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(2)
    ser.reset_input_buffer()

    msg_count = 0
    last_cmd_vel_send_time = 0
    window_start_time = time.time()

    while True:
        # Odom receive and hz calculation
        odom = read_odom(ser)
        if odom:
            msg_count += 1
            if msg_count >= N:
                elapsed = time.time() - window_start_time
                print(f"Odom Read {N/elapsed:.1f}Hz (last {N} msgs)")
                msg_count = 0
                window_start_time = time.time()
            if msg_count % 50 == 0: print("Odom Read:", odom)

        # Send cmd_vel test (in reality nav2 will only give to us at 10hz, so no timing needed)
        # now = time.time()
        # if now - last_cmd_vel_send_time >= (1.0 / 10):
            # last_cmd_vel_send_time = now
            # cmd_vel = "CMD_VEL 0.1 0.0\n"  # example: linear_x=0.1 m/s, angular_z=0.0 rad/s
            # ser.write(cmd_vel.encode())

