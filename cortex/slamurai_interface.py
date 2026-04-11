import cv2
import time
import threading
import textwrap
import os
import queue
import roslibpy
import numpy as np
import re
from colorama import init, Fore, Style
from slamurai_engine import SLAMuraiEngine

init(autoreset=True)

ROBOT_IP = '10.0.0.98'
INFERENCE_HZ = 0.5
MAIN_COL_WIDTH = 55
SIDE_COL_WIDTH = 75
ROWS_PER_ENTRY = 4
ROBOT_RADIUS_METERS = 0.15

class SLAMuraiInterface:
    def __init__(self):
        self.engine = SLAMuraiEngine()
        self.video_url = f"http://{ROBOT_IP}:8080/stream?topic=/camera/color/image_raw&type=mjpeg&quality=40"
        self.cap = cv2.VideoCapture(self.video_url)
        
        self.client = roslibpy.Ros(host=ROBOT_IP, port=9090)
        self.pose_sub = roslibpy.Topic(self.client, '/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped')
        self.map_sub = roslibpy.Topic(self.client, '/map', 'nav_msgs/msg/OccupancyGrid')
        self.log_sub = roslibpy.Topic(self.client, '/rosout', 'rcl_interfaces/msg/Log')
        
        # Publisher for Nav2 Goals
        self.nav_pub = roslibpy.Topic(self.client, '/move_base_simple/goal', 'geometry_msgs/PoseStamped')       
         
        self.map_data = {"img": None, "resolution": 0.05, "origin_x": 0.0, "origin_y": 0.0, "orig_w": 0, "orig_h": 0, "received": False}
        self.running = True
        self.fps = 0.0
        self.last_vid_time = time.time()
        self.query_queue = queue.Queue()
        self.last_chat_answer = ""
        self.last_chat_query = ""
        
        print(f"{Fore.CYAN}--- Initializing SLAMurai Chat Interface ---")

    def log_callback(self, msg):
        name = msg.get('name', '')
        if any(node in name for node in ['bt_navigator', 'controller_server', 'planner_server']):
            timestamp = time.strftime('%H:%M:%S')
            clean_msg = f"[{timestamp}] {msg['msg']}"
            self.engine.state["nav_logs"].append(clean_msg)

    def map_callback(self, msg):
        if self.map_data["received"]: return 
        try:
            info = msg['info']
            self.map_data.update({"resolution": info['resolution'], "origin_x": info['origin']['position']['x'], "origin_y": info['origin']['position']['y'], "orig_w": info['width'], "orig_h": info['height']})
            data = np.array(msg['data'], dtype=np.int8).reshape((info['height'], info['width']))
            map_img = np.zeros((info['height'], info['width']), dtype=np.uint8)
            map_img[data == -1], map_img[data == 0], map_img[data == 100] = 127, 255, 0
            flipped = cv2.flip(map_img, 0)
            self.map_data["img"] = cv2.rotate(flipped, cv2.ROTATE_90_COUNTERCLOCKWISE)
            self.map_data["received"] = True
            res = self.map_data["resolution"]
            ox, oy = self.map_data["origin_x"], self.map_data["origin_y"]
            self.engine.state["map_bounds"] = {"x": [round(ox, 2), round(ox + (self.map_data["orig_w"] * res), 2)], "y": [round(oy, 2), round(oy + (self.map_data["orig_h"] * res), 2)]}
            print(f"{Fore.GREEN}Map Calibrated.")
        except Exception as e: print(f"Map Error: {e}")

    def update_pose(self, msg):
        try:
            pos = msg['pose']['pose']['position']
            ori = msg['pose']['pose']['orientation']
            qx, qy, qz, qw = ori['x'], ori['y'], ori['z'], ori['w']
            yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            self.engine.state["pose"] = {"x": round(pos['x'], 3), "y": round(pos['y'], 3), "yaw": round(yaw, 3)}
        except: pass

    def send_nav_goal(self, x, y, yaw):
        """Converts yaw to quaternion and publishes to Nav2 with essential timestamp."""
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        
        # Nav2 requires a valid timestamp to accept the goal
        now = time.time()
        secs = int(now)
        nsecs = int((now - secs) * 1e9)
        
        goal = {
            'header': {
                'frame_id': 'map',
                'stamp': {'secs': secs, 'nsecs': nsecs}
            },
            'pose': {
                'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': qz, 'w': qw}
            }
        }
        self.nav_pub.publish(roslibpy.Message(goal))
        # Store for persistent dashboard display
        self.engine.state["last_system_msg"] = f"NAV GOAL SENT: X:{x} Y:{y} Yaw:{yaw}"

    def vlm_thread_worker(self):
        while self.running:
            if not self.query_queue.empty():
                query = self.query_queue.get()
                self.last_chat_query = query
                raw_answer = self.engine.chat(query)
                
                # Update answer immediately so user sees the response
                self.last_chat_answer = raw_answer
                
                # Robust Regex: handles case sensitivity, whitespace, and multi-line goals
                nav_match = re.search(r"NAV_GOAL:\s*navigate\((.*?)\)", raw_answer, re.IGNORECASE | re.DOTALL)
                
                if nav_match:
                    try:
                        coords = [float(c.strip()) for c in nav_match.group(1).split(',')]
                        if len(coords) == 3:
                            self.send_nav_goal(coords[0], coords[1], coords[2])
                    except Exception as e:
                        self.engine.state["last_system_msg"] = f"Nav Parse Error: {e}"
                
                self.print_cli_dashboard()
                continue

            if self.engine.state["last_frame"] is not None and not self.engine.is_thinking:
                self.engine.think()
                self.print_cli_dashboard()
            
            time.sleep(1.0 / INFERENCE_HZ)

    def create_dashboard(self, camera_frame):
        if camera_frame is None: return None
        h_cam, w_cam, _ = camera_frame.shape
        if self.map_data["received"]:
            map_viz = cv2.cvtColor(self.map_data["img"], cv2.COLOR_GRAY2BGR)
            p = self.engine.state["pose"]
            mx_orig = int((p["x"] - self.map_data["origin_x"]) / self.map_data["resolution"])
            my_orig = (self.map_data["orig_h"] - 1) - int((p["y"] - self.map_data["origin_y"]) / self.map_data["resolution"])
            mx, my = my_orig, (self.map_data["orig_w"] - 1) - mx_orig
            px_radius = max(3, int(ROBOT_RADIUS_METERS / self.map_data["resolution"]))
            if 0 <= mx < map_viz.shape[1] and 0 <= my < map_viz.shape[0]:
                cv2.circle(map_viz, (mx, my), px_radius, (0, 0, 255), -1)
                nyaw = p["yaw"] + (np.pi / 2.0)
                dx, dy = int(px_radius*4 * np.cos(nyaw)), -int(px_radius*4 * np.sin(nyaw)) 
                cv2.line(map_viz, (mx, my), (mx + dx, my + dy), (255, 0, 0), 2)
            map_resized = cv2.resize(map_viz, (h_cam, h_cam), interpolation=cv2.INTER_AREA)
        else:
            map_resized = np.zeros((h_cam, h_cam, 3), dtype=np.uint8)
        p = self.engine.state["pose"]
        cv2.rectangle(camera_frame, (0, h_cam-60), (w_cam, h_cam), (0,0,0), -1)
        cv2.putText(camera_frame, f"LIVE POSE: X:{p['x']} Y:{p['y']} YAW:{p['yaw']}", (15, h_cam-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return np.hstack((camera_frame, map_resized))

    def clear_console(self):
        os.system('cls' if os.name == 'nt' else 'clear')

    def print_cli_dashboard(self):
        self.clear_console()
        total_w = MAIN_COL_WIDTH + SIDE_COL_WIDTH + 3
        print(f"{Fore.YELLOW}{'=' * total_w}")
        print(f"{Fore.WHITE} COGNITIVE MONITOR | {Fore.GREEN}LIVE VISION + DUAL-STREAM MEMORY ".center(total_w))
        print(f"{Fore.YELLOW}{'=' * total_w}")
        main_content = f"LATEST STRUCTURED THOUGHT:\n{self.engine.state['observation']}"
        main_lines = textwrap.wrap(main_content, width=MAIN_COL_WIDTH - 2)
        main_lines.insert(0, f"POSE: {self.engine.state['pose']}")
        main_lines.insert(1, "-" * (MAIN_COL_WIDTH - 2))
        side_lines = [f"{Style.BRIGHT}{Fore.CYAN}SHORT-TERM LOG (STM)", "-" * (SIDE_COL_WIDTH - 2)]
        for thought in list(self.engine.stm)[-3:]:
            wrapped = textwrap.wrap(thought, width=SIDE_COL_WIDTH - 5)
            for row in range(ROWS_PER_ENTRY): side_lines.append(f"  {wrapped[row]}" if row < len(wrapped) else "")
            side_lines.append("  " + Fore.BLACK + "."*15 + Fore.WHITE)
        side_lines.append("") 
        side_lines.append(f"{Style.BRIGHT}{Fore.MAGENTA}LONG-TERM ARCHIVE (LTM)")
        side_lines.append("-" * (SIDE_COL_WIDTH - 2))
        if not self.engine.ltm: side_lines.append("  (Archive pending distillation...)")
        else:
            for entry in list(self.engine.ltm)[-2:]:
                wrapped = textwrap.wrap(entry, width=SIDE_COL_WIDTH - 5)
                for row in range(ROWS_PER_ENTRY): side_lines.append(f"  {wrapped[row]}" if row < len(wrapped) else "")
                side_lines.append("  " + Fore.BLACK + "."*15 + Fore.WHITE)
        max_lines = max(len(main_lines), len(side_lines))
        for i in range(max_lines):
            left, right = main_lines[i] if i < len(main_lines) else "", side_lines[i] if i < len(side_lines) else ""
            print(f"{Fore.GREEN}{left:<{MAIN_COL_WIDTH}}{Fore.WHITE}| {Fore.WHITE}{right}")
        
        # Interaction area
        if self.last_chat_answer:
            print(f"\n{Fore.YELLOW}--- INTERACTION ---")
            print(f"{Fore.CYAN}YOU: {Fore.WHITE}{self.last_chat_query}")
            wrapped_ans = textwrap.wrap(self.last_chat_answer, width=total_w - 10)
            for line in wrapped_ans: print(f"{Fore.MAGENTA}AI:  {Fore.WHITE}{line}")

        # Persistent Status Message
        if "last_system_msg" in self.engine.state:
            print(f"\n{Fore.MAGENTA}>> SYSTEM STATUS: {self.engine.state['last_system_msg']}")

        print(f"{Fore.YELLOW}{'=' * total_w}")
        stats_str = (f" GEN: {self.engine.stats['gen_time']:.1f}s | LTM: {len(self.engine.ltm)}/{self.engine.LTM_M} | PRESS ENTER TO ASK")
        print(f"{Fore.GREEN}{stats_str.center(total_w)}")

    def draw_minimal_hud(self, frame):
        dot_color = (0, 0, 255) if self.engine.is_thinking else (0, 255, 0)
        cv2.circle(frame, (30, 30), 10, dot_color, -1)
        status = "THINKING..." if self.engine.is_thinking else "WATCHING"
        cv2.putText(frame, status, (50, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, dot_color, 2)
        return frame

    def input_worker(self):
        while self.running:
            user_input = input("")
            if user_input.strip().lower() in ['q', 'exit']: self.running = False; break
            if user_input.strip(): self.query_queue.put(user_input)

    def start(self):
        self.pose_sub.subscribe(self.update_pose)
        self.map_sub.subscribe(self.map_callback)
        self.log_sub.subscribe(self.log_callback)
        self.client.run()
        threading.Thread(target=self.vlm_thread_worker, daemon=True).start()
        threading.Thread(target=self.input_worker, daemon=True).start()
        try:
            while self.client.is_connected and self.running:
                ret, frame = self.cap.read()
                if not ret: break
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                composite = self.create_dashboard(frame)
                self.engine.state["last_frame"] = composite
                now = time.time()
                dt = now - self.last_vid_time
                if dt > 0: self.fps = (self.fps * 0.9) + ((1.0 / dt) * 0.1)
                self.last_vid_time = now
                if composite is not None: cv2.imshow("SLAMurai Vision", self.draw_minimal_hud(composite.copy()))
                if cv2.waitKey(1) & 0xFF == ord('q'): break
        finally: self.cleanup()

    def cleanup(self):
        self.running = False
        if self.cap.isOpened(): self.cap.release()
        self.client.terminate()
        cv2.destroyAllWindows()
        print(f"\n{Fore.RED}System Shutdown.")

if __name__ == '__main__':
    ui = SLAMuraiInterface()
    ui.start()