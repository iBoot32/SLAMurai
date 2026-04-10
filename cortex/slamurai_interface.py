import cv2
import time
import threading
import textwrap
import os
import queue
from colorama import init, Fore, Style
from slamurai_engine import SLAMuraiEngine

init(autoreset=True)

# --- UI CONFIGURATION ---
INFERENCE_HZ = 0.5
WEBCAM_INDEX = 0
MAIN_COL_WIDTH = 55
SIDE_COL_WIDTH = 75
ROWS_PER_ENTRY = 4

class SLAMuraiInterface:
    def __init__(self):
        self.engine = SLAMuraiEngine()
        self.cap = cv2.VideoCapture(WEBCAM_INDEX)
        self.running = True
        self.fps = 0.0
        self.last_vid_time = time.time()
        self.query_queue = queue.Queue()
        self.last_chat_answer = ""
        self.last_chat_query = ""
        
        print(f"{Fore.CYAN}--- Initializing SLAMurai Chat Interface ---")

    def clear_console(self):
        os.system('cls' if os.name == 'nt' else 'clear')

    def print_cli_dashboard(self):
        self.clear_console()
        total_w = MAIN_COL_WIDTH + SIDE_COL_WIDTH + 3
        
        # Header
        print(f"{Fore.YELLOW}{'=' * total_w}")
        print(f"{Fore.WHITE} COGNITIVE MONITOR | {Fore.GREEN}LIVE VISION + DUAL-STREAM MEMORY ".center(total_w))
        print(f"{Fore.YELLOW}{'=' * total_w}")

        # 1. Main Column (Current thought and Pose)
        main_content = f"LATEST STRUCTURED THOUGHT:\n{self.engine.state['observation']}"
        main_lines = textwrap.wrap(main_content, width=MAIN_COL_WIDTH - 2)
        main_lines.insert(0, f"POSE: {self.engine.state['pose']}")
        main_lines.insert(1, "-" * (MAIN_COL_WIDTH - 2))

        # 2. Sidebar (STM then LTM)
        side_lines = []
        side_lines.append(f"{Style.BRIGHT}{Fore.CYAN}SHORT-TERM LOG (STM)")
        side_lines.append("-" * (SIDE_COL_WIDTH - 2))
        
        # Display last 3 entries for cleaner view
        for thought in list(self.engine.stm)[-3:]:
            wrapped = textwrap.wrap(thought, width=SIDE_COL_WIDTH - 5)
            for row in range(ROWS_PER_ENTRY):
                side_lines.append(f"  {wrapped[row]}" if row < len(wrapped) else "")
            side_lines.append("  " + Fore.BLACK + "."*15 + Fore.WHITE)

        side_lines.append("") 
        side_lines.append(f"{Style.BRIGHT}{Fore.MAGENTA}LONG-TERM ARCHIVE (LTM)")
        side_lines.append("-" * (SIDE_COL_WIDTH - 2))
        
        if not self.engine.ltm:
            side_lines.append("  (Archive pending distillation...)")
        else:
            for entry in list(self.engine.ltm)[-2:]:
                wrapped = textwrap.wrap(entry, width=SIDE_COL_WIDTH - 5)
                for row in range(ROWS_PER_ENTRY):
                    side_lines.append(f"  {wrapped[row]}" if row < len(wrapped) else "")
                side_lines.append("  " + Fore.BLACK + "."*15 + Fore.WHITE)

        # 3. Print combined columns
        max_lines = max(len(main_lines), len(side_lines))
        for i in range(max_lines):
            left = main_lines[i] if i < len(main_lines) else ""
            right = side_lines[i] if i < len(side_lines) else ""
            print(f"{Fore.GREEN}{left:<{MAIN_COL_WIDTH}}{Fore.WHITE}| {Fore.WHITE}{right}")

        # 4. Chat Display
        if self.last_chat_answer:
            print(f"\n{Fore.YELLOW}--- INTERACTION ---")
            print(f"{Fore.CYAN}YOU: {Fore.WHITE}{self.last_chat_query}")
            wrapped_ans = textwrap.wrap(self.last_chat_answer, width=total_w - 10)
            for line in wrapped_ans:
                print(f"{Fore.MAGENTA}AI:  {Fore.WHITE}{line}")

        # 5. Footer Stats
        print(f"{Fore.YELLOW}{'=' * total_w}")
        stats_str = (f" GEN: {self.engine.stats['gen_time']:.1f}s | "
                     f"LTM: {len(self.engine.ltm)}/{self.engine.LTM_M} | "
                     f"PRESS ENTER TO ASK A QUESTION")
        print(f"{Fore.GREEN}{stats_str.center(total_w)}")

    def draw_minimal_hud(self, frame):
        dot_color = (0, 0, 255) if self.engine.is_thinking else (0, 255, 0)
        cv2.circle(frame, (30, 30), 10, dot_color, -1)
        cv2.circle(frame, (30, 30), 11, (255, 255, 255), 1)
        status = "THINKING..." if self.engine.is_thinking else "WATCHING"
        cv2.putText(frame, status, (50, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, dot_color, 2)
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (frame.shape[1]-120, 35), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return frame

    def input_worker(self):
        """Thread for capturing user input without blocking the dashboard."""
        while self.running:
            user_input = input("") # Wait for enter or text
            if user_input.strip().lower() in ['q', 'exit']:
                self.running = False
                break
            if user_input.strip():
                self.query_queue.put(user_input)

    def vlm_thread_worker(self):
        while self.running:
            # Priority 1: Respond to User Queries
            if not self.query_queue.empty():
                query = self.query_queue.get()
                self.last_chat_query = query
                self.last_chat_answer = self.engine.chat(query)
                self.print_cli_dashboard()
                continue # Skip routine think once query is answered

            # Priority 2: Standard Cognitive Loop
            if self.engine.state["last_frame"] is not None and not self.engine.is_thinking:
                self.engine.think()
                self.print_cli_dashboard()
            
            time.sleep(1.0 / INFERENCE_HZ)

    def start(self):
        # Start the cognitive loop and input thread
        threading.Thread(target=self.vlm_thread_worker, daemon=True).start()
        threading.Thread(target=self.input_worker, daemon=True).start()
        
        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret: break
                
                self.engine.state["last_frame"] = frame
                
                now = time.time()
                dt = now - self.last_vid_time
                if dt > 0: self.fps = (self.fps * 0.9) + ((1.0 / dt) * 0.1)
                self.last_vid_time = now
                
                cv2.imshow("SLAMurai Vision", self.draw_minimal_hud(frame))
                if cv2.waitKey(1) & 0xFF == ord('q'): break
        finally:
            self.cleanup()

    def cleanup(self):
        self.running = False
        self.cap.release()
        cv2.destroyAllWindows()
        print(f"\n{Fore.RED}System Shutdown.")

if __name__ == '__main__':
    ui = SLAMuraiInterface()
    ui.start()