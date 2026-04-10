import cv2
import time
import threading
import textwrap
import os
from colorama import init, Fore, Style
from slamurai_engine import SLAMuraiEngine

init(autoreset=True)

# --- UI CONFIGURATION ---
INFERENCE_HZ = 0.5
WEBCAM_INDEX = 0
MAIN_COL_WIDTH = 50
SIDE_COL_WIDTH = 70
ROWS_PER_ENTRY = 3

class SLAMuraiInterface:
    def __init__(self):
        self.engine = SLAMuraiEngine()
        self.cap = cv2.VideoCapture(WEBCAM_INDEX)
        self.running = True
        self.fps = 0.0
        self.last_vid_time = time.time()
        
        print(f"{Fore.CYAN}--- Initializing SLAMurai Interface (UI Separated) ---")

    def clear_console(self):
        os.system('cls' if os.name == 'nt' else 'clear')

    def print_cli_dashboard(self):
        self.clear_console()
        total_w = MAIN_COL_WIDTH + SIDE_COL_WIDTH + 3
        
        # Header
        print(f"{Fore.YELLOW}{'=' * total_w}")
        print(f"{Fore.WHITE} COGNITIVE MONITOR | {Fore.GREEN}LIVE VISION + DUAL-STREAM MEMORY ".center(total_w))
        print(f"{Fore.YELLOW}{'=' * total_w}")

        # 1. Main Column
        main_content = f"LATEST STRUCTURED THOUGHT:\n{self.engine.state['observation']}"
        main_lines = textwrap.wrap(main_content, width=MAIN_COL_WIDTH - 2)
        main_lines.insert(0, f"POSE: {self.engine.state['pose']}")
        main_lines.insert(1, "-" * (MAIN_COL_WIDTH - 2))

        # 2. Sidebar (STM then LTM)
        side_lines = []
        side_lines.append(f"{Style.BRIGHT}{Fore.CYAN}SHORT-TERM LOG (Recent Iterations)")
        side_lines.append("-" * (SIDE_COL_WIDTH - 2))
        
        for thought in list(self.engine.stm)[-4:]:
            wrapped = textwrap.wrap(thought, width=SIDE_COL_WIDTH - 5)
            for row in range(ROWS_PER_ENTRY):
                side_lines.append(f"  {wrapped[row]}" if row < len(wrapped) else "")
            side_lines.append("  " + Fore.BLACK + "."*10 + Fore.WHITE)

        side_lines.append("") 
        side_lines.append(f"{Style.BRIGHT}{Fore.MAGENTA}LONG-TERM ARCHIVE (Distilled Summaries)")
        side_lines.append("-" * (SIDE_COL_WIDTH - 2))
        
        if not self.engine.ltm:
            side_lines.append("  (Archive pending first distillation...)")
        else:
            for entry in list(self.engine.ltm)[-3:]:
                wrapped = textwrap.wrap(entry, width=SIDE_COL_WIDTH - 5)
                for row in range(ROWS_PER_ENTRY):
                    side_lines.append(f"  {wrapped[row]}" if row < len(wrapped) else "")
                side_lines.append("  " + Fore.BLACK + "."*10 + Fore.WHITE)

        # 3. Print combined columns
        max_lines = max(len(main_lines), len(side_lines))
        for i in range(max_lines):
            left = main_lines[i] if i < len(main_lines) else ""
            right = side_lines[i] if i < len(side_lines) else ""
            print(f"{Fore.GREEN}{left:<{MAIN_COL_WIDTH}}{Fore.WHITE}| {Fore.WHITE}{right}")

        # 4. Footer Stats
        print(f"{Fore.YELLOW}{'=' * total_w}")
        stats_str = (f" GEN TIME: {self.engine.stats['gen_time']:.2f}s | "
                     f"SPEED: {self.engine.stats['tokens_per_sec']:.1f} t/s | "
                     f"SUMMARIZER: {self.engine.summary_counter}/{self.engine.STM_N} until offload | "
                     f"LTM: {self.engine.ltm_counter}/{self.engine.LTM_M} capacity until dequeue")
        print(f"{Fore.GREEN}{stats_str.center(total_w)}")

    def draw_minimal_hud(self, frame):
        dot_color = (0, 0, 255) if self.engine.is_thinking else (0, 255, 0)
        cv2.circle(frame, (30, 30), 10, dot_color, -1)
        cv2.circle(frame, (30, 30), 11, (255, 255, 255), 1)
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (frame.shape[1]-100, 35), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return frame

    def vlm_thread_worker(self):
        while self.running:
            time.sleep(1.0 / INFERENCE_HZ)
            if self.engine.state["last_frame"] is not None and not self.engine.is_thinking:
                self.engine.think()
                self.print_cli_dashboard()

    def start(self):
        threading.Thread(target=self.vlm_thread_worker, daemon=True).start()
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

if __name__ == '__main__':
    ui = SLAMuraiInterface()
    ui.start()