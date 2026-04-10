import ollama
import cv2
from collections import deque
from datetime import datetime

# --- CONFIGURATION ---
VLM_MODEL = "SLAMurai" 

class SLAMuraiEngine:
    def __init__(self):
        self.STM_N = 8
        self.LTM_M = 15
        self.stm = deque(maxlen=self.STM_N)
        self.ltm = deque(maxlen=self.LTM_M)
        self.summary_counter = 0
        self.ltm_counter = 0
        
        self.stats = {"gen_time": 0.0, "tokens_per_sec": 0.0, "total_tokens": 0}
        self.state = {
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}, 
            "observation": "Initializing...", 
            "last_frame": None
        }
        self.is_thinking = False

    def summarize_to_ltm(self):
        # Taking the STM raw logs and asking for a distilled index entry
        stm_context = "\n".join(list(self.stm))
        prompt = (
            f"The current time is {datetime.now().strftime('%H:%M:%S')}. "
            f"TASK: Distill the following STM logs into a single 'Key Object Index' for LTM.\n"
            f"Focus on permanent architecture and furniture. Ignore transient objects.\n"
            f"Format: [time window (x:x:x - y:y:y)] | Key: [objects] | Env: [room type]\n"
            f"LOGS:\n{stm_context}"
        )
        try:
            # think=False retained as requested
            response = ollama.generate(model=VLM_MODEL, prompt=prompt, think=False)
            self.ltm.append(response['response'].strip())
        except Exception as e:
            print(f"Summarizer Error: {e}")

    def think(self):
        if self.state["last_frame"] is None: 
            return
        
        self.is_thinking = True
        vlm_input = cv2.resize(self.state["last_frame"], (640, 480)) 
        _, buffer = cv2.imencode('.jpg', vlm_input, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        
        # EXACT prompt phrasing retained
        prompt_body = (
            f"### THE PAST (MEMORY ONLY - DO NOT REPEAT)\n"
            f"<ltm_archive>\n{chr(10).join(list(self.ltm))}\n</ltm_archive>\n"
            f"<stm_log>\n{chr(10).join(list(self.stm))}\n</stm_log>\n\n"
            f"### END THE PAST. THE PRESENT BELOW (ABSOLUTE TRUTH)\n"
            f"Pose: {self.state['pose']}\n"
            f"TASK: Observe CURRENT image. Do not hallucinate based on memory."
            f"The current time is {datetime.now().strftime('%H:%M:%S')}. "
        )
        
        try:
            resp = ollama.generate(model=VLM_MODEL, prompt=prompt_body, images=[buffer.tobytes()], think=False)
            
            self.stats["gen_time"] = resp['total_duration'] / 1e9
            eval_dur = resp.get('eval_duration', 1) / 1e9
            self.stats["tokens_per_sec"] = resp.get('eval_count', 0) / eval_dur
            
            self.state["observation"] = resp['response'].strip()
            self.stm.append(self.state["observation"])
            
            self.summary_counter += 1
            if self.summary_counter >= self.STM_N:
                self.summarize_to_ltm()
                self.summary_counter = 0
                self.ltm_counter = len(self.ltm) 

        except Exception as e: 
            print(f"Inference Error: {e}")
        finally:
            self.is_thinking = False