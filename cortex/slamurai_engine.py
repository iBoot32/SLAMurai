import ollama
import cv2
from collections import deque
from datetime import datetime

VLM_MODEL = "SLAMurai" 

class SLAMuraiEngine:
    def __init__(self):
        self.STM_N = 8
        self.LTM_M = 15
        self.NAVLOG_O = 10
        self.stm = deque(maxlen=self.STM_N)
        self.ltm = deque(maxlen=self.LTM_M)
        self.summary_counter = 0
        self.ltm_counter = 0
        
        self.stats = {"gen_time": 0.0, "tokens_per_sec": 0.0, "total_tokens": 0}
        self.state = {
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}, 
            "nav_logs": deque(maxlen=self.NAVLOG_O),
            "observation": "Initializing...", 
            "last_frame": None,
            "map_bounds": None
        }
        self.is_thinking = False

    def summarize_to_ltm(self):
        """Distills current STM into a single LTM entry to maintain long-term context."""
        stm_context = "\n".join(list(self.stm))
        prompt = (
            f"The current time is {datetime.now().strftime('%H:%M:%S')}. "
            f"TASK: Distill the following STM logs into a single 'Key Object Index' for LTM.\n"
            f"Focus on permanent architecture and furniture. Ignore transient objects.\n"
            f"Format: [time window] | Key: [ALL objects WITH THEIR POSE] | Env: [room types WITH THE RESPECTIVE x, y, yaw pose we saw them]\n"
            f"LOGS:\n{stm_context}"
        )
        try:
            response = ollama.generate(model=VLM_MODEL, prompt=prompt, think=False)
            self.ltm.append(response['response'].strip())
        except Exception as e:
            print(f"Summarizer Error: {e}")

    def think(self):
        """Structured observation loop using the rigid 'Observer' prompt."""
        if self.state["last_frame"] is None: 
            return
        
        self.is_thinking = True
        # Resize the composite image for inference
        vlm_input = cv2.resize(self.state["last_frame"], (1000, 430)) 
        _, buffer = cv2.imencode('.jpg', vlm_input, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        
        nav_context = "\n".join(list(self.state["nav_logs"]))
        
        prompt_body = (
            f"STRICT RULE: YOU ARE A REAL-TIME OBSERVER. THE IMAGE IS TRUTH AND WHAT YOU SHOULD PAY ATTENTION TO.\n"
            f"Memory tags (<ltm_archive> and <stm_log>) are for HISTORY AND RECALL ONLY and SHOULD NOT INFLUENCE YOUR CURRENT THOUGHTS.\n"
            f"Never assume the environment matches your memory.\n\n"
            f"### THE PAST (MEMORY)\n"
            f"<ltm_archive>\n{chr(10).join(list(self.ltm))}\n</ltm_archive>\n"
            f"<stm_log>\n{chr(10).join(list(self.stm))}\n</stm_log>\n\n"
            f"### SYSTEM LOGS (NAV2/ROS2)\n"
            f"{nav_context}\n\n"
            f"### THE PRESENT (ABSOLUTE TRUTH for current analysis)\n"
            f"Pose: {self.state['pose']}\n"
            f"Map Bounds: {self.state['map_bounds']}\n"
            f"Time: {datetime.now().strftime('%H:%M:%S')}\n\n"
            f"OUTPUT FORMAT (MANDATORY):\n"
            f"[Current time] Pose [X, Y, Yaw]\n"
            f"Objects (be info-dense): [object1(dist), object2(dist)...]\n"
            f"Scene: [1-2 sentences describing the layout and MANDATORY guess of room type.]\n\n"
            f"Example:\n"
            f"[x:x:x] Pose [x, y, yaw]\n"
            f"Objects: a(1.0m), b(0.5m), ...\n"
            f"Scene: A <> with a and b, likely a <room type>."
        )
        
        try:
            resp = ollama.generate(model=VLM_MODEL, prompt=prompt_body, images=[buffer.tobytes()], think=False)
            
            self.stats["gen_time"] = resp['total_duration'] / 1e9
            eval_dur = resp.get('eval_duration', 1) / 1e9
            self.stats["tokens_per_sec"] = resp.get('eval_count', 0) / eval_dur
            
            self.state["observation"] = resp['response'].strip()
            # Store formatted observation in STM
            self.stm.append(self.state["observation"])
            
            self.summary_counter += 1
            if self.summary_counter >= self.STM_N:
                self.summarize_to_ltm()
                self.summary_counter = 0

        except Exception as e: 
            print(f"Inference Error: {e}")
        finally:
            self.is_thinking = False

    def chat(self, user_query):
        """Simple chat mode. Bare instructions to allow natural responses."""
        if self.state["last_frame"] is None:
            return "Vision system not ready."

        self.is_thinking = True
        vlm_input = cv2.resize(self.state["last_frame"], (1000, 430))
        _, buffer = cv2.imencode('.jpg', vlm_input, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

        nav_context = "\n".join(list(self.state["nav_logs"]))

        prompt_body = (
            f"Current Time: {datetime.now().strftime('%H:%M:%S')}\n"
            f"Current Pose: {self.state['pose']}\n"
            f"Map Bounds: {self.state['map_bounds']}\n\n"
            f"### SYSTEM LOGS (NAV2/ROS2)\n"
            f"{nav_context}\n\n"
            f"### MEMORY LOGS\n"
            f"LTM Archive:\n{chr(10).join(list(self.ltm))}\n\n"
            f"STM Log (Recent):\n{chr(10).join(list(self.stm))}\n\n"
            f"### INSTRUCTION\n"
            f"Answer the user query naturally and CONCISELY. Use current vision and the memory logs provided. If the user is asking something like 'most recent' or 'oldest' make sure to check all the times."
            f"If you saw something in the logs that is no longer in the image, explain that.\n\n"
            f"USER QUERY: {user_query}"
        )

        try:
            resp = ollama.generate(model=VLM_MODEL, prompt=prompt_body, images=[buffer.tobytes()], think=False)
            answer = resp['response'].strip()
            
            # Record the chat interaction in STM so the robot remembers the conversation
            self.stm.append(f"[{datetime.now().strftime('%H:%M:%S')}] CHAT: User asked '{user_query}'. AI replied: '{answer}'")
            return answer
        except Exception as e:
            return f"Chat Error: {e}"
        finally:
            self.is_thinking = False