import tkinter as tk
from tkinter import ttk, scrolledtext
import hashlib
import threading
import time

class RangeSlider(tk.Canvas):
    """Custom dual-handle range slider"""
    def __init__(self, parent, min_val=0, max_val=10000000, start_val=0, end_val=10000000, **kwargs):
        super().__init__(parent, height=80, **kwargs)
        
        self.min_val = min_val
        self.max_val = max_val
        self.start_val = start_val
        self.end_val = end_val
        
        self.slider_width = 600
        self.slider_height = 10
        self.handle_radius = 10
        
        self.dragging = None
        
        self.draw_slider()
        self.bind("<Button-1>", self.on_click)
        self.bind("<B1-Motion>", self.on_drag)
        self.bind("<ButtonRelease-1>", self.on_release)
        
    def draw_slider(self):
        self.delete("all")
        
        x_start = 50
        x_end = x_start + self.slider_width
        y = 35
        
        # Draw track
        self.create_rectangle(x_start, y - self.slider_height//2, 
                             x_end, y + self.slider_height//2, 
                             fill="#ddd", outline="#999")
        
        # Calculate handle positions
        start_pos = x_start + (self.start_val - self.min_val) / (self.max_val - self.min_val) * self.slider_width
        end_pos = x_start + (self.end_val - self.min_val) / (self.max_val - self.min_val) * self.slider_width
        
        # Draw active range
        self.create_rectangle(start_pos, y - self.slider_height//2,
                             end_pos, y + self.slider_height//2,
                             fill="#4CAF50", outline="")
        
        # Draw start handle
        self.create_oval(start_pos - self.handle_radius, y - self.handle_radius,
                        start_pos + self.handle_radius, y + self.handle_radius,
                        fill="#2196F3", outline="#1976D2", width=2, tags="start_handle")
        
        # Draw end handle
        self.create_oval(end_pos - self.handle_radius, y - self.handle_radius,
                        end_pos + self.handle_radius, y + self.handle_radius,
                        fill="#f44336", outline="#d32f2f", width=2, tags="end_handle")
        
        # Draw labels with range info
        range_size = self.end_val - self.start_val
        self.create_text(start_pos, y + 25, text=f"{self.start_val:,}", font=("Arial", 9, "bold"), fill="#2196F3")
        self.create_text(end_pos, y + 25, text=f"{self.end_val:,}", font=("Arial", 9, "bold"), fill="#f44336")
        self.create_text((start_pos + end_pos) / 2, y - 20, 
                        text=f"Range: {range_size:,}", font=("Arial", 10, "bold"), fill="#4CAF50")
        
    def on_click(self, event):
        x_start = 50
        start_pos = x_start + (self.start_val - self.min_val) / (self.max_val - self.min_val) * self.slider_width
        end_pos = x_start + (self.end_val - self.min_val) / (self.max_val - self.min_val) * self.slider_width
        
        if abs(event.x - start_pos) < self.handle_radius * 2:
            self.dragging = 'start'
        elif abs(event.x - end_pos) < self.handle_radius * 2:
            self.dragging = 'end'
    
    def on_drag(self, event):
        if self.dragging:
            x_start = 50
            x_end = x_start + self.slider_width
            x = max(x_start, min(event.x, x_end))
            
            ratio = (x - x_start) / self.slider_width
            new_val = int(self.min_val + ratio * (self.max_val - self.min_val))
            
            if self.dragging == 'start':
                self.start_val = min(new_val, self.end_val - 1)
            elif self.dragging == 'end':
                self.end_val = max(new_val, self.start_val + 1)
            
            self.draw_slider()
    
    def on_release(self, event):
        self.dragging = None
    
    def get_values(self):
        return self.start_val, self.end_val
    
    def set_values(self, start, end):
        self.start_val = max(self.min_val, start)
        self.end_val = min(self.max_val, end)
        self.draw_slider()

class SHA256MinerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SHA-256 Miner - Zoom Range Control")
        self.root.geometry("700x700")
        
        self.mining = False
        self.hashes_computed = 0
        self.blocks_found = 0
        self.start_time = None
        self.mining_thread = None
        
        # Zoom levels (precision increments)
        self.zoom_levels = [1, 10, 100, 1000, 10000, 100000, 1000000]
        self.current_zoom_index = 3  # Start at 1000
        
        self.create_widgets()
        
    def create_widgets(self):
        # Title
        title = tk.Label(self.root, text="SHA-256 Miner", font=("Arial", 20, "bold"))
        title.pack(pady=10)
        
        # Configuration Frame
        config_frame = tk.LabelFrame(self.root, text="Configuration", padx=10, pady=10)
        config_frame.pack(fill="x", padx=10, pady=5)
        
        tk.Label(config_frame, text="Data/Prefix:").grid(row=0, column=0, sticky="w", pady=5)
        self.prefix_entry = tk.Entry(config_frame, width=30)
        self.prefix_entry.insert(0, "test")
        self.prefix_entry.grid(row=0, column=1, pady=5)
        
        tk.Label(config_frame, text="Difficulty (Leading Zeros):").grid(row=1, column=0, sticky="w", pady=5)
        self.difficulty_var = tk.IntVar(value=4)
        self.difficulty_spinbox = tk.Spinbox(config_frame, from_=1, to=10, textvariable=self.difficulty_var, width=10)
        self.difficulty_spinbox.grid(row=1, column=1, sticky="w", pady=5)
        
        tk.Label(config_frame, text="Batch Size:").grid(row=2, column=0, sticky="w", pady=5)
        self.batch_var = tk.IntVar(value=1000)
        self.batch_spinbox = tk.Spinbox(config_frame, from_=100, to=100000, increment=1000, textvariable=self.batch_var, width=10)
        self.batch_spinbox.grid(row=2, column=1, sticky="w", pady=5)
        
        # Range Slider Frame
        slider_frame = tk.LabelFrame(self.root, text="Nonce Range", padx=10, pady=10)
        slider_frame.pack(fill="x", padx=10, pady=5)
        
        self.range_slider = RangeSlider(slider_frame, min_val=0, max_val=100000000000000, 
                                       start_val=0, end_val=100000000000000, width=660)
        self.range_slider.pack()
        
        # Zoom Control Frame
        zoom_frame = tk.Frame(slider_frame, bg="#f5f5f5", relief="raised", bd=2)
        zoom_frame.pack(pady=10, fill="x")
        
        # Zoom Level Display
        self.zoom_label = tk.Label(zoom_frame, text=f"Precision: ±{self.zoom_levels[self.current_zoom_index]:,}", 
                                   font=("Arial", 11, "bold"), bg="#f5f5f5")
        self.zoom_label.pack(pady=5)
        
        # Single Zoom Control Button Row
        zoom_buttons = tk.Frame(zoom_frame, bg="#f5f5f5")
        zoom_buttons.pack(pady=5)
        
        # Zoom Out (expand range, decrease precision)
        tk.Button(zoom_buttons, text="🔍−  Zoom Out", command=self.zoom_out,
                 bg="#FF9800", fg="white", font=("Arial", 11, "bold"), width=15,
                 relief="raised", bd=3).pack(side="left", padx=5)
        
        # Expand Selection (increase range by current precision)
        tk.Button(zoom_buttons, text="⬌ Expand Range", command=self.expand_range,
                 bg="#4CAF50", fg="white", font=("Arial", 11, "bold"), width=15,
                 relief="raised", bd=3).pack(side="left", padx=5)
        
        # Zoom In (narrow range, increase precision)
        tk.Button(zoom_buttons, text="🔍+  Zoom In", command=self.zoom_in,
                 bg="#2196F3", fg="white", font=("Arial", 11, "bold"), width=15,
                 relief="raised", bd=3).pack(side="left", padx=5)
        
        # Control Buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)
        
        self.start_btn = tk.Button(button_frame, text="Start Mining", command=self.start_mining, 
                                   bg="#4CAF50", fg="white", font=("Arial", 12, "bold"), width=15)
        self.start_btn.grid(row=0, column=0, padx=5)
        
        self.stop_btn = tk.Button(button_frame, text="Stop Mining", command=self.stop_mining, 
                                  bg="#f44336", fg="white", font=("Arial", 12, "bold"), width=15, state="disabled")
        self.stop_btn.grid(row=0, column=1, padx=5)
        
        # Statistics Frame
        stats_frame = tk.LabelFrame(self.root, text="Statistics", padx=10, pady=10)
        stats_frame.pack(fill="x", padx=10, pady=5)
        
        tk.Label(stats_frame, text="Hashes Computed:").grid(row=0, column=0, sticky="w")
        self.hash_label = tk.Label(stats_frame, text="0", font=("Arial", 12, "bold"))
        self.hash_label.grid(row=0, column=1, sticky="w")
        
        tk.Label(stats_frame, text="Blocks Found:").grid(row=1, column=0, sticky="w")
        self.blocks_label = tk.Label(stats_frame, text="0", font=("Arial", 12, "bold"))
        self.blocks_label.grid(row=1, column=1, sticky="w")
        
        tk.Label(stats_frame, text="Hash Rate:").grid(row=2, column=0, sticky="w")
        self.hashrate_label = tk.Label(stats_frame, text="0 H/s", font=("Arial", 12, "bold"))
        self.hashrate_label.grid(row=2, column=1, sticky="w")
        
        tk.Label(stats_frame, text="Current Nonce:").grid(row=3, column=0, sticky="w")
        self.nonce_label = tk.Label(stats_frame, text="0", font=("Arial", 12, "bold"))
        self.nonce_label.grid(row=3, column=1, sticky="w")
        
        # Progress Bar
        self.progress_var = tk.DoubleVar()
        self.progress = ttk.Progressbar(self.root, variable=self.progress_var, maximum=100, length=660)
        self.progress.pack(padx=10, pady=5)
        
        # Output Log
        log_frame = tk.LabelFrame(self.root, text="Mining Log", padx=10, pady=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=6, state="disabled", wrap="word")
        self.log_text.pack(fill="both", expand=True)
    
    def zoom_in(self):
        """Increase precision (smaller steps) and narrow range"""
        if self.current_zoom_index > 0:
            self.current_zoom_index -= 1
            step = self.zoom_levels[self.current_zoom_index]
            
            # Narrow the range by 50% from center
            start, end = self.range_slider.get_values()
            center = (start + end) // 2
            half_range = (end - start) // 4
            
            new_start = max(0, center - half_range)
            new_end = min(10000000, center + half_range)
            
            self.range_slider.set_values(new_start, new_end)
            self.zoom_label.config(text=f"Precision: ±{step:,}")
    
    def zoom_out(self):
        """Decrease precision (larger steps) and widen range"""
        if self.current_zoom_index < len(self.zoom_levels) - 1:
            self.current_zoom_index += 1
            step = self.zoom_levels[self.current_zoom_index]
            
            # Widen the range by 100% from center
            start, end = self.range_slider.get_values()
            center = (start + end) // 2
            half_range = (end - start)
            
            new_start = max(0, center - half_range)
            new_end = min(10000000, center + half_range)
            
            self.range_slider.set_values(new_start, new_end)
            self.zoom_label.config(text=f"Precision: ±{step:,}")
    
    def expand_range(self):
        """Expand selection outward by current precision level"""
        step = self.zoom_levels[self.current_zoom_index]
        start, end = self.range_slider.get_values()
        
        new_start = max(0, start - step)
        new_end = min(10000000, end + step)
        
        self.range_slider.set_values(new_start, new_end)
        
    def log(self, message):
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state="disabled")
        
    def start_mining(self):
        try:
            prefix = self.prefix_entry.get()
            difficulty = self.difficulty_var.get()
            start_nonce, end_nonce = self.range_slider.get_values()
            batch_size = self.batch_var.get()
            
            self.mining = True
            self.hashes_computed = 0
            self.blocks_found = 0
            self.start_time = time.time()
            
            self.start_btn.config(state="disabled")
            self.stop_btn.config(state="normal")
            
            self.log(f"[START] Mining '{prefix}', difficulty {difficulty}")
            self.log(f"[RANGE] {start_nonce:,} to {end_nonce:,} ({end_nonce - start_nonce:,})")
            
            self.mining_thread = threading.Thread(
                target=self.mine, 
                args=(prefix, difficulty, start_nonce, end_nonce, batch_size),
                daemon=True
            )
            self.mining_thread.start()
            
        except Exception as e:
            self.log(f"[ERROR] {e}")
    
    def stop_mining(self):
        self.mining = False
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.log("[STOP] Mining stopped.")
        
    def mine(self, prefix, difficulty, start_nonce, end_nonce, batch_size):
        target_prefix = "0" * difficulty
        total_range = end_nonce - start_nonce
        nonce = start_nonce
        
        while self.mining and nonce < end_nonce:
            for i in range(batch_size):
                if not self.mining or nonce >= end_nonce:
                    break
                    
                data = f"{prefix}{nonce}"
                hash_result = hashlib.sha256(data.encode()).hexdigest()
                self.hashes_computed += 1
                
                if hash_result.startswith(target_prefix):
                    self.blocks_found += 1
                    self.log(f"\n*** BLOCK FOUND ***")
                    self.log(f"Nonce: {nonce:,} | Hash: {hash_result}\n")
                    
                nonce += 1
            
            elapsed = time.time() - self.start_time
            hashrate = self.hashes_computed / elapsed if elapsed > 0 else 0
            progress = ((nonce - start_nonce) / total_range) * 100
            
            self.root.after(0, self.update_stats, nonce, hashrate, progress)
            
        if nonce >= end_nonce:
            self.log(f"[COMPLETE] {self.blocks_found} blocks found.")
            self.stop_mining()
    
    def update_stats(self, nonce, hashrate, progress):
        self.hash_label.config(text=f"{self.hashes_computed:,}")
        self.blocks_label.config(text=f"{self.blocks_found}")
        self.hashrate_label.config(text=f"{hashrate:,.0f} H/s")
        self.nonce_label.config(text=f"{nonce:,}")
        self.progress_var.set(progress)

if __name__ == "__main__":
    root = tk.Tk()
    app = SHA256MinerGUI(root)
    root.mainloop()
