import tkinter as tk
from tkinter import ttk, scrolledtext
import hashlib
import threading
import time

class SHA256MinerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SHA-256 Miner")
        self.root.geometry("700x600")
        
        # Mining state
        self.mining = False
        self.hashes_computed = 0
        self.blocks_found = 0
        self.start_time = None
        self.mining_thread = None
        
        # Create GUI
        self.create_widgets()
        
    def create_widgets(self):
        # Title
        title = tk.Label(self.root, text="SHA-256 Miner", font=("Arial", 20, "bold"))
        title.pack(pady=10)
        
        # Configuration Frame
        config_frame = tk.LabelFrame(self.root, text="Configuration", padx=10, pady=10)
        config_frame.pack(fill="x", padx=10, pady=5)
        
        # Prefix/Data Input
        tk.Label(config_frame, text="Data/Prefix:").grid(row=0, column=0, sticky="w", pady=5)
        self.prefix_entry = tk.Entry(config_frame, width=30)
        self.prefix_entry.insert(0, "test")
        self.prefix_entry.grid(row=0, column=1, pady=5)
        
        # Difficulty
        tk.Label(config_frame, text="Difficulty (Leading Zeros):").grid(row=1, column=0, sticky="w", pady=5)
        self.difficulty_var = tk.IntVar(value=4)
        self.difficulty_spinbox = tk.Spinbox(config_frame, from_=1, to=10, textvariable=self.difficulty_var, width=10)
        self.difficulty_spinbox.grid(row=1, column=1, sticky="w", pady=5)
        
        # Start Nonce
        tk.Label(config_frame, text="Start Nonce:").grid(row=2, column=0, sticky="w", pady=5)
        self.start_nonce_entry = tk.Entry(config_frame, width=30)
        self.start_nonce_entry.insert(0, "0")
        self.start_nonce_entry.grid(row=2, column=1, pady=5)
        
        # End Nonce
        tk.Label(config_frame, text="End Nonce:").grid(row=3, column=0, sticky="w", pady=5)
        self.end_nonce_entry = tk.Entry(config_frame, width=30)
        self.end_nonce_entry.insert(0, "10000000")
        self.end_nonce_entry.grid(row=3, column=1, pady=5)
        
        # Batch Size
        tk.Label(config_frame, text="Batch Size (Hashes/Update):").grid(row=4, column=0, sticky="w", pady=5)
        self.batch_var = tk.IntVar(value=1000)
        self.batch_spinbox = tk.Spinbox(config_frame, from_=100, to=100000, increment=1000, textvariable=self.batch_var, width=10)
        self.batch_spinbox.grid(row=4, column=1, sticky="w", pady=5)
        
        # Control Buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)
        
        self.start_btn = tk.Button(button_frame, text="Start Mining", command=self.start_mining, bg="#4CAF50", fg="white", font=("Arial", 12, "bold"), width=15)
        self.start_btn.grid(row=0, column=0, padx=5)
        
        self.stop_btn = tk.Button(button_frame, text="Stop Mining", command=self.stop_mining, bg="#f44336", fg="white", font=("Arial", 12, "bold"), width=15, state="disabled")
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
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, state="disabled", wrap="word")
        self.log_text.pack(fill="both", expand=True)
        
    def log(self, message):
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state="disabled")
        
    def start_mining(self):
        try:
            prefix = self.prefix_entry.get()
            difficulty = self.difficulty_var.get()
            start_nonce = int(self.start_nonce_entry.get())
            end_nonce = int(self.end_nonce_entry.get())
            batch_size = self.batch_var.get()
            
            if start_nonce >= end_nonce:
                self.log("[ERROR] Start nonce must be less than end nonce!")
                return
                
            self.mining = True
            self.hashes_computed = 0
            self.blocks_found = 0
            self.start_time = time.time()
            
            self.start_btn.config(state="disabled")
            self.stop_btn.config(state="normal")
            
            self.log(f"[START] Mining with prefix '{prefix}', difficulty {difficulty}")
            self.log(f"[RANGE] Nonce: {start_nonce} to {end_nonce} (Range: {end_nonce - start_nonce:,})")
            
            # Start mining thread
            self.mining_thread = threading.Thread(
                target=self.mine, 
                args=(prefix, difficulty, start_nonce, end_nonce, batch_size),
                daemon=True
            )
            self.mining_thread.start()
            
        except ValueError as e:
            self.log(f"[ERROR] Invalid input: {e}")
    
    def stop_mining(self):
        self.mining = False
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.log("[STOP] Mining stopped by user.")
        
    def mine(self, prefix, difficulty, start_nonce, end_nonce, batch_size):
        target_prefix = "0" * difficulty
        total_range = end_nonce - start_nonce
        
        nonce = start_nonce
        
        while self.mining and nonce < end_nonce:
            batch_start = time.time()
            
            for i in range(batch_size):
                if not self.mining or nonce >= end_nonce:
                    break
                    
                data = f"{prefix}{nonce}"
                hash_result = hashlib.sha256(data.encode()).hexdigest()
                
                self.hashes_computed += 1
                
                if hash_result.startswith(target_prefix):
                    self.blocks_found += 1
                    self.log(f"\n*** BLOCK FOUND ***")
                    self.log(f"Nonce: {nonce}")
                    self.log(f"Hash: {hash_result}")
                    self.log(f"Data: {data}\n")
                    
                nonce += 1
            
            # Update GUI
            elapsed = time.time() - self.start_time
            hashrate = self.hashes_computed / elapsed if elapsed > 0 else 0
            progress = ((nonce - start_nonce) / total_range) * 100
            
            self.root.after(0, self.update_stats, nonce, hashrate, progress)
            
        if nonce >= end_nonce:
            self.log(f"[COMPLETE] Searched entire range. {self.blocks_found} blocks found.")
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
