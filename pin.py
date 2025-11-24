import tkinter as tk
from tkinter import ttk, messagebox
import random

class PINGuessingGame:
    def __init__(self, root):
        self.root = root
        self.root.title("PIN Guessing Game")
        self.root.geometry("500x400")
        
        # Generate random 4-digit PIN
        self.secret_pin = [random.randint(0, 9) for _ in range(4)]
        #print(f"Secret PIN (for debugging): {''.join(map(str, self.secret_pin))}")
        
        # Track attempts
        self.attempts = 0
        
        # Create UI
        self.create_widgets()
        
    def create_widgets(self):
        # Title
        title = tk.Label(self.root, text="PIN Guessing Game", font=("Arial", 20, "bold"))
        title.pack(pady=20)
        
        # Instructions
        instructions = tk.Label(self.root, text="Guess the 4-digit PIN. Each digit is verified independently!", 
                               font=("Arial", 11))
        instructions.pack(pady=10)
        
        # Frame for dropdowns
        dropdown_frame = tk.Frame(self.root)
        dropdown_frame.pack(pady=20)
        
        # Create 4 dropdown boxes
        self.dropdowns = []
        self.status_labels = []
        
        for i in range(4):
            digit_frame = tk.Frame(dropdown_frame)
            digit_frame.grid(row=0, column=i, padx=10)
            
            label = tk.Label(digit_frame, text=f"Digit {i+1}", font=("Arial", 10))
            label.pack()
            
            dropdown = ttk.Combobox(digit_frame, values=list(range(10)), 
                                   state="readonly", width=5, font=("Arial", 14))
            dropdown.set("?")
            dropdown.pack(pady=5)
            dropdown.bind("<<ComboboxSelected>>", lambda e, idx=i: self.verify_digit(idx))
            self.dropdowns.append(dropdown)
            
            # Status label (shows ✓ or ✗)
            status = tk.Label(digit_frame, text="", font=("Arial", 16))
            status.pack()
            self.status_labels.append(status)
        
        # Attempts counter
        self.attempts_label = tk.Label(self.root, text="Attempts: 0", font=("Arial", 12))
        self.attempts_label.pack(pady=10)
        
        # Check all button
        check_button = tk.Button(self.root, text="Check Complete PIN", 
                                command=self.check_complete_pin,
                                font=("Arial", 12), bg="#4CAF50", fg="white",
                                padx=20, pady=10)
        check_button.pack(pady=10)
        
        # Reset button
        reset_button = tk.Button(self.root, text="New Game", 
                                command=self.reset_game,
                                font=("Arial", 10), bg="#2196F3", fg="white",
                                padx=15, pady=5)
        reset_button.pack(pady=5)
        
    def verify_digit(self, index):
        """Verify individual digit when selected"""
        selected = self.dropdowns[index].get()
        
        if selected == "?":
            return
            
        selected_digit = int(selected)
        
        if selected_digit == self.secret_pin[index]:
            self.status_labels[index].config(text="✓", fg="green")
        else:
            self.status_labels[index].config(text="✗", fg="red")
    
    def check_complete_pin(self):
        """Check if all digits are correct"""
        self.attempts += 1
        self.attempts_label.config(text=f"Attempts: {self.attempts}")
        
        # Verify all digits first
        for i in range(4):
            self.verify_digit(i)
        
        # Check if all are correct
        all_correct = True
        for i in range(4):
            selected = self.dropdowns[i].get()
            if selected == "?" or int(selected) != self.secret_pin[i]:
                all_correct = False
                break
        
        if all_correct:
            messagebox.showinfo("Success!", 
                              f"🎉 Congratulations! You cracked the PIN in {self.attempts} attempts!\n\n" +
                              f"The PIN was: {''.join(map(str, self.secret_pin))}")
        else:
            incorrect_count = sum(1 for i in range(4) 
                                if self.dropdowns[i].get() == "?" or 
                                int(self.dropdowns[i].get()) != self.secret_pin[i])
            messagebox.showinfo("Try Again", 
                              f"Not quite! {incorrect_count} digit(s) still incorrect.\n" +
                              "Keep trying!")
    
    def reset_game(self):
        """Start a new game"""
        self.secret_pin = [random.randint(0, 9) for _ in range(4)]
        print(f"New Secret PIN (for debugging): {''.join(map(str, self.secret_pin))}")
        self.attempts = 0
        self.attempts_label.config(text="Attempts: 0")
        
        for i in range(4):
            self.dropdowns[i].set("?")
            self.status_labels[i].config(text="")

# Create and run the game
root = tk.Tk()
game = PINGuessingGame(root)
root.mainloop()