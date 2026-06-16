import tkinter as tk

class HumanInterface:
    def __init__(self, root, callbacks):
        """
        callbacks: een dictionary met functies uit de controller:
        {
            'on': self.ros_turn_on,
            'off': self.ros_turn_off,
            'reset': self.ros_reset,
            'speed': self.ros_speed
        }
        """
        self.root = root
        self.callbacks = callbacks
        
        # Venster volledig HMI
        self.root.geometry("500x350")
        self.root.title("HMI Interface")
        self.root.resizable(False, False)

        # Status variabelen 
        self.turn_on_pressed = False
        self.turn_off_pressed = True 
        self.reset_pressed = False
        self.training_pressed = False

        # Het geheugen voor de snelheid (startwaarde is 5)
        self.laatste_snelheid = 5

        # --- UI LAYOUT ---
        # Videovenster links (RVIZ)
        self.frame_left = tk.Frame(self.root, bg="lightblue", width=100, height=200)
        self.frame_left.pack(padx=10, pady=10, side=tk.LEFT, anchor="s")

        # Videovenster rechts (AI camera/vision)
        self.frame_right = tk.Frame(self.root, bg="green", width=100, height=200)
        self.frame_right.pack(padx=10, pady=10, side=tk.LEFT, anchor="s")

        # Knoppen aanmaken
        self.turn_on = tk.Button(self.root, text="ON", width=6, command=self.toggle_turn_on)
        self.turn_off = tk.Button(self.root, text="OFF", width=6, command=self.toggle_turn_off)
        self.reset = tk.Button(self.root, text="RESET", width=6, command=self.toggle_reset)
        self.training = tk.Button(self.root, text="TRAINING", width=6, bg="blue", activebackground="lightblue", fg="white", command=self.toggle_training)

        # Snelheid slider
        self.slider = tk.Scale(self.root, from_=0, to=10, orient=tk.HORIZONTAL, tickinterval=2, label="Snelheid")
        self.slider.bind("<ButtonRelease-1>", self.toon_snelheid)

        # Terminal / Logvenster
        self.log = tk.Text(self.root, height=10, width=60, state="disabled")  
        self.log.config(bg="black", fg="white", insertbackground="white")
        self.log.pack(padx=10, pady=10, expand=True, side="right", anchor="s")

        # Exacte positie van de componenten (Layout op de UI)
        self.turn_on.place(x=20, y=20)
        self.turn_off.place(x=80, y=20)   
        self.reset.place(x=140, y=20)     
        self.slider.place(x=220, y=5)
        self.training.place(x=100, y=60)  

        #startstatus van de gehele robotcel
        self.toggle_turn_off()

    # --- LOGGING FUNCTIE ---
    def update_ui_log(self, bericht):
        self.log.config(state="normal")       
        self.log.insert("end", bericht + "\n") 
        self.log.see("end")                 
        self.log.config(state="disabled")    

    # --- KNOP LOGICA ---
    def toggle_turn_on(self):
        self.turn_on_pressed = True
        self.turn_off_pressed = False
        self.reset_pressed = False
        
        # Activeer slider en herstel de opgeslagen snelheid
        self.slider.config(state="normal")  
        self.slider.set(self.laatste_snelheid)
        
        # Pas knoppenstyling aan
        self.turn_on.config(bg="green", activebackground="lightgreen")
        self.turn_off.config(bg="lightgray")
        self.reset.config(bg="lightgray")

        self.update_ui_log(f"[INFO] Systeem actief. Snelheid hersteld naar: {self.laatste_snelheid}")
        
        # Trigger ROS2 Controller
        if 'on' in self.callbacks:
            self.callbacks['on']()

    def toggle_turn_off(self):
        # Sla de snelheid alleen op als het systeem hiervoor daadwerkelijk AAN stond.
        if self.turn_on_pressed:
            huidige_waarde = self.slider.get()
            if huidige_waarde > 0:
                self.laatste_snelheid = huidige_waarde

        self.turn_off_pressed = True
        self.turn_on_pressed = False
        self.reset_pressed = False
        
        # Reset de slider visueel naar 0 en zet hem op slot
        self.slider.set(0)
        self.slider.config(state="disabled")  

        # Pas knoppenstyling aan
        self.turn_off.config(bg="red", activebackground="pink")
        self.turn_on.config(bg="lightgray")
        self.reset.config(bg="lightgray")

        self.update_ui_log(f"[INFO] Systeem stand-by (OFF). (Onthouden snelheid: {self.laatste_snelheid})")
        
        # Trigger ROS2 Controller (alleen als de controller al gekoppeld is)
        if 'off' in self.callbacks:
            self.callbacks['off']()

    def toggle_reset(self):
        # Sla alleen de snelheid op als het systeem daadwerkelijk AAN stond
        if self.turn_on_pressed:
            huidige_waarde = self.slider.get()
            if huidige_waarde > 0:
                self.laatste_snelheid = huidige_waarde

        self.reset_pressed = True
        self.turn_on_pressed = False
        self.turn_off_pressed = False
        
        # Reset de slider visueel en zet hem op slot
        self.slider.set(0)
        self.slider.config(state="disabled") 

        # Pas knoppenstyling aan
        self.reset.config(bg="orange", activebackground="yellow")
        self.turn_on.config(bg="lightgray")
        self.turn_off.config(bg="lightgray")
        
        self.update_ui_log(f"[STATUS] Systeem gereset. (Onthouden snelheid: {self.laatste_snelheid})")
        
        # Trigger ROS2 Controller
        if 'reset' in self.callbacks:
            self.callbacks['reset']()

    def toon_snelheid(self, event):
        # Als het systeem uit staat, mag de slider niks doen of loggen
        if self.turn_off_pressed:
            return
            
        eindwaarde = self.slider.get()  
        
        # Sla de nieuw gekozen snelheid direct op in het geheugen
        if eindwaarde > 0:
            self.laatste_snelheid = eindwaarde

        self.update_ui_log(f"[INPUT] Snelheid aangepast naar: {eindwaarde}")
        
        # Trigger ROS2 Controller en geef de waarde mee
        if 'speed' in self.callbacks:
            self.callbacks['speed'](eindwaarde)

# training UI

    def toggle_training(self):
        self.reset_pressed = False
        self.turn_on_pressed = False
        self.turn_off_pressed = False
        self.training_pressed = True

        self.reset.config(bg="lightgray")
        self.turn_on.config(bg="lightgray")
        self.turn_off.config(bg="lightgray")
      
        self.slider.config(state="normal")  
        self.slider.set(0)


# ===== STANDALONE TEST MODUS =====
if __name__ == '__main__':
    root = tk.Tk()
    
    # Dummy callbacks om te testen zonder ROS2
    dummy_callbacks = {
        'on': lambda: print("[TEST-ROS2] Bericht verzonden: TURN_ON"),
        'off': lambda: print("[TEST-ROS2] Bericht verzonden: TURN_OFF"),
        'reset': lambda: print("[TEST-ROS2] Bericht verzonden: RESET"),
        'speed': lambda val: print(f"[TEST-ROS2] Bericht verzonden: SPEED naar {val}")
    }
    
    ui = HumanInterface(root, dummy_callbacks)
    root.mainloop()

