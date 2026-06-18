import os
import sys
import tkinter as tk
import threading
import time
import cv2
from PIL import Image, ImageTk

class HumanInterface:
    def __init__(self, root, callbacks): #gehele UI venster
        self.root, self.callbacks = root, callbacks
        # Geüpdatete venstergrootte voor de nieuwe lay-out
        self.root.geometry("1000x530")
        self.root.title("HMI Interface - OpenCV Video Display")
        self.root.resizable(False, False)

        #Standaard status van elke topic / service / variabele
        self.ui_start_stop_active = self.ui_reset_pressed = self.ui_training_inference_active = False
        self.snelheid_laatste = 5
        self.versnelling_laatste = 2

        # --- HOOFD-LAYOUT FRAMES (Links vs Rechts) ---
        self.left_panel = tk.Frame(self.root)
        self.left_panel.pack(side=tk.LEFT, padx=15, pady=10, fill=tk.Y)

        self.right_panel = tk.Frame(self.root)
        self.right_panel.pack(side=tk.RIGHT, padx=15, pady=10, fill=tk.BOTH, expand=True)

        # =========================================================================
        # LINKS: VIDEOBEELDEN (Onder elkaar geplaatst)
        # =========================================================================
        # 1. Video DepthAI
        self.frame_left = tk.Frame(self.left_panel, bg="black", width=320, height=240)
        self.frame_left.pack(pady=(0, 10))
        self.frame_left.pack_propagate(False) 

        self.camera_label = tk.Label(self.frame_left, bg="black")
        self.camera_label.pack(fill="both", expand=True)

        # 2. Video RViz
        self.frame_right = tk.Frame(self.left_panel, bg="green", width=320, height=240) 
        self.frame_right.pack()
        self.frame_right.pack_propagate(False)

        self.rviz_label = tk.Label(self.frame_right, bg="black")
        self.rviz_label.pack(fill="both", expand=True)

        # =========================================================================
        # RECHTS BOVEN: KNOPPEN CONTAINER
        # =========================================================================
        self.buttons_frame = tk.Frame(self.right_panel)
        self.buttons_frame.pack(fill=tk.X, pady=(0, 15))

        # Trainingknop (Blauw)
        self.training = tk.Button(self.buttons_frame, text="TRAINING", width=14, height=2, bg="blue", fg="white", command=self.toggle_ui_training_inference)
        self.training.pack(side=tk.LEFT, padx=5)

        # Container voor de wisselende actie-knoppen (ON/OFF/RESET vs RESET/RETRY)
        self.action_button_container = tk.Frame(self.buttons_frame)
        self.action_button_container.pack(side=tk.LEFT, padx=5)

        self.turn_on = tk.Button(self.action_button_container, text="ON", width=10, height=2, command=self.toggle_turn_on)
        self.turn_off = tk.Button(self.action_button_container, text="OFF", width=10, height=2, command=self.toggle_turn_off)
        self.reset = tk.Button(self.action_button_container, text="RESET", width=10, height=2, command=self.toggle_ui_reset, activebackground="yellow")
        self.retry = tk.Button(self.action_button_container, text="RETRY", width=10, height=2, bg="lightgray", command=self.trigger_ui_retry)

        # =========================================================================
        # RECHTS MIDDEN: SLIDERS CONTAINER
        # =========================================================================
        self.sliders_frame = tk.Frame(self.right_panel)
        self.sliders_frame.pack(fill=tk.X, pady=(0, 15))

        self.slider_snelheid = tk.Scale(self.sliders_frame, from_=0, to=10, orient=tk.HORIZONTAL, tickinterval=2, label=f"Snelheid (Laatste: {self.snelheid_laatste})", length=240)
        self.slider_snelheid.bind("<ButtonRelease-1>", self.update_snelheid)
        self.slider_snelheid.pack(side=tk.LEFT, padx=(5, 20))
        
        self.slider_versnelling = tk.Scale(self.sliders_frame, from_=0, to=10, orient=tk.HORIZONTAL, tickinterval=2, label=f"Versnelling (Laatste: {self.versnelling_laatste})", length=240)
        self.slider_versnelling.bind("<ButtonRelease-1>", self.update_versnelling)

        # =========================================================================
        # RECHTS ONDER: TERMINAL
        # =========================================================================
        self.log_frame = tk.Frame(self.right_panel)
        self.log_frame.pack(fill=tk.BOTH, expand=True)

        self.log = tk.Text(self.log_frame, height=7, state="disabled", bg="black", fg="white", insertbackground="white")  
        self.log.pack(fill=tk.BOTH, expand=True)

        # --- HARDWARE INITIALISATIE ---
        self.cap_rviz = cv2.VideoCapture("http://localhost:8080/stream?topic=/rviz/camera_image") 
        self.cap_rviz.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap_rviz.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Start de oneindige video- en dataloop
        self._video_loop()
        
        self.toggle_turn_off()

    def _video_loop(self):
        """ Haalt live frames op via OpenCV (Zowel Cam als RViz) """
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                h, w, _ = frame.shape
                cv2.circle(frame, (int(w/2), int(h/2)), 4, (0, 255, 0), -1)
                opencv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
                captured_image = Image.fromarray(opencv_image)
                photo_image = ImageTk.PhotoImage(image=captured_image)
                self.camera_label.photo_image = photo_image
                self.camera_label.configure(image=photo_image)

        if self.cap_rviz.isOpened():
            ret_rviz, frame_rviz = self.cap_rviz.read()
            if ret_rviz:
                rviz_image_rgb = cv2.cvtColor(frame_rviz, cv2.COLOR_BGR2RGBA)
                rviz_captured_image = Image.fromarray(rviz_image_rgb)
                rviz_photo_image = ImageTk.PhotoImage(image=rviz_captured_image)
                self.rviz_label.photo_image = rviz_photo_image
                self.rviz_label.configure(image=rviz_photo_image)

        self.root.after(15, self._video_loop)

    def close_hardware(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            print("[HMI] OpenCV Camera succesvol afgesloten.")

    def update_ui_log(self, bericht):
        self.root.after(0, lambda: [self.log.config(state="normal"), self.log.insert("end", bericht + "\n"), self.log.see("end"), self.log.config(state="disabled")])

    def _sla_huidige_snelheid_op(self):
        if self.ui_start_stop_active and self.slider_snelheid.get() > 0: self.snelheid_laatste = self.slider_snelheid.get()

    def _wissel_ui_modus(self, naar_training=False):
        """ Regelt welke knoppen zichtbaar zijn op basis van de modus """
        if naar_training:
            self.turn_on.pack_forget()
            self.turn_off.pack_forget()
            
            # Toon RESET én RETRY netjes naast elkaar tijdens training
            self.reset.pack(side=tk.LEFT, padx=(0, 5))
            self.retry.pack(side=tk.LEFT)
            
            self.slider_versnelling.pack(side=tk.LEFT, padx=5)
            self.slider_versnelling.config(state="normal")
        else:
            self.retry.pack_forget()
            self.reset.pack_forget()
            self.slider_versnelling.pack_forget()
            
            self.turn_on.pack(side=tk.LEFT, padx=(0, 5))
            self.turn_off.pack(side=tk.LEFT, padx=(0, 5))
            self.reset.pack(side=tk.LEFT) # Staat ook hier in de rij

    def trigger_ui_retry(self):
        self.update_ui_log("[HMI] Retry ingedrukt. Interface geblokkeerd voor 5s...")
        
        # 1. Schakel alle knoppen uit
        self.retry.config(state="disabled", bg="lightgray")
        self.reset.config(state="disabled", bg="lightgray")
        self.training.config(state="disabled")
        
        # 2. Schakel nu ook expliciet de sliders uit tijdens de lock
        self.slider_snelheid.config(state="disabled")
        self.slider_versnelling.config(state="disabled")
        
        self.root.update_idletasks()

        self.root.after(5000, lambda: [
            self.retry.config(state="normal", bg="lightgray"),
            self.reset.config(state="normal", bg="orange" if self.ui_training_inference_active else "lightgray"),
            self.training.config(state="normal"),
            # Zet de sliders weer terug naar actief na de timer
            self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {self.snelheid_laatste})"),
            self.slider_versnelling.config(state="normal", label=f"Versnelling (Actief: {self.versnelling_laatste})") if self.ui_training_inference_active else None,
            self.update_ui_log("[HMI] Interface weer beschikbaar.")
        ])
        if 'ui_retry' in self.callbacks: self.callbacks['ui_retry']()

    def toggle_ui_reset(self):
        self._sla_huidige_snelheid_op()
        was_in_training = self.ui_training_inference_active
        self.ui_reset_pressed, self.ui_start_stop_active = True, False
        
        # Wijzig eerst de layout-modus
        self._wissel_ui_modus(naar_training=was_in_training)

        self.slider_snelheid.set(0)
        
        # Schakel direct alle knoppen uit om dubbelklikken te voorkomen
        self.turn_on.config(state="disabled", bg="lightgray")
        self.turn_off.config(state="disabled", bg="lightgray")
        self.reset.config(state="disabled", bg="lightgray")
        self.retry.config(state="disabled", bg="lightgray")
        self.training.config(state="disabled")
        
        # Schakel beide sliders hard uit (dit blijft nu staan omdat de layout al is gewisseld)
        self.slider_snelheid.config(state="disabled", label=f"Snelheid (Laatste: {self.snelheid_laatste})")
        self.slider_versnelling.config(state="disabled", label=f"Versnelling (Laatste: {self.versnelling_laatste})")
        
        self.root.update_idletasks() 

        self.root.after(5000, lambda: self._herstel_na_reset(was_in_training))
        self.update_ui_log("[HMI] Reset aangevraagd. Interface geblokkeerd voor 5s...")
        if 'ui_reset' in self.callbacks: self.callbacks['ui_reset']()

    def _herstel_na_reset(self, was_in_training):
        self.ui_reset_pressed = False
        self.training.config(state="normal")
        self.retry.config(state="normal", bg="lightgray")
        
        if was_in_training:
            self.ui_training_inference_active = True
            self.reset.config(state="normal", bg="orange")
            # Heractiveer de sliders weer netjes voor de trainingmodus
            self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {self.snelheid_laatste})")
            self.slider_versnelling.config(state="normal", label=f"Versnelling (Actief: {self.versnelling_laatste})")
            self.slider_versnelling.set(self.versnelling_laatste)
            self.update_ui_log("[HMI] Reset voltooid. Training herstart. Interface weer beschikbaar.")
        else:
            self.turn_on.config(state="normal", bg="lightgray")
            self.turn_off.config(state="normal", bg="red", activebackground="pink")
            self.reset.config(state="normal", bg="lightgray")
            self.slider_snelheid.config(state="disabled", label=f"Snelheid (Laatste: {self.snelheid_laatste})")
            self.slider_versnelling.config(state="disabled", label=f"Versnelling (Laatste: {self.versnelling_laatste})")
            self.update_ui_log("[HMI] Reset voltooid. Systeem staat in veilige OFF-state. Interface weer beschikbaar.")
            
    def toggle_turn_on(self):
        self.ui_start_stop_active, self.ui_reset_pressed = True, False
        self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {self.snelheid_laatste})")  
        self.slider_snelheid.set(self.snelheid_laatste)
        self.turn_on.config(bg="green", activebackground="lightgreen"); self.turn_off.config(bg="lightgray")
        if self.reset['state'] == 'normal': self.reset.config(bg="lightgray")
        self.update_ui_log(f"[HMI] Systeem aangezet (ON). Snelheid: {self.snelheid_laatste}")
        if 'ui_start_stop' in self.callbacks: self.callbacks['ui_start_stop'](True)

    def toggle_turn_off(self):
        self._sla_huidige_snelheid_op()
        self.ui_start_stop_active = self.ui_reset_pressed = self.ui_training_inference_active = False
        self._wissel_ui_modus(naar_training=False)
        self.slider_snelheid.set(0)
        self.slider_snelheid.config(state="disabled", label=f"Snelheid (Laatste: {self.snelheid_laatste})")  
        self.slider_versnelling.config(state="disabled", label=f"Versnelling (Laatste: {self.versnelling_laatste})")
        self.turn_on.config(state="normal", bg="lightgray")
        self.turn_off.config(state="normal", bg="red", activebackground="pink")
        if self.reset['state'] == 'normal': self.reset.config(bg="lightgray")
        self.training.config(bg="blue", fg="white", state="normal")
        self.update_ui_log(f"[HMI] Systeem uitgezet (OFF).")
        if 'ui_start_stop' in self.callbacks: self.callbacks['ui_start_stop'](False)

    def toggle_ui_training_inference(self):
        self.ui_training_inference_active = not self.ui_training_inference_active
        self.ui_start_stop_active = self.ui_reset_pressed = False
        if self.ui_training_inference_active:
            self._wissel_ui_modus(naar_training=True)
            self.training.config(bg="blue", activebackground="purple")
            if self.reset['state'] == 'normal': self.reset.config(bg="orange")
            self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {self.snelheid_laatste})") 
            self.slider_versnelling.config(state="normal", label=f"Versnelling (Actief: {self.versnelling_laatste})"); self.slider_versnelling.set(self.versnelling_laatste)
            self.update_ui_log(f"[HMI] Training Modus Actief.")
            if 'ui_training_inference' in self.callbacks: self.callbacks['ui_training_inference'](True)
        else:
            self.versnelling_laatste = self.slider_versnelling.get()
            self.toggle_turn_off()
            if 'ui_training_inference' in self.callbacks: self.callbacks['ui_training_inference'](False)

    def _safe_callback(self, key, value):
        """Voert een callback veilig uit als deze gedefinieerd is."""
        callback = self.callbacks.get(key)
        if callback:
            callback(value)

    def update_snelheid(self, event):
        # 1. Validatie (Sliders negeren als alles onactief is)
        if not (self.ui_start_stop_active or self.ui_training_inference_active): return

        # 2. Status & UI Updates
        huidige_waarde = self.slider_snelheid.get()
        if huidige_waarde > 0: self.snelheid_laatste = huidige_waarde
        self.slider_snelheid.config(label=f"Snelheid (Actief: {self.snelheid_laatste})")
        
        # Print de waarde live naar de HMI Terminal
        self.update_ui_log(f"[HMI] Snelheid handmatig aangepast naar: {huidige_waarde}")
        
        # 3. Veilige externe actie
        self._safe_callback('snelheid', huidige_waarde)

    def update_versnelling(self, event):
        # 1. Validatie
        if not self.ui_training_inference_active: return 
        
        # 2. Status & UI Updates
        huidige_waarde = self.slider_versnelling.get()
        self.versnelling_laatste = huidige_waarde
        self.slider_versnelling.config(label=f"Versnelling (Actief: {self.versnelling_laatste})")
        
        # Print de waarde live naar de HMI Terminal
        self.update_ui_log(f"[HMI] Versnelling handmatig aangepast naar: {huidige_waarde}")
        
        # 3. Veilige externe actie
        self._safe_callback('versnelling', huidige_waarde)
# ===== RUN MODUS =====
if __name__ == '__main__':
    root = tk.Tk()
    dummy_callbacks = {
        'ui_start_stop': lambda status: print(f"[ROS2] /ui_start_stop -> {status}"),
        'ui_reset': lambda: print("[ROS2] /ui_reset Service Request"),
        'ui_training_inference': lambda status: print(f"[ROS2] /ui_training_inference -> {status}"),
        'ui_retry': lambda: print("[ROS2] /ui_retry Service Request"),
        'snelheid': lambda val: print(f"[ROS2] Parameter /snelheid -> {val}"),
        'versnelling': lambda val: print(f"[ROS2] Parameter /versnelling -> {val}")
    }
    ui = HumanInterface(root, dummy_callbacks)
    root.protocol("WM_DELETE_WINDOW", lambda: [ui.close_hardware(), root.destroy()])
    root.mainloop()