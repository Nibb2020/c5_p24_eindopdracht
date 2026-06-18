import os
import sys
import tkinter as tk
import threading
import time
import cv2
from PIL import Image, ImageTk

class HumanInterface:
    def __init__(self, root, callbacks):
        self.root, self.callbacks = root, callbacks
        # Venstergrootte proportioneel vergroot naar 1000x530 om grotere 320x240 video's te accommoderen
        self.root.geometry("1000x530") 
        self.root.title("HMI Interface - Custom Industrial Layout")
        self.root.resizable(False, False)

        # Standaard status variabelen
        self.ui_start_stop_active = self.ui_reset_pressed = self.ui_training_inference_active = False
        self.snelheid_laatste = 5
        self.versnelling_laatste = 2

        # --- HOOFD-LAYOUT FRAMES (Links vs Rechts) ---
        self.left_panel = tk.Frame(self.root)
        self.left_panel.pack(side=tk.LEFT, padx=15, pady=10, fill=tk.Y)

        self.right_panel = tk.Frame(self.root)
        self.right_panel.pack(side=tk.RIGHT, padx=15, pady=10, fill=tk.BOTH, expand=True)

        # =========================================================================
        # LINKS: VIDEOBEELDEN (Groter gemaakt naar 320x240, met behoud van 4:3 ratio)
        # =========================================================================
        # 1. Video DepthAI
        self.frame_cam = tk.Frame(self.left_panel, bg="black", width=320, height=240)
        self.frame_cam.pack(pady=(0, 10))
        self.frame_cam.pack_propagate(False)
        
        self.camera_label = tk.Label(self.frame_cam, bg="black")
        self.camera_label.pack(fill="both", expand=True)

        # 2. Video RViz
        self.frame_rviz = tk.Frame(self.left_panel, bg="black", width=320, height=240)
        self.frame_rviz.pack()
        self.frame_rviz.pack_propagate(False)
        
        self.rviz_label = tk.Label(self.frame_rviz, bg="black")
        self.rviz_label.pack(fill="both", expand=True)

        # =========================================================================
        # RECHTS BOVEN: KNOPPEN (Originele kleuren behouden)
        # =========================================================================
        self.buttons_frame = tk.Frame(self.right_panel)
        self.buttons_frame.pack(fill=tk.X, pady=(0, 15))

        # Trainingknop (Blauw)
        self.training = tk.Button(self.buttons_frame, text="TRAINING", width=14, height=2, bg="blue", fg="white", command=self.toggle_ui_training_inference)
        self.training.grid(row=0, column=0, padx=5)

        # Container frame voor de wisselende knoppen
        self.action_button_container = tk.Frame(self.buttons_frame)
        self.action_button_container.grid(row=0, column=1, padx=5)

        self.turn_on = tk.Button(self.action_button_container, text="ON", width=10, height=2, command=self.toggle_turn_on)
        self.turn_off = tk.Button(self.action_button_container, text="OFF", width=10, height=2, command=self.toggle_turn_off)
        
        # De rode Resetknop (actieve achtergrond geel/pink conform origineel)
        self.reset = tk.Button(self.action_button_container, text="RESET", width=22, height=2, bg="red", fg="white", activebackground="pink", command=self.toggle_ui_reset)

        # Retryknop
        self.retry = tk.Button(self.buttons_frame, text="RETRY", width=14, height=2, bg="lightgray", command=self.trigger_ui_retry)
        self.retry.grid(row=0, column=2, padx=5)

        # =========================================================================
        # RECHTS MIDDEN: SLIDERS (Naast elkaar, lengte iets vergroot voor de verhouding)
        # =========================================================================
        self.sliders_frame = tk.Frame(self.right_panel)
        self.sliders_frame.pack(fill=tk.X, pady=(0, 15))

        self.slider_snelheid = tk.Scale(self.sliders_frame, from_=0, to=10, orient=tk.HORIZONTAL, tickinterval=2, label=f"Snelheid (Laatste: {self.snelheid_laatste})", length=240)
        self.slider_snelheid.bind("<ButtonRelease-1>", self.update_snelheid)
        self.slider_snelheid.pack(side=tk.LEFT, padx=(5, 20))
        
        self.slider_versnelling = tk.Scale(self.sliders_frame, from_=0, to=10, orient=tk.HORIZONTAL, tickinterval=2, label=f"Versnelling (Laatste: {self.versnelling_laatste})", length=240)
        self.slider_versnelling.bind("<ButtonRelease-1>", self.update_versnelling)

        # =========================================================================
        # RECHTS ONDER: TERMINAL (Compacte ratio behouden via height=7)
        # =========================================================================
        self.log_frame = tk.Frame(self.right_panel)
        self.log_frame.pack(fill=tk.BOTH, expand=True)

        self.log = tk.Text(self.log_frame, height=7, state="disabled", bg="black", fg="white", insertbackground="white")  
        self.log.pack(fill=tk.BOTH, expand=True)

        # --- CAMERA HARDWARE INITIALISATIE ---
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        self.cap_rviz = cv2.VideoCapture(1) 
        self.cap_rviz.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap_rviz.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        self._video_loop()
        self.toggle_turn_off() # Start de UI in de standaard veilige 'OFF' modus

    def _video_loop(self):
        """ Haalt live frames op via OpenCV en toont ze in de grotere rechthoekige frames """
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame_resized = cv2.resize(frame, (320, 240)) # Matchen met de nieuwe grotere ratio
                h, w, _ = frame_resized.shape
                cv2.circle(frame_resized, (int(w/2), int(h/2)), 4, (0, 255, 0), -1)
                opencv_image = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGBA)
                captured_image = Image.fromarray(opencv_image)
                photo_image = ImageTk.PhotoImage(image=captured_image)
                self.camera_label.photo_image = photo_image
                self.camera_label.configure(image=photo_image)

        if self.cap_rviz.isOpened():
            ret_rviz, frame_rviz = self.cap_rviz.read()
            if ret_rviz:
                frame_rviz_resized = cv2.resize(frame_rviz, (320, 240))
                rviz_image_rgb = cv2.cvtColor(frame_rviz_resized, cv2.COLOR_BGR2RGBA)
                rviz_captured_image = Image.fromarray(rviz_image_rgb)
                rviz_photo_image = ImageTk.PhotoImage(image=rviz_captured_image)
                self.rviz_label.photo_image = rviz_photo_image
                self.rviz_label.configure(image=rviz_photo_image)

        self.root.after(15, self._video_loop)

    def _wissel_ui_modus(self, naar_training=False):
        """ Regelt het dynamisch wisselen van de knoppen en sliders op basis van je schets """
        if naar_training:
            self.turn_on.pack_forget()
            self.turn_off.pack_forget()
            self.reset.pack(fill=tk.BOTH, expand=True)
            
            self.slider_versnelling.pack(side=tk.LEFT, padx=5)
            self.slider_versnelling.config(state="normal")
        else:
            self.reset.pack_forget()
            self.turn_on.pack(side=tk.LEFT, padx=(0, 5))
            self.turn_off.pack(side=tk.LEFT)
            
            self.slider_versnelling.pack_forget()

    def toggle_turn_on(self):
        self.ui_start_stop_active = True
        self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {self.snelheid_laatste})")  
        self.slider_snelheid.set(self.snelheid_laatste)
        self.turn_on.config(bg="green", activebackground="lightgreen")
        self.turn_off.config(bg="lightgray")
        self.update_ui_log(f"[HMI] Systeem aangezet (ON). Snelheid: {self.snelheid_laatste}")
        if 'ui_start_stop' in self.callbacks: self.callbacks['ui_start_stop'](True)

    def toggle_turn_off(self):
        self._sla_huidige_snelheid_op()
        self.ui_start_stop_active = self.ui_training_inference_active = False
        self._wissel_ui_modus(naar_training=False)
        self.slider_snelheid.set(0)
        self.slider_snelheid.config(state="disabled", label=f"Snelheid (Laatste: {self.snelheid_laatste})")  
        self.turn_on.config(state="normal", bg="lightgray")
        self.turn_off.config(state="normal", bg="red", activebackground="pink")
        self.training.config(bg="blue", fg="white", state="normal")
        self.retry.config(state="normal", bg="lightgray")
        self.update_ui_log(f"[HMI] Systeem uitgezet (OFF).")
        if 'ui_start_stop' in self.callbacks: self.callbacks['ui_start_stop'](False)

    def toggle_ui_training_inference(self):
        self.ui_training_inference_active = not self.ui_training_inference_active
        self.ui_start_stop_active = False
        if self.ui_training_inference_active:
            self._wissel_ui_modus(naar_training=True)
            self.training.config(bg="blue", activebackground="purple")
            self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {self.snelheid_laatste})") 
            self.slider_versnelling.set(self.versnelling_laatste)
            self.update_ui_log(f"[HMI] Training Modus Actief.")
            if 'ui_training_inference' in self.callbacks: self.callbacks['ui_training_inference'](True)
        else:
            self.versnelling_laatste = self.slider_versnelling.get()
            self.toggle_turn_off()
            if 'ui_training_inference' in self.callbacks: self.callbacks['ui_training_inference'](False)

    def toggle_ui_reset(self):
        """ Blokkeert ALLES tijdens de 5 seconden reset van de training """
        self.update_ui_log("[HMI] Reset aangevraagd. Interface geblokkeerd voor 5s...")
        
        self.reset.config(state="disabled", bg="lightgray")
        self.training.config(state="disabled")
        self.retry.config(state="disabled", bg="lightgray")
        self.slider_snelheid.config(state="disabled", label=f"Snelheid (Laatste: {self.snelheid_laatste})")
        self.slider_versnelling.config(state="disabled", label=f"Versnelling (Laatste: {self.versnelling_laatste})")
        
        self.root.after(5000, self._herstel_na_reset)
        if 'ui_reset' in self.callbacks: self.callbacks['ui_reset']()

    def _herstel_na_reset(self):
        self.reset.config(state="normal", bg="red")
        self.training.config(state="normal")
        self.retry.config(state="normal", bg="lightgray")
        
        self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {self.snelheid_laatste})")
        self.slider_versnelling.config(state="normal", label=f"Versnelling (Actief: {self.versnelling_laatste})")
        self.slider_versnelling.set(self.versnelling_laatste)
        
        self.update_ui_log("[HMI] Reset voltooid. Training herstart. Interface weer beschikbaar.")

    def trigger_ui_retry(self):
        self.update_ui_log("[HMI] Retry ingedrukt. Interface geblokkeerd voor 5s...")
        self.retry.config(state="disabled", bg="lightgray")
        self.reset.config(state="disabled", bg="lightgray")
        self.training.config(state="disabled")
        self.slider_snelheid.config(state="disabled")
        self.slider_versnelling.config(state="disabled")
        
        self.root.after(5000, lambda: [
            self.retry.config(state="normal", bg="lightgray"),
            self.reset.config(state="normal", bg="red") if self.ui_training_inference_active else None,
            self.training.config(state="normal"),
            self.slider_snelheid.config(state="normal"),
            self.slider_versnelling.config(state="normal") if self.ui_training_inference_active else None,
            self.update_ui_log("[HMI] Interface weer beschikbaar.")
        ])
        if 'ui_retry' in self.callbacks: self.callbacks['ui_retry']()

    def update_ui_log(self, bericht):
        self.root.after(0, lambda: [self.log.config(state="normal"), self.log.insert("end", bericht + "\n"), self.log.see("end"), self.log.config(state="disabled")])

    def _sla_huidige_snelheid_op(self):
        if self.ui_start_stop_active and self.slider_snelheid.get() > 0: self.snelheid_laatste = self.slider_snelheid.get()

    def update_snelheid(self, event):
        if not self.ui_start_stop_active and not self.ui_training_inference_active: return
        if self.slider_snelheid.get() > 0: self.snelheid_laatste = self.slider_snelheid.get()
        self.slider_snelheid.config(label=f"Snelheid (Actief: {self.snelheid_laatste})")
        if 'snelheid' in self.callbacks: self.callbacks['snelheid'](self.slider_snelheid.get())

    def update_versnelling(self, event):
        if not self.ui_training_inference_active: return
        self.versnelling_laatste = self.slider_versnelling.get()
        self.slider_versnelling.config(label=f"Versnelling (Actief: {self.versnelling_laatste})")
        if 'versnelling' in self.callbacks: self.callbacks['versnelling'](self.slider_versnelling.get())

    def close_hardware(self):
        if hasattr(self, 'cap') and self.cap.isOpened(): self.cap.release()
        if hasattr(self, 'cap_rviz') and self.cap_rviz.isOpened(): self.cap_rviz.release()

# ===== RUN MODUS =====
if __name__ == '__main__':
    root = tk.Tk()
    dummy_callbacks = {
        'ui_start_stop': lambda status: print(f"[ROS2] /ui_start_stop -> {status}"),
        'ui_reset': lambda: print("[ROS2] /ui_reset Requested"),
        'ui_training_inference': lambda status: print(f"[ROS2] /ui_training_inference -> {status}"),
        'ui_retry': lambda: print("[ROS2] /ui_retry Requested"),
        'snelheid': lambda val: print(f"[ROS2] Snelheid -> {val}"),
        'versnelling': lambda val: print(f"[ROS2] Versnelling -> {val}")
    }
    ui = HumanInterface(root, dummy_callbacks)
    root.protocol("WM_DELETE_WINDOW", lambda: [ui.close_hardware(), root.destroy()])
    root.mainloop()