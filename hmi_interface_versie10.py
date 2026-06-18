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
        self.root.geometry("750x350")
        self.root.title("HMI Interface - OpenCV Video Display")
        self.root.resizable(False, False)

        #Standaard status van elke topic / service / variabele
        self.ui_start_stop_active = self.ui_reset_pressed = self.ui_training_inference_active = False
        self.snelheid_laatste = 5
        self.versnelling_laatste = 2

        # --- UI IMPLEMENTATIE ---#
        # video camera (OAK depth AI gebruikt als standaard USB-camera)
        self.frame_left = tk.Frame(self.root, bg="black", width=320, height=240)
        self.frame_left.pack(padx=10, pady=10, side=tk.LEFT, anchor="n")
        self.frame_left.pack_propagate(False) 

        self.camera_label = tk.Label(self.frame_left, bg="black")
        self.camera_label.pack(fill="both", expand=True)

        #beeld (RVIZ manipulator)
        self.frame_right = tk.Frame(self.root, bg="green", width=100, height=200)

        # Knoppen
        self.turn_on = tk.Button(self.root, text="ON", width=6, command=self.toggle_turn_on)
        self.turn_off = tk.Button(self.root, text="OFF", width=6, command=self.toggle_turn_off)
        self.reset = tk.Button(self.root, text="RESET", width=6, command=self.toggle_ui_reset, activebackground="yellow")
        self.training = tk.Button(self.root, text="TRAINING", width=9, bg="blue", fg="white", command=self.toggle_ui_training_inference)
        self.retry = tk.Button(self.root, text="RETRY", width=6, bg="lightgray", command=self.trigger_ui_retry)

        # Sliders
        self.slider_snelheid = tk.Scale(self.root, from_=0, to=10, orient=tk.HORIZONTAL, tickinterval=2, label=f"Snelheid (Laatste: {self.snelheid_laatste})")
        self.slider_snelheid.bind("<ButtonRelease-1>", self.update_snelheid)
        
        self.slider_versnelling = tk.Scale(self.root, from_=0, to=10, orient=tk.HORIZONTAL, tickinterval=2, label=f"Versnelling (Laatste: {self.versnelling_laatste})")
        self.slider_versnelling.bind("<ButtonRelease-1>", self.update_versnelling)

        # Terminal 
        self.log = tk.Text(self.root, height=5, width=45, state="disabled", bg="black", fg="white", insertbackground="white")  
        self.log.pack(padx=10, pady=10, expand=True, side="right", anchor="s")

        #Positie basis UI en de Sliders (Nu vast onder elkaar aan de rechterkant)
        self.reset.place(x=480, y=20)     
        self.training.place(x=360, y=60)  
        self.slider_snelheid.place(x=560, y=5)
        # self.slider_versnelling wordt hier niet geplaatst, zodat deze initieel onzichtbaar is
        
        """dit is het begin van de camera configuratie"""
        # --- OPENCV STANDAARD WEBCAM CONFIGURATIE ---#
        # We openen de camera via de standaard OpenCV index (meestal 0, 1 of 2 afhankelijk van je PC)
        self.cap = cv2.VideoCapture(0)
        
        # Stel de resolutie direct in op het formaat van je HMI frame
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Start de oneindige video- en dataloop
        self._video_loop()
        """dit is het einde van de camera configuratie"""
        
        self.toggle_turn_off()

    def _video_loop(self):
        """ Haalt live frames op via OpenCV en toont ze in Tkinter """
        # Controleer of de camera open staat
        if self.cap.isOpened():
            ret, frame = self.cap.read() # Lees een los foto-frame uit de videostroom
            
            if ret:
                # Optioneel: Teken een klein richtpunt in het midden (OpenCV data-bewerking)
                h, w, _ = frame.shape
                cv2.circle(frame, (int(w/2), int(h/2)), 4, (0, 255, 0), -1)

                # Zet het OpenCV BGR frame om naar RGB en stuur het naar het Tkinter Label
                opencv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
                captured_image = Image.fromarray(opencv_image)
                photo_image = ImageTk.PhotoImage(image=captured_image)
                
                self.camera_label.photo_image = photo_image
                self.camera_label.configure(image=photo_image)

        # Vraag over 15 milliseconden het volgende frame aan (vloeiende video)
        self.root.after(15, self._video_loop)

    def close_hardware(self):
        """ Sluit de OpenCV camera verbinding netjes af """
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            print("[HMI] OpenCV Camera succesvol afgesloten.")

    # --- BESTAANDE HMI LOGICA ---
    def update_ui_log(self, bericht):
        self.root.after(0, lambda: [self.log.config(state="normal"), self.log.insert("end", bericht + "\n"), self.log.see("end"), self.log.config(state="disabled")])

    def _sla_huidige_snelheid_op(self):
        if self.ui_start_stop_active and self.slider_snelheid.get() > 0: self.snelheid_laatste = self.slider_snelheid.get()

    def _wissel_ui_modus(self, naar_training=False):
        if naar_training:
            self.turn_on.place_forget(); self.turn_off.place_forget()
            self.retry.place(x=360, y=20)
            self.slider_versnelling.place(x=560, y=75) # Toon de slider onder de snelheid bij training
            self.slider_versnelling.config(state="normal")
        else:
            self.retry.place_forget()
            self.slider_versnelling.place_forget() # Verberg de slider als we uit training gaan
            self.turn_on.place(x=360, y=20); self.turn_off.place(x=420, y=20)

    def trigger_ui_retry(self):
        self.update_ui_log("[HMI] Retry ingedrukt. Interface geblokkeerd voor 5s...")
        self.retry.config(state="disabled", bg="lightgray")
        self.reset.config(state="disabled", bg="lightgray")
        self.training.config(state="disabled")
        self.root.after(5000, lambda: [
            self.retry.config(state="normal", bg="lightgray"),
            self.reset.config(state="normal", bg="orange"),
            self.training.config(state="normal"),
            self.update_ui_log("[HMI] Interface weer beschikbaar.")
        ])
        if 'ui_retry' in self.callbacks: self.callbacks['ui_retry']()

    def toggle_ui_reset(self):
        self._sla_huidige_snelheid_op()
        was_in_training = self.ui_training_inference_active
        self.ui_reset_pressed, self.ui_start_stop_active = True, False
        self._wissel_ui_modus(naar_training=was_in_training)
        self.slider_snelheid.set(0)
        
        self.turn_on.config(state="disabled", bg="lightgray")
        self.turn_off.config(state="disabled", bg="lightgray")
        self.reset.config(state="disabled", bg="lightgray")
        self.retry.config(state="disabled", bg="lightgray")
        self.training.config(state="disabled")
        
        self.slider_snelheid.config(state="disabled", label=f"Snelheid (Laatste: {self.snelheid_laatste})")
        self.slider_versnelling.config(state="disabled", label=f"Versnelling (Laatste: {self.versnelling_laatste})")
        
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

    def update_snelheid(self, event):
        if not self.ui_start_stop_active and not self.ui_training_inference_active: return
        if self.slider_snelheid.get() > 0: self.snelheid_laatste = self.slider_snelheid.get()
        self.slider_snelheid.config(label=f"Snelheid (Actief: {self.snelheid_laatste})")
        if 'snelheid' in self.callbacks: self.callbacks['snelheid'](self.slider_snelheid.get())

    def update_versnelling(self, event):
        if not self.ui_training_inference_active: return # Alleen aanpassen tijdens training
        self.versnelling_laatste = self.slider_versnelling.get()
        self.slider_versnelling.config(label=f"Versnelling (Actief: {self.versnelling_laatste})")
        if 'versnelling' in self.callbacks: self.callbacks['versnelling'](self.slider_versnelling.get())

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