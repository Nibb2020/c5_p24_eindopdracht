import os
import sys
import tkinter as tk
import threading
import time
import cv2
from PIL import Image, ImageTk

# --- ROS2 INTERFACES IMPORTS CONFORM JOUW TABEL ---
try:
    from std_msgs.msg import Bool, String
    from project_interfaces.srv import Empty
except ImportError:
    # Veilige fallback/mock klassen voor lokaal testen zonder ROS2-omgeving
    class Bool:
        def __init__(self): self.data = False
    class String:
        def __init__(self): self.data = ""
    class Empty:
        class Request:
            pass


class HumanInterface:
    def __init__(self, root, callbacks):
        self.root, self.callbacks = root, callbacks
        self.root.geometry("1000x570")
        self.root.title("HMI Interface - OpenCV Video Display")
        self.root.resizable(False, False)

        # Statusvariabelen conform jouw originele logica
        self.ui_start_stop_active = self.ui_reset_pressed = self.ui_training_inference_active = False
        self.snelheid_laatste = 5
        self.versnelling_laatste = 2

        # --- HOOFD-LAYOUT FRAMES ---
        self.left_panel = tk.Frame(self.root)
        self.left_panel.pack(side=tk.LEFT, padx=15, pady=10, fill=tk.Y)

        self.right_panel = tk.Frame(self.root)
        self.right_panel.pack(side=tk.RIGHT, padx=15, pady=10, fill=tk.BOTH, expand=True)

        # =========================================================================
        # LINKS: VIDEOBEELDEN (Jouw OpenCV Cam & RViz / Marked Foto)
        # =========================================================================
        # 1. Video DepthAI / Camera
        self.frame_left = tk.Frame(self.left_panel, bg="black", width=320, height=240)
        self.frame_left.pack(pady=(0, 10))
        self.frame_left.pack_propagate(False) 

        self.camera_label = tk.Label(self.frame_left, bg="black")
        self.camera_label.pack(fill="both", expand=True)

        # 2. Video Stream / Gemarkeerde foto (/marked_foto)
        self.frame_right = tk.Frame(self.left_panel, bg="green", width=320, height=240) 
        self.frame_right.pack()
        self.frame_right.pack_propagate(False)

        self.rviz_label = tk.Label(self.frame_right, bg="black")
        self.rviz_label.pack(fill="both", expand=True)

        # =========================================================================
        # RECHTS BOVEN: KNOPPEN 
        # =========================================================================
        self.buttons_frame = tk.Frame(self.right_panel)
        self.buttons_frame.pack(fill=tk.X, pady=(0, 15))

        # Trainingknop -> Gekoppeld aan /ui_training_inference (Service Client)
        self.training = tk.Button(self.buttons_frame, text="TRAINING", width=14, height=2, bg="blue", fg="white", command=self.toggle_ui_training_inference)
        self.training.pack(side=tk.LEFT, padx=5)

        # Wisselende actie-knoppen container
        self.action_button_container = tk.Frame(self.buttons_frame)
        self.action_button_container.pack(side=tk.LEFT, padx=5)

        # Start/Stop Knoppen -> Gekoppeld aan /ui_start_stop (Publisher)
        self.turn_on = tk.Button(self.action_button_container, text="ON", width=10, height=2, command=self.toggle_turn_on)
        self.turn_off = tk.Button(self.action_button_container, text="OFF", width=10, height=2, command=self.toggle_turn_off)
        
        # Reset Knop -> Gekoppeld aan /ui_reset (Service Client)
        self.reset = tk.Button(self.action_button_container, text="RESET", width=10, height=2, command=self.toggle_ui_reset, activebackground="yellow")
        
        # Retry Knop -> Gekoppeld aan /ui_retry (Service Client)
        self.retry = tk.Button(self.action_button_container, text="RETRY", width=10, height=2, bg="lightgray", command=self.trigger_ui_retry)

        # =========================================================================
        # RECHTS MIDDEN: SLIDERS (Interne parameters, behouden conform jouw code)
        # =========================================================================
        self.sliders_frame = tk.Frame(self.right_panel)
        self.sliders_frame.pack(fill=tk.X, pady=(0, 15))

        self.slider_snelheid = tk.Scale(self.sliders_frame, from_=0, to=10, orient=tk.HORIZONTAL, tickinterval=2, label=f"Snelheid (Laatste: {self.snelheid_laatste})", length=240)
        self.slider_snelheid.bind("<ButtonRelease-1>", self.update_snelheid)
        self.slider_snelheid.pack(side=tk.LEFT, padx=(5, 20))
        
        self.slider_versnelling = tk.Scale(self.sliders_frame, from_=0, to=10, orient=tk.HORIZONTAL, tickinterval=2, label=f"Versnelling (Laatste: {self.versnelling_laatste})", length=240)
        self.slider_versnelling.bind("<ButtonRelease-1>", self.update_versnelling)

        # =========================================================================
        # RECHTS ONDER: TERMINAL LOGGING (Toont live updates van /ui_robot_status)
        # =========================================================================
        self.log_frame = tk.Frame(self.right_panel)
        self.log_frame.pack(fill=tk.BOTH, expand=True)

        self.log = tk.Text(self.log_frame, height=9, state="disabled", bg="black", fg="white", insertbackground="white")  
        self.log.pack(fill=tk.BOTH, expand=True)

        # --- HARDWARE INITIALISATIE (OpenCV) ---
        self.cap_rviz = cv2.VideoCapture("http://localhost:8080/stream?topic=/rviz/camera_image") 
        self.cap_rviz.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap_rviz.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Start loops
        self._video_loop()
        self.toggle_turn_off()

    def _video_loop(self):
        """ OpenCV stream verwerking """
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
        """
        Wordt extern aangeroepen door de Subscriber van '/ui_robot_status'.
        Toont de actuele status van de controller live in de GUI-terminal.
        """
        self.root.after(0, lambda: [
            self.log.config(state="normal"), 
            self.log.insert("end", bericht + "\n"), 
            self.log.see("end"), 
            self.log.config(state="disabled")
        ])

    def _sla_huidige_snelheid_op(self):
        if self.ui_start_stop_active and self.slider_snelheid.get() > 0: self.snelheid_laatste = self.slider_snelheid.get()

    def _wissel_ui_modus(self, naar_training=False):
        if naar_training:
            self.turn_on.pack_forget()
            self.turn_off.pack_forget()
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
            self.reset.pack(side=tk.LEFT)

    # =========================================================================
    # EXACTE INTERFACE TRIGGERS CONFORM JOUW TABEL
    # =========================================================================

    def toggle_turn_on(self):
        """ TOPIC PUBLISHER: Stuurt True via std_msgs/Bool naar /ui_start_stop """
        self.ui_start_stop_active, self.ui_reset_pressed = True, False
        self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {self.snelheid_laatste})")  
        self.slider_snelheid.set(self.snelheid_laatste)
        self.turn_on.config(bg="green", activebackground="lightgreen"); self.turn_off.config(bg="lightgray")
        if self.reset['state'] == 'normal': self.reset.config(bg="lightgray")
        
        self.update_ui_log("[HMI -> Publisher] Systeem START -> /ui_start_stop: True")
        
        msg = Bool()
        msg.data = True
        if 'ui_start_stop' in self.callbacks: 
            self.callbacks['ui_start_stop'](msg)

    def toggle_turn_off(self):
        """ TOPIC PUBLISHER: Stuurt False via std_msgs/Bool naar /ui_start_stop """
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
        
        self.update_ui_log("[HMI -> Publisher] Systeem STOP -> /ui_start_stop: False")
        
        msg = Bool()
        msg.data = False
        if 'ui_start_stop' in self.callbacks: 
            self.callbacks['ui_start_stop'](msg)

    def toggle_ui_reset(self):
        """ SERVICE CLIENT: Stuurt een Empty Request naar de Server op /ui_reset """
        self._sla_huidige_snelheid_op()
        was_in_training = self.ui_training_inference_active
        self.ui_reset_pressed, self.ui_start_stop_active = True, False
        
        self._wissel_ui_modus(naar_training=was_in_training)
        self.slider_snelheid.set(0)
        
        # Tijdelijke blokkade van de GUI-knoppen tijdens de service call
        self.turn_on.config(state="disabled", bg="lightgray")
        self.turn_off.config(state="disabled", bg="lightgray")
        self.reset.config(state="disabled", bg="lightgray")
        self.retry.config(state="disabled", bg="lightgray")
        self.training.config(state="disabled")
        self.slider_snelheid.config(state="disabled")
        self.slider_versnelling.config(state="disabled")
        self.root.update_idletasks() 

        # Lokale GUI-timeout simulatie (de controller deblokkeert dit zodra de response er is)
        self.root.after(4000, lambda: self._herstel_na_reset(was_in_training))
        
        self.update_ui_log("[HMI -> Service Client] Reset aangevraagd via /ui_reset...")
        
        req = Empty.Request()
        if 'ui_reset' in self.callbacks: 
            self.callbacks['ui_reset'](req)

    def toggle_ui_training_inference(self):
        """ SERVICE CLIENT: Stuurt een Empty Request naar de Server op /ui_training_inference """
        self.ui_training_inference_active = not self.ui_training_inference_active
        self.ui_start_stop_active = self.ui_reset_pressed = False
        
        if self.ui_training_inference_active:
            self._wissel_ui_modus(naar_training=True)
            self.training.config(bg="blue", activebackground="purple")
            if self.reset['state'] == 'normal': self.reset.config(bg="orange")
            self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {self.snelheid_laatste})") 
            self.slider_versnelling.config(state="normal", label=f"Versnelling (Actief: {self.versnelling_laatste})"); self.slider_versnelling.set(self.versnelling_laatste)
            
            self.update_ui_log("[HMI -> Service Client] Training/Inference modus activeren via /ui_training_inference...")
            req = Empty.Request()
            if 'ui_training_inference' in self.callbacks: 
                self.callbacks['ui_training_inference'](req)
        else:
            self.versnelling_laatste = self.slider_versnelling.get()
            self.toggle_turn_off()
            
            self.update_ui_log("[HMI -> Service Client] Training/Inference modus deactiveren via /ui_training_inference...")
            req = Empty.Request()
            if 'ui_training_inference' in self.callbacks: 
                self.callbacks['ui_training_inference'](req)

    def trigger_ui_retry(self):
        """ SERVICE CLIENT: Stuurt een Empty Request naar de Server op /ui_retry """
        self.update_ui_log("[HMI -> Service Client] Enkele cyclus aangevraagd via /ui_retry...")
        
        self.retry.config(state="disabled", bg="lightgray")
        self.reset.config(state="disabled", bg="lightgray")
        self.training.config(state="disabled")
        self.slider_snelheid.config(state="disabled")
        self.slider_versnelling.config(state="disabled")
        self.root.update_idletasks()

        self.root.after(4000, lambda: [
            self.retry.config(state="normal", bg="lightgray"),
            self.reset.config(state="normal", bg="orange" if self.ui_training_inference_active else "lightgray"),
            self.training.config(state="normal"),
            self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {self.snelheid_laatste})"),
            self.slider_versnelling.config(state="normal", label=f"Versnelling (Actief: {self.versnelling_laatste})") if self.ui_training_inference_active else None,
            self.update_ui_log("[HMI] Enkele cyclus aanvraag verwerkt.")
        ])
        
        req = Empty.Request()
        if 'ui_retry' in self.callbacks: 
            self.callbacks['ui_retry'](req)

    def update_snelheid(self, event):
        """ Interne HMI Parameter update """
        if not (self.ui_start_stop_active or self.ui_training_inference_active): return
        huidige_waarde = self.slider_snelheid.get()
        if huidige_waarde > 0: self.snelheid_laatste = huidige_waarde
        self.slider_snelheid.config(label=f"Snelheid (Actief: {self.snelheid_laatste})")
        if 'snelheid' in self.callbacks: 
            self.callbacks['snelheid'](int(huidige_waarde))

    def update_versnelling(self, event):
        """ Interne HMI Parameter update """
        if not self.ui_training_inference_active: return 
        huidige_waarde = self.slider_versnelling.get()
        self.versnelling_laatste = huidige_waarde
        self.slider_versnelling.config(label=f"Versnelling (Actief: {self.versnelling_laatste})")
        if 'versnelling' in self.callbacks: 
            self.callbacks['versnelling'](int(huidige_waarde))

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
        else:
            self.turn_on.config(state="normal", bg="lightgray")
            self.turn_off.config(state="normal", bg="red", activebackground="pink")
            self.reset.config(state="normal", bg="lightgray")
            self.slider_snelheid.config(state="disabled", label=f"Snelheid (Laatste: {self.snelheid_laatste})")
            self.slider_versnelling.config(state="disabled", label=f"Versnelling (Laatste: {self.versnelling_laatste})")


# =========================================================================
# RUN / TEST OMGEVING MET JOUW ECHTE INTERFACE-STRUCTUUR (MOCK CALLBACKS)
# =========================================================================
if __name__ == '__main__':
    root = tk.Tk()
    
    # De simulatie-callbacks bootsen exact na hoe jouw ROS2-node bestand (de brug)
    # dadelijk de data ontvangt. 
    simulatie_callbacks = {
        'ui_start_stop': lambda msg: print(f"[MOCK-ROUTER] Ontvangen op /ui_start_stop -> msg.data: {msg.data}"),
        'ui_reset': lambda req: print(f"[MOCK-ROUTER] Service Client /ui_reset aangeroepen met Empty Request"),
        'ui_training_inference': lambda req: print(f"[MOCK-ROUTER] Service Client /ui_training_inference aangeroepen met Empty Request"),
        'ui_retry': lambda req: print(f"[MOCK-ROUTER] Service Client /ui_retry aangeroepen met Empty Request"),
        'snelheid': lambda val: print(f"[MOCK-ROUTER] Interne GUI snelheid gewijzigd naar: {val}"),
        'versnelling': lambda val: print(f"[MOCK-ROUTER] Interne GUI versnelling gewijzigd naar: {val}")
    }
    
    ui = HumanInterface(root, simulatie_callbacks)
    
    # Test-simulatie om te laten zien hoe de Subscriber op '/ui_robot_status' data in de UI logt:
    root.after(2000, lambda: ui.update_ui_log("[MOCK SUBSCRIBER] /ui_robot_status: STATUS_STANDBY"))
    
    root.protocol("WM_DELETE_WINDOW", lambda: [ui.close_hardware(), root.destroy()])
    root.mainloop()