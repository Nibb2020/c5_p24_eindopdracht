import os
import sys
import tkinter as tk
import threading
import time
import cv2
from PIL import Image, ImageTk

# --- ROS2 IMPORTS ---
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from project_interfaces.msg import ObjectDataArray
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge



class HumanInterface(Node):
    def __init__(self, root):
        # Initialiseer de ROS2 Node
        super().__init__("hmi_node")

        self.root = root

        self.root.geometry("1000x570")
        self.root.title("HMI Interface - OpenCV Video Display")
        self.root.resizable(False, False)

        # ================= STATE =================
        self.ui_start_stop_active = self.ui_reset_pressed = self.ui_training_inference_active = False
        
        # Interne MoveGroup parameters (als float tussen 0.05 en 0.50 vanwege de 50% limiet)
        self.snelheid_laatste = 0.10  # Standaard 10%
        self.versnelling_laatste = 0.20  # Standaard 20%
        self.vision_confidence_laatste = 0.70

        self.robot_status = "UNKNOWN"
        self.is_countdown_active = False  # Houdt bij of er een timer loopt

        # ================= VARIABELEN =================
        self.pending_reset = False
        self.pending_retry = False

        # ================= OBJECTTELLER =================
        # Use the exact lowercase class names published by vision.
        self.vaste_klassen = ["dino", "schip", "olifant", "smiley"]

        self.gesorteerde_tellers = {object_class: 0 for object_class in self.vaste_klassen}

        self.ui_teller_labels = {}

        # Previous ObjectDataArray converted to class and position.
        self.vorige_vision_objecten = None

        # Maximum XY movement for considering two detections the same object.
        self.object_match_afstand = 0.01

        # ================= ROS2 CONFIGURATIE =================
        #publishers
        self.pub_start_stop = self.create_publisher(Bool, "/ui/start_stop", 10)

        #subscribers
        self.sub_robot_status = self.create_subscription(String,"/controller/state",self.robot_status_callback,10)
        self.sub_robot_warning = self.create_subscription(String,"/controller/warning",self.robot_warning_callback,10)
        self.sub_robot_error = self.create_subscription(String,"/controller/error",self.robot_error_callback,10)
        self.sub_vision_data = self.create_subscription(ObjectDataArray,"/vision/object_data_ui",self.vision_data_callback,10)

        #clients
        self.cli_reset = self.create_client(Trigger,"/ui/reset_error")
        self.cli_retry = self.create_client(Trigger,"/ui/retry")
        self.cli_training = self.create_client(SetBool,"/ui/training_mode")
        self.cli_move_home = self.create_client(Trigger,"/ui/move_home")

        # --- REMOTE PARAMETER CLIENT CONFIGURATIE ---
        self.andere_node_naam = "manipulator"
        self.remote_parameter_client = AsyncParameterClient(self,self.andere_node_naam)

        self.controller_node_naam = "robot_controller"
        self.controller_parameter_client = AsyncParameterClient(self,self.controller_node_naam)
        # =====================================================
        # ==================== UI LAYOUT ======================
        # =====================================================

        self.camera_panel = tk.Frame(self.root)
        self.camera_panel.pack(side=tk.RIGHT, padx=15, pady=10, fill=tk.Y)

        self.control_panel = tk.Frame(self.root)
        self.control_panel.pack(side=tk.LEFT, padx=15, pady=10, fill=tk.BOTH, expand=True)

        self.frame_left = tk.Frame(self.camera_panel, bg="black", width=320, height=240)
        self.frame_left.pack()
        self.frame_left.pack_propagate(False)

        self.camera_label = tk.Label(self.frame_left, bg="black")
        self.camera_label.pack(fill="both", expand=True)

        self.buttons_frame = tk.Frame(self.control_panel)
        self.buttons_frame.pack(fill=tk.X, pady=(0, 15))

        self.training = tk.Button(
            self.buttons_frame,
            text="TRAINING",
            width=14,
            height=2,
            bg="blue",
            fg="white",
            command=self.toggle_ui_training_inference
        )
        self.training.pack(side=tk.LEFT, padx=5)

        self.action_button_container = tk.Frame(self.buttons_frame)
        self.action_button_container.pack(side=tk.LEFT, padx=5)

        self.turn_on = tk.Button(self.action_button_container, text="ON", width=10, height=2, command=self.toggle_turn_on)
        self.turn_off = tk.Button(self.action_button_container, text="OFF", width=10, height=2, command=self.toggle_turn_off)
        self.reset = tk.Button(self.action_button_container, text="RESET", width=10, height=2, command=self.toggle_ui_reset, activebackground="yellow")
        self.retry = tk.Button(self.action_button_container, text="RETRY", width=10, height=2, bg="lightgray", command=self.trigger_ui_retry)
        self.btn_home = tk.Button(self.action_button_container,text="HOME",width=10,height=2,bg="purple",fg="white",command=self.trigger_manipulator_home)

        # Eénmalige stabiele initiële opbouw van de knoppen
        self.turn_on.pack(side=tk.LEFT, padx=(0, 5))
        self.turn_off.pack(side=tk.LEFT, padx=(0, 5))
        self.reset.pack(side=tk.LEFT, padx=(0, 5))
        self.retry.pack(side=tk.LEFT)

        self.sliders_frame = tk.Frame(self.control_panel)
        self.sliders_frame.pack(fill=tk.X, pady=(0, 15))

        # Snelheid Slider (Visueel in %: 5 tot 100)
        self.slider_snelheid = tk.Scale(
            self.sliders_frame,
            from_=5, to=100,
            resolution=1,
            orient=tk.HORIZONTAL,
            label=f"Snelheid (Laatste: {int(self.snelheid_laatste * 100)}%)",
            length=240
        )
        self.slider_snelheid.bind("<ButtonRelease-1>", self.update_snelheid)
        self.slider_snelheid.pack(side=tk.LEFT, padx=(5, 20))

        # Versnelling Slider (Visueel in %: 5 tot 100)
        self.slider_versnelling = tk.Scale(
            self.sliders_frame,
            from_=5, to=100,
            resolution=1,
            orient=tk.HORIZONTAL,
            label=f"Versnelling (Laatste: {int(self.versnelling_laatste * 100)}%)",
            length=240
        )
        self.slider_versnelling.bind("<ButtonRelease-1>", self.update_versnelling)
        self.slider_versnelling.pack(side=tk.LEFT, padx=5)

        self.vision_settings_frame = tk.Frame(self.control_panel)
        self.vision_settings_frame.pack(fill=tk.X,pady=(0,15))

        #Vision parameters interface
        self.lbl_threshold = tk.Label(self.vision_settings_frame,text="Vision confidence:")
        self.lbl_threshold.pack(side=tk.LEFT,padx=(5,10))

        self.entry_threshold = tk.Entry(self.vision_settings_frame,width=8)
        self.entry_threshold.insert(0,str(int(self.vision_confidence_laatste * 100)))
        self.entry_threshold.pack(side=tk.LEFT,padx=5)

        self.lbl_procent_teken = tk.Label(self.vision_settings_frame,text="%")
        self.lbl_procent_teken.pack(side=tk.LEFT,padx=(0,15))

        self.btn_set_threshold = tk.Button(self.vision_settings_frame,text="Toepassen",command=self.update_remote_threshold)
        self.btn_set_threshold.pack(side=tk.LEFT,padx=(0,15))

        # ================= VISUELE OBJECTTELLER =================
        self.stats_frame = tk.LabelFrame(
            self.control_panel,
            text="Sorteerstatistieken",
            font=("Arial", 11, "bold"),
            fg="blue"
        )
        self.stats_frame.pack(fill=tk.X, pady=(0, 15), padx=5)

        for object_class in self.vaste_klassen:
            row_frame = tk.Frame(self.stats_frame)
            row_frame.pack(fill=tk.X, padx=15, pady=2)

            lbl_naam = tk.Label(
                row_frame,
                text=f"{object_class.capitalize()}:",
                font=("Arial", 10),
                anchor="w"
            )
            lbl_naam.pack(side=tk.LEFT)

            lbl_aantal = tk.Label(
                row_frame,
                text="0",
                font=("Arial", 10, "bold"),
                fg="black",
                width=5,
                anchor="e"
            )
            lbl_aantal.pack(side=tk.RIGHT)

            self.ui_teller_labels[object_class] = lbl_aantal

        self.log_frame = tk.Frame(self.control_panel)
        self.log_frame.pack(fill=tk.BOTH, expand=True)

        self.log = tk.Text(self.log_frame, height=7, state="disabled",
                           bg="black", fg="white", insertbackground="white")
        self.log.pack(fill=tk.BOTH, expand=True)

        # ROS image initialisatie
        self.bridge = CvBridge()
        self.sub_marked_image = self.create_subscription(RosImage, "/vision/marked_foto", self.marked_image_callback, 10)

        self.toggle_turn_off()

    # =====================================================
    # INKOMENDE COMMUNICATIE (Subscribers)
    # =====================================================

    def robot_status_callback(self, msg: String):
        self.robot_status = msg.data
        state = msg.data.strip().casefold()

        self.root.after(0,lambda: self.update_ui_log(f"[ROBOT STATUS] {msg.data}"))

        if state == "stand-by":
            if self.pending_reset:
                self.pending_reset = False

                self.root.after(0,lambda: self._herstel_na_reset(False))

        elif state == "training/inference mode":
            if self.pending_retry:
                self.pending_retry = False

                self.root.after(0,self._deblokkeer_na_retry)

        elif state == "error":
            if self.pending_retry:
                self.pending_retry = False

                self.root.after(0,self._deblokkeer_na_retry)

    def robot_warning_callback(self, msg: String):
        warning = msg.data.strip()

        # The controller publishes an empty string when a warning is cleared.
        if not warning:
            return

        self.root.after(0,lambda text=warning: self.update_ui_log(f"[WAARSCHUWING] {text}"))


    def robot_error_callback(self, msg: String):
        error = msg.data.strip()

        # The controller publishes an empty string when an error is cleared.
        if not error:
            return

        self.root.after(0,lambda text=error: self.update_ui_log(f"[ERROR] {text}"))

    def _maak_vision_snapshot(self, msg: ObjectDataArray):
        snapshot = []

        for obj in msg.objects:
            object_class = obj.object_class.strip().casefold()

            if object_class not in self.gesorteerde_tellers:
                self.get_logger().warning(
                    f"Onbekende vision-klasse ontvangen: '{obj.object_class}'"
                )
                continue

            translation = obj.transform.transform.translation

            snapshot.append({
                "class": object_class,
                "x": float(translation.x),
                "y": float(translation.y),
            })

        return snapshot
    
    def _bereken_xy_afstand(self, object_a, object_b):
        verschil_x = object_a["x"] - object_b["x"]
        verschil_y = object_a["y"] - object_b["y"]

        return (verschil_x ** 2 + verschil_y ** 2) ** 0.5
    
    def _zoek_verdwenen_objecten(self, vorige_objecten, huidige_objecten):
        mogelijke_matches = []

        for vorige_index, vorig_object in enumerate(vorige_objecten):
            for huidige_index, huidig_object in enumerate(huidige_objecten):
                if vorig_object["class"] != huidig_object["class"]:
                    continue

                afstand = self._bereken_xy_afstand(
                    vorig_object,
                    huidig_object
                )

                if afstand <= self.object_match_afstand:
                    mogelijke_matches.append(
                        (
                            afstand,
                            vorige_index,
                            huidige_index,
                        )
                    )

        # Use the smallest distances first.
        mogelijke_matches.sort(key=lambda match: match[0])

        gekoppelde_vorige_indices = set()
        gekoppelde_huidige_indices = set()

        for afstand, vorige_index, huidige_index in mogelijke_matches:
            if vorige_index in gekoppelde_vorige_indices:
                continue

            if huidige_index in gekoppelde_huidige_indices:
                continue

            gekoppelde_vorige_indices.add(vorige_index)
            gekoppelde_huidige_indices.add(huidige_index)

        verdwenen_objecten = []

        for vorige_index, vorig_object in enumerate(vorige_objecten):
            if vorige_index not in gekoppelde_vorige_indices:
                verdwenen_objecten.append(vorig_object)

        nieuwe_objecten = []

        for huidige_index, huidig_object in enumerate(huidige_objecten):
            if huidige_index not in gekoppelde_huidige_indices:
                nieuwe_objecten.append(huidig_object)

        return verdwenen_objecten, nieuwe_objecten

    def vision_data_callback(self, msg: ObjectDataArray):
        huidige_objecten = self._maak_vision_snapshot(msg)

        # The first message only establishes the baseline.
        if self.vorige_vision_objecten is None:
            self.vorige_vision_objecten = huidige_objecten

            self.root.after(0,lambda aantal=len(huidige_objecten):self.update_ui_log(f"[TELLER] Beginmeting opgeslagen met {aantal} objecten."))
            return

        verdwenen_objecten, nieuwe_objecten = (
            self._zoek_verdwenen_objecten(
                self.vorige_vision_objecten,
                huidige_objecten
            )
        )

        verdwenen_per_klasse = {}

        for verdwenen_object in verdwenen_objecten:
            object_class = verdwenen_object["class"]

            verdwenen_per_klasse[object_class] = (
                verdwenen_per_klasse.get(object_class, 0) + 1
            )

        for object_class, aantal_verdwenen in verdwenen_per_klasse.items():
            self.gesorteerde_tellers[object_class] += aantal_verdwenen
            nieuw_totaal = self.gesorteerde_tellers[object_class]

            self.root.after(
                0,
                lambda klasse=object_class,
                    verschil=aantal_verdwenen,
                    totaal=nieuw_totaal:
                    self._verwerk_vision_telling(
                        klasse,
                        verschil,
                        totaal
                    )
            )

        if nieuwe_objecten:
            nieuwe_per_klasse = {}

            for nieuw_object in nieuwe_objecten:
                object_class = nieuw_object["class"]

                nieuwe_per_klasse[object_class] = (
                    nieuwe_per_klasse.get(object_class, 0) + 1
                )

            beschrijving = ", ".join(
                f"{object_class}: {aantal}"
                for object_class, aantal in nieuwe_per_klasse.items()
            )

            self.root.after(
                0,
                lambda tekst=beschrijving:
                    self.update_ui_log(
                        f"[TELLER] Nieuwe objecten gezien: {tekst}"
                    )
            )

        self.vorige_vision_objecten = huidige_objecten

    # =====================================================
    # UITGAANDE COMMUNICATIE (Publishers & Services)
    # =====================================================

    def publiceer_start_stop(self, status: bool):
        msg = Bool()
        msg.data = status
        self.pub_start_stop.publish(msg)
        self.get_logger().info(f"Gepubliceerd naar /ui/start_stop: {status}")

    def call_trigger_service_async(self,client,done_callback,) -> bool:
        if not client.wait_for_service(timeout_sec=1.0):
            self.root.after(
                0,
                lambda: self.update_ui_log(
                    "[ROS2 FOUT] Service niet bereikbaar!"
                ),
            )
            return False

        request = Trigger.Request()

        future = client.call_async(request)
        future.add_done_callback(done_callback)
        return True


    def call_training_service_async(self,enabled: bool,done_callback,) -> bool:
        if not self.cli_training.wait_for_service(
            timeout_sec=1.0
        ):
            self.root.after(
                0,
                lambda: self.update_ui_log(
                    "[ROS2 FOUT] Training service niet bereikbaar!"
                ),
            )
            return False

        request = SetBool.Request()
        request.data = enabled

        future = self.cli_training.call_async(request)
        future.add_done_callback(done_callback)
        return True

    def _verstuur_remote_parameter(self,param_naam,param_waarde):
        self._verstuur_parameter(self.remote_parameter_client,self.andere_node_naam,param_naam,param_waarde)

    def _verstuur_parameter(self,client,node_naam,param_naam,param_waarde):
        if not client.services_are_ready():
            self.root.after(0,lambda: self.update_ui_log(f"[ROS2 FOUT] Parameterservice van '/{node_naam}' is niet bereikbaar."))
            return

        if isinstance(param_waarde,bool): parameter_type = Parameter.Type.BOOL
        elif isinstance(param_waarde,float): parameter_type = Parameter.Type.DOUBLE
        elif isinstance(param_waarde,int): parameter_type = Parameter.Type.INTEGER
        else:
            self.root.after(0,lambda: self.update_ui_log(f"[PARAMETER FOUT] Niet ondersteund type voor '{param_naam}'."))
            return

        parameter = Parameter(param_naam,parameter_type,param_waarde)
        future = client.set_parameters([parameter])
        future.add_done_callback(lambda completed_future: self._handle_remote_param_response(completed_future,node_naam,param_naam,param_waarde))

    def _handle_remote_param_response(self,future,node_naam,naam,waarde):
        try:
            response = future.result()

            if response is None or not response.results:
                self.root.after(0,lambda: self.update_ui_log(f"[PARAMETER FOUT] Geen resultaat ontvangen voor '{naam}'."))
                return

            result = response.results[0]

            if result.successful:
                self.get_logger().info(f"Parameter '{naam}' aangepast naar {waarde} op /{node_naam}")
                self.root.after(0,lambda: self.update_ui_log(f"[HMI] {naam} ingesteld op {waarde}"))
            else:
                reason = result.reason or "Onbekende reden"
                self.root.after(0,lambda: self.update_ui_log(f"[PARAMETER FOUT] '{naam}' geweigerd: {reason}"))

        except Exception as exception:
            self.get_logger().error(f"Parameter aanpassen mislukt: {exception}")
            self.root.after(0,lambda: self.update_ui_log(f"[PARAMETER FOUT] {exception}"))

    # =====================================================
    # BUTTON INTERACTIES & VISUELE COUNTDOWN LOGICA
    # =====================================================

    def _start_countdown_timer(self, resterende_tijd, target_knop, originele_tekst, callback_functie):
        if resterende_tijd > 0:
            self.is_countdown_active = True
            target_knop.config(text=f"{originele_tekst} ({resterende_tijd}s)")
            self.root.after(1000, lambda: self._start_countdown_timer(resterende_tijd - 1, target_knop, originele_tekst, callback_functie))
        else:
            self.is_countdown_active = False
            target_knop.config(text=originele_tekst)
            callback_functie()

    def toggle_turn_on(self):
        if self.is_countdown_active: return
        self.ui_start_stop_active, self.ui_reset_pressed = True, False
        self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {int(self.snelheid_laatste * 100)}%)")
        self.slider_snelheid.set(int(self.snelheid_laatste * 100))
        self.turn_on.config(bg="green", activebackground="lightgreen")
        self.turn_off.config(bg="lightgray")
        self.reset.config(bg="lightgray")
        self.btn_home.config(state="normal")
        
        self.update_ui_log(f"[HMI] Systeem aangezet (ON). Snelheid scaling: {int(self.snelheid_laatste * 100)}%")
        self.publiceer_start_stop(True)

    def toggle_turn_off(self):
        if self.is_countdown_active: return
        self._sla_huidige_snelheid_op()
        self.ui_start_stop_active = self.ui_reset_pressed = self.ui_training_inference_active = False

        self._wissel_ui_modus(naar_training=False)

        self.slider_snelheid.set(5) 
        self.slider_snelheid.config(state="disabled", label=f"Snelheid (Laatste: {int(self.snelheid_laatste * 100)}%)")
        self.slider_versnelling.config(state="disabled", label=f"Versnelling (Laatste: {int(self.versnelling_laatste * 100)}%)")

        self.turn_on.config(state="normal", bg="lightgray")
        self.turn_off.config(state="normal", bg="red", activebackground="pink")
        self.reset.config(bg="lightgray")
        self.training.config(bg="blue", fg="white", state="normal")
        self.btn_home.config(state="normal")

        self.update_ui_log("[HMI] Systeem uitgezet (OFF).")
        self.publiceer_start_stop(False)

    def toggle_ui_training_inference(self):
        if self.is_countdown_active: return
        self.ui_training_inference_active = not self.ui_training_inference_active
        self.ui_start_stop_active = self.ui_reset_pressed = False

        if self.ui_training_inference_active:
            self._wissel_ui_modus(naar_training=True)
            self.training.config(bg="blue", activebackground="purple")
            self.reset.config(bg="orange")
            self.btn_home.config(state="disabled")

            self.slider_snelheid.config(state="normal", label=f"Snelheid (Actief: {int(self.snelheid_laatste * 100)}%)")
            self.slider_versnelling.config(state="normal", label=f"Versnelling (Actief: {int(self.versnelling_laatste * 100)}%)")
            self.slider_versnelling.set(int(self.versnelling_laatste * 100))

            self.update_ui_log("[HMI] Training Modus Actief.")
        else:
            # Sla de schaal op (omgerekend naar float)
            self.versnelling_laatste = self.slider_versnelling.get() / 100.0
            self.toggle_turn_off()

        success = self.call_training_service_async(self.ui_training_inference_active,self._handle_training_response)
        if not success:
            self.ui_training_inference_active = (not self.ui_training_inference_active)

    def _handle_training_response(self, future):
        try:
            response = future.result()

            if not response.success:
                message = (response.message or "Training mode geweigerd")

                self.root.after(0,lambda: self.update_ui_log(f"[TRAINING FOUT] {message}"),)

                self.ui_training_inference_active = (not self.ui_training_inference_active)

                self.root.after(0,self._herstel_ui_na_training_fout)
                return

            self.root.after(0,lambda: self.update_ui_log(f"[HMI] {response.message}"))

        except Exception as exception:
            self.get_logger().error(f"Training service call mislukt: {exception}")

    def _herstel_ui_na_training_fout(self):
        self._wissel_ui_modus(naar_training=self.ui_training_inference_active)
        self.btn_home.config(state="disabled" if self.ui_training_inference_active else "normal")

    def toggle_ui_reset(self):
        if self.is_countdown_active: return
        self.pending_reset = True
        self._sla_huidige_snelheid_op()
        was_in_training = self.ui_training_inference_active
        self.ui_reset_pressed, self.ui_start_stop_active = True, False
        
        self.slider_snelheid.set(5)
        self._set_buttons_state("disabled")
        self.reset.config(bg="yellow")
        
        self.update_ui_log("[HMI] Reset aangevraagd. Interface geblokkeerd voor 3s...")

        self._start_countdown_timer(
            resterende_tijd=3, 
            target_knop=self.reset, 
            originele_tekst="RESET", 
            callback_functie=lambda: self._verstuur_reset_service(was_in_training)
        )

    def _verstuur_reset_service(self, was_in_training):
        success = self.call_trigger_service_async(self.cli_reset,lambda future: self._handle_reset_response(future,was_in_training,))
        if not success:
            self.pending_reset = False
            self._herstel_na_reset(was_in_training)

    def _handle_reset_response(self,future,was_in_training,):
        try:
            response = future.result()

            if not response.success:
                self.pending_reset = False
                self.root.after(0,lambda: self.update_ui_log(f"[RESET FOUT] {response.message}"))

                self.root.after(0,lambda: self._herstel_na_reset(was_in_training))
                return

            self.root.after(0,lambda: self.update_ui_log("[HMI] Reset geaccepteerd. ""Wachten tot de controller stand-by meldt."))

        except Exception as exception:
            self.pending_reset = False
            self.get_logger().error(
                f"Reset service call mislukt: {exception}"
            )

            self.root.after(
                0,
                lambda: self._herstel_na_reset(
                    was_in_training
                ),
            )

    def trigger_ui_retry(self):
        if self.is_countdown_active: return
        self.pending_retry = True
        self.update_ui_log("[HMI] Retry ingedrukt. Interface geblokkeerd voor 3s...")
        
        self._set_buttons_state("disabled")
        self.retry.config(bg="orange")
        
        self._start_countdown_timer(
            resterende_tijd=3,
            target_knop=self.retry,
            originele_tekst="RETRY",
            callback_functie=self._verstuur_retry_service
        )

    def trigger_manipulator_home(self):
        if self.is_countdown_active: 
            return
        
        success = self.call_trigger_service_async(self.cli_move_home,self._handle_move_home_response)

        if success: 
            self.update_ui_log("[HMI] HOME aangevraagd.")

    def _handle_move_home_response(self,future):
        try:
            response = future.result()
            if response.success:
                self.root.after(0,lambda: self.update_ui_log(f"[HMI] {response.message}"))
            else:
                self.root.after(0,lambda: self.update_ui_log(f"[HOME FOUT] {response.message}"))
        except Exception as exception:
            self.get_logger().error(f"Move-home service call mislukt: {exception}")
            self.root.after(0,lambda: self.update_ui_log(f"[HOME FOUT] {exception}"))

    def _verstuur_retry_service(self):
        success = self.call_trigger_service_async(self.cli_retry,self._handle_retry_response)
        if not success:
            self.pending_retry = False
            self._deblokkeer_na_retry()

    def _handle_retry_response(self, future):
        try:
            response = future.result()

            if not response.success:
                self.pending_retry = False
                self.root.after(0,lambda: self.update_ui_log(f"[RETRY FOUT] {response.message}"))

                self.root.after(0,self._deblokkeer_na_retry,)
                return

            self.root.after(
                0,
                lambda: self.update_ui_log(
                    "[HMI] Retry geaccepteerd. "
                    "Eén sorteercyclus wordt uitgevoerd."
                ),
            )

        except Exception as exception:
            self.pending_retry = False
            self.get_logger().error(f"Retry service call mislukt: {exception}")

            self.root.after(0,self._deblokkeer_na_retry)

    def _deblokkeer_na_retry(self):
        self.pending_retry = False
        self._set_buttons_state("normal")
        self.ui_start_stop_active = False
        self.ui_training_inference_active = True

        self.retry.config(bg="lightgray")
        self.reset.config(bg="orange" if self.ui_training_inference_active else "lightgray")
        self.training.config(bg="blue", fg="white")
        
        self.slider_snelheid.config(label=f"Snelheid (Actief: {int(self.snelheid_laatste * 100)}%)")
        self.slider_versnelling.config(label=f"Versnelling (Actief: {int(self.versnelling_laatste * 100)}%)")
        
        self._wissel_ui_modus(naar_training=True)
        self.update_ui_log("[HMI] Sorteercyclus voltooid. Interface weer beschikbaar.")

    # =====================================================
    # SLIDERS LOGICA (Stuurt parameters naar REMOTE node)
    # =====================================================

    def update_snelheid(self, event):
        if self.is_countdown_active or not (self.ui_start_stop_active or self.ui_training_inference_active): return
        val_percentage = int(self.slider_snelheid.get())
        
        # BEGRENZING: Mag niet sneller dan 50%
        if val_percentage > 50:
            val_percentage = 50
            self.slider_snelheid.set(50) # Forceer slider terug naar 50%
            self.update_ui_log("[HMI WAARSCHUWING] Snelheid begrensd op maximaal 50%!")

        val_float = val_percentage / 100.0
        if val_float > 0.05: self.snelheid_laatste = val_float

        self.slider_snelheid.config(label=f"Snelheid (Actief: {val_percentage}%)")
        self.update_ui_log(f"[HMI] Snelheid scaling ingesteld op: {val_percentage}%")

        # Stuur de float-waarde (max 0.50) live naar de manipulator
        self._verstuur_remote_parameter("velocity_scaling", val_float)

    def update_versnelling(self, event):
        if self.is_countdown_active or not self.ui_training_inference_active: return
        val_percentage = int(self.slider_versnelling.get())
        
        # BEGRENZING: Mag niet sneller dan 50%
        if val_percentage > 50:
            val_percentage = 50
            self.slider_versnelling.set(50) # Forceer slider terug naar 50%
            self.update_ui_log("[HMI WAARSCHUWING] Versnelling begrensd op maximaal 50%!")

        val_float = val_percentage / 100.0
        self.versnelling_laatste = val_float

        self.slider_versnelling.config(label=f"Versnelling (Actief: {val_percentage}%)")
        self.update_ui_log(f"[HMI] Versnelling scaling ingesteld op: {val_percentage}%")

        # Stuur de float-waarde (max 0.50) live naar de manipulator
        self._verstuur_remote_parameter("acceleration_scaling", val_float)

    def update_remote_threshold(self):
        if self.is_countdown_active: return

        try:
            val_percentage = int(self.entry_threshold.get().strip())
        except ValueError:
            self.update_ui_log("[VISION FOUT] Confidence moet een geheel getal zijn.")
            return

        if not 0 <= val_percentage <= 100:
            self.update_ui_log("[VISION FOUT] Confidence moet tussen 0% en 100% liggen.")
            return

        confidence = val_percentage / 100.0
        self.vision_confidence_laatste = confidence
        self._verstuur_parameter(self.controller_parameter_client,self.controller_node_naam,"vision_confidence",confidence)

    
    def _set_buttons_state(self,state):
        self.turn_on.config(state=state)
        self.turn_off.config(state=state)
        self.reset.config(state=state)
        self.retry.config(state=state)
        self.training.config(state=state)
        self.btn_home.config(state=state)
        self.slider_snelheid.config(state=state)
        self.slider_versnelling.config(state=state)
        self.btn_set_threshold.config(state=state)
        self.entry_threshold.config(state=state)

    def _herstel_na_reset(self, was_in_training=False):
        del was_in_training

        self.pending_reset = False
        self.ui_reset_pressed = False
        self.ui_start_stop_active = False
        self.ui_training_inference_active = False
        self.vorige_vision_objecten = None
        for object_class in self.vaste_klassen:
            self.gesorteerde_tellers[object_class] = 0

            label = self.ui_teller_labels.get(object_class)

            if label is not None:
                label.config(text="0", fg="black")

        self._set_buttons_state("normal")
        self._wissel_ui_modus(naar_training=False)

        self.slider_snelheid.set(5)
        self.slider_snelheid.config(state="disabled",label=("Snelheid "f"(Laatste: {int(self.snelheid_laatste * 100)}%)"))

        self.slider_versnelling.config(state="disabled",label=("Versnelling "f"(Laatste: {int(self.versnelling_laatste * 100)}%)"))

        self.turn_on.config(bg="lightgray")
        self.turn_off.config(bg="red",activebackground="pink")
        self.reset.config(bg="lightgray")
        self.training.config(bg="blue",fg="white",state="normal")

        self.update_ui_log("[HMI] Reset voltooid. Controller staat stand-by. Tellers zijn gewist.")

    def _verwerk_vision_telling(self,object_class,aantal_verdwenen,totaal):
        self.update_ui_log(
            f"[TELLER] {aantal_verdwenen}x "
            f"{object_class.upper()} verdwenen. "
            f"Gesorteerd totaal: {totaal}"
        )

        self.update_tkinter_teller_display(
            object_class,
            totaal
        )

    def update_tkinter_teller_display(self,object_class,aantal):
        label = self.ui_teller_labels.get(object_class)

        if label is None:
            return

        label.config(
            text=str(aantal),
            fg="green"
        )

        self.root.after(1000, lambda klasse=object_class:  self._reset_label_kleur(klasse))

    def _reset_label_kleur(self, object_class):
        label = self.ui_teller_labels.get(object_class)

        if label is not None:
            label.config(fg="black")

    def update_ui_log(self, msg):
        self.log.config(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def _sla_huidige_snelheid_op(self):
        if self.ui_start_stop_active and self.slider_snelheid.get() > 5:
            self.snelheid_laatste = self.slider_snelheid.get() / 100.0

    def _wissel_ui_modus(self,naar_training=False):
        self.turn_on.pack_forget()
        self.turn_off.pack_forget()
        self.reset.pack_forget()
        self.retry.pack_forget()
        self.btn_home.pack_forget()

        if naar_training:
            self.reset.pack(side=tk.LEFT,padx=(0,5))
            self.retry.pack(side=tk.LEFT,padx=(0,5))
            self.btn_home.pack(side=tk.LEFT)
            self.slider_versnelling.pack(side=tk.LEFT,padx=5)
        else:
            self.slider_versnelling.pack_forget()
            self.turn_on.pack(side=tk.LEFT,padx=(0,5))
            self.turn_off.pack(side=tk.LEFT,padx=(0,5))
            self.reset.pack(side=tk.LEFT,padx=(0,5))
            self.btn_home.pack(side=tk.LEFT)

        self.root.update_idletasks()

    def marked_image_callback(self, msg: RosImage):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exception:
            self.get_logger().error(f"Marked image conversie mislukt: {exception}")
            return

        self.root.after(0, lambda frame=frame: self._update_camera_panel(frame))


    def _update_camera_panel(self, frame):
        frame = cv2.resize(frame, (320, 240))
        opencv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        captured_image = Image.fromarray(opencv_image)
        photo_image = ImageTk.PhotoImage(image=captured_image)

        self.camera_label.photo_image = photo_image
        self.camera_label.configure(image=photo_image)

    def close_hardware(self):
        pass


# =====================================================
# MAIN EXECUTION
# =====================================================

def main(args=None):
    rclpy.init(args=args)  # Initialiseer ROS2

    root = tk.Tk()  # Maak Tkinter root window
    ui = HumanInterface(root)  # Maak HMI node + GUI

    executor = SingleThreadedExecutor()  # Maak expliciete ROS2 executor
    executor.add_node(ui)  # Voeg HMI node toe aan executor

    shutdown_requested = threading.Event()  # Houd bij of shutdown al gestart is

    def ros_spin():
        try:
            executor.spin()  # Draai ROS2 callbacks in aparte thread
        except ExternalShutdownException:
            pass  # Normaal bij afsluiten
        except Exception as exception:
            ui.get_logger().error(f"ROS spin-thread gestopt door fout: {exception}")  # Log echte fouten

    ros_thread = threading.Thread(
        target=ros_spin,
        daemon=False,  # Niet daemon, zodat we gecontroleerd kunnen joinen
    )
    ros_thread.start()  # Start ROS2 spin-thread

    def close_application():
        if shutdown_requested.is_set():  # Voorkom dubbele shutdown door meerdere Ctrl+C/window-close events
            return

        shutdown_requested.set()  # Markeer shutdown als gestart

        try:
            ui.close_hardware()  # Sluit eventuele hardware of resources
        except Exception as exception:
            ui.get_logger().warning(f"Hardware sluiten mislukt: {exception}")  # Log maar sluit door

        try:
            executor.remove_node(ui)  # Haal node uit executor voordat node vernietigd wordt
        except Exception:
            pass  # Niet kritisch tijdens shutdown

        try:
            executor.shutdown()  # Stop executor zodat spin-thread netjes eindigt
        except Exception:
            pass  # Niet kritisch tijdens shutdown

        try:
            if rclpy.ok():  # Controleer of ROS2 nog actief is
                rclpy.shutdown()  # Sluit ROS2 context af
        except Exception:
            pass  # Niet kritisch tijdens shutdown

        try:
            ui.destroy_node()  # Vernietig HMI node
        except Exception:
            pass  # Niet kritisch tijdens shutdown

        try:
            if ros_thread.is_alive():  # Wacht kort tot spin-thread klaar is
                ros_thread.join(timeout=1.0)
        except RuntimeError:
            pass  # Kan gebeuren als join vanuit dezelfde thread zou komen

        try:
            root.destroy()  # Sluit Tkinter window
        except tk.TclError:
            pass  # Window was mogelijk al gesloten

    root.protocol(
        "WM_DELETE_WINDOW",
        close_application,
    )

    try:
        root.mainloop()  # Start Tkinter eventloop
    except KeyboardInterrupt:
        close_application()  # Sluit netjes af bij Ctrl+C
    finally:
        close_application()  # Zorg dat shutdown altijd één keer geprobeerd wordt


if __name__ == "__main__":
    main()