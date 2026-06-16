import threading
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
# Importeer je zojuist gemaakte UI klasse
from hmi_interface_versie6 import HumanInterface



class HmiControllerNode(Node):
    def __init__(self):
        super().__init__('hmi_controller_node')
        self.get_logger().info("HMI Controller Node succesvol opgestart.")

        # --- ROS2 PUBLISHERS (Voorbeeld) ---
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)
        self.speed_publisher = self.create_publisher(Int32, 'robot_speed', 10)

        # --- TKINTER EN UI INITIALISATIE ---
        self.root = tk.Tk()

        # We stoppen de functies van de controller in een 'map' (dictionary)
        # Zo weet de UI welke functies hij moet uitvoeren bij een klik.
        callbacks = {
            'on': self.controller_turn_on,
            'off': self.controller_turn_off,
            'reset': self.controller_reset,
            'speed': self.controller_set_speed
        }

        # Maak de UI aan en geef de root en de functies mee
        self.ui = HumanInterface(self.root, callbacks)

        # --- MULTI-THREADING VOOR ROS2 ---
        # We spinnen ROS2 in een aparte thread zodat Tkinter en ROS2 tegelijk ademen
        self.ros_thread = threading.Thread(target=self.run_ros_loop, daemon=True)
        self.ros_thread.start()

        # Start de Tkinter GUI loop (dit blokkeert de hoofdthread, wat de bedoeling is)
        self.root.mainloop()

    def run_ros_loop(self):
        """Draait op de achtergrond om ROS2 signalen te verwerken."""
        rclpy.spin(self)

    # --- CONTROLLER ACTIONS (Jouw printjes en ROS2 logica!) ---
    def controller_turn_on(self):
        # Dit is het printje dat je miste! Dit verschijnt nu netjes in je terminal.
        print("Turn ON button pressed -> Ontvangen in Controller!")
        self.get_logger().info("ROS2: Systeem wordt ingeschakeld...")
        
        # Stuur daadwerkelijk een ROS2 bericht naar je robot
        msg = String()
        msg.data = "ON"
        self.state_publisher.publish(msg)

    def controller_turn_off(self):
        print("Turn OFF button pressed -> Ontvangen in Controller!")
        self.get_logger().info("ROS2: Systeem wordt uitgeschakeld...")
        
        msg = String()
        msg.data = "OFF"
        self.state_publisher.publish(msg)

    def controller_reset(self):
        print("RESET button pressed -> Ontvangen in Controller!")
        self.get_logger().info("ROS2: Reset commando verzonden!")
        
        msg = String()
        msg.data = "RESET"
        self.state_publisher.publish(msg)

    def controller_set_speed(self, snelheid_waarde):
        print(f"Snelheid gewijzigd -> Ontvangen in Controller! Waarde: {snelheid_waarde}")
        self.get_logger().info(f"ROS2: Snelheid ingesteld op {snelheid_waarde}")
        
        msg = Int32()
        msg.data = int(snelheid_waarde)
        self.speed_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Start de controller (hiermee start ook direct de GUI)
    node = HmiControllerNode()
    
    # Zodra het Tkinter venster wordt gesloten, sluiten we ROS2 netjes af
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()