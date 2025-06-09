#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk

class ROSButtonPublisher:
    def __init__(self):
        # ROS node initialization
        rospy.init_node('button_publisher', anonymous=True)
        self.pub = rospy.Publisher('/chatter', String, queue_size=10)
        self.scenario_number = rospy.get_param("/scenario")
        
        # User interface
        self.root = tk.Tk()
        self.root.title("Man-in-the-loop final decision")
        self.root.geometry("600x250")
        self.root.resizable(False, False)
        self.root.configure(bg="#f5f5f5")  # Colore di sfondo chiaro

        # User interface style
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Green.TButton", font=("Helvetica", 14), padding=10, background="#4caf50", foreground="white")
        style.map("Green.TButton", background=[("active", "#45a049")])

        style.configure("Red.TButton", font=("Helvetica", 14), padding=10, background="#f44336", foreground="white")
        style.map("Red.TButton", background=[("active", "#d32f2f")])

        style.configure("TLabel", font=("Helvetica", 16), background="#f5f5f5", foreground="#333333")

        # Buttons
        if self.scenario_number == 1:
            robot_to_send = "locobot_2"
            activated_sensors_list = ["Camera_1", "Lidar_3", "Lidar_4"]

        elif self.scenario_number == 2:
            robot_to_send = "locobot_1"
            activated_sensors_list =["Camera_1", "Lidar_7", "Lidar_2"]

        elif  self.scenario_number ==3:
            robot_to_send = "locobot_1"
            activated_sensors_list = ["Camera_2", "Lidar_6", "Lidar_1"]

        else:
            rospy.loginfo("Scenario not implemented")
            return

        activated_sensor = ", ".join(activated_sensors_list)
        self.label_var = tk.StringVar(value=f"Activation of {activated_sensor}.\nDo you want to send the {robot_to_send} to check the room?")
        self.label = ttk.Label(self.root, textvariable=self.label_var, anchor="center", justify="center")
        self.label.pack(pady=30)

        # Contenitore per i bottoni (frame)
        button_frame = tk.Frame(self.root, bg="#f5f5f5")
        button_frame.pack(pady=10)

        # Crea due bottoni affiancati: verde "Yes" e rosso "No"
        self.button_yes = ttk.Button(button_frame, text="Yes", style="Green.TButton", command=self.send_yes_message)
        self.button_yes.pack(side="left", padx=20)

        self.button_no = ttk.Button(button_frame, text="No", style="Red.TButton", command=self.send_no_message)
        self.button_no.pack(side="left", padx=20)

    def send_yes_message(self):
        message = "yes"
        rospy.loginfo("Message sent: Yes")
        self.pub.publish(message)
        self.label_var.set("Message sent: Yes")

    def send_no_message(self):
        message = "no"
        rospy.loginfo("Message sent: No")
        self.pub.publish(message)
        self.label_var.set("Message sent: No")
        self.root.destroy()  # Chiude la finestra

    def run(self):
        # Avvia la GUI
        try:
            self.root.mainloop()
        except rospy.ROSInterruptException:
            rospy.loginfo("Nodo interrotto.")

if __name__ == '__main__':
    rospy.sleep(10)
    try:
        ros_button_publisher = ROSButtonPublisher()
        ros_button_publisher.run()
    except rospy.ROSInterruptException:
        pass
