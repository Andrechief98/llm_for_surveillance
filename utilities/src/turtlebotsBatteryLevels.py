#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import BatteryState
from tkinter import *
import threading

class BatteryGUI:
    def __init__(self):
        rospy.init_node('battery_gui_node', anonymous=True)

        # Publisher per ciascun robot
        self.pub_robot_1 = rospy.Publisher('/turtlebot3_1/battery_state', BatteryState, queue_size=10)
        self.pub_robot_2 = rospy.Publisher('/turtlebot3_2/battery_state', BatteryState, queue_size=10)

        # Valori iniziali (in percentuale)
        self.battery_1 = 100
        self.battery_2 = 100

        # Avvia la GUI in un thread separato
        gui_thread = threading.Thread(target=self.init_gui)
        gui_thread.daemon = True
        gui_thread.start()

        # Loop di pubblicazione ROS (1Hz)
        self.publish_loop()

    def publish_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.pub_robot_1.publish(self.create_battery_msg(self.battery_1))
            self.pub_robot_2.publish(self.create_battery_msg(self.battery_2))
            rate.sleep()

    def create_battery_msg(self, percentage):
        msg = BatteryState()
        msg.header.stamp = rospy.Time.now()
        msg.voltage = 11.1
        msg.current = -0.5
        msg.capacity = 1800
        msg.charge = percentage / 100.0 * msg.capacity
        msg.percentage = percentage / 100.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        return msg

    def init_gui(self):
        # Creazione della finestra principale
        root = Tk()
        root.title("Battery Control")
        root.configure(bg="white")
        root.geometry("400x300")
        root.resizable(False, False)

        # Imposta il frame principale con padding e usa il grid layout
        main_frame = Frame(root, bg="white")
        main_frame.pack(expand=True, fill=BOTH, padx=10, pady=10)

        # Configura le colonne del main_frame per avere peso uguale e centrare il contenuto
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=1)

        # Frame per il primo robot
        frame_robot1 = Frame(main_frame, bg="white", bd=1, relief=GROOVE)
        frame_robot1.grid(row=0, column=0, padx=10, pady=10, sticky=N+S+E+W)

        # Slider verticale per TurtleBot3_1
        self.slider_robot1 = Scale(frame_robot1, from_=100, to=0, orient=VERTICAL,
                                   length=200, showvalue=False, bg="white", highlightthickness=0)
        self.slider_robot1.set(self.battery_1)
        self.slider_robot1.config(command=self.update_battery_1)
        self.slider_robot1.pack(pady=5)

        # Visualizzazione del valore corrente per robot 1
        self.label_robot1_val = Label(frame_robot1, text=f"{self.battery_1}%", font=("Helvetica", 12), bg="white", fg="black")
        self.label_robot1_val.pack(pady=5)

        # Nome del robot 1 sotto lo slider
        label_robot1_name = Label(frame_robot1, text="TurtleBot3_1", font=("Helvetica", 12), bg="white", fg="black")
        label_robot1_name.pack(pady=(5, 10))

        # Frame per il secondo robot
        frame_robot2 = Frame(main_frame, bg="white", bd=1, relief=GROOVE)
        frame_robot2.grid(row=0, column=1, padx=10, pady=10, sticky=N+S+E+W)

        # Slider verticale per TurtleBot3_2
        self.slider_robot2 = Scale(frame_robot2, from_=100, to=0, orient=VERTICAL,
                                   length=200, showvalue=False, bg="white", highlightthickness=0)
        self.slider_robot2.set(self.battery_2)
        self.slider_robot2.config(command=self.update_battery_2)
        self.slider_robot2.pack(pady=5)

        # Visualizzazione del valore corrente per robot 2
        self.label_robot2_val = Label(frame_robot2, text=f"{self.battery_2}%", font=("Helvetica", 12), bg="white", fg="black")
        self.label_robot2_val.pack(pady=5)

        # Nome del robot 2 sotto lo slider
        label_robot2_name = Label(frame_robot2, text="TurtleBot3_2", font=("Helvetica", 12), bg="white", fg="black")
        label_robot2_name.pack(pady=(5, 10))

        root.mainloop()

    def update_battery_1(self, val):
        self.battery_1 = int(val)
        self.label_robot1_val.config(text=f"{val}%")

    def update_battery_2(self, val):
        self.battery_2 = int(val)
        self.label_robot2_val.config(text=f"{val}%")

if __name__ == '__main__':
    try:
        BatteryGUI()
    except rospy.ROSInterruptException:
        pass
