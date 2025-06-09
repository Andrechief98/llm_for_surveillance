#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

# Dizionario globale per salvare le ultime immagini ricevute
latest_images = {}
bridge = CvBridge()

def image_callback(msg, topic):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
        latest_images[topic] = cv_image
    except Exception as e:
        rospy.logerr("Errore nella conversione per {}: {}".format(topic, e))

def visualize_cameras(topic_list):
    """
    Visualizza in tempo reale le immagini provenienti da più topic di camera.
    
    Args:
        topic_list (list): Lista di stringhe, ciascuna rappresentante il topic di una camera (es. 'camera/camera_sensor1/image_raw').
    """
    rospy.init_node('multi_camera_viewer', anonymous=True)
    
    # Sottoscrizione dinamica per ciascun topic
    for topic in topic_list:
        rospy.Subscriber(topic, Image, lambda msg, t=topic: image_callback(msg, t))
        cv2.namedWindow(topic, cv2.WINDOW_NORMAL)
        latest_images[topic] = None

    rospy.loginfo("Visualizzazione delle immagini dai topic: {}".format(topic_list))
    
    rate = rospy.Rate(30)  # 30 Hz
    while not rospy.is_shutdown():
        # Lista per tenere traccia delle finestre aperte
        windows_open = []
        for topic in topic_list:
            # Verifica lo stato della finestra: se è stata chiusa, cv2.getWindowProperty restituisce un valore negativo
            if cv2.getWindowProperty(topic, cv2.WND_PROP_VISIBLE) >= 1:
                windows_open.append(topic)
                if latest_images.get(topic) is not None:
                    cv2.imshow(topic, latest_images[topic])
        # Se nessuna finestra è aperta, esce dal ciclo
        if not windows_open:
            rospy.loginfo("Tutte le finestre sono state chiuse. Terminazione del nodo.")
            break

        # Esce se viene premuto ESC
        if cv2.waitKey(30) & 0xFF == 27:
            break

        rate.sleep()
    
    cv2.destroyAllWindows()
    return

if __name__ == '__main__':
    # Esempio di lista di topic da visualizzare
    #topics = ["/gazebo_multi_camera/camera/camera_sensor1/image_raw","/gazebo_multi_camera/camera/camera_sensor2/image_raw"]
    
    topics = sys.argv[1:]
    visualize_cameras(topics)
    
