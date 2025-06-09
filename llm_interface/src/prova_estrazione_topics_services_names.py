import rospy
import rosgraph



if __name__ == "__main__":
    topics = rospy.get_published_topics()
    topics_names = [topic[0] for topic in topics]

    print("TOPICS")
    for name in topics_names:
        print(name)
   

    master = rosgraph.Master('/rospy')
    services = master.getSystemState()[2]  # [publishers, subscribers, services]

    # Estrai solo i nomi dei servizi
    service_names = [s[0] for s in services]

    # Stampa i servizi
    print("\nSERVICES")
    for name in service_names:
        print(name)