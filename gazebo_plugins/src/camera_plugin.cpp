#include <gazebo/rendering/Camera.hh> 

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <unordered_map>

namespace gazebo
{
  class MultiCameraPlugin : public SensorPlugin
  {
  public:
    MultiCameraPlugin() : SensorPlugin() {}
    virtual ~MultiCameraPlugin() {}

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override
    {
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_multi_camera", ros::init_options::NoSigintHandler);
      }
      rosNode.reset(new ros::NodeHandle("gazebo_multi_camera"));

      // Verifica se il sensore è di tipo camera
      auto cameraSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
      if (!cameraSensor)
      {
        gzerr << "Il plugin supporta solo sensori di tipo camera\n";
        return;
      }

      std::string cameraName = _sensor->Name();
      std::string topicName = "camera/" + cameraName + "/image_raw";

      // Crea un publisher ROS per la camera
      imagePublishers[cameraName] = rosNode->advertise<sensor_msgs::Image>(topicName, 1);

      // Registra l'evento di aggiornamento per questa camera
      updateConnections[cameraName] = cameraSensor->ConnectUpdated(
          std::bind(&MultiCameraPlugin::OnUpdate, this, cameraSensor, cameraName));

      cameraSensor->SetActive(true);
      gzmsg << "MultiCameraPlugin caricato per la camera: " << cameraName 
            << " (pubblica su " << topicName << ")\n";
    }

    void OnUpdate(sensors::CameraSensorPtr cameraSensor, const std::string &cameraName)
    {
      if (!cameraSensor)
        return;

      const unsigned char *imageData = cameraSensor->Camera()->ImageData();
      int width = cameraSensor->Camera()->ImageWidth();
      int height = cameraSensor->Camera()->ImageHeight();

      sensor_msgs::Image imageMsg;
      imageMsg.header.stamp = ros::Time::now();
      imageMsg.header.frame_id = cameraName;
      imageMsg.height = height;
      imageMsg.width = width;
      imageMsg.encoding = "rgb8";
      imageMsg.is_bigendian = false;
      imageMsg.step = width * 3;
      imageMsg.data.resize(height * imageMsg.step);
      memcpy(&imageMsg.data[0], imageData, imageMsg.data.size());

      // Pubblica l'immagine della camera corrispondente
      imagePublishers[cameraName].publish(imageMsg);
    }

  private:
    std::unique_ptr<ros::NodeHandle> rosNode;
    std::unordered_map<std::string, ros::Publisher> imagePublishers;
    std::unordered_map<std::string, event::ConnectionPtr> updateConnections;
  };

  GZ_REGISTER_SENSOR_PLUGIN(MultiCameraPlugin)
}
