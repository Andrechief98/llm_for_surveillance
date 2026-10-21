#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Box.hh>
#include <ros/ros.h>
#include <gazebo_plugins/ModelBoundingBoxes.h>
#include <gazebo_plugins/ActorVelocities.h>  
#include <random>
#include <string>
#include <map>

namespace gazebo
{
  class BoundingBoxPlugin : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) override
    {
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized!");
        return;
      }

      this->world = _world;
      this->nh.reset(new ros::NodeHandle("bounding_box_plugin"));
      this->bb_pub = this->nh->advertise<gazebo_plugins::ModelBoundingBoxes>(
          "/gazebo/bounding_boxes", 10);
      // Publisher per le velocità degli actor
      this->vel_pub = this->nh->advertise<gazebo_plugins::ActorVelocities>(
          "/gazebo/actor_velocities", 10);

      // Initialize random number generator
      std::srand(std::time(0));
      
      ROS_INFO("BoundingBoxPlugin loaded. Publishing bounding boxes and actor velocities...");

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&BoundingBoxPlugin::OnUpdate, this));
    }

  private:
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Publisher bb_pub;
    ros::Publisher vel_pub;

    std::map<std::string, ignition::math::AxisAlignedBox> actorBoxCache;
    // Cache per la velocità: per ogni actor, memorizziamo la posizione e il tempo dell'ultimo aggiornamento
    std::map<std::string, ignition::math::Vector3d> lastPositions;
    std::map<std::string, common::Time> lastUpdateTimes;

    ignition::math::AxisAlignedBox GenerateRandomBoundingBox(const ignition::math::Pose3d& pose)
    {
      // Generate random dimensions (0.5-1.5m width/depth, 1.0-2.0m height)
      double width = 0.5 + (std::rand() % 100) / 100.0;
      double depth = 0.5 + (std::rand() % 100) / 100.0;
      double height = 1.0 + (std::rand() % 100) / 100.0;

      // Create bounding box relative to actor position
      ignition::math::Vector3d min(
          pose.Pos().X() - width/2,
          pose.Pos().Y() - depth/2,
          pose.Pos().Z()
      );
      
      ignition::math::Vector3d max(
          pose.Pos().X() + width/2,
          pose.Pos().Y() + depth/2,
          pose.Pos().Z() + height
      );

      return ignition::math::AxisAlignedBox(min, max);
    }

    void OnUpdate()
    {
      gazebo_plugins::ModelBoundingBoxes bb_msg;
      gazebo_plugins::ActorVelocities vel_msg;

      // Ottieni il tempo simulato corrente
      common::Time currentTime = this->world->SimTime();

      for (const auto &model : this->world->Models())
      {
        std::string modelName = model->GetName();

        // Skip known problematic models
        if (modelName == "sun" || modelName == "ground_plane" || modelName == "room") {
          continue;
        }

        // Gestione degli actor (modelli dinamici)
        if (modelName.find("actor") != std::string::npos)
        {
          // Bounding box per actor
          if (actorBoxCache.find(modelName) == actorBoxCache.end()) 
          {
            // Genera dimensioni casuali (1.0-2.0m range)
            double width = 1.0 + (std::rand() % 100)/100.0;
            double depth = 1.0 + (std::rand() % 100)/100.0;
            double height = 1.5 + (std::rand() % 100)/100.0;
            
            ignition::math::Vector3d size(width, depth, height);
            actorBoxCache[modelName] = ignition::math::AxisAlignedBox(
              -size/2.0,  // min
              size/2.0    // max
            );
          }

          // Ottieni la posa corrente dell'actor
          ignition::math::Pose3d currentPose = model->WorldPose();
          ignition::math::AxisAlignedBox aabb = actorBoxCache[modelName];
          ignition::math::Vector3d offset = currentPose.Pos();
          ignition::math::AxisAlignedBox translatedBox(aabb.Min() + offset, aabb.Max() + offset);

          // Popola il messaggio delle bounding boxes
          bb_msg.name.push_back(modelName);
          bb_msg.min.push_back(ToVector3Msg(translatedBox.Min()));
          bb_msg.max.push_back(ToVector3Msg(translatedBox.Max()));

          // Calcola la velocità in base alla variazione della posizione
          if (lastPositions.find(modelName) != lastPositions.end())
          {
            double dt = (currentTime - lastUpdateTimes[modelName]).Double();
            if (dt > 0)
            {
              ignition::math::Vector3d vel = (currentPose.Pos() - lastPositions[modelName]) / dt;
              // Aggiungi al messaggio delle velocità
              vel_msg.name.push_back(modelName);
              geometry_msgs::Vector3 velVec = ToVector3Msg(vel);
              vel_msg.velocity.push_back(velVec);
            }
          }
          // Aggiorna la cache per questo actor
          lastPositions[modelName] = currentPose.Pos();
          lastUpdateTimes[modelName] = currentTime;
        }
        else  // Per gli altri modelli
        {
          ignition::math::AxisAlignedBox aabb = model->BoundingBox();
          ignition::math::Vector3d min = aabb.Min();
          ignition::math::Vector3d max = aabb.Max();
          if (min.X() > max.X() || min.Y() > max.Y() || min.Z() > max.Z()) {
            ROS_WARN_STREAM("Invalid bounding box for model: " << modelName);
            continue;
          }
          bb_msg.name.push_back(modelName);
          bb_msg.min.push_back(ToVector3Msg(min));
          bb_msg.max.push_back(ToVector3Msg(max));
        }
      }

      // Pubblica le bounding boxes e le velocità degli actor
      this->bb_pub.publish(bb_msg);
      this->vel_pub.publish(vel_msg);
    }

    geometry_msgs::Vector3 ToVector3Msg(const ignition::math::Vector3d& vec)
    {
      geometry_msgs::Vector3 msg;
      msg.x = vec.X();
      msg.y = vec.Y();
      msg.z = vec.Z();
      return msg;
    }
  };

  GZ_REGISTER_WORLD_PLUGIN(BoundingBoxPlugin)
}
