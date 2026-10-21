#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>                // Per i messaggi di tipo std_msgs::String
#include <gazebo_plugins/doorStringCommand.h> // Il file srv si chiama doorStringCommand.srv
#include <boost/thread.hpp>
#include <sstream>
#include <map>
#include <cmath>

namespace gazebo
{
  class DoorControlPlugin : public ModelPlugin
  {
    public:
      DoorControlPlugin() : ModelPlugin() {}

      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        // Inizializza ROS, se non già fatto
        if (!ros::isInitialized())
        {
          int argc = 0;
          char **argv = NULL;
          ros::init(argc, argv, "door_control_plugin",
                    ros::init_options::NoSigintHandler);
        }

        this->model = _model;
        this->modelName = _model->GetName();

        // Legge dalla configurazione SDF i nomi dei giunti che rappresentano la porta
        if (_sdf->HasElement("door_joint"))
        {
          sdf::ElementPtr jointElem = _sdf->GetElement("door_joint");
          while (jointElem)
          {
            std::string jointName = jointElem->Get<std::string>();
            physics::JointPtr joint = this->model->GetJoint(jointName);
            if (joint)
            {
              this->doorJoints[jointName] = joint;
              // Imposta il target iniziale come la posizione attuale
              targetPositions[jointName] = joint->Position(0);
              gzdbg << "Trovato joint della porta: " << jointName << "\n";
            }
            else
            {
              gzerr << "Joint non trovato: " << jointName << "\n";
            }
            jointElem = jointElem->GetNextElement("door_joint");
          }
        }
        else
        {
          gzerr << "Elemento SDF <door_joint> non presente!\n";
        }

        // Legge il parametro per la velocità (m/s)
        if (_sdf->HasElement("speed"))
        {
          speed = _sdf->Get<double>("speed");
        }
        else
        {
          speed = 1.0;
        }

        // Legge i target per i comandi "open" e "close" (se definiti)
        if (_sdf->HasElement("open_target"))
        {
          openTarget = _sdf->Get<double>("open_target");
        }
        else
        {
          openTarget = 1.0;
        }
        if (_sdf->HasElement("close_target"))
        {
          closeTarget = _sdf->Get<double>("close_target");
        }
        else
        {
          closeTarget = 0.0;
        }

        // Legge il parametro per l'intervallo di publishing (in secondi)
        if (_sdf->HasElement("publish_interval"))
        {
          publishInterval = _sdf->Get<double>("publish_interval");
        }
        else
        {
          publishInterval = 0.1;  // default a 0.5 secondi (2 Hz)
        }
        timeSinceLastPub = 0.0;

        // Inizializza il nodo ROS
        this->rosNode.reset(new ros::NodeHandle("door_control_plugin"));

        // Crea un servizio il cui nome include il nome del modello, per controllare questa porta individualmente
        std::string serviceName = "/" + this->modelName + "_control_service";
        rosService = rosNode->advertiseService(serviceName, &DoorControlPlugin::OnServiceCall, this);
        gzdbg << "Servizio per il controllo della porta pubblicato su: " << serviceName << "\n";

        // Inizializza il publisher per lo stato della porta
        // Il topic viene denominato con il nome del modello per evitare conflitti
        std::string stateTopic = "/" + this->modelName + "/door_state";
        statePub = rosNode->advertise<std_msgs::String>(stateTopic, 1);
        gzdbg << "Publisher dello stato della porta pubblicato su: " << stateTopic << "\n";

        // Avvia il thread per gestire la coda ROS
        this->rosQueueThread = boost::thread(boost::bind(&DoorControlPlugin::QueueThread, this));

        // Collega l'update del mondo Gazebo
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DoorControlPlugin::OnUpdate, this));

        lastUpdateTime = this->model->GetWorld()->SimTime();
      }

      // Callback del servizio: riceve "open" o "close" come comando
      public: bool OnServiceCall(gazebo_plugins::doorStringCommand::Request &req,
                                  gazebo_plugins::doorStringCommand::Response &res)
      {
        std::string command = req.command;
        double target = 0.0;

        if (command == "open")
        {
          target = openTarget;
        }
        else if (command == "close")
        {
          target = closeTarget;
        }
        else
        {
          gzerr << "Comando sconosciuto: " << command << "\n";
          res.success = false;
          res.message = "Comando sconosciuto: " + command;
          return true;
        }

        // Imposta il target per ciascun giunto della porta
        for (auto &pair : doorJoints)
        {
          targetPositions[pair.first] = target;
          gzdbg << "Comando " << command << " per il joint: " << pair.first << "\n";
        }
        res.success = true;
        res.message = (command == "open") ? "Door open" : "Door closed";
        return true;
      }

      // Aggiornamento ad ogni iterazione della simulazione: muove gradualmente la porta verso il target
      public: void OnUpdate()
      {
        common::Time currentTime = model->GetWorld()->SimTime();
        double dt = (currentTime - lastUpdateTime).Double();
        lastUpdateTime = currentTime;

        // Aggiornamento posizione di ciascun giunto
        for (auto &pair : doorJoints)
        {
          physics::JointPtr joint = pair.second;
          double currentPos = joint->Position(0);
          double target = targetPositions[pair.first];
          double error = target - currentPos;

          if (std::fabs(error) < 1e-3)
            continue;

          double maxStep = speed * dt;
          double step = std::min(maxStep, std::fabs(error));
          double newPos = currentPos + (error > 0 ? step : -step);
          joint->SetPosition(0, newPos);
        }

        // Determina lo stato attuale della porta.
        // Se tutti i giunti sono vicini al target "open" allora lo stato è "open",
        // se sono vicini al target "close" allora lo stato è "close",
        // altrimenti lo stato è "moving".
        std::string doorState = "moving";
        bool allOpen = true;
        bool allClose = true;
        const double threshold = 1e-2;
        for (auto &pair : doorJoints)
        {
          double currentPos = pair.second->Position(0);
          if (std::fabs(currentPos - openTarget) > threshold)
            allOpen = false;
          if (std::fabs(currentPos - closeTarget) > threshold)
            allClose = false;
        }
        if (allOpen)
          doorState = "open";
        else if (allClose)
          doorState = "close";

        // Aggiornamento del timer per il publishing
        timeSinceLastPub += dt;
        if (timeSinceLastPub >= publishInterval)
        {
          // Pubblica lo stato corrente della porta
          std_msgs::String stateMsg;
          stateMsg.data = doorState;
          statePub.publish(stateMsg);
          // Resetta il timer di publishing
          timeSinceLastPub = 0.0;
        }
      }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private:
      physics::ModelPtr model;
      std::string modelName;
      event::ConnectionPtr updateConnection;
      std::map<std::string, physics::JointPtr> doorJoints;
      std::map<std::string, double> targetPositions;
      double speed;
      double openTarget;
      double closeTarget;
      common::Time lastUpdateTime;

      // Variabili per il controllo della frequenza di publishing
      double publishInterval;
      double timeSinceLastPub;

      // Gestione ROS
      std::unique_ptr<ros::NodeHandle> rosNode;
      ros::ServiceServer rosService;
      ros::CallbackQueue rosQueue;
      boost::thread rosQueueThread;
      ros::Publisher statePub;  // Publisher per lo stato della porta
  };

  GZ_REGISTER_MODEL_PLUGIN(DoorControlPlugin)
}
