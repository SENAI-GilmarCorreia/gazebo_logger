#include <cmath>  // Para garantir que outras funções matemáticas estejam acessíveis

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <boost/filesystem.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Camera.hh>

namespace gazebo {
class LoggerPlugin : public WorldPlugin {
   private:
    std::ofstream csvFile;
    std::chrono::high_resolution_clock::time_point lastTime, initialTime;
    int frameCount = 0;
    float fps = 0.0f;
    char sep = ';';
    std::string model_name = "";

    event::ConnectionPtr updateConnection;

    // Helper function to get the current date and time for the CSV filename
    std::string getCurrentDateTime() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm* localTime = std::localtime(&now_time);
        std::ostringstream oss;
        oss << std::put_time(localTime, "%Y-%m-%d_%H-%M-%S");
        return oss.str();
    }

   public:
    LoggerPlugin() : WorldPlugin() {
        frameCount = 0;
        fps = 0.0f;
    }

    ~LoggerPlugin() {
        if (csvFile.is_open()) {
            csvFile.close();
        }
    }   

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
        
        // Save logs to the default .gazebo path
        std::string homeDir;
        #ifdef _WIN32
        homeDir = std::getenv("USERPROFILE");
        #else
        homeDir = std::getenv("HOME");
        #endif
        std::string gazeboPath = std::string(homeDir) + "/.gazebo/logger/";

        // Create logs directory if it doesn't exist
        #ifdef _WIN32
        std::filesystem::path dir(gazeboPath.c_str());
        if (!std::filesystem::exists(dir)) {
            std::filesystem::create_directories(dir);
        }
        #else
        boost::filesystem::path dir(gazeboPath.c_str());
        if (!boost::filesystem::exists(dir)) {
            boost::filesystem::create_directories(dir);
        }
        #endif

        // Create CSV file in the .gazebo/logger/ directory
        std::string dateTime = getCurrentDateTime();
        std::string fileName = gazeboPath + "gazeboLogger_" + dateTime + ".csv";

        // Open CSV file
        this->csvFile.open(fileName);
        if (!this->csvFile.is_open()) {
            std::cerr << "Unable to open CSV file: " << fileName << std::endl;
        }

        // Write CSV headers
        this->csvFile << "Frame" << this->sep 
                      << "GazeboServer - Step Size (ms)" << this->sep
                      << "GazeboServer - Simulation Time (ms)" << this->sep
                      << "GazeboServer - Real Time (ms)" << this->sep
                      << "OS - System Time (ms)" << this->sep
                      << "GazeboServer - RTF" << this->sep
                      << "OS - RTF" << this->sep
                      << "GazeboClient - Render FPS (Hz)" << this->sep
                      << "OS - Plugin FPS (Hz)" << this->sep
                      << "Pose Data" << std::endl;
                      //<< "Collision Count" << std::endl;

        // Initialize time points for FPS calculation
        this->initialTime = std::chrono::high_resolution_clock::now();
        this->lastTime = std::chrono::high_resolution_clock::now();

        // Connect to the world update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&LoggerPlugin::OnUpdate, this, _world)
        );

    }

    void OnUpdate(physics::WorldPtr _world) {
        // Increment frame count
        this->frameCount++;

        // Get current simulation time
        double simTime = _world->SimTime().Double() * 1000.0;  // convert to ms
        double realTime = _world->RealTime().Double() * 1000; // convert to ms

        // Get system time since plugin start
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto systemTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - initialTime).count();

        // Calculate FPS if at least 1 second has passed
        std::chrono::duration<float> elapsedTime = currentTime - this->lastTime;
        if (elapsedTime.count() >= 1.0f) {
            this->fps = this->frameCount / elapsedTime.count();
            this->frameCount = 0;
            this->lastTime = currentTime;
        }

        double rtf = simTime/realTime;

        // Get the current render rate (frames per second)
        // std::string worldName = _world->Name();
        // gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene(worldName);
        // if (!scene)
        // {
        //     std::cerr << "Cena não foi carregada corretamente." << std::endl;
        //     return;
        // }

        // // Obtém as câmeras conectadas à cena
        // gazebo::rendering::CameraPtr camera = scene->GetCamera("gzclient_camera");
        // double renderRate = camera->RenderRate();

        // Log active objects in the simulation
        std::string objectsData = "[";

        // Get all models in the world
        auto models = _world->Models();
        for (const auto& model : models){
            objectsData += "{ \"name\": \"" + model->GetName() + "\", ";
            ignition::math::Pose3d pose = model->WorldPose();
            objectsData += "\"pose\": [" + std::to_string(pose.Pos().X()) + ", " 
                                         + std::to_string(pose.Pos().Y()) + ", "
                                         + std::to_string(pose.Pos().Z()) + ", "
                                         + std::to_string(pose.Rot().W()) + ", "
                                         + std::to_string(pose.Rot().X()) + ", "
                                         + std::to_string(pose.Rot().Y()) + ", "
                                         + std::to_string(pose.Rot().Z()) + "]}";

            objectsData += ", ";
        }
        objectsData += "]";

        // Get simulation step size
        double stepSize = _world->Physics()->GetMaxStepSize() * 1000;  // Convert to ms

        // Write data to CSV file
        if (this->csvFile.is_open()) {
            this->csvFile << std::to_string(this->frameCount) << this->sep
                          << std::to_string(stepSize) << this->sep
                          << std::to_string(simTime) << this->sep
                          << std::to_string(realTime) << this->sep
                          << std::to_string(systemTime_ms) << this->sep
                          << "" << this->sep //<< std::to_string(rtf) << this->sep
                          << "" << this->sep // os_rtf
                          << "" << this->sep // render fps
                          << std::to_string(this->fps) << this->sep
                          << objectsData << std::endl;
                          //<< std::to_string(collisionCount) << std::endl;
        }
    }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(LoggerPlugin)
}  // namespace gazebo
