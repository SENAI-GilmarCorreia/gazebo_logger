#include <cmath>  // Para garantir que outras funções matemáticas estejam acessíveis

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
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
    int totalFrames = 0;
    float fps = 0.0f;
    char sep = ';';
    std::string model_name = "";

    event::ConnectionPtr updateConnection;

    // Helper function to get the latest folder in a directory
    std::string get_latest_folder(const std::string& path) {
        std::string latest_folder;
        std::time_t latest_time = 0;

        for (const auto& entry : std::filesystem::directory_iterator(path)) {
            if (std::filesystem::is_directory(entry)) {
                // Obtém o tempo de modificação do diretório
                auto ftime = std::filesystem::last_write_time(entry);

                // Converte o file_time_type para system_clock::time_point
                auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                    ftime - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());

                // Converte para time_t
                std::time_t folder_time = std::chrono::system_clock::to_time_t(sctp);

                // Verifica se o timestamp é mais recente
                if (folder_time > latest_time) {
                    latest_time = folder_time;
                    latest_folder = entry.path().string();
                }
            }
        }
        
        return latest_folder;
    }

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
        totalFrames = 0;
        fps = 0.0f;
    }

    ~LoggerPlugin() {
        if (csvFile.is_open()) {
            csvFile.close();
        }
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
        
        // Save logs to the default .gazebo path
        // std::string homeDir;
        // #ifdef _WIN32
        // homeDir = std::getenv("USERPROFILE");
        // #else
        // homeDir = std::getenv("HOME");
        // #endif
        // std::string gazeboPath = std::string(homeDir) + "/.gazebo/logger/";

        // Save logs to the default gazebo path
        std::string naadDir;
        naadDir = std::getenv("NAAD_WS_DIR");
        std::string gazeboPath;
        std::string fileName;

        if (std::getenv("NAAD_CONFIG_LOGS")){
            gazeboPath = get_latest_folder(std::string(naadDir) + "/logs");
            gazeboPath += "/gazebo/";

            // Create CSV file in the .gazebo/logger/ directory
            fileName = gazeboPath + "gazeboLogger.csv";
        }
        else{
            gazeboPath = std::string(naadDir) + "/logs/gazebo/";

            // Create CSV file in the .gazebo/logger/ directory
            std::string dateTime = getCurrentDateTime();
            fileName = gazeboPath + "gazeboLogger_" + dateTime + ".csv";
        }

        // Create the directory if it doesn't exist
        boost::filesystem::path dir(gazeboPath.c_str());
        if (!boost::filesystem::exists(dir)) {
            boost::filesystem::create_directories(dir);
        }       

        // Open CSV file
        this->csvFile.open(fileName);
        if (!this->csvFile.is_open()) {
            std::cerr << "Unable to open CSV file: " << fileName << std::endl;
        }

        // Write CSV headers
        this->csvFile << "Timestamp (%Y-%m-%d_%H-%M-%S)" << this->sep
                      << "Frame" << this->sep 
                      << "GazeboClassic - Step Size (ms)" << this->sep //GazeboServer
                      << "GazeboClassic - Simulation Time (ms)" << this->sep //GazeboServer
                      << "GazeboClassic - Real Time (ms)" << this->sep //GazeboServer
                      << "OS - System Time (ms)" << this->sep
                      << "GazeboClassic - RTF" << this->sep // GazeboServer
                      << "OS - RTF" << this->sep
                      << "GazeboClassic - Render FPS (Hz)" << this->sep // GazeboClient
                      << "OS - Plugin FPS (Hz)" << this->sep
                      << "Active Objects" << std::endl;
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
        this->totalFrames++;
        if(frameCount%50==0){
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
                objectsData += "{ \"alias\": \"" + model->GetName() + "\", ";
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
                this->csvFile << this->getCurrentDateTime() << this->sep
                            << std::to_string(this->totalFrames) << this->sep
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
    }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(LoggerPlugin)
}  // namespace gazebo
