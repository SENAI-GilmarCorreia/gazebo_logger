#include <cmath>  // Para garantir que outras funções matemáticas estejam acessíveis

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <chrono>
#include <ctime>
#include <fstream>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iomanip>
#include <sstream>
#include <boost/filesystem.hpp>

namespace gazebo {
class LoggerPlugin : public WorldPlugin {
   private:
    std::ofstream csvFile;
    std::chrono::high_resolution_clock::time_point lastTime, initialTime;
    int frameCount;
    float fps;
    char sep = ';';

   public:
    LoggerPlugin() : WorldPlugin() {
        frameCount = 0;
        fps = 0.0f;
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
        // Save logs to the default .gazebo path

        // const char* homeDir = std::getenv("USERPROFILE");
        // std::string gazeboPath = std::string(homeDir) + "/.gazebo/logger/";

        // // Create logs directory if it doesn't exist
        // boost::filesystem::path dir(gazeboPath);
        // if (!boost::filesystem::exists(dir)) {
        //     boost::filesystem::create_directories(dir);
        // }

        // Create CSV file in the .gazebo/logger/ directory
        std::string dateTime = getCurrentDateTime();
        std::string fileName = "gazeboLogger_" + dateTime + ".csv";//gazeboPath + "gazeboLogger_" + dateTime + ".csv";

        // Open CSV file
        csvFile.open(fileName);
        if (!csvFile.is_open()) {
            std::cerr << "Unable to open CSV file: " << fileName << std::endl;
        }

        // Write CSV headers
        csvFile << "Frame" << sep << "Simulation Time (ms)" << sep
                << "System Time (ms)" << sep << "FPS (Hz)" << sep << "Pose Data"
                << sep << "Collision Count" << std::endl;

        // Initialize time points for FPS calculation
        initialTime = std::chrono::high_resolution_clock::now();
        lastTime = std::chrono::high_resolution_clock::now();

        // Connect to the world update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&LoggerPlugin::OnUpdate, this, _world));
    }

    void OnUpdate(physics::WorldPtr _world) {
        // Increment frame count
        frameCount++;

        // Get current simulation time
        double simTime = _world->SimTime().Double() * 1000.0;  // convert to ms

        // Get system time since plugin start
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto systemTime_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(currentTime -
                                                                  initialTime)
                .count();

        // Calculate FPS if at least 1 second has passed
        std::chrono::duration<float> elapsedTime = currentTime - lastTime;
        if (elapsedTime.count() >= 1.0f) {
            fps = frameCount / elapsedTime.count();
            frameCount = 0;
            lastTime = currentTime;
        }

        // Get pose information of objects (here, we log the pose of a specific
        // model, e.g., "robot")
        std::string poseData = "[]";  // If no object, set to empty
        // physics::ModelPtr model = _world->ModelByName("robot");
        // if (model) {
        //   ignition::math::Pose3d pose = model->WorldPose();
        //   poseData = "[" + std::to_string(pose.Pos().X()) + ", "
        //                   + std::to_string(pose.Pos().Y()) + ", "
        //                   + std::to_string(pose.Pos().Z()) + ", "
        //                   + std::to_string(pose.Rot().Roll()) + ", "
        //                   + std::to_string(pose.Rot().Pitch()) + ", "
        //                   + std::to_string(pose.Rot().Yaw()) + "]";
        // }

        // Log collision count (this is simplified, you can specify more details
        // based on your world)
        int collisionCount = 0;
        // auto contacts =
        // _world->Physics()->GetContactManager()->GetContacts(); if
        // (contacts->contact_size() > 0)
        // {
        //   collisionCount = contacts->contact_size();
        // }

        // Write data to CSV file
        if (csvFile.is_open()) {
            csvFile << std::to_string(frameCount) << ";"
                    << std::to_string(simTime) << ";"
                    << std::to_string(systemTime_ms) << ";"
                    << std::to_string(fps) << ";" << poseData << ";"
                    << std::to_string(collisionCount) << "\n";
        }
    }

    ~LoggerPlugin() {
        if (csvFile.is_open()) {
            csvFile.close();
        }
    }

   private:
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
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(LoggerPlugin)
}  // namespace gazebo
