#pragma once

#include <mujoco/mujoco.h>
#include "mavlink_interface.h"
#include <memory>
#include <thread>
#include <atomic>

class MuJoCoBridge {
public:
    MuJoCoBridge(const std::string& model_path, const std::string& mavlink_addr, int mavlink_port);
    ~MuJoCoBridge();
    
    bool initialize();
    void run();
    
    // Simulation control
    void step();
    void reset();
    
    // State accessors
    //void getState(double* position, double* velocity, double* attitude, double* angular_velocity);
    //void setActuatorInputs(const double* motor_speeds);

    void updateFromMAVLink(mjModel* m, mjData *d);
    void sendToMAVLink(mjModel* m, mjData *d);
    void sendGroundTruth(mjModel* m, mjData *d);
    void sendGps(mjModel* m, mjData *d);
    
private:
    // MuJoCo simulation
    //mjModel* model_;
    //mjData* data_;
    std::string model_path_;
    
    // MAVLink communication
    std::unique_ptr<MavlinkInterface> mavlink_interface_;
    std::string mavlink_addr_;
    int mavlink_port_;
    
    // Threading
    std::thread sim_thread_;
    
    // Internal methods
    void simulationLoop();
    std::atomic<bool> running_;
    
    // Vehicle parameters
    static constexpr int NUM_MOTORS = 4;
    static constexpr double SIMULATION_TIMESTEP = 0.004; // 250 Hz
};
