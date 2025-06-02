#include "mujoco_bridge.h"
#include <iostream>
#include <signal.h>

static bool exit_requested = false;

mjModel* m = nullptr;
mjData* d = nullptr;

void signal_handler(int sig) {
    exit_requested = true;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <model_path> [mavlink_addr] [mavlink_port]" << std::endl;
        return 1;
    }
    
    std::string model_path = argv[1];
    std::string mavlink_addr = argc > 2 ? argv[2] : "127.0.0.1";
    int mavlink_port = argc > 3 ? std::atoi(argv[3]) : 4560;
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    MuJoCoBridge bridge(model_path, mavlink_addr, mavlink_port);
    //mavlink_interface_->onSigInt();
    
	const int kErrorLength = 1024;          // load error string length
	char loadError[kErrorLength] = "";
	m = mj_loadXML(model_path.c_str(), nullptr, loadError, kErrorLength);
    //m = mj_loadModel(model_path.c_str(), nullptr);
    if (!m) {
	    std::cerr << "could not load binary model " << model_path.c_str() << " with mj_loadXML." << std::endl;
		std::cerr << loadError << std::endl;
	    return 1;
    }
	if (m) d = mj_makeData(m);
	if (d) {
		mj_forward(m, d);
		std::cout << "Model and Data loaded." << std::endl;
	}

	m->opt.timestep = 0.004;
    
    while (!exit_requested) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
		bridge.sendToMAVLink(m, d); // send sensor data
									// send ground truth?
		bridge.updateFromMAVLink(m, d); // get ctrl; waits to keep it lockstep w/ px4
		mj_step(m, d);

		//const char* message = Diverged(m->opt.disableflags, d);
		//if (message) {
		//	std::cout << message << std::endl;
		//}
		int dt_ms = 4; // TODO fix this...
		std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
    }
    
    return 0;
}
