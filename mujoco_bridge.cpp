#include "mujoco_bridge.h"
#include <iostream>
#include <chrono>
#include <cmath>

MuJoCoBridge::MuJoCoBridge(const std::string& model_path, const std::string& mavlink_addr, int mavlink_port)
    : model_path_(model_path), mavlink_addr_(mavlink_addr), mavlink_port_(mavlink_port),
      //model_(nullptr), data_(nullptr),
	running_(false) {
    mavlink_interface_ = std::make_unique<MavlinkInterface>();
}

MuJoCoBridge::~MuJoCoBridge() {
	mavlink_interface_->close();
}

bool MuJoCoBridge::initialize() {
    // Initialize MAVLink interface
    mavlink_interface_->SetMavlinkAddr(mavlink_addr_);
    mavlink_interface_->SetMavlinkTcpPort(mavlink_port_);
    mavlink_interface_->SetUseTcp(true);
    mavlink_interface_->SetEnableLockstep(true);

	mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

	// set the Mavlink protocol version to use on the link
	float protocol_version_ = 2.0;
	if (protocol_version_ == 2.0) {
		chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
		std::cout << "Using MAVLink protocol v2.0\n";
	}
	else if (protocol_version_ == 1.0) {
		chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
		std::cout << "Using MAVLink protocol v1.0\n";
	}
	else {
		std::cerr << "Unkown protocol version! Using v" << protocol_version_ << "by default \n";
	}

	mavlink_interface_->Load();
    
    // IMU sensor is required
    // actuators are required
    
    /*TODO make optional gps, baro, mag, and airspeed sensor things
     * */
    return true;
}

void MuJoCoBridge::run() {
    if (!initialize()) {
        return;
    }
    
    running_ = true;
    sim_thread_ = std::thread(&MuJoCoBridge::simulationLoop, this);
    
    std::cout << "MuJoCo bridge running. MAVLink on " << mavlink_addr_ << ":" << mavlink_port_ << std::endl;
    
    // Keep main thread alive
    sim_thread_.join();
}

void MuJoCoBridge::simulationLoop() {
	/*
    auto last_time = std::chrono::high_resolution_clock::now();
    
    while (running_) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto dt = std::chrono::duration<double>(current_time - last_time).count();
        
        if (dt >= SIMULATION_TIMESTEP) {
            updateFromMAVLink();
            step();
            sendToMAVLink();
            last_time = current_time;
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    */
}

void MuJoCoBridge::step() {
    //mj_step(model_, data_);
}

void MuJoCoBridge::updateFromMAVLink(mjModel* m, mjData* d) {
	static double last_heartbeat_sent_time_=0.0;
    // Get actuator commands from PX4
    mavlink_interface_->pollForMAVLinkMessages();
	bool armed = mavlink_interface_->GetArmedState();
    auto controls = mavlink_interface_->GetActuatorControls();

    //std::cout << "mavlink controls: " << controls << std::endl;

	if (controls.size() < 16 && controls.size() > 0) {
		// Map PX4 actuator outputs to MuJoCo motor inputs
		// Assuming 4 motors: front-right, back-left, front-left, back-right
		std::cout << "controls: " << controls.size() << std::endl;
		std::cout << "armed: " << armed << std::endl;
		if (armed) {
			std::cout << "ctrl: ";
			for (int i = 0; i < m->nu; ++i) {
				// Convert normalized control [-1,1] to motor speed
				std::cout << controls[i] << ", ";
				double motor_speed = (controls[i]);// + 1.0) * 0.5; // Normalize to [0,1]
				d->ctrl[i] = motor_speed;
			}
			std::cout << std::endl;
		} else {
			for (int i = 0; i < m->nu; ++i) {
				d->ctrl[i] = 0.0;
			}
		}
	}

	//received_first_actuator_ = mavlink_interface_->GetReceivedFirstActuator();

	if ((d->time - last_heartbeat_sent_time_) > 1.0 || !mavlink_interface_->ReceivedHeartbeats()) {
		mavlink_interface_->SendHeartbeat();
		last_heartbeat_sent_time_ = d->time;
	}
}

void MuJoCoBridge::sendGroundTruth(mjModel* m, mjData* d) {
  // ground truth
  /*
    ignition::math::Pose3d pose_gr = ignitionFromGazeboMath(model_->GetWorldPose());
  ignition::math::Quaterniond q_gr = pose_gr.Rot();

  ignition::math::Quaterniond q_FLU_to_NED = q_ENU_to_NED * q_gr;
  ignition::math::Quaterniond q_nb = q_FLU_to_NED * q_FLU_to_FRD.Inverse();

  ignition::math::Vector3d vel_b = q_FLU_to_FRD.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearVel()));
  ignition::math::Vector3d vel_n = q_ENU_to_NED.RotateVector(ignitionFromGazeboMath(model_->GetWorldLinearVel()));
  ignition::math::Vector3d omega_nb_b = q_FLU_to_FRD.RotateVector(ignitionFromGazeboMath(model_->GetRelativeAngularVel()));

  ignition::math::Vector3d accel_true_ned = q_FLU_to_NED.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearAccel()));
  */

  // send ground truth
  mavlink_hil_state_quaternion_t hil_state_quat;
  hil_state_quat.time_usec = d->time * 1e6; 
  hil_state_quat.attitude_quaternion[0] = d->qpos[3]; // q_nb.W();
  hil_state_quat.attitude_quaternion[1] = d->qpos[4]; // q_nb.X();
  hil_state_quat.attitude_quaternion[2] = d->qpos[5]; // q_nb.Y();
  hil_state_quat.attitude_quaternion[3] = d->qpos[6]; // q_nb.Z();

  hil_state_quat.rollspeed  = d->qvel[3]; //omega_nb_b.X();
  hil_state_quat.pitchspeed = d->qvel[4]; //omega_nb_b.Y();
  hil_state_quat.yawspeed   = d->qvel[5]; //omega_nb_b.Z();

  hil_state_quat.lat = 0.0;//groundtruth_lat_rad_ * 180 / M_PI * 1e7;
  hil_state_quat.lon = 0.0;//groundtruth_lon_rad_ * 180 / M_PI * 1e7;
  hil_state_quat.alt = 0.0;//groundtruth_altitude_ * 1000;

  hil_state_quat.vx = d->qvel[0]*100; //vel_n.X() * 100;
  hil_state_quat.vy = d->qvel[1]*100; //vel_n.Y() * 100;
  hil_state_quat.vz = d->qvel[2]*100; //vel_n.Z() * 100;

  // assumed indicated airspeed due to flow aligned with pitot (body x)
  hil_state_quat.ind_airspeed = d->sensordata[13];//vel_b.X();

  hil_state_quat.true_airspeed = d->sensordata[13]; //(model_->GetWorldLinearVel() -  wind_vel_).GetLength() * 100;

  hil_state_quat.xacc = d->sensordata[16]; //accel_true_ned.X() * 1000;
  hil_state_quat.yacc = d->sensordata[17]; //accel_true_ned.Y() * 1000;
  hil_state_quat.zacc = d->sensordata[18]; //accel_true_ned.Z() * 1000;

  //if (!hil_mode_ || (hil_mode_ && hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_state_quat);
    mavlink_interface_->send_mavlink_message(&msg);
  //}
}

void MuJoCoBridge::sendGps(mjModel* m, mjData* d) { //GpsPtr& gps_msg, const int& id) {
  SensorData::Gps gps_data;
  gps_data.time_utc_usec = d->time * 1e6;
  gps_data.fix_type = 3;
  //47.6061° N, 122.3328° W
  // 0.0000089 is ~ 1 meter change in lat-long
  gps_data.latitude_deg = (47.6061 + d->qpos[0]*0.0000089) * 1e7; //gps_msg->latitude_deg() * 1e7;
  gps_data.longitude_deg = (122.3328 + d->qpos[1]*0.0000089) * 1e7; //gps_msg->longitude_deg() * 1e7;
  gps_data.altitude = d->qpos[2] * 1000.0;
  gps_data.eph = 1.0*100.0; //gps_msg->eph() * 100.0;
  gps_data.epv = 1.0*100.0; //gps_msg->epv() * 100.0;
  gps_data.velocity = std::sqrt(d->qvel[0]*d->qvel[0]+d->qvel[1]*d->qvel[1]) * 100.0;
  gps_data.velocity_north = d->qvel[0] * 100.0;
  gps_data.velocity_east = d->qvel[1] * 100.0;
  gps_data.velocity_down = -d->qvel[2] * 100.0;
  // MAVLINK_HIL_GPS_T CoG is [0, 360]. math::Angle::Normalize() is [-pi, pi].
  //ignition::math::Angle cog(atan2(gps_msg->velocity_east(), gps_msg->velocity_north()));
  auto cog(atan2(d->qvel[1], d->qvel[0]));
  gps_data.cog = static_cast<uint16_t>(cog*180/3.14 * 100.0);
  gps_data.satellites_visible = 10;
  gps_data.id = 0; //id;

  mavlink_interface_->SendGpsMessages(gps_data);
}

void MuJoCoBridge::sendToMAVLink(mjModel* m, mjData* d) {
    auto s = d->sensordata;
// TODO make sure IMUs are rotated into correct orientation
    SensorData::Imu imu_data;
    imu_data.gyro_b = Eigen::Vector3d(s[0], s[1], s[2]);
    imu_data.accel_b = Eigen::Vector3d(s[3], s[4], s[5]);
    mavlink_interface_->UpdateIMU(imu_data);

	SensorData::Magnetometer mag_data;
	mag_data.mag_b = Eigen::Vector3d(s[6], s[7], s[8]);
	mavlink_interface_->UpdateMag(mag_data, 1);
	//std::cout << "accel: " << imu_data.accel_b << std::endl;
	//std::cout << "gyro:  " << imu_data.gyro_b << std::endl;
    mavlink_interface_->SendSensorMessages(d->time * 1e6);
// SEND ground truth?
}
    /*
void MuJoCoBridge::getState(double* position, double* velocity, double* attitude, double* angular_velocity) {
    // Extract position (convert MuJoCo coordinates to NED)
    position[0] = data_->qpos[1];  // North
    position[1] = data_->qpos[0];  // East  
    position[2] = -data_->qpos[2]; // Down (MuJoCo Z is up)
    
    // Extract velocity
    velocity[0] = data_->qvel[1];  // North
    velocity[1] = data_->qvel[0];  // East
    velocity[2] = -data_->qvel[2]; // Down
    
    // Extract quaternion (w,x,y,z)
    attitude[0] = data_->qpos[3];  // w
    attitude[1] = data_->qpos[4];  // x
    attitude[2] = data_->qpos[5];  // y
    attitude[3] = data_->qpos[6];  // z
    
    // Extract angular velocity
    angular_velocity[0] = data_->qvel[3]; // Roll rate
    angular_velocity[1] = data_->qvel[4]; // Pitch rate
    angular_velocity[2] = data_->qvel[5]; // Yaw rate
}
					  */