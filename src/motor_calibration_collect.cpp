/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

#include "api_setup.hpp"
#include "arm_core_functions.hpp"
#include "actuator_csv_logger.hpp"
#include "motor_model_utils.hpp"

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001
constexpr double DEG2RAD = M_PI / 180.0;

float velocity = 20.0f;         // Default velocity of the actuator (degrees per seconds)
float time_duration = 20;        // Duration of the example (seconds)

bool control_loop(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, int actuator_idx)
{
    bool return_status = true;

    auto servoing_mode = k_api::Base::ServoingModeInformation();
    auto base_command = k_api::BaseCyclic::Command();
    auto base_feedback = k_api::BaseCyclic::Feedback();

    std::vector<float> commands;
    std::vector<float> original_positions;

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    int timeout = 0;

    std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        const int actuator_count = base->GetActuatorCount().count();
        constexpr int kLogEveryN = 10;  // 1 kHz loop -> 100 Hz logging to avoid heavy IO.
        kinova_teleoperation::ActuatorCsvLogger logger(
          "/workspaces/kinova-teleoperation/log/motor_calibration_log_j" + std::to_string(actuator_idx + 1) + ".csv",
          actuator_count,
          kLogEveryN,
          true,
          true);

        std::string urdf_path;
        try {
            const auto share_dir = ament_index_cpp::get_package_share_directory("kortex_description");
            urdf_path = share_dir + "/robots/gen3_lite.urdf";
        } catch (const std::exception & ex) {
            std::cerr << "Unable to locate kortex_description package: " << ex.what() << std::endl;
            return false;
        }

        std::vector<kinova_teleoperation::JointDriveModel> joint_models;
        try {
            const auto motor_model_path = kinova_teleoperation::resolveMotorModelPath();
            joint_models = kinova_teleoperation::orderJointModelsByActuatorIndex(
              kinova_teleoperation::loadJointModels(motor_model_path));
        } catch (const std::exception & ex) {
            std::cerr << "Unable to load motor model: " << ex.what() << std::endl;
            return false;
        }

        if (joint_models.size() != static_cast<size_t>(actuator_count)) {
            std::cerr << "Motor model size does not match actuator count." << std::endl;
            return false;
        }

        kinova_teleoperation::GravityModel gravity_model(
          urdf_path, "base_link", "tool_frame", joint_models.size());

        for(int i{0}; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            original_positions.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Define the callback function used in Refresh_callback
        auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
        {
            // We are printing the data of the moving actuator just for the example purpose,
            // avoid this in a real-time loop
            std::string serialized_data;
            google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
            std::cout << serialized_data << std::endl << std::endl;
        };

        // Real-time loop
        while(timer_count < (time_duration * 1000))
        {
            now = GetTickUs();
            if(now - last > 1000)
            {
                for(int i{0}; i < actuator_count; i++)
                {
        		    if(i == actuator_idx)
        		    {
                        constexpr float kAmplitudeDeg{90.0f};
                        constexpr float kPeriodSec{10.0f};
                        constexpr float kRampDurationSec{1.0f};

                        const float t = static_cast<float>(timer_count) * 0.001f;
                        const float omega = (2.0f * static_cast<float>(M_PI)) / kPeriodSec;

                        float ramp = 1.0f;
                        if (t < kRampDurationSec)
                        {
                            const float x = t / kRampDurationSec;
                            ramp = x * x * (3.0f - 2.0f * x);  // smoothstep: ramp(0)=0, ramp'(0)=0
                        }

                        commands[i] = original_positions[i] + kAmplitudeDeg * ramp * sinf(omega * t);
                        float wrapped = fmodf(commands[i], 360.0f);
                    	base_command.mutable_actuators(i)->set_position(wrapped);
        		    }
                }

                bool refreshed = false;
                try
                {
                    base_feedback = base_cyclic->Refresh(base_command);
                    refreshed = true;
                }
                catch(...)
                {
                    timeout++;
                }

                if (refreshed) {
                    std::vector<double> q_rad(actuator_count, 0.0);
                    std::vector<double> qdot_rad(actuator_count, 0.0);
                    for (int i = 0; i < actuator_count; ++i) {
                        q_rad[i] = static_cast<double>(base_feedback.actuators(i).position()) * DEG2RAD;
                        qdot_rad[i] = static_cast<double>(base_feedback.actuators(i).velocity()) * DEG2RAD;
                    }

                    const auto gravity_tau = gravity_model.computeGravityTorques(q_rad);
                    std::vector<double> model_currents(actuator_count, 0.0);
                    for (int i = 0; i < actuator_count; ++i) {
                        model_currents[i] = kinova_teleoperation::torqueToCurrent(
                          gravity_tau[i], qdot_rad[i], joint_models[i]);
                    }

                    logger.log(base_feedback, model_currents, gravity_tau);
                }
                
                timer_count++;
                last = GetTickUs();
            }
        }
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }
 
    // Set back the servoing mode to Single Level Servoing
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

int main(int argc, char **argv) {
    Robot robot{"192.168.2.11", "admin", "admin"};
    auto base = robot.getBaseClient();
    auto base_cyclic = robot.getBaseCyclicClient();

    const unsigned int actuator_count{base->GetActuatorCount().count()};

    for (unsigned int i{0}; i < actuator_count; ++i) {
        move_to_position(base, "MotorCalibJ" + std::to_string(i+1));
        auto isOk = control_loop(base, base_cyclic, i);
        if (!isOk)
        {
            std::cout << "There has been an unexpected error in example_cyclic_armbase() function." << std::endl;
        }
    }
}
