// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#include "IdleBehavior.h"

extern void stopMoving();
extern void stopBlade();
extern void setEmergencyMode(bool emergency);

extern mower_msgs::Status last_status;
extern mower_logic::MowerLogicConfig last_config;
extern dynamic_reconfigure::Server<mower_logic::MowerLogicConfig> *reconfigServer;

extern ros::ServiceClient mapClient;
extern ros::ServiceClient dockingPointClient;

IdleBehavior IdleBehavior::INSTANCE;

std::string IdleBehavior::state_name() {
    return "IDLE";
}

Behavior *IdleBehavior::execute() {

    // Check, if we have a configured map. If not, print info and go to area recorder
    mower_map::GetMowingAreaSrv mapSrv;
    mapSrv.request.index = 0;
    if (!mapClient.call(mapSrv)) {
        ROS_WARN("We don't have a map configured. Starting Area Recorder!");
        return &AreaRecordingBehavior::INSTANCE;
    }

    // Check, if we have a docking position. If not, print info and go to area recorder
    mower_map::GetDockingPointSrv get_docking_point_srv;
    if(!dockingPointClient.call(get_docking_point_srv)) {
        ROS_WARN("We don't have a docking point configured. Starting Area Recorder!");
        return &AreaRecordingBehavior::INSTANCE;
    }

    ros::Rate r(25);
    while (ros::ok()) {
        stopMoving();
        stopBlade();
        if (manual_start_mowing ||
            (last_config.automatic_start && (last_status.v_battery > last_config.battery_full_voltage && last_status.mow_esc_status.temperature_motor < last_config.motor_cold_temperature &&
             !last_config.manual_pause_mowing))) {
            return &UndockingBehavior::INSTANCE;
        }

        if(start_area_recorder) {
            return &AreaRecordingBehavior::INSTANCE;
        }

        // This gets called if we need to refresh, e.g. on clearing maps
        if(aborted) {
            return &IdleBehavior::INSTANCE;
        }

        r.sleep();
    }

    return nullptr;
}

void IdleBehavior::enter() {
    start_area_recorder = false;
    // Reset the docking behavior, to allow docking
    DockingBehavior::INSTANCE.reset();

    // disable it, so that we don't start mowing immediately
    manual_start_mowing = false;
}

void IdleBehavior::exit() {
}

void IdleBehavior::reset() {

}

bool IdleBehavior::needs_gps() {
    return false;
}

bool IdleBehavior::mower_enabled() {
    return false;
}

void IdleBehavior::command_home() {
    // IdleBehavior == docked, don't do anything.
}

void IdleBehavior::command_start() {
    manual_start_mowing = true;
}

void IdleBehavior::command_s1() {
    start_area_recorder = true;
}

void IdleBehavior::command_s2() {
    
}

bool IdleBehavior::redirect_joystick() {
    return false;
}

