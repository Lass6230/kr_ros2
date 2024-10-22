/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Kassow Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Kassow Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// #include <boost/crc.hpp>
#define CRCPP_INCLUDE_ESOTERIC_CRC_DEFINITIONS

#include "CRC.h"

#include "kord/api/kord.h"
#include "kord/utils/timex.h"
#include "kord/version.h"

#include <kr2/kord/protocol/DataFormatDescription.h>
#include <kr2/kord/protocol/KORDDataIDs.h>
#include <kr2/kord/protocol/KORDFrames.h>
#include <kr2/kord/protocol/KORDItemIDs.h>
#include <kr2/kord/protocol/ServerParameters.h>
#include <kr2/kord/protocol/version.h>

#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <bitset>
#include <ctime>
#include <random>

using namespace kr2::kord;
using namespace kr2::kord::protocol;

// #define KORD_UDP_PORT 32001
#define KORD_UDP_PORT 7582
enum { MAX_MSG_LEN = 1024 };

#define KORD_TIME_OUT 500

class KordCore::Internals {
public:
    Internals() = default;

    explicit Internals(unsigned int session_id) : session_id_(session_id) {}

    void populateFrameFromContent()
    {
        eth_kord_proto_frm_.frame_id_ = htons(KORD_FRAME_ID_CONTENT);
        eth_kord_proto_frm_.kord_version_ = htons(KORD_PROTOCOL_VERSION);
        eth_kord_proto_frm_.session_id_ = htons(session_id_);
        eth_kord_proto_frm_.tx_timestamp_ = std::chrono::steady_clock::now().time_since_epoch().count();
        size_t content_frm_size = sizeof(KORDContentFrame);
        eth_kord_proto_frm_.payload_length_ = htons(content_frm_size);
        content_builder_.getPayload(eth_kord_proto_frm_.payload_, sizeof(eth_kord_proto_frm_.payload_));
    }

    KORDFrame eth_kord_proto_frm_{};
    StatusFrameParser status_parser_{};

    // TODO Status frame format description
    DataFormatDescription status_dfd_ = DataFormatDescription::makeStatusFrameDescription();
    // KR2KORDFrame_ArmStatus_Req

    ContentFrameBuilder content_builder_;
    ContentFrameParser content_parser_;

    // clang-format off
    ContentItem rx_content_items_[MAX_CONTENT_ITEMS]{};
    ContentItem tx_content_items_[MAX_CONTENT_ITEMS]{};
    DataFormatDescription req_arm_status_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eRequestStatusArm);
    DataFormatDescription cmd_joint_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandMoveJ);
    DataFormatDescription cmd_linear_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandMoveL);
    DataFormatDescription cmd_manifold_dfd_   = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandManifold);
    DataFormatDescription cmd_direct_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandMoveDirect);
    DataFormatDescription cmd_velocity_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandMoveVelocity);
    DataFormatDescription cmd_joint_fw_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandJointFirmware);

    DataFormatDescription cmd_setFrame_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandSetFrame);
    DataFormatDescription cmd_setLoad_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandSetLoad);

    DataFormatDescription cmd_cleanAlarm_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandCleanAlarm);
    DataFormatDescription cmd_rc_api_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandRCAPI);

    DataFormatDescription req_sys_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eRequestSystem);
    DataFormatDescription req_transf_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eRequestTransfer);
    DataFormatDescription res_statusEcho_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eStatusEchoResponse);
    DataFormatDescription req_io_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eCommandSetIODigitalOut);
    DataFormatDescription req_server_dfd_ = DataFormatDescription::makeItemDescriptionLatest(EKORDItemID::eRequestServer);
    unsigned int req_sys_seq_num_{0};
    // clang-format on

    KordCore::RobotArmStatus sync_arm_status_;
    KordCore::CommandStatuses sync_command_statuses_;

    std::mutex comm_mutex_;
    std::mutex read_mutex_;

    unsigned int session_id_{0};
};

KordCore::KordCore(const std::string &a_hostname, unsigned int a_port, unsigned int a_session_id, connection conn)
    : his_(new Internals(a_session_id))
{
    conn_type_ = conn;
    switch (conn) {
    case TCP_SERVER:
        conn_container_ = std::make_shared<TCP_server>(io_service_, a_port);
        break;
    case TCP_CLIENT:
        conn_container_ = std::make_shared<TCP_client>(io_service_, a_hostname, a_port);
        break;
    case UDP_SERVER:
        conn_container_ = std::make_shared<UDP_server>(io_service_, a_hostname, a_port);
        break;
    case UDP_CLIENT:
        conn_container_ = std::make_shared<UDP_client>(io_service_, a_hostname, a_port);
        break;
    }

    hostname_ = a_hostname;
    port_ = a_port;
    session_id_ = a_session_id;

    std::cout << "[KORD-API] Version: " << API_VERSION << ", sha: " << API_SHA1 << "\n";
    std::cout << "[KORD-Protocol]: " << PROTOCOL_VERSION << ", sha: " << PROTOCOL_SHA1 << "\n";
}

KordCore::~KordCore()
{
    disconnect();
    delete his_;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::asPoseControl()
{
    this->command_type = KordCore::RobotFrameCommand::MOVE_POSE;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::asVelocityControl()
{
    this->command_type = KordCore::RobotFrameCommand::MOVE_VELOCITY;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::asPosVelControl()
{
    this->command_type = KordCore::RobotFrameCommand::MOVE_POSE_DYN;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withTargetPose(const std::array<double, 6UL> &a_pose)
{
    this->tcp_target_ = a_pose;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withTargetVelocity(const std::array<double, 6UL> &a_velocity)
{
    this->tcp_target_velocity_ = a_velocity;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withSyncValue(const long &a_sync_value)
{
    this->sync_value_ = a_sync_value;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withPeriod(const double &a_period)
{
    this->period_ = a_period;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withTimeout(const double &a_timeout)
{
    this->timeout_ = a_timeout;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withTargetAcceleration(
    const std::array<double, 6UL> &a_acceleration)
{
    this->tcp_target_acc_ = a_acceleration;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withTrackingType(const TrackingType &a_tt)
{
    this->tt_ = a_tt;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withTrackingValue(const double &a_tt_value)
{
    this->tt_value_ = a_tt_value;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withBlendType(const BlendType &a_bt)
{
    this->bt_ = a_bt;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withBlendValue(const double &a_bt_value)
{
    this->bt_value_ = a_bt_value;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withOverlayType(const OverlayType &a_ot)
{
    this->ot_ = a_ot;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::withSyncTime(const double &a_sync_time)
{
    this->sync_time_ = a_sync_time;
    return *this;
}

KordCore::RobotFrameCommand &KordCore::RobotFrameCommand::setSequenceNumber(unsigned int a_seq)
{
    this->seq_ = a_seq;
    return *this;
}

bool KordCore::connect(const char *device)
{
    using namespace std::chrono_literals;

    kr2::utils::Timex::initSingletonTimex(500, false);

    memset(&his_->eth_kord_proto_frm_, 0x00, sizeof(KORDFrame));

    if (!conn_container_->connect(device)) {
        return false;
    }

    //    std::cout << "Starting transmission...\n" ;

    // stats reset
    start_ = std::chrono::steady_clock::now();
    packets_cnt_ = 0;
    sequence_number_ = 0;
    return true;
}

bool KordCore::disconnect() { return conn_container_->disconnect(); }

bool KordCore::syncRC(long flags)
{
    if (!conn_container_->is_connected()) {
        return false;
    }
    requestArmStatus();
    int time_out_counter = 0; // counter for time out procedure

    bool full_cycle = flags & 1; // first bit of flags responsible for full cycle rotation

    bool got_first_id = false;
    bool second_loop = false;
    int first_frame_id = -1;
    int now_frame_id = -1;
    int first_load_id = -1;
    int now_load_id = -1;
    int first_cpu_state_id = -1;
    int now_cpu_state_id = -1;

    do {
        do {
            time_out_counter++;
            do {
                auto ts_sent = std::chrono::steady_clock::now();
                ts_record_.t0_ = ts_sent.time_since_epoch().count();

                memset(&his_->eth_kord_proto_frm_, 0x0, sizeof(KORDFrame));

                uint n = conn_container_->recvFrame(&his_->eth_kord_proto_frm_);
                /*
                 * Waiting for the first timed response marking the RC start of the
                 * tick. Calculate every other tick starting captured update.
                 *
                 * DEBUG: To observe the timeout counter uncomment the following line.
                 */
                // std::cout << "[" << time_out_counter << "]" << "n: " << n << "\n";
                time_out_counter++;
            } while (his_->eth_kord_proto_frm_.frame_id_ != KORD_FRAME_ID_STATUS && time_out_counter < KORD_TIME_OUT);
            if (time_out_counter >= KORD_TIME_OUT) {
                std::cout << std::endl << "Time out." << std::endl;
                return false;
            }
        } while (his_->eth_kord_proto_frm_.session_id_ != session_id_);
        std::cout << "Status captured.\n";

        if (!his_->status_parser_.setFromPayload(his_->eth_kord_proto_frm_.payload_,
                                                 his_->eth_kord_proto_frm_.payload_length_)) {
            std::cout << "Failed to set the Status frame payload.\n";
            std::cout << "KORD Payload -     length: " << his_->eth_kord_proto_frm_.payload_length_ << "\n";
            std::cout << "KORD Payload -   frame_id: " << his_->eth_kord_proto_frm_.frame_id_ << "\n";
            std::cout << "KORD Payload - session_id: " << his_->eth_kord_proto_frm_.session_id_ << "\n";
            return false;
        }
        else {
            // if we closed the cycle, no need to read
            if (!got_first_id || !second_loop || first_frame_id != now_frame_id) {
                now_frame_id = getIterativeFrameId();
            }

            if (!got_first_id || !second_loop || first_load_id != now_load_id) {
                now_load_id = getIterativeLoadId();
            }

            if (!got_first_id || !second_loop || first_cpu_state_id != now_cpu_state_id) {
                now_cpu_state_id = getIterativeCPUStateId();
            }
            // std::cout << "now: " << now_frame_id << " " << now_load_id << " " <<
            // now_cpu_state_id << '\n'; std::cout << "last: " << first_frame_id << " "
            // << first_load_id << " " << first_cpu_state_id << '\n';
            if (!got_first_id) {
                first_frame_id = now_frame_id;
                first_load_id = now_load_id;
                first_cpu_state_id = now_cpu_state_id;
                got_first_id = true;
            }
            else {
                second_loop = true;
            }
            // storing new values
            updateRecentArmStatus(his_->sync_arm_status_);
        }
        ctlrc_update_ts_ = kr2::utils::Timex::thisTick(kr2::utils::Timex::now());
    } while (full_cycle && (!second_loop || !((first_frame_id == now_frame_id) && (first_load_id == now_load_id) &&
                                              (first_cpu_state_id == now_cpu_state_id))));

    return true;
}

bool KordCore::waitSync(std::chrono::microseconds a_timeout_us, long flags)
{
    std::lock_guard<std::mutex> guard(his_->read_mutex_);

    int time_out_counter = 0;
    bool full_cycle = flags & 1;
    bool only_recent = (flags >> 1) & 1;
    const auto recent_interval = std::chrono::nanoseconds(std::chrono::milliseconds(6));
    bool not_recent = false;

    bool got_first_id = false;
    bool second_loop = false;
    int first_frame_id = -1;
    int now_frame_id = -1;
    int first_load_id = -1;
    int now_load_id = -1;
    int first_cpu_state_id = -1;
    int now_cpu_state_id = -1;

    while (true) {
        if (!waitForValidFrame(a_timeout_us, time_out_counter)) {
            return false;
        }

        parseFrame();

        if (!updateIds(got_first_id,
                       second_loop,
                       first_frame_id,
                       now_frame_id,
                       first_load_id,
                       now_load_id,
                       first_cpu_state_id,
                       now_cpu_state_id)) {
            continue;
        }

        updateRecentStatuses();

        if (only_recent) {
            not_recent = !isRecent(recent_interval);
        }

        stats.capture(std::chrono::steady_clock::now(), getTxStamp(), his_->status_parser_.getSequenceNumber());
        ctlrc_update_ts_ = kr2::utils::Timex::thisTick(kr2::utils::Timex::now());

        if (!shouldContinueLoop(not_recent,
                                full_cycle,
                                second_loop,
                                first_frame_id,
                                now_frame_id,
                                first_load_id,
                                now_load_id,
                                first_cpu_state_id,
                                now_cpu_state_id)) {
            break;
        }
    }

    return true;
}

bool KordCore::waitForValidFrame(std::chrono::microseconds a_timeout_us, int &time_out_counter)
{
    while (true) {
        time_out_counter++;
        auto ts_start = std::chrono::steady_clock::now();
        while (true) {
            memset(&his_->eth_kord_proto_frm_, 0x0, sizeof(KORDFrame));
            conn_container_->recvFrame(&his_->eth_kord_proto_frm_);

            if (hasTimedOut(ts_start, a_timeout_us)) {
                stats.update_failure();
                return false;
            }

            if (his_->eth_kord_proto_frm_.frame_id_ == KORD_FRAME_ID_STATUS ||
                his_->eth_kord_proto_frm_.frame_id_ == KORD_FRAME_ID_CONTENT) {
                break;
            }
        }

        if (time_out_counter > KORD_TIME_OUT) {
            std::cout << "Time out." << std::endl;
            return false;
        }

        if (his_->eth_kord_proto_frm_.session_id_ == session_id_) {
            return true;
        }
    }
}

bool KordCore::hasTimedOut(const std::chrono::steady_clock::time_point &start, const std::chrono::microseconds &timeout)
{
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start);
    return elapsed.count() > timeout.count();
}

void KordCore::parseFrame()
{
    if (his_->eth_kord_proto_frm_.frame_id_ == KORD_FRAME_ID_STATUS) {
        his_->status_parser_.setFromPayload(his_->eth_kord_proto_frm_.payload_,
                                            his_->eth_kord_proto_frm_.payload_length_);
    }
    else if (his_->eth_kord_proto_frm_.frame_id_ == KORD_FRAME_ID_CONTENT) {
        his_->content_parser_.setFromPayload(his_->eth_kord_proto_frm_.payload_,
                                             his_->eth_kord_proto_frm_.payload_length_);
    }
}

bool KordCore::updateIds(bool &got_first_id,
                         bool &second_loop,
                         int &first_frame_id,
                         int &now_frame_id,
                         int &first_load_id,
                         int &now_load_id,
                         int &first_cpu_state_id,
                         int &now_cpu_state_id) const
{
    bool idsUpdated = false;

    if (!got_first_id || !second_loop || first_frame_id != now_frame_id) {
        now_frame_id = getIterativeFrameId();
        idsUpdated = true;
    }

    if (!got_first_id || !second_loop || first_load_id != now_load_id) {
        now_load_id = getIterativeLoadId();
        idsUpdated = true;
    }

    if (!got_first_id || !second_loop || first_cpu_state_id != now_cpu_state_id) {
        now_cpu_state_id = getIterativeCPUStateId();
        idsUpdated = true;
    }

    if (!got_first_id) {
        first_frame_id = now_frame_id;
        first_load_id = now_load_id;
        first_cpu_state_id = now_cpu_state_id;
        got_first_id = true;
    }
    else {
        second_loop = true;
    }

    return idsUpdated;
}

void KordCore::updateRecentStatuses()
{
    updateRecentArmStatus(his_->sync_arm_status_);
    updateRecentCommandStatus(his_->sync_command_statuses_);
}

bool KordCore::isRecent(const std::chrono::nanoseconds &recent_interval) const
{
    return !(std::chrono::steady_clock::now() - stats.cap_latest_ > recent_interval ||
             getTxStamp() - stats.cap_latest_cbun_ > recent_interval.count());
}

bool KordCore::shouldContinueLoop(bool not_recent,
                                  bool full_cycle,
                                  bool second_loop,
                                  int first_frame_id,
                                  int now_frame_id,
                                  int first_load_id,
                                  int now_load_id,
                                  int first_cpu_state_id,
                                  int now_cpu_state_id)
{
    return not_recent ||
           (full_cycle && (!second_loop || !((first_frame_id == now_frame_id) && (first_load_id == now_load_id) &&
                                             (first_cpu_state_id == now_cpu_state_id))));
}

int64_t KordCore::getTxStamp() const
{
    return his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eTxStamp));
}

uint8_t KordCore::getIterativeFrameId() const
{
    return his_->status_parser_.getData<uint8_t>(his_->status_dfd_.getOffset(EKORDDataID::eFrameId)) << 1 |
           his_->status_parser_.getData<uint8_t>(
               his_->status_dfd_.getOffset(EKORDDataID::eFramePoseRef)); // to get unique id, need to consider
    // reference too
}

uint8_t KordCore::getIterativeLoadId() const
{
    return his_->status_parser_.getData<uint8_t>(his_->status_dfd_.getOffset(EKORDDataID::eLoadId));
}

uint8_t KordCore::getIterativeCPUStateId() const
{
    return his_->status_parser_.getData<uint8_t>(his_->status_dfd_.getOffset(EKORDDataID::eCPUStateId));
}

void printHorizontalLine(int len)
{
    std::cout << std::setw(len) << std::setfill('-') << "";
    std::cout << std::setfill(' ') << std::endl;
}

std::string checkForNoData(const int64_t &a_data, const double &a_factor = 1, const int &precision = 3)
{
    if (a_data == INT64_MIN || a_data == INT64_MAX) {
        return "NA";
    }
    else {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << a_data / a_factor;
        return stream.str();
    }
}

void KordCore::printStats(const CBunReceivedStatistics &a_cbun_stats)
{
    const int columns = 7;
    const int columnWidth = 15;
    const int firstColumnWidth = 35;
    const int firstColumnPart = 5;

    // Print header
    printHorizontalLine((columns - 1) * columnWidth + firstColumnWidth);
    std::cout << std::left << std::setw(firstColumnWidth) << "Metrics" << "|" << std::setw(columnWidth - 1)
              << "WindowMax[ms]" << "|" << std::setw(columnWidth - 1) << "WindowAvg[ms]" << "|"
              << std::setw(columnWidth - 1) << "WindowMin[ms]" << "|" << std::setw(columnWidth - 1) << "GlobalMax[ms]"
              << "|" << std::setw(columnWidth - 1) << "GlobalAvg[ms]" << "|" << std::setw(columnWidth - 1)
              << "GlobalMin[ms]" << "|" << std::endl;
    printHorizontalLine((columns - 1) * columnWidth + firstColumnWidth);

    // Print data
    std::cout << std::left << std::setw(firstColumnWidth / firstColumnPart) << ""
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|Command Jitter" << "|"
              << std::setw(columnWidth - 1) << a_cbun_stats.cmd_jitter_window_max / 1000. << "|"
              << std::setw(columnWidth - 1) << a_cbun_stats.cmd_jitter_window_avg / 1000. << "|"
              << std::setw(columnWidth - 1) << "" << "|" << std::setw(columnWidth - 1)
              << a_cbun_stats.cmd_jitter_global_max / 1000. << "|" << std::setw(columnWidth - 1) << "" << "|"
              << std::setw(columnWidth - 1) << "" << "|" << std::endl;
    std::cout << std::setw(firstColumnWidth / firstColumnPart) << "CBUN"
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|Round Trip Time (RTT)" << "|"
              << std::setw(columnWidth - 1) << a_cbun_stats.round_trip_window_max / 1000. << "|"
              << std::setw(columnWidth - 1) << a_cbun_stats.round_trip_window_avg / 1000. << "|"
              << std::setw(columnWidth - 1) << "" << "|" << std::setw(columnWidth - 1)
              << a_cbun_stats.round_trip_global_max / 1000. << "|" << std::setw(columnWidth - 1) << "" << "|"
              << std::setw(columnWidth - 1) << "" << "|" << std::endl;
    std::cout << std::setw(firstColumnWidth / firstColumnPart) << ""
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|System Jitter" << "|"
              << std::setw(columnWidth - 1) << a_cbun_stats.system_jitter_window_max / 1000. << "|"
              << std::setw(columnWidth - 1) << a_cbun_stats.system_jitter_window_avg / 1000. << "|"
              << std::setw(columnWidth - 1) << "" << "|" << std::setw(columnWidth - 1)
              << a_cbun_stats.system_jitter_global_max / 1000. << "|" << std::setw(columnWidth - 1) << "" << "|"
              << std::setw(columnWidth - 1) << "" << "|" << std::endl;
    printHorizontalLine((columns - 1) * columnWidth + firstColumnWidth);

    std::cout << std::left << std::setw(firstColumnWidth / firstColumnPart) << ""
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|API's TimeStamps Delay"
              << "|" << std::setw(columnWidth - 1) << "" << "|" << std::setw(columnWidth - 1) << "" << "|"
              << std::setw(columnWidth - 1) << "" << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_max_rx(), 1000000.) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_avg_rx(), 1000000.) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_min_rx(), 1000000.) << "|" << std::endl;
    std::cout << std::setw(firstColumnWidth / firstColumnPart) << "API"
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|CBun's TimeStamps Delay"
              << "|" << std::setw(columnWidth - 1) << "" << "|" << std::setw(columnWidth - 1) << "" << "|"
              << std::setw(columnWidth - 1) << "" << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_max_tx(), 1000000.) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_avg_tx(), 1000000.) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_min_tx(), 1000000.) << "|" << std::endl;
    std::cout << std::setw(firstColumnWidth / firstColumnPart) << ""
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|API's TimeStamps Jitter"
              << "|" << std::setw(columnWidth - 1) << checkForNoData(stats.get_max_jitter(), 1000000.) << "|"
              << std::setw(columnWidth - 1) << checkForNoData(stats.get_avg_jitter(), 1000000.) << "|"
              << std::setw(columnWidth - 1) << checkForNoData(stats.get_min_jitter(), 1000000.) << "|"
              << std::setw(columnWidth - 1) << checkForNoData(stats.get_max_rx_jitter(), 1000000.) << "|"
              << std::setw(columnWidth - 1) << checkForNoData(stats.get_avg_rx_jitter(), 1000000.) << "|"
              << std::setw(columnWidth - 1) << "" << "|" << std::endl;
    printHorizontalLine((columns - 1) * columnWidth + firstColumnWidth);

    // API's Lost Commands
    printHorizontalLine((5) * columnWidth + firstColumnWidth);
    std::cout << std::left << std::setw(firstColumnWidth) << "Metrics" << " " << std::setw(columnWidth - 1) << "" << "|"
              << std::setw(columnWidth - 1) << "WindowMax" << "|" << std::setw(columnWidth - 1) << "WindowAvg" << "|"
              << std::setw(columnWidth - 1) << "WindowMin" << "|" << std::setw(columnWidth - 1) << "Overall" << "|"
              << std::endl;

    printHorizontalLine((5) * columnWidth + firstColumnWidth);
    std::cout << std::left << std::setw(firstColumnWidth / firstColumnPart) << ""
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|" << " "
              << std::setw(columnWidth - 1) << "API RX Stamp" << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_max_lost_api(), 1, 0) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_avg_lost_api(), 1, 0) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_min_lost_api(), 1, 0) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.sum_lost_api, 1, 0) << "|" << std::endl;
    std::cout << std::setw(firstColumnWidth / firstColumnPart) << "API"
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|Packet Lost" << " "
              << std::setw(columnWidth - 1) << "RSHB TX Stamp" << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_max_lost_cbun(), 1, 0) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_avg_lost_cbun(), 1, 0) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_min_lost_cbun(), 1, 0) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.sum_lost_cbun, 1, 0) << "|" << std::endl;
    std::cout << std::left << std::setw(firstColumnWidth / firstColumnPart) << ""
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|" << " "
              << std::setw(columnWidth - 1) << "RSHB SEQ" << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_max_lost_seq(), 1, 0) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_avg_lost_seq(), 1, 0) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.get_min_lost_seq(), 1, 0) << "|" << std::setw(columnWidth - 1)
              << checkForNoData(stats.local_packets_lost_seq, 1, 0) << "|" << std::endl;
    printHorizontalLine((5) * columnWidth + firstColumnWidth);

    // CBun's Lost Commands
    printHorizontalLine((3) * columnWidth + firstColumnWidth);
    std::cout << std::left << std::setw(firstColumnWidth) << "Metrics" << " " << std::setw(columnWidth - 1) << "" << "|"
              << std::setw(columnWidth - 1) << "WindowNumber" << "|" << std::setw(columnWidth - 1) << "GlobalNumber"
              << "|" << std::endl;
    printHorizontalLine((3) * columnWidth + firstColumnWidth);
    std::cout << std::left << std::setw(firstColumnWidth / firstColumnPart) << ""
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|" << " "
              << std::setw(columnWidth - 1) << "SEQ" << "|" << std::setw(columnWidth - 1)
              << a_cbun_stats.cmd_lost_window_seq << "|" << std::setw(columnWidth - 1)
              << a_cbun_stats.cmd_lost_global_seq << "|" << std::endl;
    std::cout << std::setw(firstColumnWidth / firstColumnPart) << "CBUN"
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|Commands Lost" << " "
              << std::setw(columnWidth - 1) << "" << "|" << std::setw(columnWidth - 1) << "" << "|"
              << std::setw(columnWidth - 1) << "" << "|" << std::endl;
    std::cout << std::left << std::setw(firstColumnWidth / firstColumnPart) << ""
              << std::setw((firstColumnPart - 1) * firstColumnWidth / firstColumnPart) << "|" << " "
              << std::setw(columnWidth - 1) << "Stamp" << "|" << std::setw(columnWidth - 1)
              << a_cbun_stats.cmd_lost_window_timestmp << "|" << std::setw(columnWidth - 1)
              << a_cbun_stats.cmd_lost_global_timestmp << "|" << std::endl;
    printHorizontalLine((3) * columnWidth + firstColumnWidth);

    // // CBun's one
    std::cout << std::endl;
    std::cout << "FailEmpty: " << a_cbun_stats.fail_to_read_empty << "\n";
    std::cout << "FailError: " << a_cbun_stats.fail_to_read_error << "\n";

    std::cout << "API failed to receive: " << stats.get_failed_cnt() << std::endl;
}

void KordCore::setStatisticsWindow(int a_window_size) { stats.set_stats_window(a_window_size); }

int64_t KordCore::getAPIStatistics(EAPIStatistics a_value_type)
{
    switch (a_value_type) {
    case MAX_RX_GLOBAL:
        return stats.get_max_rx();
    case MIN_RX_GLOBAL:
        return stats.get_min_rx();
    case AVG_RX_GLOBAL:
        return stats.get_avg_rx();
    case RSHB_JITTER_MAX_LOCAL:
        return stats.get_max_jitter();
    case RSHB_JITTER_MIN_LOCAL:
        return stats.get_min_jitter();
    case RSHB_JITTER_AVG_LOCAL:
        return stats.get_avg_jitter();
    case RSHB_JITTER_AVG_GLOBAL:
        return stats.get_avg_rx_jitter();
    case RSHB_JITTER_MAX_GLOBAL:
        return stats.get_max_rx_jitter();
    case MAX_TX_GLOBAL:
        return stats.get_max_tx();
    case MIN_TX_GLOBAL:
        return stats.get_min_tx();
    case AVG_TX_GLOBAL:
        return stats.get_avg_tx();
    case MAX_LOST_API:
        return stats.get_max_lost_api();
    case MIN_LOST_API:
        return stats.get_min_lost_api();
    case AVG_LOST_API:
        return stats.get_avg_lost_api();
    case MAX_LOST_CBUN:
        return stats.get_max_lost_cbun();
    case MIN_LOST_CBUN:
        return stats.get_min_lost_cbun();
    case AVG_LOST_CBUN:
        return stats.get_avg_lost_cbun();
    case MAX_LOST_SEQ:
    case RSHB_CONS_LOST_COUNTER_MAX_LOCAL:
        return stats.get_max_lost_seq();
    case MIN_LOST_SEQ:
        return stats.get_min_lost_seq();
    case AVG_LOST_SEQ:
    case RSHB_CONS_LOST_COUNTER_AVG_LOCAL:
        return stats.get_avg_lost_seq();
    case LOST_TOTAL_API:
        return stats.sum_lost_api;
    case LOST_TOTAL_CBUN:
        return stats.sum_lost_cbun;
    case LOST_TOTAL_SEQ:
    case RSHB_LOST_COUNTER_LOCAL:
        return stats.local_packets_lost_seq;
    case RSHB_CONS_LOST_COUNTER_MAX_GLOBAL:
        return stats.get_max_drop_global();
    case FAILED_RCV:
        return stats.get_failed_cnt();
    default:
        break;
    }

    return {};
}

void KordCore::spin()
{
    timespec next_tick = kr2::utils::Timex::thisTick(ctlrc_update_ts_, 2);
    kr2::utils::Timex::nanosleepUntil(next_tick);
    ctlrc_update_ts_ = next_tick;
}

void KordCore::makeCommandMoveJ(ContentItem *a_content_item, const KordCore::RobotArmCommand &a_jcmd)
{
    a_content_item->clear();
    a_content_item->setItemID(EKORDItemID::eCommandMoveJ);
    auto time_now = std::chrono::steady_clock::now().time_since_epoch().count();
    a_content_item->addData<uint16_t>(static_cast<uint16_t>(a_jcmd.seq_),
                                      his_->cmd_joint_dfd_.getOffset(EKORDDataID::eSequenceNumber));
    a_content_item->addData<int64_t>(time_now, his_->cmd_joint_dfd_.getOffset(EKORDDataID::eTxStamp));
    a_content_item->addData<uint8_t>(0x01, his_->cmd_joint_dfd_.getOffset(EKORDDataID::eCTRMovementMask));
    a_content_item->addData<std::array<double, 7>>(a_jcmd.positions_,
                                                   his_->cmd_joint_dfd_.getOffset(EKORDDataID::eJConfigurationArm));
    a_content_item->addData<uint8_t>(a_jcmd.tt_, his_->cmd_joint_dfd_.getOffset(EKORDDataID::eTMovementType));
    a_content_item->addData<double>(a_jcmd.tt_value_, his_->cmd_joint_dfd_.getOffset(EKORDDataID::eTMovementValue));
    a_content_item->addData<uint8_t>(a_jcmd.bt_, his_->cmd_joint_dfd_.getOffset(EKORDDataID::eTBlendType));
    a_content_item->addData<double>(a_jcmd.bt_value_, his_->cmd_joint_dfd_.getOffset(EKORDDataID::eTBlendValue));
    a_content_item->addData<uint8_t>(a_jcmd.ot_, his_->cmd_joint_dfd_.getOffset(EKORDDataID::eTOverlayType));
    a_content_item->addData<double>(a_jcmd.sync_time_, his_->cmd_joint_dfd_.getOffset(EKORDDataID::eTSyncValue));

    std::uint32_t crc =
        CRC::Calculate(a_content_item->getItemData(), a_content_item->getItemDataLength(), CRC::CRC_16_MODBUS());
    a_content_item->addData<uint16_t>(crc, his_->cmd_joint_dfd_.getOffset(EKORDDataID::eCRCValue));
}

void KordCore::makeCommandMoveD(ContentItem *a_content_item,
                                const std::array<double, 7UL> &a_j_dcmd,
                                const std::array<double, 7UL> &a_jd_dcmd,
                                const std::array<double, 7UL> &a_jdd_dcmd,
                                const std::array<double, 7UL> &a_torque_dcmd,
                                unsigned int a_seq_num)
{
    a_content_item->clear();
    a_content_item->setItemID(EKORDItemID::eCommandMoveDirect);

    a_content_item->addData<uint16_t>(static_cast<uint16_t>(a_seq_num),
                                      his_->cmd_direct_dfd_.getOffset(EKORDDataID::eSequenceNumber));
    a_content_item->addData<int64_t>(std::chrono::steady_clock::now().time_since_epoch().count(),
                                     his_->cmd_direct_dfd_.getOffset(EKORDDataID::eTxStamp));

    a_content_item->addData<std::array<double, 7>>(a_j_dcmd,
                                                   his_->cmd_direct_dfd_.getOffset(EKORDDataID::eJConfigurationArm));
    a_content_item->addData<std::array<double, 7>>(a_jd_dcmd, his_->cmd_direct_dfd_.getOffset(EKORDDataID::eJSpeedArm));
    a_content_item->addData<std::array<double, 7>>(a_jdd_dcmd,
                                                   his_->cmd_direct_dfd_.getOffset(EKORDDataID::eJAccelerationArm));
    a_content_item->addData<std::array<double, 7>>(a_torque_dcmd,
                                                   his_->cmd_direct_dfd_.getOffset(EKORDDataID::eJTorqueArm));

    std::uint32_t crc =
        CRC::Calculate(a_content_item->getItemData(), a_content_item->getItemDataLength(), CRC::CRC_16_MODBUS());
    a_content_item->addData<uint16_t>(crc, his_->cmd_direct_dfd_.getOffset(EKORDDataID::eCRCValue));
}

void KordCore::makeCommandMoveL(ContentItem *a_content_item, const KordCore::RobotFrameCommand &a_lcmd)
{
    // cmdl_.reset();

    a_content_item->clear();
    a_content_item->setItemID(EKORDItemID::eCommandMoveL);
    auto time_now = std::chrono::steady_clock::now().time_since_epoch().count();
    a_content_item->addData<uint16_t>(a_lcmd.seq_, his_->cmd_linear_dfd_.getOffset(EKORDDataID::eSequenceNumber));
    a_content_item->addData<int64_t>(time_now, his_->cmd_linear_dfd_.getOffset(EKORDDataID::eTxStamp));
    a_content_item->addData<uint8_t>(0x01, his_->cmd_linear_dfd_.getOffset(EKORDDataID::eCTRMovementMask));
    a_content_item->addData<std::array<double, 6UL>>(a_lcmd.tcp_target_,
                                                     his_->cmd_linear_dfd_.getOffset(EKORDDataID::eFrmTCPPose));
    a_content_item->addData<uint8_t>(a_lcmd.tt_, his_->cmd_linear_dfd_.getOffset(EKORDDataID::eTMovementType));
    a_content_item->addData<double>(a_lcmd.tt_value_, his_->cmd_linear_dfd_.getOffset(EKORDDataID::eTMovementValue));
    a_content_item->addData<uint8_t>(a_lcmd.bt_, his_->cmd_linear_dfd_.getOffset(EKORDDataID::eTBlendType));
    a_content_item->addData<double>(a_lcmd.bt_value_, his_->cmd_linear_dfd_.getOffset(EKORDDataID::eTBlendValue));
    a_content_item->addData<uint8_t>(a_lcmd.ot_, his_->cmd_linear_dfd_.getOffset(EKORDDataID::eTOverlayType));
    a_content_item->addData<double>(a_lcmd.sync_time_, his_->cmd_linear_dfd_.getOffset(EKORDDataID::eTSyncValue));

    std::uint32_t crc =
        CRC::Calculate(a_content_item->getItemData(), a_content_item->getItemDataLength(), CRC::CRC_16_MODBUS());
    a_content_item->addData<uint16_t>(crc, his_->cmd_linear_dfd_.getOffset(EKORDDataID::eCRCValue));
}

void KordCore::makeCommandMoveManifold(ContentItem *a_content_item, const RobotArmCommand &a_cmd)
{
    a_content_item->clear();
    a_content_item->setItemID(EKORDItemID::eCommandManifold);
    auto time_now = std::chrono::steady_clock::now().time_since_epoch().count();

    a_content_item->addData<uint16_t>(a_cmd.seq_, his_->cmd_manifold_dfd_.getOffset(EKORDDataID::eSequenceNumber));
    a_content_item->addData<int64_t>(time_now, his_->cmd_manifold_dfd_.getOffset(EKORDDataID::eTxStamp));
    a_content_item->addData<double>(a_cmd.manifold_joint_speed_,
                                    his_->cmd_manifold_dfd_.getOffset(EKORDDataID::eManifoldJointSpeed));

    std::uint32_t crc =
        CRC::Calculate(a_content_item->getItemData(), a_content_item->getItemDataLength(), CRC::CRC_16_MODBUS());
    a_content_item->addData<uint16_t>(crc, his_->cmd_manifold_dfd_.getOffset(EKORDDataID::eCRCValue));
}

void KordCore::makeCommandSetFrame(ContentItem *a_content_item,
                                   const KordCore::RobotSetupCommand &a_lcmd,
                                   int64_t &a_timeStamp)
{
    // cmdl_.reset();

    a_content_item->clear();
    a_content_item->setItemID(EKORDItemID::eCommandSetFrame);
    auto time_now = std::chrono::steady_clock::now().time_since_epoch().count();
    a_content_item->addData<uint16_t>(static_cast<uint16_t>(a_lcmd.seq_),
                                      his_->cmd_setFrame_.getOffset(EKORDDataID::eSequenceNumber));
    a_content_item->addData<int64_t>(time_now, his_->cmd_setFrame_.getOffset(EKORDDataID::eTxStamp));
    a_timeStamp = time_now;
    a_content_item->addData<uint8_t>(a_lcmd.frame_id_, his_->cmd_setFrame_.getOffset(EKORDDataID::eCTRSetFrameId));
    a_content_item->addData<std::array<double, 6UL>>(a_lcmd.pose_,
                                                     his_->cmd_setFrame_.getOffset(EKORDDataID::eCTRSetFramePose));
    a_content_item->addData<uint8_t>(a_lcmd.ref_frame_, his_->cmd_setFrame_.getOffset(EKORDDataID::eCTRSetFrameRef));

    std::uint32_t crc =
        CRC::Calculate(a_content_item->getItemData(), a_content_item->getItemDataLength(), CRC::CRC_16_MODBUS());
    a_content_item->addData<uint16_t>(crc, his_->cmd_setFrame_.getOffset(EKORDDataID::eCRCValue));
}

void KordCore::makeCommandSetLoad(ContentItem *a_content_item,
                                  const KordCore::RobotSetupCommand &a_lcmd,
                                  int64_t &a_timeStamp)
{
    // cmdl_.reset();

    a_content_item->clear();
    a_content_item->setItemID(EKORDItemID::eCommandSetLoad);
    auto time_now = std::chrono::steady_clock::now().time_since_epoch().count();
    a_content_item->addData<uint16_t>(static_cast<uint16_t>(a_lcmd.seq_),
                                      his_->cmd_setLoad_.getOffset(EKORDDataID::eSequenceNumber));
    a_content_item->addData<int64_t>(time_now, his_->cmd_setLoad_.getOffset(EKORDDataID::eTxStamp));
    a_timeStamp = time_now;
    a_content_item->addData<uint8_t>(a_lcmd.load_id_, his_->cmd_setLoad_.getOffset(EKORDDataID::eCTRSetLoadId));
    a_content_item->addData<double>(a_lcmd.load_mass_, his_->cmd_setLoad_.getOffset(EKORDDataID::eCTRSetLoadMass));
    a_content_item->addData<std::array<double, 3UL>>(a_lcmd.load_cog_,
                                                     his_->cmd_setLoad_.getOffset(EKORDDataID::eCTRSetLoadCoG));
    a_content_item->addData<std::array<double, 6UL>>(a_lcmd.load_inertia_,
                                                     his_->cmd_setLoad_.getOffset(EKORDDataID::eCTRSetLoadInertia));

    std::uint32_t crc =
        CRC::Calculate(a_content_item->getItemData(), a_content_item->getItemDataLength(), CRC::CRC_16_MODBUS());
    a_content_item->addData<uint16_t>(crc, his_->cmd_setLoad_.getOffset(EKORDDataID::eCRCValue));
}

void KordCore::makeCommandCleanAlarm(ContentItem *a_content_item,
                                     const KordCore::RobotSetupCommand &a_lcmd,
                                     int64_t &a_timeStamp)
{
    // cmdl_.reset();

    a_content_item->clear();
    a_content_item->setItemID(EKORDItemID::eCommandCleanAlarm);
    auto time_now = std::chrono::steady_clock::now().time_since_epoch().count();
    a_content_item->addData<uint16_t>(static_cast<uint16_t>(a_lcmd.seq_),
                                      his_->cmd_cleanAlarm_.getOffset(EKORDDataID::eSequenceNumber));
    a_content_item->addData<int64_t>(time_now, his_->cmd_cleanAlarm_.getOffset(EKORDDataID::eTxStamp));
    a_timeStamp = time_now;
    a_content_item->addData<uint8_t>(a_lcmd.alarm_id_, his_->cmd_cleanAlarm_.getOffset(EKORDDataID::eCTRCleanAlarmID));

    std::uint32_t crc =
        CRC::Calculate(a_content_item->getItemData(), a_content_item->getItemDataLength(), CRC::CRC_16_MODBUS());
    a_content_item->addData<uint16_t>(crc, his_->cmd_cleanAlarm_.getOffset(EKORDDataID::eCRCValue));
}

void KordCore::makeCommandMoveDynL(ContentItem *a_content_item,
                                   const std::array<double, 6UL> &a_in_lcmd,
                                   unsigned int a_seq_num)
{
    // cmdl_.reset();
    // boost::crc_optimal<16, 0x8005, 0xFFFF, 0, true, true> crc;

    // a_out_eth_frame->frame_id_ = htons(static_cast<uint16_t>(KORD_REQ_MOVEL_DYN));
    // a_out_eth_frame->session_id_ = 0;
    // a_out_eth_frame->payload_length_ = htons(cmdl_.getLength());
    // cmdl_.sequence_number_ = static_cast<uint16_t>(a_seq_num);
    // cmdl_.setTXStampNSec(std::chrono::system_clock::now().time_since_epoch().count());

    // //TODO:
    // cmdl_.setTCPTarget(a_in_lcmd.data());

    // void setTCPTarget(const double pose[6]);
    // void setTCPVelocityTarget(const double pose[6]);
    // void setTCPAccelerationTarget(const double pose[6]);

    // crc.process_bytes(a_out_eth_frame, kord::KORDFrame::header_len_);
    // crc.process_bytes(&cmdl_, cmdl_.getLength() - 2);  // process bytes without CRC
    // cmdl_.crc_ = crc.checksum();
}

void KordCore::makeCommandMoveVelocityL(ContentItem *a_content_item,
                                        const std::array<double, 6UL> &a_vels,
                                        unsigned int a_seq_num,
                                        long a_sync_value,
                                        double a_period,
                                        double a_timeout)
{
    a_content_item->clear();
    a_content_item->setItemID(EKORDItemID::eCommandMoveVelocity);
    auto time_now = std::chrono::steady_clock::now().time_since_epoch().count();
    a_content_item->addData<uint16_t>(a_seq_num, his_->cmd_velocity_dfd_.getOffset(EKORDDataID::eSequenceNumber));
    a_content_item->addData<int64_t>(time_now, his_->cmd_velocity_dfd_.getOffset(EKORDDataID::eTxStamp));
    a_content_item->addData<std::array<double, 6UL>>(a_vels,
                                                     his_->cmd_velocity_dfd_.getOffset(EKORDDataID::eFrmTCPVelocity));
    a_content_item->addData<uint32_t>(a_sync_value, his_->cmd_velocity_dfd_.getOffset(EKORDDataID::eVelMoveSync));
    a_content_item->addData<double>(a_period, his_->cmd_velocity_dfd_.getOffset(EKORDDataID::eVelMovePeriod));
    a_content_item->addData<double>(a_timeout, his_->cmd_velocity_dfd_.getOffset(EKORDDataID::eVelMoveTimeout));

    std::uint32_t crc =
        CRC::Calculate(a_content_item->getItemData(), a_content_item->getItemDataLength(), CRC::CRC_16_MODBUS());
    a_content_item->addData<uint16_t>(crc, his_->cmd_velocity_dfd_.getOffset(EKORDDataID::eCRCValue));
}

void KordCore::makeArmStatusRequest(ContentItem *a_content_item)
{
    a_content_item->clear();
    a_content_item->setItemID(EKORDItemID::eRequestStatusArm);
    a_content_item->addData<uint16_t>(0x0, his_->req_arm_status_dfd_.getOffset(EKORDDataID::eSequenceNumber));
    a_content_item->addData<int64_t>(std::chrono::steady_clock::now().time_since_epoch().count(),
                                     his_->req_arm_status_dfd_.getOffset(EKORDDataID::eTxStamp));
    a_content_item->addData<uint16_t>(0xFFFF, his_->req_arm_status_dfd_.getOffset(EKORDDataID::eCRCValue));
}

void KordCore::makeCommandFirmware(ContentItem *a_content_item,
                                   const std::array<EJointFirmwareCommand, 7UL> &a_in_fw_cmd,
                                   unsigned int a_seq_num,
                                   int64_t &a_timeStamp)
{
    int64_t tx_ts = std::chrono::steady_clock::now().time_since_epoch().count();

    a_content_item->setItemID(EKORDItemID::eCommandJointFirmware);
    a_content_item->addData<uint16_t>(a_seq_num, his_->cmd_joint_fw_dfd_.getOffset(EKORDDataID::eSequenceNumber));
    a_content_item->addData<int64_t>(tx_ts, his_->cmd_joint_fw_dfd_.getOffset(EKORDDataID::eTxStamp));
    a_timeStamp = tx_ts;
    a_content_item->addData(a_in_fw_cmd, his_->cmd_joint_fw_dfd_.getOffset(EKORDDataID::eJControlCMD));

    std::uint32_t crc =
        CRC::Calculate(a_content_item->getItemData(), a_content_item->getItemDataLength(), CRC::CRC_16_MODBUS());
    a_content_item->addData<uint16_t>(crc, his_->cmd_joint_fw_dfd_.getOffset(EKORDDataID::eCRCValue));
}

int KordCore::setRequestContentItem(ContentItem *a_content, const Request &a_request, unsigned short a_seq)
{
    if (!a_content)
        return -1;

    a_content->clear();

    switch (a_request.system_request_type_) {
    case eRCAPICommand: {
        auto rc_request = dynamic_cast<const RequestRCAPICommand &>(a_request);
        a_content->setItemID(EKORDItemID::eCommandRCAPI);

        // a_content->addData<uint16_t>(a_seq,
        // his_->cmd_rc_api_.getOffset(EKORDDataID::eSequenceNumber));
        a_content->addData<uint16_t>(rc_request.command_id_, his_->cmd_rc_api_.getOffset(EKORDDataID::eRCAPICommandID));
        a_content->addData<uint32_t>(rc_request.payload_length_,
                                     his_->cmd_rc_api_.getOffset(EKORDDataID::eRCAPICommandLength));

        // std::cout << rc_request.payload_length_ << '\n';

        a_content->addData(rc_request.payload_, his_->cmd_rc_api_.getOffset(EKORDDataID::eRCAPICommandPayload));

        for (uint8_t i : rc_request.payload_) {
            std::cout << (int)i << '\n';
        }

        std::uint32_t crc =
            CRC::Calculate(a_content->getItemData(), a_content->getItemDataLength(), CRC::CRC_16_MODBUS());
        a_content->addData<uint16_t>(crc, his_->cmd_rc_api_.getOffset(EKORDDataID::eCRCValue));
        // std::uint32_t crc = CRC::Calculate(a_content->getItemData(),
        // a_content->getItemDataLength(), CRC::CRC_16_MODBUS());
        // a_content->addData<uint16_t>(crc,
        // his_->cmd_rc_api_.getOffset(EKORDDataID::eCRCValue));

        return 0;
    }
    case eTransferLogFiles: {
        std::cout << "Request Retrieve Log Files\n";

        auto system_request = dynamic_cast<const RequestSystem &>(a_request);
        a_content->setItemID(EKORDItemID::eRequestSystem);

        a_content->addData<uint16_t>(a_seq, his_->req_sys_dfd_.getOffset(EKORDDataID::eSequenceNumber));
        a_content->addData<int64_t>(system_request.request_rid_, his_->req_sys_dfd_.getOffset(EKORDDataID::eTxStamp));
        a_content->addData<uint16_t>(system_request.system_request_type_,
                                     his_->req_sys_dfd_.getOffset(EKORDDataID::eCTRCommandItem));

        std::uint32_t crc =
            CRC::Calculate(a_content->getItemData(), a_content->getItemDataLength(), CRC::CRC_16_MODBUS());
        a_content->addData<uint16_t>(crc, his_->req_sys_dfd_.getOffset(EKORDDataID::eCRCValue));

        return 0;
    }
    case eTransferDashboardJson: {
        std::cout << "Request Retrieve Dashboard Json Files\n";

        auto system_request = dynamic_cast<const RequestSystem &>(a_request);
        a_content->setItemID(EKORDItemID::eRequestSystem);

        a_content->addData<uint16_t>(a_seq, his_->req_sys_dfd_.getOffset(EKORDDataID::eSequenceNumber));
        a_content->addData<int64_t>(system_request.request_rid_, his_->req_sys_dfd_.getOffset(EKORDDataID::eTxStamp));
        a_content->addData<uint16_t>(system_request.system_request_type_,
                                     his_->req_sys_dfd_.getOffset(EKORDDataID::eCTRCommandItem));

        std::uint32_t crc =
            CRC::Calculate(a_content->getItemData(), a_content->getItemDataLength(), CRC::CRC_16_MODBUS());
        a_content->addData<uint16_t>(crc, his_->req_sys_dfd_.getOffset(EKORDDataID::eCRCValue));

        return 0;
    }
    case eTransferCalibrationData: {
        std::cout << "Request Retrieve Calibration Data Files\n";

        auto system_request = dynamic_cast<const RequestSystem &>(a_request);
        a_content->setItemID(EKORDItemID::eRequestSystem);

        a_content->addData<uint16_t>(a_seq, his_->req_sys_dfd_.getOffset(EKORDDataID::eSequenceNumber));
        a_content->addData<int64_t>(system_request.request_rid_, his_->req_sys_dfd_.getOffset(EKORDDataID::eTxStamp));
        a_content->addData<uint16_t>(system_request.system_request_type_,
                                     his_->req_sys_dfd_.getOffset(EKORDDataID::eCTRCommandItem));

        std::uint32_t crc =
            CRC::Calculate(a_content->getItemData(), a_content->getItemDataLength(), CRC::CRC_16_MODBUS());
        a_content->addData<uint16_t>(crc, his_->req_sys_dfd_.getOffset(EKORDDataID::eCRCValue));

        return 0;
    }

    case eTransferFiles: {
        std::cout << "Request Retrieve Multiple Files\n";

        auto transfer_request = dynamic_cast<const RequestTransfer &>(a_request);
        a_content->setItemID(EKORDItemID::eRequestTransfer);
        a_content->addData<uint16_t>(a_seq, his_->req_transf_dfd_.getOffset(EKORDDataID::eSequenceNumber));
        a_content->addData<int64_t>(transfer_request.request_rid_,
                                    his_->req_transf_dfd_.getOffset(EKORDDataID::eTxStamp));
        a_content->addData<uint16_t>(transfer_request.system_request_type_,
                                     his_->req_transf_dfd_.getOffset(EKORDDataID::eCTRCommandItem));

        a_content->addData<uint32_t>(transfer_request.tranfer_mask_,
                                     his_->req_transf_dfd_.getOffset(EKORDDataID::eTranferMask));

        std::uint32_t crc =
            CRC::Calculate(a_content->getItemData(), a_content->getItemDataLength(), CRC::CRC_16_MODBUS());
        a_content->addData<uint16_t>(crc, his_->req_transf_dfd_.getOffset(EKORDDataID::eCRCValue));

        return 0;
    }
    case eServerEnableCommunication:
    case eServerDisableCommunication: {
        auto system_request = dynamic_cast<const RequestSystem &>(a_request);
        a_content->setItemID(EKORDItemID::eRequestSystem);

        a_content->addData<uint16_t>(a_seq, his_->req_sys_dfd_.getOffset(EKORDDataID::eSequenceNumber));
        a_content->addData<int64_t>(system_request.request_rid_, his_->req_sys_dfd_.getOffset(EKORDDataID::eTxStamp));
        a_content->addData<uint16_t>(system_request.system_request_type_,
                                     his_->req_sys_dfd_.getOffset(EKORDDataID::eCTRCommandItem));

        std::uint32_t crc =
            CRC::Calculate(a_content->getItemData(), a_content->getItemDataLength(), CRC::CRC_16_MODBUS());
        a_content->addData<uint16_t>(crc, his_->req_sys_dfd_.getOffset(EKORDDataID::eCRCValue));

        return 0;
    }
    case eIOSet: {
        auto io_request = dynamic_cast<const RequestIO &>(a_request);
        a_content->setItemID(EKORDItemID::eCommandSetIODigitalOut);

        a_content->addData<uint16_t>(a_seq, his_->req_io_dfd_.getOffset(EKORDDataID::eSequenceNumber));
        a_content->addData<int64_t>(io_request.request_rid_, his_->req_io_dfd_.getOffset(EKORDDataID::eTxStamp));
        a_content->addData<uint16_t>(io_request.system_request_type_,
                                     his_->req_io_dfd_.getOffset(EKORDDataID::eCTRCommandItem));

        a_content->addData<uint8_t>(io_request.value_, his_->req_io_dfd_.getOffset(EKORDDataID::eIODigitalValue));
        a_content->addData<int64_t>(io_request.mask_, his_->req_io_dfd_.getOffset(EKORDDataID::eIODigitalOutput));

        a_content->addData<int32_t>(io_request.config_id_,
                                    his_->req_io_dfd_.getOffset(EKORDDataID::eIODigitalSafetyConfig));

        // eIOAnalogMask

        std::uint32_t crc =
            CRC::Calculate(a_content->getItemData(), a_content->getItemDataLength(), CRC::CRC_16_MODBUS());
        a_content->addData<uint16_t>(crc, his_->req_io_dfd_.getOffset(EKORDDataID::eCRCValue));

        return 0;
    }
    case eServer: {
        auto server_request = dynamic_cast<const RequestServer &>(a_request);
        a_content->setItemID(EKORDItemID::eRequestServer);

        a_content->addData<uint16_t>(a_seq, his_->req_server_dfd_.getOffset(EKORDDataID::eSequenceNumber));
        a_content->addData<int64_t>(server_request.request_rid_, his_->req_server_dfd_.getOffset(EKORDDataID::eTxStamp));

        a_content->addData<uint16_t>(server_request.command_,
                                     his_->req_server_dfd_.getOffset(EKORDDataID::eServerServiceCommand));
        a_content->addData<uint16_t>(server_request.service_id_,
                                     his_->req_server_dfd_.getOffset(EKORDDataID::eServerServiceId));

        // We should encode parameters, so we can send it via KORD, if they are set
        if (server_request.parameters_ != nullptr) {
            std::vector<uint8_t> enc_parameters = server_request.parameters_->dump();
            a_content->addData(enc_parameters, his_->req_server_dfd_.getOffset(EKORDDataID::eServerPayload));
        }

        std::uint32_t crc =
            CRC::Calculate(a_content->getItemData(), a_content->getItemDataLength(), CRC::CRC_16_MODBUS());
        a_content->addData<uint16_t>(crc, his_->req_io_dfd_.getOffset(EKORDDataID::eCRCValue));
    }
    default: {
        return -2;
    }
    }
}

int KordCore::setStatusEchoContentItem(ContentItem *a_content, const StatusFrameEcho &a_echo)
{
    if (!a_content)
        return -1;
    a_content->clear();

    a_content->setItemID(EKORDItemID::eStatusEchoResponse);
    a_content->addData<int64_t>(a_echo.tx_time_stamp_, his_->res_statusEcho_dfd_.getOffset(EKORDDataID::eTxStamp));
    a_content->addData<int64_t>(a_echo.rx_time_stamp_, his_->res_statusEcho_dfd_.getOffset(EKORDDataID::eRxStamp));

    std::uint32_t crc = CRC::Calculate(a_content->getItemData(), a_content->getItemDataLength(), CRC::CRC_16_MODBUS());
    a_content->addData<uint16_t>(crc, his_->res_statusEcho_dfd_.getOffset(EKORDDataID::eCRCValue));

    return 0;
}

unsigned int KordCore::sendFrame(const KORDFrame *a_frame) { return conn_container_->sendFrame(a_frame); }

struct timespec KordCore::ctlrcUpdateTS() const { return ctlrc_update_ts_; }

void KordCore::setCtlrcUpdateTS(struct timespec new_ts) { ctlrc_update_ts_ = new_ts; }

unsigned int KordCore::requestArmStatus()
{
    makeArmStatusRequest(&his_->tx_content_items_[0]);
    his_->content_builder_.clear();
    his_->content_builder_.addContentItem(his_->tx_content_items_[0]);
    his_->populateFrameFromContent();
    return conn_container_->sendFrame(&his_->eth_kord_proto_frm_);
}

void KordCore::updateRecentCommandStatus(CommandStatuses &a_command_statuses) const
{
    auto last_command_status_token_id_ =
        his_->status_parser_.getData<uint64_t>(his_->status_dfd_.getOffset(EKORDDataID::eLastCommandToken));
    auto last_command_status_error_code_ =
        his_->status_parser_.getData<int8_t>(his_->status_dfd_.getOffset(EKORDDataID::eLastCommandErrorCode));

    CommandStatus last_command_status = CommandStatus();
    last_command_status.token = last_command_status_token_id_;
    last_command_status.error_code = last_command_status_error_code_;
    // std::cout << last_command_status.token << " " <<
    // static_cast<int16_t>(last_command_status.error_code) << '\n';
    a_command_statuses.addStatus(last_command_status);
}

void KordCore::getRecentCommandStatus(CommandStatuses &a_command_statuses) const
{
    a_command_statuses = his_->sync_command_statuses_;
}

// Decoder of temperature, received in the form of uint16_t (a...b) = a...,b
double temperatureDecoder(const uint16_t &a_temp_encoded)
{
    int after_point = a_temp_encoded % 10;
    int before_point = a_temp_encoded / 10;
    return before_point + double(after_point) / 10;
}

std::array<double, 7> transformTemperaturesArray(const std::array<uint16_t, 7> &an_array)
{
    std::array<double, 7> tmp{};

    std::transform(an_array.begin(), an_array.end(), tmp.begin(), [](const uint16_t &c_temp) {
        return temperatureDecoder(c_temp);
    });

    return tmp;
}

void KordCore::getRecentArmStatus(RobotArmStatus &a_status) const { a_status = his_->sync_arm_status_; }

void KordCore::updateRecentArmStatus(RobotArmStatus &a_status) const
{
    // TODO check the data are recent enough
    std::array<double, 7> data{};
    std::array<double, 6> data6{};

    // TBC This should be preallocated
    std::vector<uint8_t> buffer{};

    // clang-format off
    a_status.positions_ = his_->status_parser_.getData<std::array<double, 7>>(his_->status_dfd_.getOffset(EKORDDataID::eJConfigurationArm));
    a_status.torques_ = his_->status_parser_.getData<std::array<double, 7>>(his_->status_dfd_.getOffset(EKORDDataID::eJTorqueArm));
    a_status.speed_ = his_->status_parser_.getData<std::array<double, 7>>(his_->status_dfd_.getOffset(EKORDDataID::eJSpeedArm));
    a_status.accelerations_ = his_->status_parser_.getData<std::array<double, 7>>(his_->status_dfd_.getOffset(EKORDDataID::eJAccelerationArm));
    a_status.tcp_model_ = his_->status_parser_.getData<std::array<double, 6>>(his_->status_dfd_.getOffset(EKORDDataID::eFrmTCPPose));

    // Joints Temperatures
    std::array<uint16_t, 7> coded_temperatures{};
    coded_temperatures = his_->status_parser_.getData<std::array<uint16_t, 7>>(his_->status_dfd_.getOffset(EKORDDataID::eJTemperatureBoard));

    a_status.jbs_board_temperatures_ = transformTemperaturesArray(coded_temperatures);

    coded_temperatures = his_->status_parser_.getData<std::array<uint16_t, 7>>(his_->status_dfd_.getOffset(EKORDDataID::eJTemperatureJointEncoder));
    a_status.jbs_joint_encoder_temperatures_ = transformTemperaturesArray(coded_temperatures);

    coded_temperatures = his_->status_parser_.getData<std::array<uint16_t, 7>>(his_->status_dfd_.getOffset(EKORDDataID::eJTemperatureRotorEncoder));
    a_status.jbs_rotor_encoder_temperatures_ = transformTemperaturesArray(coded_temperatures);

    // Joints Sensed Torques
    a_status.sens_torques_ = his_->status_parser_.getData<std::array<double, 7>>(his_->status_dfd_.getOffset(EKORDDataID::eJTorqueModel));

    // CPU State
    unsigned int cpu_state_id_ = his_->status_parser_.getData<uint8_t>(his_->status_dfd_.getOffset(EKORDDataID::eCPUStateId));
    a_status.cpu_state_[cpu_state_id_] = temperatureDecoder(his_->status_parser_.getData<uint16_t>(his_->status_dfd_.getOffset(EKORDDataID::eCPUStateVal)));

    // IOBoard Temperature
    a_status.iob_temperature_ = temperatureDecoder(his_->status_parser_.getData<uint16_t>(his_->status_dfd_.getOffset(EKORDDataID::eIOBCabinetTemperature)));

    // Frames
    unsigned int frame_id_ = his_->status_parser_.getData<uint8_t>(his_->status_dfd_.getOffset(EKORDDataID::eFrameId));
    unsigned int ref_frame_ = his_->status_parser_.getData<uint8_t>(his_->status_dfd_.getOffset(EKORDDataID::eFramePoseRef));
    a_status.frames_[frame_id_].pose_[ref_frame_] = his_->status_parser_.getData<std::array<double, 6>>(his_->status_dfd_.getOffset(EKORDDataID::eFramePose));

    // Loads
    unsigned int load_id_ = his_->status_parser_.getData<uint8_t>(his_->status_dfd_.getOffset(EKORDDataID::eLoadId));
    a_status.loads_[load_id_].cog = his_->status_parser_.getData<std::array<double, 3>>(his_->status_dfd_.getOffset(EKORDDataID::eLoadCoG));
    a_status.loads_[load_id_].mass = his_->status_parser_.getData<double>(his_->status_dfd_.getOffset(EKORDDataID::eLoadMass));
    a_status.loads_[load_id_].pose = his_->status_parser_.getData<std::array<double, 6>>(his_->status_dfd_.getOffset(EKORDDataID::eLoadPose));
    a_status.loads_[load_id_].inertia = his_->status_parser_.getData<std::array<double, 6>>(his_->status_dfd_.getOffset(EKORDDataID::eLoadInertia));

    a_status.cbun_stats_.fail_to_read_empty = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eFailToReadEmpty));
    a_status.cbun_stats_.fail_to_read_error = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eFailToReadError));

    // CBun stats
    a_status.cbun_stats_.cmd_jitter_window_max = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eMaxCmdJitterWindow));
    a_status.cbun_stats_.cmd_jitter_window_avg = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eAvgCmdJitterWindow));
    a_status.cbun_stats_.cmd_jitter_global_max = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eMaxCmdJitterGlobal));
    a_status.cbun_stats_.round_trip_window_max = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eMaxTickDelayWindow));
    a_status.cbun_stats_.round_trip_window_avg = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eAvgTickDelayWindow));
    a_status.cbun_stats_.round_trip_global_max = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eMaxTickDelayGlobal));
    a_status.cbun_stats_.cmd_lost_window_seq = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eCmdLostWindowSeq));
    a_status.cbun_stats_.cmd_lost_global_seq = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eCmdLostGlobalSeq));
    a_status.cbun_stats_.cmd_lost_window_timestmp = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eCmdLostWindowTimestamp));
    a_status.cbun_stats_.cmd_lost_global_timestmp = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eCmdLostGlobalTimestamp));
    a_status.cbun_stats_.system_jitter_window_max = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eMaxSysJitterWindow));
    a_status.cbun_stats_.system_jitter_window_avg = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eAvgSysJitterWindow));
    a_status.cbun_stats_.system_jitter_global_max = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eMaxSysJitterGlobal));

    // Controller
    a_status.rc_motion_flags_ = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eRCMotionFlags));
    a_status.rc_safety_flags_ = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eRCSafetyFlag));
    a_status.rc_safety_mode_ = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eRCSafetyMode));
    a_status.rc_master_speed_ = his_->status_parser_.getData<double>(his_->status_dfd_.getOffset(EKORDDataID::eRCMasterSpeed));
    a_status.rc_hw_flags_ = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eRCHWFlags));
    a_status.rc_button_flags_ = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eRCButtonFlags));
    a_status.rc_system_alarm_state_ = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eRCSystemAlarmState));

    a_status.max_frames_in_tick = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eMaxFramesInTick));
    a_status.faulty_frames_start = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eFaultyFramesStart));

    a_status.rc_digital_output_ = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eIODigitalOutput));
    a_status.rc_digital_input_ = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eIODigitalInput));
    a_status.rc_safe_digital_config = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eIODigitalSafetyMask));
    // eIOAnalogInput
    // eIOAnalogOutput

    a_status.latest_request_system_.request_rid_ = his_->status_parser_.getData<int64_t>(his_->status_dfd_.getOffset(EKORDDataID::eCTRCommandTS));
    a_status.latest_request_system_.system_request_type_ = static_cast<EControlCommandItems>(his_->status_parser_.getData<uint16_t>(his_->status_dfd_.getOffset(EKORDDataID::eCTRCommandItem)));
    a_status.latest_request_system_.request_status_ = static_cast<EControlCommandStatus>(his_->status_parser_.getData<uint16_t>(his_->status_dfd_.getOffset(EKORDDataID::eCTRCommandStatus)));
    a_status.latest_request_system_.error_code_ = his_->status_parser_.getData<uint32_t>(his_->status_dfd_.getOffset(EKORDDataID::eCTRCommandErrorCode));

    // Service monitoring
//    a_status.service_success_ = static_cast<bool>(his_->status_parser_.getData<uint8_t>(his_->status_dfd_.getOffset(EKORDDataID::eServerServiceSuccess)));
    a_status.service_status_ = static_cast<uint8_t>(his_->status_parser_.getData<uint8_t>(his_->status_dfd_.getOffset(EKORDDataID::eServerServiceStatus)));
    a_status.service_progress_ = static_cast<uint8_t>(his_->status_parser_.getData<uint8_t>(his_->status_dfd_.getOffset(EKORDDataID::eServerServiceProgress)));

    // TBC - fill the array/vector
    // TBC - Do check the return value
    his_->status_parser_.getData(buffer, his_->status_dfd_.getOffset(EKORDDataID::eEventsArray));
    // clang-format on

    // If the buffer is not empty there should be some events present
    // Do the conversion and unpack the events contained.
    // This loop should is TBC, an example fo fixed array

    for (SystemEvent &event : a_status.system_events_) {
        event.reset();
    }

    if (!buffer.empty() && buffer.size() >= sizeof(SystemEvent)) {
        int offset = 0;

        for (SystemEvent &event : a_status.system_events_) {
            if (buffer.size() - offset < sizeof(SystemEvent)) {
                break;
            }

            // Create a temporary vector representing the current slice of the buffer
            std::vector<uint8_t> current_slice(buffer.begin() + offset, buffer.begin() + offset + sizeof(SystemEvent));

            // Initialize the event from the current slice of the buffer
            if (!event.initFromByteArray(current_slice)) {
                break;
            }
            // Move the offset forward by the size of SystemEvent
            offset += sizeof(SystemEvent);
        }

        // Erase only the part of the buffer that has been processed
        buffer.erase(buffer.begin(), buffer.begin() + offset);
    }

    // std::cout << "Offset: " << his_->status_dfd_.getOffset(EKORDDataID::eCRCValue) <<
    // '\n'; CommandStatus
}

unsigned int KordCore::sendCommand(RobotArmCommand a_cmd)
{
    switch (a_cmd.command_type) {
    case RobotArmCommand::EType::eFW: {
        token_t not_used_token_;
        makeCommandFirmware(&his_->tx_content_items_[0], a_cmd.fw_cmds_, a_cmd.seq_, not_used_token_);
        break;
    }
    case RobotArmCommand::EType::eMOVE: {
        makeCommandMoveJ(&his_->tx_content_items_[0], a_cmd);
        break;
    }
    case RobotArmCommand::EType::eMOVEManifold: {
        makeCommandMoveManifold(&his_->tx_content_items_[0], a_cmd);
        break;
    }
    case RobotArmCommand::EType::eDJC: {
        makeCommandMoveD(&his_->tx_content_items_[0],
                         a_cmd.positions_,
                         a_cmd.speed_,
                         a_cmd.accelerations_,
                         a_cmd.torque_,
                         a_cmd.seq_);
        break;
    }
    default: {
        // error
        return 0;
    }
    }

    his_->content_builder_.clear();
    his_->content_builder_.addContentItem(his_->tx_content_items_[0]);
    his_->populateFrameFromContent();

    return conn_container_->sendFrame(&his_->eth_kord_proto_frm_);
}

unsigned int KordCore::sendCommand(RobotArmCommand a_cmd, int64_t &a_returned_token)
{
    switch (a_cmd.command_type) {
    case RobotArmCommand::EType::eFW: {
        makeCommandFirmware(&his_->tx_content_items_[0], a_cmd.fw_cmds_, a_cmd.seq_, a_returned_token);
        break;
    }
    case RobotArmCommand::EType::eMOVE: {
        makeCommandMoveJ(&his_->tx_content_items_[0], a_cmd);
        break;
    }
    case RobotArmCommand::EType::eMOVEManifold: {
        makeCommandMoveManifold(&his_->tx_content_items_[0], a_cmd);
        break;
    }
    case RobotArmCommand::EType::eDJC: {
        makeCommandMoveD(&his_->tx_content_items_[0],
                         a_cmd.positions_,
                         a_cmd.speed_,
                         a_cmd.accelerations_,
                         a_cmd.torque_,
                         a_cmd.seq_);
        break;
    }
    default: {
        // error
        return 0;
    }
    }

    his_->content_builder_.clear();
    his_->content_builder_.addContentItem(his_->tx_content_items_[0]);
    his_->populateFrameFromContent();

    return conn_container_->sendFrame(&his_->eth_kord_proto_frm_);
}

unsigned int KordCore::sendCommand(RobotFrameCommand a_cmd)
{
    switch (a_cmd.command_type) {
    case RobotFrameCommand::MOVE_POSE: {
        makeCommandMoveL(&his_->tx_content_items_[0], a_cmd);
        break;
    }

    case RobotFrameCommand::MOVE_VELOCITY: {
        makeCommandMoveVelocityL(&his_->tx_content_items_[0],
                                 a_cmd.tcp_target_velocity_,
                                 a_cmd.seq_,
                                 a_cmd.sync_value_,
                                 a_cmd.period_,
                                 a_cmd.timeout_);
        break;
    }

    case RobotFrameCommand::MOVE_POSE_DYN: {
        makeCommandMoveDynL(&his_->tx_content_items_[0], a_cmd.tcp_target_, a_cmd.seq_);
        break;
    }
    }

    his_->content_builder_.clear();
    his_->content_builder_.addContentItem(his_->tx_content_items_[0]);
    his_->populateFrameFromContent();

    return conn_container_->sendFrame(&his_->eth_kord_proto_frm_);
}

unsigned int KordCore::sendCommand(RobotSetupCommand a_cmd, int64_t &a_returned_token)
{
    switch (a_cmd.command_type) {
    case RobotSetupCommand::EType::SET_FRAME: {
        makeCommandSetFrame(&his_->tx_content_items_[0], a_cmd, a_returned_token);
        break;
    }

    case RobotSetupCommand::SET_LOAD: {
        makeCommandSetLoad(&his_->tx_content_items_[0], a_cmd, a_returned_token);
        break;
    }

    case RobotSetupCommand::CLEAN_ALARM: {
        makeCommandCleanAlarm(&his_->tx_content_items_[0], a_cmd, a_returned_token);
        break;
    }
    }

    his_->content_builder_.clear();
    his_->content_builder_.addContentItem(his_->tx_content_items_[0]);
    his_->populateFrameFromContent();

    return conn_container_->sendFrame(&his_->eth_kord_proto_frm_);
}

unsigned int KordCore::sendCommand(const Request &a_request)
{
    setRequestContentItem(&his_->tx_content_items_[0], a_request, his_->req_sys_seq_num_);
    his_->content_builder_.clear();
    his_->content_builder_.addContentItem(his_->tx_content_items_[0]);
    his_->populateFrameFromContent();
    return conn_container_->sendFrame(&his_->eth_kord_proto_frm_);
}

unsigned int KordCore::sendCommand(const KordCore::StatusFrameEcho &a_response)
{
    setStatusEchoContentItem(&his_->tx_content_items_[0], a_response);
    his_->content_builder_.clear();
    his_->content_builder_.addContentItem(his_->tx_content_items_[0]);
    his_->populateFrameFromContent();
    return conn_container_->sendFrame(&his_->eth_kord_proto_frm_);
}
