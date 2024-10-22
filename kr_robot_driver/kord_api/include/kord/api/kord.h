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

#ifndef KR2_KORD_CORE_H
#define KR2_KORD_CORE_H

#include <map>
#include <string>
#include <vector>

#include "kord/api/connection_interface.h"
#include "kord/utils/utils.h"

#include "kr2/kord/protocol/ContentFrameBuilder.h"
#include "kr2/kord/protocol/ContentFrameParser.h"
#include "kr2/kord/protocol/JointFirmwareCommand.h"
#include "kr2/kord/protocol/KORDFrames.h"
#include "kr2/kord/protocol/StatusFrameParser.h"

#include "kr2/kord/system/SystemAlarm.h"
#include "kr2/kord/system/SystemEvent.h"

#include "kord/api/api_request.h"
#include "kord/api/kord_io_request.h"

#include "circular_buffer.hpp"

/**
 * @file This is KordCore interface file
 *
 */

namespace kr2::kord {

enum TrackingType {
    TT_NONE,
    TT_TIME,
    TT_WS_TARGET_SPEED,
    TT_JS_TARGET_SPEED,
    TT_SP_APX_SPEED,
    TT_SP_CNST_SPEED,
    TT_SP_DURATION
};

enum BlendType { BT_NONE, BT_TIME, BT_WS_ACCELERATION, BT_WS_RADIUS, BT_JS_ACCELERATION };

enum OverlayType { OT_NONE, OT_VIAPOINT, OT_STOPPOINT };

struct LoadData {
    std::array<double, 6ul> pose;
    double mass;
    std::array<double, 3ul> cog;
    std::array<double, 6ul> inertia;
};

struct FrameData {
    std::map<unsigned int, std::array<double, 6>> pose_; // for tfc and wf references
};

struct CBunReceivedStatistics {
    //! Counter of how many times the \b recvfrom returned 0 read bytes.
    uint32_t fail_to_read_empty;
    //! Counter of how many times the \b recvfrom returned read bytes lesser than 0.
    uint32_t fail_to_read_error;

    int64_t cmd_jitter_window_max;
    int64_t cmd_jitter_window_avg;
    int64_t cmd_jitter_global_max;
    int64_t round_trip_window_max;
    int64_t round_trip_window_avg;
    int64_t round_trip_global_max;
    int64_t cmd_lost_window_seq;
    int64_t cmd_lost_global_seq;
    int64_t cmd_lost_window_timestmp;
    int64_t cmd_lost_global_timestmp;
    int64_t system_jitter_window_max;
    int64_t system_jitter_window_avg;
    int64_t system_jitter_global_max;
};

typedef int64_t token_t;

//
// Frame ID
//
/**
 * @brief Selector of the System Frame.
 *
 */
enum EFrameID {
    //! Robot Base Frame
    ROBOT_BASE_FRAME = 0,
    //! TFC - Tool Flange Center
    TFC_FRAME = 7,
    //! TCP - Tool Center Point
    TCP_FRAME = 8,
    //! Last Target Pose
    LAST_TARGET_POSE_FRAME = 9,
};

//
// Frame values
//
/**
 * @brief Selector Frame values.
 *
 */
enum EFrameValue {

    // Values Types

    //! Rotation and Position coordinates ([x, y, z, r, p, y])
    // World Frame Reference
    POSE_VAL_REF_WF = 0,
    // Tool Flange Center Reference
    POSE_VAL_REF_TFC = 1,
};

//
// Load ID
//
/**
 * @brief Selector of System Load identifiers.
 *
 * There are 2 types of load supported. The load representing the end of arm tool
 * and the load representing the payload being attached to the end of arm tool.
 *
 * For example the gripper mass and inertia is represented by LOAD1. The mass and
 * inertia of the item hold by a gripper is represented by LOAD2.
 *
 */
enum ELoadID {
    //! LOAD1 - End of Arm Tool
    LOAD1 = 0,
    //! LOAD2 - Payload
    LOAD2 = 1,
};

//
// Load values
//
/**
 * @brief Selector which Load values to retrieve.
 *
 */
enum ELoadValue {

    // Values Types

    //! Rotation and Position coordinates ([x, y, z, r, p, y])
    // Reference which is chosen on the tablet
    POSE_VAL = 0,
    //! Mass
    MASS_VAL = 2,
    //! Center of Gravity ([x, y, z])
    COG_VAL = 3,
    //! Inertia
    INERTIA_VAL = 4
};

//
// Sync Flags values
//
enum { F_NONE = 0 };

//! Using full rotation should be used only for non real-time communication or at init
//! of communication.
enum { F_SYNC_FULL_ROTATION = 1 };

//
// API Statistics
//
/**
 * @brief Selector which API statistics to retrieve.
 *
 */
enum EAPIStatistics {
    //! Global Max API Rx difference
    MAX_RX_GLOBAL,
    //! Global Min API Rx difference
    MIN_RX_GLOBAL,
    //! Average API Rx difference
    AVG_RX_GLOBAL,
    //! Max API Rx jitter (on the set window)
    RSHB_JITTER_MAX_LOCAL,
    //! Min API Rx jitter (on the set window)
    RSHB_JITTER_MIN_LOCAL,
    //! Average API Rx jitter (on the set window)
    RSHB_JITTER_AVG_LOCAL,
    //! Max API Rx jitter (global)
    RSHB_JITTER_MAX_GLOBAL,
    //! Avg API Rx jitter (global)
    RSHB_JITTER_AVG_GLOBAL,
    //! Max CBun Tx difference
    MAX_TX_GLOBAL,
    //! Min CBun Tx difference
    MIN_TX_GLOBAL,
    //! Average CBun Tx difference
    AVG_TX_GLOBAL,
    //! Consecutive packets loss max number (on the set window)
    MAX_LOST_API,
    MAX_LOST_CBUN,
    MAX_LOST_SEQ,
    RSHB_CONS_LOST_COUNTER_MAX_LOCAL, // MAX_LOST_SEQ duplicate
    //! Consecutive packets loss max number (global)
    RSHB_CONS_LOST_COUNTER_MAX_GLOBAL,
    //! Consecutive packets loss min number (on the set window)
    MIN_LOST_API,
    MIN_LOST_CBUN,
    MIN_LOST_SEQ,
    //! Consecutive packets loss average number (on the set window)
    AVG_LOST_API,
    AVG_LOST_CBUN,
    AVG_LOST_SEQ,
    RSHB_CONS_LOST_COUNTER_AVG_LOCAL, // AVG_LOST_SEQ duplicate
    //! Total number of packets lost (on the set window)
    LOST_TOTAL_API,
    LOST_TOTAL_CBUN,
    LOST_TOTAL_SEQ,
    RSHB_LOST_COUNTER_LOCAL, // LOST_TOTAL_SEQ
    //! Number of failures to receive packets (by timeout)
    FAILED_RCV
};

/**
 * @brief KordCore class takes care about communication management.
 *
 */
class KordCore {
public:
    struct CommandStatus {
        uint64_t token{};
        int8_t error_code{};
    };

    /// @brief CommandStatuses structure for handling received CommandStatus
    ///
    struct CommandStatuses {

    public:
        CircularBuffer<CommandStatus> statuses_{MAX_COMMAND_STATUSES};

        /**
         * @brief Add an Status
         *
         * \p a_status: CommandStatus
         */
        void addStatus(const CommandStatus &a_status)
        {
            if (!isTokenPresent(a_status.token)) {
                this->statuses_.push_back(a_status);
            }
        }

        /**
         * @brief Check if the provided token matches some present CommandStatus, and
         * assigned if so
         *
         * \p a_token: uint64_t CommandStatus token
         */
        bool getPresentStatus(const uint64_t &a_token, CommandStatus &a_status) const
        {
            if (auto found = std::find_if(statuses_.begin(),
                                          statuses_.end(),
                                          [a_token](CommandStatus i) { return i.token == a_token; });
                found != std::end(statuses_)) {
                a_status = *found;
                return true;
            }
            else {
                return false;
            }
        }

        /**
         * @brief Reset the list of Statuses
         *
         */
        void reset() { this->statuses_.clear(); }

        /**
         * @brief Check if the Statuses are empty
         *
         * @return true if the list is empty
         *
         */
        [[nodiscard]] bool is_empty() const { return this->statuses_.empty(); }

    private:
        static const unsigned int MAX_COMMAND_STATUSES{100};

        bool isTokenPresent(const uint64_t &a_token)
        {
            return std::find_if(statuses_.begin(), statuses_.end(), [a_token](CommandStatus i) {
                       return i.token == a_token;
                   }) != std::end(statuses_);
        }
    };

    /**
     * @brief Holds status variables related to the robot arm.
     * Relates to hardware, software, channel quality.
     *
     * Content of this structure reflects the content of the
     * Status frame. The dynamic variables related to joint
     * movement and joint hardware are captured here.
     *
     * Transmission statistics are held here as well. The statistics
     * contain maximum, minimum, and average values of the CBun
     * captured command frame jitter, delay, API.
     *
     * Latest Request is echoed in the status.
     *
     */
    struct RobotArmStatus {
        //! Sensor data, measured IOBoard temperature (C°).
        double iob_temperature_;

        //! Model joint position.
        std::array<double, 7UL> positions_;

        //! Model joint speed.
        std::array<double, 7ul> speed_;

        //! Model joint acceleration.
        std::array<double, 7ul> accelerations_;

        //! Model joint torque.
        std::array<double, 7ul> torques_;

        //! Sensed model joint torque.
        std::array<double, 7ul> sens_torques_;

        //! Sensor data, board temperature measurements (C°) for all 7 joints.
        std::array<double, 7ul> jbs_board_temperatures_;

        //! Loads
        std::unordered_map<unsigned int, LoadData> loads_;

        //! Frames
        std::unordered_map<unsigned int, FrameData> frames_;

        //! CPU State
        std::unordered_map<unsigned int, double> cpu_state_;

        //! Sensor data, joint encoder temperature measurements (C°) for all 7 joints.
        std::array<double, 7ul> jbs_joint_encoder_temperatures_;

        //! Sensor data, rotor encoder temperature measurements (C°) for all 7 joints.
        std::array<double, 7ul> jbs_rotor_encoder_temperatures_;

        /**
         * @brief Latest obtained error bits from the joint. It serves diagnostic
         * purposes. Interpretation of the bits is subject to change after every joint
         * firmware release.
         */
        std::array<double, 7ul> error_bits_;

        /**
         * @brief Latest obtained status bits from the joint. It serves diagnostic
         * purposes. Interpretation of the bits is subject to change after every joint
         * firmware release.
         */
        std::array<double, 7ul> status_bits_;

        //! Current TCP of the robot - calculated by forward kinematics from model
        //! joints.
        std::array<double, 6ul> tcp_model_;

        //! Current TCP of the robot - calculated by forward kinematics from sensor
        //! joints.
        std::array<double, 6ul> tcp_sensor_;

        //! CBun statistics parsed from the StatusFrame
        CBunReceivedStatistics cbun_stats_;

        /**
         * @brief Minimum value of tick delay over the reception time period.
         *        Tick delay is a delta of two timepoints \b tick_delay = \b
         * time_frame_processing - \b time_tick.
         */
        int64_t min_tick_delay;

        /**
         * @brief Maximum value of tick delay over the reception time period.
         *        Tick delay is a delta of two timepoints \b tick_delay = \b
         * time_frame_processing - \b time_tick.
         */
        int64_t max_tick_delay;

        /**
         * @brief Average value of tick delay over the reception time period.
         *        Tick delay is a delta of two timepoints \b tick_delay = \b
         * time_frame_processing - \b time_tick.
         */
        int64_t average_tick_delay;

        /**
         * @brief Minumum value of the delay over the reception period.
         *        Delay is a delta of two timepoints \b delay = \b time_current_capture
         * - \b time_previous_capture.
         */
        int64_t min_delay;

        /**
         * @brief Maximum value of the delay over the reception period.
         *        Delay is a delta of two timepoints \b delay = \b time_current_capture
         * - \b time_previous_capture.
         */
        int64_t max_delay;

        /**
         * @brief Average value of the delay over the reception period.
         *        Delay is a delta of two timepoints \b delay = \b time_current_capture
         * - \b time_previous_capture.
         */
        int64_t average_delay;

        //! Number of faulty frames from the start.
        int64_t faulty_frames_start;

        //! Maximum Number of frames which were present in one tick. If more than one,
        //! multiple frames are captured.
        size_t max_frames_in_tick;

        //! Current HW status flags of the responsive controller.
        unsigned int rc_hw_flags_;

        //! Current safety flags of the responsive controller.
        unsigned int rc_safety_flags_;

        //! Current motion flags of the responsive controller.
        unsigned int rc_motion_flags_;

        //! Current Button status flags of the responsive controller.
        unsigned int rc_button_flags_;

        //! SystemAlarmState
        unsigned int rc_system_alarm_state_;

        //! Safety mode set by the user.
        unsigned int rc_safety_mode_;

        //! Current Output
        int64_t rc_digital_output_;

        //! Current Digital Inputs
        int64_t rc_digital_input_;

        uint32_t rc_safe_digital_config;

        //! Current Analog Inputs

        //! Current Analog Outputs

        //! Master speed set by the user in range 0 to 1.0.
        double rc_master_speed_;

        //! Echo of the last captured request to the KORD CBun.
        RequestSystem latest_request_system_;

        //! Holds latest system events, is initialized with empty events at the
        //! beginning.
        std::array<kr2::kord::protocol::SystemEvent, 10ul> system_events_; // This should reserve space for as many as
                                                                           // can fit to byte array

        //! CPU state temperatures
        std::array<double, 5ul> cpu_state_temperatures_;

        //! Server service metadata
        bool service_success_;
        uint8_t service_status_;
        uint8_t service_progress_;
    };

    /**
     * @brief Robot arm command holds control references for movements in joints space,
     * and commands acting on Kassow Robots hardware parts.
     *
     * Movement is joints can be controlled by providing the positions. If the
     * Direct joint controls is enabled, the arm can also use the speed and
     * accelerations to calculate the used torque. Otherwise the speed and acceleration
     * is used to extrapolate the reference in case a command is missed or
     * not delivered in given time frame.
     *
     */
    struct RobotArmCommand {

        enum EType {
            //! Default.
            eInvalid = 0,
            //! Control in joint-pace by providing the target joint configurations.
            eMOVE,
            //! Use direct joint control - references require joint position,
            //! acceleration, and speed.
            eDJC,
            //! Self-motion,
            eMOVEManifold,
            //! Firmware commands - for example brake release/engage/clear commands are
            //! executed with this.
            eFW
        };

        //! Command type as defined above.
        EType command_type = eInvalid;
        //! Sequence number of the robot arm commands.
        unsigned int seq_{};
        //! Reference joint positions.
        std::array<double, 7UL> positions_{};
        //! Reference joint speed - used for extrapolation or as reference in direct
        //! control
        std::array<double, 7UL> speed_{};
        //! Reference joint acceleration - used for extrapolation or as reference in
        //! direct control
        std::array<double, 7UL> accelerations_{};
        //! Reference joint torque.
        std::array<double, 7UL> torque_{};
        //! Reference tracking type
        TrackingType tt_;
        //! Reference tracking value
        double tt_value_{};
        //! Reference blend type
        BlendType bt_;
        //! Reference blend value
        double bt_value_{};
        //! Reference overlay type
        OverlayType ot_;
        //! Reference sync time
        double sync_time_{};

        //! Self-motion joint speed
        double manifold_joint_speed_;

        //! Place holder for firmware commands.
        std::array<kr2::kord::protocol::EJointFirmwareCommand, 7UL> fw_cmds_{};
    };

    /**
     * @brief RobotSetup command holds references for modifying robot parameters.
     *
     */
    struct RobotSetupCommand {
        enum EType {
            //! Set Frame
            SET_FRAME,
            //! Set Load
            SET_LOAD,
            //! Clean alarm flags
            CLEAN_ALARM
        };

        //! Command type selection. One of the following: \b SET_TCP, \b SET_COG.
        unsigned int command_type;

        //! Sequence number of the command.
        unsigned int seq_;

        // Frame
        std::array<double, 6UL> pose_;
        unsigned int frame_id_;
        unsigned int ref_frame_;

        // Load
        double load_mass_;
        unsigned int load_id_;
        std::array<double, 3UL> load_cog_;
        std::array<double, 6UL> load_inertia_;

        // Alarms
        unsigned int alarm_id_;
    };

    /**
     * @brief Status Frame Echo.
     *
     */
    struct StatusFrameEcho {
        // Time Stamp Tx (from CBun)
        unsigned int tx_time_stamp_;

        // Time Stamp Rx (from API)
        unsigned int rx_time_stamp_;
    };

    /**
     * @brief Frame command holds references for the linear movement.
     * The references are acceptable in the positions only in the controller.
     * If there is a velocity and acceleration available, it is used
     * to extrapolate from the last known reference. Such extrapolation is
     * then passed to the controller.
     *
     */
    struct RobotFrameCommand {
        enum {
            //! Move by providing the positional references.
            MOVE_POSE,
            //! Move by providing te velocity references.
            MOVE_VELOCITY,
            //! Move by providing pose, velocity, and acceleration.
            MOVE_POSE_DYN
        };

        //! Command type selects the movement. One of the following: \b MOVE_POSE, \b
        //! MOVE_VELOCITY, \b MOVE_POSE_DYN.
        unsigned int command_type;

        //! Sequence number of the command.
        unsigned int seq_;

        //! Reference position of the TCP.
        std::array<double, 6UL> tcp_target_;
        //! Reference velocity of the TCP.
        std::array<double, 6UL> tcp_target_velocity_;
        //! Reference acceleration of the TCP.
        std::array<double, 6UL> tcp_target_acc_;
        //! Reference to tracking type
        TrackingType tt_;
        //! Reference to tracking value
        double tt_value_;
        //! Reference to blend type
        BlendType bt_;
        //! Reference to blend value
        double bt_value_;
        //! Reference to overlay type
        OverlayType ot_;
        //! Reference to synchronization time
        double sync_time_;

        //! Reference to synchronization value
        long sync_value_;
        //! Reference to period
        double period_;
        //! Reference to timeout
        double timeout_;

        //! Use the command in position control mode - only TCP pose references.
        RobotFrameCommand &asPoseControl();

        //! Use command in the velocity control mode. The velocity of the TCP will be
        //! used as a reference.
        RobotFrameCommand &asVelocityControl();

        //! Pose, velocity and acceleration are all used as a reference for the TCP.
        RobotFrameCommand &asPosVelControl();

        //! Pass the TCP pose reference.
        RobotFrameCommand &withTargetPose(const std::array<double, 6UL> &);

        //! Pass the TCP velocity reference.
        RobotFrameCommand &withTargetVelocity(const std::array<double, 6UL> &);

        //! Pass the TCP acceleration reference.
        RobotFrameCommand &withTargetAcceleration(const std::array<double, 6UL> &);

        //! Set sequence number.
        RobotFrameCommand &setSequenceNumber(unsigned int);

        //! Set Tracking Type
        RobotFrameCommand &withTrackingType(const TrackingType &);

        //! Set Tracking Value
        RobotFrameCommand &withTrackingValue(const double &);

        //! Set Blend Type
        RobotFrameCommand &withBlendType(const BlendType &);

        //! Set Blend Value
        RobotFrameCommand &withBlendValue(const double &);

        //! Set Overlay Type
        RobotFrameCommand &withOverlayType(const OverlayType &);

        //! Set Sync Time
        RobotFrameCommand &withSyncTime(const double &);

        //! Set Sync Value
        RobotFrameCommand &withSyncValue(const long &);

        //! Set Period
        RobotFrameCommand &withPeriod(const double &);

        //! Set Period
        RobotFrameCommand &withTimeout(const double &);
    };

public:
    /**
     * @brief Create a new KordCore object.
     * @param hostname IP address of host where the CBun KORD runs.
     * @param port Port of where the CBun KORD runs and is listening.
     * @param conn Connection type. Currently only UDP type is supported.
     *
     * KordCore is communication interface between the KORD-API and the
     * KORD CBun. It is based on sockets and allows creation of a UDP server
     * and client based on the \p conn param.
     *
     */
    explicit KordCore(const std::string& hostname, unsigned int port, unsigned int session_id, connection conn);

    //! @brief Calls \p disconnect first and then destroys the Kord Core object.
    ~KordCore();

    //! @brief Initializes the internal tick timer and creates the connection inteface.
    bool connect(const char *device = NULL);

    /**
     * @brief Terminates the connection interface. On success the underlying socket is
     * closed.
     * @return true After the socket is successfully closed.
     * @return false If the cocket is not closed successsfully.
     */
    bool disconnect();

    /** @brief Transmits a request to the remote host to start dissipation of
     * the status frame repeatedly. The remote host response will be
     * synchronized with the RC tick start on the remote host. By default the sync
     * completes after all rotating items in the status frame are transferred. To
     * complete the rotation can take upto 50 ms based on items rotated.
     *
     * This command should be used only once to initiate the communication. Any
     * following sync should be done by waitSync.
     *
     * @param flags flags to pass parameters to do a specific sync, first bit set for
     * full rotation
     * @return true Heartbeat captured.
     * @return false Heartbeat not captured.
     */
    bool syncRC(long flags = F_SYNC_FULL_ROTATION);

    /**
     * @brief Blocks until the heart beat frame is captured or the timeout has elapsed.
     * It stores captured data to appropriate structure (currently to a status
     * frame). If the capture was successful a receiving time stamp is recorded.
     * Moreover the capture statistics are updated with the timestamp.
     * In case of a failure, the failure to read count is increased. Failed count
     * can be increased if the time runs out or the capture ends with failure.
     *
     * @param timeout_s For how long to wait for heartbeat capture before timing out. In
     * microseconds.
     * @param flags flags to pass parameters to do a specific sync, first bit set for
     * full rotation
     * @return true if heartbeat successfully captured
     * @return false not captured within timeout
     */
    bool waitSync(std::chrono::microseconds timeout_s = std::chrono::microseconds(8000), long flags = F_NONE);

    /**
     * @brief Sleep until the next time slice. There are 4ms updates.
     * After the function entry the current tick is determined
     * based on the heart beat reception timestamp obtained from .
     * Synchronize with RC first by calling syncRC.
     */
    void spin();

    //! @brief Returns the latest update tick.
    struct timespec ctlrcUpdateTS() const;

    //! @brief Forced set of the update tick.
    void setCtlrcUpdateTS(struct timespec new_ts);

    /**
     * @brief Send Arm Status request.
     *
     * @return unsigned int Number of bytes transmitted.
     */
    unsigned int requestArmStatus();

    //! Updates the status of the latest request captured by the remote controller.
    void updateRecentArmStatus(RobotArmStatus &arm_status) const;

    //! Returns the status of the latest request captured by the remote controller.
    void getRecentArmStatus(RobotArmStatus &arm_status) const;

    //! Updates the status of the latest command from robot status captured by the
    //! remote controller.
    void updateRecentCommandStatus(CommandStatuses &a_command_statuses) const;

    //! Returns the status of the latest command from robot status captured by the
    //! remote controller.
    void getRecentCommandStatus(CommandStatuses &a_command_statuses) const;

    //! Fills the arm command and Sends it to the robot arm.
    unsigned int sendCommand(RobotArmCommand, int64_t &);

    //! Fills the arm command and Sends it to the robot arm.
    unsigned int sendCommand(RobotArmCommand);

    //! Fills the frame command and sends it to the robot arm,
    unsigned int sendCommand(RobotFrameCommand);

    //! Fills the setup command and sends it to the robot arm,
    unsigned int sendCommand(RobotSetupCommand, int64_t &);

    //! Fills the request and sends it to the robot arm.
    unsigned int sendCommand(const Request &);

    //! Fills the echo and sends it to the robot arm.
    unsigned int sendCommand(const StatusFrameEcho &);

    /**
     * @brief Print statistics of capturing the heart beat frame.
     *
     * Statistics of the KORD-API duration. It is measured within
     * the waitSync call. The timestamp is recorded if the heart
     * beat frame (aka status frame) is captured. In case the frame
     * is not received within a specified time slot.
     *
     */
    void printStats(const CBunReceivedStatistics &);

    /**
     * @brief Set window length in elements for the jitter capturing.
     *
     * The max, min, mean jitter are calculated on the recent
     * interval set with this method.
     * By default it is 1 second i.e 250 recent values
     *
     */
    void setStatisticsWindow(int);

    /**
     * @brief Get API statistics
     *
     * In case of time, the returned values are in nanoseconds.
     *
     */
    int64_t getAPIStatistics(EAPIStatistics);

    //! Get Status Frame TxStamp
    [[nodiscard]] int64_t getTxStamp() const;

    //! Get iterative Frame Id from Status
    [[nodiscard]] uint8_t getIterativeFrameId() const;

    //! Get iterative Load Id from Status
    [[nodiscard]] uint8_t getIterativeLoadId() const;

    //! Get iterative CPU State Id from Status
    [[nodiscard]] uint8_t getIterativeCPUStateId() const;

private:
    //! Take the joint space references and translate them to the content item.
    void makeCommandMoveJ(kr2::kord::protocol::ContentItem *, const KordCore::RobotArmCommand &in_jcmd);

    //! Take linear references and store them to the content item.
    void makeCommandMoveL(kr2::kord::protocol::ContentItem *, const KordCore::RobotFrameCommand &in_lcmd);

    //! Take joint speed and create a content item for self-motion.
    void makeCommandMoveManifold(kr2::kord::protocol::ContentItem *, const KordCore::RobotArmCommand &a_cmd);

    //! Take input references and create the content item for velocity control.
    void makeCommandMoveVelocityL(kr2::kord::protocol::ContentItem *,
                                  const std::array<double, 6UL> &in_lcmd,
                                  unsigned int seq_num,
                                  long sync,
                                  double period,
                                  double timeout);

    //! Take input references and create the content item for control by position,
    //! velocity and acceleration.
    void makeCommandMoveDynL(kr2::kord::protocol::ContentItem *,
                             const std::array<double, 6UL> &in_lcmd,
                             unsigned int seq_num);

    //! Take input references and create the content item for control by joint position,
    //! joint velocity, joint acceleration and torque.
    void makeCommandMoveD(kr2::kord::protocol::ContentItem *,
                          const std::array<double, 7UL> &a_j_dcmd,
                          const std::array<double, 7UL> &a_jd_dcmd,
                          const std::array<double, 7UL> &a_jdd_dcmd,
                          const std::array<double, 7UL> &a_torque_dcmd,
                          unsigned int a_seq_num);

    //! Make the initial arm status request to initiate the dissipation of the
    //! heartbeat.
    void makeArmStatusRequest(kr2::kord::protocol::ContentItem *);

    //! Convert the joint firmware request to the appropriate content item.
    void makeCommandFirmware(kr2::kord::protocol::ContentItem *,
                             const std::array<kr2::kord::protocol::EJointFirmwareCommand, 7UL> &in_fw_cmd,
                             unsigned int seq_num,
                             int64_t &a_timeStamp);

    //! Take input reference and create the content item for set Frame Command.
    void makeCommandSetFrame(kr2::kord::protocol::ContentItem *a_content_item,
                             const KordCore::RobotSetupCommand &a_lcmd,
                             int64_t &a_timeStamp);

    //! Take input reference and create the content item for set Load Command.
    void makeCommandSetLoad(kr2::kord::protocol::ContentItem *a_content_item,
                            const KordCore::RobotSetupCommand &a_lcmd,
                            int64_t &a_timeStamp);

    //! Take input reference and create the content item for set Clean Alarm Command.
    void makeCommandCleanAlarm(kr2::kord::protocol::ContentItem *a_content_item,
                               const KordCore::RobotSetupCommand &a_lcmd,
                               int64_t &a_timeStamp);

    //! Take a request and fill in the content frame with a request content item.
    int setRequestContentItem(kr2::kord::protocol::ContentItem *, const Request &, unsigned short seq);

    //! Take an echo and fill in the content frame with an echo content item.
    int setStatusEchoContentItem(kr2::kord::protocol::ContentItem *a_content, const KordCore::StatusFrameEcho &a_echo);

    //    void makeRequestServiceParameters(kr2::kord::protocol::ServiceParameters)

    //! Dispatch the frame to the remote controller.
    unsigned int sendFrame(const kr2::kord::protocol::KORDFrame *);

    /**
     * Waits for a valid frame within the timeout period.
     *
     * @param a_timeout_us The timeout duration in microseconds.
     * @param time_out_counter Counter to track the number of timeout attempts.
     * @return True if a valid frame is received within the timeout, false otherwise.
     */
    bool waitForValidFrame(std::chrono::microseconds a_timeout_us, int &time_out_counter);

    /**
     * Checks if the elapsed time has exceeded the specified timeout.
     *
     * @param start The start time point.
     * @param timeout The timeout duration.
     * @return True if the elapsed time has exceeded the timeout, false otherwise.
     */
    static bool hasTimedOut(const std::chrono::steady_clock::time_point &start,
                            const std::chrono::microseconds &timeout);

    /**
     * Parses the frame from the received payload.
     */
    void parseFrame();

    /**
     * Updates the frame, load, and CPU state IDs, handling the first and second loops.
     *
     * @param got_first_id Reference to a flag indicating if the first ID has been obtained.
     * @param second_loop Reference to a flag indicating if the second loop has started.
     * @param first_frame_id Reference to the first frame ID.
     * @param now_frame_id Reference to the current frame ID.
     * @param first_load_id Reference to the first load ID.
     * @param now_load_id Reference to the current load ID.
     * @param first_cpu_state_id Reference to the first CPU state ID.
     * @param now_cpu_state_id Reference to the current CPU state ID.
     * @return True if IDs are updated, false otherwise.
     */
    bool updateIds(bool &got_first_id,
                   bool &second_loop,
                   int &first_frame_id,
                   int &now_frame_id,
                   int &first_load_id,
                   int &now_load_id,
                   int &first_cpu_state_id,
                   int &now_cpu_state_id) const;

    /**
     * Updates recent statuses for arm and command.
     */
    void updateRecentStatuses();

    /**
     * Checks if the data is within the recent interval.
     *
     * @param recent_interval The interval considered as recent.
     * @return True if the data is recent, false otherwise.
     */
    [[nodiscard]] bool isRecent(const std::chrono::nanoseconds &recent_interval) const;

    /**
     * Determines if the main loop should continue based on the conditions.
     *
     * @param not_recent Flag indicating if the data is not recent.
     * @param full_cycle Flag indicating if a full cycle rotation is required.
     * @param second_loop Flag indicating if the second loop has started.
     * @param first_frame_id The first frame ID.
     * @param now_frame_id The current frame ID.
     * @param first_load_id The first load ID.
     * @param now_load_id The current load ID.
     * @param first_cpu_state_id The first CPU state ID.
     * @param now_cpu_state_id The current CPU state ID.
     * @return True if the loop should continue, false otherwise.
     */
    static bool shouldContinueLoop(bool not_recent,
                                   bool full_cycle,
                                   bool second_loop,
                                   int first_frame_id,
                                   int now_frame_id,
                                   int first_load_id,
                                   int now_load_id,
                                   int first_cpu_state_id,
                                   int now_cpu_state_id);

private:
    std::shared_ptr<Connection_interface> conn_container_;

    enum connection conn_type_;

    asio::io_service io_service_;

    std::string hostname_;
    unsigned int port_;
    unsigned int session_id_;

    // unix sockets
    int sock_ = 0;
    struct sockaddr_in server_{}, from_{};

    class Internals;
    Internals *his_;

    // stats
    unsigned int packets_cnt_ = 0;
    std::chrono::time_point<std::chrono::steady_clock> start_;

    // sync members
    kr2::utils::StampsLog ts_record_;
    uint16_t sequence_number_{};
    struct timespec ctlrc_update_ts_ {
        0, 0
    };

    utils::Stats stats;
};

} // namespace kr2::kord

#endif // KR2_KORD_CORE_H
