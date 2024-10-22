/*////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2022 by Kassow Robots, ApS
//
// The information contained herein is confidential, proprietary to Kassow Robots,
// ApS, and considered a trade secret as defined in section 263 and section 264
// under the Danish Criminal Code. Use of this information by anyone
// other than authorized employees of Kassow Robots, ApS is granted only under a
// written non-disclosure agreement, expressly prescribing the scope and
// manner of such use.
//
// Authors: Ondrej Bruna  obr@kassowrobots.com
//
/////////////////////////////////////////////////////////////////////////////*/

namespace kr2::kord {

/**
 * @brief Holds status variables related to the robot arm.
 * Relates to hardware, software, channel quality.
 *
 * Content of this structure reflects the content of the
 * Status frame. The dynamic variables related to joint
 * movement and joint hardware are captured here.
 *
 * Transmission statistics are held here are well. The statistics
 * contain maximum, minimum, and average values of the CBun
 * captured command frame jitter, delay, API.
 *
 */
struct RobotArmStatus {
    std::array<double, 7UL> positions_;
    std::array<double, 7ul> speed_;
    std::array<double, 7ul> accelerations_;
    std::array<double, 7ul> torques_;
    std::array<double, 7ul> temperatures_;
    std::array<double, 7ul> error_bits_;
    std::array<double, 7ul> status_bits_;

    std::array<double, 6ul> tcp_model_;
    std::array<double, 6ul> tcp_sensor_;

    uint32_t fail_to_read_empty;
    uint32_t fail_to_read_error;
    int64_t min_tick_delay;
    int64_t max_tick_delay;
    int64_t average_tick_delay;

    int64_t min_delay;
    int64_t max_delay;
    int64_t average_delay;

    int64_t faulty_frames_start;
    size_t max_frames_in_tick;

    unsigned int rc_safety_flags_;
    unsigned int rc_motion_flags_;
};

struct RobotArmCommand {

    enum EType { eInvalid = 0, eMOVE, eDJC, eFW };

    EType command_type = eInvalid;
    unsigned int seq_;
    std::array<double, 7UL> positions_;
    std::array<double, 7UL> speed_;
    std::array<double, 7UL> accelerations_;

    std::array<EJointFirmwareCommand, 7UL> fw_cmds_;
};

struct RobotFrameCommand {
    enum { MOVE_POSE, MOVE_VELOCITY, MOVE_POSE_DYN };

    unsigned int command_type;
    unsigned int seq_;
    std::array<double, 6UL> tcp_target_;
    std::array<double, 6UL> tcp_target_velocity_;
    std::array<double, 6UL> tcp_target_acc_;

    RobotFrameCommand &asPoseControl();
    RobotFrameCommand &asVelocityControl();
    RobotFrameCommand &asPosVelControl();
    RobotFrameCommand &withTargetPose(const std::array<double, 6UL> &);
    RobotFrameCommand &withTargetVelocity(const std::array<double, 6UL> &);
    RobotFrameCommand &withTargetAcceleration(const std::array<double, 6UL> &);
    RobotFrameCommand &setSequenceNumber(unsigned int);
};

} // namespace kr2::kord
