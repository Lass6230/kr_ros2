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

#ifndef KR2_KORD_CONTROL_INTERFACE_H
#define KR2_KORD_CONTROL_INTERFACE_H

#include <memory>
#include <vector>

#include "kord/api/api_request.h"
#include <kord/api/kord.h>
#include <map>

namespace kr2::kord {

typedef int64_t token_t;

class KordCore;

/**
 * @brief Control interface to move the robot arm.
 * The arm can be controlled in both joint space
 * and linear space.
 *
 * Alternative means of control will be available in future
 * versions.
 *
 */
class ControlInterface {
public:
    /**
     * @brief Construct a new Control Interface object
     *
     * @param kord Instance of the \b KordCore class to provide communication interface.
     */
    explicit ControlInterface(std::shared_ptr<kord::KordCore> kord);

    /**
     * @brief Destroy the Control Interface object
     *
     */
    ~ControlInterface();

    /**
     * @brief Robot move commands in joint space providing joint references in radians.
     *
     * @param q Reference joint configuration to assume at the end of defined time.
     * @param tt ...
     * @param tt_value_ How long the movement should take. Currently omitted ans 4ms is
     * assumed.
     * @param bt ...
     * @param bt_value_ The blend setting of the target point. Start blending at the
     * defined time.
     * @param ot ...
     * @param sync_time ...
     * @return true command is successfully sent
     * @return false command is not sent
     */
    bool moveJ(const std::array<double, 7UL> &q,
               const TrackingType &tt,
               const double &tt_value,
               const BlendType &bt,
               const double &bt_value,
               const OverlayType &ot,
               const double &sync_time = std::numeric_limits<double>::infinity());

    /**
     * @brief Robot move commands in linear space providing pose references in meters.
     *
     * @param p Reference TCP pose [x, y, z, r, p, y] to assume at the end of defined
     * time.
     * @param tt ...
     * @param tt_value_ How long the movement should take. Currently omitted ans 4ms is
     * assumed.
     * @param bt ...
     * @param bt_value_ The blend setting of the target point. Start blending at the
     * defined time.
     * @param ot ...
     * @param sync_time ...
     * @return true command is successfully sent
     * @return false command is not sent
     */
    bool moveL(const std::array<double, 6UL> &p,
               const TrackingType &tt,
               const double &tt_value_,
               const BlendType &bt,
               const double &bt_value_,
               const OverlayType &ot,
               const double &sync_time = std::numeric_limits<double>::infinity());

    /**
     * @brief Robot move commands in linear space providing velocity references in
     * meters.
     *
     * @param v Reference TCP velocity [x, y, z, r, p, y] to assume at the end of
     * defined time.
     * @param sync_value Sync value (?meaning?)
     * @param period How long the movement should take.
     * @param timeout The time between the end of the movement and full stop (zero
     * velocity)
     * @return true command is successfully sent
     * @return false command is not sent
     */
    bool moveV(const std::array<double, 6UL> &v, long sync_value, double period, double timeout);

    /**
     * @brief Use self-motion to move robot keeping its head position.
     *
     * @param manifold_joint_speed Speed of joints to use when using self-motion.
     * @return false if command is not sent.
     */
    bool moveManifold(const double manifold_joint_speed);

    /**
     * @brief Set new Frame coordinates
     *
     * @param frame_id EFrameID.
     * @param pose Rotation and Position coordinates [x, y, z, r, p, y]
     * @param ref_frame Position reference, World Frame by default.
     * @param token Token of the sent command, populated token can then be used by
     * getCommandStatus(token)
     *
     * @return true if command is successfully sent
     * @return false when command is not sent
     */
    bool setFrame(EFrameID frame_id, std::array<double, 6> pose, EFrameValue ref_frame, token_t &token);

    /**
     * @brief Set new Frame coordinates
     *
     * @param frame_id EFrameID.
     * @param pose Rotation and Position coordinates [x, y, z, r, p, y]
     * @param ref_frame Position reference, World Frame by default.
     *
     * @return true if command is successfully sent
     * @return false when command is not sent
     */
    bool setFrame(EFrameID frame_id, std::array<double, 6> pose, EFrameValue ref_frame);

    /**
     * @brief Set specified Load parameters
     *
     * @param load_id ELoadID.
     * @param mass Mass
     * @param cog Center of Gravity.
     * @param inertia Inertia
     * @param token Token of the sent command, populated token can then be used by
     * getCommandStatus(token)
     *
     * @return true if command is successfully sent
     * @return false when command is not sent
     */
    bool setLoad(ELoadID load_id, double mass, std::array<double, 3> cog, std::array<double, 6> inertia, token_t &token);

    /**
     * @brief Set specified Load parameters
     *
     * @param load_id ELoadID.
     * @param mass Mass
     * @param cog Center of Gravity.
     * @param inertia Inertia
     *
     * @return true if command is successfully sent
     * @return false when command is not sent
     */
    bool setLoad(ELoadID load_id, double mass, std::array<double, 3> cog, std::array<double, 6> inertia);

    //
    // Miscellaneous/Experimental control
    //

    //! Attempt to engage brakes of listed joints ranging from 1 to 7.
    //  populated token can then be used by getCommandStatus(token)
    bool engageBrakes(const std::vector<int> &b, token_t &token);

    //! Attempt to engage brakes of listed joints ranging from 1 to 7.
    bool engageBrakes(const std::vector<int> &b);

    //! Attempt to disengage brakes of listed joints ranging from 1 to 7.
    //  populated token can then be used by getCommandStatus(token)
    bool disengageBrakes(const std::vector<int> &b, token_t &token);

    //! Attempt to disengage brakes of listed joints ranging from 1 to 7.
    bool disengageBrakes(const std::vector<int> &b);

    /**
     * @brief [Warning: Experimental Feature] Use direct joint control with torque. The
     * Commands are directly translated into the frames and send to the robot arm.
     *
     * @param q Target joint position.
     * @param qd Target joint speed.
     * @param qdd Target joint acceleration.
     * @param torque Target joint torque.
     *
     * @return True in case the frame was created, added to tx batch and sent. Otherwise
     * it returns false
     */
    bool directJControl(const std::array<double, 7UL> &q,
                        const std::array<double, 7UL> &qd,
                        const std::array<double, 7UL> &qdd);

    /**
     * @brief [Warning: Experimental Feature] Use direct joint control with torque. The
     * Commands are directly translated into the frames and send to the robot arm.
     *
     * @param q Target joint position.
     * @param qd Target joint speed.
     * @param qdd Target joint acceleration.
     * @param torque Target joint torque.
     *
     * @return True in case the frame was created, added to tx batch and sent. Otherwise
     * it returns false
     */
    bool directJControl(const std::array<double, 7UL> &q,
                        const std::array<double, 7UL> &qd,
                        const std::array<double, 7UL> &qdd,
                        const std::array<double, 7UL> &torque);

    enum EClearRequest {
        //! Tries to clear HALT state
        CLEAR_HALT = 0,
        //! Tries to unsuspend robot
        UNSUSPEND = 1,
        //! Tries to reinitialize robot if initialization is stopped
        CONTINUE_INIT = 2,
        //! Resumes paused movement
        RESUME = 3,
        //! CBunEvent Error
        CBUN_EVENT = 4
    };

    /**
     * @brief Clear robot alarm states.
     *
     * @param alarm_request Alarm cleaning request
     * @return token in case of success, token = 0 otherwise
     */
    int64_t clearAlarmRequest(EClearRequest alarm_request);

    /**
     * @brief Send a new API request to the remote target.
     *
     * @return true if successfully sent
     * @return false if failed to sent
     */
    bool transmitRequest(const Request &);

private:
    struct Internals;
    std::shared_ptr<Internals> his_;
};

} // namespace kr2::kord

/*

 Sample program:


    robot_ctl = kord::ControlInterface("192.168.38.1")

    if (!robot_clt.connect()) {
        std::err << "Connecting KR failed";
        return 1;
    }

    std::vector<double> q;
    robot_ctl.moveJ([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
 kord::MoveJParams().withTime(0.008).withBlendTime(0.004).asynchronous());

    robot_ctl.moveJ(q,
 kord::MoveJParams().withSpeed(1.0).withBlendMaxAcceleration(0.5).synchronous());

 */

#endif // KR2_KORD_CONTROL_INTERFACE_H