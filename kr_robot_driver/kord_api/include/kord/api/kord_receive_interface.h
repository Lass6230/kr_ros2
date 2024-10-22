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

#ifndef KR2_KORD_RECEIVER_INTERFACE_H
#define KR2_KORD_RECEIVER_INTERFACE_H

#include <kord/api/kord.h>

#include <bitset>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <kord/utils/worker.h>
// #include <boost/interprocess/sync/scoped_lock.hpp>
// #include <boost/interprocess/sync/interprocess_mutex.hpp>

namespace kr2::kord {

/**
 * @brief Receiver interface that provides captured data from the remote controller
 *
 * It serves for the purposes of monitoring and evaluating the state of teh system.
 * Part of the information provided are also statistics related to the transmission
 * channel.
 *
 */
class ReceiverInterface {
public:
    // mutable boost::interprocess::interprocess_mutex comm_mutex_;

    mutable std::mutex comm_mutex_;

    /**
     * @brief Construct a new Receiver Interface object
     *
     * @param kord Instance of the \b KordCore class to provide communication interface.
     */
    explicit ReceiverInterface(std::shared_ptr<kord::KordCore> kord);

    /**
     * @brief Destroy the Receiver Interface object
     *
     */
    ~ReceiverInterface();

    /**
     * @brief Wait for the capture of the heartbeat from the remote and update
     * internal capture state.
     *
     * @return true if synced successfully
     * @return false if not synced and timed out
     */
    bool sync();

    /**
     * @brief Copy the data from the capture buffer to the state
     * structures where it can be retrieved from.
     *
     */
    void fetchData();

    /**
     * @brief Wait for Status and FetchData()
     *
     */
    bool fetchStatus();

    bool sendStatusEcho(int64_t);

    //
    // Joint values
    //
    /**
     * @brief Selector to which joint values to retrieve.
     *
     */
    enum class EJointValue {
        //! Target reference joint configuration in radians.
        T_REFERENCE_Q = 1,
        //! Reference joint speed in rad/sec.
        T_REFERENCE_QD = 2,
        //! Reference joint accelerations in rad/sec^2.
        T_REFERENCE_QDD = 3,
        //! Reference torqe [Nm]
        T_REFERENCE_TRQ = 4,
        //! Actual joint configurations as provided by sensors.
        S_ACTUAL_Q = 5,
        //! Actual joint speeds as provided by sensors.
        S_ACTUAL_QD = 6,
        //! Actual joint accelerations as provided by sensors.
        S_ACTUAL_QDD = 7,
        //! Torque estimation
        S_ACTUAL_TRQ = 8,
        //! Joint Board temperature sensors.
        S_TEMP_BOARD = 9,
        //! Joint Encoder temperature sensors.
        S_TEMP_JOINT_ENCODER = 10,
        //! Rotor Encoder temperature sensors.
        S_TEMP_ROTOR_ENCODER = 11,
        //! Joint Sensed Torques
        S_SENSED_TRQ = 12
    };

    /**
     * @brief Get the Joint specified values.
     *
     * @return std::array<double, 7UL> holding the values specified by the \b
     * EJointValue.
     */
    std::array<double, 7UL> getJoint(EJointValue);

    /**
     * @brief Get frame values as specified by the \b eFrameId and \b eFrameValue
     *
     * @return std::vector<std::variant<double, int>> holds the items specified by
     * argument, coordinates in the form [x (m), y (m), z (m), r (rads), p (rads), y
     * (rads)].
     */
    std::vector<std::variant<double, int>> getFrame(EFrameID eFrameId, EFrameValue eFrameValue);

    /**
     * @brief Get status of the command provided using token.
     * Please refer to ``ECommandStatusFlags`` to find out possible return values.
     *
     * @return ECommandStatusFlags value represented by int8_t.
     */

    int8_t getCommandStatus(token_t);

    /**
     * @brief Get load values as specified by the \b eLoadId and \b eLoadValue
     *
     * @return std::vector<std::variant<double, int>> holds the items specified by
     * argument (for CoG: in meters [x, y, z]; for Mass: in kg [m]; for Inertia: in kg *
     * m^2 [XX, XY, XZ, YY, YZ, ZZ].
     */
    std::vector<std::variant<double, int>> getLoad(ELoadID eLoadId, ELoadValue eLoadValue);

    /**
     * @brief Get  the actual TCP coordinates system expressed within the World Frame
     * refernce..
     *
     * @return std::array<double, 6UL> holds coordinates in the form [x (m), y (m), z
     * (m), r (rads), p (rads), y (rads)].
     */
    std::array<double, 6UL> getTCP() const;

    //
    // State information
    //

    /**
     * @brief Get the Robot HW status flags.
     *
     * @return unsigned int HW flags, see \link kr2::kord::protocol::EHWFlags \endlink.
     */
    unsigned int getHWFlags() const;

    /**
     * @brief Get the Robot Button status flags.
     *
     * @return unsigned int Button flags, see \link kr2::kord::protocol::EButtonFlags
     * \endlink.
     */
    unsigned int getButtonFlags() const;

    /**
     * @brief Selector to which CPU State values to retrieve.
     *
     */
    enum ECPUStateValue {
        //! Package ID 0 temperature (average)
        PACKAGE_ID0_TEMP = 0,
        //! Core 0 temperature
        CORE_0_TEMP = 1,
        //! Core 1 temperature
        CORE_1_TEMP = 2,
        //! Core 2 temperature
        CORE_2_TEMP = 3,
        //! Core 3 temperature
        CORE_3_TEMP = 4,
    };

    /**
     * @brief Get cpu values as specified by the \b ECPUStateValue
     *
     * For CPU temperatures, if **lm-sensors version 1:3.4.0-4** is not installed to the
     * controller, all returned values are 0.
     *
     * @return double holds the item specified by argument.
     */
    double getCPUState(unsigned int);

    /**
     * @brief Get the Robot Safety Flags.
     *
     * @return unsigned int safety flags.
     */
    unsigned int getRobotSafetyFlags() const;

    /**
     * @brief Get the Master Speed from the system.
     *
     * @return double ranging from 0 to 1.0.
     */
    double getMasterSpeed() const;

    /**
     * @brief Get the IOBoard Temperature from the system.
     *
     * @return double
     */
    double getIOBoardTemperature() const;

    /**
     * @brief Get the Safety Mode of the robot.
     *
     * @return int representing the safety mode.
     */
    int getSafetyMode() const;

    /**
     * @brief Get the Motion Flags from the robots.
     *
     * @return unsigned int motion flags.
     */
    unsigned int getMotionFlags() const;

    /**
     * @brief Returns encoded alarm state. In case the system doesn't contain any
     * present error or alarm state, the function returns 0.
     *
     * The alarms and errors are encoded in the following way:
     *
     * SAFETY EVENT (read from CAT):
     *
     *      31             23     20         15            7       3     0
     *       +---------------------+-----------------------+-------+-----+
     *       |      reserved       |   Condition ID        | CNTXT | CAT |
     *       +---------------------+-----------------------+-------------+
     *
     * CAT (category), describes the category of the contained alarm state:
     *
     *   1 = SafetyEvent  \n
     *   2 = SoftStopEvent \n
     *   3 = HwStat \n
     *
     * CNTXT (context), collateral error states also present in the system:
     *
     *   0x01 = ESTOP \n
     *   0x02 = PSTOP \n
     *   0x04 = SSTOP \n
     *   0x08 = SYSERR \n
     *
     * For more details regarding the *conditions coding*, please check the
     * [Appendix](../api/appendix.html) .
     *
     * @return encoded error state or 0 if no alarms/errors are present.
     */
    uint32_t systemAlarmState() const;

    //
    //
    // Statistics values
    //
    /**
     * @brief Selector to which Statistics values to retrieve.
     *
     */
    enum class EStatsValue {
        //! number of times the recvfrom returned 0.
        FAIL_TO_READ_EMPTY = 0,
        //! number of times the recvfrom returned <0.
        FAIL_TO_READ_ERROR = 1,

        //! Deviation from expected synchronicity of the commands. - micros
        CMD_JITTER_MAX_LOCAL = 2,
        CMD_JITTER_AVG_LOCAL = 3,
        CMD_JITTER_MAX_GLOBAL = 4,

        //! Time elapsed between tick start and capture of the command. - micros
        ROUND_TRIP_TIME_MAX_LOCAL = 5,
        ROUND_TRIP_TIME_AVG_LOCAL = 6,
        ROUND_TRIP_TIME_MAX_GLOBAL = 7,

        //! Number of lost commands
        CMD_LOST_COUNTER_LOCAL = 8, // <- SEQ
        CMD_LOST_COUNTER_GLOBAL = 9,

        CMD_LOST_COUNTER_LOCAL_TIMESTAMP = 10,
        CMD_LOST_COUNTER_GLOBAL_TIMESTAMP = 11,

        //! Time deviation of the control thread wake up time from the scheduled time. -
        //! micros
        SYS_JITTER_MAX_LOCAL = 12,
        SYS_JITTER_AVG_LOCAL = 13,
        SYS_JITTER_MAX_GLOBAL = 14,
    };

    int64_t getStatistics(EStatsValue);

    CBunReceivedStatistics getStatisticsStructure();

    /**
     * @brief Get the latest digital IO status.
     *
     * @return int64_t bit representation
     */
    int64_t getDigitalOutput();

    /**
     * @brief Get the latest digital IO status.
     *
     * @return int64_t bit representation
     */
    int64_t getDigitalInput();

    /**
     * @brief Get the latest safe digital IO config.
     *
     * @return uint32_t where each 8 bits represent the config value of i-th safe output.
     */
    uint32_t getSafeDigitalOutputConfig();

    /**
     * @brief Get the formatted bit status
     *
     * Example: "0000 0000 ... 0000".
     *
     * @return std::string
     */
    std::string getFormattedInputBits();

    std::string getFormattedOutputBits();

    //! Reports the maximum of frames that were captured in one time slot.
    size_t getMaxFramesInTick();

    //! Returns counter of how many times receiver was unable to read data from the
    //! socket.
    int64_t getFaultyTickFrame();

    /**
     * @brief Get the Latest Request data
     *
     * @return Request reporting about the last one sent to the remote.
     */
    Request getLatestRequest();

    /**
     * @brief Get the latest System Events. Only event timestamp, group and ID are
     * reported. The System Events are cleared every time ``waitSync`` is successfully called.
     * If the connection times out, the latest known System Events are returned.
     *
     * @return std::vector<kr2::kord::SystemEvent>
     */
    std::vector<kr2::kord::protocol::SystemEvent> getSystemEvents();

    /**
     * @brief Clear the SystemEvent buffer.
     *
     */
    void clearSystemEventsBuffer();

    /**
     * @brief Get a flag containing information about server status response
     *
     * @code
     * uint64_t: | ... | service_status_ | service_progress | service_success |
     *           64    17     uint8_t     9     uint8_t      1       bool      0
     * @endcode
     * @return uint64_t with encoded values.
     */
    uint64_t getServerResponseFlag();

private:
    struct Internals;
    std::shared_ptr<Internals> his_;

    class HeartBeatMonitor : public Worker {
    public:
        static HeartBeatMonitor &getInstance(kr2::kord::ReceiverInterface *a_rcv_interface_)
        {
            static HeartBeatMonitor instance;
            if (!instance.is_interface_set()) {
                instance.set_interface(a_rcv_interface_);
            }
            return instance;
        }

        /// @brief Get the latest status of the job
        [[nodiscard]] bool get_status() const { return this->status_; }

        /// @brief Reset the Reader
        void reset() { this->status_ = true; }

        /// @brief Stop cycling
        void stop() { keep_running_.store(false); }

        bool is_interface_set() { return rcv_interface_ != nullptr; }

        void set_interface(kr2::kord::ReceiverInterface *a_rcv_interface_) { rcv_interface_ = a_rcv_interface_; }

    private:
        std::atomic<bool> keep_running_{};

        kr2::kord::ReceiverInterface *rcv_interface_ = nullptr;

        void fetchStats()
        {
            this->status_ = rcv_interface_->fetchStatus();
            if (!this->status_) {
                // std::cout << "fetched status with " << this->status_ << '\n';
            }
        }

        /// implement run for your specific Worker
        void run() override
        {
            keep_running_.store(true);
            while (keep_running_.load()) {
                fetchStats();
                // std::this_thread::sleep_for(std::chrono::milliseconds(3)); // every
                // heartbeat
            }
            this->finished();
        }

        bool status_ = true;
    };
    ReceiverInterface::HeartBeatMonitor &monitor_instance = ReceiverInterface::HeartBeatMonitor::getInstance(this);
};

} // namespace kr2::kord

#endif // KR2_KORD_RECEIVER_INTERFACE_H
