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

#ifndef KR2_KORD_UTILS_H
#define KR2_KORD_UTILS_H

#include <algorithm>
#include <arpa/inet.h>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <math.h>
#include <queue>

#include <getopt.h>

#include <array>
#include <iomanip>

#include "kr2/kord/system/SystemAlarm.h"
#include "kr2/kord/system/SystemEvent.h"

namespace kr2::utils {

struct StampsLog {
    StampsLog();

    void reset();

    std::string header();

    std::string asString();

    int64_t t0_;
    int64_t t1_;
    int64_t t2_;
};

class StatGeneral {
public:
    StatGeneral();

    void reset();

    void update(long a_uval);

    void printStat(std::ostream &a_stream) const;

private:
    long avg_ = 0;
    long max_ = 0;
    long min_ = 1e6;
    long counter_ = 0;
};

class StatsTimepointDifference {

public:
    using timespan = std::chrono::duration<int64_t, std::nano>;

    StatsTimepointDifference();

    void reset();

    void updateFailure(const std::chrono::time_point<std::chrono::steady_clock> &a_capture_ts);

    void updateCapture(const std::chrono::time_point<std::chrono::steady_clock> &a_capture_ts);

    void printStats(std::ostream &a_stream) const;

private:
    unsigned long captured_message_cnt_ = 0;
    unsigned int failed_capture_cnt_ = 0;

    std::chrono::time_point<std::chrono::steady_clock> cap_first_;
    std::chrono::time_point<std::chrono::steady_clock> cap_latest_;
    timespan min_;
    timespan max_;
};

struct LongOption {
    struct option long_option_;
    std::string help_string_;

    std::string printHelp() const
    {
        std::stringstream ss;
        ss << "    --" << std::left << std::setw(15) << long_option_.name << " ";
        if (long_option_.has_arg == required_argument) {
            ss << "<arg>" << std::left << std::setw(10);
        }
        else if (long_option_.has_arg == optional_argument) {
            ss << "[<arg>]" << std::left << std::setw(8);
        }
        else {
            ss << std::left << std::setw(15);
        }
        ss << " " << help_string_ << std::left << std::setw(0);
        return ss.str();
    }
};

/**
 * @brief Struct of Arrays of LongOptions to be used for processing dedicated arguments
 *        from the examples.
 *
 * @tparam N
 */
template <size_t N> class SOALongOptions {
public:
    SOALongOptions(std::array<LongOption, N> a_long_options)
    {
        for (int i = 0; i < N; i++) {
            long_options_[i] = a_long_options[i].long_option_;
            help_strings_[i] = a_long_options[i].help_string_;
        }
        long_options_[N] = {0, 0, 0, 0}; // terminator
    }

    std::string helpString() const
    {
        std::stringstream ss;
        ss << "[DEDICATED OPTIONS]\n";
        for (int i = 0; i < N; i++) {
            ss << formatHelp(long_options_[i], help_strings_[i]) << "\n";
        }
        return ss.str();
    }

    struct option *getLongOptions() { return long_options_; }

    size_t getNumberOfOptions() const { return N; }

private:
    std::string formatHelp(const struct option &a_option, const std::string &a_help_string) const
    {
        std::stringstream ss;
        ss << "    --" << std::left << std::setw(15) << a_option.name << " ";
        if (a_option.has_arg == required_argument) {
            ss << "<arg>" << std::left << std::setw(10);
        }
        else if (a_option.has_arg == optional_argument) {
            ss << "[<arg>]" << std::left << std::setw(8);
        }
        else {
            ss << std::left << std::setw(15);
        }
        ss << " " << a_help_string << std::left << std::setw(0);
        return ss.str();
    }

private:
    struct option long_options_[N + 1]; // +1 for the termination
    std::array<std::string, N> help_strings_;
};

class SystemAlarmStateDecoder {
public:
    static std::string decodeAsString(uint32_t systemAlarmState)
    {
        std::ostringstream oss;
        oss << "-------" << '\n';
        oss << "System Alarm state: " << systemAlarmState << '\n';

        oss << "Category: ";
        switch (systemAlarmState & 0b1111) {
        case static_cast<uint32_t>(kr2::kord::protocol::ESystemAlarmCategory::CAT_SAFETY_EVENT):
            oss << "Safety Event\n";
            break;
        case static_cast<uint32_t>(kr2::kord::protocol::ESystemAlarmCategory::CAT_SOFT_STOP_EVENT):
            oss << "Soft Stop Event\n";
            if ((systemAlarmState >> 8) &
                static_cast<uint32_t>(kr2::kord::protocol::ESoftStopEventConditionID::CBUN_KORD_BAD_CONN_QUALITY)) {
                oss << "Condition: CBun connection quality based on the limits set\n";
            }
            break;
        case static_cast<uint32_t>(kr2::kord::protocol::ESystemAlarmCategory::CAT_HW_STAT):
            oss << "Hardware Event\n";
            break;
        default:
            oss << "No Events\n";
            break;
        }

        oss << "Context: ";
        switch (systemAlarmState & 0b111110000) {
        case static_cast<uint32_t>(kr2::kord::protocol::ESystemAlarmContext::CNTXT_ESTOP):
            oss << "EStop\n";
            break;
        case static_cast<uint32_t>(kr2::kord::protocol::ESystemAlarmContext::CNTXT_PSTOP):
            oss << "PStop\n";
            break;
        case static_cast<uint32_t>(kr2::kord::protocol::ESystemAlarmContext::CNTXT_SSTOP):
            oss << "SSTOP\n";
            break;
        case static_cast<uint32_t>(kr2::kord::protocol::ESystemAlarmContext::CNTXT_SYSERR):
            oss << "SYSERR\n";
            break;
        default:
            oss << "No context\n";
        }

        oss << "-------\n";
        return oss.str();
    }
};

struct LaunchParameters {

    using ExternalArgParser = std::function<void(int)>;
    static const int INVALID_INDEX = -1;

    static LaunchParameters processLaunchArguments(int argc, char **argv, ExternalArgParser a_parser = [](int) {})
    {
        LaunchParameters parameters;

        static struct option long_options[] =
            {// Add only long options shared among all examples.
             // Specific options should be added and handled in the example using the ExternalArgParser
             {0, 0, 0, 0},
             {nullptr, 0, nullptr, 0}};

        if (argc <= 1) {
            parameters.init_time_ = std::chrono::steady_clock::now();
            parameters.valid_ = false;
            return parameters;
        }

        int opt;
        int option_index = 0;
        opterr = 0;
        while ((opt = getopt_long(argc, argv, "ht:r:p:c:i:n:", long_options, &option_index)) != -1) {
            switch (opt) {
            case 'h': {
                parameters.help_ = true;
                printUsage();
                a_parser(INVALID_INDEX); // It should print help
                break;
            }
            case 'r': {
                parameters.rt_prio_ = atoi(optarg);
                break;
            }
            case 't': {
                parameters.runtime_ = std::chrono::seconds(atoi(optarg));
                break;
            }
            case 'p': {
                parameters.port_ = atoi(optarg);
                if (parameters.port_ > 65535) {
                    parameters.valid_ = false;
                    std::cerr << "ERROR: Invalid port number: " << parameters.port_ << "\n";
                    return parameters;
                }
                break;
            }
            case 'c': {
                unsigned char buf[sizeof(struct in6_addr)];
                parameters.remote_controller_ = optarg;

                if (inet_pton(AF_INET, parameters.remote_controller_.c_str(), buf) <= 0) {
                    parameters.valid_ = false;
                    std::cerr << "ERROR: Malformed IP address: " << parameters.remote_controller_ << "\n";
                    return parameters;
                }
                break;
            }
            case 'i': {
                parameters.session_id_ = atoi(optarg);
                if (parameters.session_id_ > 255) {
                    parameters.valid_ = false;
                    std::cerr << "ERROR: Invalid session ID: " << parameters.session_id_ << "\n";
                    return parameters;
                }
                break;
            }
            case 'n': {
                parameters.rpose = atoi(optarg);
                break;
            }
            // If anything else is here, handover to external parser
            case 0:
            case '?':
            default: {
                a_parser(optind - 1);
            }
            }
        }

        parameters.init_time_ = std::chrono::steady_clock::now();
        parameters.valid_ = true;
        return parameters;
    }

    static void printUsage(bool with_time = false)
    {
        std::cout << "[OPTIONS]\n";
        std::cout << "    -h               show this help\n";
        if (with_time)
            std::cout << "    -t <runtime>     set for how long the example should run\n";
        std::cout << "    -r <prio>        execute as a realtime process with priority set to <prio>\n";
        std::cout << "    -p <port>        port number to connect to\n";
        std::cout << "    -c <IP Address>  remote controller IP address\n";
        std::cout << "    -i <Session ID>  KORD session ID | Default: 1\n";
        std::cout << "    -n <Pose number> Predefined Robot pose\n";
    }

    bool runtimeElapsed()
    {
        if (runtime_.count() == 0) {
            return false;
        }

        if ((std::chrono::steady_clock::now() - init_time_) > runtime_) {
            return true;
        }

        return false;
    }

    bool useRealtime() { return rt_prio_ > 0; }

    std::string printParameters() const
    {
        std::stringstream ss;
        ss << "Started with following parameters:\n";
        ss << "    Remote RC IP/port: " << remote_controller_ << ":" << port_ << "\n";
        ss << "    Session ID       : " << session_id_ << "\n";
        ss << "    Realtime Priority: " << rt_prio_ << "\n";
        ss << "    Runtime          : " << runtime_.count() << " seconds\n";
        ss << "    Robot Pose       : " << rpose << "\n";
        return ss.str();
    }

    bool valid_ = false;
    bool help_ = false;
    int rt_prio_ = -1;
    std::chrono::seconds runtime_{0};
    int port_ = 7582;
    int session_id_ = 1;
    int rpose = 0;
    std::string remote_controller_ = "192.168.38.1";

    std::chrono::time_point<std::chrono::steady_clock> init_time_;
};

class Stats {
public:
    Stats()
        : failed_capture_cnt_(0), cnt_(0), max_rx(INT64_MIN), min_rx(INT64_MAX), max_tx(INT64_MIN), min_tx(INT64_MAX),
          recents_buffer_size_(STATS_INTERVAL_ / (KORD_REF_PERIOD_ / 1000000))
    {
    }

    void reset()
    {
        max_rx = INT64_MIN;
        min_rx = INT64_MAX;
        max_tx = INT64_MIN;
        min_tx = INT64_MAX;
        cnt_ = 0;
        failed_capture_cnt_ = 0;
        sum_jitter = 0;
        recents_buffer_size_ = STATS_INTERVAL_ / (KORD_REF_PERIOD_ / 1000000);
        last_received_seq_number_ = -1;
        local_packets_lost_seq = 0;
        global_packets_lost_seq = 0;
        global_cons_packets_lost_max = 0;
        clear(jitters_buffer);
        clear(lost_packets_buffer_cbun);
        clear(stats_sequences_buffer);
    }

    void capture(const std::chrono::time_point<std::chrono::steady_clock> a_capture_ts,
                 int64_t a_capture_ts_cbun = -1,
                 int a_status_sequence_num = -1)
    {
        if (cnt_ == 0) {
            cap_first_ = a_capture_ts;
            cap_latest_ = a_capture_ts;
            cap_first_cbun_ = a_capture_ts_cbun;
            cap_latest_cbun_ = a_capture_ts_cbun;
            ++cnt_;
            return;
        }
        ///
        // rx analysis
        //

        std::chrono::duration<int64_t, std::nano> elapsed_rx = a_capture_ts - cap_latest_;

        long t_diff_rx = elapsed_rx.count();

        // global extremums
        if (t_diff_rx < min_rx) {
            min_rx = t_diff_rx;
        }
        if (t_diff_rx > max_rx) {
            max_rx = t_diff_rx;
        }

        window_jitter_statistics(t_diff_rx);
        window_lost_statistics(t_diff_rx, lost_packets_buffer_api, sum_lost_api);
        // window_lost_statistics(t_diff_rx);

        if (a_capture_ts_cbun != -1) {
            ///
            // tx analysis
            //
            int64_t t_diff_tx = a_capture_ts_cbun - cap_latest_cbun_;

            // global extremums
            if (t_diff_tx < min_tx) {
                min_tx = t_diff_tx;
            }
            if (t_diff_tx > max_tx) {
                max_tx = t_diff_tx;
            }

            window_lost_statistics(t_diff_tx, lost_packets_buffer_cbun, sum_lost_cbun);

            cap_latest_cbun_ = a_capture_ts_cbun;
        }

        if (a_status_sequence_num != -1) {
            //
            // Processing of the status's sequence number
            processNewSequenceNumber(a_status_sequence_num);
        }

        cap_latest_ = a_capture_ts;
        ++cnt_;
    }

    void processNewSequenceNumber(const uint16_t &a_rcv_sequence_number)
    {

        checkOfTheReceivedSequenceNumberGlobal(a_rcv_sequence_number);
        checkOfTheReceivedSequenceNumberLocal(a_rcv_sequence_number);

        last_received_seq_number_ = a_rcv_sequence_number;
    }

    // checks sequence numbers continuity
    void checkOfTheReceivedSequenceNumberGlobal(const uint16_t &a_rcv_sequence_number)
    {
        if ((last_received_seq_number_ == -1) || (a_rcv_sequence_number == 0)) {
            // start, just reset and record
            return;
        }
        int diff_ = int(a_rcv_sequence_number) - int(last_received_seq_number_);
        if (diff_ > 1) {
            // lost packet error
            global_packets_lost_seq += 1;
        }
        if (diff_ - 1 > global_cons_packets_lost_max) {
            global_cons_packets_lost_max = diff_ - 1;
        }
    }

    // checks sequence numbers continuity (on STATS_INTERVAL_)
    void checkOfTheReceivedSequenceNumberLocal(const uint16_t &a_rcv_sequence_number)
    {
        if ((last_received_seq_number_ == -1) || (a_rcv_sequence_number == 0)) {
            // start, just record
            return;
        }
        int diff_ = int(a_rcv_sequence_number) - int(last_received_seq_number_);
        if (diff_ > 1) { // ok difference is 1
            local_packets_lost_seq += (diff_ - 1);
            stats_sequences_buffer.push((diff_ - 1));
        }
        else {
            stats_sequences_buffer.push(0);
        }
        if (stats_sequences_buffer.size() > recents_buffer_size_) { // STATS_INTERVAL_ passed
            local_packets_lost_seq -= stats_sequences_buffer.front();
            stats_sequences_buffer.pop();
        }

        last_received_seq_number_ = a_rcv_sequence_number;
    }

    int number_of_lost_calculation(const long &a_diff)
    {
        if (a_diff > MAX_TX_PACKETS_DIFF_) {
            return std::max((long)(a_diff / KORD_REF_PERIOD_ - 1), long(1));
        }
        return 0;
    }

    int jitter_calculation(const long &a_diff, const long &a_ref)
    {
        return sqrt(abs(a_diff * a_diff - KORD_REF_PERIOD_ * KORD_REF_PERIOD_));
    }

    void window_jitter_statistics(const long &a_rx_diff)
    {

        int moment_jitter = jitter_calculation(a_rx_diff, KORD_REF_PERIOD_);
        jitters_buffer.push(moment_jitter);
        sum_jitter += moment_jitter;

        if (jitters_buffer.size() > recents_buffer_size_) { // STATS_INTERVAL_ passed

            int deleted_element = jitters_buffer.front();
            sum_jitter -= deleted_element;
            jitters_buffer.pop();
        }
    }

    void global_jitter_statistics(const long &a_rx_diff)
    {

        int moment_jitter = jitter_calculation(a_rx_diff, KORD_REF_PERIOD_);
        jitters_buffer.push(moment_jitter);
        sum_jitter += moment_jitter;

        if (jitters_buffer.size() > recents_buffer_size_) { // STATS_INTERVAL_ passed

            int deleted_element = jitters_buffer.front();
            sum_jitter -= deleted_element;
            jitters_buffer.pop();
        }
    }

    void window_lost_statistics(const long &a_tx_diff, std::queue<int> &lost_packets_buffer, long &sum_lost)
    {

        int64_t moment_lost = number_of_lost_calculation(a_tx_diff);
        lost_packets_buffer.push(moment_lost);
        sum_lost += moment_lost;

        if (lost_packets_buffer.size() > recents_buffer_size_) { // STATS_INTERVAL_ passed

            int deleted_element = lost_packets_buffer.front();
            sum_lost -= deleted_element;
            lost_packets_buffer.pop();
        }
    }

    void update_failure()
    {
        ++failed_capture_cnt_;

        capture(std::chrono::steady_clock::now());
    }

    // global
    int64_t get_avg_rx()
    {
        int64_t avg_cap =
            std::chrono::duration_cast<std::chrono::duration<int64_t, std::nano>>(cap_latest_ - cap_first_).count();
        return avg_cap / static_cast<int64_t>(cnt_);
    }

    int64_t get_max_rx() { return max_rx; }

    int64_t get_min_rx() { return min_rx; }

    int64_t get_max_rx_jitter() { return jitter_calculation(get_max_rx(), KORD_REF_PERIOD_); }

    int64_t get_avg_rx_jitter() { return jitter_calculation(get_avg_rx(), KORD_REF_PERIOD_); }

    int64_t get_avg_tx()
    {
        int64_t avg_cap = cap_latest_cbun_ - cap_first_cbun_;
        if (cnt_ == 0){
            return 0;
        } 
        return avg_cap / static_cast<int64_t>(cnt_);
    }

    int64_t get_max_tx() { return max_tx; }

    int64_t get_min_tx() { return min_tx; }

    int64_t get_max_drop_global() { return global_cons_packets_lost_max; }

    // local
    int64_t get_avg_jitter() { return sum_jitter / std::max(size_t(1), jitters_buffer.size()); }

    int64_t get_min_jitter()
    {
        int64_t min_jitter = INT64_MAX;
        std::queue<int> copy_queue = jitters_buffer;
        while (!copy_queue.empty()) {
            int current = copy_queue.front();
            if (current < min_jitter) {
                min_jitter = current;
            }
            copy_queue.pop();
        }
        return min_jitter;
    }

    int64_t get_max_jitter()
    {
        int64_t max_jitter = INT64_MIN;
        std::queue<int> copy_queue = jitters_buffer;
        while (!copy_queue.empty()) {
            int current = copy_queue.front();
            if (current > max_jitter) {
                max_jitter = current;
            }
            copy_queue.pop();
        }

        return max_jitter;
    }

    int64_t get_avg_lost_cbun() { return sum_lost_cbun / std::max(size_t(1), lost_packets_buffer_cbun.size()); }

    int64_t get_avg_lost_seq() { return local_packets_lost_seq / std::max(size_t(1), stats_sequences_buffer.size()); }
    int64_t get_avg_lost_api() { return sum_lost_api / std::max(size_t(1), lost_packets_buffer_api.size()); }

    int64_t get_min_lost_cbun() { return get_min_lost(lost_packets_buffer_cbun); }

    int64_t get_min_lost_api() { return get_min_lost(lost_packets_buffer_api); }

    int64_t get_min_lost_seq() { return get_min_lost(stats_sequences_buffer); }

    int64_t get_min_lost(std::queue<int> &buffer)
    {
        int64_t min_lost = INT64_MAX;
        // finding minimum using timestamps
        std::queue<int> copy_queue = buffer;
        while (!copy_queue.empty()) {
            int current = copy_queue.front();
            if (current < min_lost) {
                min_lost = current;
            }
            copy_queue.pop();
        }
        return min_lost;
    }

    int64_t get_max_lost_cbun() { return get_max_lost(lost_packets_buffer_cbun); }

    int64_t get_max_lost_api() { return get_max_lost(lost_packets_buffer_api); }

    int64_t get_max_lost_seq() { return get_max_lost(stats_sequences_buffer); }

    int64_t get_max_lost(std::queue<int> &buffer)
    {
        int64_t max_lost = INT64_MIN;
        // finding maximum using timestamps
        std::queue<int> copy_queue = buffer;
        while (!copy_queue.empty()) {
            int current = copy_queue.front();
            if (current > max_lost) {
                max_lost = current;
            }
            copy_queue.pop();
        }
        return max_lost;
    }

    int64_t get_failed_cnt() { return failed_capture_cnt_; }

    void set_stats_window(const int32_t &a_number_of_recents) { recents_buffer_size_ = a_number_of_recents; }

    void clear(std::queue<int> &q)
    {
        std::queue<int> empty;
        std::swap(q, empty);
    }

public:
    std::chrono::time_point<std::chrono::steady_clock> cap_first_;
    std::chrono::time_point<std::chrono::steady_clock> cap_latest_;

    int64_t cap_first_cbun_;
    int64_t cap_latest_cbun_;

    int64_t failed_capture_cnt_;

    size_t cnt_;

    int64_t max_rx = INT64_MIN;
    int64_t min_rx = INT64_MAX;

    int64_t max_tx = INT64_MIN;
    int64_t min_tx = INT64_MAX;

    const int64_t STATS_INTERVAL_ = 1000;     // default, in milliseconds
    const int64_t KORD_REF_PERIOD_ = 4000000; // nanoseconds
    // TODO: derive from init

    const int64_t MAX_TX_PACKETS_DIFF_ = KORD_REF_PERIOD_ + KORD_REF_PERIOD_ / 2; // in nanoseconds

    int64_t recents_buffer_size_ = STATS_INTERVAL_ / (KORD_REF_PERIOD_ / 1000000);

    std::queue<int> jitters_buffer;
    std::queue<int> lost_packets_buffer_cbun;
    std::queue<int> lost_packets_buffer_api;

    long sum_jitter = 0;
    long sum_lost_cbun = 0;
    long sum_lost_api = 0;

    int last_received_seq_number_ = -1;
    int64_t local_packets_lost_seq = 0;
    int64_t global_packets_lost_seq = 0;

    int64_t global_cons_packets_lost_max = 0;

    std::queue<int> stats_sequences_buffer;
};

namespace realtime {

bool set_latency_target(void);

bool init_realtime_params(int a_prio);

} // namespace realtime

} // namespace kr2::utils

#endif // KR2_KORD_UTILS_H
