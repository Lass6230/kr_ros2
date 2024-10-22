#include <stdio.h>
#include <pthread.h>
#include <stdint.h>

#include <fcntl.h>
#include <unistd.h>
#include <csignal>

#include <sys/mman.h>
#include <sys/stat.h>

#include <climits>
#include <iostream>
#include <sstream>

#include <kord/utils/utils.h>

#define DEFAULT_PRIORITY     20
#define DEFAULT_PRIORITY_MAX 50

using namespace kr2::utils;


StampsLog::StampsLog()
{
    reset();
}

void StampsLog::reset() {
    t0_ = INT64_MAX;
    t1_ = INT64_MAX;
    t2_ = INT64_MAX;
}

std::string StampsLog::header(){
    std::stringstream ss;
    ss << "t0, t1, t2";
    return ss.str();
}

std::string StampsLog::asString(){
    std::stringstream ss;
    ss << t0_ << ", " << t1_ << ", " << t2_;
    return ss.str();
}

StatGeneral::StatGeneral()
{
    reset();
}

void StatGeneral::reset() {
    avg_ = 0;
    max_ = INT64_MIN;
    min_ = INT64_MAX;
    counter_ = 0;
}

void StatGeneral::update(long a_uval) {
    if (a_uval < min_) min_ = a_uval;
    if (a_uval > max_) max_ = a_uval;

    if (counter_ == 0) {
        avg_ = a_uval;
        counter_++;
        return;
    }

    long avg_t = (counter_ * avg_) + a_uval;
    avg_ = avg_t / (counter_ + 1);
    counter_++;
}

void StatGeneral::printStat(std::ostream &a_stream) const{
    a_stream << "min, avg, max\n";
    a_stream << min_ << ", "<< avg_ << ", " << max_ << " \n";
}


StatsTimepointDifference::StatsTimepointDifference() {
    this->reset();
}

void StatsTimepointDifference::reset() {
    using namespace std::chrono_literals;
    min_ = 1000s;
    max_ = 0us;
}

void StatsTimepointDifference::updateFailure(const std::chrono::time_point<std::chrono::steady_clock> &a_capture_ts){
    failed_capture_cnt_++;
}

void StatsTimepointDifference::updateCapture(const std::chrono::time_point<std::chrono::steady_clock> &a_capture_ts){
    if (captured_message_cnt_ == 0){
        cap_first_ = a_capture_ts;
        cap_latest_ = a_capture_ts;
        captured_message_cnt_++;
        return;
    }

    timespan elapsed = a_capture_ts - cap_latest_;
    if (elapsed < min_){
        min_ = elapsed;
    }

    if (elapsed > max_){
        max_ = elapsed;
    }

    cap_latest_ = a_capture_ts;
    captured_message_cnt_++;
}

void StatsTimepointDifference::printStats(std::ostream & a_stream) const{
    int64_t avg_cap = std::chrono::duration_cast<timespan>(cap_latest_ - cap_first_).count();
    avg_cap /= static_cast<int64_t>(captured_message_cnt_);
    a_stream << "Captured msgs; Failed msgs; min[ns]; avg capture[ns]; max[ns]\n";
    a_stream << captured_message_cnt_ << ";  " << failed_capture_cnt_ << "; " << min_.count() << "; " << avg_cap << "; " << max_.count() << "\n";
}

bool realtime::set_latency_target(void)
{
    struct stat s;
    int latency_target_fd = -1;
    int err;

    err = stat("/dev/cpu_dma_latency", &s);
    if (err == -1) {
        std::cerr << "[RealTime] getting file '/dev/cpu_dma_latency' status failed";
        return false;
    }

    latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1) {
        std::cerr << "[RealTime] opening '/dev/cpu_dma_latency' failed\n";
        return false;
    }

    int32_t latency_target_value = 0;

    ssize_t written = write(latency_target_fd, &latency_target_value, 4);
    if (written < 1) {
        close(latency_target_fd);
        std::cerr << "[RealTime] setting the cpu_dma_latency to " + std::to_string(latency_target_value)+ "us failed\n";
        return false;
    }

    close(latency_target_fd);

    return true;
}

bool realtime::init_realtime_params(int a_prio)
{
    if (!set_latency_target()){
        std::cerr << "Latency target error\n";
        return false;
    }

    // We'll operate on the currently running thread.
    pthread_t this_thread = pthread_self();

    // struct sched_param is used to store the scheduling priority
    struct sched_param params;
    // We'll set the priority to the maximum.
    params.sched_priority = DEFAULT_PRIORITY;

    if ((a_prio > 0) && (a_prio <= DEFAULT_PRIORITY_MAX)) {
        params.sched_priority = a_prio;
    }

    // Attempt to set thread real-time priority to the SCHED_FIFO policy
    std::cout << "[RealTime] setting thread realtime priority = " << params.sched_priority << "\n";
    int ret_value = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    if (ret_value != 0) {
        // Print the error
        std::cerr << "[RealTime] setting thread real-time priority failed";
        return false;
    }
    // Now verify the change in thread priority
    int policy = 0;
    ret_value = pthread_getschedparam(this_thread, &policy, &params);
    if (ret_value != 0) {
        std::cerr << "[RealTime] retrieving real-time scheduling parameters failed\n";
        return false;
    }

    // Check the correct policy was applied
    if(policy != SCHED_FIFO) {
        std::cerr << "[RealTime] Scheduling is NOT SCHED_FIFO! Setting scheduler failed\n";
        return false;
    } else {
        std::cout << "[RealTime] 'SCHED_FIFO' ok\n";
    }

    // Print thread scheduling priority
    std::cout << "[RealTime] thread priority is: " << params.sched_priority << "\n";

    // block SIGALRM
    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGALRM);
    sigprocmask(SIG_BLOCK, &sigset, NULL);

    // Lock process pages in RAM from swapping
    std::cout << "[RealTime] memory lock all\n";
    if (mlockall(MCL_FUTURE))
    {
        std::cerr << "[RealTime] mlockall failed\n";
        return false;
    } else {
        std::cout << "[RealTime] mlockall successful";
    }

    // Try detaching this thread
    pthread_detach(this_thread);

    return true;

}
