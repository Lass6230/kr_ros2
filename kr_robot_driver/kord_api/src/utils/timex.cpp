/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, KR2013ApS
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
 *   * Neither the name of the KR2013ApS nor the names of its
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
#include <kord/utils/timex.h>

#include <iostream>

#include <sstream>
#include <iomanip>
#include <limits>
// #include <boost/format.hpp>
#include <memory>
#include <cassert>

#define SECS_PER_DAY (24*60*60)

using namespace kr2::utils;
    
    class Timex::ClockBase{
    public:
        ClockBase(){};
        virtual ~ClockBase(){};
        
        virtual const struct timespec &now() = 0;
        virtual bool now(struct timespec &now) = 0;
        virtual int nanosleepUntil(const struct timespec &a_wakeup_time) = 0;
        virtual int nanosleepRelative(const struct timespec &a_relative_time) = 0;
        virtual void incrementTime() { return; };
        
        struct timespec ts_init_ = {0, 0};
        
        static struct timespec diffTimeTs(const struct timespec& a_time1, const struct timespec& a_time2) {
            struct timespec res, time1, time2;
            
            time1 = a_time1;
            time2 = a_time2;
            
            /* Perform the carry for the later subtraction by updating y. */
            if (time1.tv_nsec < time2.tv_nsec) {
                unsigned long nsec = (time2.tv_nsec - time1.tv_nsec) / NSEC_PER_SEC + 1;
                time2.tv_nsec -= NSEC_PER_SEC * nsec;
                time2.tv_sec += nsec;
            }
            
            if (time1.tv_nsec > time2.tv_nsec + (signed long)(NSEC_PER_SEC)) {
                unsigned long nsec = (time1.tv_nsec - time2.tv_nsec) / NSEC_PER_SEC;
                time2.tv_nsec += NSEC_PER_SEC * nsec;
                time2.tv_sec -= nsec;
            }
            
            /* Compute the time remaining to wait.
                tv_usec is certainly positive. */
            res.tv_sec = time1.tv_sec - time2.tv_sec;
            res.tv_nsec = time1.tv_nsec - time2.tv_nsec;
            
            return res;
    }
        static void addNs2Timspec(struct timespec &a_time, long long int a_ns) {
            a_time.tv_sec += a_ns / ((long long int) NSEC_PER_SEC);
            a_time.tv_nsec += a_ns % ((long long int) NSEC_PER_SEC);
            if (a_time.tv_nsec >= (long long int) NSEC_PER_SEC)
            {
                a_time.tv_nsec -= (long long int)  NSEC_PER_SEC;
                a_time.tv_sec += 1;
            }
            if (a_time.tv_nsec < 0)
            {
                a_time.tv_nsec += (long long int)  NSEC_PER_SEC;
                a_time.tv_sec -= 1;
            }
        }
    };

    class ClockRT:public Timex::ClockBase{
    public:
        ClockRT(){
#ifdef __MACH__
            mach_timebase_info(&mach_tbase_);
            mach_timeinit_ = 0; // mach_absolute_time();
#endif // __MACH__
            
            now(ts_init_);
        };
        virtual ~ClockRT(){};
        
        virtual const struct timespec& now() {
#ifdef __MACH__
            uint64_t d = mach_absolute_time() - mach_timeinit_;
            d *= mach_tbase_.numer;
            d /= mach_tbase_.denom;
            
            stamp_.tv_sec = d / NSEC_PER_SEC;
            stamp_.tv_nsec = d % NSEC_PER_SEC;
#else
            clock_gettime(USED_CLOCK, &stamp_);
#endif
            return stamp_;
        }
        
        virtual bool now(struct timespec &a_out_ts_now) {
#ifdef __MACH__
            uint64_t d = mach_absolute_time() - mach_timeinit_;
            d *= mach_tbase_.numer;
            d /= mach_tbase_.denom;
            
            a_out_ts_now.tv_sec = d / NSEC_PER_SEC;
            a_out_ts_now.tv_nsec = d % NSEC_PER_SEC;
            return true;
#else
            return clock_gettime(USED_CLOCK, &a_out_ts_now) == 0;
#endif
        }
        
        virtual int nanosleepUntil(const struct timespec& a_wakeup_time) {
            struct timespec relative_sleep_time, ts_now;
#ifdef __MACH__
            ts_now = now(); //clock_gettime(USED_CLOCK, &now);
            relative_sleep_time = diffTimeTs(a_wakeup_time, ts_now);
            return nanosleep(&relative_sleep_time, NULL);
#else
            clock_gettime(USED_CLOCK, &ts_now);
            relative_sleep_time = diffTimeTs(a_wakeup_time, ts_now);
            return clock_nanosleep(USED_CLOCK, 0, &relative_sleep_time, NULL);
#endif
        }

        virtual int nanosleepRelative(const struct timespec& a_sleep_time) {
#ifdef __MACH__
            return nanosleep(&a_sleep_time, NULL);
#else
            return clock_nanosleep(USED_CLOCK, 0, &a_sleep_time, NULL);
#endif
        }
        
    private:
        struct timespec stamp_;
#ifdef __MACH__
        mach_timebase_info_data_t mach_tbase_;
        uint64_t mach_timeinit_;
#endif // __MACH__
    };

    class ClockSim:public Timex::ClockBase{
    public:
        ClockSim(int frequency):ClockBase(),
        simulation_time_{0,0}
        {
            sim_step_ns_ = 0;
            if (frequency > 0) sim_step_ns_ = (1.0/double(frequency))*NSEC_PER_SEC;
        }
        
        virtual ~ClockSim(){};
        
        virtual const struct timespec& now() { return simulation_time_; }
        virtual bool now(struct timespec &a_out_ts_now) { a_out_ts_now = simulation_time_; return true; }
            
        virtual int nanosleepUntil(const struct timespec& a_wakeup_time) {
            if (  a_wakeup_time.tv_sec > simulation_time_.tv_sec
                || (a_wakeup_time.tv_sec == simulation_time_.tv_sec && a_wakeup_time.tv_nsec > simulation_time_.tv_nsec)
                ){
                simulation_time_ = a_wakeup_time;
            }
            return 0;
        }
        
        virtual int nanosleepRelative(const struct timespec& a_sleep_time) {
            return 0;
        }
        
        virtual void incrementTime(){ addNs2Timspec(simulation_time_,sim_step_ns_); }
        
    private:
        struct timespec simulation_time_;
        uint64_t sim_step_ns_;
    };


static std::unique_ptr<Timex> g_timex_instance;


Timex::Timex(int a_frequency, bool a_simulation)
:freq_(a_frequency), nsecs_per_tick_(a_frequency > 0 ? NSEC_PER_SEC / a_frequency : 0)
{
    if (a_simulation)
    {
        clock_.reset(new ClockSim(a_frequency));
    }
    else
    {
        clock_.reset(new ClockRT());
    }
}

Timex::~Timex()
{
}

void Timex::initSingletonTimex(int a_frequency, bool a_simulation)
{
    if (!g_timex_instance)
        g_timex_instance.reset(new Timex(a_frequency, a_simulation));
    else {
        std::cerr << "[SYSERR] timex instance reinit not allowed\n";
    }
}

void Timex::resetSingletonTimex(int frequency, bool simulation)
{
    if (g_timex_instance)
        g_timex_instance.reset(new Timex(frequency, simulation));
    else {
        std::cerr << "[SYSERR] timex instance not initialized\n";
    }
}

const struct timespec& Timex::now()
{   
    if (g_timex_instance)
        return g_timex_instance->clock_->now();
    else {
        static timespec void_ts {0, 0};
        std::cerr << "[SYSERR] timex not initialized\n";
        assert(false);
        return void_ts;
    }
}

bool Timex::now(struct timespec &a_out_ts_now)
{
    if (g_timex_instance)
        return g_timex_instance->clock_->now(a_out_ts_now);
    else {
        std::cerr << "[SYSERR] timex not initialized";
        return false;
    }
}

struct timespec Timex::now2()
{
    return clock_->now();
}

int Timex::nanosleepUntil(const struct timespec& a_wakeup_time)
{
    if (g_timex_instance)
        return g_timex_instance->clock_->nanosleepUntil(a_wakeup_time);
    else
        return CLOCK_NOT_INITIALIZED;
}

int Timex::nanosleepUntil2(const struct timespec &a_wakeup_time)
{
    return clock_->nanosleepUntil(a_wakeup_time);
}

int Timex::nanosleep(const struct timespec &a_period_time)
{
    if (g_timex_instance)
        return g_timex_instance->clock_->nanosleepRelative(a_period_time);
    else
        return CLOCK_NOT_INITIALIZED;
}

void Timex::tick()
{
    if (g_timex_instance)
        return g_timex_instance->clock_->incrementTime();
}

void Timex::tick2()
{
    clock_->incrementTime();
}

double Timex::diff(const struct timespec& a_time1, const struct timespec& a_time2)
{
    struct timespec ts_res = ClockBase::diffTimeTs(a_time1, a_time2);
    return ((double)ts_res.tv_sec) + ((double)ts_res.tv_nsec)/NSEC_PER_SEC;
}

long long Timex::nsecDiff(const struct timespec &tm1, const struct timespec &tm2)
{
    return (tm2.tv_sec == tm1.tv_sec ? (tm1.tv_nsec - tm2.tv_nsec) : ((tm1.tv_sec - tm2.tv_sec - 1)*NSEC_PER_SEC + NSEC_PER_SEC - tm2.tv_nsec + tm1.tv_nsec));
}

bool Timex::eq(const struct timespec &time1, const struct timespec &time2)
{
    return (time1.tv_sec == time2.tv_sec && time1.tv_nsec == time2.tv_nsec);
}

bool Timex::soonerEq(const struct timespec& time1, const struct timespec& time2)
{
    return !Timex::later(time1, time2);
}

bool Timex::laterEq(const struct timespec& time1, const struct timespec& time2)
{
    if (time1.tv_sec == time2.tv_sec)
    {
        return time1.tv_nsec >= time2.tv_nsec;
    }
    return time1.tv_sec > time2.tv_sec;
}

bool Timex::sooner(const struct timespec& time1, const struct timespec& time2)
{
    return !Timex::laterEq(time1, time2);
}

bool Timex::later(const struct timespec& time1, const struct timespec& time2)
{
    if (time1.tv_sec == time2.tv_sec)
    {
        return time1.tv_nsec > time2.tv_nsec;
    }
    return time1.tv_sec > time2.tv_sec;
}

struct timespec Timex::thisTick2(const struct timespec& a_ts, int a_period_offset)
{
    struct timespec tick;
    
    tick.tv_sec = a_ts.tv_sec;
    tick.tv_nsec = (a_ts.tv_nsec / nsecs_per_tick_) * nsecs_per_tick_;
    
    if (a_period_offset)
        ClockBase::addNs2Timspec(tick, nsecs_per_tick_ * a_period_offset);
    
    return tick;
}

struct timespec Timex::thisTick(const struct timespec& a_ts, int a_period_offset)
{

    return g_timex_instance->thisTick2(a_ts, a_period_offset);
}


struct timespec Timex::nextTick2(const struct timespec &a_ts, unsigned *a_err_flags)
{
    struct timespec ret = a_ts;
    
    unsigned long tickidx = a_ts.tv_nsec / nsecs_per_tick_;
    ret.tv_nsec = tickidx * nsecs_per_tick_;
    
    if (tickidx < freq_ - 1) {
        ret.tv_nsec = (tickidx + 1) * nsecs_per_tick_;
    } else if (tickidx == freq_ - 1) {
        ClockBase::addNs2Timspec(ret, nsecs_per_tick_);
    } else {
        if (a_err_flags) *a_err_flags = ERR_TIME_OP_INCONSISTENCY;
        ClockBase::addNs2Timspec(ret, nsecs_per_tick_);
    }
    
    return ret;
}

struct timespec Timex::nextTick(const struct timespec &a_ts, unsigned *a_err_flags)
{
    return g_timex_instance->nextTick2(a_ts, a_err_flags);
}

void Timex::addTicks(struct timespec &a_ts, unsigned short a_nticks)
{
    long long int nsecs = a_nticks * g_timex_instance->nsecs_per_tick_;
    ClockBase::addNs2Timspec(a_ts, nsecs);
}

void Timex::addFraction(struct timespec& ts, double a_fraction)
{
    ClockBase::addNs2Timspec(ts, g_timex_instance->nsecs_per_tick_ * a_fraction);
}

std::string Timex::timeStampStr()
{
    if (!g_timex_instance) return "0.00000.0000";
    struct timespec ts = g_timex_instance->now();
    struct timespec ts_from_init = ClockBase::diffTimeTs(ts, g_timex_instance->clock_->ts_init_);
    
    long d = ts_from_init.tv_sec / SECS_PER_DAY;
    long s = ts_from_init.tv_sec % SECS_PER_DAY;
    long mms = ts_from_init.tv_nsec / ( USEC_PER_SEC / 10 );

    return format(d, s, mms);
    
}

std::string Timex::format(long d, long s, long mms){
    std::string res = std::to_string(d) + ".";
    std::stringstream secs;
    secs << std::setw(5) << std::setfill('0') << s;
    std::stringstream mmsecs;
    mmsecs << std::setw(4) << std::setfill('0') << mms;
    return std::to_string(d) + "." + secs.str() + "." + mmsecs.str();
}

double Timex::period()
{
    if (g_timex_instance)
        return 1.0f/double(g_timex_instance->freq_);

    return -std::numeric_limits<double>::max();
}

double Timex::period2() const
{
    return 1.0/double(freq_);
}

bool Timex::valid()
{
    if (g_timex_instance)
        return true;
    
    return false;
}

const timespec &Timex::tsInit()
{
    return g_timex_instance->clock_->ts_init_;
}

