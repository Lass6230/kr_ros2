
#ifndef __WORKER_H_
#define __WORKER_H_

#include <thread>
#include <atomic>
#include <iostream>


class Worker {

    public:
        
        /// @brief Start execution of defined job
        inline void start() {
            if (nullptr == t) 
            {
                t = new std::thread(Worker::exec, this);
                // mark the thread as working
                this->running();
            } 
            else // not new, but is it finished?
            {
                if (this->is_done()) {
                    t = new std::thread(Worker::exec, this);
                    // mark the thread as working
                    this->running();
                }
            }
        }

        /// @brief Wait for the end of the executing
        inline void join() {
            if (nullptr != t) {
                t->join();
                delete t;
                t = nullptr;
            }
        }

        /// @brief Wait for the end of the executing
        inline bool joinable() {
            if (nullptr != t) {
                return t->joinable();
            }
	    return false;
        }

        /// @brief  Is the task finished
        /// @return bool 
        inline bool is_done() {
            return this->done;
        }

    protected:
        
        /// @brief The method to overload in order to provide your functionality
        virtual void run() = 0;	

    private:
        
        // pointer to the thread
        std::thread* t         = nullptr;
        std::atomic<bool> done = true;

        // static function which points back to the instance
        static void exec(Worker* Worker) {
            Worker->run();
        }

        inline void running()  { this->done = false; }

    protected:
        
        inline void finished() { this->done = true;  }

};

#endif
