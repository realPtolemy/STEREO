#include "talking/class1.hpp"
#include <chrono>

Class1::Class1(shared_state& shared_state)
{
    shared_state_ptr = &shared_state;
}

void Class1::method1()
{
    int count = 0;
    while (true)
    {
        // std::cout << "Class1 method1" << std::endl;
        {
            std::unique_lock<std::mutex> lock(shared_state_ptr->pcl_mtx);
            if(shared_state_ptr->msg == "msg1")
            {
                shared_state_ptr->msg = "msg2";
                // count++;
            }
            else
            {
                shared_state_ptr->msg = "msg1";
                // if (count == 4)
                // {
                    // shared_state_ptr->ready = true;
                    // shared_state_ptr->cv.notify_all();
                    // count = 0;
                // }
            }
            std::cout << "Class1 msg: " << shared_state_ptr-> msg << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}