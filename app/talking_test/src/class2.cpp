#include "talking/class2.hpp"

Class2::Class2(shared_state& shared_state)
{
    shared_state_ptr = &shared_state;
}

void Class2::method1()
{
    while (true)
    {
        // waitForReady(*shared_state_ptr);

        {
            std::unique_lock<std::mutex> lock(shared_state_ptr->pcl_mtx);
            std::cout << "Class2 method1: " << shared_state_ptr->msg << std::endl;
            shared_state_ptr->person.name = shared_state_ptr->msg;
        }
        // shared_state_ptr->ready = false;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}