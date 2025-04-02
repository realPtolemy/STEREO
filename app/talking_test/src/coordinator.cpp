#include "talking/coordinator.hpp"
#include "talking/class1.hpp"
#include "talking/class2.hpp"

Coordinator::Coordinator() {
    shared_state state;
    // shared_state som pointers? Shared pointer?
    Class1 class1_instance(state);
    Class2 class2_instance(state);

    std::thread thread1(&Class1::method1, &class1_instance);
    std::thread thread2(&Class2::method1, &class2_instance);

    // class1_instance.shared_state_ptr->msg
    thread1.join();
    thread2.join();
    std::cout << "Class1 method1 finished" << std::endl;
};
// Coordinator::~Coordinator()
// {
// }