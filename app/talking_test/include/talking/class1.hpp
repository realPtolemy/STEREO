#pragma once
#include "talking/common.hpp"

class Class1 {
public:
    shared_state* shared_state_ptr;

    Class1(shared_state& shared_state);
    void method1();
};