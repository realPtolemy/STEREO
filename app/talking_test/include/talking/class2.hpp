#pragma once
#include "talking/common.hpp"


class Class2 {
public:
    shared_state* shared_state_ptr;
    Class2(shared_state& shared_state);
    void method1();

};