#pragma once
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <iostream>
#include <vector>

struct Person{
    std::string name;
    int age;
    std::string address;
};

struct shared_state{
    std::string msg;
    Person person;
    std::mutex pcl_mtx;
    std::mutex pose_mtx;
    std::condition_variable cv;
    bool ready = false;
};

bool waitForReady(shared_state& state);