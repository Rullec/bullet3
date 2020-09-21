#include "TimeUtil.hpp"
#include <ctime>
#include <iostream>

using namespace std;
using namespace std::chrono;

void btTimeUtil::Begin(const std::string &name)
{
    mTimeTable[name] = high_resolution_clock::now();
}

void btTimeUtil::End(const std::string &name)
{
    time_it = mTimeTable.find(name);
    if (time_it == mTimeTable.end())
    {
        std::cout << "[error] btTimeUtil::End No static info about " << name
                  << std::endl;
        exit(0);
    }

    std::cout << "[log] " << name << " cost time = "
              << (high_resolution_clock::now() - time_it->second).count() * 1e-6
              << " ms\n";
    mTimeTable.erase(time_it);
}

std::string btTimeUtil::GetSystemTime()
{
    // not thread safe
    std::time_t now =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string s(30, '\0');
    std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    s.resize(s.find('\0'));
    return s;
}

void btTimeUtil::BeginLazy(const std::string &name) { btTimeUtil::Begin(name); }

void btTimeUtil::EndLazy(const std::string &name)
{
    time_it = mTimeTable.find(name);
    if (time_it == mTimeTable.end())
    {
        std::cout << "[error] btTimeUtil::End No static info about " << name
                  << std::endl;
        exit(0);
    }

    mLazyTimeTable[name] +=
        (high_resolution_clock::now() - time_it->second).count() * 1e-6;
}

void btTimeUtil::ClearLazy(const std::string &name)
{
    std::cout << "[log] segment lazy " << name
              << " cost time = " << mLazyTimeTable[name] << " ms\n";
    mLazyTimeTable[name] = 0;
}