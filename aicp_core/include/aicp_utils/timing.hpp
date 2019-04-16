#ifndef AICP_TIMING_UTILS_HPP_
#define AICP_TIMING_UTILS_HPP_

#include <iostream>
#include <stack>
#include <ctime>

class TimingUtils{
  public:
    TimingUtils(){}
    ~TimingUtils(){}   

    static void tic();
    static void toc(std::string message = "");
    static std::string currentDateTime();

    static void sleepSeconds(clock_t sec);

  private:
    static std::stack<clock_t> tictoc_stack;    
};

#endif
