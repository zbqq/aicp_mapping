#include "aicp_common_utils/timing.hpp"

std::stack<clock_t> TimingUtils::tictoc_stack;

// Get tim elapsed in seconds
void TimingUtils::tic() {
  tictoc_stack.push(clock());   
}

void TimingUtils::toc() {
  std::cout << "Time elapsed: "
  << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
  << " sec"
  << std::endl;
  tictoc_stack.pop();
}

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
std::string TimingUtils::currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

// clock_t is a like typedef unsigned int clock_t. Use clock_t instead of integer in this context
void TimingUtils::sleepSeconds(clock_t sec)
{
  clock_t start_time = clock();
  clock_t end_time = sec * 1000 + start_time;
  while(clock() != end_time)
    {std::cout << "Waiting...\n";}
}
