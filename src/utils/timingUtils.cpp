#include "timingUtils.hpp"

std::stack<clock_t> TimingUtils::tictoc_stack;

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