#include <iostream>
#include <stack>
#include <ctime>

class TimingUtils{
  public:
    TimingUtils(){}
    ~TimingUtils(){}   

    static void tic();
    static void toc();

  private:
    static std::stack<clock_t> tictoc_stack;    
};