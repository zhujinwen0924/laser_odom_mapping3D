#ifndef misc_h
#define misc_h
#include <chrono>
#include <iostream>
#include <string>
using namespace std;
class Timer
{
private:
  std::string name_;
  std::chrono::time_point<std::chrono::high_resolution_clock> timer_start_;
  std::chrono::time_point<std::chrono::high_resolution_clock> timer_stop_;
  std::chrono::duration<double> duration_;

public:
  Timer(std::string name) : name_(name)
  {
  }
  ~Timer() = default;
  void Start()
  {
    timer_start_ = std::chrono::high_resolution_clock::now();
  }
  void Stop(bool show, std::string unit = "ms")
  {
    timer_stop_ = std::chrono::high_resolution_clock::now();
    duration_ = timer_stop_ - timer_start_;
    if (show)
    {
      if (unit == "ms")
      {
        std::cout << name_ << " takes " << GetDuration<std::chrono::milliseconds>() << " ms" << std::endl;
      }
      else if (unit == "us")
      {
        std::cout << name_ << " takes " << GetDuration<std::chrono::microseconds>() << " us" << std::endl;
      }
      else if (unit == "ns")
      {
        std::cout << name_ << " takes " << GetDuration<std::chrono::nanoseconds>() << " ns" << std::endl;
      }
      else if (unit == "s")
      {
        std::cout << name_ << " takes " << GetDuration<std::chrono::seconds>() << " s" << std::endl;
      }
      else
      {
        std::cout << "timer unit error!" << std::endl;
      }
    }
  }
  template <class U>
  int GetDuration()
  {
    return std::chrono::duration_cast<U>(duration_).count();
  }
  int GetDuration(std::string unit = "ms")
  {
    if (unit == "ms")
    {
      return std::chrono::duration_cast<std::chrono::milliseconds>(duration_).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(duration_).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(duration_).count();
    }
    else if (unit == "s")
    {
      return std::chrono::duration_cast<std::chrono::seconds>(duration_).count();
    }
    else
    {
      std::cout << "timer unit error!" << std::endl;
      return 0;
    }
  }
};


#endif