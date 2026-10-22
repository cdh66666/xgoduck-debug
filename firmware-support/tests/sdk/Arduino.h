#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>
#include <functional>
#include <utility>
extern unsigned long fakeUs;
inline unsigned long millis() { return fakeUs/1000; }
class HardwareSerial {
public:
  std::deque<unsigned char> rx;
  std::vector<unsigned char> tx;
  std::function<void(const std::vector<unsigned char>&)> onPacket;
  std::deque<unsigned char> future;
  unsigned long readyUs=0;
  unsigned flushCalls=0;
  unsigned long readCalls=0;
  bool endlessFf=false;
  unsigned long endlessBytes=0;
  std::deque<std::pair<unsigned long,unsigned char>> fragments;
  int read() {
    ++fakeUs;
    ++readCalls;
    while(!fragments.empty() && fragments.front().first<=fakeUs) {
      rx.push_back(fragments.front().second);fragments.pop_front();
    }
    if(fakeUs>=readyUs && !future.empty()) {
      rx.insert(rx.end(),future.begin(),future.end()); future.clear();
    }
    if(rx.empty()) { if(endlessFf){++endlessBytes;return 255;} return -1; }
    int value=rx.front(); rx.pop_front(); return value;
  }
  size_t write(const unsigned char *data,size_t count) {
    tx.insert(tx.end(),data,data+count);
    if(tx.size()>=4 && tx.size()==static_cast<size_t>(tx[3])+4) {
      if(onPacket) onPacket(tx);
      tx.clear();
    }
    return count;
  }
  void flush() { ++flushCalls; }
};
