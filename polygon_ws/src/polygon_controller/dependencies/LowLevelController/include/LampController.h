#ifndef LC_H
#define LC_H

#include <string>

#include "UDPSocket.h"

using namespace std;

class LampController : public UDPSocket {
private:
  bool redCurrState;
  bool blueCurrState;
  bool greenCurrState;
  bool orangeCurrState;

public:
  LampController(const string &ip, const int port);
  bool init() { return sendMessage("r") == 0; }

  bool setRed(const bool state);
  bool setGreen(const bool state);
  bool setBlue(const bool state);
  bool setOrange(const bool state);
  bool set(const bool redState, const bool blueState, const bool greenState,
           const bool orangeState);

  LampController() = delete;
  LampController(const LampController &) = delete;
  LampController(LampController &&) = delete;
  LampController &operator=(const LampController &) = delete;
  LampController &operator=(LampController &&) = delete;

private:
  string createMsg(const bool redState, const bool blueState,
                   const bool greenState, const bool orangeState);
};

#endif
