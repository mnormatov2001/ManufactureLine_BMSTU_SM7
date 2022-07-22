#ifndef PC_H
#define PC_H

#include <string>

#include "UDPSocket.h"

using namespace std;

class PalletizerController : public UDPSocket {
public:
  struct Position {
    int X;
    int Y;
    int Z;
    Position() : X(0), Y(0), Z(0) {}
    Position(const int X, const int Y, const int Z) : X(X), Y(Y), Z(Z) {}
    bool operator>(Position const &rhs) const {
      return X > rhs.X && Y > rhs.Y && Z > rhs.Z;
    }
    bool operator<(Position const &rhs) const {
      return X < rhs.X && Y < rhs.Y && Z < rhs.Z;
    }
  };
  struct Zone {
    Position min;
    Position max;
    bool minSetted;
    bool maxSetted;
    Zone() : min(), max(), minSetted(false), maxSetted(false) {}
    Zone(const Position &min, const Position &max)
        : min(min), max(max), minSetted(true), maxSetted(true) {}
    bool check(const Position &pos) { return pos > min && pos < max; }
    bool setted() { return minSetted && maxSetted; }
  };

private:
  Position prevPos;
  int pompPrevState;
  Zone zone;

public:
  PalletizerController(const string &ip, const int port);
  bool init() { return sendMessage("r") == 0; }
  
  bool moveTo(const int X, const int Y, const int Z);
  bool moveTo(const Position &pos);
  bool changePompState(const bool state);
  bool moveToAndChangePompState(const int X, const int Y, const int Z, const bool state);
  void setZone(const Position &min, const Position &max) {
    zone = Zone(min, max);
  }

  PalletizerController() = delete;
  PalletizerController(const PalletizerController &) = delete;
  PalletizerController(PalletizerController &&) = delete;
  PalletizerController &operator=(const PalletizerController &) = delete;
  PalletizerController &operator=(PalletizerController &&) = delete;

private:
  string createMsg(const Position &pos, const int state);
};

#endif