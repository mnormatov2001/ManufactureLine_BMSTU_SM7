#ifndef AMC_H
#define AMC_H

#include <string>

#include "UDPSocket.h"

using namespace std;

class AngleManipulatorController : public UDPSocket {
public:
  struct Position {
    int X;
    int Y;
    int Z;
    int A;
    Position() : X(0), Y(0), Z(0), A(0) {}
    Position(const int X, const int Y, const int Z, const int A)
        : X(X), Y(Y), Z(Z), A(A) {}
    bool operator>(Position const &rhs) const {
      return X > rhs.X && Y > rhs.Y && Z > rhs.Z && A > rhs.A;
    }
    bool operator<(Position const &rhs) const {
      return X < rhs.X && Y < rhs.Y && Z < rhs.Z && A < rhs.A;
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
  AngleManipulatorController(const string &ip, const int port);
  bool init() { return sendMessage("r") == 0; }

  bool moveTo(const int X, const int Y, const int Z, const int angle);
  bool moveTo(const Position &pos);
  bool changePompState(const bool state);
  bool moveToAndChangePompState(const int X, const int Y, const int Z, const int angle,
                   const bool state);
  void setZone(const Position &min, const Position &max) {
    zone = Zone(min, max);
  }

  AngleManipulatorController() = delete;
  AngleManipulatorController(const AngleManipulatorController &) = delete;
  AngleManipulatorController(AngleManipulatorController &&) = delete;
  AngleManipulatorController &operator=(const AngleManipulatorController &) = delete;
  AngleManipulatorController &operator=(AngleManipulatorController &&) = delete;

private:
  string createMsg(const Position &pos, const int state);
};

#endif