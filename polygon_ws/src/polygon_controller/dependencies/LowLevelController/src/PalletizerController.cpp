#include "PalletizerController.h"

PalletizerController::PalletizerController(const string &ip, const int port)
    : UDPSocket(ip, port), prevPos(), pompPrevState(0) {}

string PalletizerController::createMsg(const Position &pos, const int state) {

  string msg = "p:" + std::to_string(pos.X) + ":" + std::to_string(pos.Y) +
               ":" + std::to_string(pos.Z) + ":" + std::to_string(state) + "#";

  return msg;
}

bool PalletizerController::moveTo(const Position &pos) {
  if (!zone.setted()) {
    fprintf(stderr, "%s", "Set zone at first!\n");
    return false;
  }

  if (!zone.check(pos)) {
    fprintf(stderr, "%s", "Out Of Zone\n");
    return false;
  }

  string msg = createMsg(pos, pompPrevState);
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Palletizer moveTo error while sending\n");
    return false;
  }
  prevPos = pos;

  return true;
}

bool PalletizerController::moveTo(const int X, const int Y, const int Z) {
  Position pos(X, Y, Z);
  return moveTo(pos);
}

bool PalletizerController::changePompState(const bool state) {
  string msg = createMsg(prevPos, state);
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Palletizer changePompState error while sending\n");
    return false;
  }
  pompPrevState = state;

  return true;
}

bool PalletizerController::moveToAndChangePompState(const int X, const int Y, const int Z,
                                       const bool state) {
  if (!zone.setted()) {
    fprintf(stderr, "%s", "Set zone at first!\n");
    return false;
  }

  Position newPos(X, Y, Z);

  if (!zone.check(newPos)) {
    fprintf(stderr, "%s", "Out Of Zone\n");
    return false;
  }

  string msg = createMsg(newPos, state);

  cout << msg << endl;
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Palletizer set error while sending\n");
    return false;
  }
  prevPos = newPos;
  pompPrevState = state;

  return true;
}