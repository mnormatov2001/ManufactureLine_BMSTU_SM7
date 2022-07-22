#include "AngleManipulatorController.h"

AngleManipulatorController::AngleManipulatorController(const string &ip,
                                                       const int port)
    : UDPSocket(ip, port), prevPos(), pompPrevState(0) {}

string AngleManipulatorController::createMsg(const Position &pos,
                                             const int state) {

  string msg = "g:" + std::to_string(pos.X) + ":" + std::to_string(pos.Y) +
               ":" + std::to_string(pos.A) + ":" + std::to_string(pos.Z) + ":" +
               std::to_string(state) + "#";

  return msg;
}

bool AngleManipulatorController::moveTo(const int X, const int Y, const int Z,
                                        const int angle) {
  Position pos(X, Y, Z, angle);
  return moveTo(pos);
}

bool AngleManipulatorController::moveTo(const Position &newPos) {

  if (!zone.setted()) {
    fprintf(stderr, "%s", "Set zone at first!\n");
    return false;
  }

  if (!zone.check(newPos)) {
    fprintf(stderr, "%s", "Out Of Zone\n");
    return false;
  }

  string msg = createMsg(newPos, pompPrevState);
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Angle Manipulator  moveTo error while sending\n");
    return false;
  }
  prevPos = newPos;

  return true;
}

bool AngleManipulatorController::changePompState(const bool state) {
  string msg = createMsg(prevPos, state);
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Angle Manipulator  changePompState error while sending\n");
    return false;
  }
  pompPrevState = state;

  return true;
}

bool AngleManipulatorController::moveToAndChangePompState(const int X, const int Y,
                                             const int Z, const int angle,
                                             const bool state) {
  if (!zone.setted()) {
    fprintf(stderr, "%s", "Set zone at first!\n");
    return false;
  }

  Position newPos(X, Y, Z, angle);

  if (!zone.check(newPos)) {
    fprintf(stderr, "%s", "Out Of Zone\n");
    return false;
  }

  string msg = createMsg(newPos, state);

  cout << msg << endl;
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Angle Manipulator  set error while sending\n");
    return false;
  }
  prevPos = newPos;
  pompPrevState = state;

  return true;
}