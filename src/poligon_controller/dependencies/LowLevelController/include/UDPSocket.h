#ifndef UDPS_H
#define UDPS_H

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <netdb.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

using namespace std;

class UDPSocket {
private:
  struct sockaddr_in addr;
  int sockfd;
  static const int DEF_PORT = 8888;
  static const string DEF_ADDR;

public:
  UDPSocket(const string &ip, const int port);
  ~UDPSocket() { close(sockfd); }

  int sendMessage(const string &msg);

  UDPSocket() = delete;
  UDPSocket(const UDPSocket &) = delete;
  UDPSocket(UDPSocket &&) = delete;
  UDPSocket &operator=(const UDPSocket &) = delete;
  UDPSocket &operator=(UDPSocket &&) = delete;

private:
  bool validateIpAddress(const string &ipAddress);
};

#endif
