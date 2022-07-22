#include "UDPSocket.h"

const string UDPSocket::DEF_ADDR = "192.168.1.10";
int gport = 8888;

UDPSocket::UDPSocket(const string &ip, const int port) : addr(), sockfd(0) {
  int _port = port; //!!
  string _ip = ip;

  if (port <= 0) {
    fprintf(stderr, "Invalid port. Using default: %d", DEF_PORT);
    _port = DEF_PORT;
  }
  if (!validateIpAddress(ip)) {
    fprintf(stderr, "Invalid addres. Using default: %s", DEF_ADDR.c_str());
    _ip = DEF_ADDR;
  }
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (0 > sockfd) {
    perror("Socket err\n");
    return;
  }
  addr.sin_family = AF_INET;
  addr.sin_port = htons(_port);
  addr.sin_addr.s_addr = inet_addr(_ip.c_str());

  struct sockaddr_in my_addr;
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(++gport); // short, network byte order
  my_addr.sin_addr.s_addr = inet_addr("192.168.1.50");
  memset(my_addr.sin_zero, '\0', sizeof my_addr.sin_zero);

  bind(sockfd, (struct sockaddr *)&my_addr, sizeof(my_addr));
}

int UDPSocket::sendMessage(const string &msg) {
  int err = sendto(sockfd, msg.c_str(), msg.length() + 1, 0,
                   (struct sockaddr *)&addr, sizeof(addr));
  if (err <= 0) {
    perror("Sometthing went wrong\n");
    perror(strerror(errno));
  }

  return errno;
}

bool UDPSocket::validateIpAddress(const string &ipAddress) {
  struct sockaddr_in sa;
  int result = inet_pton(AF_INET, ipAddress.c_str(), &(sa.sin_addr));
  return result != 0;
}