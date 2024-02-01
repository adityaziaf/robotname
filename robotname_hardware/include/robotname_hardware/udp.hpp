#ifndef _UDP_HPP_INCLUDED_
#define _UDP_HPP_INCLUDED_

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>	
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>

class UDP {
private:
    int socketID;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    std::string ip_server;
    std::string ip_client;
    uint16_t port;
    uint8_t tx_buff[1024];
public:
    uint8_t rx_buff[1024];
    UDP(std::string ip, uint16_t _port) : ip_server(ip), port(_port) {}
    int init();
    int setClient(std::string ip, uint16_t port);
    int send(uint8_t* buff, size_t l);
    int receive();
    virtual ~UDP();
};

#endif