#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>
#include <pthread.h>

class TcpServer
{
public:
    TcpServer(int);
    void accept_link();
    void recv_msg(int num);
    void sendMessage(std::string);

public:
    //std::string rg_message;
    std::string ins; //instruction
    int flag;



private:

    static const int MAXSIZE = 1024;
    static const int THREAD_NUM = 10;
    int socket_fd;
    int accept_fd[THREAD_NUM];
    sockaddr_in myserver;
    sockaddr_in remote_addr;
    int thread_count;
    int whichisrg;
    pthread_t thr;
    pthread_t tid[THREAD_NUM];

};

// used to transmit params to pthread
struct S
{
    TcpServer *server;
    int num;
};

void* run1(void*);
void* run2(void*);
//static void* accept_link(void*);
//static void* recv_msg(void*);

#endif // TCPSERVER_H
