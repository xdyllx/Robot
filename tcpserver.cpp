#include "tcpserver.h"
#include <stdio.h>
#include <unistd.h>
TcpServer::TcpServer(int listen_port)
{
    thread_count = 0;
    whichisrg = -1;
    flag = 0;


    if(( socket_fd = socket(PF_INET,SOCK_STREAM,IPPROTO_TCP)) < 0 ){
            throw "socket() failed";
    }

    memset(&myserver,0,sizeof(myserver));
    myserver.sin_family = AF_INET;
    myserver.sin_addr.s_addr = htonl(INADDR_ANY);
    myserver.sin_port = htons(listen_port);
    if( bind(socket_fd,(sockaddr*) &myserver,sizeof(myserver)) < 0 ) {
            throw "bind() failed";
    }

    if( listen(socket_fd,THREAD_NUM) < 0 ) {
            throw "listen() failed";
    }

    pthread_create(&thr,NULL,run1,this);
    pthread_detach(thr);
}

void *run1(void* args)
{
    TcpServer *t = static_cast<TcpServer *>(args);
    t->acceptLink();
}

void *run2(void* args)
{
    S *t = static_cast<S *>(args);
    t->server->recvMessage(t->num);
}

void TcpServer::sendMessage(std::string response)
{
    if(whichisrg == -1)
        return;
    write(accept_fd[whichisrg], response.c_str(), strlen(response.c_str()));
    std::cout << "send "<< response <<std::endl;
}


void TcpServer::acceptLink()
{
    socklen_t sin_size = sizeof(struct sockaddr_in);
    while(1){
        if(( accept_fd[thread_count] = accept(socket_fd,(struct sockaddr*) &remote_addr,&sin_size)) == -1 )
        {
                throw "Accept error!";
        }
        printf("Received a connection from %s\n",(char*) inet_ntoa(remote_addr.sin_addr));
        S *tmp = new S();
        tmp->server = this;
        tmp->num = thread_count;
        pthread_create(&tid[thread_count], NULL, run2, (void*)tmp);
        pthread_detach(tid[thread_count]);
        ++thread_count;
    }
}

void TcpServer::recvMessage(int num)
{
    std::cout << "num=" <<num << std::endl;
    while(1)
    {
        int len;
        char buffer[TcpServer::MAXSIZE];
        memset(buffer,0,MAXSIZE);
        if( ( len = recv(accept_fd[num],buffer,MAXSIZE,0)) < 0 ) {
                throw("Read() error!");
        }
        else if(len>0)
        {
//            if(memcmp(buffer,"00turnggg",9) == 0)
//            {
//                std::cout << "rgnum=" <<std::endl;
//                whichisrg = num;
//            }
//            if(num == whichisrg)
//            {
//                rg_message = "";
//                for(int i=2;i<len;i++)
//                    rg_message += buffer[i];
//                std::cout << "receive rg_message:"<<rg_message << std::endl;
//                if(rg_message == "turnrrr")
//                {
//                    flag = 1;
//                    ins = "stop";
//                }
//                rgflag = true;
//                for(int i=0;i<3;i++)
//                {
//                    if(rg_message[i+4] == 'r')
//                        rg[i] = true;
//                    else if(rg_message[i+4] == 'g')
//                        rg[i] = false;
//                    else
//                    {
//                        std::cout << "shit error,reveive "<<rg_message << std::endl;
//                    }
//                }
//            }
//            else
            {
                flag = 1;
                ins = "";
                for(int i=2;i<len;i++) //安卓客户端发送的消息前两位为消息长度，去掉
                    ins += buffer[i];
                std::cout << "receive control message:" << ins << std::endl;
            }

        }
    }
}
