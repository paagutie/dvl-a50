#ifndef TCPSOCKET_H
#define TCPSOCKET_H

#include <stdio.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

#include <poll.h>
#include <time.h>
#include <thread>

#define UNUSED(expr) do { (void)(expr); } while (0)


class TCPSocket
{
public:

    TCPSocket(char *address, int port)
    {
        memset(&serv_addr, 0, sizeof(serv_addr));
        this->address = address;
        buffer_size = 1;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        serv_addr.sin_addr.s_addr = inet_addr(address);  /// server ip
    }
    
    int Create(void)
    {
        if ((this->sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            std::cout << "Socket creation error" << std::endl;
            return -1;
        }
        return 0;
    }
    

    //int Send(const char *bytes, size_t byteslength);

    int Connect(unsigned int timeout_ms, std::string &error, int &error_code)
    {
        // Convert IPv4 and IPv6 addresses from text to binary form
        if(inet_pton(AF_INET, this->address, &serv_addr.sin_addr)<=0) 
	    {
            error = "Invalid address / Address not supported";
            return -1;
        }

        
        // Set non-blocking 
        if( (arg = fcntl(this->sock, F_GETFL, NULL)) < 0) {
           error =  std::string("Error fcntl(..., F_GETFL)") + "(" + strerror(errno) + ")";
           return -1;
        } 
       
        arg |= O_NONBLOCK; 
        if( fcntl(this->sock, F_SETFL, arg) < 0) {  
           error =  std::string("Error fcntl(..., F_SETFL)") + "(" + strerror(errno) + ")";
           return -1;
        } 
        
	
        if (connect(this->sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {	    
            // Trying to connect with timeout 
            if (errno == EINPROGRESS) { 
                //fprintf(stderr, "EINPROGRESS in connect() - selecting\n"); 
                do { 
                    tv = this->TimevalFromMsec(timeout_ms);
                    //tv.tv_sec = 5; 
                    //tv.tv_usec = 0; 
                    FD_ZERO(&myset); 
                    FD_SET(this->sock, &myset); 
                    res = select(this->sock+1, NULL, &myset, NULL, &tv); 
                    if (res < 0 && errno != EINTR) { 
                        //fprintf(stderr, "Error connecting %d - %s\n", errno, strerror(errno));
                        error = "Error connecting " + std::to_string(errno) + " - " + strerror(errno);
                        error_code = errno;
                        return -1; 
                    } 
                    else if (res > 0) { 
                        // Socket selected for write 
                        lon = sizeof(int); 
                        if (getsockopt(this->sock, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &lon) < 0) { 
                            error = "Error in getsockopt() " + std::to_string(errno) + " - " + strerror(errno); //error 114
                            error_code = errno;
                            return -1; 
                        } 
                        // Check the value returned... 
                        if (valopt) { 
                            error = "Error in delayed connection() " + std::to_string(valopt) + " - " + strerror(valopt);
                            error_code = valopt;
                            return -1; 
                        } 
                        break; 
                    } 
                   else { 
                        //fprintf(stderr, "Timeout in select() - Cancelling!\n");
                        error = "Timeout in select() - Cancelling!"; 
                        error_code = 100;
                        return -1; 
                    } 
                }while (1); 
            } 
            else { 
                error = "Error connecting " + std::to_string(errno) + " - " + strerror(errno);  //error 103
                error_code = errno;
                return -1; 
            } 
        } 
       
       // Set to blocking mode again... 
       if( (arg = fcntl(this->sock, F_GETFL, NULL)) < 0) { 
         error =  std::string("Error fcntl(..., F_GETFL)") + "(" + strerror(errno) + ")";
         return -1;  
       }
       arg &= (~O_NONBLOCK); 
       if( fcntl(this->sock, F_SETFL, arg) < 0) { 
         error =  std::string("Error fcntl(..., F_SETFL)") + "(" + strerror(errno) + ")";
         return -1; 
       }  
   
       return 0;
    }
   

    int Receive(char *tempBuffer)
    {
        return recv(this->sock, tempBuffer, buffer_size, 0);
    }

    void Send(char *tempBuffer)
    {
        send(this->sock, tempBuffer, strlen(tempBuffer), 0);
    }
    
    void Close(void)
    {
        close(this->sock);
    }

 
    bool SetRcvTimeout(unsigned int msec_timeout) {
        struct timeval t = this->TimevalFromMsec(msec_timeout);
        return this->SetRcvTimeout(t);
    }


private:
    int sock = 0;
    char *address;
    long arg;
    int res; 
    struct timeval tv; 
    socklen_t lon; 
    fd_set myset; 
    int valopt; 
    int buffer_size;
    //sockaddr_in address;
    struct sockaddr_in serv_addr;


    bool SetRcvTimeout(struct timeval timeout) {
        int iErr = setsockopt(this->sock, SOL_SOCKET, SO_RCVTIMEO, (char*) &timeout, sizeof(struct timeval));
        if (iErr < 0) {
            close(sock);
            return false;
        }
        return true;
    }

    struct timeval TimevalFromMsec(unsigned int time_msec){
       struct timeval t;

       t.tv_sec = time_msec / 1000;
       t.tv_usec = (time_msec % 1000) * 1000;

       return t;
    }

};

#endif
