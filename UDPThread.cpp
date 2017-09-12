//#undef UNICODE

//#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <stdio.h>
//#include <string>
//#include <sstream>


#include "JSON.h"
//#include "json.hpp"



#pragma comment(lib,"ws2_32.lib") //Winsock Library
 
#define BUFLENxyz 512  //Max length of buffer
#define PORTxyz 8080   //The port on which to listen for incoming data


extern float g_input_x;
extern float g_input_y;
extern float g_input_z;

extern bool g_input_new_data;

JSONObject root;

//Thread Safety Variable (mutex)
HANDLE hMutex_UDPxyz;

int countPrintf_server = 0;

//Function is used in thread, creates a UDP Server Thread for xyz input data

void xyzUDPserver(void *dummy)
{

	//Mutex
	hMutex_UDPxyz = (HANDLE)dummy;

	SOCKET s;
    struct sockaddr_in server, si_other;
    int slen , recv_len;
    char bufxyz[BUFLENxyz];
	 
    WSADATA wsa;
 
    slen = sizeof(si_other) ;
     
    //Initialise winsock
    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        printf("Failed. Error Code : %d",WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    printf("Initialised.\n");
     
    //Create a socket
    if((s = socket(AF_INET , SOCK_DGRAM , 0 )) == INVALID_SOCKET)
    {
        printf("Could not create socket : %d" , WSAGetLastError());
    }
    printf("Socket created.\n");
     
    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( PORTxyz );
     
    //Bind
    if( bind(s ,(struct sockaddr *)&server , sizeof(server)) == SOCKET_ERROR)
    {
        printf("Bind failed with error code : %d" , WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    puts("Bind done");

	//Mutex
	DWORD dwWaitResult;
	float x, y, z;
	 
    //keep listening for data
    while(1)
    {
		
        //printf("Waiting for data...");
        fflush(stdout);
         
        //clear the buffer by filling null, it might have previously received data
        memset(bufxyz,'\0', BUFLENxyz);
         
        //try to receive some data, this is a blocking call
        if ((recv_len = recvfrom(s, bufxyz, BUFLENxyz, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR)
        {
            printf("recvfrom() failed with error code : %d" , WSAGetLastError());
            exit(EXIT_FAILURE);
        }
         
        //print details of the client/peer and the data received
        //printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        //printf("Data: %s\n" , bufxyz);

		//json j = buf _json;
		//received data here
		JSONValue *value = JSON::Parse(bufxyz);

		
		root = value->AsObject();

		DWORD dwWaitResult = WaitForSingleObject(hMutex_UDPxyz, INFINITE);
			if (dwWaitResult == WAIT_OBJECT_0)
			{
				g_input_x = root[L"x"]->AsNumber();
				g_input_y = root[L"y"]->AsNumber();
				g_input_z = root[L"z"]->AsNumber();

				g_input_new_data = true;
			}
			ReleaseMutex(hMutex_UDPxyz);

		
		
		
		if (countPrintf_server == 400){
		printf("X_ref_received = [%f]\nY_ref_received = [%f]\nZ_ref_received = [%f]\n", root[L"x"]->AsNumber(), root[L"y"]->AsNumber(), root[L"z"]->AsNumber());

			//printf("x: %f\n",g_input_x );
			//g_input_y = data.json(2);
			//g_input_z = data.json(3);
			countPrintf_server = 0;
		}
		countPrintf_server ++;

		delete value;

        //now reply the client with the same data
        //if (sendto(s, bufxyz, recv_len, 0, (struct sockaddr*) &si_other, slen) == SOCKET_ERROR)
       // {
        //    printf("sendto() failed with error code : %d" , WSAGetLastError());
       //     exit(EXIT_FAILURE);
       // }
    }
 
    closesocket(s);
    WSACleanup();

	//return 0;
			
}
