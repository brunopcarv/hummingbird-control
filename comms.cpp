#include "comms.h"
#include "Globals.h"


#include<stdio.h>
#include<winsock2.h>
 
#pragma comment(lib,"ws2_32.lib") //Winsock Library
 
#define SERVER "127.0.0.1"  //ip address of udp server
#define BUFLEN 512  //Max length of buffer
#define PORT 8888   //The port on which to listen for incoming data

int CreatePacket(char*, float, float, float, float);

//Thread Safety Variable
HANDLE hMutex_comm;

extern float g_roll_cmd;
extern float g_pitch_cmd;
extern float g_yaw_cmd;
extern float g_thrust_cmd;

extern bool g_new_data;


void UDPClient(void *dummy)
{
	hMutex_comm = (HANDLE)dummy;

	struct sockaddr_in si_other;
    int s, slen=sizeof(si_other);
    char buf[BUFLEN];
    char message[BUFLEN];
    WSADATA wsa;
 
    //Initialise winsock
    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        printf("Failed. Error Code : %d",WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    printf("Initialised.\n");
     
    //create socket
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
    {
        printf("socket() failed with error code : %d" , WSAGetLastError());
        exit(EXIT_FAILURE);
    }
     
    //setup address structure
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);
     
	DWORD dwWaitResult;
	float roll_cmd, pitch_cmd, yaw_cmd, thrust_cmd;
	bool b_new_data = false;

    //start communication
    while(1)
    {
        printf("Enter message : ");
        gets_s(message);



		//If a new mocap frame was processed
        if (/*b_new_data == true*/ true)
		{
			//Shared memory should be accessed using the mutex below
			dwWaitResult = WaitForSingleObject(hMutex_comm, INFINITE);
			if (dwWaitResult == WAIT_OBJECT_0)
			{
				roll_cmd = g_roll_cmd;
				pitch_cmd = g_pitch_cmd;
				yaw_cmd = g_yaw_cmd;
				thrust_cmd = g_thrust_cmd;
				b_new_data = g_new_data;
				
				//Debug
				roll_cmd = 5.5; pitch_cmd = 30.0; yaw_cmd = -80.0; thrust_cmd = 100.0;
			}
			ReleaseMutex(hMutex_comm);

			int packet_size = CreatePacket(message, roll_cmd, pitch_cmd, yaw_cmd, thrust_cmd);

			//send the message
			if (sendto(s, message, packet_size , 0 , (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
			{
				printf("sendto() failed with error code : %d" , WSAGetLastError());
				exit(EXIT_FAILURE);
			}

			b_new_data = false;

			dwWaitResult = WaitForSingleObject(hMutex_comm, INFINITE);
			if (dwWaitResult == WAIT_OBJECT_0)
			{
				g_new_data = b_new_data;
			}
			ReleaseMutex(hMutex_comm);
		}
         
        //receive a reply and print it
        //clear the buffer by filling null, it might have previously received data
        memset(buf,'\0', BUFLEN);
        //try to receive some data, this is a blocking call
        if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) == SOCKET_ERROR)
        {
            printf("recvfrom() failed with error code : %d" , WSAGetLastError());
            exit(EXIT_FAILURE);
        }
         
        puts(buf);
    }
 
    closesocket(s);
    WSACleanup();
}


//Constructs the packet containing the attitude and thrust commands. 
int CreatePacket(char* buffer, float roll,  float pitch,  float yaw,  float thrust)
{
	int packet_size = 0;
	//Create header for packet identification
	((UINT8*)buffer)[0] = '$';
	((UINT8*)buffer)[1] = 'M';
	((UINT8*)buffer)[2] = '>';
	packet_size += 4;//4 bytes corresponding to header and data size

	//Map roll, pitch, yaw values to positive integer range
	UINT16 ROLL = (roll*100.0f) + 1000.0f;
	UINT16 PITCH = (pitch*100.0f) + 1000.0f;
	UINT16 YAW = (yaw*100.0f) + 1000.0f;

	//Map thrust to integer
	UINT16 THRUST = thrust*100.0f;

	//Copy data into the send buffer
	((UINT16 *) (buffer+4))[0] = ROLL;
	((UINT16 *) (buffer+4))[1] = PITCH;
	((UINT16 *) (buffer+4))[2] = YAW; 
	((UINT16 *) (buffer+4))[3] = THRUST;
	packet_size += 8;//8 bytes corresponding to the 4x2 byte data above

	((UINT8*)buffer)[3] = 8 + 1;//send the data size, current packet size + 1 for the crc value

	//Compute XOR of all bytes to help determine packet integrity on receiver end
	UINT8 crc = 0;
	for (int i = 0; i < (packet_size - 1); i++)
	{
		crc ^= buffer[i];
	}

	//Add crc to end of packet
	((UINT8*)buffer)[12] = crc;
	packet_size += 1; //1 byte corresponding to crc

	return packet_size;
}
