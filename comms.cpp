#include "comms.h"
#include "Globals.h"


#include<stdio.h>
#include<winsock2.h>
 
#pragma comment(lib,"ws2_32.lib") //Winsock Library
 
#define SERVER "127.0.0.1"  //ip address of udp server
#define BUFLEN 512  //Max length of buffer
#define PORT 8888   //The port on which to listen for incoming data

int CreatePacket(char*, float, float, float, float);

void ClearScreen();

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
        //printf("Enter message : ");
        //gets_s(message);

			dwWaitResult = WaitForSingleObject(hMutex_comm, INFINITE);
			if (dwWaitResult == WAIT_OBJECT_0)
			{
				roll_cmd = g_roll_cmd;
				pitch_cmd = g_pitch_cmd;
				yaw_cmd = g_yaw_cmd;
				thrust_cmd = g_thrust_cmd;
				b_new_data = g_new_data;
				
				//Debug
				//roll_cmd = 5.5; pitch_cmd = 30.0; yaw_cmd = -80.0; thrust_cmd = 100.0;
			}
			ReleaseMutex(hMutex_comm);

		//If a new mocap frame was processed
        if (b_new_data == true)
		{

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

			printf("Sending Thrust: [%f]\n", thrust_cmd);
			printf("Sending roll: [%f]\n", roll_cmd);
			printf("Sending pitch: [%f]\n", pitch_cmd);
			printf("Sending yaw: [%f]\n", yaw_cmd);
			printf("##############################\n");

		}
         
        //receive a reply and print it
        //clear the buffer by filling null, it might have previously received data
        memset(buf,'\0', BUFLEN);
        //try to receive some data, this is a blocking call
        //if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) == SOCKET_ERROR)
        //{
        //    printf("recvfrom() failed with error code : %d" , WSAGetLastError());
        //    exit(EXIT_FAILURE);
        //}
         
        //puts(buf);
    }
 
    closesocket(s);
    WSACleanup();
}


//Constructs the packet containing the attitude and thrust commands. 
int CreatePacket(char* buffer, float roll,  float pitch,  float yaw,  float thrust)
{
	//All sizes are in bytes
	int header_size = 4;
	int data_size = 8;
	int crc_size = 1;
	int packet_size = header_size + data_size + crc_size;
	//Create header for packet identification
	((UINT8*)buffer)[0] = '$';
	((UINT8*)buffer)[1] = 'M';
	((UINT8*)buffer)[2] = '>';
	((UINT8*)buffer)[3] = data_size + crc_size;//Payload after header

	//Map roll, pitch, yaw values to positive integer range
	UINT16 ROLL = (roll*100.0f) + 1000.0f;
	UINT16 PITCH = (pitch*100.0f) + 1000.0f;
	UINT16 YAW = (yaw*100.0f) + 1000.0f;

	//Map thrust to integer
	UINT16 THRUST = thrust*10.0f;

	//Copy data into the send buffer
	((UINT16 *) (buffer+4))[0] = ROLL;
	((UINT16 *) (buffer+4))[1] = PITCH;
	((UINT16 *) (buffer+4))[2] = YAW; 
	((UINT16 *) (buffer+4))[3] = THRUST;

	//Compute XOR of all bytes to help determine packet integrity on receiver end
	UINT8 crc = 0;
	for (int i = 0; i < data_size; i++)
	{
		crc ^= buffer[header_size + i];
	}

	//Add crc to end of packet
	((UINT8*)buffer)[12] = crc;

	return packet_size;
}

void ClearScreen()
{
	HANDLE                     hStdOut;
	CONSOLE_SCREEN_BUFFER_INFO csbi;
	DWORD                      count;
	DWORD                      cellCount;
	COORD                      homeCoords = { 0, 0 };

	hStdOut = GetStdHandle( STD_OUTPUT_HANDLE );
	if (hStdOut == INVALID_HANDLE_VALUE) return;

	/* Get the number of cells in the current buffer */
	if (!GetConsoleScreenBufferInfo( hStdOut, &csbi )) return;
	cellCount = csbi.dwSize.X *csbi.dwSize.Y;

	/* Fill the entire buffer with spaces */
	if (!FillConsoleOutputCharacter(
		hStdOut,
		(TCHAR) ' ',
		cellCount,
		homeCoords,
		&count
		)) return;

	/* Fill the entire buffer with the current colors and attributes */
	if (!FillConsoleOutputAttribute(
		hStdOut,
		csbi.wAttributes,
		cellCount,
		homeCoords,
		&count
		)) return;

	/* Move the cursor home */
	SetConsoleCursorPosition( hStdOut, homeCoords );
}
