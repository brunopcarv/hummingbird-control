//=============================================================================
// Copyright © 2014 NaturalPoint, Inc. All Rights Reserved.
// 
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall NaturalPoint, Inc. or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//=============================================================================


/*

SampleClient.cpp

This program connects to a NatNet server, receives a data stream, and writes that data stream
to an ascii file.  The purpose is to illustrate using the NatNetClient class.

Usage [optional]:

	SampleClient [ServerIP] [LocalIP] [OutputFilename]

	[ServerIP]			IP address of the server (e.g. 192.168.0.107) ( defaults to local machine)
	[OutputFilename]	Name of points file (pts) to write out.  defaults to Client-output.pts

*/

#include <stdio.h>
#include <winsock2.h>
#include <windows.h>
//#include <time.h> 
//#include <fstream>
#include <math.h> 
#include <tchar.h>
#include <conio.h>
//#include <winsock2.h>
#include <process.h> //multi-threading
#include "comms.h"
#include "Globals.h"
//#include "UDPTHREAD.h"

#include "NatNetTypes.h"
#include "NatNetClient.h"

#pragma comment(lib, "winmm.lib")
#pragma warning( disable : 4996 )
#define M_PI 3.14159265358979323846

void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs);
void _WriteFrame(FILE* fp, sFrameOfMocapData* data);
void _WriteFooter(FILE* fp);
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
void __cdecl MessageHandler(int msgType, char* msg);		            // receives NatNet error mesages
void resetClient();
int CreateClient(int iConnectionType);

void GetEulers(double qx, double qy, double qz, double qw, double *angle1,double *angle2, double *angle3); // Quaternions to Euler Angle conversion
float PID(float error, float _kp, float _ki, float _kd, double *_last_t, float *_last_error, float *_integrator, float *_last_derivative);
void Position_Control(float xd, float yd, float zd, float xc, float yc, float zc, float yawd, float yaw, float roll, float pitch, float *roll_sp, float *pitch_sp, float *yaw_sp, float *d_hov_speed, int nBody);
unsigned int MyServersDataPort = 3130;
unsigned int MyServersCommandPort = 3131;
int iConnectionType = ConnectionType_Multicast;
//int iConnectionType = ConnectionType_Unicast;

//Global commands (outputs)
extern float g_roll_cmd;
extern float g_pitch_cmd;
extern float g_yaw_cmd;
extern float g_thrust_cmd;

extern bool g_new_data;

//Global waypoints (inputs)
extern float g_input_x;
extern float g_input_y;
extern float g_input_z;

extern bool g_input_new_data;

//Global flags
bool g_user_control = false;

//Thread Safety Variable
HANDLE hcommsMutex;
HANDLE huserMutex;

NatNetClient* theClient;
FILE* fp;

char szMyIPAddress[128] = "";
char szServerIPAddress[128] = "";

int analogSamplesPerMocapFrame = 0;

int countPrintf = 0;
void ClearScreen2();

// -- PID Control
const float fCut = 90;//30; //[Hz] Cut off frequency for the low pass filter
const float g = 9.81f;
const float _imax = 3; //Used in the PID controller for saturating the integrator component

//==================================PID Variables=========================================
float _last_error_x[2]={0,0};
float _integrator_x[2]={0,0};
float _last_derivative_x[2]={0,0};
double _last_t_x[2]={0,0};

float _last_error_y[2]={0,0};
float _integrator_y[2]={0,0};
float _last_derivative_y[2]={0,0};
double _last_t_y[2]={0,0};

float _last_error_z[2]={0,0};
float _integrator_z[2]={0,0};
float _last_derivative_z[2]={0,0};
double _last_t_z[2]={0,0};

float _last_error_yaw[2]={0,0};
float _integrator_yaw[2]={0,0};
float _last_derivative_yaw[2]={0,0};
double _last_t_yaw[2]={0,0};

// PID Gains for X,Y,Z (roll, pitch and yaw are onboard accessed through baseflight software)
float kp_x=0.6,ki_x=0.1,kd_x=1.6;
float kp_y=0.6,ki_y=0.1,kd_y=1.6;
float kp_z=12,ki_z=0.2,kd_z=8;
float kp_yaw=2.1,ki_yaw=0,kd_yaw=0.66;
//===========================================================================================

// Desired Reference
float x_ref=0.0, y_ref=0.0, z_ref=0.5, yaw_ref=0.0; //
float roll_ref=0.0, pitch_ref=0.0;  //


// HUMMINBIRD CONSTANTS
const double m = 0.70;

//Record data in a CSV file
FILE *Record; // File for recording data

double  start_time; //program start time


//Maneuvers
double  start_maneuver_time = 0;
double  maneuver_time = 0;
int show_maneuver_num = 10;

int _tmain(int argc, _TCHAR* argv[])
{
	start_time = timeGetTime();
	// -- Prepare the USB port to send data
	// Open the highest available serial port number 
	Record = fopen("Record.csv","w"); // File for recording data
	//fprintf(stderr, "Opening serial port...");
	//Record.csv column headers
	fprintf(Record, "PC Time, Mocap_X, Mocap_y, Mocap_z, Mocap_yaw, Mocap_pitch, Mocap_roll, X_ref, Y_ref, Z_ref \n"); 


    hcommsMutex = CreateMutex(NULL,FALSE,NULL);
	if (hcommsMutex == NULL)
	{
		printf("CreateMutex Error: %d\n",GetLastError());
		return 1;
	}

	huserMutex = CreateMutex(NULL,FALSE,NULL);
	if (huserMutex == NULL)
	{
		printf("CreateMutex Error: %d\n",GetLastError());
		return 1;
	}

    int iResult;
     
    // parse command line args
    if(argc>1)
    {
        strcpy(szServerIPAddress, argv[1]);	// specified on command line
        printf("Connecting to server at %s...\n", szServerIPAddress);
    }
    else
    {
        strcpy(szServerIPAddress, "");		// not specified - assume server is local machine
        printf("Connecting to server at LocalMachine\n");
    }
    if(argc>2)
    {
        strcpy(szMyIPAddress, argv[2]);	    // specified on command line
        printf("Connecting from %s...\n", szMyIPAddress);
    }
    else
    {
        strcpy(szMyIPAddress, "");          // not specified - assume server is local machine
        printf("Connecting from LocalMachine...\n");
    }

    // Create NatNet Client
    iResult = CreateClient(iConnectionType);
    if(iResult != ErrorCode_OK)
    {
        printf("Error initializing client.  See log for details.  Exiting");
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }


	// send/receive test request
	printf("[SampleClient] Sending Test Request\n");
	void* response;
	int nBytes;
	iResult = theClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	// Retrieve Data Descriptions from server
	printf("\n\n[SampleClient] Requesting Data Descriptions...");
	sDataDescriptions* pDataDefs = NULL;
	int nBodies = theClient->GetDataDescriptions(&pDataDefs);
	if(!pDataDefs)
	{
		printf("[SampleClient] Unable to retrieve Data Descriptions.");
	}
	else
	{
        printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet
                sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                printf("MarkerSet Name : %s\n", pMS->szName);
                for(int i=0; i < pMS->nMarkers; i++)
                    printf("%s\n", pMS->szMarkerNames[i]);

            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                printf("RigidBody Name : %s\n", pRB->szName);
                printf("RigidBody ID : %d\n", pRB->ID);
                printf("RigidBody Parent ID : %d\n", pRB->parentID);
                printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
            {
                // Skeleton
                sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
                printf("Skeleton Name : %s\n", pSK->szName);
                printf("Skeleton ID : %d\n", pSK->skeletonID);
                printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
                for(int j=0; j < pSK->nRigidBodies; j++)
                {
                    sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
                    printf("  RigidBody Name : %s\n", pRB->szName);
                    printf("  RigidBody ID : %d\n", pRB->ID);
                    printf("  RigidBody Parent ID : %d\n", pRB->parentID);
                    printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
                }
            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
            {
                // Force Plate
                sForcePlateDescription* pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
                printf("Force Plate ID : %d\n", pFP->ID);
                printf("Force Plate Serial : %s\n", pFP->strSerialNo);
                printf("Force Plate Width : %3.2f\n", pFP->fWidth);
                printf("Force Plate Length : %3.2f\n", pFP->fLength);
                printf("Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX,pFP->fOriginY, pFP->fOriginZ);
                for(int iCorner=0; iCorner<4; iCorner++)
                    printf("Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0],pFP->fCorners[iCorner][1],pFP->fCorners[iCorner][2]);
                printf("Force Plate Type : %d\n", pFP->iPlateType);
                printf("Force Plate Data Type : %d\n", pFP->iChannelDataType);
                printf("Force Plate Channel Count : %d\n", pFP->nChannels);
                for(int iChannel=0; iChannel<pFP->nChannels; iChannel++)
                    printf("\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
            }
            else
            {
                printf("Unknown data type.");
                // Unknown
            }
        }      
	}

	
	// Create data file for writing received stream into
	char szFile[MAX_PATH];
	char szFolder[MAX_PATH];
	GetCurrentDirectory(MAX_PATH, szFolder);
	if(argc > 3)
		sprintf(szFile, "%s\\%s", szFolder, argv[3]);
	else
		sprintf(szFile, "%s\\Client-output.pts",szFolder);
	fp = fopen(szFile, "w");
	if(!fp)
	{
		printf("error opening output file %s.  Exiting.", szFile);
		exit(1);
	}
	if(pDataDefs)
		_WriteHeader(fp, pDataDefs);


	//Create UDP Client Thread for ACI Communication
	HANDLE h_UDPClientThread = (HANDLE)_beginthread(UDPClient, 0, (void*)hcommsMutex);

	//Create UDP Server Thread for xyz input data
	//HANDLE h_xyzUDPserverThread = (HANDLE)_beginthread(xyzUDPserver, 0, (void*)hcommsMutex);

	// Ready to receive marker stream!
	printf("\nClient is connected to server and listening for data...\n");

	int c;
	bool bExit = false;
	DWORD dwWaitResult;

	while(c =_getch())
	{
			switch(c)
			{
				case 'x':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						g_thrust_cmd += 0.1f;
						if (g_thrust_cmd > 6)
							g_thrust_cmd = 6;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 'z':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 10;
						g_thrust_cmd -= 0.1f;
						if (g_thrust_cmd < 0)
							g_thrust_cmd = 0;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 's':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						g_roll_cmd += 0.5;
						if (g_roll_cmd > 52)
							g_roll_cmd = 52;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 'a':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 10;
						g_roll_cmd -= 0.5;
						if (g_roll_cmd < -52)
							g_roll_cmd = -52;
					}
					ReleaseMutex(hcommsMutex);	
					break;
				case 'w':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						g_pitch_cmd += 0.5;
						if (g_pitch_cmd > 52)
							g_pitch_cmd = 52;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 'q':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 10;
						g_pitch_cmd -= 0.5;
						if (g_pitch_cmd < -52)
							g_pitch_cmd = -52;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case '2':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						/*if (g_yaw_cmd < -1999)
							g_yaw_cmd += 1;
						else
							g_yaw_cmd += 100;
						*/
						g_yaw_cmd += 1;
						if (g_yaw_cmd > 100)
							g_yaw_cmd = 100;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case '1':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 10;
						//if (g_yaw_cmd < -1999)
						//	g_yaw_cmd -= 1;
						//else
						//	g_yaw_cmd -= 100;
						g_yaw_cmd -= 1;
						if (g_yaw_cmd < -100)
							g_yaw_cmd = -100;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 'v':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						z_ref += 0.05;
						//if (z_ref > 1)
							//z_ref = 1;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 'c':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						z_ref -= 0.05;
						if (z_ref < 0)
							z_ref = 0;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 'f':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						x_ref += 0.05;
						if (x_ref > 1)
							x_ref = 1;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 'd':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						x_ref -= 0.05;
						if (x_ref < -1)
							x_ref = -1;
					}
					ReleaseMutex(hcommsMutex);	
					break;	

				case 'r':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						y_ref += 0.05;
						if (y_ref > 1)
							y_ref = 1;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 'e':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						y_ref -= 0.05;
						if (y_ref < -1)
							y_ref = -1;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				
				case '4':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						yaw_ref += 5;
						if (yaw_ref > 180)
							yaw_ref = 180;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case '3':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
						show_maneuver_num = 10;
						yaw_ref -= 5;
						if (yaw_ref < -100)
							yaw_ref = -180;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
					
				case 'i':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 10;
						g_thrust_cmd = 0;
						g_roll_cmd = 0;
						g_pitch_cmd = 0;
						g_yaw_cmd = -2047;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 'u':
					dwWaitResult = WaitForSingleObject(huserMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						g_user_control = !g_user_control;
						show_maneuver_num = 10;
					}
					ReleaseMutex(huserMutex);	
					break;
				case 'l':
					dwWaitResult = WaitForSingleObject(huserMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 0; //forced origin landing		
					}
					ReleaseMutex(huserMutex);	
					break;
				case 'o':
					dwWaitResult = WaitForSingleObject(huserMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						maneuver_time = 0;
						start_maneuver_time = timeGetTime();
						show_maneuver_num = 1; //circle		
					}
					ReleaseMutex(huserMutex);	
					break;
				case 'p':
					dwWaitResult = WaitForSingleObject(huserMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 2; //wand height control	
					}
					ReleaseMutex(huserMutex);	
					break;
				case '[':
					dwWaitResult = WaitForSingleObject(huserMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 7; //wand 2 DOF control
					}
					ReleaseMutex(huserMutex);	
					break;
				case ']':
					dwWaitResult = WaitForSingleObject(huserMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 8; //wand 3 DOF contcccvuqrol	
					}
					ReleaseMutex(huserMutex);	
					break;

				case ';':
					dwWaitResult = WaitForSingleObject(huserMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 3; //landing	
					}
					ReleaseMutex(huserMutex);	
					break;
				case ',':
					dwWaitResult = WaitForSingleObject(huserMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						maneuver_time = 0;
						start_maneuver_time = timeGetTime();
						show_maneuver_num = 4; //lissajous curve	
					}
					ReleaseMutex(huserMutex);	
					break;
				case '.':
					dwWaitResult = WaitForSingleObject(huserMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						maneuver_time = 0;
						start_maneuver_time = timeGetTime();
						show_maneuver_num = 5; //lissajous curve	
					}
					ReleaseMutex(huserMutex);	
					break;
				case '6':
					dwWaitResult = WaitForSingleObject(huserMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						show_maneuver_num = 6; //PID tunning
					}
					ReleaseMutex(huserMutex);	
					break;
				case 'b':
					bExit = true;		
					break;	
				default:
					break;

			}
		
		if(bExit)
			break;

	}

	// Done - clean up.
	theClient->Uninitialize();
	_WriteFooter(fp);
	fclose(fp);

	fclose(Record);

	return ErrorCode_OK;
}

// Establish a NatNet Client connection
int CreateClient(int iConnectionType)
{
    // release previous server
    if(theClient)
    {
        theClient->Uninitialize();
        delete theClient;
    }

    // create NatNet client
    theClient = new NatNetClient(iConnectionType);



    // set the callback handlers
    theClient->SetVerbosityLevel(Verbosity_Warning);
    theClient->SetMessageCallback(MessageHandler);
    theClient->SetDataCallback( DataHandler, theClient );	// this function will receive data from the server
    // [optional] use old multicast group
    //theClient->SetMulticastAddress("224.0.0.1");

    // print version info
    unsigned char ver[4];
    theClient->NatNetVersion(ver);
    printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

    // Init Client and connect to NatNet server
    // to use NatNet default port assignments
    int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress);
    // to use a different port for commands and/or data:
    //int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // get # of analog samples per mocap frame of data
        void* pResult;
        int ret = 0;
        int nBytes = 0;
        ret = theClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d", analogSamplesPerMocapFrame);
        }

        // print server info
        sServerDescription ServerDescription;
        memset(&ServerDescription, 0, sizeof(ServerDescription));
        theClient->GetServerDescription(&ServerDescription);
        if(!ServerDescription.HostPresent)
        {
            printf("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        printf("[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
            ServerDescription.HostAppVersion[1],ServerDescription.HostAppVersion[2],ServerDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
            ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", szMyIPAddress);
        printf("Server IP:%s\n", szServerIPAddress);
        printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
    }

    return ErrorCode_OK;

}

// DataHandler receives data from the server
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	//Loops for each Rigidbody
	double yaw=0.0;// in degrees
	double pitch;// in degrees
	double roll;// in degrees


	NatNetClient* pClient = (NatNetClient*) pUserData;
	//float roll_cmd = 5.5, pitch_cmd = 30.0, yaw_cmd = -80.0, thrust_cmd = 1000.0;
	float roll_cmd = 0.0, pitch_cmd = 0.0, yaw_cmd = 0.0, thrust_cmd = 0.0;
	bool bUserControl = false;

	DWORD dwWaitResultUserCtrl = WaitForSingleObject(huserMutex, INFINITE);
	if (dwWaitResultUserCtrl == WAIT_OBJECT_0)
	{	
		bUserControl = g_user_control;
	}
	ReleaseMutex(huserMutex);	

	if(fp)
		_WriteFrame(fp,data);
	
    int i=0;


	// Rigid Bodies
	//printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
	for(i=0; i < data->nRigidBodies; i++)
	{
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;

		/*printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
		printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
			data->RigidBodies[i].x,
			data->RigidBodies[i].y,
			data->RigidBodies[i].z,
			data->RigidBodies[i].qx,
			data->RigidBodies[i].qy,
			data->RigidBodies[i].qz,
			data->RigidBodies[i].qw);*/
	}


	// Change global reference frame x = -x
	data->RigidBodies[0].x = data->RigidBodies[0].x*-1; // x = -x because the frame of reference for cameras is different from the frame used in the controller

	// Calculate Euler Angles from quaternions
	GetEulers(data->RigidBodies[0].qx,data->RigidBodies[0].qy, data->RigidBodies[0].qz, data->RigidBodies[0].qw, &yaw, &pitch, &roll);
	
	// Correcting the reference frame
	yaw = -yaw;

	//Tme
	double  time = 1.0*(timeGetTime()-start_time)/1000.0;// [s]

	if (countPrintf == 60)
	{
	ClearScreen2();
	printf("RIGID BODY POSITION\n");
	printf("X [%f]\n", data->RigidBodies[0].x);
	printf("Y [%f]\n", -data->RigidBodies[0].z);
	printf("Z [%f]\n", data->RigidBodies[0].y);
	//printf("qx [%f]\n", data->RigidBodies[0].qx);
	//printf("qy [%f]\n", data->RigidBodies[0].qy);
	//printf("qz [%f]\n", data->RigidBodies[0].qz);
	//printf("qw [%f]\n", data->RigidBodies[0].qw);
	printf("ROLL [%f]  PITCH [%f] YAW [%f]\t\n", roll,pitch,yaw);
	printf("Z_REF: [%f]  X_REF [%f]  Y_REF [%f]  YAW_REF [%f]\n ", z_ref,x_ref,y_ref,yaw_ref);
	
	printf("##############################\n");
	countPrintf = 0;
	}
	countPrintf ++;

	// WAND as reference
	// for height
	//z_ref = data->RigidBodies[1].y;
	//for Y Axis
	//y_ref = -data->RigidBodies[1].z; 


	if (show_maneuver_num == 0) { //forced origin landing
		x_ref = 0;
		y_ref = 0;
		z_ref = 0;
		yaw_ref = 0;
						
	}
	else if (show_maneuver_num == 1) { //circle
		double  omega = 30; //[degrees/s]
		double  A = 0.3; //[degrees/s]
		double  B = 0.3; //[degrees/s]
		
		maneuver_time = 1.0*(timeGetTime()-start_maneuver_time)/1000.0;//[s]
		x_ref = A*cos(omega*maneuver_time*M_PI/180);
		y_ref = B*sin(omega*maneuver_time*M_PI/180);
	
	}
	else if (show_maneuver_num ==2) { //wand height control	
		z_ref = data->RigidBodies[1].y;
	}
	else if (show_maneuver_num ==7) { //wand height control	and lateral control
		z_ref = data->RigidBodies[1].y;
		y_ref = -data->RigidBodies[1].z; 
	}
	else if (show_maneuver_num ==8) { //wand height control, and lateral control, and longitudinal control
		z_ref = data->RigidBodies[1].y;
		y_ref = -data->RigidBodies[1].z; 
		x_ref = -data->RigidBodies[1].x-0.6; // 
	}


	else if (show_maneuver_num ==3) { //landing
		x_ref = data->RigidBodies[0].x;
		y_ref = -data->RigidBodies[0].z;
		z_ref = 0;
		yaw_ref = 0;
	}
	else if (show_maneuver_num ==4) { //lissajous curve
		double  A = 0.38; //[m]
		double  B = 0.38; //[m]
		double  a = 9; //[degrees/s ]
		double  b = 8; //[degrees/s]
		double  delta = 90; //[degrees]
		double  w = 11.9; //[degrees/s] period for a=3 b=4 and w =5 is 72seconds

		maneuver_time = 1.0*(timeGetTime()-start_maneuver_time)/1000.0;//[s]

		x_ref = A*sin((w*a*maneuver_time+delta)*M_PI/180);
		y_ref = B*sin(w*b*maneuver_time*M_PI/180);
	}
	else if (show_maneuver_num ==5) { //space filling curve (Hilbert curve)
		double  d = 1; //[time = distance in 10^1 cm]
		double  v = 0.12;
		double aux;
		maneuver_time = 1.0*(timeGetTime()-start_maneuver_time)/1000.0;//[s]
		double t;
		
		if (maneuver_time>=0 & maneuver_time<16*d) {
			t =  maneuver_time;
		}
		else if (maneuver_time>=16*d & maneuver_time<32*d) {
			t =  maneuver_time-16*d;
		}
		else if (maneuver_time>=32*d & maneuver_time<48*d) {
			t =  maneuver_time-32*d;
		}
		else if (maneuver_time>=48*d & maneuver_time<63*d) {
			t =  maneuver_time-48*d;
		}


		if (t>=0 & t<d) {
			y_ref = 0;
			x_ref = t;
		}
		else if (t>=d & t<2*d) {
			y_ref = t - d;
			x_ref = d;
		}
		else if (t>=2*d & t<3*d) {
			y_ref = d;
			x_ref = -t +3*d;
		}
		else if (t>=3*d & t<5*d) {
			y_ref = t - 2*d;
			x_ref = 0;
		}
		else if (t>=5*d & t<6*d) {
			y_ref = 3*d;
			x_ref = t - 5*d;
		}
		else if (t>=6*d & t<7*d) {
			y_ref = -t + 9*d;
			x_ref = d;
		}
		else if (t>=7*d & t<8*d) {
			y_ref = 2*d;
			x_ref = t - 6*d;
		}
		else if (t>=8*d & t<9*d) {
			y_ref = t - 6*d;
			x_ref = 2*d;
		}
		else if (t>=9*d & t<10*d) {
			y_ref = 3*d;
			x_ref = t - 7*d;
		}
		else if (t>=10*d & t<12*d) {
			y_ref = -t + 13*d;
			x_ref = 3*d;
		}
		else if (t>=12*d & t<13*d) {
			y_ref = d;
			x_ref = -t + 15*d;
		}
		else if (t>=13*d & t<14*d) {
			y_ref = -t + 14*d;
			x_ref = 2*d;
		} 
		else if (t>=14*d & t<16*d) {
			y_ref = 0;
			x_ref = t-12*d;
		}




		
		if (maneuver_time>=0 & maneuver_time<16*d) {
			x_ref = x_ref*v;
			y_ref = y_ref*v;
		}
		else if (maneuver_time>=16*d & maneuver_time<32*d) {
			aux = (y_ref+4*d)*v;
			y_ref = x_ref*v;
			x_ref = aux;
		}
		else if (maneuver_time>=32*d & maneuver_time<47*d) {
			aux = (y_ref+4*d)*v;
			y_ref = (x_ref+4*d)*v;
			x_ref = aux;
		}
		else if (maneuver_time>=47*d & maneuver_time<48*d) {
			x_ref = (-t + 19*d)*v;
			y_ref = (7*d)*v;
		}
		else if (maneuver_time>=48*d & maneuver_time<63*d) {
			x_ref = (-x_ref+3*d)*v;
			y_ref = (-y_ref+7*d)*v;
		}
		else if (maneuver_time>=63*d) {
			x_ref = 0;
			y_ref = 7*d*v;
		}

	}


	else if (show_maneuver_num == 6) { //PID Tunning
		x_ref = 0.5;
		y_ref = 0.5;
	}

	// Sefaty code to keep the refs always inside the safe region
	if (z_ref > 1.5) z_ref = 1.5;
	if (z_ref < 0) z_ref = 0;
	if (x_ref > 0.5) x_ref = 0.5;
	if (x_ref < -0.5) x_ref = -0.5;
	if (y_ref > 0.5) y_ref = 0.5;
	if (y_ref < -0.5) y_ref = 0.5;


	// CONTROLLER GOES HERE
	Position_Control(x_ref, y_ref, z_ref, data->RigidBodies[0].x, -data->RigidBodies[0].z, data->RigidBodies[0].y, yaw_ref, yaw, roll, pitch, &roll_cmd, &pitch_cmd, &yaw_cmd, &thrust_cmd, 0);

	fprintf(Record,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",time,data->RigidBodies[0].x,-data->RigidBodies[0].z,data->RigidBodies[0].y,yaw,pitch,roll,x_ref,y_ref,z_ref);
	fprintf(Record,"\n"); //new line

		//Shared memory should be accessed using the mutex below
		DWORD dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
		if (dwWaitResult == WAIT_OBJECT_0)
		{
			//printf("DATA_HANDLER(): Updating Globals\n");
			if (!bUserControl)
			{
				g_roll_cmd = roll_cmd;
				g_pitch_cmd = pitch_cmd;
				g_yaw_cmd = yaw_cmd;
				g_thrust_cmd = thrust_cmd;
				
				
			}
			g_new_data = true;
		}
		ReleaseMutex(hcommsMutex);
}


// Quaternion to Euler Angle conversion
void GetEulers(double qx, double qy, double qz, double qw, double *angle1,double *angle2, double *angle3)
{
	double &heading = *angle1; //yaw
	double &attitude = *angle2; //pitch
	double &bank = *angle3; //roll

	double test = qx*qy + qz*qw;
	if (test > 0.499)   // singularity at north pole
	{ 
		heading = (double) (180/M_PI)*2.0f * atan2(qx,qw);
		attitude = (double) (180/M_PI)*M_PI/2.0f;
		bank = 0;
	}
	else if (test < -0.499)  // singularity at south pole
	{  
		heading = (double) -(180/M_PI)*2.0f * atan2(qx,qw);
		attitude = (double)  -(180/M_PI)*M_PI/2.0f;
		bank = 0;
	}
	else
	{
		double sqx = qx*qx;
		double sqy = qy*qy;
		double sqz = qz*qz;
		heading = (double) (180/M_PI)*atan2((double)2.0*qy*qw-2.0*qx*qz , (double)1 - 2.0*sqy - 2.0*sqz); //yaw
		attitude = (double)(180/M_PI)*asin(2.0*test); //pitch
		bank = (double) (180/M_PI)*atan2((double)2.0*qx*qw-2.0*qy*qz , (double)1.0 - 2.0*sqx - 2.0*sqz); //roll
	}

	//heading = (180/M_PI)*(heading);
	//attitude = (180/M_PI)*(attitude);
	//bank = (180/M_PI)*(bank);
}
//float yaw(float qx,float qy, float qz, float qw)
//{
//  float aux;
//  float test = qx*qy + qz*qw;;
//  float num,den;
//  
//  if (test > 0.499)   // singularity at north pole
//	{ 
//		aux = 2.0f * atan2(qx,qw);
//	}
//  else if (test < -0.499)  // singularity at south pole
//	{ 
//		aux = 2.0f * atan2(qx,qw);
//	}
//  else
//	{
//		float sqx = qx*qx;
//		float sqy = qy*qy;
//		float sqz = qz*qz;
//		num = 2.0*qy*qw-2.0*qx*qz;
//		den = 1 - 2.0*sqy - 2.0*sqz;
//		aux = atan2(num, den); //yaw
//  }
//  return aux;
//}


void Position_Control(float xd, float yd, float zd, float xc, float yc, float zc, float yawd, float yaw, float roll, float pitch,float *roll_sp, float *pitch_sp, float *yaw_sp, float *d_hov_speed, int nBody){

	float error_x=0, error_y=0, error_z=0, error_yaw=0;
	float a_x=0, a_y=0, a_z=0; // Acceleration in each axis
	float yaw_rate=0;

	error_x = (xd-xc);
	error_y = (yd-yc);
	error_z = (zd-zc);
	error_yaw = (yawd-yaw);

	//printf("\nERROR x:%f y: %f z: %f yaw: %f\n ",error_x,error_y,error_z,error_yaw);

	a_x = PID(error_x,kp_x,ki_x,kd_x, &_last_t_x[nBody], &_last_error_x[nBody], &_integrator_x[nBody], &_last_derivative_x[nBody]);
	a_y = PID(error_y,kp_y,ki_y,kd_y, &_last_t_y[nBody], &_last_error_y[nBody], &_integrator_y[nBody], &_last_derivative_y[nBody]);
	a_z = PID(error_z,kp_z,ki_z,kd_z, &_last_t_z[nBody], &_last_error_z[nBody], &_integrator_z[nBody], &_last_derivative_z[nBody]);
	yaw_rate = PID(error_yaw,kp_yaw,ki_yaw,kd_yaw, &_last_t_yaw[nBody], &_last_error_yaw[nBody], &_integrator_yaw[nBody], &_last_derivative_yaw[nBody]);


	//printf("\nAfter PID Ax:%f ay: %f az: %f yaw: %f Sinyaw: %f Cosyaw: %f\n ",a_x,a_y,a_z,yaw, sin(yaw*M_PI/180), cos(yaw*M_PI/180));

	*roll_sp= -(float)(a_x*sin(yaw*M_PI/180)-a_y*cos(yaw*M_PI/180))/M_PI*180;
	*pitch_sp= (float)(-a_x*cos(yaw*M_PI/180)-a_y*sin(yaw*M_PI/180))/M_PI*180;
	//*roll_sp= (float)1/g*(a_x*sin(yaw)-a_y*cos(yaw))/M_PI*180;
	//*pitch_sp= (float)1/g*(a_x*cos(yaw)+a_y*sin(yaw))/M_PI*180;
	*yaw_sp= yaw_rate;
	/*if (abs((cos(roll)*cos(pitch)))<0.6){
	*d_hov_speed= (float)(a_z+g)*m;
	}
	else{
	*d_hov_speed= (float)(a_z+g)*m/(cos(roll)*cos(pitch));
	}
*/
	*d_hov_speed= (float)(a_z+g)*m;

	//*d_hov_speed= (int) (m/(8*KF*hov_speed))*a_z;

	//Spiri Values
	//*d_hov_speed= (int) (m_spiri/(8*KF_spiri*hov_speed))*a_z;

	return;
}
//
//void reset_PID(){
//	//_last_error_x=0;
//	//_integrator_x=0;
//	//_last_derivative_x=0;
//	//_last_t_x=0;
//
//	//_last_error_y=0;
//	//_integrator_y=0;
//	//_last_derivative_y=0;
//	//_last_t_y=0;
//
//	//_last_error_z=0;
//	//_integrator_z=0;
//	//_last_derivative_z=0;
//	//_last_t_z=0;
//
//	return;
//}
//
float PID(float error, float _kp, float _ki, float _kd, double *_last_t, float *_last_error, float *_integrator, float *_last_derivative)
{
	//printf("\n");

	float output = 0;
	double  tnow = timeGetTime();
	double dt = tnow - *_last_t;
	float delta_time;

	if(*_last_t == 0 || dt > 1000){
		dt=0;	// If this PID hasn't been used for a full second then zero
		// the integrator term. This prevents I buildup from a previous
		// flight mode causing a massive return before the integrator
		//	gets a chance to correct itself.
		* _integrator = 0;
	}

	*_last_t = tnow;
	delta_time= (float) dt/1000.0;

	//Compute proportional component
	output += error * _kp;

	//Compute derivative component if time has elapsed
	if (((_kd*_kd)>0)&&(dt>0)){
		float derivative = (error - *_last_error)/delta_time;

		//discrete low pass filter, cuts ou the
		//high frequency noise that can drive the controller crazy
		float RC = 1/(2*M_PI*fCut);

		derivative = (float) (*_last_derivative +   (derivative - *_last_derivative));

		//update state
		*_last_error = error;
		*_last_derivative = derivative;

		//add in derivative component
		derivative = derivative * _kd;
		output += derivative;
		//printf("derivative: %f ",derivative);
	}

	//Compute integral component if time has elapsed
	if (((_ki*_ki)>0)&&(dt>0)){
		//printf("integrator before: %f",*_integrator);
		*_integrator = *_integrator + (error * _ki) * delta_time;
		//printf("(error * _ki) * delta_time: %f ", (error * _ki) * delta_time);
		//printf("delta_time: %f error: %f _ki: %f ", delta_time,error,_ki);
		if(*_integrator < -_imax){
			*_integrator = -_imax;
		} else if(*_integrator >_imax){
			*_integrator = _imax;
		}

		output += *_integrator;

		//	printf("inegrator: %f ",*_integrator);
	}

	//printf("proportional %f output: %f \n ",error * _kp, output);

	return output;
}



void ClearScreen2()
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


// MessageHandler receives NatNet error/debug messages
void __cdecl MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}

/* File writing routines */
void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs)
{
	int i=0;

    if(!pBodyDefs->arrDataDescriptions[0].type == Descriptor_MarkerSet)
        return;
        
	sMarkerSetDescription* pMS = pBodyDefs->arrDataDescriptions[0].Data.MarkerSetDescription;

	fprintf(fp, "<MarkerSet>\n\n");
	fprintf(fp, "<Name>\n%s\n</Name>\n\n", pMS->szName);

	fprintf(fp, "<Markers>\n");
	for(i=0; i < pMS->nMarkers; i++)
	{
		fprintf(fp, "%s\n", pMS->szMarkerNames[i]);
	}
	fprintf(fp, "</Markers>\n\n");

	fprintf(fp, "<Data>\n");
	fprintf(fp, "Frame#\t");
	for(i=0; i < pMS->nMarkers; i++)
	{
		fprintf(fp, "M%dX\tM%dY\tM%dZ\t", i, i, i);
	}
	fprintf(fp,"\n");

}

void _WriteFrame(FILE* fp, sFrameOfMocapData* data)
{
	fprintf(fp, "%d", data->iFrame);
	for(int i =0; i < data->MocapData->nMarkers; i++)
	{
		fprintf(fp, "\t%.5f\t%.5f\t%.5f", data->MocapData->Markers[i][0], data->MocapData->Markers[i][1], data->MocapData->Markers[i][2]);
	}
	fprintf(fp, "\n");
}

void _WriteFooter(FILE* fp)
{
	fprintf(fp, "</Data>\n\n");
	fprintf(fp, "</MarkerSet>\n");
}

void resetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = theClient->Uninitialize();
	if(iSuccess != 0)
		printf("error un-initting Client\n");

	iSuccess = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	if(iSuccess != 0)
		printf("error re-initting Client\n");


}

