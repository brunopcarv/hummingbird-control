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
#include <math.h> 
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>
#include <process.h> //multi-threading
#include "comms.h"
#include "Globals.h"

#include "NatNetTypes.h"
#include "NatNetClient.h"

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

unsigned int MyServersDataPort = 3130;
unsigned int MyServersCommandPort = 3131;
int iConnectionType = ConnectionType_Multicast;
//int iConnectionType = ConnectionType_Unicast;

extern float g_roll_cmd;
extern float g_pitch_cmd;
extern float g_yaw_cmd;
extern float g_thrust_cmd;

extern bool g_new_data;

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

int _tmain(int argc, _TCHAR* argv[])
{
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
						g_thrust_cmd += 0.1;
						if (g_thrust_cmd > 6)
							g_thrust_cmd = 6;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 'z':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
						g_thrust_cmd -= 0.1;
						if (g_thrust_cmd < 0)
							g_thrust_cmd = 0;
					}
					ReleaseMutex(hcommsMutex);	
					break;	
				case 's':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{
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
				case 'i':
					dwWaitResult = WaitForSingleObject(hcommsMutex, INFINITE);
					if (dwWaitResult == WAIT_OBJECT_0)
					{	
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
					}
					ReleaseMutex(huserMutex);	
					break;
				case 'o':
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
	double yaw;// in degrees
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

	GetEulers(data->RigidBodies[0].qx,data->RigidBodies[0].qy, data->RigidBodies[0].qz, data->RigidBodies[0].qw, &yaw, &pitch, &roll);
	
	

	if (countPrintf == 100)
	{
	ClearScreen2();
	printf("RIGID BODY POSITION\n");
	printf("X [%f]\n", data->RigidBodies[0].x);
	printf("Y [%f]\n", data->RigidBodies[0].y);
	printf("Z [%f]\n", data->RigidBodies[0].z);
	//printf("qx [%f]\n", data->RigidBodies[0].qx);
	//printf("qy [%f]\n", data->RigidBodies[0].qy);
	//printf("qz [%f]\n", data->RigidBodies[0].qz);
	//printf("qw [%f]\n", data->RigidBodies[0].qw);
	printf("ROLL [%f]\tPITCH [%f]\tYAW [%f]\t\n", roll,pitch,yaw);
	//printf("YAAAWWW [%f]\n", yaw(3,1,100,1));
	printf("##############################\n");
	countPrintf = 0;
	}
	countPrintf ++;


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

