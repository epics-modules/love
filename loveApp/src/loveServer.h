// loveServer.h

// Author: Jim Kowalkowski
// Date: 7/94
// Changes for MPF: Marty Kraimer
// Date: 27AUG98
// Added Additional Functionality  Mohan Ramanathan
// Date: 7 May 1999
//  Revised to accomadate both 1600 series and 16A series.
//  August 31, 2000
//  Revised to add more functionality.  -Mohan Ramanathan
//  March 5, 2002

typedef enum 
{
        GetValue, GetSP1, GetSP2, GetALLo, GetALHi, GetPeak, 
        GetValley, GetALStatus, GetALType, GetInpType, GetCommStatus,
        PutSP1, PutSP2, PutALLo, PutALHi, 
        ResetPeak, ResetValley, SetRemote 
} cmdType;

#define R_PV        	0x00	//  read process variable with Alarm status
#define W_REM      		0x0400 //  set Remote Mode
#define W_LOC      		0x0401 //  set Local Mode

//for Model 1600:
#define R0_DPT          0x0324 //  read decimal point location
#define R0_SP1V    		0x0100 //  read Set Point 1 Value
#define R0_SP2V     	0x0102 //  read Set Point 2 Value
#define R0_ALLO      	0x0104 //  read Alarm Low Value
#define R0_ALHI       	0x0105 //  read Alarm High Value
#define R0_PEAK       	0x011A //  read Peak Value
#define R0_VALY      	0x011B //  read Valley Value
#define R0_ALTY      	0x0337 //  read Alarm type
#define R0_INTY      	0x0323 //  read Input type
#define R0_COMM      	0x032A //  read Communication Mode

#define W0_SP1V    		0x0200 //  write Set Point 1 Value
#define W0_SP2V     	0x0202 //  write Set Point 2 Value
#define W0_ALLO      	0x0204 //  write Alarm Low Value
#define W0_ALHI       	0x0205 //  write Alarm High Value
#define W0_PEAK       	0x0407 //  reset Peak Value
#define W0_VALY      	0x0408 //  reset Valley Value

// for model 16A, 32A, 2600 & 8600:
#define R1_DPT          0x031A //  read decimal point location
#define R1_SP1V    		0x0101 //  read Set Point 1 Value
#define R1_SP2V     	0x0105 //  read Set Point 2 Value
#define R1_ALLO      	0x0106 //  read Alarm Low Value
#define R1_ALHI       	0x0107 //  read Alarm High Value
#define R1_PEAK       	0x011D //  read Peak Value
#define R1_VALY       	0x011E //  read Valley Value
#define R1_ALTY      	0x031D //  read Alarm type
#define R1_INTY      	0x0317 //  read Input type
#define R1_COMM      	0x0324 //  read Communication Mode

#define W1_SP1V    		0x0200 //  write Set Point 1 Value
#define W1_SP2V     	0x0204 //  write Set Point 2 Value
#define W1_ALLO      	0x0207 //  write Alarm Low Value
#define W1_ALHI       	0x0208 //  write Alarm High Value
#define W1_PEAK       	0x040A //  reset Peak Value
#define W1_VALY      	0x040B //  reset Valley Value

