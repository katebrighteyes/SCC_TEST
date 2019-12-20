#include <stdio.h>
#include <stdlib.h>
#include <memory>

#include <unistd.h>

#include <string.h>

#include <chrono>
#include <mutex>
#include <thread>

#include "cansocket.h"

#define MAX_VALUE_LEN (32+1)
#define MAX_VALUE16_LEN (16+1)
#define RXTEST
#define NFACTOR 10000

static volatile bool gRun = true;

float rxObjID = 0.0f;
float rxXDist = 0;
float rxYDist = 0;
float ObjXVel = 0;

float ego_speed = 0;

float txAccel = 0;
float txRelVel = 0;
float txRelDist = 0;
int accCmd;


#if 1//def TEST_RXTX_DATA
//      ID      msg_Name      sig_Name       value   startbit   length  utype   factor      offset 
CanDBData gRxData[4] =  {
    {0x200, "TEST_object",   "objVel",         0.0,       0,       16,    1,    (1*0.01),  0.0 },
    {0x200, "TEST_object",   "objID",         0.0,      16,       16,    1,    (1*0.01),  0.0 },
    {0x200, "TEST_object",   "X_Dist",        0.0,      32,       16,    1,    (1*0.01),  0.0 },
    {0x201, "TEST_vehicle",  "Vehicle_Velo",  0.0,       0,       16,    1,    (1*0.01),  0.0 },
};
#endif

//-----------------------------------
//      << filed name >>
//      ID   msg_Name      sig_Name        value   startbit     length  utype   factor      offset 
//
CanDBData gTxData[4] =  {
    {0x301, "TEST_ACC",      "accCMD",        0.0,      0,       16,     1,     (1*0.01),     0.0 },
    {0x301, "TEST_ACC",      "accel",        0.0,     16,       16,     1,     (1),          0.0 },
    {0x301, "TEST_ACC",      "Rel_Dist",     0.0,     32,       16,     1,     (1*0.01),     0.0 },
    {0x301, "TEST_ACC",      "Rel_Velo",     0.0,     48,       16,     1,     (1*0.01),     0.0 },
};

//------------------------------------------------------------------------------
std::thread sendThread1;

/*-----------------------------------------*/
/* Automatic Cruise Control (ACC) Controller Parameters */
/*-----------------------------------------*/

typedef struct CanDBData_
{
    uint32_t id;
    char message_name[32];
    char signal_name[32];
    float value;
    uint8_t startbit;
    uint8_t length;
    uint8_t utype; //unsigned = 0, signed = 1
    float factor;
    float offset;

} CanDBData;


int fillMessage4Data(struct can_frame* msg, CanDBData* tx1, CanDBData* tx2, CanDBData* tx3, CanDBData* tx4)
{
    float fval1 = (tx1->value + tx1->offset)/ tx1->factor;
    float fval2 = (tx2->value + tx2->offset)/ tx2->factor;
    float fval3 = (tx3->value + tx3->offset)/ tx3->factor;
    float fval4 = (tx4->value + tx4->offset)/ tx4->factor;

    std::cout <<std::hex<<">>>>>msg.id:"<<msg->id<<"  ["<<tx1->message_name <<"]:"<<std::endl;
              //<<"  ["<<tx1->signal_name <<std::dec<<"]:"<<tx1->value<<", "<<std::endl;
    std::cout <<"  ["<<tx2->signal_name <<std::dec<<"]:"<<tx2->value<<", "<<std::endl;
      //std::cout <<"FILLMESSAGE"<< std::endl;
    union endi {
        uint16_t sv;
        uint8_t un[2];
    } ev1, ev2, ev3, ev4;

    //Sig1
    uint16_t nval1 = (uint16_t)fval1;
    ev1.sv = htons(nval1);
    for(int i=0;i<2;i++) {
        std::cout <<std::hex<< (int)ev1.un[i]<<" ";
        msg->data[2-i-1] = ev1.un[i];
    }
    //Sig2
    uint16_t nval2 = (uint16_t)fval2;
    ev2.sv = htons(nval2);
    for(int i=0;i<2;i++) {
       //   std::cout <<std::hex<< (int)ev2.un[i]<<" ";
        msg->data[4-i-1] = ev2.un[i];
    }
    //Sig3
    uint16_t nval3 = (uint16_t)fval3;
    ev3.sv = htons(nval3);
    for(int i=0;i<2;i++) {
       //   std::cout <<std::hex<< (int)ev2.un[i]<<" ";
        msg->data[6-i-1] = ev3.un[i];
    }
    //Sig4
    uint16_t nval4 = (uint16_t)fval4;
    ev4.sv = htons(nval4);
    for(int i=0;i<2;i++) {
       //   std::cout <<std::hex<< (int)ev2.un[i]<<" ";
        msg->data[8-i-1] = ev4.un[i];
    }
    return 0;
}

//------------------------------------------------------------------------------
void convert10(char* tempv, uint16_t *decnum) {
    int lenv = 4;//strlen(tempv);
    int pos = 0;
    uint16_t u16=0;
    //std::cout <<"tempv:" <<tempv << std::endl;
    for(int i = lenv-1; i>=0;i--) {
        int num1 = tempv[i];
        if(num1 >=48 && num1 <=57) { //0~9
            u16 += (num1-48) * pow(16, pos);
        } else if(num1 >=65 && num1 <=70) { //A~F
            u16 += (num1-(65-10)) * pow(16, pos);
        } else if(num1 >=97 && num1 <=102) { //a~f
            u16 += (num1-(97-10)) * pow(16, pos);
        }    
        pos++;
    	//std::cout <<"tempv[i]:"<<tempv[i]<<std::dec<<" u16:"<<u16 << std::endl;
    }  
    *decnum = u16;
}

int interprete16_3(struct can_frame* msg, CanDBData* rxdata1, CanDBData* rxdata2, CanDBData* rxdata3)
{
    std::cout << " msg->id : " <<std::hex << msg->id << " "<< std::endl;
    char tempv1[MAX_VALUE16_LEN];
    uint16_t u16val1 = 0;
    char tempv2[MAX_VALUE16_LEN];
    uint16_t u16val2 = 0;
    char tempv3[MAX_VALUE16_LEN];
    uint16_t u16val3 = 0;
//*
    for(int i=0;i<6;i++) {
        std::cout <<std::hex<< (int)msg->data[i]<<" ";
    }
 // */
    sprintf(tempv1,"%2x%2x", msg->data[1], msg->data[0]);
    sprintf(tempv2,"%2x%2x", msg->data[3], msg->data[2]);
    sprintf(tempv3,"%2x%2x", msg->data[5], msg->data[4]);
    int lenv = strlen(tempv1);
    tempv1[lenv] = 0;
    tempv2[lenv] = 0;
    tempv3[lenv] = 0;

    std::cout <<std::dec;
    std::cout << "tempv1:"<< tempv1 << " tempv2:"<< tempv2<<" tempv3:"<< tempv3<< std::endl;

    convert10(tempv1, &u16val1);
    convert10(tempv2, &u16val2);
    convert10(tempv3, &u16val3);
    rxdata1->value = (double)u16val1 * rxdata1->factor - rxdata1->offset;
    rxdata2->value = (double)u16val2 * rxdata2->factor - rxdata2->offset;
    rxdata3->value = (double)u16val3 * rxdata3->factor - rxdata3->offset;
    std::cout <<std::dec<<"rxdata1->value : " << rxdata1->value << " "<< std::endl;
    std::cout <<std::dec<<"rxdata2->value : " << rxdata2->value << " "<< std::endl;
    std::cout <<std::dec<<"rxdata3->value : " << rxdata3->value << " "<< std::endl;
    return 0;

}

//------------------------------------------------------------------------------
int interprete16(struct can_frame* msg, CanDBData* rxdata)
{
    std::cout << " msg->id : " <<std::hex << msg->id << " "<< std::endl;
    char tempv[MAX_VALUE16_LEN];
    uint16_t u16 = 0;

//sprintf(tempv,"%2x%2x%2x%2x", msg->data[3], msg->data[2], msg->data[1],msg->data[0]);
    sprintf(tempv,"%x%x", msg->data[0], msg->data[1]);
    int lenv = strlen(tempv);
    tempv[lenv] = 0;

    std::cout <<std::dec;
    std::cout << "tvalue:"<< tempv << " length of value:"<< lenv<< std::endl;

    convert10(tempv, &u16);
//*//
    rxdata->value = u16 * rxdata->factor - rxdata->offset;
   // std::cout <<std::dec<<"factor:" << rxdata->factor << " offset:" << rxdata->offset;
    std::cout <<std::dec<<"rxdata->value : " << rxdata->value << " "<< std::endl;
    std::cout << std::endl;
    return 0;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void sig_int_handler(int sig)
{
    (void)sig;

    gRun = false;

    close_port();

}


int main(void)
{
    open_port("can0");
    read_port();

    struct sigaction action = {};
    action.sa_handler = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command

    gRun = true;

    //Time_t sendInterval = 0;
    unsigned int sendTestId = 0x6FF;

	
	static int setdataCnt = 0;

		while (gRun)
		{
			//std::this_thread::sleep_for(std::chrono::microseconds(10));
	
			//std::cout << "++++++ READ ++++++" << std::endl;
			struct can_frame msg;
			memset((void*)&msg, 0x00, sizeof(struct can_frame));
			int ret = read_port(&msg);

			//gRxData
			//memset((void*)gRxData, 0x00, sizeof(CanDBData)*4);
			if(msg.id == 0x200) //test_object 
			{
				//std::cout << "==> msg.id :"<< msg.id << " gRxData.ID :" << gRxData[k].id << std::endl;			
				interprete16_3(&msg, &gRxData[0], &gRxData[1], &gRxData[2]);
				std::cout << "***msg.id:"<< std::uppercase<< std::setw(3) << std::hex << msg.id 
				  << std::dec << "	(" << msg.size << ") ";
				std::cout << "["<< gRxData[0].signal_name <<"] :"<< gRxData[0].value<< std::endl;
				std::cout << "["<< gRxData[1].signal_name <<"] :"<< gRxData[1].value<< std::endl;
				std::cout << "["<< gRxData[2].signal_name <<"] :"<< gRxData[2].value<< std::endl;
				ObjXVel = gRxData[0].value;
				rxObjID = gRxData[1].value;
				rxXDist = gRxData[2].value;
			}
			if(msg.id == 0x201) //test_vehicle
			{
				//			std::cout << "==> msg.id :"<< msg.id << " gRxData.ID :" << gRxData[k].id << std::endl;			
				interprete16(&msg, &gRxData[3]);
				std::cout << "***msg.id:"<< std::uppercase<< std::setw(3) << std::hex << msg.id 
				  << std::dec << "	(" << msg.size << ") ";
				std::cout << "["<< gRxData[3].signal_name <<"] :"<< gRxData[3].value<< std::endl;
				//std::cout << "["<< gRxData[rxIdx+1].signal_name <<"] :"<< gRxData[rxIdx+1].value<< std::endl;
				ego_speed = gRxData[3].value;
				setdataCnt = 1;
			}
			
			int acc_cmd = 0;
			if(setdataCnt < 1) {
				continue;
			}
	
			static float prevVel = 0;
			if(prevVel == 0) {
				prevVel = ego_speed;
				continue;
			}else
			{	
				double accel = accLogic(rxXDist, ego_speed, ObjXVel);
				txAccel = calEgoSpeed(prevVel, accel, 0.01);
				prevVel = ego_speed;;
				//Set gTxData
				gTxData[0].value = (float)acc_cmd;
				gTxData[1].value = txAccel;
				gTxData[2].value = txRelDist;
				gTxData[3].value = txRelVel;
			}

			std::cout << std::endl;
		    std::cout << "[ID]:"	 << (float)gTxData[0].value;
			std::cout << " [CMD]:" << (float)gTxData[1].value;
			std::cout << std::endl;
			std::cout << " [VEL]:" << (float)gTxData[2].value;
			std::cout << " [DIST]:"<< (float)gTxData[3].value;
			std::cout << std::endl;

			struct can_frame testMsg;
			std::cout<<std::dec;
			testMsg.id		 = sendTestId;
			testMsg.size		 = DW_SENSORS_CAN_MAX_MESSAGE_LEN;
			testMsg.timestamp_us = 0;

			//5
			testMsg.id			 = gTxData[0].id;
			fillMessage4Data(&testMsg, &gTxData[0], &gTxData[1], &gTxData[2], &gTxData[3]);

//TODO 2 send message
			std::cout<<std::dec;
			std::cout<<std::endl;

		   // std::cout << std::endl;
			send_port(&testMsg);

	} //whilea
	
	close_port();


    return 0;
}

