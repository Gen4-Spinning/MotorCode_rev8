/*
 * CAN_Motor.c
 *
 *  Created on: 10-Mar-2023
 *      Author: harsha
 */

#include "CAN_Motor.h"
#include "Console.h"
#include "stdio.h"
extern char UART_buffer[50];
extern UART_HandleTypeDef huart3;
extern console C;

uint8_t temp;
void FDCAN_runtimedataFromMotor(void)
{
	TxHeader.Identifier =(0xE0901<<8)|S.CAN_ID;//set to transmit runtime data frame from flyer to motherboard
	TxHeader.DataLength = FDCAN_DLC_BYTES_12;

	TxData[0]=(R.targetRPM)>>8;
	TxData[1]=R.targetRPM;
	TxData[2]=(R.presentRPM)>>8;
	TxData[3]=R.presentRPM;
	TxData[4]=(R.appliedDuty)>>8;
	TxData[5]=R.appliedDuty;
	TxData[6]=R.FETtemp;
	TxData[7]=R.MOTtemp;
	TxData[8]=(R.busCurrentADC)>>8;
	TxData[9]=R.busCurrentADC;
	TxData[10]=(R.busVoltageADC)>>8;
	TxData[11]=R.busVoltageADC;

	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>1){
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	}
}

void FDCAN_analysisdataFromMotor(void)
{
	TxHeader.Identifier =(0xA0801<<8)|S.CAN_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_12;

	TxData[0]=(R.targetRPM)>>8;
	TxData[1]=R.targetRPM;
	TxData[2]=(R.presentRPM)>>8;
	TxData[3]=R.presentRPM;
	TxData[4]=(R.appliedDuty)>>8;
	TxData[5]=R.appliedDuty;
	TxData[6]=(R.proportionalTerm)>>8;
	TxData[7]=R.proportionalTerm;
	TxData[8]=(R.IntegralTerm)>>8;
	TxData[9]=R.IntegralTerm;
	TxData[10]=(R.feedforwardTerm)>>8;
	TxData[11]=R.feedforwardTerm;

	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>1){
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	}
}

void FDCAN_errorFromMotor(void)
{
	TxHeader.Identifier =(0x60201<<8)|S.CAN_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	TxData[0]=(R.motorError)>>8;
	TxData[1]=(R.motorError);
	//while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)==0);
	//Above line blocks when no CAN connection is there, so prevents lob error resolution with the serial port
	//instead we send a msg if there is any free space.
	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>0){
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	}
}

void FDCAN_parseForMotor(uint8_t my_address)
{

	functionID=((RxHeader.Identifier)&0xFF0000)>>16;
	source_address=(RxHeader.Identifier)&0xFF;

	switch (functionID) {

		case MOTORSTATE_FUNCTIONID:
			S.CAN_MSG=RxData[0];
			FDCAN_ACKresponseFromMotor(my_address);
			//sprintf(UART_buffer,"\r\n CAN-Received MotorState-%02d",RxData[0]);
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,29);
			break;

		case DIAGNOSTICSDATA_FUNCTIONID:
			FDCAN_ACKresponseFromMotor(my_address);
			RM.runType = DIAGNOSIS_RUN;
			FDCAN_parseDiagnosticData(&RM);
			S.RM_state = RECEIVED_RAMP_SETTINGS;
			//sprintf(UART_buffer,"\r\n CAN-Received Diagnostics Setup");
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,35);
			break;

		case FULLRUN_SETUP_FUNCTIONID:
			FDCAN_ACKresponseFromMotor(my_address);
			RM.runType = NORMAL_RUN;
			FDCAN_parseFullRunSetupData(&RM);
			S.RM_state = RECEIVED_RAMP_SETTINGS;
			//sprintf(UART_buffer,"\r\n CAN-Full Run Setup");
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,32);
			break;

		case CHANGETARGET_FUNCTIONID:
			FDCAN_ACKresponseFromMotor(my_address);
			S.CAN_MSG = CHANGE_RPM;
			S.RM_state = RECEIVED_CHANGE_RPM_SETTINGS;
			FDCAN_parseChangeTargetFrame(&RM);
			//sprintf(UART_buffer,"\r\n CAN-Received Change Target");
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,29);
			break;

		case CANCHK_RESPONSE_FUNCTIONID:
			C.canCHKRecieved = 1;
			break;

		case DATA_REQ_RESPONSE_FUNCTIONID:
			temp = RxData[0];
			if (temp == PID_SETTINGS_REQUEST){
				FDCAN_send_DataResponse_FromMotor(my_address);
			}
			break;

		case PID_UPDATE_FUNCTIONID:




		default:
			break;
	}
}


void FDCAN_driveresponseFromMotor(uint8_t source)
{
	TxHeader.Identifier =(0xA0401<<8)|source;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=source;
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)==0);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_send_DataResponse_FromMotor(uint8_t source)
{
	TxHeader.Identifier =(0xA0501<<8)|source;
	TxHeader.DataLength = FDCAN_DLC_BYTES_16;

	uint16_t Kp_out = (uint16_t)(sV.Kp * 1000);
	uint16_t Ki_out = (uint16_t)(sV.Ki * 1000);
	uint16_t FF_out = (uint16_t)(sV.ff_percent * 1000);
	uint16_t SO_out = (uint16_t)(sV.start_offset);
	TxData[0]=1;//PID SETTINGS RESPONSE
	TxData[1]= Kp_out>>8;
	TxData[2]= Kp_out;
	TxData[3]= Ki_out>>8;
	TxData[4]= Ki_out;
	TxData[5]= FF_out>>8;
	TxData[6]= FF_out;
	TxData[7]= SO_out>>8;
	TxData[8]= SO_out;
	TxData[9]= 0;
	TxData[10]= 0;
	TxData[11]= 0;
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)==0);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}


void FDCAN_ACKresponseFromMotor(uint8_t source)
{
	TxHeader.Identifier =(0x060F01<<8)|source;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=1;
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)==0);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}


void FDCAN_parseFullRunSetupData(RunMgmtTypeDef *r){
	r->rampRampUpTime_s = RxData[0];
	r->rampRampDownTime_s = RxData[1];
	r->rampTarget = (RxData[2]<<8)|(RxData[3]);
	r->controlType = CLOSED_LOOP;
	r->rampSteadyRunTime_s = RUN_FOREVER;
	r->rampFunction = RAMP_RPM;
	r->rotateDirection = sV.default_direction;
	InitRampRPMStruct(&rampRPM,r->rampTarget,r->rampRampUpTime_s*1000,r->rampRampDownTime_s*1000,r->rampSteadyRunTime_s);
}


void FDCAN_parseDiagnosticData(RunMgmtTypeDef *r)
{	uint8_t temp = 0;
	r->controlType=RxData[0];
	r->rampTarget = (RxData[1]<<8)|(RxData[2]);
	r->rampRampUpTime_s = RxData[3];
	r->rampRampDownTime_s = RxData[4];
	r->rampSteadyRunTime_s = (RxData[5]<<8)|(RxData[6]);

	// on the app we show to default dir or reverse dir, and not CW/CCW.
	temp = RxData[7];
	if (temp == DEFAULT_DIR_COMMAND){
		r->rotateDirection = sV.default_direction;
	}else{
		r->rotateDirection = !sV.default_direction;
	}

	if (r->controlType == CLOSED_LOOP){
		r->rampFunction = RAMP_RPM;
		//fill up the Ramp RPM Struct- TargetRPRm,rampUp Time, rampDownTime, and steadY state runTime
		InitRampRPMStruct(&rampRPM,r->rampTarget,r->rampRampUpTime_s*1000,r->rampRampDownTime_s*1000,r->rampSteadyRunTime_s);
		//Later we send StepRPM through Diagnosis

	}else if (r->controlType == OPEN_LOOP){
		r->rampFunction = RAMP_DUTY;
		//fill up the Ramp Duty Struct- TargetRPRm,rampUp Time, rampDownTime, and steadY state runTime
		InitRampDutyStruct(&rampDuty,r->rampTarget,r->rampRampUpTime_s*1000,r->rampRampUpTime_s*1000,r->rampSteadyRunTime_s);
		//Later we send StepDuty through Diagnosis
	}else{

	}
}

void FDCAN_parseChangeTargetFrame(RunMgmtTypeDef *r){
	r->transitionTarget = (RxData[0]<<8|RxData[1]);
	r->transitionTime_ms = (RxData[2]<<8|RxData[3]);
}

void FDCAN_sendDiagDoneFrame(void){
	TxHeader.Identifier =(0xA1401<<8)|S.CAN_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=1;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_send_CANchk_Frame(void){
	TxHeader.Identifier =(0x0E1801<<8)|S.CAN_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=1;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

/*void FDCAN_ParsePIDUpdate(void){
	settingVar PU;
	PU.Ki = (RxData[1]<<8)|(RxData[2])/1000.0f;
	PU.Kp = (RxData[3]<<8)|(RxData[4])/1000.0f;
	PU.ff_percent = (RxData[5]<<8)|(RxData[6])/1000.0f;
	PU.start_offset = (RxData[7]<<8)|(RxData[8]);
	if(checkEEPROM_PIDSettings(&PU)){
		// send Success msg to mother board.
		// and then delay and restart this board.
	}else{
		//send failure msg to main board.
	}
}*/

