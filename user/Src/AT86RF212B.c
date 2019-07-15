#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../Inc/AT86RF212B.h"
#include "../Inc/AT86RF212B_HAL.h"
#include "../Inc/MainController.h"
#include "../Inc/AT86RF212B_Regesters.h"
#include "../Inc/AT86RF212B_Constants.h"
#include "../Inc/Buffer.h"
#include "../Inc/ErrorsAndLogging.h"
#include "../Settings/AT86RF212B_Settings.h"
#include "../Settings/TerminalSettings.h"

//TODO: *******NEED TO WRITE FUNCTIONALITY TO RECALIBRATE EVERY FIVE MINUITS*****************

//------------Private Function Prototypes----------------//
static void 	AT86RF212B_PowerOnReset(void);
static void 	AT86RF212B_BitWrite(uint8_t reg, uint8_t mask, uint8_t pos, uint8_t value);
static void 	AT86RF212B_IrqInit (void);
static uint8_t 	AT86RF212B_FrameLengthRead(void);
static void 	AT86RF212B_SetPhyMode(void);
static void 	AT86RF212B_PhySetChannel(void);
static uint8_t 	AT86RF212B_WaitForIRQ(uint8_t expectedIRQ);
static uint8_t 	StateChangeCheck(uint8_t newState);
static void 	UpdateState(void);
static uint8_t 	IsStateActive(void);
static uint8_t 	IsStatePllActive(void);
static uint8_t 	IsStateTxBusy(void);
static uint8_t 	IsStateRxBusy(void);
static uint8_t 	IsStateBusy(void);
static void 	AT86RF212B_UpdateIRQ(void);
static void 	AT86RF212B_FrameWrite(uint8_t * frame, uint8_t length, uint8_t sequenceNumber);
static void 	AT86RF212B_Delay(uint8_t time);
static void 	AT86RF212B_WrongStateError(void);
static void 	AT86RF212B_SetRegisters(void);
static void 	AT86RF212B_TRX_Reset(void);
static void 	AT86RF212B_PrintBuffer(uint8_t nLength, uint8_t* pData);
static void 	AT86RF212B_TxData(void);

//------------Private Global Variables----------------//
static AT86RF212B_Config config;
static volatile uint8_t interupt = 0;
static uint8_t irqState = 0;

//==============================================================================================//
//                                       Public Functions                                       //
//==============================================================================================//

void AT86RF212B_Open(){
	//------------Power On Settings-------------//
	//Initial State
	config.state = P_ON;
	//Change this to set the RF mode
	config.phyMode = AT86RF212B_PHY_MODE;   // <----AT86RF212B_O_QPSK_1000
	//scrambler configuration for O-QPSK_{400,1000}; values { 0: disabled, 1: enabled (default)}.
	config.scramen = AT86RF212B_SCRAMEN;    //������
	//transmit signal pulse shaping for O-QPSK_{250,500,1000}; values {0 : half-sine filtering (default), 1 : RC-0.8 filtering}.
	config.rcen = AT86RF212B_RCEN;
	//Set the TX power level (Table 9-15) 0x03 = 0dBm
	config.txPower = AT86RF212B_TX_POWER;
	//Set the RX sensitivity RX threshold = RSSI_BAS_VAL + rxSensLvl * 3
	//rxSensLvl = 0 - 15, 0 = max sensitivity
	config.rxSensLvl = AT86RF212B_RX_SENSE_LVL;
	//Enable TX CRC generation 1 = on 0 = off
	config.txCrc = AT86RF212B_TX_CRC;
	//Enables Rx Safe Mode
	config.rxSafeMode = AT86RF212B_RX_SAFE_MODE;
	config.AACK_UPLD_RES_FT = AT86RF212B_AACK_UPLD_RES_FT;
	config.AACK_FLTR_RES_FT = AT86RF212B_AACK_FLTR_RES_FT;
	//Enables the IRQ pin to be used as a frame buffer indicator during frame buffer reads
	config.RX_BL_CTRL = AT86RF212B_RX_BL_CTRL;
	config.AACK_ACK_TIME = AT86RF212B_AACK_ACK_TIME;
	//Address Filtering
	config.panId_7_0 = AT86RF212B_PAN_ID_7_0;
	config.panId_15_8 = AT86RF212B_PAN_ID_15_8;
	config.shortAddr_7_0 = AT86RF212B_SHORT_ADDR_7_0;
	config.shortAddr_15_8 = AT86RF212B_SHORT_ADDR_15_8;
	config.extAddr_7_0 = AT86RF212B_EXT_ADDR_7_0;
	config.extAddr_15_8 = AT86RF212B_EXT_ADDR_15_8;
	config.extAddr_23_16 = AT86RF212B_EXT_ADDR_23_16;
	config.extAddr_31_24 = AT86RF212B_EXT_ADDR_31_24;
	config.extAddr_39_32 = AT86RF212B_EXT_ADDR_39_32;
	config.extAddr_47_40 = AT86RF212B_EXT_ADDR_47_40;
	config.extAddr_55_48 = AT86RF212B_EXT_ADDR_55_48;
	config.extAddr_63_56 = AT86RF212B_EXT_ADDR_63_56;
	config.maxFrameRetries = AT86RF212B_MAX_FRAME_RETRIES;
	config.CSMA_LBT_Mode = AT86RF212B_CSMA_LBT_MODE;
	config.maxCSMA_Retries = AT86RF212B_MAX_CSMA_RETRIES;
	config.minBe = AT86RF212B_MIN_BE;
	config.maxBe = AT86RF212B_MAX_BE;
	config.slottedOperatin = AT86RF212B_SLOTTED_OPERATION;
	config.AACK_I_AmCoord = AT86RF212B_AACK_I_AM_COORD;
	config.AACK_SetPd = AT86RF212B_AACK_SET_PD;

	AT86RF212B_OpenHAL(1000);

	//Time to wait after power on
	AT86RF212B_Delay(AT86RF212B_tTR1);

	AT86RF212B_PowerOnReset();
	AT86RF212B_SetRegisters();
}

static void AT86RF212B_SetRegisters(){
	if(IsLogging()){
		LOG(LOG_LVL_DEBUG, (uint8_t*)"Registers initiated\r\n");
	}
	AT86RF212B_SetPhyMode();
	AT86RF212B_PhySetChannel();
}

uint8_t AT86RF212B_GetState(){
	return config.state;
}

void AT86RF212B_ISR_Callback(){
	interupt = 1;
}

void AT86RF212B_Main(){
	//Main State Machine
	switch(config.state){
		case P_ON:
			break;
		case TRX_OFF:
			AT86RF212B_PhyStateChange(RX_AACK_ON);
			break;
		case SLEEP:
			break;
		case RX_ON:
			break;
		case PLL_ON:
			break;
		case RX_AACK_ON:
			AT86RF212B_UpdateIRQ();
			if(irqState & (TRX_IRQ_TRX_END)){
				AT86RF212B_FrameRead();
			}
			break;
		case TX_ARET_ON:
			AT86RF212B_TxData();
			break;
		case BUSY_RX_AACK:
			break;
		case BUSY_TX:
			break;
		case BUSY_TX_ARET:
			break;
		case BUSY_RX:
			break;
		case RX_ON_NOCLK:
			break;
		default:
			if(IsLogging()){
				ASSERT((int)0);
				LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown state, changing to RX_ON\r\n");
			}
			AT86RF212B_PhyStateChange(RX_ON);
			break;
	}
}

static void AT86RF212B_TxData(){
	static uint8_t sequenceNumber = 0;
	uint8_t frame[128];
	uint8_t txByte = 0;
	uint8_t bytesToSend = 0;
	uint8_t status = 0;

	//TODO: This should be the constant AT86RF212B_MAX_DATA however the RaspberryPi is having problems transmitting larger frames
	//and until that gets resolved a smaller max frame size is being used, this does effect speed as the higher data rates
	//of the AT86RF212B only apply on the frame transmission, the headers of each framed are transmitted at a lower data rate
	const uint8_t tmpMaxData = 64;

	UpdateState();

	if(config.state != TX_ARET_ON){
		AT86RF212B_PhyStateChange(TX_ARET_ON);
		AT86RF212B_TxData();
		return;
	}
	else if(config.state == TX_ARET_ON)
	{
		while(PopFromTxBuffer(&txByte))
	   {
			frame[bytesToSend] = txByte;
			bytesToSend++;
			if(bytesToSend == tmpMaxData){
				break;
			}
		}

		if(bytesToSend){
			AT86RF212B_FrameWrite(frame, bytesToSend, sequenceNumber);
			sequenceNumber++;
			//Wait until done transmitting data
			//TODO: This may affect speed
			status = AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);

//			//TODO: Currently not getting ACKs so this is not useful
			uint8_t txStatus = AT86RF212B_BitRead(SR_TRAC_STATUS);
			if(txStatus > 1 || status != 0)
			{
				WriteToOutputHAL((uint8_t*)"FAIL.\r\n", 7);
				uint8_t tmpStr[10];
				sprintf((char*)tmpStr, "%03d\r\n", txStatus);
				WriteToOutputHAL(tmpStr, 5);
			}
			else{
				WriteToOutputHAL((uint8_t*)"SUCCESS\r\n", 7);
			}

			//Sent full frame, check to see if there is more data on the buffer to send
			if(bytesToSend == tmpMaxData){
				AT86RF212B_TxData();
			}
		}
	}
}

//-------------------Primitive Functions from AT86RF212 Programming Manual----------------------//

uint8_t AT86RF212B_RegRead(uint8_t reg){
	uint8_t pRxData[2] = {0, 0};
	uint8_t pTxData[2] = {0, 0};

	//Set the MSB and MSB-1 of the 8 bit register to a 1 0 for read access
	reg |= 1 << 7;
	reg &= ~(1 << 6);
	pTxData[0] = reg;
	AT86RF212B_SPIreadAndWriteHAL(pTxData, pRxData, 2);
	//First byte is a configurable status and the 2nd byte is the register value
	return pRxData[1];
}

uint8_t AT86RF212B_RegWrite(uint8_t reg, uint8_t value){
	uint8_t pRxData[2] = {0, 0};
	uint8_t pTxData[2] = {0, 0};

	//Set the MSB and MSB-1 of the 8 bit register to a 1 1 for write access
	reg |= 1 << 7;
	reg |= 1 << 6;
	pTxData[0] = reg;
	pTxData[1] = value;
	AT86RF212B_SPIreadAndWriteHAL(pTxData, pRxData, 2);

	return pRxData[1];
}

//==============================================================================================//
//                                     Private Functions                                        //
//==============================================================================================//

//-------------------Primitive Functions from AT86RF212 Programming Manual----------------------//

static void AT86RF212B_BitWrite(uint8_t reg, uint8_t mask, uint8_t pos, uint8_t value){
	uint8_t pRxData[2] = {0, 0};
	uint8_t pTxData[2] = {0, 0};

	uint8_t currentValue = AT86RF212B_RegRead(reg);

	//Set the MSB and MSB-1 of the 8 bit register to a 1 1 for write access
	reg |= 1 << 7;
	reg |= 1 << 6;
	pTxData[0] = reg;

	pTxData[1] = (currentValue & (~mask)) | (value << pos);
	AT86RF212B_SPIreadAndWriteHAL(pTxData, pRxData, 2);

	return;
}

uint8_t AT86RF212B_BitRead (uint8_t addr, uint8_t mask, uint8_t pos){
	uint8_t currentValue = AT86RF212B_RegRead(addr);
	currentValue = currentValue & mask;
	currentValue = currentValue >> pos;
	return currentValue;
}

static uint8_t 	AT86RF212B_FrameLengthRead(){
	uint8_t pTxData[2] = {0x20, 0};
	uint8_t pRxData[2] = {0};
	AT86RF212B_SPIreadAndWriteHAL(pTxData, pRxData, 2);
	return pRxData[1];
}

static void AT86RF212B_PrintBuffer(uint8_t nLength, uint8_t* pData) {
	char tmpStr[20];
	int i = 0;
	for (i = 0; i < nLength; i++) {
		if (pData[i] < 32 || pData[i] > 126) {
			sprintf(tmpStr, "0x%02X : \r\n", pData[i]);
		} else {
			sprintf(tmpStr, "0x%02X : %c\r\n", pData[i], pData[i]);
		}
		LOG(LOG_LVL_INFO, (uint8_t*)tmpStr);
	}
	LOG(LOG_LVL_INFO, (uint8_t*)"\r\n");
}


static void AT86RF212B_IrqInit(){
	//Set IRQ Polarity to active high
	AT86RF212B_BitWrite(SR_IRQ_POLARITY, 0);
	//Enable Awake IRQ
	//AT86RF212B_RegWrite(RG_IRQ_MASK, (TRX_IRQ_AWAKE_END | TRX_IRQ_PLL_LOCK | TRX_IRQ_TRX_END | TRX_IRQ_RX_START | TRX_IRQ_AMI));
	AT86RF212B_RegWrite(RG_IRQ_MASK, 0xFF);
	//Only show enabled interrupts in the IRQ register
	AT86RF212B_BitWrite(SR_IRQ_MASK_MODE, 0);

}

//--------------------------Routines from AT86RF212 Programming Manual--------------------------//

void AT86RF212B_FrameRead(){
	uint8_t pTxData[150];
	uint8_t pRxData[150];
	uint8_t length = 0;

	static uint8_t prevSequenceNumber = 99;
	//Read frame command
	pTxData[0] = 0x20;

	//Disable preamble detector to stop receiving
	AT86RF212B_BitWrite(SR_RX_PDT_DIS, 1);

	length = AT86RF212B_FrameLengthRead();

	if(length == 0){
		if(IsLogging()){
			ASSERT(0);
			LOG(LOG_LVL_ERROR, (uint8_t*)"No data on frame\r\n");
		}
		//Enable preamble detector to start receiving again
		AT86RF212B_BitWrite(SR_RX_PDT_DIS, 0);
		return;
	}
	else if(length > 127){
		if(IsLogging()){
			ASSERT(0);
			LOG(LOG_LVL_ERROR, (uint8_t*)"Frame too large\r\n");
		}
		//Enable preamble detector to start receiving again
		AT86RF212B_BitWrite(SR_RX_PDT_DIS, 0);
		return;
	}
	else{
		if(IsLogging()){
			uint8_t tmpStr[20];
			sprintf((char*)tmpStr, "Reading frame of size %i\r\n", length);
			LOG(LOG_LVL_DEBUG, (uint8_t*)tmpStr);
		}

		AT86RF212B_SPIreadAndWriteHAL(pTxData, pRxData, length+3);
	}

	if(config.txCrc){
		if(!AT86RF212B_BitRead(SR_RX_CRC_VALID)){
			if(IsLogging()){
				LOG(LOG_LVL_DEBUG, (uint8_t*)"CRC Failed\r\n");
			}
			//Enable preamble detector to start receiving again
			AT86RF212B_BitWrite(SR_RX_PDT_DIS, 0);
			return;
		}
		else{
			if(IsLogging()){
				LOG(LOG_LVL_DEBUG, (uint8_t*)"CRC Passed\r\n");
			}

			//Check if it is a data frame
			if((pRxData[2] & 0x07) == 1){
				if(pRxData[4] != prevSequenceNumber){
					uint8_t tmpStr[8];
					sprintf((char*)tmpStr, "RX");
					sprintf((char*)&tmpStr[2], "%03i\r\n", length-AT86RF212B_DATA_OFFSET);
					WriteToOutputHAL(tmpStr, 7);
					WriteToOutputHAL(&pRxData[AT86RF212B_DATA_OFFSET], length-AT86RF212B_DATA_OFFSET);
					WriteToOutputHAL((uint8_t*)"\r\n", 2);
					prevSequenceNumber = pRxData[4];
				}
				else{
					if(IsLogging()){
						LOG(LOG_LVL_DEBUG, (uint8_t*)"Dropped Dupe Frame\r\n");
					}
				}
			}
			//Check if it is an ACK
			else if((pRxData[2] & 0x07) == 2){
				//ackReceived = 1;
			}
			else{
				if(IsLogging()){
					ASSERT(0);
					LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Frame Type\r\n");
				}
			}

			if(IsLogging()){
				LOG(LOG_LVL_INFO, (uint8_t*)"Data Received\r\n");
				AT86RF212B_PrintBuffer(length+3, pRxData);
			}
		}
	}



	//Enable preamble detector to start receiving again
	AT86RF212B_BitWrite(SR_RX_PDT_DIS, 0);
}

static void AT86RF212B_FrameWrite(uint8_t * pData, uint8_t length, uint8_t sequenceNumber){
	//The length here has to be the length of the data and header plus 2 for the command and PHR plus 2 for the frame check sequence if enabled
#if AT86RF212B_TX_CRC
	uint8_t nLength = length+11;
#else
	uint8_t nLength = length+9;
#endif

	uint8_t pRxData[128];
	uint8_t pTxData[128] = {
	//Frame write command
	0x60,

	//PHR (PHR is just the length of the data and header and does not include one for the command or one the PHR its self so it is nLength-2)
	0x00,

	//FCF !!!BE CAREFUL OF BYTE ORDER, MSB IS ON THE RIGHT IN THE DATASHEET!!!
	//Ack requested 0x20 + frame type 1 0x01 = 0x21
	0x21,
	0x08,
	//Sequence number
	0x00,
	//Target PAN
	AT86RF212B_PAN_ID_7_0,
	AT86RF212B_PAN_ID_15_8,
	//Target ID
	AT86RF212B_SHORT_ADDR_TARGET_7_0,
	AT86RF212B_SHORT_ADDR_TARGET_15_8};

	pTxData[1] = nLength-2;
	pTxData[4] = sequenceNumber;

	memcpy(&pTxData[9], pData, length);

	AT86RF212B_SPIreadAndWriteHAL(pTxData, pRxData, nLength);
	AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_HIGH);
	AT86RF212B_Delay(AT86RF212B_t7);
	AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);

	if(IsLogging()){
		LOG(LOG_LVL_INFO, (uint8_t*)"\r\nData Sent: \r\n");
		AT86RF212B_PrintBuffer(nLength, pTxData);
	}
}


static void AT86RF212B_PowerOnReset(){
	//The following programming sequence should be executed after power-on to
	//completely reset the radio transceiver. The MCU can not count on CLKM
	//before finalization of this sequence.

	/* AT86RF212::P_ON */
	if(config.state == P_ON){
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_HIGH);
		HALDelayUs(400);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_Delay(AT86RF212B_t10);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_HIGH);
		//Turn off CLKM clock (available as a clock reference if needed)
		AT86RF212B_RegWrite(RG_TRX_CTRL_0, 0x18);

		//*************Enable interrupts****************//
		//      These don't look like they work for     //
		//      the P_ON -> TRX_OFF transition          //
		//      but work after this transition          //
		//**********************************************//
		AT86RF212B_IrqInit();

		//Change to TRX_OFF state
		AT86RF212B_RegWrite(RG_TRX_STATE, CMD_FORCE_TRX_OFF);

		AT86RF212B_Delay(AT86RF212B_tTR13);

		if(IsLogging()){
			uint8_t tmpStr[32];
			sprintf((char*)tmpStr, "IRQ Mask Reg: 0x%02X\r\n", AT86RF212B_RegRead(RG_IRQ_MASK));
			LOG(LOG_LVL_DEBUG, tmpStr);
		}

		/* AT86RF212::TRX_OFF */
		StateChangeCheck(TRX_OFF);
	}
	else{
		if(IsLogging()){
			ASSERT(0);
			LOG(LOG_LVL_ERROR, (uint8_t*)"Incorrect State to Run Function: Resetting\r\n");
		}
		AT86RF212B_TRX_Reset();
	}
}

void AT86RF212B_ID(){
	config.partid = AT86RF212B_RegRead(RG_PART_NUM);
	config.version = AT86RF212B_RegRead(RG_VERSION_NUM);
	config.manid0 = AT86RF212B_RegRead(RG_MAN_ID_0);
	config.manid1 = AT86RF212B_RegRead(RG_MAN_ID_1);

	if(IsLogging()){
		uint8_t tmpStr[32];
		sprintf((char*)tmpStr, "Part ID: 0x%02X\r\n", config.partid);
		LOG(LOG_LVL_DEBUG, tmpStr);
		sprintf((char*)tmpStr, "Version: 0x%02X\r\n", config.version);
		LOG(LOG_LVL_DEBUG, tmpStr);
		sprintf((char*)tmpStr, "ManID0:  0x%02X\r\n", config.manid0);
		LOG(LOG_LVL_DEBUG, tmpStr);
		sprintf((char*)tmpStr, "ManID1:  0x%02X\r\n", config.manid1);
		LOG(LOG_LVL_DEBUG, tmpStr);
	}
}

static void AT86RF212B_TRX_Reset(){
	//This routine will bring the radio transceiver into a known state,
	//e.g. in case of a fatal error. The use case assumes, that the
	//radio transceiver is in one of the [ACTIVE] states (any state except P_ON and SLEEP)
	//and will do a reset, so that all registers get initialized
	//with their default values.

	/* AT86RF212::[ACTIVE] */
	if(IsStateActive()){
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_Delay(AT86RF212B_t10);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_HIGH);
		AT86RF212B_IrqInit();
		AT86RF212B_Delay(AT86RF212B_tTR13);
		/* AT86RF212::TRX_OFF */
		StateChangeCheck(TRX_OFF);

		AT86RF212B_SetRegisters();
	}
	else{
		if(IsLogging()){
			ASSERT(0);
			LOG(LOG_LVL_ERROR, (uint8_t*)"Incorrect State to Run Function: This is real bad\r\n");
		}
	}
}

void AT86RF212B_PhyStateChange(uint8_t newState){
	UpdateState();
	if(config.state == P_ON){
		if(IsLogging()){
			LOG(LOG_LVL_DEBUG, (uint8_t*)"Power on startup beginning\r\n");
		}
		AT86RF212B_SetRegisters();
		AT86RF212B_PhyStateChange(newState);
	}
	switch(newState){
	case PLL_ON:
		if(config.state == PLL_ON){
			return;
		}
		if(config.state == TRX_OFF){
			/* AT86RF212::TRX_OFF */
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_PLL_ON);
			AT86RF212B_WaitForIRQ(TRX_IRQ_PLL_LOCK);
			StateChangeCheck(PLL_ON);
		}
		else if(IsStatePllActive()){
			/* AT86RF212::[PLL_ACTIVE] */
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_PLL_ON);
			AT86RF212B_Delay(AT86RF212B_tTR9);
			StateChangeCheck(PLL_ON);
		}
		else if(IsStateBusy()){
			/* AT86RF212::[BUSY] */
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_PLL_ON);
			AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
			StateChangeCheck(PLL_ON);
		}
		else if(config.state == BUSY_RX_AACK){
			/* AT86RF212::BUSY_RX_AACK */
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_PLL_ON);
			AT86RF212B_Delay(AT86RF212B_tFrame);
			StateChangeCheck(PLL_ON);
		}
		//TODO: May need to add a state to force to pll on, force to pll on is an unimplemented transition in the programmers guide
		else{
			AT86RF212B_WrongStateError();
		}
		break;


	case RX_AACK_ON:
		if(config.state == RX_AACK_ON){
			return;
		}
		/* AT86RF212::TRX_OFF */
		if(config.state == TRX_OFF){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_AACK_ON);
			AT86RF212B_Delay(AT86RF212B_tTR4);
			StateChangeCheck(RX_AACK_ON);
		}
		 /* AT86RF212::PLL_ON */
		else if(config.state ==  PLL_ON){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_AACK_ON);
			AT86RF212B_Delay(AT86RF212B_tTR8);
			StateChangeCheck(CMD_RX_AACK_ON);
		}
		/* AT86RF212::RX_AACK_ON */
		else if(config.state ==  TX_ARET_ON){
			AT86RF212B_PhyStateChange(PLL_ON);
			AT86RF212B_PhyStateChange(RX_AACK_ON);
		}
		/* AT86RF212::BUSY_TX */
		else if(IsStateTxBusy()){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_AACK_ON);
			AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
			StateChangeCheck(CMD_RX_AACK_ON);
		}
		else{
			AT86RF212B_WrongStateError();
		}
		break;
	case TX_ARET_ON:
		if(config.state == TX_ARET_ON){
			return;
		}
		/* AT86RF212::TRX_OFF */
		if(config.state == TRX_OFF){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_TX_ARET_ON);
			AT86RF212B_Delay(AT86RF212B_tTR4);
			StateChangeCheck(CMD_TX_ARET_ON);
		}
		 /* AT86RF212::PLL_ON */
		else if(config.state ==  PLL_ON){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_TX_ARET_ON);
			AT86RF212B_Delay(AT86RF212B_tTR8);
			StateChangeCheck(CMD_TX_ARET_ON);
		}
		 /* AT86RF212::RX_AACK_ON */
		else if(config.state ==  RX_AACK_ON){
			AT86RF212B_PhyStateChange(PLL_ON);
			AT86RF212B_PhyStateChange(TX_ARET_ON);
		}
		 /* AT86RF212::RX_ON */
		else if(config.state ==  RX_ON){
			AT86RF212B_PhyStateChange(PLL_ON);
			AT86RF212B_PhyStateChange(TX_ARET_ON);
		}
		/* AT86RF212::BUSY_TX */
		else if(IsStateTxBusy()){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_TX_ARET_ON);
			AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
			StateChangeCheck(CMD_TX_ARET_ON);
		}
		else{
			AT86RF212B_WrongStateError();
		}
		break;
	case RX_ON:
		if(config.state == RX_ON){
			return;
		}
		 /* AT86RF212::TX_ARET_ON */
		else if(config.state ==  TX_ARET_ON){
			AT86RF212B_PhyStateChange(PLL_ON);
			AT86RF212B_PhyStateChange(TX_ARET_ON);
		}
		/* AT86RF212::TRX_OFF */
		if(config.state == TRX_OFF){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_ON);
			AT86RF212B_Delay(AT86RF212B_tTR6);
			StateChangeCheck(RX_ON);
		}
		 /* AT86RF212::PLL_ON */
		else if(config.state ==  PLL_ON){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_ON);
			AT86RF212B_Delay(AT86RF212B_tTR8);
			StateChangeCheck(RX_ON);
		}
		/* AT86RF212::BUSY_TX */
		else if(IsStateTxBusy()){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_ON);
			AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
			StateChangeCheck(RX_ON);
		}
		else{
			AT86RF212B_WrongStateError();
		}
		break;
	}
}

static void AT86RF212B_WrongStateError(){
	if(IsLogging()){
		ASSERT(0);
		LOG(LOG_LVL_ERROR, (uint8_t*)"Incorrect State to Run Function: Resetting\r\n");
	}
	AT86RF212B_TRX_Reset();
}

static void AT86RF212B_PhySetChannel(){
	/* AT86RF212::TRX_OFF */
	if(config.state == TRX_OFF){
		AT86RF212B_BitWrite(SR_CC_BAND, 6);
		//F[MHz] = 902.0[MHz] + 0.1[MHz]  x CC_NUMBER
		AT86RF212B_BitWrite(SR_CC_NUMBER, 0x00);
	}
	else{
		if(IsLogging()){
			ASSERT(0);
			LOG(LOG_LVL_ERROR, (uint8_t*)"Incorrect State to Run Function: Resetting\r\n");
		}
		AT86RF212B_TRX_Reset();
	}
}


static void AT86RF212B_SetPhyMode(){
	/* AT86RF212::TRX_OFF */
	if(config.state == TRX_OFF){
		//TODO: Need to implement the exceptions for the gctxOffset for the OQPSK-RC-(100,200,400) cases (Table 9-15)
		switch(config.phyMode){
		case AT86RF212B_BPSK_20:
			config.useOQPSK = 0;
			config.submode = 0;
			config.OQPSK_Rate = 0;
			config.gctxOffset = 3;
			break;
		case AT86RF212B_BPSK_40:
			config.useOQPSK = 0;
			config.submode = 1;
			config.OQPSK_Rate = 0;
			config.gctxOffset = 3;
			break;
		case AT86RF212B_O_QPSK_100:
			config.useOQPSK = 1;
			config.submode = 0;
			config.OQPSK_Rate = 0;
			config.gctxOffset = 2;
			break;
		case AT86RF212B_O_QPSK_200:
			config.useOQPSK = 1;
			config.submode = 0;
			config.OQPSK_Rate = 1;
			config.gctxOffset = 2;
			break;
		case AT86RF212B_O_QPSK_250:
			config.useOQPSK = 1;
			config.submode = 0;
			config.OQPSK_Rate = 2;
			config.gctxOffset = 2;
			break;
		case AT86RF212B_O_QPSK_400:
			config.useOQPSK = 1;
			config.submode = 1;
			config.OQPSK_Rate = 0;
			config.gctxOffset = 2;
			break;
		case AT86RF212B_O_QPSK_500:
			config.useOQPSK = 1;
			config.submode = 1;
			config.OQPSK_Rate = 1;
			config.gctxOffset = 2;
			break;
		case AT86RF212B_O_QPSK_1000:
			config.useOQPSK = 1;
			config.submode = 1;
			config.OQPSK_Rate = 2;
			config.gctxOffset = 2;
			break;
		default:
			if(IsLogging()){
				ASSERT(0);
				LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Phy Configuration\r\n");
			}
			return;
		}
		AT86RF212B_BitWrite(SR_BPSK_OQPSK, config.useOQPSK);
		AT86RF212B_BitWrite(SR_SUB_MODE, config.submode);
		AT86RF212B_BitWrite(SR_OQPSK_DATA_RATE, config.OQPSK_Rate);
		AT86RF212B_BitWrite(SR_OQPSK_SCRAM_EN, config.scramen);
		AT86RF212B_BitWrite(SR_OQPSK_SUB1_RC_EN, config.rcen);
		AT86RF212B_BitWrite(SR_GC_TX_OFFS, config.gctxOffset);
		AT86RF212B_RegWrite(RG_PHY_TX_PWR, config.txPower);
		AT86RF212B_BitWrite(SR_RX_PDT_LEVEL, config.rxSensLvl);
		AT86RF212B_BitWrite(SR_TX_AUTO_CRC_ON, config.txCrc);
		AT86RF212B_BitWrite(SR_MAX_FRAME_RETRIES, config.maxFrameRetries);
		AT86RF212B_BitWrite(SR_CSMA_LBT_MODE, config.CSMA_LBT_Mode);
		AT86RF212B_BitWrite(SR_MAX_CSMA_RETRIES, config.maxCSMA_Retries);
		AT86RF212B_BitWrite(SR_MIN_BE, config.minBe);
		AT86RF212B_BitWrite(SR_MAX_BE, config.maxBe);
		AT86RF212B_BitWrite(SR_SLOTTED_OPERATION, config.slottedOperatin);
		AT86RF212B_BitWrite(SR_AACK_I_AM_COORD, config.AACK_I_AmCoord);
		AT86RF212B_BitWrite(SR_AACK_SET_PD, config.AACK_SetPd);
		AT86RF212B_BitWrite(SR_AACK_UPLD_RES_FT, config.AACK_UPLD_RES_FT);
		AT86RF212B_BitWrite(SR_AACK_FLTR_RES_FT, config.AACK_FLTR_RES_FT);
		AT86RF212B_BitWrite(SR_RX_SAFE_MODE, config.rxSafeMode);
		AT86RF212B_BitWrite(SR_RX_BL_CTRL, config.RX_BL_CTRL);
		AT86RF212B_BitWrite(SR_AACK_ACK_TIME, config.AACK_ACK_TIME);
		AT86RF212B_RegWrite(RG_PAN_ID_0, config.panId_7_0);
		AT86RF212B_RegWrite(RG_PAN_ID_1, config.panId_15_8);
		AT86RF212B_RegWrite(RG_SHORT_ADDR_0, config.shortAddr_7_0);
		AT86RF212B_RegWrite(RG_SHORT_ADDR_1, config.shortAddr_15_8);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_0, config.extAddr_7_0);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_1, config.extAddr_15_8);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_2, config.extAddr_23_16);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_3, config.extAddr_31_24);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_4, config.extAddr_39_32);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_5, config.extAddr_47_40);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_6, config.extAddr_55_48);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_7, config.extAddr_63_56);
	}
	else{
		if(IsLogging()){
			ASSERT(0);
			LOG(LOG_LVL_ERROR, (uint8_t*)"Incorrect State to Run Function\r\n");
		}
	}
	return;
}

//---------------------------------Custom Functions---------------------------------//

static void AT86RF212B_UpdateIRQ(){
	if(interupt){
		//Clear the interrupt flag
		interupt = 0;
		irqState = AT86RF212B_RegRead(RG_IRQ_STATUS);
	}
	else{
		irqState = 0;
	}
}

static uint8_t AT86RF212B_WaitForIRQ(uint8_t expectedIRQ){
	//Max time in ms to wait for an IRQ before timing out
	uint32_t maxTime = 100;

	//TODO: What happens if the timer rolls
	uint32_t timeout = HALGetMs()+maxTime;
	while(!interupt){
		if(HALGetMs() > timeout){
			if(IsLogging())
				{
					ASSERT(0);
					LOG(LOG_LVL_DEBUG, (uint8_t*)"Timeout while waiting for IRQ\r\n");
				}
			//Unsuccessful
			return 1;
		}
	}
	//Clear the interrupt flag
	interupt = 0;

	irqState = AT86RF212B_RegRead(RG_IRQ_STATUS);

	if(!(irqState & expectedIRQ)){
		if(IsLogging()){
			ASSERT(0);
			uint8_t tmpStr[20];
			sprintf((char *)tmpStr, "Wrong Interrupt: %02X\r\n", irqState);
			LOG(LOG_LVL_ERROR, tmpStr);
		}
		uint8_t tmpRet = AT86RF212B_WaitForIRQ(expectedIRQ);
		return tmpRet;
	}
	else if(IsLogging()){
		LOG(LOG_LVL_DEBUG, (uint8_t*)"Expected IRQ received, exiting loop!\r\n");
	}
	//Successful
	return 0;
}

static uint8_t StateChangeCheck(uint8_t newState){
	UpdateState();

	if(config.state != newState){
		if(IsLogging()){
			ASSERT(0);
			LOG(LOG_LVL_ERROR, (uint8_t*)"State Change Failed Trying Again\r\n");
		}
		return 0;
	}
	else{
		if(IsLogging()){
			LOG(LOG_LVL_DEBUG, (uint8_t*)"State Change Success!\r\n");
		}
		return 1;
	}
}

static void UpdateState(){
	config.state = AT86RF212B_RegRead(RG_TRX_STATUS);
}

static uint8_t IsStateActive(){
//	any state except P_ON and SLEEP
	return ((config.state == P_ON) || (config.state == SLEEP)) ? 0: 1;
}

static uint8_t IsStatePllActive(){
//	RX_ON, PLL_ON, TX_ARET_ON, RX_AACK_ON
	return ((config.state == RX_ON) || (config.state == PLL_ON) || (config.state == TX_ARET_ON) || (config.state == RX_AACK_ON)) ? 1: 0;
}

static uint8_t IsStateTxBusy(){
//	BUSY_TX, BUSY_TX_ARET
	return ((config.state == BUSY_TX) || (config.state == BUSY_TX_ARET)) ? 1: 0;
}

static uint8_t IsStateRxBusy(){
//	BUSY_RX, BUSY_RX_AACK
	return ((config.state == BUSY_RX) || (config.state == BUSY_RX_AACK)) ? 1: 0;
}

static uint8_t IsStateBusy(){
//	[TX_BUSY], [RX_BUSY]
	return (IsStateTxBusy() || IsStateRxBusy()) ? 1: 0;
}

static void AT86RF212B_Delay(uint8_t time){
	switch(time){
		case AT86RF212B_t7:
			//t7 	SLP_TR pulse width
			//    62.5 ns
			HALDelayUs(1);
			break;
		case AT86RF212B_t8:
			//t8 	SPI idle time: SEL rising to falling edge
			//    250 ns
			HALDelayUs(1);
			break;
		case AT86RF212B_t8a:
			//t8a 	SPI idle time: SEL rising to falling edge
			//    500 ns
			HALDelayUs(1);
			break;
		case AT86RF212B_t9:
			//t9 	SCLK rising edge LSB to /SEL rising edge
			//    250 ns
			HALDelayUs(1);
			break;
		case AT86RF212B_t10:
			//t10 	Reset pulse width
			//    625 ns
			HALDelayUs(1);
			break;
		case AT86RF212B_t12:
			//t12 	AES core cycle time
			//    24 �s
			HALDelayUs(1);
			break;
		case AT86RF212B_t13:
			//t13 	Dynamic frame buffer protection: IRQ latency
			//    750 ns
			HALDelayUs(1);
			break;
		case AT86RF212B_tTR1:
			//tTR1 	State transition from P_ON until CLKM is available
			//    330 �s

			//However the datasheet (7.1.4.1) says 420 �s typical and 1ms max
			HALDelayMs(1);
			break;
		case AT86RF212B_tTR2:
			//tTR2 	State transition from SLEEP to TRX_OFF
			//    380 �s
			HALDelayUs(380);
			break;
		case AT86RF212B_tTR3:
			//tTR3 	State transition from TRX_OFF to SLEEP
			//    35 CLKM cycles

			//TODO: Implement this better
			HALDelayUs(2);
			break;
		case AT86RF212B_tTR4:
			//tTR4 	State transition from TRX_OFF to PLL_ON
			//    110 �s
			HALDelayUs(170);
			break;
		case AT86RF212B_tTR5:
			//tTR5 	State transition from PLL_ON to TRX_OFF
			//    1 �s
			HALDelayUs(1);
			break;
		case AT86RF212B_tTR6:
			//tTR6 	State transition from TRX_OFF to RX_ON
			//    110 �s
			HALDelayUs(170);
			break;
		case AT86RF212B_tTR7:
			//tTR7 	State transition from RX_ON to TRX_OFF
			//    1 �s
			HALDelayUs(1);
			break;
		case AT86RF212B_tTR8:
			//tTR8 	State transition from PLL_ON to RX_ON
			//    1 �s
			HALDelayUs(1);
			break;
		case AT86RF212B_tTR9:
			//tTR9 	State transition from RX_ON to PLL_ON
			//    1 �s
			HALDelayUs(1);
			break;
		case AT86RF212B_tTR10:
			//tTR10 	State transition from PLL_ON to BUSY_TX
			//    1 symbol

			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					HALDelayUs(50);
					break;
				case AT86RF212B_BPSK_40:
					HALDelayUs(25);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					HALDelayUs(40);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					HALDelayUs(16);
					break;
				default:
					if(IsLogging()){
						ASSERT(0);
						LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Phy Mode");
					}
					break;
			}

			break;
		case AT86RF212B_tTR12:
			//tTR12 	Transition from all states to TRX_OFF
			//    1 �s
			HALDelayUs(1);
			break;
		case AT86RF212B_tTR13:
			//tTR13 	State transition from RESET to TRX_OFF
			//    26 �s
			HALDelayUs(26);
			break;
		case AT86RF212B_tTR14:
			//tTR14 	Transition from various states to PLL_ON
			//    1 �s
			HALDelayUs(1);
			break;
		case AT86RF212B_tTR16:
			//tTR16 	FTN calibration time
			//    25 �s
			HALDelayUs(25);
			break;
		case AT86RF212B_tTR20:
			//tTR20 	PLL settling time on channel switch
			//    11 �s
			HALDelayUs(11);
			break;
		case AT86RF212B_tTR21:
			//tTR21 	PLL CF calibration time
			//    8 �s
			HALDelayUs(8);
			break;
		case AT86RF212B_tTR25:
			//tTR25 	RSSI update interval
			//    32 �s : BPSK20
			//    24 �s : BPSK40
			//    8 �s : OQPSK

			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					HALDelayUs(32);
					break;
				case AT86RF212B_BPSK_40:
					HALDelayUs(24);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_400:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					HALDelayUs(8);
					break;
				default:
					if(IsLogging()){
						ASSERT(0);
						LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Phy Mode");
					}
					break;
			}
			break;
		case AT86RF212B_tTR26:
			//tTR26 	ED measurement time
			//    8 symbol : Low Data Rate Mode (LDRM) and manual measurement in High Data Rate Mode (HDRM)
			//    2 symbol : automatic measurement in High Data Rate Mode (HDRM)

			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					//    8 symbol
					HALDelayUs(400);
					break;
				case AT86RF212B_BPSK_40:
					//    8 symbol
					HALDelayUs(200);
					break;
				case AT86RF212B_O_QPSK_100:
					//    8 symbol
					HALDelayUs(320);
					break;
				case AT86RF212B_O_QPSK_200:
					//    2 symbol
					HALDelayUs(80);
					break;
				case AT86RF212B_O_QPSK_250:
					//    8 symbol
					HALDelayUs(128);
					break;
				case AT86RF212B_O_QPSK_400:
					//    2 symbol
					HALDelayUs(80);
					break;
				case AT86RF212B_O_QPSK_500:
					//    2 symbol
					HALDelayUs(32);
					break;
				case AT86RF212B_O_QPSK_1000:
					//    2 symbol
					HALDelayUs(32);
					break;
				default:
					if(IsLogging()){
						ASSERT(0);
						LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Phy Mode");
					}
					break;
			}
			break;
		case AT86RF212B_tTR28:
			//tTR28 	CCA measurement time
			//    8 symbol
			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					HALDelayUs(400);
					break;
				case AT86RF212B_BPSK_40:
					HALDelayUs(200);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					HALDelayUs(320);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					HALDelayUs(128);
					break;
				default:
					if(IsLogging()){
						ASSERT(0);
						LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Phy Mode");
					}
					break;
			}
			break;
		case AT86RF212B_tTR29:
			//tTR29 	SR_RND_VALUE update time
			//    1 �s
			HALDelayUs(1);
			break;
		case AT86RF212B_tMSNC:
			//tMSNC 	Minimum time to synchronize to a preamble and receive an SFD
			//    2 symbol
			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					HALDelayUs(100);
					break;
				case AT86RF212B_BPSK_40:
					HALDelayUs(400);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					HALDelayUs(80);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					HALDelayUs(32);
					break;
			}
			break;
		case AT86RF212B_tFrame:
			//tMSNC 	Minimum time to synchronize to a preamble and receive an SFD
			//    2 symbol
			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					HALDelayMs(52);
					break;
				case AT86RF212B_BPSK_40:
					HALDelayMs(26);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					HALDelayMs(11);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					HALDelayUs(5);
					break;
				default:
					if(IsLogging()){
						ASSERT(0);
						LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Phy Mode");
					}
					break;
			}
			break;
		default:
			if(IsLogging()){
				ASSERT(0);
				LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Time Mode");
			}
			break;
	}
	return;
}
