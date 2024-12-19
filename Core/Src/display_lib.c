/*
 * display_lib.c
 *
 *  Created on: Dec 17, 2024
 *      Author: Mikasnik
 */


#include "display_lib.h"
#include "stm32f3xx_ll_utils.h"
#include <string.h>


void DispRegisterCallback_i2c_mread_single(void *callback) {
        if(callback != 0) disp_i2c_mread_single = callback;
}
void DispRegisterCallback_i2c_mread_multi(void *callback) {
        if(callback != 0) disp_i2c_mread_multi = callback;
}
void DispRegisterCallback_i2c_mwrite(void *callback) {
        if(callback != 0) disp_i2c_mwrite = callback;
}



int8_t display_init(){
	int8_t status = 1;
	uint16_t Gama = 0;

	while (status == 1){
		status = 0;
		status |= DISP_WrWord(EXPANDER1_ADD, GPDR_REG, 0xFFFF);
		status |= DISP_WrWord(EXPANDER1_ADD, GPSR_REG, 0xFDFB);
		status |= DISP_WrWord(EXPANDER2_ADD, GPDR_REG, 0xFFFF);
		status |= DISP_WrWord(EXPANDER2_ADD, GPSR_REG, 0xFDFB);

	}
	return status;
}


int8_t display_strongLeft(){
	int8_t status = 0;
	status |= DISP_WrWord(EXPANDER2_ADD, GPSR_REG, 0xFDE0); // left 2 digits
	status |= DISP_WrWord(EXPANDER1_ADD, GPSR_REG, 0xFDFB); // right 2 digits
	return status;
}

int8_t display_weakLeft(){
	int8_t status = 0;
	status |= DISP_WrWord(EXPANDER2_ADD, GPSR_REG, 0xF07F); // left 2 digits
	status |= DISP_WrWord(EXPANDER1_ADD, GPSR_REG, 0xFFFB); // right 2 digits
	return status;
}

int8_t display_forward(){
	int8_t status = 0;
	status |= DISP_WrWord(EXPANDER2_ADD, GPSR_REG, 0xC1FB); // left 2 digits
	status |= DISP_WrWord(EXPANDER1_ADD, GPSR_REG, 0xFDE0); // right 2 digits
	return status;
}

int8_t display_backward(){
	int8_t status = 0;
	status |= DISP_WrWord(EXPANDER2_ADD, GPSR_REG, 0xFDE0); // left 2 digits
	status |= DISP_WrWord(EXPANDER1_ADD, GPSR_REG, 0xC1FB); // right 2 digits
	return status;
}

int8_t display_weakRight(){
	int8_t status = 0;
	status |= DISP_WrWord(EXPANDER2_ADD, GPSR_REG, 0xFDFF); // left 2 digits
	status |= DISP_WrWord(EXPANDER1_ADD, GPSR_REG, 0xFF83); // right 2 digits
	return status;
}

int8_t display_strongRight(){
	int8_t status = 0;
	status |= DISP_WrWord(EXPANDER2_ADD, GPSR_REG, 0xFDFB); // left 2 digits
	status |= DISP_WrWord(EXPANDER1_ADD, GPSR_REG, 0xC1FB); // right 2 digits
	return status;
}

int8_t display_none(){
	int8_t status = 0;
	status |= DISP_WrWord(EXPANDER2_ADD, GPSR_REG, 0xFDFB); // left 2 digits
	status |= DISP_WrWord(EXPANDER1_ADD, GPSR_REG, 0xFDFB); // right 2 digits
	return status;
}





int8_t DISP_WriteMulti(uint8_t dev, uint8_t index, uint8_t *pdata, uint8_t count) {
	int8_t status = 1;
	status = disp_i2c_mwrite(pdata, count, index, dev);
	return status; // to be implemented
}

int8_t DISP_ReadMulti(uint8_t dev, uint8_t index, uint8_t *pdata, uint8_t count) {
	int8_t status = 1;
	status = disp_i2c_mread_multi(pdata, count, index, dev);
	return status; // to be implemented
}

int8_t DISP_WrByte(uint8_t dev, uint8_t index, uint8_t data) {
	int8_t status = 1;
	status = disp_i2c_mwrite((uint8_t*)(&data), 1, index, dev);
	return status; // to be implemented
}

int8_t DISP_WrWord(uint8_t dev, uint8_t index, uint16_t data) {
	int8_t status = 1;
	status = disp_i2c_mwrite((uint8_t*)(&data), 2, index, dev);
	return status; // to be implemented
}

int8_t DISP_WrDWord(uint8_t dev, uint8_t index, uint32_t data) {
	int8_t status = 1;
	status = disp_i2c_mwrite((uint8_t*)(&data), 4, index, dev);
	return status; // to be implemented
}

int8_t DISP_RdByte(uint8_t dev, uint8_t index, uint8_t *data) {
	int8_t status = 1;
	status = disp_i2c_mread_single(data, index, dev);
	return status;
}

int8_t DISP_RdWord(uint8_t dev, uint8_t index, uint16_t *data) {
	int8_t status = 1;
	status = disp_i2c_mread_multi((uint8_t*)data, 2, index, dev);
	return status; // to be implemented
}

int8_t DISP_RdDWord(uint8_t dev, uint8_t index, uint32_t *data) {
	int8_t status = 1;
	status = disp_i2c_mread_multi((uint8_t*)data, 4, index, dev);
	return status; // to be implemented
}

int8_t DISP_WaitMs(uint8_t dev, uint32_t wait_ms) {
	LL_mDelay(wait_ms);
	return 0; // to be implemented
}



