/*
 * medianfilter.h
 *
 *  Created on: Oct 8, 2025
 *      Author: Minh
 */

#ifndef INC_MEDIANFILTER_H_
#define INC_MEDIANFILTER_H_



#endif /* INC_MEDIANFILTER_H_ */

//Include

#include"stdint.h"

//Define
#define WindowLength 10

//Struct
typedef struct{
	uint32_t Filter_Elements[WindowLength]; //Array for storing values of filter window
	uint32_t Sum;					//Sum of elements in window
	uint32_t WindowPointer;			//Pointer to 1st element of window
}FilterTypeDef;

//Function prototypes

void Moving_Average_Init(FilterTypeDef* filter_struct);

uint32_t Moving_Average_Compute(uint32_t raw_data, FilterTypeDef* filter_struct);
