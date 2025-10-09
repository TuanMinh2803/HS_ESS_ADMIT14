/*
 * medianfilter.c
 *
 *  Created on: Oct 8, 2025
 *      Author: Minh
 */

#include"medianfilter.h"


//Init data struct for filter

void Moving_Average_Init(FilterTypeDef* filter_struct){
	filter_struct->Sum=0;
	filter_struct->WindowPointer=0;

	for(int i=0; i<WindowLength; i++){
		filter_struct->Filter_Elements[i]=0;
	}
}

//Compute filter datas with moving window

uint32_t Moving_Average_Compute(uint32_t raw_data, FilterTypeDef* filter_struct){
	filter_struct->Sum+=raw_data;		//Add latest raw input
	filter_struct->Sum-=filter_struct->Filter_Elements[filter_struct->WindowPointer];    //Subtract oldest element
	filter_struct->Filter_Elements[filter_struct->WindowPointer] = raw_data;

	if(filter_struct->WindowPointer == 0){
		filter_struct->WindowPointer=WindowLength-1;
	}
	else{
		filter_struct->WindowPointer-=1;
	}

	return filter_struct->Sum/WindowLength;
}

