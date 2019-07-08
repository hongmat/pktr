#ifndef __MALEKSHAN_H_
#define __MALEKSHAN_H


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#define MAX_NUMBER_OF_TIME 100
#define MAX_NUMBER_OF_LINK 100

typedef struct {
  int L,T;

  double interf_power[MAX_NUMBER_OF_LINK][MAX_NUMBER_OF_TIME]; //Interference measured at receiver side

  double tx_power[MAX_NUMBER_OF_LINK][MAX_NUMBER_OF_TIME]; //tx_power measured at receiver side

  double target_tx_power[MAX_NUMBER_OF_LINK]; //target tx_power of links

  double target_interf_power[MAX_NUMBER_OF_LINK]; //target interference power of links

  double eNB_x_position[MAX_NUMBER_OF_LINK], eNB_y_position[MAX_NUMBER_OF_LINK];

  double UE_x_position[MAX_NUMBER_OF_LINK], UE_y_position[MAX_NUMBER_OF_LINK];

  double dist_link[MAX_NUMBER_OF_LINK]; //distance between tx and rx 

  double dist_tx_origin[MAX_NUMBER_OF_LINK]; //distance from transmitter to origin 

  double beta[MAX_NUMBER_OF_LINK], alpha; 
} set_link_time;


/***** Prototypes ****/

int test_V_empty(set_link_time V);
void argmax_target_tx_interf(double dist_link, double beta, double alpha, int *target_tx_addr, int *target_interf_addr);


#endif
