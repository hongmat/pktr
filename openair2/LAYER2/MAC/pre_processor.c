/*******************************************************************************
    OpenAirInterface
    Copyright(c) 1999 - 2014 Eurecom

    OpenAirInterface is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.


    OpenAirInterface is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with OpenAirInterface.The full GNU General Public License is
    included in this distribution in the file called "COPYING". If not,
    see <http://www.gnu.org/licenses/>.

  Contact Information
  OpenAirInterface Admin: openair_admin@eurecom.fr
  OpenAirInterface Tech : openair_tech@eurecom.fr
  OpenAirInterface Dev  : openair4g-devel@lists.eurecom.fr

  Address      : Eurecom, Campus SophiaTech, 450 Route des Chappes, CS 50193 - 06904 Biot Sophia Antipolis cedex, FRANCE

*******************************************************************************/
/*! \file pre_processor.c
 * \brief eNB scheduler preprocessing fuction prior to scheduling
 * \author Navid Nikaein and Ankit Bhamri
 * \date 2013 - 2014
 * \email navid.nikaein@eurecom.fr
 * \version 1.0
 * @ingroup _mac

 */
#include <stdio.h>
#include "assertions.h"
#include "PHY/defs.h"
#include "PHY/extern.h"

//pktR and UCS
#include <math.h> //for error function
#include <pthread.h>

#include "SCHED/defs.h"
#include "SCHED/extern.h"

#include "LAYER2/MAC/defs.h"
#include "LAYER2/MAC/proto.h"
#include "LAYER2/MAC/extern.h"
#include "UTIL/LOG/log.h"
#include "UTIL/LOG/vcd_signal_dumper.h"
#include "UTIL/OPT/opt.h"
#include "OCG.h"
#include "OCG_extern.h"
#include "RRC/LITE/extern.h"
#include "RRC/L2_INTERFACE/openair_rrc_L2_interface.h"
#include "rlc.h"

#include <netdb.h>
#include <sys/socket.h>
#include <fcntl.h>
#define MAX 80 
#define PORT 3456 
#define SA struct sockaddr 
#define MAX_NUMBER_eNB 10

#define DEBUG_eNB_SCHEDULER 1
#define DEBUG_HEADER_PARSING 1
//#define DEBUG_PACKET_TRACE 1

//#define ICIC 0

/*
  #ifndef USER_MODE
  #define msg debug_msg
  #endif
*/


/**********************************************
 * pktR globals used in the scheduler
 *
 **********************************************/
uint16_t g_last_rx_sn = 0;
int g_call_nb_getK = 0;
int g_call_tpc = 0;
int g_call_sched_ctrl = 0;
int tx_pow;
double value_K[NUMBER_OF_UE_MAX] = {10};
int ER[NUMBER_OF_UE_MAX] = {1};
double E_NI = 0, NI_tmp = 0, standardDeviation, sqr_target;
int converge[NUMBER_OF_UE_MAX] = {0};
uint32_t ue_pl = 0;
int32_t delta = 0;
double txpower = 1, txp = 1;
double meanRX, PL;
int32_t w_txpower = 1, mean_rx = 0, test;
int g_thread_call = 0;
int count_calls = 0, g_count_calls = 0;
int32_t current_NI_0 = 0;
uint32_t store_NI = 0;
int g_tmp = 0;
pthread_mutex_t mutex_tpc = PTHREAD_MUTEX_INITIALIZER;

//local signal map
Gain_map gain_map_eNB[NUMBER_OF_UE_MAX] = {0};
K_map k_map_eNB[NUMBER_OF_UE_MAX];
//Gain_map gain_map_UE[NUMBER_OF_eNB_MAX];
//K_map k_map_UE[NUMBER_OF_eNB_MAX];

//UCS
int UE_status[NUMBER_OF_UE_MAX][N_RBG_MAX];
int priority[NUMBER_OF_UE_MAX][N_RBG_MAX];
int conflict[NUMBER_OF_UE_MAX][NUMBER_OF_UE_MAX];
// This function stores the downlink buffer for all the logical channels
void store_dlsch_buffer (module_id_t Mod_id,
                         frame_t     frameP,
                         sub_frame_t subframeP)
{

  int                   UE_id,i;
  rnti_t                rnti;
  mac_rlc_status_resp_t rlc_status;
  UE_list_t             *UE_list = &eNB_mac_inst[Mod_id].UE_list;
  UE_TEMPLATE           *UE_template;

  for (UE_id=UE_list->head; UE_id>=0; UE_id=UE_list->next[UE_id]) {

    UE_template = &UE_list->UE_template[UE_PCCID(Mod_id,UE_id)][UE_id];

    // clear logical channel interface variables
    UE_template->dl_buffer_total = 0;
    UE_template->dl_pdus_total = 0;

    for(i=0; i< MAX_NUM_LCID; i++) {
      UE_template->dl_buffer_info[i]=0;
      UE_template->dl_pdus_in_buffer[i]=0;
      UE_template->dl_buffer_head_sdu_creation_time[i]=0;
      UE_template->dl_buffer_head_sdu_remaining_size_to_send[i]=0;
    }

    rnti = UE_RNTI(Mod_id,UE_id);

    for(i=0; i< MAX_NUM_LCID; i++) { // loop over all the logical channels

      rlc_status = mac_rlc_status_ind(Mod_id,rnti, Mod_id,frameP,ENB_FLAG_YES,MBMS_FLAG_NO,i,0 );
      UE_template->dl_buffer_info[i] = rlc_status.bytes_in_buffer; //storing the dlsch buffer for each logical channel
      UE_template->dl_pdus_in_buffer[i] = rlc_status.pdus_in_buffer;
      UE_template->dl_buffer_head_sdu_creation_time[i] = rlc_status.head_sdu_creation_time ;
      UE_template->dl_buffer_head_sdu_creation_time_max = cmax(UE_template->dl_buffer_head_sdu_creation_time_max,
          rlc_status.head_sdu_creation_time );
      UE_template->dl_buffer_head_sdu_remaining_size_to_send[i] = rlc_status.head_sdu_remaining_size_to_send;
      UE_template->dl_buffer_head_sdu_is_segmented[i] = rlc_status.head_sdu_is_segmented;
      UE_template->dl_buffer_total += UE_template->dl_buffer_info[i];//storing the total dlsch buffer
      UE_template->dl_pdus_total   += UE_template->dl_pdus_in_buffer[i];

#ifdef DEBUG_eNB_SCHEDULER

      /* note for dl_buffer_head_sdu_remaining_size_to_send[i] :
       * 0 if head SDU has not been segmented (yet), else remaining size not already segmented and sent
       */
      if (UE_template->dl_buffer_info[i]>0)
        LOG_D(MAC,
              "[eNB %d] Frame %d Subframe %d : RLC status for UE %d in LCID%d: total of %d pdus and size %d, head sdu queuing time %d, remaining size %d, is segmeneted %d \n",
              Mod_id, frameP, subframeP, UE_id,
              i, UE_template->dl_pdus_in_buffer[i],UE_template->dl_buffer_info[i],
              UE_template->dl_buffer_head_sdu_creation_time[i],
              UE_template->dl_buffer_head_sdu_remaining_size_to_send[i],
              UE_template->dl_buffer_head_sdu_is_segmented[i]
             );

#endif

    }

    //#ifdef DEBUG_eNB_SCHEDULER
    if ( UE_template->dl_buffer_total>0)
      LOG_D(MAC,"[eNB %d] Frame %d Subframe %d : RLC status for UE %d : total DL buffer size %d and total number of pdu %d \n",
            Mod_id, frameP, subframeP, UE_id,
            UE_template->dl_buffer_total,
            UE_template->dl_pdus_total
           );

    //#endif
  }
}


// This function returns the estimated number of RBs required by each UE for downlink scheduling
void assign_rbs_required (module_id_t Mod_id,
                          frame_t     frameP,
                          sub_frame_t subframe,
                          uint16_t    nb_rbs_required[MAX_NUM_CCs][NUMBER_OF_UE_MAX],
                          int         min_rb_unit[MAX_NUM_CCs])
{


  rnti_t           rnti;
  uint16_t         TBS = 0;
  LTE_eNB_UE_stats *eNB_UE_stats[MAX_NUM_CCs];
  int              UE_id,n,i,j,CC_id,pCCid,tmp;
  UE_list_t        *UE_list = &eNB_mac_inst[Mod_id].UE_list;
  //  UE_TEMPLATE           *UE_template;
  LTE_DL_FRAME_PARMS   *frame_parms[MAX_NUM_CCs];

  // clear rb allocations across all CC_ids
  for (UE_id=UE_list->head; UE_id>=0; UE_id=UE_list->next[UE_id]) {
    pCCid = UE_PCCID(Mod_id,UE_id);
    rnti = UE_list->UE_template[pCCid][UE_id].rnti;

    //update CQI information across component carriers
    for (n=0; n<UE_list->numactiveCCs[UE_id]; n++) {

      CC_id = UE_list->ordered_CCids[n][UE_id];
      frame_parms[CC_id] = mac_xface->get_lte_frame_parms(Mod_id,CC_id);
      eNB_UE_stats[CC_id] = mac_xface->get_eNB_UE_stats(Mod_id,CC_id,rnti);
      /*
      DevCheck(((eNB_UE_stats[CC_id]->DL_cqi[0] < MIN_CQI_VALUE) || (eNB_UE_stats[CC_id]->DL_cqi[0] > MAX_CQI_VALUE)),
      eNB_UE_stats[CC_id]->DL_cqi[0], MIN_CQI_VALUE, MAX_CQI_VALUE);
      */
      eNB_UE_stats[CC_id]->dlsch_mcs1=cqi_to_mcs[eNB_UE_stats[CC_id]->DL_cqi[0]];

      eNB_UE_stats[CC_id]->dlsch_mcs1 = cmin(eNB_UE_stats[CC_id]->dlsch_mcs1,openair_daq_vars.target_ue_dl_mcs);

    }

    // provide the list of CCs sorted according to MCS
    for (i=0; i<UE_list->numactiveCCs[UE_id]; i++) {
      for (j=i+1; j<UE_list->numactiveCCs[UE_id]; j++) {
        DevAssert( j < MAX_NUM_CCs );

        if (eNB_UE_stats[UE_list->ordered_CCids[i][UE_id]]->dlsch_mcs1 >
            eNB_UE_stats[UE_list->ordered_CCids[j][UE_id]]->dlsch_mcs1) {
          tmp = UE_list->ordered_CCids[i][UE_id];
          UE_list->ordered_CCids[i][UE_id] = UE_list->ordered_CCids[j][UE_id];
          UE_list->ordered_CCids[j][UE_id] = tmp;
        }
      }
    }

    /*
    if ((mac_get_rrc_status(Mod_id,1,UE_id) < RRC_RECONFIGURED)){  // If we still don't have a default radio bearer
      nb_rbs_required[pCCid][UE_id] = PHY_vars_eNB_g[Mod_id][pCCid]->lte_frame_parms.N_RB_DL;
      continue;
    }
    */
    /* NN --> RK
     * check the index of UE_template"
     */
    //    if (UE_list->UE_template[UE_id]->dl_buffer_total> 0) {
    if (UE_list->UE_template[pCCid][UE_id].dl_buffer_total> 0) {
      LOG_D(MAC,"[preprocessor] assign RB for UE %d\n",UE_id);

      for (i=0; i<UE_list->numactiveCCs[UE_id]; i++) {
        CC_id = UE_list->ordered_CCids[i][UE_id];
        frame_parms[CC_id] = mac_xface->get_lte_frame_parms(Mod_id,CC_id);
        eNB_UE_stats[CC_id] = mac_xface->get_eNB_UE_stats(Mod_id,CC_id,rnti);

        if (eNB_UE_stats[CC_id]->dlsch_mcs1==0) {
          nb_rbs_required[CC_id][UE_id] = 4;  // don't let the TBS get too small
        } else {
          nb_rbs_required[CC_id][UE_id] = min_rb_unit[CC_id];
        }

        TBS = mac_xface->get_TBS_DL(eNB_UE_stats[CC_id]->dlsch_mcs1,nb_rbs_required[CC_id][UE_id]);

        LOG_D(MAC,"[preprocessor] start RB assignement for UE %d CC_id %d dl buffer %d (RB unit %d, MCS %d, TBS %d) \n",
              UE_id, CC_id, UE_list->UE_template[pCCid][UE_id].dl_buffer_total,
              nb_rbs_required[CC_id][UE_id],eNB_UE_stats[CC_id]->dlsch_mcs1,TBS);

        /* calculating required number of RBs for each UE */
        while (TBS < UE_list->UE_template[pCCid][UE_id].dl_buffer_total)  {
          nb_rbs_required[CC_id][UE_id] += min_rb_unit[CC_id];

          if (nb_rbs_required[CC_id][UE_id] > frame_parms[CC_id]->N_RB_DL) {
            TBS = mac_xface->get_TBS_DL(eNB_UE_stats[CC_id]->dlsch_mcs1,frame_parms[CC_id]->N_RB_DL);
            nb_rbs_required[CC_id][UE_id] = frame_parms[CC_id]->N_RB_DL;
            break;
          }

          TBS = mac_xface->get_TBS_DL(eNB_UE_stats[CC_id]->dlsch_mcs1,nb_rbs_required[CC_id][UE_id]);
        } // end of while

        LOG_D(MAC,"[eNB %d] Frame %d: UE %d on CC %d: RB unit %d,  nb_required RB %d (TBS %d, mcs %d)\n",
              Mod_id, frameP,UE_id, CC_id,  min_rb_unit[CC_id], nb_rbs_required[CC_id][UE_id], TBS, eNB_UE_stats[CC_id]->dlsch_mcs1);
      }
    }
  }
}


// This function scans all CC_ids for a particular UE to find the maximum round index of its HARQ processes

int maxround(module_id_t Mod_id,uint16_t rnti,int frame,sub_frame_t subframe,uint8_t ul_flag )
{

  uint8_t round,round_max=0,UE_id;
  int CC_id;
  UE_list_t *UE_list = &eNB_mac_inst[Mod_id].UE_list;

  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {

    UE_id = find_UE_id(Mod_id,rnti);
    round    = UE_list->UE_sched_ctrl[UE_id].round[CC_id];
    if (round > round_max) {
      round_max = round;
    }
  }

  return round_max;
}

// This function scans all CC_ids for a particular UE to find the maximum DL CQI

int maxcqi(module_id_t Mod_id,int32_t UE_id)
{

  LTE_eNB_UE_stats *eNB_UE_stats = NULL;
  UE_list_t *UE_list = &eNB_mac_inst[Mod_id].UE_list;
  int CC_id,n;
  int CQI = 0;

  for (n=0; n<UE_list->numactiveCCs[UE_id]; n++) {
    CC_id = UE_list->ordered_CCids[n][UE_id];
    eNB_UE_stats = mac_xface->get_eNB_UE_stats(Mod_id,CC_id,UE_RNTI(Mod_id,UE_id));

    if (eNB_UE_stats==NULL) {
      mac_xface->macphy_exit("maxcqi: could not get eNB_UE_stats\n");
      return 0; // not reached
    }

    if (eNB_UE_stats->DL_cqi[0] > CQI) {
      CQI = eNB_UE_stats->DL_cqi[0];
    }
  }

  return(CQI);
}



// This fuction sorts the UE in order their dlsch buffer and CQI
void sort_UEs (module_id_t Mod_idP,
               int         frameP,
               sub_frame_t subframeP)
{


  int               UE_id1,UE_id2;
  int               pCC_id1,pCC_id2;
  int               cqi1,cqi2,round1,round2;
  int               i=0,ii=0;//,j=0;
  rnti_t            rnti1,rnti2;

  UE_list_t *UE_list = &eNB_mac_inst[Mod_idP].UE_list;

  for (i=UE_list->head; i>=0; i=UE_list->next[i]) {

    for(ii=UE_list->next[i]; ii>=0; ii=UE_list->next[ii]) {

      UE_id1  = i;
      rnti1 = UE_RNTI(Mod_idP,UE_id1);
      if(rnti1 == NOT_A_RNTI)
	continue;
      if (UE_list->UE_sched_ctrl[UE_id1].ul_out_of_sync == 1)
	continue;
      pCC_id1 = UE_PCCID(Mod_idP,UE_id1);
      cqi1    = maxcqi(Mod_idP,UE_id1); //
      round1  = maxround(Mod_idP,rnti1,frameP,subframeP,0);

      UE_id2 = ii;
      rnti2 = UE_RNTI(Mod_idP,UE_id2);
      if(rnti2 == NOT_A_RNTI)
        continue;
      if (UE_list->UE_sched_ctrl[UE_id2].ul_out_of_sync == 1)
	continue;
      cqi2    = maxcqi(Mod_idP,UE_id2);
      round2  = maxround(Mod_idP,rnti2,frameP,subframeP,0);  //mac_xface->get_ue_active_harq_pid(Mod_id,rnti2,subframe,&harq_pid2,&round2,0);
      pCC_id2 = UE_PCCID(Mod_idP,UE_id2);

      if(round2 > round1) { // Check first if one of the UEs has an active HARQ process which needs service and swap order
        swap_UEs(UE_list,UE_id1,UE_id2,0);
      } else if (round2 == round1) {
        // RK->NN : I guess this is for fairness in the scheduling. This doesn't make sense unless all UEs have the same configuration of logical channels.  This should be done on the sum of all information that has to be sent.  And still it wouldn't ensure fairness.  It should be based on throughput seen by each UE or maybe using the head_sdu_creation_time, i.e. swap UEs if one is waiting longer for service.
        //  for(j=0;j<MAX_NUM_LCID;j++){
        //    if (eNB_mac_inst[Mod_id][pCC_id1].UE_template[UE_id1].dl_buffer_info[j] <
        //      eNB_mac_inst[Mod_id][pCC_id2].UE_template[UE_id2].dl_buffer_info[j]){

        // first check the buffer status for SRB1 and SRB2

        if ( (UE_list->UE_template[pCC_id1][UE_id1].dl_buffer_info[1] + UE_list->UE_template[pCC_id1][UE_id1].dl_buffer_info[2]) <
             (UE_list->UE_template[pCC_id2][UE_id2].dl_buffer_info[1] + UE_list->UE_template[pCC_id2][UE_id2].dl_buffer_info[2])   ) {
          swap_UEs(UE_list,UE_id1,UE_id2,0);
        } else if (UE_list->UE_template[pCC_id1][UE_id1].dl_buffer_head_sdu_creation_time_max <
                   UE_list->UE_template[pCC_id2][UE_id2].dl_buffer_head_sdu_creation_time_max   ) {
          swap_UEs(UE_list,UE_id1,UE_id2,0);
        } else if (UE_list->UE_template[pCC_id1][UE_id1].dl_buffer_total <
                   UE_list->UE_template[pCC_id2][UE_id2].dl_buffer_total   ) {
          swap_UEs(UE_list,UE_id1,UE_id2,0);
        } else if (cqi1 < cqi2) {
          swap_UEs(UE_list,UE_id1,UE_id2,0);
        }
      }
    }
  }
}

/******* ONAMA functions *******/

void init_onama(int UE_status[][N_RBG_MAX],
		int priority[][N_RBG_MAX],
		int conflict[][NUMBER_OF_UE_MAX])
{
	int i, iii;

	for(i=0; i<NUMBER_OF_UE_MAX; i++) {
		for(iii=0; iii<N_RBG_MAX; iii++)
			priority[i][iii]=0;
	}

	for(i=0; i<NUMBER_OF_UE_MAX; i++) {
		 for(iii=0; iii<N_RBG_MAX; iii++)
	   	UE_status[i][iii]=0;
	}

	for(i=0; i<NUMBER_OF_UE_MAX; i++) {
		for(iii=0; iii<NUMBER_OF_UE_MAX; iii++)
			conflict[i][iii]=0;
	}
}

	
/****************************
 * Priority calculation
 ****************************/
int hash_priority(int index) 
{
  int pre_priority;
  pre_priority = index % 100;
  return pre_priority;
}

/********************************************************************
 * This function calculates the priority of each UE for each channel
 ********************************************************************/
void priority_UEs(int k,
		  int CC_id,
                  uint16_t dkmean)
{
  //fprintf(stdout, "[UCS][PRIORITY_UE] dkmean = %d\n", dkmean);
  int rb, ii = 0;
  int t_TTI=rand();
  int maxprio = 0;
  
  for (ii=0; ii<dkmean; ii++) {
    for (rb=0; rb<N_RBG_MAX; rb++) {  
      if ((hash_priority((k^ii^t_TTI^rb)^k^ii)) > maxprio) {
        maxprio = hash_priority((k^ii^t_TTI^rb)^k^ii);
      }
    }
  }
  priority[k][rb] = maxprio;
  fprintf(stdout, "[UCS]{priority_UEs} priority[%d][%d]=%d\n", k, CC_id, priority[k][rb]);

}


/*************************************************************************
 * This function builds the map/table of conflict nodes for each channel
 **************************************************************************/
void conflict_UEs(module_id_t  Mod_idP,
		int conflict[][NUMBER_OF_UE_MAX])
{
  int i=0;
  int ii=0;

  UE_list_t *UE_list = &eNB_mac_inst[Mod_idP].UE_list;

  for(i=UE_list->head; i>=0; i=UE_list->next[i]) {
    for (ii=UE_list->head; ii>=0; ii=UE_list->next[ii])
      // If DL CQI of UE_ii is greater than DL CQI/K_i of UE_i => UEs are interfering with each other
      if(maxcqi(Mod_idP,ii) >= maxcqi(Mod_idP,i) && i != ii)
      {
        conflict[i][ii]=1;
      }
    }
}



/*************************** PKTR *********************************
 * - pktR algorithm and associated functions
 * - 3 threads run in parallel
 * - refer to vars.h for structures and global variables
 ******************************************************************/



/********************** X2 CLIENT ********************/
void * clientThread(void *arg)
{
  int clientSocket;
  struct sockaddr_in servaddr, cli;
  char message[1000];
  char buffer[1024];
  
  // socket creation
  clientSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (clientSocket == -1) {
      perror("socket creation failed...\n");
      exit(1);
  } 
  else
     printf("Socket successfully created..\n");
  bzero(&servaddr, sizeof(servaddr));
  
  // assign IP, PORT
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(PORT);
  
  // connect the client socket to server socket
  if (connect(clientSocket, (SA*)&servaddr, sizeof(servaddr)) != 0) {
      perror("connection with the server failed...\n");
      exit(1);
  }
  else
      printf("connected to the server..\n");
  
  //Read the message from the server into the buffer
  if(recv(clientSocket, buffer, 1024, 0) < 0) {
    printf("Receive failed\n");
  }

  //Print the received message
  printf("Data received: %s\n",buffer);

  close(clientSocket);
  //pthread_exit(NULL);

}


void X2_TCP_client_eNB(module_id_t eNB_id)
{
  int i = 0;
  pthread_t tid[50];

  if( pthread_create(&tid[i], NULL, clientThread, NULL) != 0 )
    printf("Failed to create thread\n");
  
  pthread_join(tid[i++],NULL);

}
/********************* X2 SERVER **************************/
char x2_message[2000];
char buffer[1024];
pthread_mutex_t lock_cli = PTHREAD_MUTEX_INITIALIZER;

void * socketThread(void *arg)
{
  struct arg_struct_g *args = (struct arg_struct_g *)arg;
  //int newfd = *((int *)arg);
  fcntl(args->connfd, F_SETFL, O_NONBLOCK);
  recv(args->connfd, x2_message, 2000, 0);

  // Send message to the client socket 
  pthread_mutex_lock(&lock_cli);
  char *message = malloc(sizeof(x2_message)+20);
  strcpy(message,"X2 payload");
  strcat(message, x2_message);
  //strcat(message,"\n");
  strcpy(buffer, message);
  free(message);
  pthread_mutex_unlock(&lock_cli);
  //sleep(1);
  //
  // Convert double into string with sscanf
  send(args->connfd, buffer, 2, 0);
  printf("Exit socketThread \n");
  close(args->connfd);
  pthread_exit(NULL);
}

void X2_TCP_server_eNB(module_id_t eNB_id, double gain)
{
  int sockfd, last_fd, len;
  socklen_t addr_size;
  struct sockaddr_in servaddr, cli;
  struct sockaddr_storage serverStorage;
  char buffer[1024];
  char message[1000];
  pthread_t tid[MAX_NUMBER_eNB];

  g_args_t->gain = gain;

  fcntl(sockfd, F_SETFL, O_NONBLOCK);

  // socket creation
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      perror("socket creation failed...\n");
      exit(1);
  }
  else
      printf("Socket successfully created..\n");
  
  last_fd = sockfd;

  // Configure server addr
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(PORT);
  bzero(&(servaddr.sin_zero), 8);


  // Binding newly created socket
  if ((bind(sockfd, (SA*)&servaddr, sizeof(servaddr))) == -1) {
      perror("socket bind failed...\n");
      exit(1);
  }
  else
      printf("Socket successfully binded..\n");
  
  // Now server is ready to listen
  if ((listen(sockfd, 10)) != 0) {
      perror("Listen failed...\n");
      exit(1);
  } 
  else
      printf("Server listening..\n");
  //len = sizeof(cli);
  int i = 0;
  // Accept X2 data
  //while (1) {
    len = sizeof(cli);
    // new fd for each incoming connection
    int connfd = accept(sockfd, (SA*)&cli, &len);
    g_args_t->connfd = connfd;

    if (connfd == -1) {
      //perror("server acccept failed...\n");
        if (errno == EWOULDBLOCK) {
          printf("No pending connections; sleeping for one second.\n");
          sleep(1);
        } else {
          perror("error when accepting connection");
          //exit(1);
        }
    }
    else {
      if( pthread_create(&tid[i], NULL, socketThread, (void *)g_args_t) != 0 )
        perror("Failed to create pthread...");
      
      while (i < MAX_NUMBER_eNB) {
        pthread_join(tid[i++],NULL);
      }
      i = 0;
      //printf("server acccept the client...\n");
      //strcpy(message,"Hello");

      /*if( send(connfd, message, strlen(message), 0) < 0) {
        printf("Send failed\n");
      }*/

      // After chatting close the socket
      //close(connfd);
    }
  //}
  return EXIT_SUCCESS;
}




/********************
 * Share k map
 *******************/
void insert_k_entry(int UE_id, int K, module_id_t Mod_id)
{
  FILE *fp;
  int i = 0; //@TODO declare as global var for the multiple UEs use case
  // list node contents from first to last
  //map_entry_t *r;
  UE_list_t *UE_list = &eNB_mac_inst[Mod_id].UE_list;
  
  fp = fopen("/opt/pktr/context-cell-k", "w");
  if( fp == NULL ) {
    perror("Error while opening file");
    exit(1);
  }
  //for(i=UE_list->head; i>=0; i=UE_list->next[i]) {
  //fwrite (&smentry, sizeof(map_entry_t), 1, fp);
    //r = signal_map + i;
    fprintf(fp, "%d %d %d\n", UE_id, K, Mod_id);
    fprintf(stdout, "%d %d %d\n", UE_id, K, Mod_id);
	
    //if(UE_id == UE_list->head)
      //r->next = NULL;
    //else
      //r->next = r+1;
  //}
   
  fclose(fp);
}


/********************
 * Share signal map
 *******************/
void insert_map_entry(module_id_t Mod_id, map_entry_t smentry)
{
  FILE *fp;
  int i = 0; //@TODO declare as global var for the multiple UEs use case
  // list node contents from first to last
  //map_entry_t *r;
  UE_list_t *UE_list = &eNB_mac_inst[Mod_id].UE_list;
  
  fp = fopen("/opt/pktr/context-cell", "w");
  if( fp == NULL ) {
    perror("Error while opening file");
    exit(1);
  }
  //for(i=UE_list->head; i>=0; i=UE_list->next[i]) {
  //fwrite (&smentry, sizeof(map_entry_t), 1, fp);
    //r = signal_map + i;
    fprintf(fp, "%d %d %d %d\n", smentry.rnti, smentry.pathloss, smentry.mode, smentry.eNB_id);
    fprintf(stdout, "%d %d %d %d\n", smentry.rnti, smentry.pathloss, smentry.mode, smentry.eNB_id);
	
    //if(UE_id == UE_list->head)
      //r->next = NULL;
    //else
      //r->next = r+1;
  //}
   
  fclose(fp);
}


/********************
 * Update signal map
 *******************/
void read_map_entries(void)
{
  FILE *fp;
  char line[20];
  int rnti, enb, args, i=0, j;
  link_type_t ltype;
  uint32_t pl;
  
  fp = fopen("/opt/pktr/context-cell", "r");
  if( fp == NULL ) {
    perror("Error while opening file");
    exit(1);
  }
  
  // Read signal map from file context-rx
  while(fscanf(fp, "%d %d %d %d", &rnti, &pl, &ltype, &enb) == 4) {
          fprintf(stdout, "RNTI = %d : pathloss = %d link type = %d associated eNB = %d\n", signal_map[i].rnti, signal_map[i].pathloss, signal_map[i].mode = ltype, signal_map[i].eNB_id);
	  signal_map[i].rnti = rnti;
	  signal_map[i].pathloss = pl;
	  signal_map[i].mode = ltype;
	  signal_map[i].eNB_id = enb;
	  i++;   
  }

  fclose(fp);
}



int g_call_pdr = 1;
/***************************************
 * Computes reliability (PDR) of a link
 *
 ***************************************/

uint16_t reliability(module_id_t Mod_idP, int UE_id)
{
  int i, CC_id, ii;
  int count_sn = 1;
  uint16_t pdr = 0;
  uint16_t last_rx_sn = 0;
  FILE *fp;

  UE_list_t *UE_list = &eNB_mac_inst[Mod_idP].UE_list;
        
  eNB_MAC_INST *mac = &eNB_mac_inst[Mod_idP];
  
  rnti_t rnti_ue;

  rnti_ue = UE_RNTI(Mod_idP, UE_id);
  //fprintf(stdout, "[reliability] UE_RNTI = %x, UE_id = %d\n", rnti_ue, UE_id);

  // To estimate PDR: retrieve seq_num stored in PDCP hdr	
	
  for (i=0; i<UE_list->numactiveCCs[UE_id]; i++) { 
    CC_id=UE_list->ordered_CCids[i][UE_id];
    for(ii=0; ii < 100; ii++) {
      if(ii != 0 && mac->eNB_stats[CC_id].seqnum_long[ii][rnti_ue] != 0)
        count_sn++;
      if(ii!=0 && mac->eNB_stats[CC_id].seqnum_long[ii][rnti_ue] == 0 && mac->eNB_stats[CC_id].seqnum_long[ii-1][rnti_ue] != 0) {
        last_rx_sn = mac->eNB_stats[CC_id].seqnum_long[ii-1][rnti_ue];

        //fprintf(stdout, "++++++LAST RX SEQNUM[%d] == %d\n", ii-1, last_rx_sn);
        //fprintf(stdout, "++++++SEQNUM[%d] == %d\n", ii, mac->eNB_stats[CC_id].seqnum_long[ii]);
      }
    }
  }
  g_last_rx_sn = last_rx_sn;
  
  pdr = (uint16_t)floor((count_sn * 100) / (last_rx_sn + 1));
  
  //pdr = (uint16_t)floor(((last_rx_sn + 1)*100)/count_sn);
  //pdr = (uint16_t)floor( (100 * (last_rx_sn+1)) / count_sn );
  //fprintf(stdout, "Link reliability (PDR) of [UE %d] == %d\n", UE_id, pdr);
  //fprintf(stdout, "[reliability] count_sn = %d, last_rx_sn = %d\n", count_sn, last_rx_sn);
  /*fp = fopen("/opt/pktr/reliability.dat","a+");
  if(fp == NULL){
    perror("Error opening file");   
    exit(1);
  }*/

  //fprintf(fp, "%d %d\n", g_call_pdr, pdr);
  //fclose(fp);
  g_call_pdr++;

  return pdr; 
}


// Thread args structure for get_value_K_pktR()
struct arg_struct_getk {
  int UE_id;
  module_id_t Mod_id;
  uint8_t CC_id;
};

int ind_g = 0;
/****************************************************************************
 * This function is responsible for maintaining the PRK-model parameter K
 * Calls: reliability() that computes the PDR of a link based on traffic from previous n slots
 **********************************************************************************************/

//void * get_value_K_pktR(void *params)
void get_value_K_pktR(int UE_id, module_id_t Mod_id, uint8_t CC_id)
{
  //struct arg_struct_getk *args = (struct arg_struct_getk *)params;
  FILE *f;
  int i, j, C, neighb, min_rssi_id, tmp_ind;
  int32_t tmp, sinr_ue;
  double I_R, f1, NI_0 = 0;
  double G_SR = 0, K_SR = 1, G_CR = 0;
  double c = (15/16);
  double a_result, param_erf, value_K_now, sum = 0;
  UE_list_t *UE_list = &eNB_mac_inst[Mod_id].UE_list;
  typedef struct rssi_sort {
    int32_t power;
    int index;
  }rssi_sort_t;
  rssi_sort_t rssi[MAX_MOBILES_PER_ENB], rssi_dec[MAX_MOBILES_PER_ENB];

  pthread_mutex_lock(&mutex_tpc);

  //Initialization
  ind_g = 0;

  //char Sinr = UE_mac_inst[Mod_idP].Def_meas[0].Wideband_sinr;
  /*f = fopen("/opt/pktr/meas-rx", "r");
  if( f == NULL ) {
    perror("Error while opening file");
    exit(1);
  }
  fscanf(f, "%d", &sinr_ue);
  //fprintf(stdout, "[COMPUTE_K] sinr = %f\n", sinr);
  fclose(f);*/


  int sinr_g = PHY_vars_UE_g[UE_id][CC_id]->PHY_measurements.wideband_cqi_avg[UE_id];
  //int sinr_t = PHY_vars_eNB_g[args->UE_id][args->CC_id]->PHY_measurements_eNB->wideband_cqi_dB[args->UE_id][0];

  //fprintf(stdout, "[COMPUTE K] sinr_g = %d, sinr_t = %d\n", sinr_g, sinr_t);

  //sinr_ue = -20;

  // Compute inverse of error function f1
  double gamma = (TARGET_RE * 0.01);
  //fprintf(stdout, "[COMPUTE_K] gamma = %f\n", gamma);
  f1 = erf(gamma);
  //fprintf(stdout, "[COMPUTE_K] f1 = %f\n", f1);
  f1 = (1/f1);
  //fprintf(stdout, "[COMPUTE_K] f1 after div = %f\n", f1);
  //fprintf(stdout, "[COMPUTE_K] delta_NI = %d\n", delta);
  // Sum of C
  //for (C=UE_list->head; C>=0; C=UE_list->next[C]){
  int conctx_total = 0;

  fprintf(stdout, "GTMP = %d\n", g_tmp);
  for (C=0; C < g_tmp; C++) {
    if (ER[C] == 1 && C != UE_id){
        sum++;
   // if (UE_mac_inst[C].ue_tx_ind == 1 && ER[C] == 0)
     //     conctx_total++;
    }
    //rssi[C].power = UE_list->eNB_UE_stats[args->CC_id][C].normalized_rx_power;
    rssi[C].power = PHY_vars_UE_g[C][CC_id]->PHY_measurements.rx_power_avg_dB[0];
    rssi[C].index = C;
    //rssi_dec[C].power = UE_list->eNB_UE_stats[args->CC_id][C].normalized_rx_power;
    rssi_dec[C].power = PHY_vars_UE_g[C][CC_id]->PHY_measurements.rx_power_avg_dB[0];
    rssi_dec[C].index = C;
  }
  //fprintf(stdout, "TOTAL NUMBER OF CONCURRENT TX = %d\n", conctx_total);
  fprintf(stdout, "SUM = %f\n", sum);

  // Sort RSSI in increasing order
  for(i=0; i<=C; ++i) {
    for(j=i+1; j<=C; ++j){
      if(rssi[i].power > rssi[j].power){
        tmp = rssi[i].power;
        tmp_ind = rssi[i].index;
        rssi[i].power = rssi[j].power;
        rssi[i].index = rssi[j].index;
        rssi[j].power = tmp;
        rssi[j].index = tmp_ind;
      }
    }
    //fprintf(stdout, "[SORTRSSI] rssi[%d].power = %d\n", i, rssi[i].power);
    //fprintf(stdout, "[SORTRSSI] rssi[%d].index = %d\n", i, rssi[i].index);
  }
  // Sort RSSI in decreasing order
  for(i=0; i<=C; ++i) {
    for(j=i+1; j<=C; ++j){
      if(rssi_dec[i].power < rssi_dec[j].power){
        tmp = rssi_dec[i].power;
        tmp_ind = rssi_dec[i].index;
        rssi_dec[i].power = rssi_dec[j].power;
        rssi_dec[i].index = rssi_dec[j].index;
        rssi_dec[j].power = tmp;
        rssi_dec[j].index = tmp_ind;
      }
    }
  }

  // Converge
  i = 0;

  NI_0 = (double)(mean_rx - (standardDeviation * sqr_target) - TARGET_SINR);
  //fprintf(stdout, "[COMPUTE K] NI_0 == %f\n", NI_0);

  if(fabs(E_NI) == ceil(NI_0)) {
    converge[UE_id] = 1;
  }
  
  if (fabs(E_NI) != ceil(NI_0)) {
     
     // Compute NI_0
     NI_0 = (double)(mean_rx - (standardDeviation * sqr_target) - TARGET_SINR);
     // Compute a(t)
     a_result = (double)((TARGET_RE - reliability(Mod_id, UE_id)) / (f1 - sinr_g));
     //fprintf(stdout, "[COMPUTE K] a(t) == %f\n", a_result);
     //fprintf(stdout, "[COMPUTE_K] f1 = %f\n", f1);
     // Compute Delta(I_R)
     I_R = ((((1+c) * reliability(Mod_id, UE_id)) - (c * reliability(Mod_id, UE_id)) - TARGET_RE) / (1 - c) * a_result);
     //fprintf(stdout, "[COMPUTE K] I_R == %f\n", I_R);     

     if( delta < 0){
     //  if (E_NI - NI_0){
       ind_g = 1;
       fprintf(stdout, "[COMPUTE K] Expand exclusion region\n");
       fprintf(stdout, "[COMPUTE K] E[NI] = %f\n", fabs(E_NI));
       fprintf(stdout, "[COMPUTE K] delta = %d\n", delta);
       fprintf(stdout, "[COMPUTE K] converged = %d\n", converge[UE_id]);
       fprintf(stdout, "[COMPUTE K] NI_0 = %f and ceil(fabs(I_R)) = %f\n", NI_0, ceil(fabs(I_R)));
       // Expand exclusion region by increasing K 
       // Add a new node to exclusion region in non-increasing order of P(C_i, R)
       if (sum < ceil(fabs(I_R)) && sum != 0 && g_tmp >= ceil(fabs(I_R))){
         if(ER[rssi[i].index] == 0){
           ER[rssi[i].index] = 1;
           sum++;
           i++;
         }
       }

      } else {
        ind_g = 2;
        fprintf(stdout, "[COMPUTE K] Shrink exclusion region\n");
	fprintf(stdout, "[COMPUTE K] converge = %d\n", converge[UE_id]);
	fprintf(stdout, "[COMPUTE K] delta = %d\n", delta);
	fprintf(stdout, "[COMPUTE K] E[NI] = %f\n", fabs(E_NI));
	fprintf(stdout, "[COMPUTE K] I_R = %f\n", I_R);
	fprintf(stdout, "[COMPUTE K] NI_0 = %f and ceil(fabs(I_R)) = %f\n", NI_0, (double)ceil(fabs(I_R)));
        // Shrink exclusion region by increasing K
        // Remove a new node from exclusion region in non-decreasing order of P(C_i, R)
        if (sum <= ceil(fabs(I_R)) && sum != 0 && g_tmp >= ceil(fabs(I_R))){
          if(ER[rssi_dec[i].index] == 1){
            ER[rssi_dec[i].index] = 0;
            sum++;
            i++;
          }
        }
      }

      fprintf(stdout, "i of C_i = %d\n", i);
      // Update K
      // @TODO To refactor with a switch/case

      if(g_call_nb_getK == 0) {
        value_K[UE_id] = 10;
      } else {
        switch (ind_g)
        {
          case 1: //Expand
            value_K_now = ( PHY_vars_UE_g[UE_id][CC_id]->PHY_measurements.path_loss[Mod_id] / PHY_vars_UE_g[rssi[i].index][CC_id]->PHY_measurements.path_loss[Mod_id] );
            //if (value_K_now < value_K[args->UE_id]) {
              value_K[UE_id] = value_K_now;
            //}
            break;

          case 2: //Shrink
            value_K_now = ( PHY_vars_UE_g[UE_id][CC_id]->PHY_measurements.path_loss[Mod_id] / PHY_vars_UE_g[rssi_dec[i].index][CC_id]->PHY_measurements.path_loss[Mod_id] );
            //if (value_K_now < value_K[args->UE_id]) {
              value_K[UE_id] = value_K_now;
            //}
            break;

          default:
            break;
        }
      }
      //value_K[args->UE_id] = value_K_now;
   /*   if (ind_g == 1) {
        if(g_call_nb_getK != 0 && PHY_vars_UE_g[rssi[i].index][args->CC_id]->PHY_measurements.path_loss != 0) {
          value_K_now = ceil( PHY_vars_UE_g[args->UE_id][args->CC_id]->PHY_measurements.path_loss / PHY_vars_UE_g[rssi[i].index][args->CC_id]->PHY_measurements.path_loss );
          //fprintf(stdout, "VALUEKNOW == %f\n", value_K_now);
          //value_K[args->UE_id] = cmin(value_K[args->UE_id], value_K_now);
          //if (value_K_now < value_K[args->UE_id]) {
            value_K[args->UE_id] = value_K_now;
          //}
        } else {
          value_K[args->UE_id] = 10;
        }
      } else {
        if(g_call_nb_getK != 0 && PHY_vars_UE_g[rssi_dec[i].index][args->CC_id]->PHY_measurements.path_loss != 0) {
          value_K_now = ceil( PHY_vars_UE_g[args->UE_id][args->CC_id]->PHY_measurements.path_loss / PHY_vars_UE_g[rssi_dec[i].index][args->CC_id]->PHY_measurements.path_loss );
          //fprintf(stdout, "VALUEKNOW == %f\n", value_K_now);
          //if (value_K_now < value_K[args->UE_id]) {
            value_K[args->UE_id] = value_K_now;
          //}
        } else {
          value_K[args->UE_id] = 10;
        }
      }*/
      
      fprintf(stdout, "[COMPUTE_K] value_K[UE %d] of eNB %d = %f\n", UE_id, value_K[UE_id], Mod_id);
      
  } else {
    //fprintf(stdout, "IT JUST CONVERGED !!!!!\n");
    converge[UE_id] = 1;
  }

  /************************
   *  Interference model
   ************************/

  fprintf(stdout, "[GRK] PATHLOSS eNB %d to UE %d = %f\n", Mod_id, UE_id, PHY_vars_UE_g[UE_id][CC_id]->PHY_measurements.path_loss[Mod_id]);

  G_SR = PHY_vars_UE_g[UE_id][CC_id]->PHY_measurements.path_loss[Mod_id];
  fprintf(stdout, "GSR == %f\n", G_SR);
  K_SR = value_K[UE_id];
  fprintf(stdout, "KSR = %f\n", K_SR);
  for (C=UE_list->head; C>=0; C=UE_list->next[C]){
    if ((C != UE_id) && (PHY_vars_UE_g[C][CC_id]->PHY_measurements.associated_eNB_index == Mod_id)) {
      G_CR = PHY_vars_UE_g[C][CC_id]->PHY_measurements.path_loss[Mod_id];
      fprintf(stdout, "[interference] G_CR = %f, GSR/KSR = %f\n", G_CR, (G_SR/K_SR));
      if (G_CR >= (G_SR/K_SR)) {
        ER[C] = 1;
      } else {
        ER[C] = 0;
      }
      fprintf(stdout,"[exclusion] ER[%d] = %d\n", C, ER[C]);
    }
  }

  g_call_nb_getK++;

  /*******************
   *  Share K values *
   *******************/
  // X2: create k_entry_t
  uint8_t sm_index = 0;
  fprintf(stdout, "[SignalMap UL]     UE_id     Gain_C,R(t)     K_C,R(t)\n");
  //Uplink
  for (C=UE_list->head; C>=0; C=UE_list->next[C]) {   	
    if (ER[C] == 1 && C != UE_id) {
      gain_map_eNB[UE_id].C_id = C; //C_i 
	    gain_map_eNB[UE_id].G_CR[sm_index] = PHY_vars_UE_g[C][0]->PHY_measurements.path_loss[Mod_id]; ///mutual gain 
	    k_map_eNB[UE_id].value_K[sm_index] = value_K[C];
	    fprintf(stdout, "     C_%d             %d         %.2f         %.2f\n", sm_index, gain_map_eNB[UE_id].C_id, gain_map_eNB[UE_id].G_CR[sm_index], k_map_eNB[UE_id].value_K[sm_index]);
	    sm_index++;   
      //insert_k_entry(C, value_K[C], args->Mod_id);
    }
	}
  if (gain_map_eNB[UE_id].C_id != 0) {
    //X2_TCP_server_eNB(Mod_id, gain_map_eNB[UE_id].G_CR[sm_index]);
    //X2_TCP_client_eNB(Mod_id);
  }

	fprintf(stdout, "Target Receipient: ");
	for (i=0; i<NB_eNB_INST; i++) {
		if ((PHY_vars_eNB_g[Mod_id][0]->signalmap_candidate[i] == 1) && ( Mod_id != i ))
  		fprintf(stdout, "eNB[%d] ", i);
	}
  fprintf(stdout, "\n");
 

/* //Downlink
  G_SR = PHY_vars_UE_g[UE_id][CC_id]->PHY_measurements.path_loss[Mod_id];
  for (C=0; C<NB_eNB_INST; C++) {
	if (C != Mod_id) {
	  G_CR = PHY_vars_UE_g[UE_id][CC_id]->PHY_measurements.path_loss[C]; 
	  
	}
	
} */

  // Find UE that belongs to current eNB
  ///rnti_ue = UE_RNTI(args->Mod_id, args->UE_id);
  //int8_t UE_id = find_ue(rnti,PHY_vars_eNB_g[Mod_id][CC_id]);

  pthread_mutex_unlock(&mutex_tpc);

}


// Thread args structure
struct arg_struct_tpc {
  //etheraddr_t ue_mac;
  int UE_id;
  uint8_t CC_id;
  module_id_t Mod_id;
};

//#define BUFFER_SIZE 10
int32_t txp_buffer[50];
/******************************
 *   Compute Min Tx Power     *
 *****************************/

void *process_tpc_pktR (void *params)
{
  FILE *fp, *fp_stats_tpc;
  //char buffer[BUFFER_SIZE];
  int ue, i;
  int tmp = 0;
  struct arg_struct_tpc *args = (struct arg_struct_tpc *)params;
  UE_list_t *UE_list = &(eNB_mac_inst[args->Mod_id].UE_list);

  pthread_mutex_lock(&mutex_tpc);


  g_tmp = 0;
  for (ue = UE_list->head; ue >= 0; ue = UE_list->next[ue]) {
    tmp++;
  }
  g_tmp = tmp; //quick fix


  fprintf(stdout, "NB OF UEs = %d, NUMBER_OF_UE_MAX=%d\n", g_tmp, NUMBER_OF_UE_MAX);

  meanRX = (mean_rx * 1.0);
  txp = txpower;
  PL = (txp - meanRX);
  // Compute Tx Power
  txpower = (E_NI + meanRX + (standardDeviation * sqr_target) + TARGET_SINR);

  /*if(FRAC){
    

  }*/
  
  fprintf(stdout, "*******TPC precompute (dBm) => %f dBm\n", (txpower + txp));
  w_txpower = (int32_t)ceil((txpower + txp));
  fprintf(stdout, "*******TPC (dBm) => %d dBm for %d\n", w_txpower, args->UE_id);
  fp = fopen("/opt/pktr/txpower-tx","w");
  if(fp == NULL){
    perror("Error opening file");
    exit(1);
  }

  txp_buffer[args->UE_id] = w_txpower;

  for (i=0; i < g_tmp; i++) {
    fprintf(fp, "%d ", txp_buffer[i]);
  }

  fclose(fp);

  /***** Stats TPC *****/
  /*fp_stats_tpc = fopen("/opt/pktr/tpc-stats.dat","a+");
  if(fp_stats_tpc == NULL){
    perror("Error opening file");
    exit(1);
  }

  fprintf(fp_stats_tpc, "%d ", g_call_tpc);
  for (i=0; i < g_tmp; i++) {
    fprintf(fp_stats_tpc, "%d ", txp_buffer[i]);
  }
  fprintf(fp_stats_tpc, "\n");

  fclose(fp_stats_tpc);
*/
  //Allocate HERE
  //PHY_vars_UE_g[args->UE_id][args->CC_id]->tx_power_dBm = txp_buffer[args->UE_id]; 
  //fprintf(stdout, "[TPC] UE tx_power_dBm = %f\n", PHY_vars_UE_g[args->UE_id][0]);

  g_call_tpc++;
  
  
  // Send L2 frame with txpower
  //send_txpower(args->ue_mac, w_txpower);

  pthread_mutex_unlock(&mutex_tpc);
}


// Thread args structure
struct arg_struct_meas {
  int UE_id;
  module_id_t Mod_id;
  uint8_t CC_id;
  frame_t frameP;
  sub_frame_t subframeP;
};

/************************************
 * pktR measurements
 * Get periodical measurements
 ************************************/
void *get_meas_pktR (void *params)
{
  FILE *fp, *fpl;
  struct arg_struct_meas *args = (struct arg_struct_meas *)params;

  pthread_mutex_lock(&mutex_tpc);
  count_calls++;
  //count_c_tmp = count_calls;
  //printf("reliability count calls = %d\n", count_calls);
  int sigma = 1, i;
  //uint16_t last_rx_sn_tmp;
  int32_t rx_power, current_NI_1, new_mean_NI, diff_NI, delta_NI = 0;
  double standardDeviationPre;
  //rnti_t rnti_ue;

  UE_list_t *UE_list = &eNB_mac_inst[args->Mod_id].UE_list;
  UE_sched_ctrl   *UE_sched_ctrl = &UE_list->UE_sched_ctrl[args->UE_id];

  //PHY_VARS_UE *UE = PHY_vars_UE_g[args->UE_id][args->CC_id];

  // Get PHY measurements: background noise + interference power
  // Assumptiom: NI = RSSI
  //@TODO current_NI = MeasureNIRxSide();


  /*if(PHY_vars_eNB_g[args->Mod_id][0]->sub_rx_ind == 1)
    fprintf(stdout, "[RXSUBFRAME] START RX SUBFRAME\n");
  else
    fprintf(stdout, "[RXSUBFRAME] NO SUBFRAME BEING RX\n");
*/

  // First round to get samples
  if (count_calls == 1) {
    ////current_NI_0 = UE_list->eNB_UE_stats[args->CC_id][args->UE_id].normalized_rx_power;
    current_NI_0 = PHY_vars_UE_g[args->UE_id][args->CC_id]->PHY_measurements.rx_power_avg_dB[0];
    g_count_calls = 0;
    store_NI = 0;
  }

  // Second round to get samples
  if (count_calls == 100) {

    //fprintf(stdout, "[pktr] TEST RSSI UE[%d] = %d\n", args->UE_id, PHY_vars_UE_g[args->UE_id][args->CC_id]->PHY_measurements.rx_power_avg_dB[0]);

    //current_NI_1 = UE_list->eNB_UE_stats[args->CC_id][args->UE_id].normalized_rx_power;
    current_NI_1 = PHY_vars_UE_g[args->UE_id][args->CC_id]->PHY_measurements.rx_power_avg_dB[0];
    fprintf(stdout, "[pktR] RSSI_1 = %d\n", current_NI_1);
    fprintf(stdout, "[pktR] RSSI_0 = %d\n", current_NI_0);

    //decoding
    //if(phy_vars_eNB_g->sub_rx_decoded_ind == 1)
      //fprintf(stdout, "[DECODE] RX SUBFRAME DECODED\n");
   /* if(PHY_vars_eNB_g[args->Mod_id][0]->sub_rx_ind == 1)
      fprintf(stdout, "[RXSUBFRAME] START RX SUBFRAME\n");
    else
      fprintf(stdout, "[RXSUBFRAME] NO SUBFRAME BEING RX\n");
*/
    // Mean rx power
    mean_rx = ((current_NI_0 + current_NI_1)/2);

    // Get NI samples + compute standard deviation of NI
    store_NI = store_NI + (mean_rx);

    g_count_calls++;
    new_mean_NI = (store_NI/g_count_calls);
    //new_mean_NI = fabs(new_mean_NI);
    for (i=0; i<g_count_calls; i++) {
      standardDeviationPre += (double)pow((current_NI_1 - new_mean_NI), 2);
    }
    fprintf(stdout, "[pktR] standardDeviationPre = %lf\n", standardDeviationPre);
    standardDeviation = (double)sqrtf(standardDeviationPre/g_count_calls);
    fprintf(stdout, "[pktR] standardDeviationNEW = %lf\n", standardDeviation);

    // Mean rx power
    //mean_rx = ((current_NI_0 + current_NI_1)/2);

    diff_NI = (current_NI_0 - current_NI_1);
    fprintf(stdout, "[pktR] RSSI0 - RSSI1 = %d\n", diff_NI);
    //fprintf(stdout, "[pktR] PKTR_RSSI PHY UE = %3.1f dBm\n", UE_list->eNB_UE_stats[CC_id][UE_id].pktr_rssi);


    fprintf(stdout, "[pktR] E[NI] = %u\n", mean_rx);

    double target = (double)(1 - TARGET_RE);
    target = target * -1;
    double div_target = (double)(TARGET_RE/target);
    //fprintf(stdout, "[COMPUTE NI] div == %f\n", div_target);
    sqr_target = (double)sqrtf(div_target);
    sqr_target = sqr_target * -1;
    fprintf(stdout, "[COMPUTE NI] test == %f\n", sqr_target);
    //NI_tmp = (new_mean_NI - (standardDeviation * (uint32_t)sqrt((double)TARGET_RE/(1-TARGET_RE))) - TARGET_SINR);
    NI_tmp = (double)(mean_rx - (standardDeviation * sqr_target) - TARGET_SINR);
    fprintf(stdout, "[COMPUTE NI] NI_0_tmp == %f\n", NI_tmp);


    //Compute delta between RSSI_0 and RSSI_1
    delta = delta_NI = (mean_rx - (current_NI_1));
    fprintf(stdout, "[pktR] DELTA_NI = %d\n", delta_NI);

    // Compute Path Loss (does not work)
    //int pCC_id = UE_PCCID(Mod_id, UE_id);
    //int32_t tx_power_ue = UE_list->UE_template[pCC_id][UE_id].ue_tx_power;
    //fprintf(stdout, "[TEST] PATHLOSS TEMPLATE == %u dB\n", get_PL(Mod_id,0,0));
    
    ue_pl = (w_txpower - mean_rx);

    /********************/
    /*** Get Pathloss ***/
    /********************/
    /*fpl = fopen("/opt/pktr/pl_rx", "r");
    if( fpl == NULL ) {
      perror("Error while opening file");
      exit(1);
    }
    fscanf(fpl, "%d", &ue_pl);
    fclose(fpl);*/
    fprintf(stdout, "[pktR] Desired TxPower = %d, Pathloss UE [%d] = %d\n", tx_pow, args->UE_id, ue_pl);

    // X2: create sm_entry_t
  //  map_entry_t entry = {args->UE_id, ue_pl, m_cellular, args->Mod_id, 0};
    // Call share and update signal maps functions (X2)
    // insert_map_entry(args->Mod_id, entry);
    // read_map_entries();

    fprintf(stdout, "Here is something: %f\n", ((standardDeviation * sqr_target) + TARGET_SINR));
    E_NI = (new_mean_NI * -1.0);
    
    //rnti_ue = UE_RNTI(Mod_id, UE_id);
    // Schedule next transmission with new tx power level
    //schedule_ulsch_DCI0(Mod_id, rnti_ue, frameP, subframeP, 1);

  
    //char Sinr = UE_mac_inst[Mod_idP].Def_meas[0].Wideband_sinr;
    //printf("Wideband SINR == %c\n", UE_mac_inst[Mod_idP].Def_meas[0].Wideband_sinr);
    count_calls = 0;

  }

  pthread_mutex_unlock(&mutex_tpc);

}


/************************************
 * pktR scheduling controller
 * Maintains one K for each channel
 *  
 ************************************/

void sched_controller_pktR(int UE_id, module_id_t Mod_id, uint8_t CC_id, frame_t frameP, sub_frame_t subframeP)
{
  int rc;
  pthread_t thread_meas, thread_tpc, thread_getk;
  //struct if_nameindex *if_nidxs, *intf;
  //int i=0;

  struct arg_struct_meas *l_args = malloc(sizeof(struct arg_struct_meas));
  l_args->UE_id = UE_id;
  l_args->Mod_id = Mod_id;
  l_args->CC_id = CC_id;
  l_args->frameP = frameP;
  l_args->subframeP = subframeP;
  
  //struct arg_struct_getk *l_args_k = malloc(sizeof(struct arg_struct_getk));
  //Fill in arg struct
  //l_args_k->UE_id = UE_id;
  //l_args_k->Mod_id = Mod_id;
  //l_args_k->CC_id = CC_id;

  struct arg_struct_tpc *l_args_t = malloc(sizeof(struct arg_struct_tpc));
  l_args_t->UE_id = UE_id;
  l_args->CC_id = CC_id;
  l_args_t->Mod_id = Mod_id;


  if(pthread_create(&thread_meas, NULL, get_meas_pktR, (void *)l_args) != 0) {
    perror("Error creating pthread");
    exit(1);
  }

  if(pthread_create(&thread_tpc, NULL, process_tpc_pktR, (void *)l_args_t) != 0) {
    perror("Error creating pthread");
    exit(1);
  }

  if(PHY_vars_UE_g[UE_id][CC_id]->PHY_measurements.associated_eNB_index == Mod_id){
  fprintf(stdout, "\n************************K Adaptation eNB[%d] UE[%d]**********************************\n", Mod_id, UE_id);
  get_value_K_pktR(UE_id, Mod_id, CC_id);
  usleep(10000);
  fprintf(stdout, "\n************************End K Adaptation at Frame %d SubFrame %d*********************************\n", frameP, subframeP);
  }
  //if(pthread_create(&thread_getk, NULL, get_value_K_pktR, (void *)l_args_k) != 0) {
    //perror("Error creating pthread");
    //exit(1);
  //}

  pthread_join(thread_meas, NULL);
  pthread_join(thread_tpc, NULL);
  //pthread_join(thread_getk, NULL);
  free(l_args);
  free(l_args_t);
  //free(l_args_k);

  g_call_sched_ctrl++;

}





// This function assigns pre-available RBS to each UE in specified sub-bands before scheduling is done
void dlsch_scheduler_pre_processor (module_id_t   Mod_id,
                                    frame_t       frameP,
                                    sub_frame_t   subframeP,
                                    int           N_RBG[MAX_NUM_CCs],
                                    int           *mbsfn_flag)
{

  unsigned char rballoc_sub[MAX_NUM_CCs][N_RBG_MAX],harq_pid=0,round=0,total_ue_count;
  unsigned char MIMO_mode_indicator[MAX_NUM_CCs][N_RBG_MAX];
  int                     UE_id, i, n; 
  uint16_t                ii,j;
  uint16_t                nb_rbs_required[MAX_NUM_CCs][NUMBER_OF_UE_MAX];
  uint16_t                nb_rbs_required_remaining[MAX_NUM_CCs][NUMBER_OF_UE_MAX];
  uint16_t                nb_rbs_required_remaining_1[MAX_NUM_CCs][NUMBER_OF_UE_MAX];
  uint16_t                available_rbs[MAX_NUM_CCs]; // ONAMA
  uint16_t                average_rbs_per_user[MAX_NUM_CCs] = {0};
  rnti_t             rnti;
  int                min_rb_unit[MAX_NUM_CCs];
  uint16_t r1=0;
  uint8_t CC_id;
  UE_list_t *UE_list = &eNB_mac_inst[Mod_id].UE_list;
  LTE_DL_FRAME_PARMS   *frame_parms[MAX_NUM_CCs] = {0};

  int transmission_mode = 0;
  UE_sched_ctrl *ue_sched_ctl;
  //  int rrc_status           = RRC_IDLE;


#ifdef TM5
  int harq_pid1=0,harq_pid2=0;
  int round1=0,round2=0;
  int UE_id2;
  uint16_t                i1,i2,i3;
  rnti_t             rnti1,rnti2;
  LTE_eNB_UE_stats  *eNB_UE_stats1 = NULL;
  LTE_eNB_UE_stats  *eNB_UE_stats2 = NULL;
  UE_sched_ctrl *ue_sched_ctl1,*ue_sched_ctl2;
#endif


  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {

    if (mbsfn_flag[CC_id]>0)  // If this CC is allocated for MBSFN skip it here
      continue;

    frame_parms[CC_id] = mac_xface->get_lte_frame_parms(Mod_id,CC_id);


    min_rb_unit[CC_id]=get_min_rb_unit(Mod_id,CC_id);

    // ONAMA init
    init_onama(UE_status, priority, conflict);

    for (i=UE_list->head; i>=0; i=UE_list->next[i]) {
      UE_id = i;
      // Initialize scheduling information for all active UEs
      


      dlsch_scheduler_pre_processor_reset(Mod_id,
        UE_id,
        CC_id,
        frameP,
        subframeP,
        N_RBG[CC_id],
        nb_rbs_required,
        nb_rbs_required_remaining,
        rballoc_sub,
        MIMO_mode_indicator);

    }
  }

  // Get the conflict relationship
  conflict_UEs(Mod_id, conflict);
  // Store the DLSCH buffer for each logical channel
  store_dlsch_buffer (Mod_id,frameP,subframeP);



  // Calculate the number of RBs required by each UE on the basis of logical channel's buffer
  assign_rbs_required (Mod_id,frameP,subframeP,nb_rbs_required,min_rb_unit);



  // Sorts the user on the basis of dlsch logical channel buffer and CQI
  sort_UEs (Mod_id,frameP,subframeP);

  ///// PKTR is called here! /////
  if (PKTR) {
    for (UE_id = UE_list->head; UE_id >= 0; UE_id = UE_list->next[UE_id]) {
      reliability(Mod_id, UE_id);
      for (n = 0; n < UE_list->numactiveCCs[UE_id]; n++) {
        sched_controller_pktR(UE_id, Mod_id, n, frameP, subframeP);
      }
    }
  }
  /////////////////////////////////

  total_ue_count =0;
  int conc_tx=0;

  // loop over all active UEs
  for (i=UE_list->head; i>=0; i=UE_list->next[i]) {
    rnti = UE_RNTI(Mod_id,i);

    //if (UE_mac_inst[i].ue_tx_ind == 1 && ER[i] == 0)
      //conc_tx++;

    if(rnti == NOT_A_RNTI)
      continue;
    if (UE_list->UE_sched_ctrl[i].ul_out_of_sync == 1)
      continue;
    UE_id = i;

    // if there is no available harq_process, skip the UE
    if (UE_list->UE_sched_ctrl[UE_id].harq_pid[CC_id]<0)
      continue;

    for (ii=0; ii<UE_num_active_CC(UE_list,UE_id); ii++) {
      CC_id = UE_list->ordered_CCids[ii][UE_id];
      ue_sched_ctl = &UE_list->UE_sched_ctrl[UE_id];
      harq_pid = ue_sched_ctl->harq_pid[CC_id];
      round    = ue_sched_ctl->round[CC_id];

      average_rbs_per_user[CC_id]=0;

      frame_parms[CC_id] = mac_xface->get_lte_frame_parms(Mod_id,CC_id);

      //      mac_xface->get_ue_active_harq_pid(Mod_id,CC_id,rnti,frameP,subframeP,&harq_pid,&round,0);

      if(round>0) {
        nb_rbs_required[CC_id][UE_id] = UE_list->UE_template[CC_id][UE_id].nb_rb[harq_pid];
      }

      //nb_rbs_required_remaining[UE_id] = nb_rbs_required[UE_id];
      if (nb_rbs_required[CC_id][UE_id] > 0) {
        total_ue_count = total_ue_count + 1;
      }


      // hypotetical assignement
      /*
       * If schedule is enabled and if the priority of the UEs is modified
       * The average rbs per logical channel per user will depend on the level of
       * priority. Concerning the hypothetical assignement, we should assign more
       * rbs to prioritized users. Maybe, we can do a mapping between the
       * average rbs per user and the level of priority or multiply the average rbs
       * per user by a coefficient which represents the degree of priority.
       */

      if (total_ue_count == 0) {
        average_rbs_per_user[CC_id] = 0;
      } else if( (min_rb_unit[CC_id] * total_ue_count) <= (frame_parms[CC_id]->N_RB_DL) ) {
        average_rbs_per_user[CC_id] = (uint16_t) floor(frame_parms[CC_id]->N_RB_DL/total_ue_count);
      } else {
        average_rbs_per_user[CC_id] = min_rb_unit[CC_id]; // consider the total number of use that can be scheduled UE
      }
    }
  }

  fprintf(stdout, "[CONCTX] Nb of concurrent UEs = %d\n", conc_tx);


  // note: nb_rbs_required is assigned according to total_buffer_dl
  // extend nb_rbs_required to capture per LCID RB required
  for(i=UE_list->head; i>=0; i=UE_list->next[i]) {
    rnti = UE_RNTI(Mod_id,i);

    for (ii=0; ii<UE_num_active_CC(UE_list,i); ii++) {
      CC_id = UE_list->ordered_CCids[ii][i];

      // control channel
      if (mac_eNB_get_rrc_status(Mod_id,rnti) < RRC_RECONFIGURED) {
        nb_rbs_required_remaining_1[CC_id][i] = nb_rbs_required[CC_id][i];
      } else {
        nb_rbs_required_remaining_1[CC_id][i] = cmin(average_rbs_per_user[CC_id],nb_rbs_required[CC_id][i]);

      }
    }
  }


  

  //Allocation to UEs is done in 2 rounds,
  // 1st stage: average number of RBs allocated to each UE
  // 2nd stage: remaining RBs are allocated to high priority UEs
  for(r1=0; r1<2; r1++) {

    for(i=UE_list->head; i>=0; i=UE_list->next[i]) {
      for (ii=0; ii<UE_num_active_CC(UE_list,i); ii++) {
        CC_id = UE_list->ordered_CCids[ii][i];

        if(r1 == 0) {
          nb_rbs_required_remaining[CC_id][i] = nb_rbs_required_remaining_1[CC_id][i];
        } else { // rb required based only on the buffer - rb allloctaed in the 1st round + extra reaming rb form the 1st round
          nb_rbs_required_remaining[CC_id][i] = nb_rbs_required[CC_id][i]-nb_rbs_required_remaining_1[CC_id][i]+nb_rbs_required_remaining[CC_id][i];
        }

        if (nb_rbs_required[CC_id][i]> 0 )
          LOG_D(MAC,"round %d : nb_rbs_required_remaining[%d][%d]= %d (remaining_1 %d, required %d,  pre_nb_available_rbs %d, N_RBG %d, rb_unit %d)\n",
                r1, CC_id, i,
                nb_rbs_required_remaining[CC_id][i],
                nb_rbs_required_remaining_1[CC_id][i],
                nb_rbs_required[CC_id][i],
                UE_list->UE_sched_ctrl[i].pre_nb_available_rbs[CC_id],
                N_RBG[CC_id],
                min_rb_unit[CC_id]);

      }
    }

    if (total_ue_count > 0 ) {
      for(i=UE_list->head; i>=0; i=UE_list->next[i]) {
        UE_id = i;

        for (ii=0; ii<UE_num_active_CC(UE_list,UE_id); ii++) {
          CC_id = UE_list->ordered_CCids[ii][UE_id];
	  ue_sched_ctl = &UE_list->UE_sched_ctrl[UE_id];
	  harq_pid = ue_sched_ctl->harq_pid[CC_id];
	  round    = ue_sched_ctl->round[CC_id];

          rnti = UE_RNTI(Mod_id,UE_id);

          // LOG_D(MAC,"UE %d rnti 0x\n", UE_id, rnti );
          if(rnti == NOT_A_RNTI)
            continue;
	  if (UE_list->UE_sched_ctrl[UE_id].ul_out_of_sync == 1)
	    continue;

          transmission_mode = mac_xface->get_transmission_mode(Mod_id,CC_id,rnti);
	  //          mac_xface->get_ue_active_harq_pid(Mod_id,CC_id,rnti,frameP,subframeP,&harq_pid,&round,0);
          //rrc_status = mac_eNB_get_rrc_status(Mod_id,rnti);
          /* 1st allocate for the retx */

          // retransmission in data channels
          // control channel in the 1st transmission
          // data channel for all TM
          LOG_T(MAC,"calling dlsch_scheduler_pre_processor_allocate .. \n ");
          dlsch_scheduler_pre_processor_allocate (Mod_id,
                                                  UE_id,
                                                  CC_id,
                                                  N_RBG[CC_id],
                                                  transmission_mode,
                                                  min_rb_unit[CC_id],
                                                  frame_parms[CC_id]->N_RB_DL,
                                                  nb_rbs_required,
                                                  nb_rbs_required_remaining,
                                                  rballoc_sub,
                                                  MIMO_mode_indicator,
                                                  average_rbs_per_user,
                                                  available_rbs,
                                                  total_ue_count);

#ifdef TM5

          // data chanel TM5: to be revisted
          if ((round == 0 )  &&
              (transmission_mode == 5)  &&
              (ue_sched_ctl->dl_pow_off[CC_id] != 1)) {

            for(j=0; j<N_RBG[CC_id]; j+=2) {

              if( (((j == (N_RBG[CC_id]-1))&& (rballoc_sub[CC_id][j] == 0) && (ue_sched_ctl->rballoc_sub_UE[CC_id][j] == 0))  ||
                   ((j < (N_RBG[CC_id]-1)) && (rballoc_sub[CC_id][j+1] == 0) && (ue_sched_ctl->rballoc_sub_UE[CC_id][j+1] == 0)) ) &&
                  (nb_rbs_required_remaining[CC_id][UE_id]>0)) {

                for (ii = UE_list->next[i+1]; ii >=0; ii=UE_list->next[ii]) {

                  UE_id2 = ii;
                  rnti2 = UE_RNTI(Mod_id,UE_id2);
		  ue_sched_ctl2 = &UE_list->UE_sched_ctrl[UE_id2];
		  harq_pid2 = ue_sched_ctl2->harq_pid[CC_id];
		  round2    = ue_sched_ctl2->round[CC_id];
                  if(rnti2 == NOT_A_RNTI)
                    continue;
		  if (UE_list->UE_sched_ctrl[UE_id2].ul_out_of_sync == 1)
		    continue;

                  eNB_UE_stats2 = mac_xface->get_eNB_UE_stats(Mod_id,CC_id,rnti2);
                  //mac_xface->get_ue_active_harq_pid(Mod_id,CC_id,rnti2,frameP,subframeP,&harq_pid2,&round2,0);

                  if ((mac_eNB_get_rrc_status(Mod_id,rnti2) >= RRC_RECONFIGURED) &&
                      (round2==0) &&
                      (mac_xface->get_transmission_mode(Mod_id,CC_id,rnti2)==5) &&
                      (ue_sched_ctl->dl_pow_off[CC_id] != 1)) {

                    if( (((j == (N_RBG[CC_id]-1)) && (ue_sched_ctl->rballoc_sub_UE[CC_id][j] == 0)) ||
                         ((j < (N_RBG[CC_id]-1)) && (ue_sched_ctl->rballoc_sub_UE[CC_id][j+1] == 0))  ) &&
                        (nb_rbs_required_remaining[CC_id][UE_id2]>0)) {

                      if((((eNB_UE_stats2->DL_pmi_single^eNB_UE_stats1->DL_pmi_single)<<(14-j))&0xc000)== 0x4000) { //MU-MIMO only for 25 RBs configuration

                        rballoc_sub[CC_id][j] = 1;
                        ue_sched_ctl->rballoc_sub_UE[CC_id][j] = 1;
                        ue_sched_ctl2->rballoc_sub_UE[CC_id][j] = 1;
                        MIMO_mode_indicator[CC_id][j] = 0;

                        if (j< N_RBG[CC_id]-1) {
                          rballoc_sub[CC_id][j+1] = 1;
                          ue_sched_ctl->rballoc_sub_UE[CC_id][j+1] = 1;
                          ue_sched_ctl2->rballoc_sub_UE[CC_id][j+1] = 1;
                          MIMO_mode_indicator[CC_id][j+1] = 0;
                        }

                        ue_sched_ctl->dl_pow_off[CC_id] = 0;
                        ue_sched_ctl2->dl_pow_off[CC_id] = 0;


                        if ((j == N_RBG[CC_id]-1) &&
                            ((PHY_vars_eNB_g[Mod_id][CC_id]->lte_frame_parms.N_RB_DL == 25) ||
                             (PHY_vars_eNB_g[Mod_id][CC_id]->lte_frame_parms.N_RB_DL == 50))) {
			  
                          nb_rbs_required_remaining[CC_id][UE_id] = nb_rbs_required_remaining[CC_id][UE_id] - min_rb_unit[CC_id]+1;
                          ue_sched_ctl->pre_nb_available_rbs[CC_id] = ue_sched_ctl->pre_nb_available_rbs[CC_id] + min_rb_unit[CC_id]-1;
                          nb_rbs_required_remaining[CC_id][UE_id2] = nb_rbs_required_remaining[CC_id][UE_id2] - min_rb_unit[CC_id]+1;
                          ue_sched_ctl2->pre_nb_available_rbs[CC_id] = ue_sched_ctl2->pre_nb_available_rbs[CC_id] + min_rb_unit[CC_id]-1;
                        } else {
                          
			  nb_rbs_required_remaining[CC_id][UE_id] = nb_rbs_required_remaining[CC_id][UE_id] - 4;
                          ue_sched_ctl->pre_nb_available_rbs[CC_id] = ue_sched_ctl->pre_nb_available_rbs[CC_id] + 4;
                          nb_rbs_required_remaining[CC_id][UE_id2] = nb_rbs_required_remaining[CC_id][UE_id2] - 4;
                          ue_sched_ctl2->pre_nb_available_rbs[CC_id] = ue_sched_ctl2->pre_nb_available_rbs[CC_id] + 4;
                        }

                        break;
                      }
                    }
                  }
                }
              }
            }
          }

#endif
        }
      }
    } // total_ue_count
  } // end of for for r1 and r2

#ifdef TM5

  // This has to be revisited!!!!
  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
    i1=0;
    i2=0;
    i3=0;

    for (j=0; j<N_RBG[CC_id]; j++) {
      if(MIMO_mode_indicator[CC_id][j] == 2) {
        i1 = i1+1;
      } else if(MIMO_mode_indicator[CC_id][j] == 1) {
        i2 = i2+1;
      } else if(MIMO_mode_indicator[CC_id][j] == 0) {
        i3 = i3+1;
      }
    }

    if((i1 < N_RBG[CC_id]) && (i2>0) && (i3==0)) {
      PHY_vars_eNB_g[Mod_id][CC_id]->check_for_SUMIMO_transmissions = PHY_vars_eNB_g[Mod_id][CC_id]->check_for_SUMIMO_transmissions + 1;
    }

    if(i3 == N_RBG[CC_id] && i1==0 && i2==0) {
      PHY_vars_eNB_g[Mod_id][CC_id]->FULL_MUMIMO_transmissions = PHY_vars_eNB_g[Mod_id][CC_id]->FULL_MUMIMO_transmissions + 1;
    }

    if((i1 < N_RBG[CC_id]) && (i3 > 0)) {
      PHY_vars_eNB_g[Mod_id][CC_id]->check_for_MUMIMO_transmissions = PHY_vars_eNB_g[Mod_id][CC_id]->check_for_MUMIMO_transmissions + 1;
    }

    PHY_vars_eNB_g[Mod_id][CC_id]->check_for_total_transmissions = PHY_vars_eNB_g[Mod_id][CC_id]->check_for_total_transmissions + 1;

  }

#endif

  for(i=UE_list->head; i>=0; i=UE_list->next[i]) {
    UE_id = i;
    ue_sched_ctl = &UE_list->UE_sched_ctrl[UE_id];

    for (ii=0; ii<UE_num_active_CC(UE_list,UE_id); ii++) {
      CC_id = UE_list->ordered_CCids[ii][UE_id];
      //PHY_vars_eNB_g[Mod_id]->mu_mimo_mode[UE_id].dl_pow_off = dl_pow_off[UE_id];

      if (ue_sched_ctl->pre_nb_available_rbs[CC_id] > 0 ) {
	      LOG_D(MAC,"******************DL Scheduling Information for UE%d ************************\n",UE_id);
	      LOG_D(MAC,"dl power offset UE%d = %d \n",UE_id,ue_sched_ctl->dl_pow_off[CC_id]);
        LOG_D(MAC,"***********RB Alloc for every subband for UE%d ***********\n",UE_id);

        for(j=0; j<N_RBG[CC_id]; j++) {
          //PHY_vars_eNB_g[Mod_id]->mu_mimo_mode[UE_id].rballoc_sub[i] = rballoc_sub_UE[CC_id][UE_id][i];
          LOG_D(MAC,"RB Alloc for UE%d and Subband%d = %d\n",UE_id,j,ue_sched_ctl->rballoc_sub_UE[CC_id][j]);
        }

        //PHY_vars_eNB_g[Mod_id]->mu_mimo_mode[UE_id].pre_nb_available_rbs = pre_nb_available_rbs[CC_id][UE_id];
        LOG_D(MAC,"Total RBs allocated for UE%d = %d\n",UE_id,ue_sched_ctl->pre_nb_available_rbs[CC_id]);
      }
    }
  }
}

#define SF05_LIMIT 1

void dlsch_scheduler_pre_processor_reset (int module_idP,
					  int UE_id,
					  uint8_t  CC_id,
					  int frameP,
					  int subframeP,					  
					  int N_RBG,
					  uint16_t nb_rbs_required[MAX_NUM_CCs][NUMBER_OF_UE_MAX],
					  uint16_t nb_rbs_required_remaining[MAX_NUM_CCs][NUMBER_OF_UE_MAX],
					  unsigned char rballoc_sub[MAX_NUM_CCs][N_RBG_MAX],
					  unsigned char MIMO_mode_indicator[MAX_NUM_CCs][N_RBG_MAX])
  
{
  int i,j;
  UE_list_t *UE_list=&eNB_mac_inst[module_idP].UE_list;
  UE_sched_ctrl *ue_sched_ctl = &UE_list->UE_sched_ctrl[UE_id];
  rnti_t rnti = UE_RNTI(module_idP,UE_id);
  uint8_t *vrb_map = eNB_mac_inst[module_idP].common_channels[CC_id].vrb_map;
  int RBGsize = PHY_vars_eNB_g[module_idP][CC_id]->lte_frame_parms.N_RB_DL/N_RBG;
#ifdef SF05_LIMIT
  //int subframe05_limit=0;
  int sf05_upper=-1,sf05_lower=-1;
#endif
  LTE_eNB_UE_stats *eNB_UE_stats = mac_xface->get_eNB_UE_stats(module_idP,CC_id,rnti);
  // initialize harq_pid and round
  mac_xface->get_ue_active_harq_pid(module_idP,CC_id,rnti,
				    frameP,subframeP,
				    &ue_sched_ctl->harq_pid[CC_id],
				    &ue_sched_ctl->round[CC_id],
				    0);
  if (ue_sched_ctl->ta_timer == 0) {

    // WE SHOULD PROTECT the eNB_UE_stats with a mutex here ...

    ue_sched_ctl->ta_timer = 20;  // wait 20 subframes before taking TA measurement from PHY
    switch (PHY_vars_eNB_g[module_idP][CC_id]->lte_frame_parms.N_RB_DL) {
    case 6:
      ue_sched_ctl->ta_update = eNB_UE_stats->timing_advance_update;
      break;
      
    case 15:
      ue_sched_ctl->ta_update = eNB_UE_stats->timing_advance_update/2;
      break;
      
    case 25:
      ue_sched_ctl->ta_update = eNB_UE_stats->timing_advance_update/4;
      break;
      
    case 50:
      ue_sched_ctl->ta_update = eNB_UE_stats->timing_advance_update/8;
      break;
      
    case 75:
      ue_sched_ctl->ta_update = eNB_UE_stats->timing_advance_update/12;
      break;
      
    case 100:
      ue_sched_ctl->ta_update = eNB_UE_stats->timing_advance_update/16;
      break;
    }
    // clear the update in case PHY does not have a new measurement after timer expiry
    eNB_UE_stats->timing_advance_update =  0;
  }
  else {
    ue_sched_ctl->ta_timer--;
    ue_sched_ctl->ta_update =0; // don't trigger a timing advance command
  }
  if (UE_id==0) {
    VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME(VCD_SIGNAL_DUMPER_VARIABLES_UE0_TIMING_ADVANCE,ue_sched_ctl->ta_update);
  }
  nb_rbs_required[CC_id][UE_id]=0;
  ue_sched_ctl->pre_nb_available_rbs[CC_id] = 0;
  ue_sched_ctl->dl_pow_off[CC_id] = 2;
  nb_rbs_required_remaining[CC_id][UE_id] = 0;

#ifdef SF05_LIMIT  
  switch (N_RBG) {
  case 6:
    sf05_lower=0;
    sf05_upper=5;
    break;
  case 8:
    sf05_lower=2;
    sf05_upper=5;
    break;
  case 13:
    sf05_lower=4;
    sf05_upper=7;
    break;
  case 17:
    sf05_lower=7;
    sf05_upper=9;
    break;
  case 25:
    sf05_lower=11;
    sf05_upper=13;
    break;
  }
#endif
  // Initialize Subbands according to VRB map
  for (i=0; i<N_RBG; i++) {
    ue_sched_ctl->rballoc_sub_UE[CC_id][i] = 0;
    rballoc_sub[CC_id][i] = 0;
#ifdef SF05_LIMIT
    // for avoiding 6+ PRBs around DC in subframe 0-5 (avoid excessive errors)

    if ((subframeP==0 || subframeP==5) && 
	(i>=sf05_lower && i<=sf05_upper))
      rballoc_sub[CC_id][i]=1;
#endif
    // for SI-RNTI,RA-RNTI and P-RNTI allocations
    for (j=0;j<RBGsize;j++) {
      if (vrb_map[j+(i*RBGsize)]!=0)  {
	rballoc_sub[CC_id][i] = 1;
	LOG_D(MAC,"Frame %d, subframe %d : vrb %d allocated\n",frameP,subframeP,j+(i*RBGsize));
	break;
      }
    }
    LOG_D(MAC,"Frame %d Subframe %d CC_id %d RBG %i : rb_alloc %d\n",frameP,subframeP,CC_id,i,rballoc_sub[CC_id][i]);
    MIMO_mode_indicator[CC_id][i] = 2;
  }
}


void dlsch_scheduler_pre_processor_allocate (module_id_t   Mod_id,
    int           UE_id,
    uint8_t       CC_id,
    int           N_RBG,
    int           transmission_mode,
    int           min_rb_unit,
    uint8_t       N_RB_DL,
    uint16_t      nb_rbs_required[MAX_NUM_CCs][NUMBER_OF_UE_MAX],
    uint16_t      nb_rbs_required_remaining[MAX_NUM_CCs][NUMBER_OF_UE_MAX],
    unsigned char rballoc_sub[MAX_NUM_CCs][N_RBG_MAX],
    unsigned char MIMO_mode_indicator[MAX_NUM_CCs][N_RBG_MAX],
    uint16_t      average_rbs_per_user[MAX_NUM_CCs],
    uint16_t      available_rbs[MAX_NUM_CCs],
    unsigned char total_ue_count)
{

  int i;
  uint8_t n, ii;
  UE_list_t *UE_list=&eNB_mac_inst[Mod_id].UE_list;
  UE_sched_ctrl *ue_sched_ctl = &UE_list->UE_sched_ctrl[UE_id];

  //ONAMA
  uint16_t di = 0;
  uint16_t dimean = 0;
  uint8_t done = 1;
  int rb, rb2, status_cli, jj, k;

  if (UCS) {
  /******************** UCS Step 1: Prealloc **************************/
  // state.i.rb = UNDECIDED
  fprintf(stdout, "[UCS] average_rbs_per_user[%d] = %d\n", CC_id, average_rbs_per_user[CC_id]);
  for (UE_id = UE_list->head; UE_id >= 0; UE_id = UE_list->next[UE_id]) {
    // Compute reliability for each UE
    reliability(Mod_id, UE_id);
    for (rb=0; rb<N_RBG_MAX; rb++) {
      for (n = 0; n < UE_list->numactiveCCs[UE_id]; n++) {
        // Perform K adaptation
        //@TODO Call pktr scheduler here
        UE_status[UE_id][rb] = State_UNDECIDED;
        //fprintf(stdout, "ONAMA: UE_status[%d][rb:%d]==%d\n", UE_id, rb, UE_status[UE_id][rb]);
      }
    }
  } 
  
  //k.rb = min{d_k, mean} for each k E M_i U i
  for (UE_id = UE_list->head; UE_id >= 0; UE_id = UE_list->next[UE_id]) {
    for (rb=0; rb<N_RBG_MAX; rb++) {
      for (ii = UE_list->head; ii >= 0; ii = UE_list->next[ii]) {
        conflict[UE_id][ii] = 1; //test single UE
        if (conflict[UE_id][ii]==1) {
          for (i = 0; i < UE_num_active_CC(UE_list, UE_id); i++) {
            CC_id = UE_list->ordered_CCids[i][UE_id];
            ue_sched_ctl = &UE_list->UE_sched_ctrl[UE_id];
            //available_rbs[CC_id] = ue_sched_ctl->max_rbs_allowed_slice[CC_id][slice_idx];
            available_rbs[CC_id] = ue_sched_ctl->pre_nb_available_rbs[CC_id];
            if (total_ue_count > 0)
              average_rbs_per_user[CC_id] = (uint16_t)floor(available_rbs[CC_id]/total_ue_count);
             // average_rbs_per_user[CC_id] = (uint16_t)floor(available_rbs[CC_id]/ue_count_newtx[CC_id]);
            else
              average_rbs_per_user[CC_id] = available_rbs[CC_id];
           UE_list->UE_sched_ctrl[ii].pre_nb_available_rbs[CC_id]  = cmin(nb_rbs_required[CC_id][ii], average_rbs_per_user[CC_id]);
           fprintf(stdout, "ONAMA: nb_rbs_required[%d][%d] = %d average_rbs_per_user[%d]=%d\n", CC_id, ii, nb_rbs_required[CC_id][ii], CC_id, average_rbs_per_user[CC_id]);
           fprintf(stdout, "ONAMA: nb_rbs_required_remaining_1[%d][%d] = %d\n", CC_id, ii, UE_list->UE_sched_ctrl[i].pre_nb_available_rbs[CC_id]);
            
            //MAX priority
            priority[ii][rb] = MAX_UCS_PRIORITY;
            // Status becomes ACTIVE
            UE_status[ii][rb] = State_ACTIVE;

            if( nb_rbs_required[CC_id][ii] <= average_rbs_per_user[CC_id] ) {
              for (rb2 = rb+1; rb2<N_RBG_MAX; ++rb2) {
                if(rb2 != rb)
                  UE_status[ii][rb2] = State_INACTIVE;
              }
            }
          }
        }
      }
    }
  }
  /**************************************************************/


  /************** UCS Step 2: Priority Calculation **************/
  for (UE_id = UE_list->head; UE_id >= 0; UE_id = UE_list->next[UE_id]) {
    for (ii = UE_list->head; ii >= 0; ii = UE_list->next[ii]) {
      if (conflict[UE_id][ii]==1) {
        for (i = 0; i < UE_num_active_CC(UE_list, UE_id); i++) {
          CC_id = UE_list->ordered_CCids[i][UE_id];
          priority_UEs(ii, CC_id, UE_list->UE_sched_ctrl[ii].pre_nb_available_rbs[CC_id]);
        }
      }
    }
  }
  /**************************************************************/



  /************** UCS Step 3: State Selection  ******************/
  while(done == 1) {
    //printf("ONAMA: STEP3\n");
    done = 0;
    for (UE_id = UE_list->head; UE_id >= 0; UE_id = UE_list->next[UE_id]) {
      for (jj=UE_list->head; jj>=0; jj=UE_list->next[jj]) {
        for (rb=0; rb<N_RBG_MAX; rb++) {
          //First condition: di - mean > 0
          dimean = (nb_rbs_required[CC_id][UE_id] - average_rbs_per_user[CC_id]);
          if( (dimean>0) && (conflict[UE_id][jj]==1) && (UE_status[jj][rb]!=State_INACTIVE) && (priority[UE_id][rb]>priority[jj][rb]) ) {
            //state.i.rb = ACTIVE
            UE_status[UE_id][rb] = State_ACTIVE;
            //di=di-mean-1
            nb_rbs_required[CC_id][UE_id] = (nb_rbs_required[CC_id][UE_id] - average_rbs_per_user[CC_id] - 1);
            //if di == 0
            if( nb_rbs_required[CC_id][UE_id] == 0 ) {
              //state.i.rb2 = INACTIVE for each rb2 where state.i.rb2 == UNDECIDED
              for (k=rb+1; k<N_RBG_MAX; k++ ) {
                if( UE_status[UE_id][k] == State_UNDECIDED )
                  UE_status[UE_id][k]=State_INACTIVE;
              }
            } // end di == 0
	  } //end dimean cond

          //Second condition
          if( (conflict[UE_id][jj]==1) && (UE_status[jj][rb]==State_ACTIVE) && (priority[UE_id][rb]<priority[jj][rb]) ) {
            UE_status[UE_id][rb]=State_INACTIVE; 
          }
          //Third condition
          if ( UE_status[jj][rb]==State_UNDECIDED ) {
            done = 1;
          }

        } // end for rb
      } // end for neighbours of UE_id    
    } // end for UE_id
  } // end of while
  /**********************************************************/
  } // END IF(UCS)





  for(i=0; i<N_RBG; i++) {

    if((rballoc_sub[CC_id][i] == 0)           &&
        (ue_sched_ctl->rballoc_sub_UE[CC_id][i] == 0) &&
        (nb_rbs_required_remaining[CC_id][UE_id]>0)   &&
        (ue_sched_ctl->pre_nb_available_rbs[CC_id] < nb_rbs_required[CC_id][UE_id])) {

      // if this UE is not scheduled for TM5
      if (ue_sched_ctl->dl_pow_off[CC_id] != 0 )  {

	if ((i == N_RBG-1) && ((N_RB_DL == 25) || (N_RB_DL == 50))) {
	  rballoc_sub[CC_id][i] = 1;
	  ue_sched_ctl->rballoc_sub_UE[CC_id][i] = 1;
	  MIMO_mode_indicator[CC_id][i] = 1;
	  if (transmission_mode == 5 ) {
	    ue_sched_ctl->dl_pow_off[CC_id] = 1;
	  }   
	  nb_rbs_required_remaining[CC_id][UE_id] = nb_rbs_required_remaining[CC_id][UE_id] - min_rb_unit+1;
          ue_sched_ctl->pre_nb_available_rbs[CC_id] = ue_sched_ctl->pre_nb_available_rbs[CC_id] + min_rb_unit - 1;
        } else {
	  if (nb_rbs_required_remaining[CC_id][UE_id] >=  min_rb_unit){
	    rballoc_sub[CC_id][i] = 1;
	    ue_sched_ctl->rballoc_sub_UE[CC_id][i] = 1;
	    MIMO_mode_indicator[CC_id][i] = 1;
	    if (transmission_mode == 5 ) {
	      ue_sched_ctl->dl_pow_off[CC_id] = 1;
	    }
	    nb_rbs_required_remaining[CC_id][UE_id] = nb_rbs_required_remaining[CC_id][UE_id] - min_rb_unit;
	    ue_sched_ctl->pre_nb_available_rbs[CC_id] = ue_sched_ctl->pre_nb_available_rbs[CC_id] + min_rb_unit;
      fprintf(stdout, "[allocate] ue_sched_ctl->pre_nb_available_rbs[%d] = %d\n", CC_id, ue_sched_ctl->pre_nb_available_rbs[CC_id]);
	  }
	}
      } // dl_pow_off[CC_id][UE_id] ! = 0
    }
  }

}


/// ULSCH PRE_PROCESSOR


void ulsch_scheduler_pre_processor(module_id_t module_idP,
                                   int frameP,
                                   sub_frame_t subframeP,
                                   uint16_t *first_rb,
                                   uint8_t aggregation)
{

  int16_t            i;
  uint16_t           UE_id,n,r;
  uint8_t            CC_id, round, harq_pid;
  uint16_t           nb_allocated_rbs[MAX_NUM_CCs][NUMBER_OF_UE_MAX],total_allocated_rbs[MAX_NUM_CCs],average_rbs_per_user[MAX_NUM_CCs];
  int16_t            total_remaining_rbs[MAX_NUM_CCs];
  uint16_t           max_num_ue_to_be_scheduled=0,total_ue_count=0;
  rnti_t             rnti= -1;
  UE_list_t          *UE_list = &eNB_mac_inst[module_idP].UE_list;
  UE_TEMPLATE        *UE_template = 0;
  LTE_DL_FRAME_PARMS   *frame_parms = 0;


  //LOG_I(MAC,"assign max mcs min rb\n");
  // maximize MCS and then allocate required RB according to the buffer occupancy with the limit of max available UL RB
  assign_max_mcs_min_rb(module_idP,frameP, subframeP, first_rb);

  //LOG_I(MAC,"sort ue \n");
  // sort ues
  sort_ue_ul (module_idP,frameP, subframeP);


  // we need to distribute RBs among UEs
  // step1:  reset the vars
  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
    total_allocated_rbs[CC_id]=0;
    total_remaining_rbs[CC_id]=0;
    average_rbs_per_user[CC_id]=0;

    for (i=UE_list->head_ul; i>=0; i=UE_list->next_ul[i]) {
      nb_allocated_rbs[CC_id][i]=0;
    }
  }

  //LOG_I(MAC,"step2 \n");
  // step 2: calculate the average rb per UE
  total_ue_count =0;
  max_num_ue_to_be_scheduled=0;

  for (i=UE_list->head_ul; i>=0; i=UE_list->next_ul[i]) {

    rnti = UE_RNTI(module_idP,i);

    if (rnti==NOT_A_RNTI)
      continue;

    if (UE_list->UE_sched_ctrl[i].ul_out_of_sync == 1)
      continue;

    UE_id = i;

    for (n=0; n<UE_list->numactiveULCCs[UE_id]; n++) {
      // This is the actual CC_id in the list
      CC_id = UE_list->ordered_ULCCids[n][UE_id];
      UE_template = &UE_list->UE_template[CC_id][UE_id];
      average_rbs_per_user[CC_id]=0;
      frame_parms = mac_xface->get_lte_frame_parms(module_idP,CC_id);

      if (UE_template->pre_allocated_nb_rb_ul > 0) {
        total_ue_count+=1;
      }
      /*
      if((mac_xface->get_nCCE_max(module_idP,CC_id,3,subframeP) - nCCE_to_be_used[CC_id])  > (1<<aggregation)) {
        nCCE_to_be_used[CC_id] = nCCE_to_be_used[CC_id] + (1<<aggregation);
        max_num_ue_to_be_scheduled+=1;
	}*/

      max_num_ue_to_be_scheduled+=1;

      if (total_ue_count == 0) {
        average_rbs_per_user[CC_id] = 0;
      } else if (total_ue_count == 1 ) { // increase the available RBs, special case,
        average_rbs_per_user[CC_id] = frame_parms->N_RB_UL-first_rb[CC_id]+1;
      } else if( (total_ue_count <= (frame_parms->N_RB_DL-first_rb[CC_id])) &&
                 (total_ue_count <= max_num_ue_to_be_scheduled)) {
        average_rbs_per_user[CC_id] = (uint16_t) floor((frame_parms->N_RB_UL-first_rb[CC_id])/total_ue_count);
      } else if (max_num_ue_to_be_scheduled > 0 ) {
        average_rbs_per_user[CC_id] = (uint16_t) floor((frame_parms->N_RB_UL-first_rb[CC_id])/max_num_ue_to_be_scheduled);
      } else {
        average_rbs_per_user[CC_id]=1;
        LOG_W(MAC,"[eNB %d] frame %d subframe %d: UE %d CC %d: can't get average rb per user (should not be here)\n",
              module_idP,frameP,subframeP,UE_id,CC_id);
      }
    }
  }
  if (total_ue_count > 0)
    LOG_D(MAC,"[eNB %d] Frame %d subframe %d: total ue to be scheduled %d/%d\n",
	  module_idP, frameP, subframeP,total_ue_count, max_num_ue_to_be_scheduled);

  //LOG_D(MAC,"step3\n");

  // step 3: assigne RBS
  for (i=UE_list->head_ul; i>=0; i=UE_list->next_ul[i]) {
    rnti = UE_RNTI(module_idP,i);

    if (rnti==NOT_A_RNTI)
      continue;
    if (UE_list->UE_sched_ctrl[i].ul_out_of_sync == 1)
      continue;

    UE_id = i;

    for (n=0; n<UE_list->numactiveULCCs[UE_id]; n++) {
      // This is the actual CC_id in the list
      CC_id = UE_list->ordered_ULCCids[n][UE_id];

      mac_xface->get_ue_active_harq_pid(module_idP,CC_id,rnti,frameP,subframeP,&harq_pid,&round,1);

      if(round>0) {
        nb_allocated_rbs[CC_id][UE_id] = UE_list->UE_template[CC_id][UE_id].nb_rb_ul[harq_pid];
      } else {
        nb_allocated_rbs[CC_id][UE_id] = cmin(UE_list->UE_template[CC_id][UE_id].pre_allocated_nb_rb_ul, average_rbs_per_user[CC_id]);
      }

      total_allocated_rbs[CC_id]+= nb_allocated_rbs[CC_id][UE_id];

    }
  }

  // step 4: assigne the remaining RBs and set the pre_allocated rbs accordingly
  for(r=0; r<2; r++) {

    for (i=UE_list->head_ul; i>=0; i=UE_list->next_ul[i]) {
      rnti = UE_RNTI(module_idP,i);

      if (rnti==NOT_A_RNTI)
        continue;
      if (UE_list->UE_sched_ctrl[i].ul_out_of_sync == 1)
	continue;

      UE_id = i;

      for (n=0; n<UE_list->numactiveULCCs[UE_id]; n++) {
        // This is the actual CC_id in the list
        CC_id = UE_list->ordered_ULCCids[n][UE_id];
        UE_template = &UE_list->UE_template[CC_id][UE_id];
        frame_parms = mac_xface->get_lte_frame_parms(module_idP,CC_id);
        total_remaining_rbs[CC_id]=frame_parms->N_RB_UL - first_rb[CC_id] - total_allocated_rbs[CC_id];

        if (total_ue_count == 1 ) {
          total_remaining_rbs[CC_id]+=1;
        }

        if ( r == 0 ) {
          while ( (UE_template->pre_allocated_nb_rb_ul > 0 ) &&
                  (nb_allocated_rbs[CC_id][UE_id] < UE_template->pre_allocated_nb_rb_ul) &&
                  (total_remaining_rbs[CC_id] > 0)) {
            nb_allocated_rbs[CC_id][UE_id] = cmin(nb_allocated_rbs[CC_id][UE_id]+1,UE_template->pre_allocated_nb_rb_ul);
            total_remaining_rbs[CC_id]--;
            total_allocated_rbs[CC_id]++;
          }
        } else {
          UE_template->pre_allocated_nb_rb_ul= nb_allocated_rbs[CC_id][UE_id];
          LOG_D(MAC,"******************UL Scheduling Information for UE%d CC_id %d ************************\n",UE_id, CC_id);
          LOG_D(MAC,"[eNB %d] total RB allocated for UE%d CC_id %d  = %d\n", module_idP, UE_id, CC_id, UE_template->pre_allocated_nb_rb_ul);
        }
      }
    }
  }

  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
    frame_parms= mac_xface->get_lte_frame_parms(module_idP,CC_id);

    if (total_allocated_rbs[CC_id]>0) {
      LOG_D(MAC,"[eNB %d] total RB allocated for all UEs = %d/%d\n", module_idP, total_allocated_rbs[CC_id], frame_parms->N_RB_UL - first_rb[CC_id]);
    }
  }
}


void assign_max_mcs_min_rb(module_id_t module_idP,int frameP, sub_frame_t subframeP, uint16_t *first_rb)
{

  int                i;
  uint16_t           n,UE_id;
  uint8_t            CC_id;
  rnti_t             rnti           = -1;
  int                mcs;
  int                rb_table_index=0,tbs,tx_power;
  eNB_MAC_INST       *eNB = &eNB_mac_inst[module_idP];
  UE_list_t          *UE_list = &eNB->UE_list;

  UE_TEMPLATE       *UE_template;
  LTE_DL_FRAME_PARMS   *frame_parms;


  for (i=UE_list->head_ul; i>=0; i=UE_list->next_ul[i]) {

    rnti = UE_RNTI(module_idP,i);

    if (rnti==NOT_A_RNTI)
      continue;
    if (UE_list->UE_sched_ctrl[i].ul_out_of_sync == 1)
      continue;

    if (UE_list->UE_sched_ctrl[i].phr_received == 1)
      mcs = 20; // if we've received the power headroom information the UE, we can go to maximum mcs
    else
      mcs = 10; // otherwise, limit to QPSK PUSCH

    UE_id = i;

    for (n=0; n<UE_list->numactiveULCCs[UE_id]; n++) {
      // This is the actual CC_id in the list
      CC_id = UE_list->ordered_ULCCids[n][UE_id];

      if (CC_id >= MAX_NUM_CCs) {
        LOG_E( MAC, "CC_id %u should be < %u, loop n=%u < numactiveULCCs[%u]=%u",
               CC_id,
               MAX_NUM_CCs,
               n,
               UE_id,
               UE_list->numactiveULCCs[UE_id]);
      }

      AssertFatal( CC_id < MAX_NUM_CCs, "CC_id %u should be < %u, loop n=%u < numactiveULCCs[%u]=%u",
                   CC_id,
                   MAX_NUM_CCs,
                   n,
                   UE_id,
                   UE_list->numactiveULCCs[UE_id]);
      frame_parms=mac_xface->get_lte_frame_parms(module_idP,CC_id);
      UE_template = &UE_list->UE_template[CC_id][UE_id];

      // if this UE has UL traffic
      if (UE_template->ul_total_buffer > 0 ) {

        tbs = mac_xface->get_TBS_UL(mcs,3);  // 1 or 2 PRB with cqi enabled does not work well!
        // fixme: set use_srs flag
        tx_power= mac_xface->estimate_ue_tx_power(tbs,rb_table[rb_table_index],0,frame_parms->Ncp,0);

        while ((((UE_template->phr_info - tx_power) < 0 ) || (tbs > UE_template->ul_total_buffer))&&
               (mcs > 3)) {
          // LOG_I(MAC,"UE_template->phr_info %d tx_power %d mcs %d\n", UE_template->phr_info,tx_power, mcs);
          mcs--;
          tbs = mac_xface->get_TBS_UL(mcs,rb_table[rb_table_index]);
          tx_power = mac_xface->estimate_ue_tx_power(tbs,rb_table[rb_table_index],0,frame_parms->Ncp,0); // fixme: set use_srs
        }

        while ((tbs < UE_template->ul_total_buffer) &&
               (rb_table[rb_table_index]<(frame_parms->N_RB_UL-first_rb[CC_id])) &&
               ((UE_template->phr_info - tx_power) > 0) &&
               (rb_table_index < 32 )) {
          //  LOG_I(MAC,"tbs %d ul buffer %d rb table %d max ul rb %d\n", tbs, UE_template->ul_total_buffer, rb_table[rb_table_index], frame_parms->N_RB_UL-first_rb[CC_id]);
          rb_table_index++;
          tbs = mac_xface->get_TBS_UL(mcs,rb_table[rb_table_index]);
          tx_power = mac_xface->estimate_ue_tx_power(tbs,rb_table[rb_table_index],0,frame_parms->Ncp,0);
        }

        UE_template->ue_tx_power = tx_pow = tx_power;

        if (rb_table[rb_table_index]>(frame_parms->N_RB_UL-first_rb[CC_id]-1)) {
          rb_table_index--;
        }

        // 1 or 2 PRB with cqi enabled does not work well!
	if (rb_table[rb_table_index]<3) {
          rb_table_index=2; //3PRB
        }

        UE_template->pre_assigned_mcs_ul=mcs;
        UE_template->pre_allocated_rb_table_index_ul=rb_table_index;
        UE_template->pre_allocated_nb_rb_ul= rb_table[rb_table_index];
        LOG_D(MAC,"[eNB %d] frame %d subframe %d: for UE %d CC %d: pre-assigned mcs %d, pre-allocated rb_table[%d]=%d RBs (phr %d, tx power %d)\n",
              module_idP, frameP, subframeP, UE_id, CC_id,
              UE_template->pre_assigned_mcs_ul,
              UE_template->pre_allocated_rb_table_index_ul,
              UE_template->pre_allocated_nb_rb_ul,
              UE_template->phr_info,tx_power);
      } else {
        UE_template->pre_allocated_rb_table_index_ul=-1;
        UE_template->pre_allocated_nb_rb_ul=0;
      }
    }
  }
}


void sort_ue_ul (module_id_t module_idP,int frameP, sub_frame_t subframeP)
{

  int               UE_id1,UE_id2;
  int               pCCid1,pCCid2;
  int               round1,round2;
  int               i=0,ii=0;
  rnti_t            rnti1,rnti2;

  UE_list_t *UE_list = &eNB_mac_inst[module_idP].UE_list;

  for (i=UE_list->head_ul; i>=0; i=UE_list->next_ul[i]) {

    //LOG_I(MAC,"sort ue ul i %d\n",i);
    for (ii=UE_list->next_ul[i]; ii>=0; ii=UE_list->next_ul[ii]) {
      //LOG_I(MAC,"sort ul ue 2 ii %d\n",ii);
 
      UE_id1  = i;
      rnti1 = UE_RNTI(module_idP,UE_id1);
      
      if(rnti1 == NOT_A_RNTI)
	continue;
      if (UE_list->UE_sched_ctrl[i].ul_out_of_sync == 1)
	continue;

      pCCid1 = UE_PCCID(module_idP,UE_id1);
      round1  = maxround(module_idP,rnti1,frameP,subframeP,1);
      
      UE_id2  = ii;
      rnti2 = UE_RNTI(module_idP,UE_id2);
      
      if(rnti2 == NOT_A_RNTI)
        continue;
      if (UE_list->UE_sched_ctrl[UE_id2].ul_out_of_sync == 1)
	continue;

      pCCid2 = UE_PCCID(module_idP,UE_id2);
      round2  = maxround(module_idP,rnti2,frameP,subframeP,1);

      if(round2 > round1) {
        swap_UEs(UE_list,UE_id1,UE_id2,1);
      } else if (round2 == round1) {
        if (UE_list->UE_template[pCCid1][UE_id1].ul_buffer_info[LCGID0] < UE_list->UE_template[pCCid2][UE_id2].ul_buffer_info[LCGID0]) {
          swap_UEs(UE_list,UE_id1,UE_id2,1);
        } else if (UE_list->UE_template[pCCid1][UE_id1].ul_total_buffer <  UE_list->UE_template[pCCid2][UE_id2].ul_total_buffer) {
          swap_UEs(UE_list,UE_id1,UE_id2,1);
        } else if (UE_list->UE_template[pCCid1][UE_id1].pre_assigned_mcs_ul <  UE_list->UE_template[pCCid2][UE_id2].pre_assigned_mcs_ul) {
          if (UE_list->UE_template[pCCid2][UE_id2].ul_total_buffer > 0 ) {
            swap_UEs(UE_list,UE_id1,UE_id2,1);
          }
        }
      }
    }
  }
}

