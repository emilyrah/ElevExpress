/******************************************/
/* SIGMA RUN DEFAULTS FOR TRANSLATED MODEL */
/*******************************************/
/*

         MODEL DEFAULTS
         --------------

Model Name:         ElevatorModelFinalRun
Model Description:  AN AUTOMATIC CARWASH
Output File:        untitled.out
Output Plot Style:  NOAUTO_FIT
Run Mode:           SINGLE_STEP
Trace Vars:         DEST,QUP[1],QUP[2],QUP[3],QUP[4],QUP[5],QUP[6],QUP[7],QUP[8],QUP[9],QUP[10],QUP[11],QUP[12],QUP[13],QUP[14],QUP[15],QUP[16],QUP[17],QUP[18],QUP[19],QUP[20],QUP[21],QUP[22],QUP[23],QUP[24],QUP[25],QUP[26],QUP[27],QUP[28],QUP[29],QUP[30],QUP[31],QUP[32],QUP[33],QUP[34],QUP[35],QUP[36],QUP[37],QUP[38],QUP[39],QUP[40],QUP[41],QUP[42],QUP[43],QUP[44],QUP[45],QUP[46],QUP[47],QUP[48],QUP[49],QUP[50],QUP[51],QUP[52],QUP[53],QUP[54],QUP[55],QUP[56],QUP[57],QUP[58],QUP[59],QUP[60],QUP[61],QUP[62],QUP[63],QUP[64],QUP[65],QUP[66],QUP[67],QUP[68],QUP[69],QUP[70],QUP[71],QUP[72],QUP[73],QUP[74],QUP[75],QUP[76],QUP[77],QUP[78],QUP[79],QUP[80],QUP[81],QUP[82],QUP[83],QUP[84],QUP[85],QUP[86],QUP[87],QUP[88],QUP[89],QUP[90],QUP[91],QUP[92],QUP[93],QUP[94],QUP[95],QUP[96],QUP[97],QUP[98],QUP[99],QDOWN[1],QDOWN[2],QDOWN[3],QDOWN[4],QDOWN[5],QDOWN[6],QDOWN[7],QDOWN[8],QDOWN[9],QDOWN[10],QDOWN[11],QDOWN[12],QDOWN[13],QDOWN[14],QDOWN[15],QDOWN[16],QDOWN[17],QDOWN[18],QDOWN[19],QDOWN[20],QDOWN[21],QDOWN[22],QDOWN[23],QDOWN[24],QDOWN[25],QDOWN[26],QDOWN[27],QDOWN[28],QDOWN[29],QDOWN[30],QDOWN[31],QDOWN[32],QDOWN[33],QDOWN[34],QDOWN[35],QDOWN[36],QDOWN[37],QDOWN[38],QDOWN[39],QDOWN[40],QDOWN[41],QDOWN[42],QDOWN[43],QDOWN[44],QDOWN[45],QDOWN[46],QDOWN[47],QDOWN[48],QDOWN[49],QDOWN[50],QDOWN[51],QDOWN[52],QDOWN[53],QDOWN[54],QDOWN[55],QDOWN[56],QDOWN[57],QDOWN[58],QDOWN[59],QDOWN[60],QDOWN[61],QDOWN[62],QDOWN[63],QDOWN[64],QDOWN[65],QDOWN[66],QDOWN[67],QDOWN[68],QDOWN[69],QDOWN[70],QDOWN[71],QDOWN[72]
Random Number Seed: 12345
Initial Values:     15,2,10,1,0.5
Ending Condition:   STOP_ON_TIME
Ending Time:        720.000
Trace Events:       ALL EVENTS TRACED
Hide Edges:         

*/
/*******************************************/


#include "sigmafns.h"
#include "sigmalib.h"

/* EVENT FUNCTIONS USED IN THE SIMULATION MODEL */
void   RUN(void);	/*** THE SIMULATION RUN IS STARTED ***/
void   ARRIVE(void);	/*** PASSENGERS ENTER THE LINE ***/
void   START(void);	/*** ELEVATOR STARTS ***/
void   CALL(void);	/*** Finds an idling elevator to satisfy the call ***/
void   SEND(void);	/*** Schedule to send the elevator from the idle point to the desired call location, setting the elevator as busy.  ***/
void   STOP(void);	/*** Elevator stops at the next level with passengers scheduled to leave, dropping them off. It picks up any passengers headed in the same direction.  ***/
void   IDLE(void);	/*** Set elevator to idle if it becomes empty ***/
void   EDGE(void);	/*** Flip the elevator's orientation if it's on an edge floor ***/
void   CHECK(void);	/*** Elevator checks for waiting passengers ***/
void   INIELEV(void);	/*** Initiate the elevator vector ***/
void   INIQ(void);	/*** All queues are initialized ***/
void   INIFLOR(void);	/*** Initialize floor arrival nodes ***/
void   DECIDE(void);	/***  ***/
void   REMOVE(void);	/***  ***/

/*   STATE VARIABLE DELARATIONS   */
long   QDOWN[100];	/***  PASSENGERS IN LINE WANTING TO GO DOWN IN A LEVEL  ***/
long   DEST;	/***  FLOOR AT WHICH PASSENGER DEPARTS  ***/
long   ELEVATOR[100];	/***  FLOOR OF ELEVATOR  ***/
long   NFLOORS;	/***  Number of floors  ***/
long   ENTER;	/***  FLOOR AT WHICH PASSENGER ENTERS  ***/
long   QUP[100];	/***  PASSENGERS IN LINE WANTING TO GO UP IN A LEVEL  ***/
long   UPDOWN;	/***  Call to elevator going up or down  ***/
long   CALFLOOR;	/***  The floor at which an elevator is called  ***/
long   TARGELEV;	/***  The target elevator to be brought to the psngr  ***/
long   ID;	/***  Identifier of elevator  ***/
long   IDLELEV;	/***  Number of idle elevators  ***/
long   IDLEFLOR;	/***  Floor at which the elevators idle if not working  ***/
long   ELEVPEOP[100];	/***  Number of people in a given elevator  ***/
long   ELEVCAP;	/***  Preset elevator capacity  ***/
long   REMFLOOR;	/***  Remaining floors for the elevator to travel  ***/
long   X;	/***  Holder  ***/
double Y;	/***  Holder2  ***/
long   OVERF;	/***  Overflow passengers who can't fit in elevator  ***/
long   CHCKEDGE;	/***  Set to 1 if the elevator checks the top floor  ***/
long   NELEV;	/***  Number of elevators  ***/
long   F;	/***  Temporary Variable for Floor Initialization  ***/
long   ROW;	/***  What row to read data from  ***/
long   COL;	/***  What column to read data from  ***/
double READ;	/***  want to see what's being read  ***/
double DELAY;	/***  Time it takes the elevator to traverse one floor  ***/
long   MAXPEOP;	/***  The number of people the elevator has at a stop  ***/
long   LEAVE;	/***  how many people leave an elevator  ***/
long   NUM;	/***  person id  ***/

/*   EVENT FUNCTIONS   */
enum
   {
   run_end_event,
   RUN_event,
   ARRIVE_event,
   START_event,
   CALL_event,
   SEND_event,
   STOP_event,
   IDLE_event,
   EDGE_event,
   CHECK_event,
   INIELEV_event,
   INIQ_event,
   INIFLOR_event,
   DECIDE_event,
   REMOVE_event,
   };

/*    MAIN PROGRAM     */
int main(int argc, char** argv)
{
  int  next_event;
  char keytoclose = 'p';

  if(!startup_check(0))
    return -1;

  /* Initialize csiglib and simulation */
  while (initialize(argc, (const char * *)argv)) {;

  /* Schedule beginning of simulation */
  event_time = current_time;
  event_type = RUN_event;
  schedule_event();

  /* Schedule end of simulation */
  event_time = stop_time;
  event_type = run_end_event;
  event_priority = 9999;
  schedule_event();

/*  EVENT EXECUTION CONTROL LOOP */
  while (!run_error && !done) {
    /* Pull next event from event list */
    next_event = c_timing();

    /* increment the event count for this event */
    event_count[next_event]++;

    /* Call appropriate event routine */
    switch ( next_event ) {
      case run_end_event:  run_end();
               break;

      case RUN_event:  RUN();
               event_trace("RUN",event_count[next_event]);
               break;

      case ARRIVE_event:  ARRIVE();
               event_trace("ARRIVE",event_count[next_event]);
               break;

      case START_event:  START();
               event_trace("START",event_count[next_event]);
               break;

      case CALL_event:  CALL();
               event_trace("CALL",event_count[next_event]);
               break;

      case SEND_event:  SEND();
               event_trace("SEND",event_count[next_event]);
               break;

      case STOP_event:  STOP();
               event_trace("STOP",event_count[next_event]);
               break;

      case IDLE_event:  IDLE();
               event_trace("IDLE",event_count[next_event]);
               break;

      case EDGE_event:  EDGE();
               event_trace("EDGE",event_count[next_event]);
               break;

      case CHECK_event:  CHECK();
               event_trace("CHECK",event_count[next_event]);
               break;

      case INIELEV_event:  INIELEV();
               event_trace("INIELEV",event_count[next_event]);
               break;

      case INIQ_event:  INIQ();
               event_trace("INIQ",event_count[next_event]);
               break;

      case INIFLOR_event:  INIFLOR();
               event_trace("INIFLOR",event_count[next_event]);
               break;

      case DECIDE_event:  DECIDE();
               event_trace("DECIDE",event_count[next_event]);
               break;

      case REMOVE_event:  REMOVE();
               event_trace("REMOVE",event_count[next_event]);
               break;

      }
    }
  }
// experiments terminated
printf("Experiments ended! If runs end early: \n\r1. check fields in *.exp file. \n\r2. check if output file was already open. \n\r");
return 0;
}

void
event_trace(const char * name_of_event,const long count)
{
  c_timest(DEST, 1, 0);
  c_sampst(DEST, 1, 0);
  c_timest(QUP[1], 2, 0);
  c_sampst(QUP[1], 2, 0);
  c_timest(QUP[2], 3, 0);
  c_sampst(QUP[2], 3, 0);
  c_timest(QUP[3], 4, 0);
  c_sampst(QUP[3], 4, 0);
  c_timest(QUP[4], 5, 0);
  c_sampst(QUP[4], 5, 0);
  c_timest(QUP[5], 6, 0);
  c_sampst(QUP[5], 6, 0);
  c_timest(QUP[6], 7, 0);
  c_sampst(QUP[6], 7, 0);
  c_timest(QUP[7], 8, 0);
  c_sampst(QUP[7], 8, 0);
  c_timest(QUP[8], 9, 0);
  c_sampst(QUP[8], 9, 0);
  c_timest(QUP[9], 10, 0);
  c_sampst(QUP[9], 10, 0);
  c_timest(QUP[10], 11, 0);
  c_sampst(QUP[10], 11, 0);
  c_timest(QUP[11], 12, 0);
  c_sampst(QUP[11], 12, 0);
  c_timest(QUP[12], 13, 0);
  c_sampst(QUP[12], 13, 0);
  c_timest(QUP[13], 14, 0);
  c_sampst(QUP[13], 14, 0);
  c_timest(QUP[14], 15, 0);
  c_sampst(QUP[14], 15, 0);
  c_timest(QUP[15], 16, 0);
  c_sampst(QUP[15], 16, 0);
  c_timest(QUP[16], 17, 0);
  c_sampst(QUP[16], 17, 0);
  c_timest(QUP[17], 18, 0);
  c_sampst(QUP[17], 18, 0);
  c_timest(QUP[18], 19, 0);
  c_sampst(QUP[18], 19, 0);
  c_timest(QUP[19], 20, 0);
  c_sampst(QUP[19], 20, 0);
  c_timest(QUP[20], 21, 0);
  c_sampst(QUP[20], 21, 0);
  c_timest(QUP[21], 22, 0);
  c_sampst(QUP[21], 22, 0);
  c_timest(QUP[22], 23, 0);
  c_sampst(QUP[22], 23, 0);
  c_timest(QUP[23], 24, 0);
  c_sampst(QUP[23], 24, 0);
  c_timest(QUP[24], 25, 0);
  c_sampst(QUP[24], 25, 0);
  c_timest(QUP[25], 26, 0);
  c_sampst(QUP[25], 26, 0);
  c_timest(QUP[26], 27, 0);
  c_sampst(QUP[26], 27, 0);
  c_timest(QUP[27], 28, 0);
  c_sampst(QUP[27], 28, 0);
  c_timest(QUP[28], 29, 0);
  c_sampst(QUP[28], 29, 0);
  c_timest(QUP[29], 30, 0);
  c_sampst(QUP[29], 30, 0);
  c_timest(QUP[30], 31, 0);
  c_sampst(QUP[30], 31, 0);
  c_timest(QUP[31], 32, 0);
  c_sampst(QUP[31], 32, 0);
  c_timest(QUP[32], 33, 0);
  c_sampst(QUP[32], 33, 0);
  c_timest(QUP[33], 34, 0);
  c_sampst(QUP[33], 34, 0);
  c_timest(QUP[34], 35, 0);
  c_sampst(QUP[34], 35, 0);
  c_timest(QUP[35], 36, 0);
  c_sampst(QUP[35], 36, 0);
  c_timest(QUP[36], 37, 0);
  c_sampst(QUP[36], 37, 0);
  c_timest(QUP[37], 38, 0);
  c_sampst(QUP[37], 38, 0);
  c_timest(QUP[38], 39, 0);
  c_sampst(QUP[38], 39, 0);
  c_timest(QUP[39], 40, 0);
  c_sampst(QUP[39], 40, 0);
  c_timest(QUP[40], 41, 0);
  c_sampst(QUP[40], 41, 0);
  c_timest(QUP[41], 42, 0);
  c_sampst(QUP[41], 42, 0);
  c_timest(QUP[42], 43, 0);
  c_sampst(QUP[42], 43, 0);
  c_timest(QUP[43], 44, 0);
  c_sampst(QUP[43], 44, 0);
  c_timest(QUP[44], 45, 0);
  c_sampst(QUP[44], 45, 0);
  c_timest(QUP[45], 46, 0);
  c_sampst(QUP[45], 46, 0);
  c_timest(QUP[46], 47, 0);
  c_sampst(QUP[46], 47, 0);
  c_timest(QUP[47], 48, 0);
  c_sampst(QUP[47], 48, 0);
  c_timest(QUP[48], 49, 0);
  c_sampst(QUP[48], 49, 0);
  c_timest(QUP[49], 50, 0);
  c_sampst(QUP[49], 50, 0);
  c_timest(QUP[50], 51, 0);
  c_sampst(QUP[50], 51, 0);
  c_timest(QUP[51], 52, 0);
  c_sampst(QUP[51], 52, 0);
  c_timest(QUP[52], 53, 0);
  c_sampst(QUP[52], 53, 0);
  c_timest(QUP[53], 54, 0);
  c_sampst(QUP[53], 54, 0);
  c_timest(QUP[54], 55, 0);
  c_sampst(QUP[54], 55, 0);
  c_timest(QUP[55], 56, 0);
  c_sampst(QUP[55], 56, 0);
  c_timest(QUP[56], 57, 0);
  c_sampst(QUP[56], 57, 0);
  c_timest(QUP[57], 58, 0);
  c_sampst(QUP[57], 58, 0);
  c_timest(QUP[58], 59, 0);
  c_sampst(QUP[58], 59, 0);
  c_timest(QUP[59], 60, 0);
  c_sampst(QUP[59], 60, 0);
  c_timest(QUP[60], 61, 0);
  c_sampst(QUP[60], 61, 0);
  c_timest(QUP[61], 62, 0);
  c_sampst(QUP[61], 62, 0);
  c_timest(QUP[62], 63, 0);
  c_sampst(QUP[62], 63, 0);
  c_timest(QUP[63], 64, 0);
  c_sampst(QUP[63], 64, 0);
  c_timest(QUP[64], 65, 0);
  c_sampst(QUP[64], 65, 0);
  c_timest(QUP[65], 66, 0);
  c_sampst(QUP[65], 66, 0);
  c_timest(QUP[66], 67, 0);
  c_sampst(QUP[66], 67, 0);
  c_timest(QUP[67], 68, 0);
  c_sampst(QUP[67], 68, 0);
  c_timest(QUP[68], 69, 0);
  c_sampst(QUP[68], 69, 0);
  c_timest(QUP[69], 70, 0);
  c_sampst(QUP[69], 70, 0);
  c_timest(QUP[70], 71, 0);
  c_sampst(QUP[70], 71, 0);
  c_timest(QUP[71], 72, 0);
  c_sampst(QUP[71], 72, 0);
  c_timest(QUP[72], 73, 0);
  c_sampst(QUP[72], 73, 0);
  c_timest(QUP[73], 74, 0);
  c_sampst(QUP[73], 74, 0);
  c_timest(QUP[74], 75, 0);
  c_sampst(QUP[74], 75, 0);
  c_timest(QUP[75], 76, 0);
  c_sampst(QUP[75], 76, 0);
  c_timest(QUP[76], 77, 0);
  c_sampst(QUP[76], 77, 0);
  c_timest(QUP[77], 78, 0);
  c_sampst(QUP[77], 78, 0);
  c_timest(QUP[78], 79, 0);
  c_sampst(QUP[78], 79, 0);
  c_timest(QUP[79], 80, 0);
  c_sampst(QUP[79], 80, 0);
  c_timest(QUP[80], 81, 0);
  c_sampst(QUP[80], 81, 0);
  c_timest(QUP[81], 82, 0);
  c_sampst(QUP[81], 82, 0);
  c_timest(QUP[82], 83, 0);
  c_sampst(QUP[82], 83, 0);
  c_timest(QUP[83], 84, 0);
  c_sampst(QUP[83], 84, 0);
  c_timest(QUP[84], 85, 0);
  c_sampst(QUP[84], 85, 0);
  c_timest(QUP[85], 86, 0);
  c_sampst(QUP[85], 86, 0);
  c_timest(QUP[86], 87, 0);
  c_sampst(QUP[86], 87, 0);
  c_timest(QUP[87], 88, 0);
  c_sampst(QUP[87], 88, 0);
  c_timest(QUP[88], 89, 0);
  c_sampst(QUP[88], 89, 0);
  c_timest(QUP[89], 90, 0);
  c_sampst(QUP[89], 90, 0);
  c_timest(QUP[90], 91, 0);
  c_sampst(QUP[90], 91, 0);
  c_timest(QUP[91], 92, 0);
  c_sampst(QUP[91], 92, 0);
  c_timest(QUP[92], 93, 0);
  c_sampst(QUP[92], 93, 0);
  c_timest(QUP[93], 94, 0);
  c_sampst(QUP[93], 94, 0);
  c_timest(QUP[94], 95, 0);
  c_sampst(QUP[94], 95, 0);
  c_timest(QUP[95], 96, 0);
  c_sampst(QUP[95], 96, 0);
  c_timest(QUP[96], 97, 0);
  c_sampst(QUP[96], 97, 0);
  c_timest(QUP[97], 98, 0);
  c_sampst(QUP[97], 98, 0);
  c_timest(QUP[98], 99, 0);
  c_sampst(QUP[98], 99, 0);
  c_timest(QUP[99], 100, 0);
  c_sampst(QUP[99], 100, 0);
  c_timest(QDOWN[1], 101, 0);
  c_sampst(QDOWN[1], 101, 0);
  c_timest(QDOWN[2], 102, 0);
  c_sampst(QDOWN[2], 102, 0);
  c_timest(QDOWN[3], 103, 0);
  c_sampst(QDOWN[3], 103, 0);
  c_timest(QDOWN[4], 104, 0);
  c_sampst(QDOWN[4], 104, 0);
  c_timest(QDOWN[5], 105, 0);
  c_sampst(QDOWN[5], 105, 0);
  c_timest(QDOWN[6], 106, 0);
  c_sampst(QDOWN[6], 106, 0);
  c_timest(QDOWN[7], 107, 0);
  c_sampst(QDOWN[7], 107, 0);
  c_timest(QDOWN[8], 108, 0);
  c_sampst(QDOWN[8], 108, 0);
  c_timest(QDOWN[9], 109, 0);
  c_sampst(QDOWN[9], 109, 0);
  c_timest(QDOWN[10], 110, 0);
  c_sampst(QDOWN[10], 110, 0);
  c_timest(QDOWN[11], 111, 0);
  c_sampst(QDOWN[11], 111, 0);
  c_timest(QDOWN[12], 112, 0);
  c_sampst(QDOWN[12], 112, 0);
  c_timest(QDOWN[13], 113, 0);
  c_sampst(QDOWN[13], 113, 0);
  c_timest(QDOWN[14], 114, 0);
  c_sampst(QDOWN[14], 114, 0);
  c_timest(QDOWN[15], 115, 0);
  c_sampst(QDOWN[15], 115, 0);
  c_timest(QDOWN[16], 116, 0);
  c_sampst(QDOWN[16], 116, 0);
  c_timest(QDOWN[17], 117, 0);
  c_sampst(QDOWN[17], 117, 0);
  c_timest(QDOWN[18], 118, 0);
  c_sampst(QDOWN[18], 118, 0);
  c_timest(QDOWN[19], 119, 0);
  c_sampst(QDOWN[19], 119, 0);
  c_timest(QDOWN[20], 120, 0);
  c_sampst(QDOWN[20], 120, 0);
  c_timest(QDOWN[21], 121, 0);
  c_sampst(QDOWN[21], 121, 0);
  c_timest(QDOWN[22], 122, 0);
  c_sampst(QDOWN[22], 122, 0);
  c_timest(QDOWN[23], 123, 0);
  c_sampst(QDOWN[23], 123, 0);
  c_timest(QDOWN[24], 124, 0);
  c_sampst(QDOWN[24], 124, 0);
  c_timest(QDOWN[25], 125, 0);
  c_sampst(QDOWN[25], 125, 0);
  c_timest(QDOWN[26], 126, 0);
  c_sampst(QDOWN[26], 126, 0);
  c_timest(QDOWN[27], 127, 0);
  c_sampst(QDOWN[27], 127, 0);
  c_timest(QDOWN[28], 128, 0);
  c_sampst(QDOWN[28], 128, 0);
  c_timest(QDOWN[29], 129, 0);
  c_sampst(QDOWN[29], 129, 0);
  c_timest(QDOWN[30], 130, 0);
  c_sampst(QDOWN[30], 130, 0);
  c_timest(QDOWN[31], 131, 0);
  c_sampst(QDOWN[31], 131, 0);
  c_timest(QDOWN[32], 132, 0);
  c_sampst(QDOWN[32], 132, 0);
  c_timest(QDOWN[33], 133, 0);
  c_sampst(QDOWN[33], 133, 0);
  c_timest(QDOWN[34], 134, 0);
  c_sampst(QDOWN[34], 134, 0);
  c_timest(QDOWN[35], 135, 0);
  c_sampst(QDOWN[35], 135, 0);
  c_timest(QDOWN[36], 136, 0);
  c_sampst(QDOWN[36], 136, 0);
  c_timest(QDOWN[37], 137, 0);
  c_sampst(QDOWN[37], 137, 0);
  c_timest(QDOWN[38], 138, 0);
  c_sampst(QDOWN[38], 138, 0);
  c_timest(QDOWN[39], 139, 0);
  c_sampst(QDOWN[39], 139, 0);
  c_timest(QDOWN[40], 140, 0);
  c_sampst(QDOWN[40], 140, 0);
  c_timest(QDOWN[41], 141, 0);
  c_sampst(QDOWN[41], 141, 0);
  c_timest(QDOWN[42], 142, 0);
  c_sampst(QDOWN[42], 142, 0);
  c_timest(QDOWN[43], 143, 0);
  c_sampst(QDOWN[43], 143, 0);
  c_timest(QDOWN[44], 144, 0);
  c_sampst(QDOWN[44], 144, 0);
  c_timest(QDOWN[45], 145, 0);
  c_sampst(QDOWN[45], 145, 0);
  c_timest(QDOWN[46], 146, 0);
  c_sampst(QDOWN[46], 146, 0);
  c_timest(QDOWN[47], 147, 0);
  c_sampst(QDOWN[47], 147, 0);
  c_timest(QDOWN[48], 148, 0);
  c_sampst(QDOWN[48], 148, 0);
  c_timest(QDOWN[49], 149, 0);
  c_sampst(QDOWN[49], 149, 0);
  c_timest(QDOWN[50], 150, 0);
  c_sampst(QDOWN[50], 150, 0);
  c_timest(QDOWN[51], 151, 0);
  c_sampst(QDOWN[51], 151, 0);
  c_timest(QDOWN[52], 152, 0);
  c_sampst(QDOWN[52], 152, 0);
  c_timest(QDOWN[53], 153, 0);
  c_sampst(QDOWN[53], 153, 0);
  c_timest(QDOWN[54], 154, 0);
  c_sampst(QDOWN[54], 154, 0);
  c_timest(QDOWN[55], 155, 0);
  c_sampst(QDOWN[55], 155, 0);
  c_timest(QDOWN[56], 156, 0);
  c_sampst(QDOWN[56], 156, 0);
  c_timest(QDOWN[57], 157, 0);
  c_sampst(QDOWN[57], 157, 0);
  c_timest(QDOWN[58], 158, 0);
  c_sampst(QDOWN[58], 158, 0);
  c_timest(QDOWN[59], 159, 0);
  c_sampst(QDOWN[59], 159, 0);
  c_timest(QDOWN[60], 160, 0);
  c_sampst(QDOWN[60], 160, 0);
  c_timest(QDOWN[61], 161, 0);
  c_sampst(QDOWN[61], 161, 0);
  c_timest(QDOWN[62], 162, 0);
  c_sampst(QDOWN[62], 162, 0);
  c_timest(QDOWN[63], 163, 0);
  c_sampst(QDOWN[63], 163, 0);
  c_timest(QDOWN[64], 164, 0);
  c_sampst(QDOWN[64], 164, 0);
  c_timest(QDOWN[65], 165, 0);
  c_sampst(QDOWN[65], 165, 0);
  c_timest(QDOWN[66], 166, 0);
  c_sampst(QDOWN[66], 166, 0);
  c_timest(QDOWN[67], 167, 0);
  c_sampst(QDOWN[67], 167, 0);
  c_timest(QDOWN[68], 168, 0);
  c_sampst(QDOWN[68], 168, 0);
  c_timest(QDOWN[69], 169, 0);
  c_sampst(QDOWN[69], 169, 0);
  c_timest(QDOWN[70], 170, 0);
  c_sampst(QDOWN[70], 170, 0);
  c_timest(QDOWN[71], 171, 0);
  c_sampst(QDOWN[71], 171, 0);
  c_timest(QDOWN[72], 172, 0);
  c_sampst(QDOWN[72], 172, 0);
  if(trace_flag) fprintf(output_fp, "%9.3f\t %6s\t%6d ",current_time,name_of_event,count);
  if(trace_flag) fprintf(output_fp, "	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g 	%7.3g \n"
,(double)DEST, (double)QUP[1], (double)QUP[2], (double)QUP[3], (double)QUP[4], (double)QUP[5], (double)QUP[6], (double)QUP[7], (double)QUP[8], (double)QUP[9], (double)QUP[10], (double)QUP[11], (double)QUP[12], (double)QUP[13], (double)QUP[14], (double)QUP[15], (double)QUP[16], (double)QUP[17], (double)QUP[18], (double)QUP[19], (double)QUP[20], (double)QUP[21], (double)QUP[22], (double)QUP[23], (double)QUP[24], (double)QUP[25], (double)QUP[26], (double)QUP[27], (double)QUP[28], (double)QUP[29], (double)QUP[30], (double)QUP[31], (double)QUP[32], (double)QUP[33], (double)QUP[34], (double)QUP[35], (double)QUP[36], (double)QUP[37], (double)QUP[38], (double)QUP[39], (double)QUP[40], (double)QUP[41], (double)QUP[42], (double)QUP[43], (double)QUP[44], (double)QUP[45], (double)QUP[46], (double)QUP[47], (double)QUP[48], (double)QUP[49], (double)QUP[50], (double)QUP[51], (double)QUP[52], (double)QUP[53], (double)QUP[54], (double)QUP[55], (double)QUP[56], (double)QUP[57], (double)QUP[58], (double)QUP[59], (double)QUP[60], (double)QUP[61], (double)QUP[62], (double)QUP[63], (double)QUP[64], (double)QUP[65], (double)QUP[66], (double)QUP[67], (double)QUP[68], (double)QUP[69], (double)QUP[70], (double)QUP[71], (double)QUP[72], (double)QUP[73], (double)QUP[74], (double)QUP[75], (double)QUP[76], (double)QUP[77], (double)QUP[78], (double)QUP[79], (double)QUP[80], (double)QUP[81], (double)QUP[82], (double)QUP[83], (double)QUP[84], (double)QUP[85], (double)QUP[86], (double)QUP[87], (double)QUP[88], (double)QUP[89], (double)QUP[90], (double)QUP[91], (double)QUP[92], (double)QUP[93], (double)QUP[94], (double)QUP[95], (double)QUP[96], (double)QUP[97], (double)QUP[98], (double)QUP[99], (double)QDOWN[1], (double)QDOWN[2], (double)QDOWN[3], (double)QDOWN[4], (double)QDOWN[5], (double)QDOWN[6], (double)QDOWN[7], (double)QDOWN[8], (double)QDOWN[9], (double)QDOWN[10], (double)QDOWN[11], (double)QDOWN[12], (double)QDOWN[13], (double)QDOWN[14], (double)QDOWN[15], (double)QDOWN[16], (double)QDOWN[17], (double)QDOWN[18], (double)QDOWN[19], (double)QDOWN[20], (double)QDOWN[21], (double)QDOWN[22], (double)QDOWN[23], (double)QDOWN[24], (double)QDOWN[25], (double)QDOWN[26], (double)QDOWN[27], (double)QDOWN[28], (double)QDOWN[29], (double)QDOWN[30], (double)QDOWN[31], (double)QDOWN[32], (double)QDOWN[33], (double)QDOWN[34], (double)QDOWN[35], (double)QDOWN[36], (double)QDOWN[37], (double)QDOWN[38], (double)QDOWN[39], (double)QDOWN[40], (double)QDOWN[41], (double)QDOWN[42], (double)QDOWN[43], (double)QDOWN[44], (double)QDOWN[45], (double)QDOWN[46], (double)QDOWN[47], (double)QDOWN[48], (double)QDOWN[49], (double)QDOWN[50], (double)QDOWN[51], (double)QDOWN[52], (double)QDOWN[53], (double)QDOWN[54], (double)QDOWN[55], (double)QDOWN[56], (double)QDOWN[57], (double)QDOWN[58], (double)QDOWN[59], (double)QDOWN[60], (double)QDOWN[61], (double)QDOWN[62], (double)QDOWN[63], (double)QDOWN[64], (double)QDOWN[65], (double)QDOWN[66], (double)QDOWN[67], (double)QDOWN[68], (double)QDOWN[69], (double)QDOWN[70], (double)QDOWN[71], (double)QDOWN[72]);
}



int
initialize(int argc, const char** argv)
{
static int first_time = 1;     /* First time in initialize? */
static FILE *input_fp;     /* For reading from the input file */
char *exp_file_name;       /* For constructing input file name */
char y_n = 'p';            /* yes/no for file overwrite*/

       char dir[256];
       char fname[256];
       char ext[256];
       char simulation[1024];
       char experient_name[1024];
        _splitpath( argv[0], NULL, dir, fname, ext );
       strcpy(simulation, fname);
       strcat(simulation, ext);
       strcpy(experient_name, fname);
       strcat(experient_name, ".exp");
     printf("Running the simulation: %s\n", simulation);
    if(strlen(dir) !=0)
       printf("In Path: %s\n",dir);
  if (first_time) {
    exp_file_name = _strdup(argv[0]);
    exp_file_name[strlen(exp_file_name)-1] = 'p';
    printf("\nLooking for experiment file: %s\n",experient_name);
    }

  if ((first_time && (input_fp=fopen(exp_file_name,"r"))!=NULL)
                                           || input_fp!=NULL) {
  if (first_time) {
     first_time = 0; /* Reset for next time into initialize */
     printf("Found. Use [Control]-C to abort replications.\n");
     }

  /* We have run control file of type *.exp          */
  /* Read next set of data from run control file.    */
  if (fscanf(input_fp,"%s %1s %ld %lf %d", output_file_name, &y_n, &rndsd, &stop_time, &trace_flag)<4
     || fscanf(input_fp,"%ld", &NFLOORS)<1
     || fscanf(input_fp,"%ld", &NELEV)<1
     || fscanf(input_fp,"%ld", &ELEVCAP)<1
     || fscanf(input_fp,"%ld", &IDLEFLOR)<1
     || fscanf(input_fp,"%lf", &DELAY)<1
     ) {
     /* End of run control file */
     fclose(input_fp);
     return 0;
     }

  if (y_n != 'y' && y_n != 'Y' && y_n != 'n' && y_n != 'N') { 
  fprintf(stderr,"INPUT ERROR: Invalid append file flag in *.exp file: (y=append, n=overwrite old file)\n"); 
  return 0; 
  }

  if (y_n == 'y' || y_n == 'Y') {
     if ((output_fp = fopen(output_file_name,"a"))==NULL) { 
     /* Output file can't be opened. */
     fprintf(stderr,"\nINPUT ERROR: Cannot open output file %s in *.exp file\n",argv[1]);
  return 0; 
  }
  }
if (y_n == 'n' || y_n == 'N') {
     if ((output_fp = fopen(output_file_name,"w"))==NULL) { 
     /* Output file can't be opened. */
     fprintf(stderr,"\nINPUT ERROR: Cannot open output file %s in *.exp file\n",argv[1]);
     return 0;
     }
     }

  if (rndsd < 1 || rndsd > 65534) {
     fprintf(stderr,"\nINPUT ERROR: Random seed %ld is not between 0 and 65534\n",rndsd);
     return 0;
     }

  if (stop_time <= 0.0) {
     fprintf(stderr,"\nINPUT ERROR: Stopping time %lf is negative!\n",stop_time);
     return 0;
     }

  if (trace_flag != 0 && trace_flag != 1) {
     fprintf(stderr,"\nINPUT ERROR: Invalid trace_flag=%d: (1=full trace, 0=summary only)\n",trace_flag);
     return 0;
     }

  done = 0;
     }

 else if (first_time) { /* And open failed, implies data from stdin */
    first_time = 0; /* Reset for next time into initialize */
    printf("Not found, input data at the keyboard.\n");
     /* Give output file name */
     while(y_n != 'y' && y_n != 'Y' && y_n != 'n' && y_n != 'N')
         {
         printf("\nOUTPUT FILE (Enter File Name with Path):\n");
         scanf("%s", output_file_name);
         fflush(stdin);
         sprintf(filename,"%.20s", output_file_name);
         printf("WARNING:File %.20s must not be open!!\n If file does not exist it will be created.\n",filename);
         printf("Do you want the new output appended to this file? (yes/[no])\n");
         scanf("%1s",&y_n);
         fflush(stdin);
         }
     if(y_n == 'y' || y_n == 'Y') output_fp = fopen(filename,"a");
     if(y_n == 'n' || y_n == 'N') output_fp = fopen(filename,"w");

     /* Read in random number seed */
     printf("\n\nRANDOM NUMBER SEED (Enter Integer Between 0 and 65534):\n");
     scanf("%ld", &rndsd);
     fflush(stdin);

     /* Read in run stopping time */
     printf("\nSTOPPING TIME (Enter number of time units until run termination):\n");
     scanf("%lf", &stop_time);
     fflush(stdin);

     /* Read in trace_flag */
     printf("\n\nTRACE (1 = Event Trace, 0 = Summary Only):\n");
     scanf("%d", &trace_flag);
     fflush(stdin);

     /* Parameters for the initial event */;
     printf ( "\nEnter initial value for NFLOORS: \n");
     scanf  ( "%ld", &NFLOORS);
     printf ( "\nEnter initial value for NELEV: \n");
     scanf  ( "%ld", &NELEV);
     printf ( "\nEnter initial value for ELEVCAP: \n");
     scanf  ( "%ld", &ELEVCAP);
     printf ( "\nEnter initial value for IDLEFLOR: \n");
     scanf  ( "%ld", &IDLEFLOR);
     printf ( "\nEnter initial value for DELAY: \n");
     scanf  ( "%lf", &DELAY);
   }

 else {
    /* this is not the first time and there is no .exp file */
    return 0;
    }
  
   /* PLACE CUSTOMIZED INITIALIZATIONS HERE */

if (trace_flag)
   {
   fprintf(output_fp,"    Time\t  Event\t Count");
   fprintf(output_fp,"	            DEST");
   fprintf(output_fp,"	          QUP[1]");
   fprintf(output_fp,"	          QUP[2]");
   fprintf(output_fp,"	          QUP[3]");
   fprintf(output_fp,"	          QUP[4]");
   fprintf(output_fp,"	          QUP[5]");
   fprintf(output_fp,"	          QUP[6]");
   fprintf(output_fp,"	          QUP[7]");
   fprintf(output_fp,"	          QUP[8]");
   fprintf(output_fp,"	          QUP[9]");
   fprintf(output_fp,"	         QUP[10]");
   fprintf(output_fp,"	         QUP[11]");
   fprintf(output_fp,"	         QUP[12]");
   fprintf(output_fp,"	         QUP[13]");
   fprintf(output_fp,"	         QUP[14]");
   fprintf(output_fp,"	         QUP[15]");
   fprintf(output_fp,"	         QUP[16]");
   fprintf(output_fp,"	         QUP[17]");
   fprintf(output_fp,"	         QUP[18]");
   fprintf(output_fp,"	         QUP[19]");
   fprintf(output_fp,"	         QUP[20]");
   fprintf(output_fp,"	         QUP[21]");
   fprintf(output_fp,"	         QUP[22]");
   fprintf(output_fp,"	         QUP[23]");
   fprintf(output_fp,"	         QUP[24]");
   fprintf(output_fp,"	         QUP[25]");
   fprintf(output_fp,"	         QUP[26]");
   fprintf(output_fp,"	         QUP[27]");
   fprintf(output_fp,"	         QUP[28]");
   fprintf(output_fp,"	         QUP[29]");
   fprintf(output_fp,"	         QUP[30]");
   fprintf(output_fp,"	         QUP[31]");
   fprintf(output_fp,"	         QUP[32]");
   fprintf(output_fp,"	         QUP[33]");
   fprintf(output_fp,"	         QUP[34]");
   fprintf(output_fp,"	         QUP[35]");
   fprintf(output_fp,"	         QUP[36]");
   fprintf(output_fp,"	         QUP[37]");
   fprintf(output_fp,"	         QUP[38]");
   fprintf(output_fp,"	         QUP[39]");
   fprintf(output_fp,"	         QUP[40]");
   fprintf(output_fp,"	         QUP[41]");
   fprintf(output_fp,"	         QUP[42]");
   fprintf(output_fp,"	         QUP[43]");
   fprintf(output_fp,"	         QUP[44]");
   fprintf(output_fp,"	         QUP[45]");
   fprintf(output_fp,"	         QUP[46]");
   fprintf(output_fp,"	         QUP[47]");
   fprintf(output_fp,"	         QUP[48]");
   fprintf(output_fp,"	         QUP[49]");
   fprintf(output_fp,"	         QUP[50]");
   fprintf(output_fp,"	         QUP[51]");
   fprintf(output_fp,"	         QUP[52]");
   fprintf(output_fp,"	         QUP[53]");
   fprintf(output_fp,"	         QUP[54]");
   fprintf(output_fp,"	         QUP[55]");
   fprintf(output_fp,"	         QUP[56]");
   fprintf(output_fp,"	         QUP[57]");
   fprintf(output_fp,"	         QUP[58]");
   fprintf(output_fp,"	         QUP[59]");
   fprintf(output_fp,"	         QUP[60]");
   fprintf(output_fp,"	         QUP[61]");
   fprintf(output_fp,"	         QUP[62]");
   fprintf(output_fp,"	         QUP[63]");
   fprintf(output_fp,"	         QUP[64]");
   fprintf(output_fp,"	         QUP[65]");
   fprintf(output_fp,"	         QUP[66]");
   fprintf(output_fp,"	         QUP[67]");
   fprintf(output_fp,"	         QUP[68]");
   fprintf(output_fp,"	         QUP[69]");
   fprintf(output_fp,"	         QUP[70]");
   fprintf(output_fp,"	         QUP[71]");
   fprintf(output_fp,"	         QUP[72]");
   fprintf(output_fp,"	         QUP[73]");
   fprintf(output_fp,"	         QUP[74]");
   fprintf(output_fp,"	         QUP[75]");
   fprintf(output_fp,"	         QUP[76]");
   fprintf(output_fp,"	         QUP[77]");
   fprintf(output_fp,"	         QUP[78]");
   fprintf(output_fp,"	         QUP[79]");
   fprintf(output_fp,"	         QUP[80]");
   fprintf(output_fp,"	         QUP[81]");
   fprintf(output_fp,"	         QUP[82]");
   fprintf(output_fp,"	         QUP[83]");
   fprintf(output_fp,"	         QUP[84]");
   fprintf(output_fp,"	         QUP[85]");
   fprintf(output_fp,"	         QUP[86]");
   fprintf(output_fp,"	         QUP[87]");
   fprintf(output_fp,"	         QUP[88]");
   fprintf(output_fp,"	         QUP[89]");
   fprintf(output_fp,"	         QUP[90]");
   fprintf(output_fp,"	         QUP[91]");
   fprintf(output_fp,"	         QUP[92]");
   fprintf(output_fp,"	         QUP[93]");
   fprintf(output_fp,"	         QUP[94]");
   fprintf(output_fp,"	         QUP[95]");
   fprintf(output_fp,"	         QUP[96]");
   fprintf(output_fp,"	         QUP[97]");
   fprintf(output_fp,"	         QUP[98]");
   fprintf(output_fp,"	         QUP[99]");
   fprintf(output_fp,"	        QDOWN[1]");
   fprintf(output_fp,"	        QDOWN[2]");
   fprintf(output_fp,"	        QDOWN[3]");
   fprintf(output_fp,"	        QDOWN[4]");
   fprintf(output_fp,"	        QDOWN[5]");
   fprintf(output_fp,"	        QDOWN[6]");
   fprintf(output_fp,"	        QDOWN[7]");
   fprintf(output_fp,"	        QDOWN[8]");
   fprintf(output_fp,"	        QDOWN[9]");
   fprintf(output_fp,"	       QDOWN[10]");
   fprintf(output_fp,"	       QDOWN[11]");
   fprintf(output_fp,"	       QDOWN[12]");
   fprintf(output_fp,"	       QDOWN[13]");
   fprintf(output_fp,"	       QDOWN[14]");
   fprintf(output_fp,"	       QDOWN[15]");
   fprintf(output_fp,"	       QDOWN[16]");
   fprintf(output_fp,"	       QDOWN[17]");
   fprintf(output_fp,"	       QDOWN[18]");
   fprintf(output_fp,"	       QDOWN[19]");
   fprintf(output_fp,"	       QDOWN[20]");
   fprintf(output_fp,"	       QDOWN[21]");
   fprintf(output_fp,"	       QDOWN[22]");
   fprintf(output_fp,"	       QDOWN[23]");
   fprintf(output_fp,"	       QDOWN[24]");
   fprintf(output_fp,"	       QDOWN[25]");
   fprintf(output_fp,"	       QDOWN[26]");
   fprintf(output_fp,"	       QDOWN[27]");
   fprintf(output_fp,"	       QDOWN[28]");
   fprintf(output_fp,"	       QDOWN[29]");
   fprintf(output_fp,"	       QDOWN[30]");
   fprintf(output_fp,"	       QDOWN[31]");
   fprintf(output_fp,"	       QDOWN[32]");
   fprintf(output_fp,"	       QDOWN[33]");
   fprintf(output_fp,"	       QDOWN[34]");
   fprintf(output_fp,"	       QDOWN[35]");
   fprintf(output_fp,"	       QDOWN[36]");
   fprintf(output_fp,"	       QDOWN[37]");
   fprintf(output_fp,"	       QDOWN[38]");
   fprintf(output_fp,"	       QDOWN[39]");
   fprintf(output_fp,"	       QDOWN[40]");
   fprintf(output_fp,"	       QDOWN[41]");
   fprintf(output_fp,"	       QDOWN[42]");
   fprintf(output_fp,"	       QDOWN[43]");
   fprintf(output_fp,"	       QDOWN[44]");
   fprintf(output_fp,"	       QDOWN[45]");
   fprintf(output_fp,"	       QDOWN[46]");
   fprintf(output_fp,"	       QDOWN[47]");
   fprintf(output_fp,"	       QDOWN[48]");
   fprintf(output_fp,"	       QDOWN[49]");
   fprintf(output_fp,"	       QDOWN[50]");
   fprintf(output_fp,"	       QDOWN[51]");
   fprintf(output_fp,"	       QDOWN[52]");
   fprintf(output_fp,"	       QDOWN[53]");
   fprintf(output_fp,"	       QDOWN[54]");
   fprintf(output_fp,"	       QDOWN[55]");
   fprintf(output_fp,"	       QDOWN[56]");
   fprintf(output_fp,"	       QDOWN[57]");
   fprintf(output_fp,"	       QDOWN[58]");
   fprintf(output_fp,"	       QDOWN[59]");
   fprintf(output_fp,"	       QDOWN[60]");
   fprintf(output_fp,"	       QDOWN[61]");
   fprintf(output_fp,"	       QDOWN[62]");
   fprintf(output_fp,"	       QDOWN[63]");
   fprintf(output_fp,"	       QDOWN[64]");
   fprintf(output_fp,"	       QDOWN[65]");
   fprintf(output_fp,"	       QDOWN[66]");
   fprintf(output_fp,"	       QDOWN[67]");
   fprintf(output_fp,"	       QDOWN[68]");
   fprintf(output_fp,"	       QDOWN[69]");
   fprintf(output_fp,"	       QDOWN[70]");
   fprintf(output_fp,"	       QDOWN[71]");
   fprintf(output_fp,"	         QDOWN[72] ");
   fprintf(output_fp,"\n");
   }
  /* Initialize CSIGLIB variables and files */
  c_initlk(rndsd);
  c_initfiles();

  return(1);
}



void
run_end()
{
  printf("\r\nNormal completion after %f time units\n",current_time);
  printf("The Next Seed In the Random Input Stream is %ld\n",rndsd);

  ///  Summary statistics ///
  fprintf(output_fp,"SUMMARY STATISTICS\n");
  printf("SUMMARY STATISTICS\n");
  c_timest(DEST, 1, 1);
   fprintf(output_fp, "DEST:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("DEST:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(DEST, 1, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[1], 2, 1);
   fprintf(output_fp, "QUP[1]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[1]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[1], 2, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[2], 3, 1);
   fprintf(output_fp, "QUP[2]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[2]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[2], 3, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[3], 4, 1);
   fprintf(output_fp, "QUP[3]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[3]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[3], 4, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[4], 5, 1);
   fprintf(output_fp, "QUP[4]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[4]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[4], 5, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[5], 6, 1);
   fprintf(output_fp, "QUP[5]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[5]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[5], 6, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[6], 7, 1);
   fprintf(output_fp, "QUP[6]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[6]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[6], 7, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[7], 8, 1);
   fprintf(output_fp, "QUP[7]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[7]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[7], 8, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[8], 9, 1);
   fprintf(output_fp, "QUP[8]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[8]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[8], 9, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[9], 10, 1);
   fprintf(output_fp, "QUP[9]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[9]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[9], 10, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[10], 11, 1);
   fprintf(output_fp, "QUP[10]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[10]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[10], 11, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[11], 12, 1);
   fprintf(output_fp, "QUP[11]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[11]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[11], 12, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[12], 13, 1);
   fprintf(output_fp, "QUP[12]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[12]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[12], 13, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[13], 14, 1);
   fprintf(output_fp, "QUP[13]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[13]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[13], 14, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[14], 15, 1);
   fprintf(output_fp, "QUP[14]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[14]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[14], 15, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[15], 16, 1);
   fprintf(output_fp, "QUP[15]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[15]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[15], 16, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[16], 17, 1);
   fprintf(output_fp, "QUP[16]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[16]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[16], 17, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[17], 18, 1);
   fprintf(output_fp, "QUP[17]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[17]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[17], 18, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[18], 19, 1);
   fprintf(output_fp, "QUP[18]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[18]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[18], 19, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[19], 20, 1);
   fprintf(output_fp, "QUP[19]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[19]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[19], 20, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[20], 21, 1);
   fprintf(output_fp, "QUP[20]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[20]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[20], 21, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[21], 22, 1);
   fprintf(output_fp, "QUP[21]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[21]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[21], 22, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[22], 23, 1);
   fprintf(output_fp, "QUP[22]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[22]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[22], 23, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[23], 24, 1);
   fprintf(output_fp, "QUP[23]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[23]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[23], 24, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[24], 25, 1);
   fprintf(output_fp, "QUP[24]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[24]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[24], 25, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[25], 26, 1);
   fprintf(output_fp, "QUP[25]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[25]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[25], 26, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[26], 27, 1);
   fprintf(output_fp, "QUP[26]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[26]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[26], 27, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[27], 28, 1);
   fprintf(output_fp, "QUP[27]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[27]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[27], 28, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[28], 29, 1);
   fprintf(output_fp, "QUP[28]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[28]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[28], 29, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[29], 30, 1);
   fprintf(output_fp, "QUP[29]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[29]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[29], 30, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[30], 31, 1);
   fprintf(output_fp, "QUP[30]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[30]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[30], 31, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[31], 32, 1);
   fprintf(output_fp, "QUP[31]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[31]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[31], 32, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[32], 33, 1);
   fprintf(output_fp, "QUP[32]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[32]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[32], 33, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[33], 34, 1);
   fprintf(output_fp, "QUP[33]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[33]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[33], 34, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[34], 35, 1);
   fprintf(output_fp, "QUP[34]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[34]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[34], 35, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[35], 36, 1);
   fprintf(output_fp, "QUP[35]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[35]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[35], 36, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[36], 37, 1);
   fprintf(output_fp, "QUP[36]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[36]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[36], 37, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[37], 38, 1);
   fprintf(output_fp, "QUP[37]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[37]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[37], 38, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[38], 39, 1);
   fprintf(output_fp, "QUP[38]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[38]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[38], 39, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[39], 40, 1);
   fprintf(output_fp, "QUP[39]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[39]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[39], 40, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[40], 41, 1);
   fprintf(output_fp, "QUP[40]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[40]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[40], 41, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[41], 42, 1);
   fprintf(output_fp, "QUP[41]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[41]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[41], 42, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[42], 43, 1);
   fprintf(output_fp, "QUP[42]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[42]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[42], 43, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[43], 44, 1);
   fprintf(output_fp, "QUP[43]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[43]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[43], 44, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[44], 45, 1);
   fprintf(output_fp, "QUP[44]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[44]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[44], 45, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[45], 46, 1);
   fprintf(output_fp, "QUP[45]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[45]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[45], 46, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[46], 47, 1);
   fprintf(output_fp, "QUP[46]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[46]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[46], 47, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[47], 48, 1);
   fprintf(output_fp, "QUP[47]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[47]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[47], 48, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[48], 49, 1);
   fprintf(output_fp, "QUP[48]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[48]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[48], 49, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[49], 50, 1);
   fprintf(output_fp, "QUP[49]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[49]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[49], 50, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[50], 51, 1);
   fprintf(output_fp, "QUP[50]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[50]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[50], 51, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[51], 52, 1);
   fprintf(output_fp, "QUP[51]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[51]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[51], 52, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[52], 53, 1);
   fprintf(output_fp, "QUP[52]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[52]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[52], 53, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[53], 54, 1);
   fprintf(output_fp, "QUP[53]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[53]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[53], 54, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[54], 55, 1);
   fprintf(output_fp, "QUP[54]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[54]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[54], 55, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[55], 56, 1);
   fprintf(output_fp, "QUP[55]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[55]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[55], 56, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[56], 57, 1);
   fprintf(output_fp, "QUP[56]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[56]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[56], 57, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[57], 58, 1);
   fprintf(output_fp, "QUP[57]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[57]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[57], 58, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[58], 59, 1);
   fprintf(output_fp, "QUP[58]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[58]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[58], 59, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[59], 60, 1);
   fprintf(output_fp, "QUP[59]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[59]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[59], 60, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[60], 61, 1);
   fprintf(output_fp, "QUP[60]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[60]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[60], 61, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[61], 62, 1);
   fprintf(output_fp, "QUP[61]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[61]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[61], 62, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[62], 63, 1);
   fprintf(output_fp, "QUP[62]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[62]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[62], 63, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[63], 64, 1);
   fprintf(output_fp, "QUP[63]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[63]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[63], 64, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[64], 65, 1);
   fprintf(output_fp, "QUP[64]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[64]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[64], 65, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[65], 66, 1);
   fprintf(output_fp, "QUP[65]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[65]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[65], 66, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[66], 67, 1);
   fprintf(output_fp, "QUP[66]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[66]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[66], 67, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[67], 68, 1);
   fprintf(output_fp, "QUP[67]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[67]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[67], 68, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[68], 69, 1);
   fprintf(output_fp, "QUP[68]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[68]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[68], 69, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[69], 70, 1);
   fprintf(output_fp, "QUP[69]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[69]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[69], 70, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[70], 71, 1);
   fprintf(output_fp, "QUP[70]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[70]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[70], 71, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[71], 72, 1);
   fprintf(output_fp, "QUP[71]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[71]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[71], 72, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[72], 73, 1);
   fprintf(output_fp, "QUP[72]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[72]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[72], 73, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[73], 74, 1);
   fprintf(output_fp, "QUP[73]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[73]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[73], 74, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[74], 75, 1);
   fprintf(output_fp, "QUP[74]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[74]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[74], 75, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[75], 76, 1);
   fprintf(output_fp, "QUP[75]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[75]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[75], 76, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[76], 77, 1);
   fprintf(output_fp, "QUP[76]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[76]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[76], 77, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[77], 78, 1);
   fprintf(output_fp, "QUP[77]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[77]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[77], 78, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[78], 79, 1);
   fprintf(output_fp, "QUP[78]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[78]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[78], 79, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[79], 80, 1);
   fprintf(output_fp, "QUP[79]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[79]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[79], 80, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[80], 81, 1);
   fprintf(output_fp, "QUP[80]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[80]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[80], 81, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[81], 82, 1);
   fprintf(output_fp, "QUP[81]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[81]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[81], 82, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[82], 83, 1);
   fprintf(output_fp, "QUP[82]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[82]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[82], 83, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[83], 84, 1);
   fprintf(output_fp, "QUP[83]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[83]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[83], 84, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[84], 85, 1);
   fprintf(output_fp, "QUP[84]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[84]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[84], 85, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[85], 86, 1);
   fprintf(output_fp, "QUP[85]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[85]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[85], 86, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[86], 87, 1);
   fprintf(output_fp, "QUP[86]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[86]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[86], 87, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[87], 88, 1);
   fprintf(output_fp, "QUP[87]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[87]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[87], 88, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[88], 89, 1);
   fprintf(output_fp, "QUP[88]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[88]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[88], 89, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[89], 90, 1);
   fprintf(output_fp, "QUP[89]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[89]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[89], 90, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[90], 91, 1);
   fprintf(output_fp, "QUP[90]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[90]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[90], 91, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[91], 92, 1);
   fprintf(output_fp, "QUP[91]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[91]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[91], 92, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[92], 93, 1);
   fprintf(output_fp, "QUP[92]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[92]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[92], 93, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[93], 94, 1);
   fprintf(output_fp, "QUP[93]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[93]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[93], 94, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[94], 95, 1);
   fprintf(output_fp, "QUP[94]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[94]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[94], 95, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[95], 96, 1);
   fprintf(output_fp, "QUP[95]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[95]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[95], 96, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[96], 97, 1);
   fprintf(output_fp, "QUP[96]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[96]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[96], 97, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[97], 98, 1);
   fprintf(output_fp, "QUP[97]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[97]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[97], 98, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[98], 99, 1);
   fprintf(output_fp, "QUP[98]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[98]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[98], 99, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QUP[99], 100, 1);
   fprintf(output_fp, "QUP[99]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QUP[99]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QUP[99], 100, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[1], 101, 1);
   fprintf(output_fp, "QDOWN[1]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[1]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[1], 101, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[2], 102, 1);
   fprintf(output_fp, "QDOWN[2]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[2]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[2], 102, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[3], 103, 1);
   fprintf(output_fp, "QDOWN[3]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[3]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[3], 103, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[4], 104, 1);
   fprintf(output_fp, "QDOWN[4]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[4]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[4], 104, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[5], 105, 1);
   fprintf(output_fp, "QDOWN[5]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[5]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[5], 105, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[6], 106, 1);
   fprintf(output_fp, "QDOWN[6]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[6]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[6], 106, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[7], 107, 1);
   fprintf(output_fp, "QDOWN[7]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[7]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[7], 107, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[8], 108, 1);
   fprintf(output_fp, "QDOWN[8]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[8]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[8], 108, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[9], 109, 1);
   fprintf(output_fp, "QDOWN[9]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[9]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[9], 109, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[10], 110, 1);
   fprintf(output_fp, "QDOWN[10]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[10]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[10], 110, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[11], 111, 1);
   fprintf(output_fp, "QDOWN[11]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[11]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[11], 111, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[12], 112, 1);
   fprintf(output_fp, "QDOWN[12]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[12]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[12], 112, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[13], 113, 1);
   fprintf(output_fp, "QDOWN[13]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[13]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[13], 113, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[14], 114, 1);
   fprintf(output_fp, "QDOWN[14]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[14]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[14], 114, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[15], 115, 1);
   fprintf(output_fp, "QDOWN[15]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[15]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[15], 115, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[16], 116, 1);
   fprintf(output_fp, "QDOWN[16]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[16]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[16], 116, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[17], 117, 1);
   fprintf(output_fp, "QDOWN[17]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[17]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[17], 117, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[18], 118, 1);
   fprintf(output_fp, "QDOWN[18]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[18]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[18], 118, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[19], 119, 1);
   fprintf(output_fp, "QDOWN[19]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[19]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[19], 119, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[20], 120, 1);
   fprintf(output_fp, "QDOWN[20]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[20]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[20], 120, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[21], 121, 1);
   fprintf(output_fp, "QDOWN[21]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[21]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[21], 121, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[22], 122, 1);
   fprintf(output_fp, "QDOWN[22]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[22]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[22], 122, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[23], 123, 1);
   fprintf(output_fp, "QDOWN[23]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[23]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[23], 123, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[24], 124, 1);
   fprintf(output_fp, "QDOWN[24]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[24]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[24], 124, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[25], 125, 1);
   fprintf(output_fp, "QDOWN[25]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[25]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[25], 125, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[26], 126, 1);
   fprintf(output_fp, "QDOWN[26]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[26]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[26], 126, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[27], 127, 1);
   fprintf(output_fp, "QDOWN[27]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[27]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[27], 127, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[28], 128, 1);
   fprintf(output_fp, "QDOWN[28]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[28]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[28], 128, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[29], 129, 1);
   fprintf(output_fp, "QDOWN[29]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[29]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[29], 129, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[30], 130, 1);
   fprintf(output_fp, "QDOWN[30]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[30]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[30], 130, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[31], 131, 1);
   fprintf(output_fp, "QDOWN[31]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[31]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[31], 131, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[32], 132, 1);
   fprintf(output_fp, "QDOWN[32]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[32]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[32], 132, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[33], 133, 1);
   fprintf(output_fp, "QDOWN[33]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[33]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[33], 133, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[34], 134, 1);
   fprintf(output_fp, "QDOWN[34]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[34]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[34], 134, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[35], 135, 1);
   fprintf(output_fp, "QDOWN[35]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[35]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[35], 135, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[36], 136, 1);
   fprintf(output_fp, "QDOWN[36]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[36]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[36], 136, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[37], 137, 1);
   fprintf(output_fp, "QDOWN[37]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[37]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[37], 137, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[38], 138, 1);
   fprintf(output_fp, "QDOWN[38]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[38]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[38], 138, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[39], 139, 1);
   fprintf(output_fp, "QDOWN[39]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[39]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[39], 139, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[40], 140, 1);
   fprintf(output_fp, "QDOWN[40]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[40]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[40], 140, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[41], 141, 1);
   fprintf(output_fp, "QDOWN[41]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[41]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[41], 141, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[42], 142, 1);
   fprintf(output_fp, "QDOWN[42]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[42]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[42], 142, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[43], 143, 1);
   fprintf(output_fp, "QDOWN[43]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[43]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[43], 143, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[44], 144, 1);
   fprintf(output_fp, "QDOWN[44]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[44]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[44], 144, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[45], 145, 1);
   fprintf(output_fp, "QDOWN[45]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[45]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[45], 145, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[46], 146, 1);
   fprintf(output_fp, "QDOWN[46]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[46]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[46], 146, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[47], 147, 1);
   fprintf(output_fp, "QDOWN[47]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[47]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[47], 147, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[48], 148, 1);
   fprintf(output_fp, "QDOWN[48]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[48]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[48], 148, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[49], 149, 1);
   fprintf(output_fp, "QDOWN[49]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[49]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[49], 149, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[50], 150, 1);
   fprintf(output_fp, "QDOWN[50]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[50]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[50], 150, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[51], 151, 1);
   fprintf(output_fp, "QDOWN[51]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[51]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[51], 151, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[52], 152, 1);
   fprintf(output_fp, "QDOWN[52]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[52]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[52], 152, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[53], 153, 1);
   fprintf(output_fp, "QDOWN[53]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[53]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[53], 153, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[54], 154, 1);
   fprintf(output_fp, "QDOWN[54]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[54]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[54], 154, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[55], 155, 1);
   fprintf(output_fp, "QDOWN[55]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[55]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[55], 155, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[56], 156, 1);
   fprintf(output_fp, "QDOWN[56]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[56]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[56], 156, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[57], 157, 1);
   fprintf(output_fp, "QDOWN[57]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[57]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[57], 157, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[58], 158, 1);
   fprintf(output_fp, "QDOWN[58]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[58]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[58], 158, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[59], 159, 1);
   fprintf(output_fp, "QDOWN[59]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[59]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[59], 159, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[60], 160, 1);
   fprintf(output_fp, "QDOWN[60]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[60]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[60], 160, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[61], 161, 1);
   fprintf(output_fp, "QDOWN[61]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[61]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[61], 161, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[62], 162, 1);
   fprintf(output_fp, "QDOWN[62]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[62]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[62], 162, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[63], 163, 1);
   fprintf(output_fp, "QDOWN[63]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[63]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[63], 163, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[64], 164, 1);
   fprintf(output_fp, "QDOWN[64]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[64]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[64], 164, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[65], 165, 1);
   fprintf(output_fp, "QDOWN[65]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[65]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[65], 165, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[66], 166, 1);
   fprintf(output_fp, "QDOWN[66]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[66]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[66], 166, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[67], 167, 1);
   fprintf(output_fp, "QDOWN[67]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[67]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[67], 167, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[68], 168, 1);
   fprintf(output_fp, "QDOWN[68]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[68]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[68], 168, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[69], 169, 1);
   fprintf(output_fp, "QDOWN[69]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[69]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[69], 169, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[70], 170, 1);
   fprintf(output_fp, "QDOWN[70]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[70]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[70], 170, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[71], 171, 1);
   fprintf(output_fp, "QDOWN[71]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[71]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[71], 171, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  c_timest(QDOWN[72], 172, 1);
   fprintf(output_fp, "QDOWN[72]:\n Time Ave. = \t%7.4g Time Sample Var. =\t%7.4g\n", transfer[4], transfer[7]);
   printf("QDOWN[72]:\n Time Ave. = 	%7.4g Time Sample Var. = 	%7.4g\n", transfer[4], transfer[7]);
  c_sampst(QDOWN[72], 172, 1);
   fprintf(output_fp, " Event Ave. =\t%7.4g Event Sample Var. =\t%7.4g\n", transfer[4], transfer[8]);
   fprintf(output_fp, " Minimum =\t%7.4g\n", transfer[7]);
   fprintf(output_fp, " Maximum =\t%7.4g\n", transfer[6]);
   printf(" Event Ave.  = 	%7.4g Event Sample Var.= 	%7.4g\n", transfer[4], transfer[8]);
   printf(" Minimum  = 	%7.4g\n", transfer[7]);
   printf(" Maximum  = 	%7.4g\n", transfer[6]);
  printf("Output written to, %s\n",output_file_name);
  fclose(output_fp);
  c_closedisk();
  done = 1;
}



/****************************/
/*     EVENT FUNCTIONS      */
/****************************/

/*** THE SIMULATION RUN IS STARTED ***/
void
RUN()
{
int  _edge_condition[2];

  /* Attribute Value(s) Passed to this Event */

  /* state changes */
  IDLELEV=NELEV;

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( 1==1 );
  _edge_condition[1] = ( 1==1 );

  /* schedule future events */
  if (_edge_condition[0])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = 1;
    for ( t_index=4; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = INIELEV_event;
    event_priority = 5;
    schedule_event();
    }

  if (_edge_condition[1])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = 1;
    for ( t_index=4; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = INIFLOR_event;
    event_priority = 10;
    schedule_event();
    }

}


/*** PASSENGERS ENTER THE LINE ***/
void
ARRIVE()
{
int  _edge_condition[2];

  /* Attribute Value(s) Passed to this Event */
  F = (long) transfer[3];

  /* state changes */
  ROW=CLK/24;
  COL=MAX(F/NFLOORS*10,1);
  DEST=RND<DISK(DEST.DAT,COL);
  QUP[F]=QUP[F]+DEST;
  QDOWN[F]=QDOWN[F]+(1-DEST);

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( 1==1 );
  _edge_condition[1] = ( IDLELEV );

  /* schedule future events */
  if (_edge_condition[0])
    /***  THE NEXT PASSENGER ENTERS EVERY 0 TO 1 MINUTES  ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = F;
    for ( t_index=4; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + (24/DISK(NHPP.DAT,ROW*10+COL))*ERL(1);
    event_type = ARRIVE_event;
    event_priority = 6;
    schedule_event();
    }

  if (_edge_condition[1])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = F;
    transfer[4] = DEST;
    transfer[5] = 1;
    transfer[6] = 0;
    for ( t_index=7; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = CALL_event;
    event_priority = 5;
    schedule_event();
    }

}


/*** ELEVATOR STARTS ***/
void
START()
{
int  _edge_condition[1];

  /* Attribute Value(s) Passed to this Event */
  CALFLOOR = (long) transfer[3];
  UPDOWN = (long) transfer[4];
  TARGELEV = (long) transfer[5];

  /* state changes */
  ELEVPEOP[TARGELEV]=MIN(QUP[CALFLOOR]*UPDOWN+QDOWN[CALFLOOR]*(1-UPDOWN),ELEVCAP);
  QUP[CALFLOOR]=QUP[CALFLOOR]-UPDOWN*MIN(ELEVCAP,QUP[CALFLOOR]);
  QDOWN[CALFLOOR]=QDOWN[CALFLOOR]-(1-UPDOWN)*MIN(ELEVCAP,QDOWN[CALFLOOR]);

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( 1==1 );

  /* schedule future events */
  if (_edge_condition[0])
    /***  Elevator moves from its pickup to evaluate its next floor.   ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR+1*UPDOWN-1*(1-UPDOWN);
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    for ( t_index=6; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + DELAY+DELAY/5*(UPDOWN)*QUP[CALFLOOR+1]+DELAY/5*(1-UPDOWN)*QDOWN[CALFLOOR-1];
    event_type = STOP_event;
    event_priority = 5;
    schedule_event();
    }

}


/*** Finds an idling elevator to satisfy the call ***/
void
CALL()
{
int  _edge_condition[2];

  /* Attribute Value(s) Passed to this Event */
  CALFLOOR = (long) transfer[3];
  UPDOWN = (long) transfer[4];
  ID = (long) transfer[5];
  TARGELEV = (long) transfer[6];

  /* state changes */
  TARGELEV=(ELEVATOR[ID]==0)*ID;

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( (ID<=NFLOORS)&&(TARGELEV==0) );
  _edge_condition[1] = ( TARGELEV>0 );

  /* schedule future events */
  if (_edge_condition[0])
    /***  Iterate through elevators to find an idle one  ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR;
    transfer[4] = UPDOWN;
    transfer[5] = ID+1;
    transfer[6] = 0;
    for ( t_index=7; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = CALL_event;
    event_priority = 5;
    schedule_event();
    }

  if (_edge_condition[1])
    /***  The elevator is scheduled to go to the passenger who called it.   ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR;
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    for ( t_index=6; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = SEND_event;
    event_priority = 1;
    schedule_event();
    }

}


/*** Schedule to send the elevator from the idle point to the desired call location, setting the elevator as busy.  ***/
void
SEND()
{
int  _edge_condition[1];

  /* Attribute Value(s) Passed to this Event */
  CALFLOOR = (long) transfer[3];
  UPDOWN = (long) transfer[4];
  TARGELEV = (long) transfer[5];

  /* state changes */
  ELEVATOR[TARGELEV]=(-1);
  IDLELEV=IDLELEV-1;

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( 1==1 );

  /* schedule future events */
  if (_edge_condition[0])
    /***  Start the elevator run  ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR;
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    for ( t_index=6; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + DELAY*MAX(IDLEFLOR-CALFLOOR,CALFLOOR-IDLEFLOR);
    event_type = START_event;
    event_priority = 5;
    schedule_event();
    }

}


/*** Elevator stops at the next level with passengers scheduled to leave, dropping them off. It picks up any passengers headed in the same direction.  ***/
void
STOP()
{
int  _edge_condition[1];

  /* Attribute Value(s) Passed to this Event */
  CALFLOOR = (long) transfer[3];
  UPDOWN = (long) transfer[4];
  TARGELEV = (long) transfer[5];

  /* state changes */
  LEAVE=0;

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( 1==1 );

  /* schedule future events */
  if (_edge_condition[0])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR;
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    transfer[6] = 1;
    for ( t_index=7; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = DECIDE_event;
    event_priority = 5;
    schedule_event();
    }

}


/*** Set elevator to idle if it becomes empty ***/
void
IDLE()
{
int  _edge_condition[1];

  /* Attribute Value(s) Passed to this Event */
  TARGELEV = (long) transfer[3];

  /* state changes */
  ELEVATOR[TARGELEV]=0;
  IDLELEV=IDLELEV+1;

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( 1==1 );

  /* schedule future events */
  if (_edge_condition[0])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = IDLEFLOR;
    transfer[4] = 0;
    transfer[5] = TARGELEV;
    for ( t_index=6; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = CHECK_event;
    event_priority = 5;
    schedule_event();
    }

}


/*** Flip the elevator's orientation if it's on an edge floor ***/
void
EDGE()
{
int  _edge_condition[2];

  /* Attribute Value(s) Passed to this Event */
  CALFLOOR = (long) transfer[3];
  UPDOWN = (long) transfer[4];
  TARGELEV = (long) transfer[5];

  /* state changes */
  UPDOWN=1-UPDOWN;
  ELEVPEOP[TARGELEV]=MIN(QUP[CALFLOOR]*UPDOWN+QDOWN[CALFLOOR]*(1-UPDOWN),ELEVCAP);
  QUP[CALFLOOR]=QUP[CALFLOOR]-UPDOWN*MIN(ELEVCAP,QUP[CALFLOOR]);
  QDOWN[CALFLOOR]=QDOWN[CALFLOOR]-(1-UPDOWN)*MIN(ELEVCAP,QDOWN[CALFLOOR]);

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( ELEVPEOP[TARGELEV]>0 );
  _edge_condition[1] = ( ELEVPEOP[TARGELEV]==0 );

  /* schedule future events */
  if (_edge_condition[0])
    /***  If the elevator finds people at an edge, it returns to the adjacent floor to continue its path.  EDGE TO STOP  ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR;
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    for ( t_index=6; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = STOP_event;
    event_priority = 5;
    schedule_event();
    }

  if (_edge_condition[1])
    /***  If there is no one on an edge floor, the elevator returns to idle.   ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = TARGELEV;
    for ( t_index=4; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + DELAY*MAX(CALFLOOR-IDLEFLOR,IDLEFLOR-CALFLOOR);
    event_type = IDLE_event;
    event_priority = 5;
    schedule_event();
    }

}


/*** Elevator checks for waiting passengers ***/
void
CHECK()
{
int  _edge_condition[2];

  /* Attribute Value(s) Passed to this Event */
  CALFLOOR = (long) transfer[3];
  UPDOWN = (long) transfer[4];
  TARGELEV = (long) transfer[5];

  /* state changes */
  UPDOWN=QUP[CALFLOOR]>QDOWN[CALFLOOR];
  CHCKEDGE=(CALFLOOR==NFLOORS);

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( QUP[CALFLOOR]==0&&QDOWN[CALFLOOR]==0&&((CALFLOOR+1-CHCKEDGE*NFLOORS)==IDLEFLOR)==0 );
  _edge_condition[1] = ( (QUP[CALFLOOR]>0)&&(QDOWN[CALFLOOR]>0)==1 );

  /* schedule future events */
  if (_edge_condition[0])
    /***  Iterates through floors, bias is up at the moment  ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR+1-CHCKEDGE*NFLOORS;
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    for ( t_index=6; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = CHECK_event;
    event_priority = 5;
    schedule_event();
    }

  if (_edge_condition[1])
    /***  Start elevator system if there is a queue at the given level  ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR;
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    for ( t_index=6; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = SEND_event;
    event_priority = 5;
    schedule_event();
    }

}


/*** Initiate the elevator vector ***/
void
INIELEV()
{
int  _edge_condition[2];

  /* Attribute Value(s) Passed to this Event */
  ID = (long) transfer[3];

  /* state changes */
  ELEVATOR[ID]=0;

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( ID<NELEV );
  _edge_condition[1] = ( 1==1 );

  /* schedule future events */
  if (_edge_condition[0])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = ID+1;
    for ( t_index=4; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = INIELEV_event;
    event_priority = 5;
    schedule_event();
    }

  if (_edge_condition[1])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = 1;
    for ( t_index=4; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = INIQ_event;
    event_priority = 5;
    schedule_event();
    }

}


/*** All queues are initialized ***/
void
INIQ()
{
int  _edge_condition[1];

  /* Attribute Value(s) Passed to this Event */
  ID = (long) transfer[3];

  /* state changes */
  QUP[ID]=0;
  QDOWN[ID]=0;

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( ID<NFLOORS );

  /* schedule future events */
  if (_edge_condition[0])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = ID+1;
    for ( t_index=4; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = INIQ_event;
    event_priority = 5;
    schedule_event();
    }

}


/*** Initialize floor arrival nodes ***/
void
INIFLOR()
{
int  _edge_condition[2];

  /* Attribute Value(s) Passed to this Event */
  F = (long) transfer[3];

  /* state changes */
  COL=MAX(F/NFLOORS*10,1);

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( F<NFLOORS );
  _edge_condition[1] = ( 1==1 );

  /* schedule future events */
  if (_edge_condition[0])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = F+1;
    for ( t_index=4; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = INIFLOR_event;
    event_priority = 4;
    schedule_event();
    }

  if (_edge_condition[1])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = F;
    for ( t_index=4; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + (24/DISK(NHPP.DAT,COL))*ERL(1);
    event_type = ARRIVE_event;
    event_priority = 5;
    schedule_event();
    }

}


/***  ***/
void
DECIDE()
{
int  _edge_condition[2];

  /* Attribute Value(s) Passed to this Event */
  CALFLOOR = (long) transfer[3];
  UPDOWN = (long) transfer[4];
  TARGELEV = (long) transfer[5];
  NUM = (long) transfer[6];

  /* state changes */
  REMFLOOR=UPDOWN*(NFLOORS-CALFLOOR+1)+(1-UPDOWN)*(CALFLOOR);
  LEAVE=LEAVE+(RND<1/REMFLOOR);

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( NUM==ELEVPEOP[TARGELEV] );
  _edge_condition[1] = ( NUM<ELEVPEOP[TARGELEV] );

  /* schedule future events */
  if (_edge_condition[0])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR;
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    transfer[6] = LEAVE;
    for ( t_index=7; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = REMOVE_event;
    event_priority = 5;
    schedule_event();
    }

  if (_edge_condition[1])
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR;
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    transfer[6] = NUM+1;
    for ( t_index=7; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = DECIDE_event;
    event_priority = 5;
    schedule_event();
    }

}


/***  ***/
void
REMOVE()
{
int  _edge_condition[3];

  /* Attribute Value(s) Passed to this Event */
  CALFLOOR = (long) transfer[3];
  UPDOWN = (long) transfer[4];
  TARGELEV = (long) transfer[5];
  LEAVE = (long) transfer[6];

  /* state changes */
  ELEVPEOP[TARGELEV]=ELEVPEOP[TARGELEV]-LEAVE;
  OVERF=QUP[CALFLOOR]*UPDOWN+(1-UPDOWN)*QDOWN[CALFLOOR]+ELEVPEOP[TARGELEV]-ELEVCAP;
  ELEVPEOP[TARGELEV]=MIN(QUP[CALFLOOR]*UPDOWN+(1-UPDOWN)*QDOWN[CALFLOOR]+ELEVPEOP[TARGELEV],ELEVCAP);
  QUP[CALFLOOR]=QUP[CALFLOOR]-UPDOWN*(QUP[CALFLOOR]-MAX(OVERF,0));
  QDOWN[CALFLOOR]=QDOWN[CALFLOOR]-(1-UPDOWN)*(QDOWN[CALFLOOR]-MAX(OVERF,0));

  /* Evaluate edge conditions now so that they will*/
  /* not be changed by preemptive event execution  */
  _edge_condition[0] = ( (CALFLOOR>1)&&(CALFLOOR<NFLOORS)&&(ELEVPEOP[TARGELEV]>0) );
  _edge_condition[1] = ( 1==(CALFLOOR==NFLOORS)+(CALFLOOR==1) );
  _edge_condition[2] = ( (ELEVPEOP[TARGELEV]==0)&&CALFLOOR<NFLOORS&&CALFLOOR>1 );

  /* schedule future events */
  if (_edge_condition[0])
    /***  Elevator goes to the next floors, REMOVE to STOP  ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR+1*UPDOWN-1*(1-UPDOWN);
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    for ( t_index=6; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + DELAY+DELAY/5*(UPDOWN)*QUP[CALFLOOR+1]+DELAY/5*(1-UPDOWN)*QDOWN[CALFLOOR-1];
    event_type = STOP_event;
    event_priority = 5;
    schedule_event();
    }

  if (_edge_condition[1])
    /***  Special case for edge floors, REMOVE TO EDGE  ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = CALFLOOR;
    transfer[4] = UPDOWN;
    transfer[5] = TARGELEV;
    for ( t_index=6; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + 0;
    event_type = EDGE_event;
    event_priority = 5;
    schedule_event();
    }

  if (_edge_condition[2])
    /***  Set an elevator to idle if it becomes empty, REMOVE TO IDLE  ***/
    {
    /*** attribute value(s) to be transferred to event ***/
    transfer[3] = TARGELEV;
    for ( t_index=4; t_index<maxatr; t_index++) transfer[t_index] = 0.0;
    event_time = current_time + DELAY*MAX(CALFLOOR-IDLEFLOR,IDLEFLOR-CALFLOOR);
    event_type = IDLE_event;
    event_priority = 5;
    schedule_event();
    }

}

