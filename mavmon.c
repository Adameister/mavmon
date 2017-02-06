// The MIT License (MIT)
// 
// Copyright (c) 2016 Trevor Bakker 
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/*
 * Name: Adam Hudson
 * ID #: 1000991758
 * Programming Assignment 2
 * Description: Create mavmon.c to reflect conditions found on MAV.pdf on BlackBoard
 */

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#include "train.h"

// Declaration of mutex locker
// Locks in trainCross() and unLocks in trainLeaves()
// Initialized in init()

// for the intersection
pthread_mutex_t mutex;
// for the creation of threads
pthread_mutex_t createLock;
// used to send signals in mediate()
pthread_cond_t priority[ NUM_DIRECTIONS ];
pthread_mutex_t priorityMutex[ NUM_DIRECTIONS ];

void trainArrives( uint32_t train_id, enum TRAIN_DIRECTION train_direction );
void trainCross  ( uint32_t train_id, enum TRAIN_DIRECTION train_direction );
void trainLeaves ( uint32_t train_id, enum TRAIN_DIRECTION train_direction );

// Current time of day in seconds since midnight
int32_t  current_time;
uint32_t clock_tick;

// Current train in the intersection
uint32_t in_intersection;

// Global varables that keep track of train starvation
int north, east, south, west;
int starvation[NUM_DIRECTIONS];
int previousDirection;

// Global array for trains waiting to enter intersection
int inQueue[NUM_DIRECTIONS];

// Struct for the MAVs (threads)
struct train_struct{
	int id;
	int direction;
};
struct train_struct *ts;

/*
 *	trainLogic() will lock the wait condition for the train's direction and
 *	unlock it immediately after calling it.
 *	trainlogic() will then lock and send the thread to trainCross()
 */
void * trainLogic( void * val ){
	struct train_struct *ts = (struct train_struct*)val;

	// locking the wait condition per direction
	pthread_mutex_lock ( &priorityMutex[ ts->direction ] );
	pthread_cond_wait( &priority[ ts->direction ], &priorityMutex[ ts->direction ]);
	pthread_mutex_unlock( &priorityMutex[ ts->direction ]);
	
	//fprintf(stderr, "Trying intersection lock %d\n", (int)pthread_self());
	
	// lock the intersection
	pthread_mutex_lock( &mutex );
	
	//fprintf(stderr, "Got the intersection lock %d\n", (int)pthread_self());
	
	trainCross( ts->id, ts->direction );

	// added in the free and flush
	// was causing unknown issues
	free( ts );
	fflush( NULL );

}

/*
 *	trainLeavs() will print out which train is leaving,
 *	then the function will empty the intersection.
 *	The inQueue array will get decremented because that thread has logiclly left.
 *	previousDirection gets updated here to keep track of starvation.
 *	The mutex will then get unlocked and the thread will exit.
 */
void trainLeaves( uint32_t train_id, enum TRAIN_DIRECTION train_direction ){
	fprintf( stdout, "Current time: %d MAV %d heading %s leaving the intersection\n", 
		current_time, train_id, directionAsString[ train_direction ] );

	in_intersection = INTERSECTION_EMPTY;

	// TODO: Handle any cleanup
	inQueue[ train_direction ] -= 1;

	// Keep track of who went through the intersection last
	// to prevent starvation
	previousDirection = train_direction;

	pthread_mutex_unlock( &mutex );

	// sleep to keep thread logic consistant
	sleep(1);

	// clean up
	pthread_exit(NULL);
}

/*
 *	trainCross() will print the thread that is heading into the intersection
 *	it will then add a thread to the intersection and check for starvation.
 *	Starvatoin will check if the current direction is the same as the last direction.
 *	If it is, then the direction will be incremented. if it is not, then the direction
 *	will be zeroed out, meaning nobody starving.
 *	trainCross() will then call trainLeaves()
 */
void trainCross( uint32_t train_id, enum TRAIN_DIRECTION train_direction ){
	// TODO: Handle any crossing logic

	fprintf( stdout, "Current time: %d MAV %d heading %s entering the intersection\n", 
		current_time, train_id, directionAsString[ train_direction ] );

	// Lockup the intersection
	//pthread_mutex_lock( &mutex );
	if( in_intersection == INTERSECTION_EMPTY ){
		in_intersection = train_id;	
		// starvation logic update
		if( previousDirection == train_direction ){
			starvation[ train_direction ] += 1;
		}
		else{
			// if the direction is diffrent from the last, then there is no starvation
			// the direction will get reset
			starvation[ previousDirection ] = 0;
		}
	}
	else{
	fprintf( stderr, "CRASH: Train %d collided with train %d\n",
    	train_id, in_intersection );
	exit( EXIT_FAILURE );
	}

	// Sleep for 10 microseconds to simulate crossing
	// the intersection
	usleep( 10 * 1000000 / clock_tick );

	// Leave the intersection
	trainLeaves( train_id, train_direction );

	return;
}

/*
 *	trainArrives() will first print out the thread heading into the intersection.
 *	The program will lock the creation of the thread so that nothing will interfear.
 *	The inQueue will get updated with who is waiting in the intersection.
 *	The thread will run trainLogic
 *	program unlocks the creation of the threads
 */
void trainArrives( uint32_t train_id, enum TRAIN_DIRECTION train_direction ){
	fprintf( stdout, "Current time: %d MAV %d heading %s arrived at the intersection\n", 
		current_time, train_id, directionAsString[ train_direction ] );

	// Thread initialization:
	pthread_mutex_lock( &createLock );
	pthread_t tid;

	// Update who is waiting in the intersection
	inQueue[ train_direction ] += 1;

	ts = (struct train_struct*) malloc( sizeof( struct train_struct ) );
	ts->id = train_id;
	ts->direction = train_direction;

	pthread_create(&tid, NULL, trainLogic, ts);
	pthread_mutex_unlock( &createLock );

	// TODO: Handle the intersection logic

	return;
}

/*
 *	Called once a second for train routing
 *	mediate will first check for starvation
 *	then for single train in the intersection
 *	then for trains from two diffrent directions
 *	then for trains comming in from opposite directions
 *	then for trains coming in from three directions
 *	then for trains coming in from all directions
 */
void mediate( ){

	// is needed?
	if( in_intersection != INTERSECTION_EMPTY ) return ;

	if( starvation[NORTH] == 4 ){
		// Signal East to go
		//printf("\n STARVED TRIGGERED: going east\n\n");
		pthread_cond_signal( &priority[EAST] );
		return;
	}
	if( starvation[EAST] == 4 ){
		// Signal South
		//printf("\n STARVED TRIGGERED: going south\n\n");
		pthread_cond_signal( &priority[SOUTH] );
		return;
	}
	if( starvation[SOUTH] == 4 ){
		// Signal West
		//printf("\n STARVED TRIGGERED: going west\n\n");
		pthread_cond_signal( &priority[WEST] );
		return;
	}
	if( starvation[WEST] == 4 ){
		// Signal North
		//printf("\n STARVED TRIGGERED: going north\n\n");
		pthread_cond_signal( &priority[NORTH] );
		return;
	}

	////////////////////////////////////////////////////////////////////////////
	//
	//	Single train in the intersection
	//
	////////////////////////////////////////////////////////////////////////////
	if( inQueue[NORTH] && !inQueue[EAST] && !inQueue[SOUTH] && !inQueue[WEST] ){
		pthread_cond_signal( &priority[NORTH] );
		return;
	}
	if( !inQueue[NORTH] && inQueue[EAST] && !inQueue[SOUTH] && !inQueue[WEST] ){
		pthread_cond_signal( &priority[EAST] );
		return;
	}
	if( !inQueue[NORTH] && !inQueue[EAST] && inQueue[SOUTH] && !inQueue[WEST] ){
		pthread_cond_signal( &priority[SOUTH] );
		return;
	}
	if( !inQueue[NORTH] && !inQueue[EAST] && !inQueue[SOUTH] && inQueue[WEST] ){
		pthread_cond_signal( &priority[WEST] );
		return;
	}

	////////////////////////////////////////////////////////////////////////////
	//
	//	Right angle in the intersection
	//
	////////////////////////////////////////////////////////////////////////////
	if( inQueue[NORTH] && !inQueue[EAST] && !inQueue[SOUTH] && inQueue[WEST] ){
		pthread_cond_signal( &priority[WEST] );
		return;
	}
	if( !inQueue[NORTH] && !inQueue[EAST] && inQueue[SOUTH] && inQueue[WEST] ){
		pthread_cond_signal( &priority[SOUTH] );
		return;
	}
	if( !inQueue[NORTH] && inQueue[EAST] && inQueue[SOUTH] && !inQueue[WEST] ){
		pthread_cond_signal( &priority[EAST] );
		return;
	}
	if( inQueue[NORTH] && inQueue[EAST] && !inQueue[SOUTH] && !inQueue[WEST] ){
		//fprintf(stderr, "Signalling NORTH\n");
		//fflush(NULL);
		pthread_cond_signal( &priority[NORTH] );
		return;
	}

	////////////////////////////////////////////////////////////////////////////
	//
	//	Coming from the opposite direction
	//
	////////////////////////////////////////////////////////////////////////////
	if( inQueue[NORTH] && !inQueue[EAST] && inQueue[SOUTH] && !inQueue[WEST] ){
		pthread_cond_signal( &priority[NORTH] );
		return;
	}
	if( !inQueue[NORTH] && inQueue[EAST] && !inQueue[SOUTH] && inQueue[WEST] ){
		pthread_cond_signal( &priority[EAST] );
		return;
	}

	////////////////////////////////////////////////////////////////////////////
	//
	//	Three trains in the intersection
	//
	////////////////////////////////////////////////////////////////////////////
	if( inQueue[NORTH] && inQueue[EAST] && inQueue[SOUTH] && !inQueue[WEST] ){
		pthread_cond_signal( &priority[NORTH] );
		return;
	}
	if( inQueue[NORTH] && inQueue[EAST] && !inQueue[SOUTH] && inQueue[WEST] ){
		pthread_cond_signal( &priority[WEST] );
		return;
	}
	if( inQueue[NORTH] && !inQueue[EAST] && inQueue[SOUTH] && inQueue[WEST] ){
		pthread_cond_signal( &priority[SOUTH] );
		return;
	}
	if( !inQueue[NORTH] && inQueue[EAST] && inQueue[SOUTH] && inQueue[WEST] ){
		pthread_cond_signal( &priority[EAST] );
		return;
	}

	////////////////////////////////////////////////////////////////////////////
	//
	//	Deadlock case (1 1 1 1)
	//
	////////////////////////////////////////////////////////////////////////////
	if( inQueue[NORTH] && inQueue[EAST] && inQueue[SOUTH] && inQueue[WEST] ){
		pthread_cond_signal( &priority[NORTH] );
		return;
	}	

}

/*
 *	Contains the initialization of:
 *		Starvation directions
 *		Starvation array
 *		previousDirection (used for starvation)
 *		inQueue array (who is waiting)
 *		pthread_mutexes
 *			intersection
 *			thread creation
 *			condition handel
 */
void init( ){

	int i = 0;

	// this locks from the call of trainLogic() to trainLeaves()
	pthread_mutex_init( &mutex, NULL);
	// this locks the creation of the threads
	pthread_mutex_init( &createLock, NULL );

	for( i = 0; i < NUM_DIRECTIONS; i++ ){
		// for the signal of who to cross in the intersection
		pthread_cond_init( &priority[i], NULL );
		pthread_mutex_init( &priorityMutex[i], NULL);
		starvation[i] = 0;
		inQueue[i] = 0;
	}
	previousDirection = 0;
	north = 0;
	east = 0;
	south = 0;
	west = 0;
}


/*
 *
 *
 *  DO NOT MODIFY CODE BELOW THIS LINE
 *
 *
 *
 */

int process( )
{
  // If there are no more scheduled train arrivals
  // then return and exit
  if( scheduleEmpty() ) return 0;

  // If we're done with a day's worth of schedule then
  // we're done.
  if( current_time > SECONDS_IN_A_DAY ) return 0;

  // Check for deadlocks
  mediate( );

  // While we still have scheduled train arrivals and it's time
  // to handle an event
  while( !scheduleEmpty() && current_time >= scheduleFront().arrival_time ) 
  {

#ifdef DEBUG
    fprintf( stdout, "Dispatching schedule event: time: %d train: %d direction: %s\n",
                      scheduleFront().arrival_time, scheduleFront().train_id, 
                      directionAsString[ scheduleFront().train_direction ] );
#endif  

    trainArrives( scheduleFront().train_id, scheduleFront().train_direction );

    // Remove the event from the schedule since it's done
    schedulePop( );

  }

  // Sleep for a simulated second. Depending on clock_tick this
  // may equate to 1 real world second down to 1 microsecond
  usleep( 1 * 1000000 / clock_tick );

  current_time ++;

  return 1;
}

int main ( int argc, char * argv[] )
{

  // Initialize time of day to be midnight
  current_time = 0;
  clock_tick   = 1;

  // Verify the user provided a data file name.  If not then
  // print an error and exit the program
  if( argc < 2 )
  {
    fprintf( stderr, "ERROR: You must provide a train schedule data file.\n");
    exit(EXIT_FAILURE);
  }

  // See if there's a second parameter which specifies the clock
  // tick rate.  
  if( argc == 3 )
  {
    int32_t tick = atoi( argv[2] );
    
    if( tick <= 0 )
    {
      fprintf( stderr, "ERROR: tick rate must be positive.\n");
      exit(EXIT_FAILURE);
    }
    else
    {
      clock_tick = tick;
    }
  }

  buildTrainSchedule( argv[1] );

  // Initialize the intersection to be empty
  in_intersection = INTERSECTION_EMPTY;

  // Call user specific initialization
  init( );

  // Start running the MAV manager
  while( process() );
 
  return 0;
}
