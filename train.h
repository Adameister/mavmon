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

#ifndef __TRAIN_H__
#define __TRAIN_H__

#include <ctype.h>
#include <limits.h>

/*
 *
 *
 * DO NOT MODIFY ANY OF THE CODE BELOW
 *
 *
 */

#define SECONDS_IN_A_DAY 86400
#define MAX_TRAIN_EVENTS SECONDS_IN_A_DAY / 10

#define INTERSECTION_EMPTY -1

enum TRAIN_DIRECTION
{
  UNKNOWN = 0,
  NORTH   = 1,
  EAST    = 2,
  SOUTH   = 3,
  WEST    = 4,
  NUM_DIRECTIONS = 5
};

char * directionAsString[ NUM_DIRECTIONS ] =
{
  "Unknown",
  "North",
  "East",
  "South",
  "West"
};

struct ScheduleEntry
{
  uint32_t arrival_time;
  uint32_t train_id;
  uint32_t train_direction;
};

typedef struct ScheduleEntry ScheduleEntry;

static ScheduleEntry schedule[ MAX_TRAIN_EVENTS ];
static uint32_t      schedule_front = 0;
static uint32_t      schedule_back  = 0;

void scheduleInit( )
{
  int i = 0;
  for( i = 0; i < MAX_TRAIN_EVENTS; ++i )
  {
    schedule[ i ] . arrival_time    = INT_MAX;
    schedule[ i ] . train_id        = -1;
    schedule[ i ] . train_direction = UNKNOWN;
  }
}

void schedulePush( ScheduleEntry newEntry )
{
  schedule[ schedule_back ] . arrival_time    = newEntry . arrival_time;
  schedule[ schedule_back ] . train_id        = newEntry . train_id;
  schedule[ schedule_back ] . train_direction = newEntry . train_direction;

  schedule_back ++;
}

int scheduleEmpty( )
{
  return schedule_front >= schedule_back;
}

ScheduleEntry scheduleFront( )
{
  return schedule[ schedule_front ];
}

void schedulePop( )
{
  if( scheduleEmpty( ) ) return;

  schedule_front ++;
}

void buildTrainSchedule( char * filename )
{
  // Check that the file exists.  If not then print an error
  // and exit the program.

  struct stat statbuf;
  int retval = stat( filename, &statbuf );

  if( retval == -1 )
  {
    perror("Can't not open train schedule data file:");
    exit( EXIT_FAILURE );
  }

  // open the data file and read the train schedule
  FILE * fp = fopen( filename, "r" );

  if( fp == NULL )
  {
    perror("Can't not open train schedule data file:");
    exit( EXIT_FAILURE );
  }

  uint32_t             arrival_time;
  uint32_t             train_id;
  enum TRAIN_DIRECTION direction;
  char                 destination_direction;

  // Read the data file and populate the train schedule.
  while( fscanf( fp, "%d %d %c", &arrival_time, &train_id, &destination_direction ) != EOF )
  {
    ScheduleEntry val;
    enum TRAIN_DIRECTION direction;

    switch ( tolower( destination_direction )  )
    {
      case 'n'  : direction = NORTH;
                  break;

      case 'e'  : direction = EAST;
                  break;

      case 's'  : direction = SOUTH;
                  break;

      case 'w'  : direction = WEST;
                  break;

      default   : direction = UNKNOWN;
                  break;
    };

    val.arrival_time     = arrival_time;
    val.train_id         = train_id;
    val.train_direction  = direction;

    schedulePush( val );

#ifdef DEBUG
    fprintf(stdout, "Scheduled: Time: %d Train: %d Direction: %d\n", arrival_time, train_id, direction );
#endif
  }

  // Verify there was relevant scheduling data in the file, otherwise exit out
  if( scheduleEmpty( ) )
  {
    fprintf( stderr, "Error: Could not parse any schedule data from %s.\n", filename );
    exit( EXIT_FAILURE );
  }


  // Done with the schedule file so close it
  fclose( fp );
}

#endif

