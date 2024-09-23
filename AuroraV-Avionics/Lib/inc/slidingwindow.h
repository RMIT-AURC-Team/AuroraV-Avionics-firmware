#ifndef _SLIDINGWINDOW_H
#define _SLIDINGWINDOW_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct SlidingWindow {
  float *buff;
  int index;
  int count;
  int buffSize;
  void (*append)(struct SlidingWindow *, float);
  void (*calculateMovingAverage)(struct SlidingWindow *, float *);
} SlidingWindow;

void SlidingWindow_init(SlidingWindow *, float *, int);
void SlidingWindow_append(SlidingWindow *, float);
void SlidingWindow_calculateMovingAverage(SlidingWindow *, float *);

#endif
