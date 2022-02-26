#ifndef MAPPING_H
#define MAPPING_H

#include <iostream>
#include "mode.h"

extern const float PI;
extern const int RAW_WIDTH;
extern const int RAW_HEIGHT;
extern const float TRACK_WIDTH;
extern const float TRACK_HALF_WIDTH;
extern const float TRACK_ONE_QUARTER_WIDTH;
extern const float TRACK_THREE_QUARTER_WIDTH;
extern const float CAR_HALF_WIDTH;
extern const float CAR_REAL_X;
extern const float CAR_REAL_Y;
extern const float CAR_REAL_Y_FRONT;
extern const float CAR_REAL_Y_MIDDLE;
extern const float CAR_REAL_Y_REAR;

extern const int LCD_WIDTH;
extern const int LCD_HEIGHT;
extern const int REMAP_X_OFFSET;
extern const int REMAP_Y_OFFSET;
extern const float REMAP_RATIO;

#ifdef car_sim_v160
extern const float remapImg[120][160][2];
#endif

#ifndef car_sim_v160
extern const float remapImg[120][184][2];
#endif

#endif