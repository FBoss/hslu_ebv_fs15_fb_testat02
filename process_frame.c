/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*(OSC_CAM_MAX_IMAGE_WIDTH/2)*(OSC_CAM_MAX_IMAGE_HEIGHT/2)
#define GAUSS

const int nc = OSC_CAM_MAX_IMAGE_WIDTH / 2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT / 2;

int TextColor;

void CalcDeriv();
void AvgDeriv(int Index);
void Eckenmass();
void FindLocalMaxima();

//Variables
int Border = 6;
int avgDxy[3][IMG_SIZE];
int helpBuf[IMG_SIZE];
int k = 5;
int SizeBox = 2;

void ResetProcess() {
	//called when "reset" button is pressed
	if (TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
}

void ProcessFrame() {
	uint32 t1, t2;
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {
		//example for time measurement
		t1 = OscSupCycGet();
		//example for copying sensor image to background image
		//memcpy(data.u8TempImage[THRESHOLD], data.u8TempImage[SENSORIMG],IMG_SIZE);
		//example for time measurement
		CalcDeriv();
		AvgDeriv(0);
		AvgDeriv(1);
		AvgDeriv(2);
		Eckenmass();
		FindLocalMaxima();

		t2 = OscSupCycGet();

		//example for log output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2 - t1));

		//example for drawing output
		//draw line
		//DrawLine(10, 100, 200, 20, RED);
		//draw open rectangle
		//DrawBoundingBox(20, 10, 50, 40, false, GREEN);
		//draw filled rectangle
		//DrawBoundingBox(80, 100, 110, 120, true, BLUE);
		//DrawString(200, 200, strlen(Text), TINY, TextColor, Text);
	}
}
void CalcDeriv() {
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip the first and last line */
		for (c = 1; c < nc - 1; c++) {
			/* do pointer arithmetics with respect to center pixel location */
			unsigned char* p = &data.u8TempImage[SENSORIMG][r + c];
			/* implement Sobel filter */
			int dx = -(int) *(p - nc - 1) + (int) *(p - nc + 1)
					- 2 * (int) *(p - 1) + 2 * (int) *(p + 1)
					- (int) *(p + nc - 1) + (int) *(p + nc + 1);

			int dy = -(int) *(p - nc - 1) - 2 * (int) *(p - nc)
					- (int) *(p - nc + 1) + (int) *(p + nc - 1)
					+ 2 * (int) *(p + nc) + (int) *(p + nc + 1);

			avgDxy[0][r + c] = dx * dx;
			avgDxy[1][r + c] = dy * dy;
			avgDxy[2][r + c] = dx * dy;

			//	data.u8TempImage[BACKGROUND][r + c] = MAX(0,
			//			MIN(255, 128+(dx >> 2)));
			//	data.u8TempImage[THRESHOLD][r + c] = (uint8) MIN(255,
			//			MAX(0, (dx*dx) >> 10));
		}
	}
}

#ifndef GAUSS
#define GAUSS
void AvgDeriv(int Index) {
//do average in x-direction
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &avgDxy[Index][r + c];
			int sx = (*(p - 6) + *(p + 6)) * 1 + (*(p - 5) + *(p + 5)) * 4
			+ (*(p - 4) + *(p + 4)) * 11 + (*(p - 3) + *(p + 3)) * 27
			+ (*(p - 2) + *(p + 2)) * 50 + (*(p - 1) + *(p + 1)) * 72
			+ (*p) * 82;
//now averaged
			helpBuf[r + c] = (sx >> 8);
		}
	}
//do average in y-direction
	for (r = nc * (Border + 1); r < nr * nc - nc * (Border - 1); r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &helpBuf[r + c];
			int sy = (*(p - 6 * nc) + *(p + 6 * nc)) * 1
			+ (*(p - 5 * nc) + *(p + 5 * nc)) * 4
			+ (*(p - 4 * nc) + *(p + 4 * nc)) * 11
			+ (*(p - 3 * nc) + *(p + 3 * nc)) * 27
			+ (*(p - 2 * nc) + *(p + 2 * nc)) * 50
			+ (*(p - 1 * nc) + *(p + 1 * nc)) * 72 + (*p) * 82;
//now averaged
			avgDxy[Index][r + c] = (sy >> 8);
			//data.u8TempImage[BACKGROUND][r+c] = MAX(0, MIN(255, (avgDxy[Index][r+c] >> 11)));
		}
	}
}
#else
void AvgDeriv(int Index) {
//do average in x-direction
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &avgDxy[Index][r + c];
			int sx = *(p - 6) + *(p + 6) + ((*(p - 5) + *(p + 5)) << 2)
					+ ((*(p - 4) + *(p + 4)) << 3)
					+ ((*(p - 3) + *(p + 3)) << 5)
					+ ((*(p - 2) + *(p + 2)) << 6)
					+ ((*(p - 1) + *(p + 1)) << 6) + ((*p) << 7);
//now averaged
			helpBuf[r + c] = (sx >> 8);
		}
	}
//do average in y-direction
	for (r = nc * (Border + 1); r < nr * nc - nc * (Border - 1); r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &helpBuf[r + c];
			int sy = (*(p - 6 * nc) + *(p + 6 * nc))
					+ ((*(p - 5 * nc) + *(p + 5 * nc)) << 2)
					+ ((*(p - 4 * nc) + *(p + 4 * nc)) << 3)
					+ ((*(p - 3 * nc) + *(p + 3 * nc)) << 5)
					+ ((*(p - 2 * nc) + *(p + 2 * nc)) << 6)
					+ ((*(p - 1 * nc) + *(p + 1 * nc)) << 6) + ((*p) << 7);
//now averaged
			avgDxy[Index][r + c] = (sy >> 8);
			//data.u8TempImage[BACKGROUND][r + c] = MAX(0,
			//		MIN(255, (avgDxy[Index][r+c] >> 11)));
		}
	}
}
#endif

void Eckenmass() {
	int c, r;
	for (r = nc * (Border + 1); r < nr * nc - nc * (Border + 1); r += nc) {
		for (c = Border + 1; c < nc - (Border + 1); c++) {
			int dIx2 = (avgDxy[0][r + c] >> 7);
			int dIy2 = (avgDxy[1][r + c] >> 7);
			int dIxy = (avgDxy[2][r + c] >> 7);
			avgDxy[0][r + c] = (dIx2 * dIy2 - dIxy * dIxy)
					- ((k * (dIx2 + dIy2) * (dIx2 + dIy2)) >> 7);
			data.u8TempImage[BACKGROUND][r + c] = MAX(0,
					MIN(255, (avgDxy[0][r+c]>>7)));
		}
	}
}

void FindLocalMaxima() {
	int c, r;

	//Find Global Max
	int globalMax = 0;
	for (int i = 0; i < IMG_SIZE; i++) {
		globalMax = MAX(globalMax, avgDxy[0][i]);
	}

	//Calculate Threshold
	int threshold = (int) (globalMax * data.ipc.state.nThreshold / 100);

	for (r = nc * (Border + 1); r < nr * nc - nc * (Border - 1); r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			if (threshold < avgDxy[0][r + c]) {

				int* p = &avgDxy[0][r + c];

				//X Direction
				int MaxX =
						MAX(
								MAX(MAX(MAX(MAX(MAX((*(p - 6)),(*(p + 6))), MAX((*(p - 5)),(*(p + 5)))), MAX((*(p - 4)),(*(p + 4)))), MAX((*(p - 3)),(*(p + 3)))), MAX((*(p - 2)),(*(p + 2)))),
								MAX((*(p - 1)),(*(p + 1))));
				//Y Direction
				int MaxY =
						MAX(
								MAX(MAX(MAX(MAX(MAX((*(p - 6*nc)),(*(p + 6*nc))), MAX((*(p - 5*nc)),(*(p + 5*nc)))), MAX((*(p - 4*nc)),(*(p + 4*nc)))), MAX((*(p - 3*nc)),(*(p + 3*nc)))), MAX((*(p - 2*nc)),(*(p + 2*nc)))),
								MAX((*(p - 1*nc)),(*(p + 1*nc))));

				//Validate Value
				if ((*p) >= MaxX && (*p) >= MaxY) {
					DrawBoundingBox(c - SizeBox, (r / nc) + SizeBox,
							c + SizeBox, (r / nc) - SizeBox, false, GREEN);
				}
			}
		}
	}
}
