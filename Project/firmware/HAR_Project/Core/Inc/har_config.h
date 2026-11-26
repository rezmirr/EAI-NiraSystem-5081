#ifndef HAR_CONFIG_H
#define HAR_CONFIG_H

//#define DEBUG_PRINT_RAW

#define RAW_CHANNELS				6U
#define DATA_WINDOW					100U
#define WINDOW_HOP					50U

#define IMU_BUF_LEN 				256U

#if IMU_BUF_LEN < (DATA_WINDOW + WINDOW_HOP)
	#error "IMU_BUF_LEN too small"
#endif

#endif

