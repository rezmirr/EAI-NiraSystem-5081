#include "preprocessing.h"
#include "string.h"

typedef float sample_t[RAW_CHANNELS];
static sample_t imu_buf[IMU_BUF_LEN],
								*wr_ptr = imu_buf;

#define BUF_END (&imu_buf[IMU_BUF_LEN])

static uint32_t total_samples	= 0U,
								since_last_window = 0U;
static uint8_t window_ready = 0;


void PP_PushSample(const float s[RAW_CHANNELS]){
	memcpy(*wr_ptr, s, sizeof(sample_t));
	wr_ptr++;
	if(wr_ptr == BUF_END) wr_ptr = imu_buf;
	
	if(total_samples < UINT32_MAX) total_samples++; 
	since_last_window++;
	
	if((total_samples >= DATA_WINDOW) && (since_last_window >= WINDOW_HOP)){
		since_last_window = 0;
		window_ready = 1;
	}
}

