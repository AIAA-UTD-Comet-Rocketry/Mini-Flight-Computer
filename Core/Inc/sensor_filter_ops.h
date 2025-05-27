#ifndef SENSOR_FILTER_OPS_H
#define SENSOR_FILTER_OPS_H

#include "main.h"
#include "motion_gc.h"
#include "motion_ac.h"
#include "motion_fx.h"

#define SAMPLE_FREQ	        100.0f //100Hz
#define GBIAS_GYRO_TH_SC    (0.050f)
#define GBIAS_ACC_TH_SC     (0.008f)
#define STATE_SIZE          2450
#define GYRO_FILTER_SIZE 512

extern uint32_t cal_data[MAC_ACC_CAL_DATA_SIZE / sizeof(uint32_t)];

void GetAltitude(dataframe_t *);

void MotionGC_Init(void);
int CalibrateGyroData(LSM6DSR_Object_t *);
void ApplyGyroCalibration(LSM6DSR_Axes_t *, LSM6DSR_Axes_t *);
void GetGyroRollingAverage(LSM6DSR_Axes_t, uint32_t);

void MotionAC_Init(void);
uint8_t CalibrateAccData(LSM6DSR_Object_t *);
void ApplyAccCalibration(LSM6DSR_Axes_t *, float *);

void MotionFX_Init(void);
void ProcessSensorData(dataframe_t *);

#endif // SENSOR_FILTER_OPS_H
