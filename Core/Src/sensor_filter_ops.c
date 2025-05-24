
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "sensor_filter_ops.h"

extern uint8_t MotionAC_SaveCalInNVM(uint16_t, uint32_t *);

/* MotionGC/AC variables */
uint32_t cal_data[MAC_ACC_CAL_DATA_SIZE / sizeof(uint32_t)];
MGC_mcu_type_t mcu_type = MGC_MCU_STM32;
MGC_knobs_t gc_knobs;
MGC_output_t gc_data_out;
MAC_knobs_t ac_knobs;
MAC_output_t ac_data_out;

/* MotionFX */
uint8_t mfx_state[STATE_SIZE]; // MotionFX engine state
MFX_input_t motion_input;
MFX_output_t motion_output;

const float SEALEVELPRESSURE_HPA = 1013.25f; //Change it according to location

/*Calculate Altitude from Pressure and Temperature Data*/
void GetAltitude(dataframe_t *nextFrame) {
    const float R = 287.05f;   // Specific gas constant for dry air (J/(kg·K))
    const float g = 9.80665f;  // Gravity (m/s²)
    float new_altitude = 0;
    float T = nextFrame->currTemp;
    float P = nextFrame->currPress;

    // Convert temperature to Kelvin
    T += 273.15f;

    // Calculate altitude using the hypsometric equation
    new_altitude = (R * T / g) * log(SEALEVELPRESSURE_HPA / P);

    //meter to feet
    new_altitude *= 3.28f;

    nextFrame->altitude = new_altitude;

    // For debugging
    static uint32_t alcounter = 0;
    if (alcounter++ % 1000 == 0) {
    	char msg[100];
		int len = sprintf(msg, "Alt: %.2f ft\n", nextFrame->altitude);
		_write(0, msg, len);
    }
}


/* Initialize the MotionGC library */
void MotionGC_Init(void) {
	float sample_freq = SAMPLE_FREQ;
	MotionGC_Initialize(mcu_type, &sample_freq); // Initialize with sample frequency
	MotionGC_GetKnobs(&gc_knobs); // Get default knobs

	// Configure for your sensor
	gc_knobs.AccThr = 0.008f;    // 8 mg threshold for stillness detection
	gc_knobs.GyroThr = 0.05f;    // 50 mdps threshold

	MotionGC_SetKnobs(&gc_knobs);
}


/* Calibrate gyroscope sensor (LSM6DSR) */
int CalibrateGyroData(LSM6DSR_Object_t *hlsm6d) {
  MGC_input_t data_in;
  LSM6DSR_Axes_t gyro_data;
  LSM6DSR_Axes_t acc_data;
  int bias_updated;

  LSM6DSR_ACC_GetAxes(hlsm6d, &acc_data);
  LSM6DSR_GYRO_GetAxes(hlsm6d, &gyro_data); // Get angular rate from LSM6DSR

  // data cast and copy - mdps -> dps & mg -> g
  data_in.Acc[0]  = ((float)acc_data.x)/1000.0f;
  data_in.Acc[1]  = ((float)acc_data.y)/1000.0f;
  data_in.Acc[2]  = ((float)acc_data.z)/1000.0f;
  data_in.Gyro[0] = ((float)gyro_data.x)/1000.0f;
  data_in.Gyro[1] = ((float)gyro_data.y)/1000.0f;
  data_in.Gyro[2] = ((float)gyro_data.z)/1000.0f;

  // Calculate the compensated gyroscope data
  MotionGC_Update(&data_in, &gc_data_out, &bias_updated);

  if (bias_updated) {
	  printf("Gyro calibration done!\n");
	  printf("Gyro offset:- X: %d Y: %d Z: %d\n", (int)(gc_data_out.GyroBiasX*1000), (int)(gc_data_out.GyroBiasY*1000), (int)(gc_data_out.GyroBiasZ*1000));
  }

  return bias_updated;
}


/* Main loop for filtering  */
void ApplyGyroCalibration(LSM6DSR_Axes_t *currGyro, LSM6DSR_Axes_t *currAcc) {
  MGC_input_t data_in;
  int bias_updated;
  static uint32_t gccounter = 0;
  uint32_t len;
  char msg[100] = {};

  if(!(gccounter++%1000)){
    //printf("Gyro before Cal mdps:- X: %d Y: %d Z: %d\n", (int)currGyro->x, (int)currGyro->y, (int)currGyro->z);
    //printf("Gyro before Cal mdps:- X: Y: Z: \n");
    len = sprintf(msg, "Gyro B mdps:- X: %d Y: %d Z: %d\n", (int)currGyro->x, (int)currGyro->y, (int)currGyro->z);
    _write(0, msg, len);
  }

  // data cast and copy - mdps -> dps & mg -> g
  data_in.Acc[0]  = ((float)currAcc->x)/1000.0f;
  data_in.Acc[1]  = ((float)currAcc->y)/1000.0f;
  data_in.Acc[2]  = ((float)currAcc->z)/1000.0f;
  data_in.Gyro[0] = ((float)currGyro->x)/1000.0f;
  data_in.Gyro[1] = ((float)currGyro->y)/1000.0f;
  data_in.Gyro[2] = ((float)currGyro->z)/1000.0f;

  // Calculate the compensated gyroscope data
  MotionGC_Update(&data_in, &gc_data_out, &bias_updated);

  currGyro->x = currGyro->x - (int32_t)(gc_data_out.GyroBiasX*1000);
  currGyro->y = currGyro->y - (int32_t)(gc_data_out.GyroBiasY*1000);
  currGyro->z = currGyro->z - (int32_t)(gc_data_out.GyroBiasZ*1000);

  if(!(gccounter%1000)) {
    //printf("Gyro after Cal mdps:- X: %d Y: %d Z: %d\n\n", (int)currGyro->x, (int)currGyro->y, (int)currGyro->z);
    //printf("Gyro after Cal mdps:- X: Y: Z:\n\n");
    len = sprintf(msg, "Gyro A mdps:- X: %d Y: %d Z: %d\n", (int)currGyro->x, (int)currGyro->y, (int)currGyro->z);
    _write(0, msg, len);
  }
  GetGyroRollingAverage(*currGyro, gccounter);
}


void GetGyroRollingAverage(LSM6DSR_Axes_t avgGyro, uint32_t gccounter) {
	static LSM6DSR_Axes_t gyroBuffer[GYRO_FILTER_SIZE];
	static uint32_t gyroIndex = 0;
	static uint8_t gyroBufferFilled = 0;
	static int32_t sumX = 0, sumY = 0, sumZ = 0;

	// Remove oldest value from sum
	sumX -= gyroBuffer[gyroIndex].x;
	sumY -= gyroBuffer[gyroIndex].y;
	sumZ -= gyroBuffer[gyroIndex].z;

	// Store new calibrated value in buffer
	gyroBuffer[gyroIndex].x = avgGyro.x;
	gyroBuffer[gyroIndex].y = avgGyro.y;
	gyroBuffer[gyroIndex].z = avgGyro.z;

	// Add new value to sum
	sumX += avgGyro.x;
	sumY += avgGyro.y;
	sumZ += avgGyro.z;

	// Advance buffer index
	gyroIndex = (gyroIndex + 1) % GYRO_FILTER_SIZE;
	if (gyroIndex == 0) {
	    gyroBufferFilled = 1;
	}

	// Compute average
	uint32_t count = gyroBufferFilled ? GYRO_FILTER_SIZE : gyroIndex;
	avgGyro.x = (int32_t)(sumX / count);
	avgGyro.y = (int32_t)(sumY / count);
	avgGyro.z = (int32_t)(sumZ / count);

	uint32_t len = 0;
	char msg[100] = {};
	if(!(gccounter%1000)) {
		len = sprintf(msg, "Gyro Avg mdps:- X: %d Y: %d Z: %d\n", (int)avgGyro.x, (int)avgGyro.y, (int)avgGyro.z);
		_write(0, msg, len);
	}
}


/* Initialize the MotionAC library */
void MotionAC_Init(void) {
	MotionAC_Initialize(true); // Initialize with sample frequency
	MotionAC_GetKnobs(&ac_knobs);

	//ac_knobs.Sample_ms = 1;
	// Configure for 6-point calibration
	ac_knobs.Run6PointCal = 1;       // Enable 6-point calibration
	ac_knobs.Sample_ms = 10;         // Match your 100Hz sensor ODR
	ac_knobs.MoveThresh_g = 0.15f;   // Critical for detecting stillness

	MotionAC_SetKnobs(&ac_knobs);
}


/* Calibrate accelerometer sensor (LSM6DSR)*/
uint8_t CalibrateAccData(LSM6DSR_Object_t *hlsm6d) {
  MAC_input_t data_in;
  LSM6DSR_Axes_t acc_data;
  LSM6DSR_ACC_GetAxes(hlsm6d, &acc_data);

  // data cast and copy - mdps -> dps & mg -> g
  data_in.Acc[0]  = ((float)acc_data.x)/1000.0f;
  data_in.Acc[1]  = ((float)acc_data.y)/1000.0f;
  data_in.Acc[2]  = ((float)acc_data.z)/1000.0f;
  data_in.TimeStamp = HAL_GetTick(); // Use direct timestamp

  uint8_t is_calibrated = 0;
  // Calculate the compensated accelerometer data
  MotionAC_Update(&data_in, &is_calibrated);

  MotionAC_GetCalParams(&ac_data_out);

  static uint8_t counter = 0;
  if (counter++ % 100 == 0) {
	  printf("Time: %d ", counter);
	  int status = ac_data_out.CalQuality;
	  switch(status) {
	  	  case 1:
			printf("MAC_CALQSTATUS: POOR\n");
			break;
	  	case 2:
			printf("MAC_CALQSTATUS: OK\n");
			break;
	  	case 3:
			printf("MAC_CALQSTATUS: GOOD!!!!!\n");
			break;
	  	default:
	  		printf("MAC_CALQSTATUS: UNKNOWN!\n");
	  }
	  //printf("Status: %d\n", ac_data_out.CalQuality);
	  printf("Acc Bias:- X: %d Y: %d Z: %d\n", (int)(ac_data_out.AccBias[0]*1000), (int)(ac_data_out.AccBias[1]*1000), (int)(ac_data_out.AccBias[2]*1000));
  }

  if (is_calibrated) {
	  memcpy(cal_data, &ac_data_out, sizeof(ac_data_out)); // Serialize
	  MotionAC_SaveCalInNVM(sizeof(cal_data), cal_data);

	  printf("Acc cal done!!\n");
	  printf("Cal Q: %d | Acc Bias:- X: %d Y: %d Z: %d\n", (int)ac_data_out.CalQuality, (int)(ac_data_out.AccBias[0]*1000), (int)(ac_data_out.AccBias[1]*1000), (int)(ac_data_out.AccBias[2]*1000));
  }

  return is_calibrated;
}


/* Main loop for filtering  */
void ApplyAccCalibration(LSM6DSR_Axes_t *currAcc) {
	uint32_t len;
	char msg[100] = {};

	static uint32_t accounter = 0;

	if(!(accounter++%1000)) {
		//printf("Acc after Cal:- X: %d Y: %d Z: %d\n", currAcc->x, currAcc->y, currAcc->z);
		len = sprintf(msg, "Acc B:- X: %d Y: %d Z: %d\n", (int)currAcc->x, (int)currAcc->y, (int)currAcc->z);
		_write(0, msg, len);
	}

	//int-mg        int-mg           fl-g                   fl-g
	currAcc->x = (currAcc->x - (int32_t)(ac_data_out.AccBias[0]*1000)) * ac_data_out.SF_Matrix[0][0];
	currAcc->y = (currAcc->y - (int32_t)(ac_data_out.AccBias[1]*1000)) * ac_data_out.SF_Matrix[1][1];
	currAcc->z = (currAcc->z - (int32_t)(ac_data_out.AccBias[2]*1000)) * ac_data_out.SF_Matrix[2][2];

	if(!(accounter%1000)) {
		//printf("Acc before Cal:- X: %d Y: %d Z: %d\n\n", currAcc->x, currAcc->y, currAcc->z);
		len = sprintf(msg, "Acc A:- X: %d Y: %d Z: %d\n", (int)currAcc->x, (int)currAcc->y, (int)currAcc->z);
		_write(0, msg, len);

		float gravity = sqrt((currAcc->x * currAcc->x) + (currAcc->y * currAcc->y) + (currAcc->z * currAcc->z));
		len = sprintf(msg, "Gravity: %d mg\n", (int)gravity);
		_write(0, msg, len);
	}
}


void MotionFX_Init(void) {
	MFX_knobs_t knobs;

	if(STATE_SIZE < MotionFX_GetStateSize()) {
		Error_Handler();
	}

	// Initialize MotionFX with state pointer
	MotionFX_initialize((MFXState_t *)mfx_state);

	knobs.acc_orientation[0] = 'n';  // East
	knobs.acc_orientation[1] = 'w';  // North
	knobs.acc_orientation[2] = 'u';  // Up
	knobs.gyro_orientation[0] = 'n';
	knobs.gyro_orientation[1] = 'w';
	knobs.gyro_orientation[2] = 'u';

	MotionFX_getKnobs(mfx_state, &knobs);

	// Configure parameters
	knobs.modx = 1; // Update every propagate (Set 1 for STM32F4)
	knobs.LMode = 1;
	knobs.start_automatic_gbias_calculation = 1; // Auto Gyro Bias Calibraton
	knobs.output_type = MFX_ENGINE_OUTPUT_ENU; // Set orientation format
	knobs.gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;
	knobs.gbias_acc_th_sc = GBIAS_ACC_TH_SC;

	MotionFX_setKnobs(mfx_state, &knobs);
	MotionFX_enable_6X(mfx_state, MFX_ENGINE_ENABLE); // Enable 6-axis fusion (gyro + acc)
	MotionFX_enable_9X(mfx_state, MFX_ENGINE_DISABLE);

	// print library version
	char msg[100] = {};
	char version[35] = {};
	MotionFX_GetLibVersion(version);
	uint32_t len = sprintf(msg, "MotionFX version: %s\n", version);
	_write(0, msg, len);
}


void ProcessSensorData(dataframe_t *nextFrame) {
	float deltaTime = 1.0f / SAMPLE_FREQ; //0.01f or 10ms
	uint32_t len = 0;
	char msg[100] = {};
	MFX_input_t motion_input;
	MFX_output_t motion_output;

	motion_input.gyro[0] = nextFrame->currGyro.x / 1000.0f; // mdps -> dps
	motion_input.gyro[1] = nextFrame->currGyro.y / 1000.0f;
	motion_input.gyro[2] = nextFrame->currGyro.z / 1000.0f;

	motion_input.acc[0] = nextFrame->currAcc.x / 1000.0f; // mg -> g
	motion_input.acc[1] = nextFrame->currAcc.y / 1000.0f;
	motion_input.acc[2] = nextFrame->currAcc.z / 1000.0f;

	motion_input.mag[0] = 0.0f; // Not used in 6X
	motion_input.mag[1] = 0.0f;
	motion_input.mag[2] = 0.0f;

	/* Run Sensor Fusion algorithm */
	MotionFX_propagate(mfx_state, &motion_output, &motion_input, &deltaTime); //predictive
	MotionFX_update(mfx_state, &motion_output, &motion_input, &deltaTime, NULL); //corrective

	// Store formatted outputs
	nextFrame->roll = motion_output.rotation[0];
	nextFrame->pitch = motion_output.rotation[1];
	nextFrame->yaw = motion_output.rotation[2];
	memcpy(nextFrame->linAcc, motion_output.linear_acceleration, sizeof(float)*3);
	memcpy(nextFrame->gravity, motion_output.gravity, sizeof(float)*3);

	static uint32_t fxcounter = 0;

	if((++fxcounter % 1000 == 0)) {
		len = sprintf(msg, "Roll: %d deg\n", (int)nextFrame->roll);
		_write(0, msg, len);
		len = sprintf(msg, "Pitch: %d deg\n", (int)nextFrame->pitch);
		_write(0, msg, len);
		len = sprintf(msg, "Yaw: %d deg\n", (int)nextFrame->yaw);
		_write(0, msg, len);
		len = sprintf(msg, "X_Acc: %d mg\n", (int)(nextFrame->linAcc[0]*1000));
		_write(0, msg, len);
		len = sprintf(msg, "Y_Acc: %d mg\n", (int)(nextFrame->linAcc[1]*1000));
		_write(0, msg, len);
		len = sprintf(msg, "Z_Acc: %d mg\n\n", (int)(nextFrame->linAcc[2]*1000));
		_write(0, msg, len);
//		printf("Roll: %d deg\n", (int)nextFrame.roll);
//		printf("Pitch: %d deg\n", (int)nextFrame.pitch);
//		printf("Yaw: %d deg\n", (int)nextFrame.yaw);
//		printf("X_Gravity: %d g\n", nextFrame.gravity[0]);
//		printf("Y_Gravity: %d g\n", nextFrame.gravity[1]);
//		printf("Z_Gravity: %d g\n\n", nextFrame.gravity[2]);

		// Gravity - converted to fixed-point for embedded safety
//		len = sprintf(msg, "X_Gravity: %d mg\n", (int)(nextFrame.gravity[0]*1000));
//		_write(0, msg, len);
//		len = sprintf(msg, "Y_Gravity: %d mg\n", (int)(nextFrame.gravity[1]*1000));
//		_write(0, msg, len);
//		len = sprintf(msg, "Z_Gravity: %d mg\n\n", (int)(nextFrame.gravity[2]*1000));
//		_write(0, msg, len);
	}
}

