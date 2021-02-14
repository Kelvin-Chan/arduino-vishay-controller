/**
 * @file Sensor.h
 * @author Kelvin Chan
 * @date 29 Jan 2021
 * @brief Header file for Sensor struct, and related variables, methods
 * 
 * The Sensor object is intended to represent the data collected from Vishay's VCNLXXXX IR gesture, proximity
 * sensor series. This would provide the state information for light zone selection, intensity setting, and
 * the overall controller for light intensity.
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <string.h>
#ifdef __GNUC__			/* GNU Compiler Test */
#include <stdint.h>
#else
#include <PE_Types.h>
#endif
#include <math.h>

/** @brief Length of the sensor value history */
#define SENSOR_HIST_LEN 50

/** @brief Length of proximity window */
#define PS_WINDOW 25

/** @brief Length of ALS window */
#define ALS_WINDOW 25

/** @brief Length of #distanceTable array */
#define DIST_LOOKUP_LEN 16

/**
 * @brief Distance reference values for distance lookup via proximity counts
 */
extern const volatile uint16_t distanceTable[DIST_LOOKUP_LEN];

/**
 * @struct Sensor_t
 * @brief Sensor struct for storing sensor states, window history, parameters
 */
typedef struct __attribute__ ((__packed__)) Sensor_t
{	
	/** @brief Index value of sensor for identifying position */
	uint8_t index;
	
	/** @brief Number of samples collected by sensor */
	uint32_t sampleCount;
	
	/** @brief Proximity lookup table with respect to #distanceTable */
	uint16_t* proxTable;
	
	/** @brief Proximity history for mean, STD calculation */
	uint16_t psHist[SENSOR_HIST_LEN];
	
	/** @brief ALS history for mean, STD calculation */
	uint16_t alsHist[SENSOR_HIST_LEN];
	
	/** @brief PS mean value calculated from historical window */
	uint16_t psMean;
	
	/** @brief PS STD value calculated from historical window */
	double psSTD;
	
	/** @brief ALS mean value calculated from historical window */
	uint16_t alsMean;
	
	/** @brief ALS STD value calculated from historical window */
	double alsSTD;
	
	/** @brief Estimated distance looked up from mean proximity */
	double estimatedDistance;
	
	/** @brief Hysteresis exit threshold for #Sensor.inProximity */
	uint16_t psProxMin;
	
	/** @brief Hysteresis enter threshold for \ref Sensor.inProximity */
	uint16_t psProxMax;
	
	/** @brief Flag for target detected within sensor proximity */
	uint8_t inProximity;
	
	/** @brief Flag for target detected obstructing sensor */
	uint8_t isBlocked;
	
	/** @brief Sum of PS_WINDOW elements within \ref Sensor.psHist */
	uint32_t psWindowSum;
	
	/** @brief Sum of ALS_WINDOW latest elements within \ref Sensor.alsHist */
	uint32_t alsWindowSum;
} Sensor;

/**
 * @brief Initialize sensor with required parameters
 * 
 * @param [out] sensor
 * @param [in] index
 * @param [in] psProxMin
 * @param [in] psProxMax
 * @param [in] proxTable
 */
void Init_Sensor(Sensor* sensor, uint8_t index, uint16_t psProxMin, uint16_t psProxMax, uint16_t* proxTable);

/**
 * @brief Reset sensor's states to default
 * 
 * @param [out] sensor
 */
void Reset_Sensor(Sensor* sensor);

/**
 * @brief Update sensor with latest proximity, ALS value
 * 
 * @param [out] sensor
 * @param [in] psVal
 * @param [in] alsVal
 */
void Update_Sensor(Sensor* sensor, uint16_t psVal, uint16_t alsVal);

/**
 * @brief Distance lookup for proximity counts based on linear interpolation
 * 
 * @param [in] psVal
 * @param [in] proxTable
 * @param [in] distTable
 * @param [in] tableLen
 * @return estimatedDistance (in cm)
 */
double Distance_Lookup(uint16_t psVal, uint16_t* proxTable, uint16_t* distTable, uint8_t tableLen);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* SENSOR_H_ */
