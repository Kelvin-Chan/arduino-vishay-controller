/**
 * @file Sensor.c
 * @author Kelvin Chan
 * @date 29 Jan 2021
 * @brief Source file for Sensor struct, and related variables, methods
 * 
 * The Sensor object is intended to represent the data collected from Vishay's VCNLXXXX IR gesture, proximity
 * sensor series. This would provide the state information for light zone selection, intensity setting, and
 * the overall controller for light intensity.
 */


#include "Sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

const volatile uint16_t distanceTable[DIST_LOOKUP_LEN] =
{
		0,	2,	4,	6,	8,
		10,	12,	14,	16,	18,
		20,	22, 24,	26,	28,
		30
};

void Init_Sensor(Sensor* sensor, uint8_t index, uint16_t psProxMin, uint16_t psProxMax, uint16_t* proxTable)
{
	sensor->index = index;
	sensor->psProxMin = psProxMin;
	sensor->psProxMax = psProxMax;
	sensor->proxTable = proxTable;
	
	Reset_Sensor(sensor);
}

void Reset_Sensor(Sensor* sensor)
{
	sensor->sampleCount = 0;
	sensor->psMean = 0;
	sensor->psSTD = 0;
	sensor->alsMean = 0;
	sensor->alsSTD = 0;
	sensor->inProximity = 0;
	sensor->isBlocked = 0;
	sensor->psWindowSum = 0;
	sensor->alsWindowSum = 0;
	memset(sensor->psHist, 0, SENSOR_HIST_LEN * sizeof(uint16_t));
	memset(sensor->alsHist, 0, SENSOR_HIST_LEN * sizeof(uint16_t));
}

void Update_Sensor(Sensor* sensor, uint16_t psVal, uint16_t alsVal)
{
	uint16_t i, ind, windowInd;
	double errorSum;
	double meanDouble;	// Keeps precision when calculating STD

	//	Update PS rolling window sum
	if (sensor->sampleCount < PS_WINDOW)
	{
		sensor->psWindowSum += psVal;
	}
	else
	{
		windowInd = (sensor->sampleCount - PS_WINDOW) % SENSOR_HIST_LEN;
		sensor->psWindowSum -= sensor->psHist[windowInd];
		sensor->psWindowSum += psVal;
	}

	//	Update ALS rolling window sum
	if (sensor->sampleCount < ALS_WINDOW)
	{
		sensor->alsWindowSum += alsVal;
	}
	else
	{
		windowInd = (sensor->sampleCount - ALS_WINDOW) % SENSOR_HIST_LEN;
		sensor->alsWindowSum += ((int) alsVal) - ((int) sensor->alsHist[windowInd]);
	}
	
	// Circular buffer for history
	ind = sensor->sampleCount % SENSOR_HIST_LEN;
	sensor->psHist[ind] = psVal;
	sensor->alsHist[ind] = alsVal;

	//	Calculate PS Mean from window sum
	if ((sensor->sampleCount + 1) >= PS_WINDOW)
		meanDouble = (double) sensor->psWindowSum / PS_WINDOW;
	else
		meanDouble = (double) sensor->psWindowSum / (sensor->sampleCount + 1);
	
	sensor->psMean = (uint16_t) floor(meanDouble);
	
	//	Get estimated distance from PS mean
	sensor->estimatedDistance = Distance_Lookup(sensor->psMean, sensor->proxTable, (uint16_t *) distanceTable, DIST_LOOKUP_LEN);
	
	//	Calculate PS STD
	errorSum = 0;
	for (i = 0; i < PS_WINDOW; i++)
	{
		if ((((int) sensor->sampleCount) - i) >= 0)
		{
			windowInd = (((int) sensor->sampleCount) - i) % SENSOR_HIST_LEN;
			errorSum += pow((double)(sensor->psHist[windowInd] - meanDouble), 2);
		}
		else
			break;
	}
	sensor->psSTD = sqrt((double) errorSum / i);

	//	Calculate ALS mean from window
	if (sensor->sampleCount >= ALS_WINDOW - 1)
		meanDouble = (double) sensor->alsWindowSum / ALS_WINDOW;
	else
		meanDouble = (double) sensor->alsWindowSum / (sensor->sampleCount + 1);
	
	sensor->alsMean = (uint16_t) floor(meanDouble);
	
	//	Calculate ALS STD
	errorSum = 0;
	for (i = 0; i < ALS_WINDOW; i++)
	{
		if ((((int) sensor->sampleCount) - i) >= 0)
		{
			windowInd = (((int) sensor->sampleCount) - i) % SENSOR_HIST_LEN;
			errorSum += pow((double)(sensor->alsHist[windowInd] - meanDouble), 2);
		}
		else
			break;
	}
	
	sensor->alsSTD = sqrt((double) errorSum / i);

	//	Update inProximity flag
	if (sensor->inProximity && (psVal <= sensor->psProxMin))
	{
#ifdef _DEBUG
		printf("Exiting from proximity at: %d\n", sensor->psMean);
		printf("Estimated Distance at: %f cm\n", sensor->estimatedDistance);
#endif
		sensor->inProximity = 0;
	}
	else if (!sensor->inProximity && (psVal >= sensor->psProxMax))
	{
#ifdef _DEBUG
		printf("Entering into proximity at: %d\n", sensor->psMean);
		printf("Estimated Distance at: %f cm\n", sensor->estimatedDistance);
#endif
		sensor->inProximity = 1;
	}

	//	Update isBlocked flag
	if (!sensor->isBlocked && sensor->inProximity && (sensor->alsMean == 0) && (sensor->alsSTD == 0))
	{
		sensor->isBlocked = 1;
	}
	else if (sensor->isBlocked && !sensor->inProximity)
	{
		sensor->isBlocked = 0;
	}

	sensor->sampleCount++;
}

double Distance_Lookup(uint16_t psVal, uint16_t* proxTable, uint16_t* distTable, uint8_t tableLen)
{
	uint8_t i;
	double distance;
	
	for (i = 0; i < tableLen; i++)
	{	
		if (psVal >= proxTable[i])
		{
			if (i == 0) return (double) distTable[i];
			
			// Linear interpolation of distance between two PS values
			distance = ((double) distTable[i-1]) + 
						((double) psVal - (double) proxTable[i-1]) *
						((double) distTable[i] - (double) distTable[i-1]) / ((double) proxTable[i] - (double) proxTable[i-1]);
      			
			return distance;
		}
	}
	
	// PS value not found in distance table
	return (double) distTable[tableLen - 1];
}

#ifdef __cplusplus
}
#endif
