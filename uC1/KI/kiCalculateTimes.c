/*
* kiWatchRobotPosition.c
*
* Created: 15.11.2023 20:30:17
*  Author: marku
*/

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>

#include "kiCalculateTimes.h"
#include "ki.h"
#include "define.h"
#include "global.h"
#include "command.h"
#include "Pfadplanung.h"
#include "multitask.h"
#include "spielZeit.h"
#include "nextion.h"

void InitKiCalculateTimesTask(void)
{
	// KiWatchRobotPositionTask initialisieren
	SET_TASK(KI_Calculate_Times_TASKNBR, CYCLE);
	SET_CYCLE(KI_Calculate_Times_TASKNBR, 100);
	SET_TASK_HANDLE(KI_Calculate_Times_TASKNBR, KiCalculateTimesTask);
}

/**************************************************************************
***   FUNCTIONNAME:        KiWatchRobotPositionTask                     ***
***   FUNCTION:            Sendet die Daten an den Nebenroboter			***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t KiCalculateTimesTask(void)
{
	// zyklischer Task - Zykluszeit: 100 ms
	SET_CYCLE(KI_Calculate_Times_TASKNBR, 100);
	
	
	// Start Position to begin movement from
	point_t start;
	start.Xpos = xPos;
	start.Ypos = yPos;
	
	
	/**************************************************************************
	***   Calculate Time to Drive Home					                    ***
	**************************************************************************/
	float distanceToHome;
	distanceToHome = CalcDistance(start,PosHome);
	
	//Time To Drive Home from Actual Position
	TimeToHome = (uint16_t)((distanceToHome /(float)STANDARD_VELOCITY)*10.0); //In 10tel seconds
	
	
	/**************************************************************************
	***   Time to Handle next Plant						                    ***
	**************************************************************************/
	TimeHandleNextPlant = 20; //Muss später live umgedated werden je nachdem wo aktuell der Roboter Pflanzen in sich hat
	
	/**************************************************************************
	***   Time to Park next Plant						                    ***
	**************************************************************************/
	TimeParkNextPlantInField = 0; //Muss später live umgedated werden je nachdem wo aktuell der Roboter Pflanzen in sich hat
	TimeParkNextPlantInPlanter = 30; //Muss später live umgedated werden je nachdem wo aktuell der Roboter Pflanzen in sich hat
	
	/**************************************************************************
	***   Calculate Time to ParkPlants at Home Position	   			        ***
	**************************************************************************/
	if(PlantsInRobot == 3)
	{
		TimeParkPlantsAtHome = TimePark3PlantsAtHome;
		TimeParkPlantsPlus1AtHome = TimePark3PlantsAtHome;
		TimeParkPlantsMinus1AtHome = TimePark2PlantsAtHome;
	}
	else if(PlantsInRobot == 2)
	{
		TimeParkPlantsAtHome = TimePark2PlantsAtHome;
		TimeParkPlantsPlus1AtHome = TimePark3PlantsAtHome;
		TimeParkPlantsMinus1AtHome = TimePark1PlantAtHome;
	}
	else if(PlantsInRobot == 1)
	{
		TimeParkPlantsAtHome = TimePark1PlantAtHome;
		TimeParkPlantsPlus1AtHome = TimePark2PlantsAtHome;
		TimeParkPlantsMinus1AtHome = 0;
	}
	else
	{
		TimeParkPlantsAtHome = 0;
		TimeParkPlantsPlus1AtHome = TimePark1PlantAtHome;
		TimeParkPlantsMinus1AtHome = 0;
	}
	
	/**************************************************************************
	***   Calculate Time to GetPlant					                    ***
	**************************************************************************/
	float distanceToPlant1000 = CalcDistance(start,PosPlant1000);
	float distancePlant1000ToHome = CalcDistance(PosPlant1000,PosHome);
	float distanceToPlant2000 = CalcDistance(start,PosPlant2000);
	float distancePlant2000ToHome = CalcDistance(PosPlant2000,PosHome);
	float distanceToPlant3000 = CalcDistance(start,PosPlant3000);
	float distancePlant3000ToHome = CalcDistance(PosPlant3000,PosHome);
	float distanceToPlant4000 = CalcDistance(start,PosPlant4000);
	float distancePlant4000ToHome = CalcDistance(PosPlant4000,PosHome);
	float distanceToPlant5000 = CalcDistance(start,PosPlant5000);
	float distancePlant5000ToHome = CalcDistance(PosPlant5000,PosHome);
	float distanceToPlant6000 = CalcDistance(start,PosPlant6000);
	float distancePlant6000ToHome = CalcDistance(PosPlant6000,PosHome);
	
	TimeForPlant1000 = (uint16_t)((distanceToPlant1000 + distancePlant1000ToHome)/(float)STANDARD_VELOCITY) + TimeHandleNextPlant + TimeParkPlantsPlus1AtHome;
	TimeForPlant2000 = (uint16_t)((distanceToPlant2000 + distancePlant2000ToHome)/(float)STANDARD_VELOCITY) + TimeHandleNextPlant + TimeParkPlantsPlus1AtHome;
	TimeForPlant3000 = (uint16_t)((distanceToPlant3000 + distancePlant3000ToHome)/(float)STANDARD_VELOCITY) + TimeHandleNextPlant + TimeParkPlantsPlus1AtHome;
	TimeForPlant4000 = (uint16_t)((distanceToPlant4000 + distancePlant4000ToHome)/(float)STANDARD_VELOCITY) + TimeHandleNextPlant + TimeParkPlantsPlus1AtHome;
	TimeForPlant5000 = (uint16_t)((distanceToPlant5000 + distancePlant5000ToHome)/(float)STANDARD_VELOCITY) + TimeHandleNextPlant + TimeParkPlantsPlus1AtHome;
	TimeForPlant6000 = (uint16_t)((distanceToPlant6000 + distancePlant6000ToHome)/(float)STANDARD_VELOCITY) + TimeHandleNextPlant + TimeParkPlantsPlus1AtHome;
	
	/**************************************************************************
	***   Calculate Time to Park					                    ***
	**************************************************************************/
	float distancePlanterLMiddle = CalcDistance(start,PosPlanterMidleBlue);
	float distancePlanterLMiddleToHome = CalcDistance(PosPlanterMidleBlue,PosFieldL3);
	float distanceFieldL1 = CalcDistance(start,PosFieldL1);
	float distanceFieldL1ToHome = CalcDistance(PosFieldL1,PosFieldL3);
	float distancePlanterL1 = CalcDistance(start,PosPlanterL1);
	float distancePlanterL1ToHome = CalcDistance(PosPlanterL1,PosFieldL3);
	float distancePlanterL2 = CalcDistance(start,PosPlanterL2);
	float distancePlanterL2ToHome = CalcDistance(PosPlanterL2,PosFieldL3);
	
	float distancePlanterRMiddle = CalcDistance(start,PosPlanterMidleYellow);
	float distancePlanterRMiddleToHome = CalcDistance(PosPlanterMidleYellow,PosFieldR3);
	float distanceFieldR1 = CalcDistance(start,PosFieldR1);
	float distanceFieldR1ToHome = CalcDistance(PosFieldR1,PosFieldR3);
	float distancePlanterR1 = CalcDistance(start,PosPlanterR1);
	float distancePlanterR1ToHome = CalcDistance(PosPlanterR1,PosFieldR3);
	float distancePlanterR2 = CalcDistance(start,PosPlanterR2);
	float distancePlanterR2ToHome = CalcDistance(PosPlanterR2,PosFieldR3);
	
	TimeParkPlanterLMiddle = (uint16_t)((distancePlanterLMiddle+distancePlanterLMiddleToHome)/(float)STANDARD_VELOCITY) + TimeParkNextPlantInPlanter + TimeParkPlantsMinus1AtHome;
	TimeParkFieldL1 = (uint16_t)((distanceFieldL1+distanceFieldL1ToHome)/(float)STANDARD_VELOCITY) + TimeParkNextPlantInField + TimeParkPlantsMinus1AtHome;
	TimeParkPlanterL1 = (uint16_t)((distancePlanterL1+distancePlanterL1ToHome)/(float)STANDARD_VELOCITY) + TimeParkNextPlantInPlanter + TimeParkPlantsMinus1AtHome;
	TimeParkPlanterL2 = (uint16_t)((distancePlanterL2+distancePlanterL2ToHome)/(float)STANDARD_VELOCITY) + TimeParkNextPlantInPlanter + TimeParkPlantsMinus1AtHome;
	TimeParkPlanterRMiddle = (uint16_t)((distancePlanterRMiddle+distancePlanterRMiddleToHome)/(float)STANDARD_VELOCITY) + TimeParkNextPlantInPlanter + TimeParkPlantsMinus1AtHome;
	TimeParkFieldR1 = (uint16_t)((distanceFieldR1+distanceFieldR1ToHome)/(float)STANDARD_VELOCITY) + TimeParkNextPlantInField + TimeParkPlantsMinus1AtHome;
	TimeParkPlanterR1 = (uint16_t)((distancePlanterR1+distancePlanterR1ToHome)/(float)STANDARD_VELOCITY) + TimeParkNextPlantInPlanter + TimeParkPlantsMinus1AtHome;
	TimeParkPlanterR2 = (uint16_t)((distancePlanterR2+distancePlanterR2ToHome)/(float)STANDARD_VELOCITY) + TimeParkNextPlantInPlanter + TimeParkPlantsMinus1AtHome;
	
	/**************************************************************************
	***   Calculate Times SolarPanelsMiddle					             ***
	**************************************************************************/
	float distanceToSolarPanelsMiddle = CalcDistance(start,PosSolarPanelsMiddle);
	
	//Time To Drive to Solar Panels Middle from Actual Position
	TimeToSolarPanelsMiddle = (uint16_t)((distanceToSolarPanelsMiddle /(float)STANDARD_VELOCITY)*10.0); //In 10tel seconds
	
	//Time To Drive Home form Solar Panels Middle
	TimeSolarPanelsMiddleToHome = (uint16_t)((1050.0/(float)STANDARD_VELOCITY)*10.0);
	
	/**************************************************************************
	***   Calculate Times Solar Panels Home			                        ***
	**************************************************************************/
	//Time to Drive to Solar Panels at Home Position and make all Solar Panels
	TimeAllSolarPanelsHome = TimeToHome + 3*TimeSolarpanels + TimeSetPosAtSolarPanels + TimeParkPlantsAtHome;
	//Time to Drive to Solar Panels at Home Position and make at least 1 Solar Panel
	TimeSolarPanelsHome = TimeToHome + TimeSolarpanels + TimeSetPosAtSolarPanels + TimeParkPlantsAtHome;

	return(CYCLE);
}
