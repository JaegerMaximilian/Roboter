/*
* usDataFusion.h
*
* Created: 17.11.2021 10:45:10
*  Author: johan
*/


#ifndef US_DATA_FUSION_H_
#define US_DATA_FUSION_H_

/* intern/extern switch */
#ifndef _US_DATA_FUSION_EXTERN
#define	 _US_DATA_FUSION_EXTERN extern
#endif

void usDataFusion_Init(void);
uint8_t dataFusionTask();

_US_DATA_FUSION_EXTERN uint8_t validWIFI, validEnemyDetection;


#endif /* US_DATA_FUSION_H_ */