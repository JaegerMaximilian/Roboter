/*
 * wifi.h
 *
 * Created: 13.05.2021 09:27:35
 *  Author: P20087
 */ 



#ifndef WIFI_H_
#define WIFI_H_

#ifndef _WIFI_EXTERN
#define _WIFI_EXTERN extern
#endif

/* point on playground (x,y) */
typedef struct {
	int8_t ID;
	int16_t Xpos;
	int16_t Ypos;
	int16_t phi;
	int16_t iDetected;	
} aruco_t;
_WIFI_EXTERN aruco_t arucoCodes[20];


/* ******************************** */
/* ********      enums     ******** */
/* ******************************** */
/* character for block-start */
#define BS_WIFI		'#'
/* character for block-end */
#define BE_WIFI		'*'
/* port-command -> north/south (#Pp* :: p -> 0 .. not defined, 1 .. south, 2 .. north) */
#define CMD_PORT			'p'
/* master position-command (#Mxxxxyyyy* :: xxxx -> x-position [mm], yyyy -> y-position [mm]) */
#define CMD_MASTER_POS		'm'
/* slave position-command (#Sxxxxyyyy* :: xxxx -> x-position [mm], yyyy -> y-position [mm]) */
#define CMD_SLAVE_POS		's'
/* enemy position-command (#Sxxxxyyyy* :: xxxx -> x-position [mm], yyyy -> y-position [mm], vvvv -> variance)         */
#define CMD_ENEMY_POS		'e'

/* hexagon position-command (#Hiixxxyyyyaa :: ii -> Hexagon ID, xxx-position [mm], yyy -> y-position [mm], aa -> angel)         */
#define CMD_HEX_POS		'h'
#define ARUCO_NO_ID		-1
#define ARUCO_GREEN		36
#define ARUCO_RED		47
#define ARUCO_BLUE		13
#define ARUCO_BROWN	    50
#define ARUCO_YELLOW_ROBOT1	
#define ARUCO_YELLOW_ROBOT2
#define ARUCO_PURPLE_ROBOT1
#define ARUCO_PURPLE_ROBOT2




typedef struct 
{
	uint16_t x[5];
	uint16_t y[5];
	uint16_t var[5];
	uint8_t count;

}EnemyDataRaw_t;



_WIFI_EXTERN uint8_t EndPositionSlave,DriveHome;
_WIFI_EXTERN EnemyDataRaw_t EnemyDataRaw;

/* ******************************** */
/* ********   prototypes   ******** */
/* ******************************** */
void InitWifi(void);
uint8_t WifiTask(void);
void wifi_PortCommand(uint8_t port);
void wifi_MasterPosCommand(int16_t x, int16_t y);
void wifi_SlavePosCommand(int16_t x, int16_t y);
uint8_t wifi_CyclicTask(void);


#endif /* WIFI_H_ */