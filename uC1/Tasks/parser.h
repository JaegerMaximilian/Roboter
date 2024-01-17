/*
 * parser.h
 *
 * Created: 27.10.2020 10:07:56
 *  Author: P20087
 */ 


#ifndef PARSER_H_
#define PARSER_H_

/* ******************************** */
/* ********      enums     ******** */
/* ******************************** */
/* character for block-start */
#define BS		'#'
/* character for block-end */
#define BE		'*'

#define DEL		'|'
/* servo-command */
#define CMD_SERVO		'S'
/* motor-command */
#define CMD_MOTOR		'M'
/* digital-output-command */
#define CMD_DIGITAL_OUT	'O'
/* vacuum-command */
#define CMD_VACUUM		'v'
/* velocity-command */
#define CMD_VELOCITY		'V'
/* position-command */
#define CMD_POSITION		'P'
/* Teile von Vision -command */
#define CMD_TEILE		'T'
/* Roboter Position -command */
#define CMD_ROBOTER		'R'
/* Punkte -command */
#define CMD_PUNKTE		'p'
/* Sperrgebiet -command */
#define CMD_ZONE		'Z'
/* Debugmessage -command */
#define CMD_DEBUG		'D'
/* Cherry -command */
#define CMD_CHERRY		'C'
/*Robot 1 (Master)*/
#define ADR_MASTER_ID	'1'
/*Robot 2 (Slave)*/
#define ADR_SLAVE_ID	'2'
/*Vision 1*/
#define ADR_VISION_ID_V		'V'
/*Computer*/
#define ADR_COMPUTER_ID		'T'
/*Broadcast*/
#define ADR_BROADCAST_ID	'B'
/*Vision 2*/
#define ADR_VISION_ID_W		'W'
/*Cherry Count*/
#define ADR_CHERRY_ID		'K'



/* ******************************** */
/* ********   prototypes   ******** */
/* ******************************** */
void InitParser(void);
uint8_t ParserTask(void);


#endif /* PARSER_H_ */