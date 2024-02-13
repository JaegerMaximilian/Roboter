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
/* blumen: ob Plant.Status */
#define CMD_BLUMEN		'B'
/* Teile von Vision -command */
#define CMD_POSITION	'T'
/* Debugmessage -command */
#define CMD_DEBUG		'D'

/*Robot 1 (Master)*/
#define ADR_MASTER_ID	'1'
/*Robot 2 (Slave)*/
#define ADR_SLAVE_ID	'2'
/*Vision 1*/
#define ADR_VISION_ID_V		'V'
/*Computer*/
#define ADR_LOGGER_ID		'T'
/*Broadcast*/
#define ADR_BROADCAST_ID	'B'
/*Vision 2*/
#define ADR_VISION_ID_W		'W'




/* ******************************** */
/* ********   prototypes   ******** */
/* ******************************** */
void InitParser(void);
uint8_t ParserTask(void);


#endif /* PARSER_H_ */