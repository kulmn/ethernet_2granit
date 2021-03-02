/*
 * convert_fn.h
 *
 *  Created on: 14 ���� 2015 �.
 *      Author: kulish_y
 */

#ifndef CONVERT_FN_H_
#define CONVERT_FN_H_

#include <stdint.h>

uint32_t uint32_to_bcd(uint32_t value);
uint8_t *Uint32ToStr(uint32_t value, uint8_t *buffer);
uint16_t str_to_uint16( uint8_t *str);



#endif /* CONVERT_FN_H_ */
