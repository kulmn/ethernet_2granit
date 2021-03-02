#ifndef __WEB_IF_H_
#define __WEB_IF_H_

#include "lan.h"
#include "convert_fn.h"
#include <libopencm3/stm32/flash.h>

#define last_page  FLASH_BASE + (31 * 1024)

extern Lan_Config_TypeDef lan_config;
extern uint32_t 	ip_addr;
extern uint32_t ip_mask;
extern uint32_t ip_gateway;

void webif_data(uint8_t id, eth_frame_t *frame, uint16_t len);
uint8_t *ip2str(uint32_t ipaddr, uint8_t *buffer);
uint32_t str2ip(uint8_t* addr);
void save_config(void);



#endif
