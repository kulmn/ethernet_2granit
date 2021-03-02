#include <web_if.h>

#include <strings.h>

const uint8_t http_404_reply[] =
	"HTTP/1.0 404 Not Found\r\n"
	"Content-Type: text/html; charset=windows-1251\r\n"
	"Server: STM32\r\n"
	"\r\n"
	"<pre>Page not found\r\n\r\n"
	"<a href='/'>Home page</a></pre>\r\n";

const uint8_t http_200_header[] =
	"HTTP/1.0 200 OK\r\n"
	"Content-Type: text/html; charset=windows-1251\r\n"
	"Server: STM32\r\n"
	"\r\n";

uint16_t		serial_speed;
uint16_t				granit_n_kp0;
uint16_t				granit_n_kp1;
extern uint32_t second_count;

extern uint32_t serial_pct_rx0_cnt;
extern uint32_t serial_pct_rx1_cnt;

/******************************************************************************/
void fill_buf(uint8_t **buf, const uint8_t *str)
{
	while(*str) *((*buf)++) = *(str++);
}

/******************************************************************************/
uint8_t *html_root_page( uint8_t *buffer)
{
		fill_buf(&buffer, http_200_header);
		fill_buf(&buffer, "<pre><font size=\"5\" >Ethernet to RS232/Granit GATEWAY V0.2</font><font size=\"4\" >");
		fill_buf(&buffer, "\r\n\r\n Uptime: ");
		buffer=Uint32ToStr(second_count , buffer);
		fill_buf(&buffer, " seconds");

		fill_buf(&buffer, "\r\n\r\n Port 0 packets rx: ");
		buffer=Uint32ToStr(serial_pct_rx0_cnt , buffer);
		fill_buf(&buffer, "\r\n\r\n Port 1 packets rx: ");
		buffer=Uint32ToStr(serial_pct_rx1_cnt , buffer);
//		fill_buf(&buffer, "\r\n\r\n Serial packets rx errors: ");
//		buffer=Uint32ToStr(serial_pct_rx_err_cnt , buffer);

		fill_buf(&buffer, "\r\n\r\n<a href='/ether'>Ethernet port settings</a>");
		fill_buf(&buffer, "\r\n\r\n<a href='/serial'>Serial port settings</a>");

		fill_buf(&buffer, "</font></pre>");

		return buffer;
}


/******************************************************************************/
uint8_t *html_serial_page( uint8_t *buffer)
{
		fill_buf(&buffer, http_200_header);
		fill_buf(&buffer, "<pre><font size=\"4\" >");

		fill_buf(&buffer, "\r\n\r\n Serial port speed: <a href='/s_baud'>");
		buffer=Uint32ToStr(serial_speed , buffer);
		fill_buf(&buffer, "</a>");

		fill_buf(&buffer, "\r\n\r\n Port 0 Granit KP #: <a href='/n_kp0'>");
		buffer=Uint32ToStr(granit_n_kp0 , buffer);
		fill_buf(&buffer, "</a> (0 - disabled)");

		fill_buf(&buffer, "\r\n\r\n Port 1 Granit KP #: <a href='/n_kp1'>");
		buffer=Uint32ToStr(granit_n_kp1 , buffer);
		fill_buf(&buffer, "</a> (0 - disabled)");

		fill_buf(&buffer, "\r\n\r\n <a href='/save'>Save and reboot</a>");
		fill_buf(&buffer, "\r\n\r\n <a href='/'>Home page</a></font></pre>");


		return buffer;
}

/******************************************************************************/
uint8_t *html_ether_page( uint8_t *buffer)
{
		fill_buf(&buffer, http_200_header);
		fill_buf(&buffer, "<pre><font size=\"4\" >");

		fill_buf(&buffer, "\r\n\r\n IP: <a href='/l_ip'>");
		buffer=ip2str(lan_config.ip_address,buffer);
		fill_buf(&buffer, "</a>");

		fill_buf(&buffer, "\r\n\r\n MASK: <a href='/mask'>");
		buffer=ip2str(lan_config.ip_mask,buffer);
		fill_buf(&buffer, "</a>");

		fill_buf(&buffer, "\r\n\r\n GATEWAY: <a href='/ip_gw'>");
		buffer=ip2str(lan_config.ip_gateway,buffer);
		fill_buf(&buffer, "</a>");

		fill_buf(&buffer, "\r\n\r\n Remote IP: <a href='/r_ip'>");
		buffer=ip2str(lan_config.rem_ip_addr,buffer);
		fill_buf(&buffer, "</a>");

		fill_buf(&buffer, "\r\n\r\n Local UDP Ports: <a href='/l_udp0'>");
		buffer=Uint32ToStr(ntohs(lan_config.loc_udp_port0) , buffer);
		fill_buf(&buffer, "</a>   <a href='/l_udp1'>");
		buffer=Uint32ToStr(ntohs(lan_config.loc_udp_port1) , buffer);
		fill_buf(&buffer, "</a>");

		fill_buf(&buffer, "\r\n\r\n Remote UDP Ports: <a href='/r_udp0'>");
		buffer=Uint32ToStr(ntohs(lan_config.rem_udp_port0) , buffer);
		fill_buf(&buffer, "</a>   <a href='/r_udp1'>");
		buffer=Uint32ToStr(ntohs(lan_config.rem_udp_port1) , buffer);
		fill_buf(&buffer, "</a>");

		fill_buf(&buffer, "\r\n\r\n <a href='/save'>Save and reboot</a>");
		fill_buf(&buffer, "\r\n\r\n <a href='/'>Home page</a></font></pre>");

		return buffer;
}

/******************************************************************************/
uint8_t *html_get_pass( uint8_t *buffer)
{
		fill_buf(&buffer, http_200_header);
		fill_buf(&buffer,"<pre><form action='/' method='GET'>\r\nEnter password:\r\n<input type=password name=passw size=10 maxlength=4>");

		fill_buf(&buffer,"<a href=/><input name=re type=button value=Refresh></a>");
		fill_buf(&buffer, "</font></pre>");
		return buffer;
}


/******************************************************************************/
uint8_t *html_set_form( uint8_t *buffer, uint8_t num)
{
	fill_buf(&buffer, http_200_header);

	switch (num)
	{
		case 0:
		{
			fill_buf(&buffer,"<pre><form action='/ether' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"l_ip' size='15' value='");
			buffer=ip2str(lan_config.ip_address,buffer);
			break;
		}
		case 1:
		{
			fill_buf(&buffer,"<pre><form action='/ether' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"mask' size='15' value='");
			buffer=ip2str(lan_config.ip_mask,buffer);
			break;
		}
		case 2:
		{
			fill_buf(&buffer,"<pre><form action='/ether' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"ip_gw' size='15' value='");
			buffer=ip2str(lan_config.ip_gateway,buffer);
			break;
		}
		case 3:
		{
			fill_buf(&buffer,"<pre><form action='/ether' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"l_udp0' size='15' value='");
			buffer=Uint32ToStr(ntohs(lan_config.loc_udp_port0) , buffer);
			break;
		}
		case 4:
		{
			fill_buf(&buffer,"<pre><form action='/ether' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"r_ip' size='15' value='");
			buffer=ip2str(lan_config.rem_ip_addr,buffer);
			break;
		}
		case 5:
		{
			fill_buf(&buffer,"<pre><form action='/ether' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"r_udp0' size='15' value='");
			buffer=Uint32ToStr(ntohs(lan_config.rem_udp_port0) , buffer);
			break;
		}
		case 6:
		{
			fill_buf(&buffer,"<pre><form action='/serial' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"s_baud' size='15' value='");
			buffer=Uint32ToStr(serial_speed , buffer);
			break;
		}
		case 7:
		{
			fill_buf(&buffer,"<pre><form action='/serial' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"n_kp0' size='15' value='");
			buffer=Uint32ToStr(granit_n_kp0 , buffer);
			break;
		}
		case 8:
		{
			fill_buf(&buffer,"<pre><form action='/serial' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"n_kp1' size='15' value='");
			buffer=Uint32ToStr(granit_n_kp1 , buffer);
			break;
		}
		case 9:
		{
			fill_buf(&buffer,"<pre><form action='/ether' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"l_udp1' size='15' value='");
			buffer=Uint32ToStr(ntohs(lan_config.loc_udp_port1) , buffer);
			break;
		}
		case 10:
		{
			fill_buf(&buffer,"<pre><form action='/ether' method='GET'>\r\nEnter new value:\r\n<input type='text' name='");
			fill_buf(&buffer,"r_udp1' size='15' value='");
			buffer=Uint32ToStr(ntohs(lan_config.rem_udp_port1) , buffer);
			break;
		}

	}

	fill_buf(&buffer,"'>  <input type='submit' value='OK'>\r\n</form></pre>");
	return buffer;

}



/******************************************************************************/
void webif_data(uint8_t id, eth_frame_t *frame, uint16_t len)
{
	ip_packet_t *ip = (void*)(frame->data);
	tcp_packet_t *tcp = (void*)(ip->data);
	uint8_t *req = (void*)tcp_get_data(tcp);
	uint8_t *buf = (void*)(tcp->data);
	uint8_t *url, *p, *params, *name, *value;
	uint8_t *buf_ptr;

	buf_ptr=buf;

	if(!len) return;


	if( (memcmp(req, "GET ", 4) == 0)  &&	((p = strchr(req + 4, ' ')) != 0) )
	{
		url = req + 4;
		*p = 0;

		if((params = strchr(url, '?')))
			*(params++) = 0;

		if(strcmp(url,"/") == 0) 		buf_ptr = html_root_page(buf_ptr);

		else if(strcmp(url,"/ether") == 0)
		{
			while(params)
			{
				if((p = strchr(params, '&')))	*(p++) = 0;
				name = params;
				if((value = strchr(name, '=')))	*(value++) = 0;
				if( (strcmp(name, "l_ip") == 0 ) && value )	lan_config.ip_address=str2ip(value);
				if( (strcmp(name, "mask") == 0 ) && value )	lan_config.ip_mask=str2ip(value);
				if( (strcmp(name, "ip_gw") == 0 ) && value )	lan_config.ip_gateway=str2ip(value);
				if( (strcmp(name, "l_udp0") == 0 ) && value )	lan_config.loc_udp_port0=htons( str_to_uint16(value) );
				if( (strcmp(name, "l_udp1") == 0 ) && value )	lan_config.loc_udp_port1=htons( str_to_uint16(value) );
				if( (strcmp(name, "r_ip") == 0 ) && value )	lan_config.rem_ip_addr=str2ip(value);
				if( (strcmp(name, "r_udp0") == 0 ) && value )	lan_config.rem_udp_port0=htons( str_to_uint16(value) );
				if( (strcmp(name, "r_udp1") == 0 ) && value )	lan_config.rem_udp_port1=htons( str_to_uint16(value) );
				if( (strcmp(name, "s_baud") == 0 ) && value )	serial_speed=str_to_uint16(value);
				params = p;
			}
			buf_ptr=html_ether_page(buf_ptr);
		}

		else if(strcmp(url,"/serial") == 0)
		{
			while(params)
			{
				if((p = strchr(params, '&')))	*(p++) = 0;
				name = params;
				if((value = strchr(name, '=')))	*(value++) = 0;
				if( (strcmp(name, "s_baud") == 0 ) && value )	serial_speed=str_to_uint16(value);
				if( (strcmp(name, "n_kp0") == 0 ) && value )	granit_n_kp0=str_to_uint16(value);
				if( (strcmp(name, "n_kp1") == 0 ) && value )	granit_n_kp1=str_to_uint16(value);
				params = p;
			}
			buf_ptr=html_serial_page(buf_ptr);
		}
		
		else if(strcmp(url, "/l_ip") == 0) buf_ptr=html_set_form(buf_ptr, 0);
		else if(strcmp(url, "/mask") == 0) buf_ptr=html_set_form(buf_ptr, 1);
		else if(strcmp(url, "/ip_gw") == 0) buf_ptr=html_set_form(buf_ptr, 2);
		else if(strcmp(url, "/l_udp0") == 0) buf_ptr=html_set_form(buf_ptr, 3);
		else if(strcmp(url, "/r_ip") == 0) buf_ptr=html_set_form(buf_ptr, 4);
		else if(strcmp(url, "/r_udp0") == 0) buf_ptr=html_set_form(buf_ptr, 5);
		else if(strcmp(url, "/s_baud") == 0) buf_ptr=html_set_form(buf_ptr, 6);
		else if(strcmp(url, "/n_kp0") == 0) buf_ptr=html_set_form(buf_ptr, 7);
		else if(strcmp(url, "/n_kp1") == 0) buf_ptr=html_set_form(buf_ptr, 8);
		else if(strcmp(url, "/l_udp1") == 0) buf_ptr=html_set_form(buf_ptr, 9);
		else if(strcmp(url, "/r_udp1") == 0) buf_ptr=html_set_form(buf_ptr, 10);


		else if(strcmp(url, "/save") == 0)
		{
			fill_buf(&buf_ptr, http_200_header);
			fill_buf(&buf_ptr,"<pre>Rebooting (~ 5 sec) \r\n\r\n<a href='/'>Home page</a></pre>\r\n");
			tcp_send(id, frame, buf_ptr-buf, TCP_OPTION_CLOSE);
			save_config();
		}

		else		fill_buf(&buf_ptr, http_404_reply);
	}

	tcp_send(id, frame, buf_ptr-buf, TCP_OPTION_CLOSE);
}



/******************************************************************************/
uint8_t *ip2str(uint32_t ipaddr, uint8_t *buffer)
{
	for(uint8_t i=0; i<3; i++)
	{
		buffer=Uint32ToStr( (ipaddr &  0x000000FF), buffer);
		*buffer = '.';	buffer++;
		ipaddr = ipaddr >> 8;
	}
	buffer=Uint32ToStr( (ipaddr &  0x000000FF), buffer);

	return buffer;
}


/******************************************************************************/
uint32_t str2ip(uint8_t* str)
{
	uint8_t	tmp_str[4], i=0 ,x=0;
	uint32_t		address=0, adr_b;

	while(*str)
	{
		if (i>3) return 0;						//error
		tmp_str[i]=*str++;
		i++;
		if(*str=='.')
		{
				tmp_str[i]=0;
				adr_b=str_to_uint16(tmp_str);
				address |= adr_b << (x<<3);
				x++;		i=0;
		}
		if (*str==0)
		{
				tmp_str[i]=0;
				adr_b=str_to_uint16(tmp_str);
				address |= adr_b << (x<<3);
		}
	}
	return address;
}

/******************************************************************************/
void save_config(void)
{
	uint32_t		address;

	address = last_page;

	flash_unlock();
	flash_erase_page(address);

	flash_program_word(address,lan_config.ip_address);
	address += sizeof(lan_config.ip_address);
	flash_program_word(address,lan_config.ip_gateway);
	address += sizeof(lan_config.ip_gateway);
	flash_program_word(address,lan_config.ip_mask);
	address += sizeof(lan_config.ip_mask);
	flash_program_word(address,lan_config.rem_ip_addr);
	address += sizeof(lan_config.rem_ip_addr);

	flash_program_half_word(address,lan_config.loc_udp_port0);
	address += sizeof(lan_config.loc_udp_port0);
	flash_program_half_word(address,lan_config.loc_udp_port1);
	address += sizeof(lan_config.loc_udp_port1);
	flash_program_half_word(address,lan_config.rem_udp_port0);
	address += sizeof(lan_config.rem_udp_port0);
	flash_program_half_word(address,lan_config.rem_udp_port1);
	address += sizeof(lan_config.rem_udp_port1);
	flash_program_half_word(address, serial_speed);
	address += sizeof(serial_speed);
	flash_program_half_word(address, granit_n_kp0);
	address += sizeof(granit_n_kp0);
	flash_program_half_word(address, granit_n_kp1);


	while(1) {};
}

