/*! \file bitbuf.h \brief Multipurpose bit buffer structure and methods. */
//*****************************************************************************
//
// File Name	: 'bitbuf.c'
// Title		: Multipurpose bit buffer structure and methods
// Author		: Pascal Stang - Copyright (C) 2001-2002
// Created		: 7/10/2002
// Revised		: 7/10/2002
// Version		: 0.5
// Target MCU	: any
// Editor Tabs	: 4
//
///	\ingroup general
/// \defgroup bitbuf Generic Bit-Buffer Structure and Function Library (bitbuf.c)
/// \code #include "bitbuf.h" \endcode
/// \par Overview
///		This bit-buffer structure provides an easy and efficient way to store and
///		process bits. You can create as many bit buffers as you like (within
///		memory limits), and then use this common set of functions to access each
///		buffer.	Supported functions include sequential getting and storing of
///		bits, array-like get, buffer flush (dump data), and reset-to-beginning.
///		This buffer is not dynamically allocated, it has a user-defined fixed 
///		maximum size.
///
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************
//@{

#ifndef BITBUF_H
#define BITBUF_H

#include <stdint.h>
// structure/typdefs

// the BitBuffer structure
typedef struct struct_BitBuf
{
	uint8_t *dataptr;			// the physical memory address where the buffer is stored
	uint8_t	size;			// the allocated byte size of the buffer
	uint8_t bytePos;			// current byte position
	uint8_t bitPos;			// current bit position
	uint16_t datalength;		// the length of the data (in bits) currently in the buffer
} BitBuf;

// function prototypes

//! initialize a buffer to start at a given address and have given size
void bitbufInit(BitBuf* bitBuffer, uint8_t *start, uint8_t bytesize);

//! get the bit at the current position in the buffer
uint8_t bitbufGet(BitBuf* bitBuffer);

//! get a bit at the specified index in the buffer (kind of like array access)
// ** note: this does not remove/delete the bit that was read
uint8_t bitbufGetAtIndex(BitBuf* bitBuffer, uint16_t bitIndex);

//! store a bit at the current position in the buffer
void bitbufStore(BitBuf* bitBuffer, uint8_t bit);

//! return the number of bits in the buffer
uint16_t bitbufGetDataLength(BitBuf* bitBuffer);

// check if the buffer is full/not full (returns non-zero value if not full)
//uint8_t  bitbufIsNotFull(cBuffer* buffer);

//! resets the read/write position of the buffer to beginning
void bitbufReset(BitBuf* bitBuffer);

//! flush (clear) the contents of the buffer
void bitbufFlush(BitBuf* bitBuffer);

#endif
//@}
