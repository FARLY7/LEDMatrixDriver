/*
 * LEDMatrixDriver.cpp
 *
 *  Created on: 30.03.2017
 *      Author: Bartosz Bielawski
 * 		Modified: Sean Farrelly
 */
#include "LEDMatrixDriver.hpp"

#include <string.h>

/* MAX7219/MAX7221 commands, as defined in the datasheet */
const static uint16_t ENABLE =		0x0C00;
const static uint16_t TEST =	 	0x0F00;
const static uint16_t INTENSITY =	0x0A00;
const static uint16_t SCAN_LIMIT =	0x0B00;
const static uint16_t DECODE =		0x0900;

/***************************************************/
/* Static functions 							   */
/***************************************************/
/**
 * @brief Reverse the bits in a byte
 * @param b Reference to byte
 */
static void reverse(uint8_t& b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
}

/***************************************************/
/* Class		 								   */
/***************************************************/
LEDMatrixDriver::LEDMatrixDriver(uint8_t N, uint8_t flags, uint8_t* fb):
	N(N),
	flags(flags),
	frameBuffer(fb),
	selfAllocated(fb == nullptr),
{
	if (selfAllocated)
		frameBuffer = new uint8_t[N*8];

	clear();	// initally clear the buffer as the memory will not be initialized on reset (old content will be in memory yet)
	disable();
	setIntensity(0);
	_sendCommand(LEDMatrixDriver::TEST);			//no test
	_sendCommand(LEDMatrixDriver::DECODE);			//no decode
	_sendCommand(LEDMatrixDriver::SCAN_LIMIT | 7);	//all lines
}

LEDMatrixDriver::~LEDMatrixDriver()
{
	if (selfAllocated)
		delete[] frameBuffer;
}

/**
 * @brief Enable all matrix modules
 */
void LEDMatrixDriver::enable()
{
	_sendCommand(ENABLE | 1);
}

/**
 * @brief Disable all matrix modules
 */
void LEDMatrixDriver::disable()
{
	_sendCommand(ENABLE | 0);
}

/**
 * @brief Flush the entire frame buffer to the display
 */
void LEDMatrixDriver::display()
{
	for (uint8_t y = 0; y < 8; y++) {
		_displayRow(y);
	}
}

/**
 * @brief Set the brightness intensity of all matrix modules
 * 		  Intensity levels will be snapped to bounds.
 * @param level (0-15)
 */
void LEDMatrixDriver::setIntensity(uint8_t level)
{
	/* Maximum intensity is 15 (0xF) */
	if (level > 15) {
		level = 15;
	}	
	_sendCommand(INTENSITY | level);
}

/**
 * @brief Set the state of an individual pixel
 * 
 * @param x x coordinate of pixel
 * @param y y coordinate of pixel
 * @param enabled whether pixel is enabled or not
 */
void LEDMatrixDriver::setPixel(int16_t x, int16_t y, bool enabled)
{
	uint8_t* p = _getBufferPtr(x, y);
	if (!p)
		return;

	uint16_t b = 7 - (x & 7);		//bit

	if (enabled)
		*p |=  (1<<b);
	else
		*p &= ~(1<<b);
}

/**
 * @brief Get the state of an individual pixel
 * 
 * @param x x coordinate of pixel
 * @param y y coordinate of pixel
 * @return true pixel is ON
 * @return false pixel is OFF
 */
bool LEDMatrixDriver::getPixel(int16_t x, int16_t y) const
{
	uint8_t* p = _getBufferPtr(x, y);
	if (!p)
		return false;

	uint16_t b = 7 - (x & 7);		//bit

	return *p & (1 << b);
}


void LEDMatrixDriver::setColumn(int16_t x, uint8_t value)
{
	//no need to check x, will be checked by setPixel
	for (uint8_t y = 0; y < 8; ++y)
	{
		setPixel(x, y, value & 1);
		value >>= 1;
	}
}

/**
 * @brief Scroll the current framebuffer 1 pixel in the given direction
 * 
 * @param direction Direction which to scroll the frame buffer
 * @param wrap Enable wrap-around of the scroll
 */
void LEDMatrixDriver::scroll(ScrollDirection direction, bool wrap)
{
	switch(direction)
	{
		case ScrollDirection::SCROLL_UP:
		{
			uint8_t tmp[N];							//space for extra row
			if (wrap)
				memcpy(tmp, frameBuffer, N);		//save the first row
			else
				memset(tmp, 0, N);					//or zero the memory
			
			memmove(frameBuffer, frameBuffer + N, 7*N);	//shift 7 rows
			memcpy(frameBuffer + (7*N), tmp, N);		//last row is zeros or copy of the first row
			break;
		}

		case ScrollDirection::SCROLL_DOWN:
		{
			uint8_t tmp[N];
			if (wrap) 
				memcpy(tmp, frameBuffer + (7*N), N);
			else
				memset(tmp, 0, N);
			
			memmove(frameBuffer+N, frameBuffer, 7*N);
			memcpy(frameBuffer, tmp, N);
			break;
		}

		case ScrollDirection::SCROLL_RIGHT:
			// Scrolling right needs to be done by bit shifting every uint8_t in the frame buffer
			// Carry is reset between rows
			for (int y = 0; y < 8; y++)
			{
				uint8_t carry = 0x00;
				for (int x = 0; x < N; x++)
				{
					uint8_t& v = frameBuffer[y*N+x];
					uint8_t newCarry = v & 1;
					v = (carry << 7) | (v >> 1);
					carry = newCarry;
				}
				if (wrap) setPixel(0, y, carry);
			}
			break;

		case ScrollDirection::SCROLL_LEFT:
			// Scrolling left needs to be done by bit shifting every uint8_t in the frame buffer
			// Carry is reset between rows
			for (int y = 0; y < 8; y++)
			{
				uint8_t carry = 0x00;
				for (int x = N-1; x >= 0; x--)
				{
					uint8_t& v = frameBuffer[y*N+x];
					uint8_t newCarry = v & 0x80;
					v = (carry >> 7) | (v << 1);
					carry = newCarry;
				}
				if (wrap) setPixel(8*N-1, y, carry);
			}
			break;
	}
}

void LEDMatrixDriver::_sendCommand(uint16_t command)
{
	/* spi_transfer_callback */

	// SPI.beginTransaction(spiSettings);
	// digitalWrite(ssPin, 0);
	// //send the same command to all segments
	// for (uint8_t i = 0; i < N; ++i)
	// {
	// 	SPI.transfer16(command);
	// }
	// digitalWrite(ssPin, 1);
	// SPI.endTransaction();
}


void LEDMatrixDriver::_displayRow(uint8_t row)
{
	//calculates row address based on flags
	uint8_t address_row = flags & INVERT_Y ? 7 - row: row;

	bool display_x_inverted = flags & INVERT_DISPLAY_X;
	bool segment_x_inverted = flags & INVERT_SEGMENT_X;

	//for x inverted display change iterating order
	//inverting segments may still be needed!
	int16_t from = display_x_inverted ? N-1 : 0;		//start from ...
	int16_t to =   display_x_inverted ? -1  : N;		//where to stop
	int16_t step = display_x_inverted ? -1  : 1;		//directon

	/* SPI Transfer Callback */

	// SPI.beginTransaction(spiSettings);
	// digitalWrite(ssPin, 0);

	// for (int16_t d = from; d != to; d += step)
	// {
	// 	uint8_t data = frameBuffer[d + row*N];
	// 	if (segment_x_inverted)
	// 		reverse(data);
	// 	uint16_t cmd = ((address_row + 1) << 8) | data;
	// 	SPI.transfer16(cmd);
	// }

	// digitalWrite(ssPin, 1);
	// SPI.endTransaction();
}

uint8_t* LEDMatrixDriver::_getBufferPtr(int16_t x, int16_t y) const
{
	if ((y < 0) or (y >= 8))
		return nullptr;
	if ((x < 0) or (x >= (8*N)))
		return nullptr;

	uint16_t B = x >> 3; // Byte

	return frameBuffer + (y*N) + B;
}