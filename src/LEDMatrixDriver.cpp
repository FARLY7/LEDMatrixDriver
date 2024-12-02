/*
 * LEDMatrixDriver.cpp
 *
 *  Created on: 30.03.2017
 *      Author: Bartosz Bielawski
 * 		Modified: Sean Farrelly
 */
#include "LEDMatrixDriver.hpp"

/* MAX7219/MAX7221 commands, as defined in the datasheet */
const static uint8_t ENABLE =		0x0C;
const static uint8_t TEST =	 		0x0F;
const static uint8_t INTENSITY =	0x0A;
const static uint8_t SCAN_LIMIT =	0x0B;
const static uint8_t DECODE =		0x09;

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
LEDMatrixDriver::LEDMatrixDriver(uint8_t N, spi_transfer_t spi_transfer, uint8_t flags, uint8_t* fb):
	N(N),
	flags(flags),
	frameBuffer(fb),
	selfAllocated(fb == nullptr),
	spi_transfer(spi_transfer)
{
	if (selfAllocated)
		frameBuffer = new uint8_t[N*8];

	clear();	// initally clear the buffer as the memory will not be initialized on reset (old content will be in memory yet)
	disable();
	setIntensity(0);
	_sendCommand(TEST, 0);		// No test
	_sendCommand(DECODE, 0);	// No decode
	_sendCommand(SCAN_LIMIT, 7);// All lines
	display();
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
	_sendCommand(ENABLE, 1);
}

/**
 * @brief Disable all matrix modules
 */
void LEDMatrixDriver::disable()
{
	_sendCommand(ENABLE, 0);
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
	_sendCommand(INTENSITY, level);
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

void LEDMatrixDriver::_sendCommand(uint8_t reg, uint8_t data)
{
	for (uint8_t i = 0 ; i < MAX_LED_MATRIX_MODULES ; ++i)
	{
		this->cmd_buffer[(i*2)] 	= reg; 
		this->cmd_buffer[(i*2) + 1] = data; 
	} 

	this->spi_transfer(this->cmd_buffer, MAX_LED_MATRIX_MODULES * 2);
}


void transpose8(uint8_t A[8], int m, int n, 
                uint8_t B[8]) {
   unsigned x, y, t; 

   // Load the array and pack it into x and y. 

   x = (A[0]<<24)   | (A[m]<<16)   | (A[2*m]<<8) | A[3*m]; 
   y = (A[4*m]<<24) | (A[5*m]<<16) | (A[6*m]<<8) | A[7*m]; 

   t = (x ^ (x >> 7)) & 0x00AA00AA;  x = x ^ t ^ (t << 7); 
   t = (y ^ (y >> 7)) & 0x00AA00AA;  y = y ^ t ^ (t << 7); 

   t = (x ^ (x >>14)) & 0x0000CCCC;  x = x ^ t ^ (t <<14); 
   t = (y ^ (y >>14)) & 0x0000CCCC;  y = y ^ t ^ (t <<14); 

   t = (x & 0xF0F0F0F0) | ((y >> 4) & 0x0F0F0F0F); 
   y = ((x << 4) & 0xF0F0F0F0) | (y & 0x0F0F0F0F); 
   x = t; 

   B[0]=x>>24;    B[n]=x>>16;    B[2*n]=x>>8;  B[3*n]=x; 
   B[4*n]=y>>24;  B[5*n]=y>>16;  B[6*n]=y>>8;  B[7*n]=y; 
}

void LEDMatrixDriver::_displayRow(uint8_t row)
{
	/* Calculates row address based on flags */
	uint8_t address_row = flags & INVERT_Y ? 7 - row: row;

	bool display_x_inverted = flags & INVERT_DISPLAY_X;
	bool segment_x_inverted = flags & INVERT_SEGMENT_X;

	/* For x inverted display change iterating order
	   inverting segments may still be needed! */
	int16_t from = display_x_inverted ? N-1 : 0; /* Start from ... */
	int16_t to =   display_x_inverted ? -1  : N; /* Where to stop */
	int16_t step = display_x_inverted ? -1  : 1; /* Directon */



	uint8_t block[8];
	uint8_t block_tran[8];
	uint8_t bytes_out[8*4];
	if(flags & ROTATE_SEGMENT)
	{
		memset(block, 0, sizeof(block));
		memset(block_tran, 0, sizeof(block_tran));
		memset(bytes_out, 0, sizeof(bytes_out));

		for(int i = 0 ; i < 4; i++)
		{
			/* Copy into 8x8 block */
			for(int j = 0 ; j < 32 ; j += 4)
			{
				block[j/4] = frameBuffer[j + i];
			}

			transpose8(block, 1, 1, block_tran);

			/* Copy from transposed 8x8 block */
			for(int j = 0 ; j < 32 ; j += 4)
			{
				bytes_out[j + i] = block_tran[j/4];
			}
		}
	}

	uint8_t i = 0;
	uint8_t data;
	for (int16_t d = from; d != to; d += step)
	{
		if(flags & ROTATE_SEGMENT) {
			data = bytes_out[d + row*N];
		} else {
			data = frameBuffer[d + row*N];
		}

		if (segment_x_inverted) {
			reverse(data);
		}

		uint8_t reg = address_row + 1;		
		this->cmd_buffer[(2*i)] 	= reg;
		this->cmd_buffer[(2*i) + 1] = data;
		i++;
	}

	this->spi_transfer(this->cmd_buffer, i*2);
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