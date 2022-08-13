/*
 * LEDMatrixDriver.h
 *
 *  Created on: 30.03.2017
 *      Author: Bartosz Bielawski
 * 		Modified: Sean Farrelly (sean.farrelly@outlook.com)
 */

/*
 * This is a driver for MAX7219 based LED matrix displays.
 * It was written to replace the one already available as a library in Arduino repository.
 *
 * The differences are:
 * 	* This driver uses hardware SPI which makes it much faster (with exception of soft SS)
 * 	* Display or displayRow() has to be used to refresh the display
 * 	* up to 255 segments are supported.
 * 	* can use an external memory or self-allocated buffer
 */

#ifndef LEDMATRIXDRIVER_H
#define LEDMATRIXDRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <functional>


// typedef callback_t std::function<int(uint8_t *buf, size_t len)>;


/**
 * @brief LED Matrix Driver module
 */
class LEDMatrixDriver
{

public:

	enum class ScrollDirection 
	{
		SCROLL_UP = 0,
		SCROLL_DOWN,
		SCROLL_LEFT,
		SCROLL_RIGHT
	};

	const static uint8_t INVERT_SEGMENT_X = 1 << 0;
	const static uint8_t INVERT_DISPLAY_X = 1 << 1;
	const static uint8_t INVERT_Y 		  = 1 << 2;

	/**
	 * @brief Construct a new LEDMatrixDriver object
	 * 
	 * @param N The number of cascaded matrix modules
	 * @param flags Configuration flags to determine orientation of modules
	 * @param frameBuffer Pointer to pre-allocated frame buffer (optional)
	 */
	LEDMatrixDriver(uint8_t N, uint8_t flags=0, uint8_t* frameBuffer=nullptr);
	~LEDMatrixDriver();

	/* We don't want to copy the object */
	LEDMatrixDriver(const LEDMatrixDriver& other) = delete;
	LEDMatrixDriver(LEDMatrixDriver&& other) = delete;
	LEDMatrixDriver& operator=(const LEDMatrixDriver& other) = delete;

	/**
	 * @brief Enable all matrix modules
	 */
	void enable(void);

	/**
	 * @brief Disable all matrix modules
	 */
	void disable(void);

	/**
	 * @brief Clear the frame buffer
	 */
	void clear() { memset(frameBuffer, 0, 8 * N); }

	/**
	 * @brief Flush the entire frame buffer to the display
	 */
	void display();

	/**
	 * @brief Flush a single to row to the display
	 * @param row The row which to flush
	 */
	void displayRow(uint8_t row) { _displayRow(row); }

	/**
	 * @brief Set the brightness intensity of all matrix modules
	 * 		  Intensity levels will be snapped to bounds.
	 * @param level (0-15)
	 */
	void setIntensity(uint8_t level);

	/**
	 * @brief Set the state of an individual pixel
	 * 
	 * @param x x coordinate of pixel
	 * @param y y coordinate of pixel
	 * @param enabled whether pixel is enabled or not
	 */
	void setPixel(int16_t x, int16_t y, bool enabled);
	
	/**
	 * @brief Get the state of an individual pixel
	 * 
	 * @param x x coordinate of pixel
	 * @param y y coordinate of pixel
	 * @return true pixel is ON
	 * @return false pixel is OFF
	 */
	bool getPixel(int16_t x, int16_t y) const;

	//sets pixels in the column acording to value (LSB => y=0)
	void setColumn(int16_t x, uint8_t value);

	uint8_t getSegments() const { return N; }

	uint8_t* getFrameBuffer() const { return frameBuffer; }

	/**
	 * @brief Scroll the current framebuffer 1 pixel in the given direction
	 * 
	 * @param direction Direction which to scroll the frame buffer
	 * @param wrap Enable wrap-around of the scroll
	 */
	void scroll(ScrollDirection direction, bool wrap=false);

private:
	uint8_t* _getBufferPtr(int16_t x, int16_t y) const;
	void _sendCommand(uint16_t command);
	void _displayRow(uint8_t row);

	const uint8_t N;
	uint8_t flags;
	uint8_t* frameBuffer;
	bool selfAllocated;
};

#endif /* LEDMATRIXDRIVER_H */
