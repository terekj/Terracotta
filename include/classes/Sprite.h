#pragma once
#include <stdint.h>

class Sprite {
	private:
		const uint8_t* bitmap;
		const uint16_t* colormap;
	public:
		Sprite(const uint8_t* bitmap, const uint16_t* colormap);
		const uint8_t* getBitmap() const;
		const uint16_t* getColormap() const;
};