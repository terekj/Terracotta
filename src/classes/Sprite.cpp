#include "classes/Sprite.h"

Sprite::Sprite(const uint8_t* bitmap, const uint16_t* colormap)
    : bitmap(bitmap), colormap(colormap) {}

const uint8_t* Sprite::getBitmap() const {return bitmap;}
const uint16_t* Sprite::getColormap() const {return colormap;}