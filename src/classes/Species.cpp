#include "classes/Species.h"

Species::Species() : speciesType(SpeciesType::MOSS), spriteTable(nullptr), spriteCount(0) {};
Species::Species(const SpeciesType type, const Sprite* const sprites[], uint8_t count)
	: speciesType(type),
	  spriteTable(sprites),
	  spriteCount(count) {}

const Sprite& Species::getCharacterSprite(uint8_t spriteLevel) const {
	if (spriteCount == 0 || spriteLevel >= spriteCount) {
		spriteLevel = 0;
	}
	return *(spriteTable[spriteLevel]);
}
