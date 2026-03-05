#pragma once
#include <stdint.h>
#include "Sprite.h"

enum class SpeciesType {
	MOSS,
	MIDNIGHT,
	CARMINE,
	RAZZMIC
};

class Species {
	private:
		const SpeciesType speciesType;

		const Sprite* const* spriteTable;
		uint8_t spriteCount;

	public:
		Species();
		Species(const SpeciesType, const Sprite* const sprites[], uint8_t);

		SpeciesType getSpeciesType() const { return speciesType;};
		const Sprite& getCharacterSprite(uint8_t spriteLevel) const;
};
