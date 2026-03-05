#include "classes/Game.h"
#include "classes/Sprite.h"

#include "title.h"
#include "sprite_egg_moss.h"
#include "sprite_egg_midnight.h"
#include "sprite_egg_carmine.h"
#include "sprite_egg_razzmic.h"
#include "sprite_flame.h"
#include "sprite_gecko_moss.h"
#include "sprite_gecko_midnight.h"
#include "sprite_gecko_carmine.h"
#include "sprite_gecko_razzmic.h"
#include "sprite_dragon_moss.h"
#include "sprite_dragon_midnight.h"
#include "sprite_dragon_carmine.h"
#include "sprite_dragon_razzmic.h"

Sprite title(sprite_title_bitmap, sprite_title_colormap);
Sprite flame(sprite_flame_bitmap, sprite_flame_colormap);

// MOSS SPRITES
Sprite egg_moss(sprite_egg_moss_bitmap, sprite_egg_moss_colormap);
Sprite gecko_moss(sprite_gecko_moss_bitmap, sprite_gecko_moss_colormap);
Sprite dragon_moss(sprite_dragon_moss_bitmap, sprite_dragon_moss_colormap);

const Sprite* mossSprites[] {
	&egg_moss, &flame, &gecko_moss, &dragon_moss
};

// MIDNIGHT SPRITES
Sprite egg_midnight(sprite_egg_midnight_bitmap, sprite_egg_midnight_colormap);
Sprite gecko_midnight(sprite_gecko_midnight_bitmap, sprite_gecko_midnight_colormap);
Sprite dragon_midnight(sprite_dragon_midnight_bitmap, sprite_dragon_midnight_colormap);

const Sprite* midnightSprites[] {
	&egg_midnight, &flame, &gecko_midnight, &dragon_midnight
};

// CARMINE SPRITES
Sprite egg_carmine(sprite_egg_carmine_bitmap, sprite_egg_carmine_colormap);
Sprite gecko_carmine(sprite_gecko_carmine_bitmap, sprite_gecko_carmine_colormap);
Sprite dragon_carmine(sprite_dragon_carmine_bitmap, sprite_dragon_carmine_colormap);

const Sprite* carmineSprites[] {
	&egg_carmine, &flame, &gecko_carmine, &dragon_carmine
};

// RAZZMIC SPRITES
Sprite egg_razzmic(sprite_egg_razzmic_bitmap, sprite_egg_razzmic_colormap);
Sprite gecko_razzmic(sprite_gecko_razzmic_bitmap, sprite_gecko_razzmic_colormap);
Sprite dragon_razzmic(sprite_dragon_razzmic_bitmap, sprite_dragon_razzmic_colormap);

const Sprite* razzmicSprites[] {
	&egg_razzmic, &flame, &gecko_razzmic, &dragon_razzmic
};

Game* Game::instance = nullptr;
Game::Game(): time(0), timeMax(24),
titleSprite(title),
moss(SpeciesType::MOSS, mossSprites, 4),
midnight(SpeciesType::MIDNIGHT, midnightSprites, 4),
carmine(SpeciesType::CARMINE, carmineSprites, 4),
razzmic(SpeciesType::RAZZMIC, razzmicSprites, 4) {}

Game& Game::getInstance() {
	if (!instance) {
		static Game singleton;
		instance = &singleton;
	}
	return *instance;
}

bool Game::isNight() const { return (time >= 18 || time < 6); }
void Game::incrementTime() { time = (time + 1) % timeMax; }
Species& Game::getSpecies(SpeciesType type) {
	switch (type) {
		case SpeciesType::MOSS:     return moss;
		case SpeciesType::MIDNIGHT: return midnight;
		case SpeciesType::CARMINE:  return carmine;
		case SpeciesType::RAZZMIC:  return razzmic;
	}
	// fallback: return moss to avoid undefined behavior
	return moss;
}

const Sprite& Game::getTitleSprite() const {
	return titleSprite;
}