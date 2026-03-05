#pragma once
#include "Species.h"

class Game {
	private:
		int time;
		int timeMax;
		Species moss;
		Species midnight;
		Species carmine;
		Species razzmic;

		Sprite titleSprite;

		static Game* instance;
		Game();

	public:
		static Game& getInstance();

		int getTime() const { return time; }
		bool isNight() const;      
		void incrementTime();
		Species& getSpecies(SpeciesType);
		const Sprite& getTitleSprite() const;
};
