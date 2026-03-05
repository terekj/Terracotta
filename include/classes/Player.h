#pragma once
#include "Species.h"

class Player {
private:
	static Player* instance;

	Species species;
	char level;
	char levelProgress;
	char levelGoal;
	float hungerProgress;
	float hungerGoal;
	float sleepProgress;
	float sleepGoal;

	Player();

	void setLevel(char);
	void setLevelProgress(char);
	void setLevelGoal(char);
	void setHungerProgress(float);
	void setHungerGoal(float);
	void setSleepProgress(float);
	void setSleepGoal(float);

public:
	static Player& getInstance();
	Species& getSpecies();
	
	char getLevel() const { return level; }
	char getLevelProgress() const { return levelProgress; }
	char getLevelGoal() const { return levelGoal; }
	float getHungerProgress() const { return hungerProgress; }
	float getHungerGoal() const { return hungerGoal; }
	float getSleepProgress() const { return sleepProgress; }
	float getSleepGoal() const { return sleepGoal; }

	bool isFull() const;
	bool isFullyAwake() const;

	void feed(char amount);
	void sleep(char amount);
	void gainExp(char amount);

	void decrementHunger(char amount);
	void decrementSleep(char amount);
};
