#include "classes/Player.h"
#include "classes/Game.h"

Player* Player::instance = nullptr;

Player::Player()
    : species(Game::getInstance().getSpecies(SpeciesType::MOSS)), level(0), levelProgress(0), levelGoal(4),
      hungerProgress(100.0f), hungerGoal(100.0f),
      sleepProgress(100.0f), sleepGoal(100.0f) {}

Player& Player::getInstance() {
	if (!instance) {
		static Player singleton;
		instance = &singleton;
	}
	return *instance;
}

Species& Player::getSpecies() {
    return species;
}

void Player::setLevel(char val) { level = val; }
void Player::setLevelProgress(char val) { levelProgress = val; }
void Player::setLevelGoal(char val) { levelGoal = val; }
void Player::setHungerProgress(float val) { hungerProgress = val; }
void Player::setHungerGoal(float val) { hungerGoal = val; }
void Player::setSleepProgress(float val) { sleepProgress = val; }
void Player::setSleepGoal(float val) { sleepGoal = val; }

bool Player::isFull() const {
    return (getHungerProgress() >= (getHungerGoal() * 0.5));
}
// Example logic methods
void Player::feed(char amount) {
    hungerProgress += amount;
    if (hungerProgress > hungerGoal) hungerProgress = hungerGoal;
}

void Player::sleep(char amount) {
    sleepProgress += amount;
    if (sleepProgress > sleepGoal) sleepProgress = sleepGoal;
}

void Player::gainExp(char amount) {
    levelProgress += amount;
    while (levelProgress > levelGoal) {
        levelProgress -= levelGoal;
        level++;
    }
}
void Player::decrementHunger(char amount) {
    if ((hungerProgress - amount) > 0) {
        hungerProgress -= amount;
    }
}
void Player::decrementSleep(char amount) {
    if ((sleepProgress - amount) > 0) {
        sleepProgress -= amount;
    }
}
