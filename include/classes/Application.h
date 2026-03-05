#pragma once

#include <stdint.h>

class Game;
class Player;
class Sprite;

class Application {
private:
    static Application* instance;

    Game&   game;
    Player& player;

    Application();  // private constructor

public:
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    static Application& getInstance();

    Game&   getGame()   { return game; }
    Player& getPlayer() { return player; }

    void drawSprite(const Sprite& sprite);
};
