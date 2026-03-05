#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "utils/lcdUtils.h"
#include "utils/spiAVR.h"
#include "utils/timerISR.h"
#include "utils/serialATmega.h"

#include "classes/Application.h"
#include "classes/Game.h"
#include "classes/Player.h"
#include "classes/Species.h"
#include "classes/Sprite.h"

#define COLOR_HUNGER 0xFF00
#define COLOR_SLEEP 0x04df
#define COLOR_LEVEL 0x05e3
inline void playNote(uint16_t freq) {
    uint16_t ocr = (16000000UL / (128.0 * freq)) - 1;
    OCR1A = ocr;
    TCCR1A |= (1 << COM1A0);   // toggle mode = sound
}

inline void stopNote() {
    TCCR1A &= ~(1 << COM1A0);  // disconnect OC1A
    PORTB &= ~(1 << PB1);      // force pin LOW (silence)
}

#define NUM_TASKS 8

typedef struct _task {
    signed char state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*TickFct)(int);
} task;

const unsigned long T1_PERIOD = 200;
const unsigned long T2_PERIOD = 1;
const unsigned long T3_PERIOD = 100;
const unsigned long T4_PERIOD = 1;
const unsigned long T5_PERIOD = 1;
const unsigned long T6_PERIOD = 1;
const unsigned long T7_PERIOD = 100;
const unsigned long T8_PERIOD = 100;

const unsigned long GCD_PERIOD = 1;

task tasks[NUM_TASKS];

void TimerISR() {
    for (unsigned int i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].elapsedTime >= tasks[i].period) {
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += GCD_PERIOD;
    }
}

// ENUMS
enum T1_States {T1_Init, T1_Limbo, T1_NoLimbo};
enum T2_States {T2_ReadBtn};
enum T3_States {T3_DaylightCycle};
enum T4_States {T4_Init, T4_Press, T4_Feed};
enum T5_States {T5_Init, T5_Press, T5_LevelUp};
enum T6_States {T6_Init, T6_Press, T6_Sleep};
enum T7_States {T7_RefreshScreen};
enum T8_States {T8_Init, T8_p1, T8_p2, T8_p3, T8_p4, T8_p5, T8_p6, T8_p7, T8_p8};

// GLOBALS
bool limbo;
bool leftbtn;
bool rightbtn;
bool midbtn;
bool jingle;


// ================= TASK FUNCTIONS ====================

int T1_TickFct(int state) {
    Application& app = Application::getInstance();
    Game& game = Game::getInstance();

    switch (state) {
        case T1_Init:
            limbo = 1;
            jingle = 1;
            app.drawSprite(game.getTitleSprite());
            state = T1_Limbo;
            break;
        case T1_Limbo:
            if (leftbtn || rightbtn || midbtn) {
                limbo = 0;
                state = T1_NoLimbo;
                fillColor(0xFFFF);
            }
            break;

        case T1_NoLimbo:
            break;
    }
    return state;
}

int T2_TickFct(int state) {
    leftbtn  = ((PINC >> PC2) & 0x01);
    midbtn   = ((PINC >> PC1) & 0x01);
    rightbtn = ((PINC >> PC0) & 0x01);

    return state;
}

int T3_TickFct(int state) {
    Game& game = Game::getInstance();
    Player& player = Player::getInstance();

    game.incrementTime();

    if ((game.getTime() % 4) == 0) {
         player.decrementHunger(5);
        player.decrementSleep(5);
    }
    return state;
}

int T4_TickFct(int state) {
    Application& app = Application::getInstance();
    Player& player = app.getPlayer();
    switch (state) {
        case T4_Init:
            if (leftbtn && !limbo) state = T4_Press;
            break;

        case T4_Press:
            if (!leftbtn) state = T4_Feed;
            else if (leftbtn) state = T4_Press;
            break;

        case T4_Feed:
            serial_println("yum!");
            player.getInstance().feed(10);
            state = T4_Init;
            break;
    }
    return state;
}

int T5_TickFct(int state) {
    switch (state) {
        case T5_Init:
            if (midbtn && !limbo) state = T5_Press;
            break;

        case T5_Press:
            if (!midbtn) state = T5_LevelUp;
            break;

        case T5_LevelUp: {
            Application& app = Application::getInstance();
            Player& player = app.getPlayer();
            if (player.isFull()) {
                playNote(888);
                playNote(1042);
                playNote(888);
                playNote(1042);
                player.gainExp(1);
            }
            state = T5_Init;
            break;
        }
    }
    return state;
}

int T6_TickFct(int state) {
    Application& app = Application::getInstance();

    switch (state) {
        case T6_Init:
            if (rightbtn && !limbo) state = T6_Press;
            break;

        case T6_Press:
            if (!rightbtn && app.getGame().isNight()) state = T6_Sleep;
            else if (!rightbtn) state = T6_Init;
            break;

        case T6_Sleep:
            playNote(889);
            playNote(1046);
            app.getPlayer().sleep(app.getPlayer().getSleepGoal() - app.getPlayer().getSleepProgress());
            state = T6_Init;
            break;
    }
    return state;
}

int T7_TickFct(int state) {
    if (limbo) { return state; }
    Application& app = Application::getInstance();
    Game& game = app.getGame();

    switch (state) {
        case T7_RefreshScreen:
            if (!limbo) {
                Player& player = app.getPlayer();
                if (game.isNight()) {
                    fillRect(0xf005, 0, 120, 7, 127);
                } else {
                    fillRect(0xFFFF, 0, 120, 7, 127);
                }
                app.drawSprite(player.getSpecies().getCharacterSprite(player.getLevel()));
                
                uint8_t hungerBar = floor(127.0 * ((float)player.getHungerProgress() / player.getHungerGoal()));
                fillRect(COLOR_WHITE, 0, 0, 127, 17);
                fillRect(COLOR_HUNGER, 0, 0, hungerBar, 5);

                uint8_t sleepBar = floor(127.0 * ((float)player.getSleepProgress() / player.getSleepGoal()));
                fillRect(COLOR_SLEEP, 0, 6, sleepBar, 11);

                uint8_t levelBar = floor(127.0 * ((float)player.getLevelProgress() / player.getLevelGoal()));
                fillRect(COLOR_LEVEL, 0, 12, levelBar, 17);
            }
    }
    return state;
}


// ===================== JINGLE USING OC1A =====================

int T8_TickFct(int state) {
    switch (state) {
        case T8_Init:
            if (jingle) {
                state = T8_p1;
                break;
            }
            stopNote();
            break;
        case T8_p1:
            playNote(932);
            state = T8_p2;
            break;
        case T8_p2: 
            playNote(1046);
            state = T8_p3;
            break;
        case T8_p3:
            stopNote();
            state = T8_p4;
            break;
        case T8_p4:
            playNote(880);
            state = T8_p5;
            break;
        case T8_p5:
            playNote(932);
            state = T8_p6;
            break;
        case T8_p6:
            playNote(1046);
            state = T8_p7;
            break;
        case T8_p7:
            stopNote();
            state = T8_p8;
            break;
        case T8_p8:
            playNote(932);
            jingle = 0;
            state = T8_Init;
            break;
    }

    return state;
}


// ======================= MAIN ==========================

int main() {

    SPI_INIT();
    lcd_init();
    serial_init(9600);

    // Buttons: PC0, PC1, PC2
    DDRC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2));
    PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC2);

    PORTB &= ~(1 << PB1);
    // =====================
    // TIMER1 → OC1A (PB1)
    // =====================
    // TIMER1 → OC1A (PB1)
    DDRB |= (1 << PB1);
    PORTB &= ~(1 << PB1);

    // CTC mode, TOP = OCR1A
    TCCR1A = 0;
    TCCR1B = (1 << WGM12);

    // Prescaler = 64
    TCCR1B |= (1 << CS11) | (1 << CS10);

    OCR1A = 0;  // no tone initially
    
    fillColor(0xFFFF);
    // Pre-fetch singletons once
    
    tasks[0].period = T1_PERIOD;
	tasks[0].state = T1_Init;
	tasks[0].elapsedTime = T1_PERIOD;
	tasks[0].TickFct = &T1_TickFct;
    
    tasks[1].period = T2_PERIOD;
	tasks[1].state = T2_ReadBtn;
	tasks[1].elapsedTime = T2_PERIOD;
	tasks[1].TickFct = &T2_TickFct;
    
    tasks[2].period = T3_PERIOD;
	tasks[2].state = T3_DaylightCycle;
	tasks[2].elapsedTime = T3_PERIOD;
	tasks[2].TickFct = &T3_TickFct;
    
    tasks[3].period = T4_PERIOD;
	tasks[3].state = T4_Init;
	tasks[3].elapsedTime = T4_PERIOD;
	tasks[3].TickFct = &T4_TickFct;
    
    tasks[4].period = T5_PERIOD;
	tasks[4].state = T5_Init;
	tasks[4].elapsedTime = T5_PERIOD;
	tasks[4].TickFct = &T5_TickFct;
    
    tasks[5].period = T6_PERIOD;
	tasks[5].state = T6_Init;
	tasks[5].elapsedTime = T6_PERIOD;
	tasks[5].TickFct = &T6_TickFct;
    
    tasks[6].period = T7_PERIOD;
	tasks[6].state = T7_RefreshScreen;
	tasks[6].elapsedTime = T7_PERIOD;
	tasks[6].TickFct = &T7_TickFct;
    
    tasks[7].period = T8_PERIOD;
	tasks[7].state = T8_Init;
	tasks[7].elapsedTime = T8_PERIOD;
	tasks[7].TickFct = &T8_TickFct;

    TimerSet(GCD_PERIOD);
    TimerOn();

    while (1) {}

    return 0;
}
