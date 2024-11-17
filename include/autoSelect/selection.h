#pragma once

#include <string>

//selector configuration
#define HUE 320
#define DEFAULT 0
#define AUTONS "Left", "Right", "Do Nothing"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
