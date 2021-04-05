#pragma once
#include "main.h"

typedef void(*chicken)();

void debug();

void bL();

void bR();

void rL();

void rR();

void skill();

chicken bruh[6] = {debug, rL, rR, bL, bR, skill};;
