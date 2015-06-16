#ifndef UTILS
#define UTILS

#pragma once

#include "basic_drawing.h"
#include "point.h"

#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <vector>
#include <cmath>

namespace Utils
{
enum Colors
{
    Blue = 0,
    Cyan,
    Green,
    Magenta,
    Red,
    White,
    Yellow
};

void print(const std::string& s, Utils::Colors color = Utils::White);

void println(const std::string& s, Utils::Colors color = Utils::White);

double deg2rad(double d);

double rad2deg(double d);

std::string to_string(float number);

}

#endif
