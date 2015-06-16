#include "utils.h"

static std::map<Utils::Colors,std::string> initColorsTable()
{
    std::map<Utils::Colors, std::string> table;

    table.insert(std::make_pair(Utils::Blue,"\033[22;34;1m"));
    table.insert(std::make_pair(Utils::Cyan,"\033[22;36;1m"));
    table.insert(std::make_pair(Utils::Green,"\033[0;32;1m"));
    table.insert(std::make_pair(Utils::Magenta,"\033[22;35;1m"));
    table.insert(std::make_pair(Utils::Red,"\033[22;31;1m"));
    table.insert(std::make_pair(Utils::White,"\033[22;37;1m"));
    table.insert(std::make_pair(Utils::Yellow,"\033[22;33;1m"));

    return table;
}

static std::map<Utils::Colors, std::string> colorsTable(initColorsTable());

void Utils::print(const std::string& s, Utils::Colors color)
{
    const std::map<Utils::Colors,std::string>::const_iterator& entryColorsTable = colorsTable.find(color);

    if (entryColorsTable != colorsTable.end()) std::cerr << entryColorsTable->second << s << "\033[0m" << std::flush;
    else std::cerr << s << std::flush;
}

void Utils::println(const std::string& s, Utils::Colors color)
{
    print(s,color);

    std::cerr << std::endl;
}

double Utils::deg2rad(double d)
{
    return (d * M_PI) / 180.0;
}

double Utils::rad2deg(double d)
{
    return (d * 180.0) / M_PI;
}

std::string Utils::to_string(float number)
{
   std::stringstream ss;
   ss << number;
   return ss.str();
}
