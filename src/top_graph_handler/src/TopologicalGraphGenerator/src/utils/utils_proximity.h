#ifndef UTILS_PROX
#define UTILS_PROX

#pragma once

#include "utils.h"

#include <map>
#include <vector>

#define MIN_INTIMATE_SPACE 0.f
#define MAX_INTIMATE_SPACE 0.45f
#define MAX_PERSONAL_SPACE 1.2f
#define MAX_SOCIAL_SPACE 3.6f
#define MAX_COLLECTIVE_SPACE 8.0f

#define MIN_RIGHT_ANGLE -1.57f
#define MIN_MIDDLE_ANGLE -0.52f //30
#define MIN_LEFT_ANGLE 0.52f
#define MAX_LEFT_ANGLE 1.57f

namespace ProxUtils
{
enum Space
{
    intimate = 1,
    personal,
    social,
    collective
};

enum Approach
{
    right = 1,
    left,
    middle
};

float proxemics_param(ProxUtils::Space p_space, std::string bound);
float approach_param(ProxUtils::Approach approach, std::string bound);

}

static std::map< ProxUtils::Space, std::vector<float> > initProxParam()
{
    std::map< ProxUtils::Space, std::vector<float> > param;

    // intimate
    std::vector<float> tmp; tmp.push_back(MIN_INTIMATE_SPACE); tmp.push_back(MAX_INTIMATE_SPACE);
    param.insert(std::make_pair( ProxUtils::intimate, tmp) );

    // personal
    tmp.clear(); tmp.push_back(MAX_INTIMATE_SPACE); tmp.push_back(MAX_PERSONAL_SPACE);
    param.insert(std::make_pair( ProxUtils::personal, tmp) );

    // social
    tmp.clear(); tmp.push_back(MAX_PERSONAL_SPACE); tmp.push_back(MAX_SOCIAL_SPACE);
    param.insert(std::make_pair( ProxUtils::social, tmp) );

    // collective
    tmp.clear(); tmp.push_back(MAX_SOCIAL_SPACE); tmp.push_back(MAX_COLLECTIVE_SPACE);
    param.insert(std::make_pair( ProxUtils::collective, tmp) );

    return param;
}

static std::map< ProxUtils::Approach, std::vector<float> > initApprParam()
{
    std::map< ProxUtils::Approach, std::vector<float> > param;

    // left
    std::vector<float> tmp; tmp.push_back(MIN_LEFT_ANGLE); tmp.push_back(MAX_LEFT_ANGLE);
    param.insert(std::make_pair( ProxUtils::left, tmp) );

    // middle
    tmp.clear(); tmp.push_back(MIN_MIDDLE_ANGLE); tmp.push_back(MIN_LEFT_ANGLE);
    param.insert(std::make_pair( ProxUtils::middle, tmp) );

    // right
    tmp.clear(); tmp.push_back(MIN_RIGHT_ANGLE); tmp.push_back(MIN_MIDDLE_ANGLE);
    param.insert(std::make_pair( ProxUtils::right, tmp) );

    return param;
}

static std::map< ProxUtils::Space, std::vector<float> > prox_param( initProxParam() );
static std::map< ProxUtils::Approach, std::vector<float> > appr_param( initApprParam() );

#endif
