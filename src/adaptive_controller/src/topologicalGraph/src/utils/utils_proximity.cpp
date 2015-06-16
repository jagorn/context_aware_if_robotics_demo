#include "utils_proximity.h"

namespace  ProxUtils
{

float proxemics_param(ProxUtils::Space p_space, std::string bound)
{
    if (bound == "min") return prox_param[p_space].at(0);
    else if(bound == "max") return prox_param[p_space].at(1);
    else
    {
        Utils::println("[ProxUtils::proxemics_param] Error! You can only use \"min\" or \"max\" keywords for this function!",Utils::Red);
        return -1.f;
    }
}

float approach_param(ProxUtils::Approach approach, std::string bound)
{
    if (bound == "min") return appr_param[approach].at(0);
    else if(bound == "max") return appr_param[approach].at(1);
    else
    {
        Utils::println("[ProxUtils::approach_param] Error! You can only use \"min\" or \"max\" keywords for this function!",Utils::Red);
        return -1.f;
    }
}

}
