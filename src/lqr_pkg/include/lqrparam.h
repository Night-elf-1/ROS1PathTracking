#ifndef _LQRPARAM_H
#define _LQRPARAM_H

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>

namespace cpprobotics{
class lqr_params{
public:
    double T = 50;
    double xend = 80.0;
    double yend = 30.0;
}

}