#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "Constants.h"
#include "Point3D.h"


class MyStrategy : public Strategy {
public:
    MyStrategy();

    //Point3D ball;
    //Point3D ball_velocity;
    //model::Game simGame;
    std::string custom_rendering() override;
    void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;
};

#endif
