#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include <queue>
#include "Strategy.h"
#include "Context.h"

class MyStrategy : public Strategy {
    using UpdateTickByVehicleId = std::unordered_map<long long, int>;
    using MoveQueue = std::queue<std::pair<int,model::Move>>;

public:
    MyStrategy();
    void move(const model::Player& me, const model::World& world, const model::Game& game, model::Move& move) override;

private:
    void initializeStrategy(const model::Game &game);
    void initializeTick(const model::Player &me, const model::World &world, const model::Game &game, const model::Move &move);
    bool executeDelayedMove(model::Move& move);
    void move(const model::Player& me, const model::World& world, const model::Game& game);
    void congregate();
    std::pair<double, double> center() const;
    std::pair<double,double> span() const;
    std::pair<double, double> target() const;
    double distToEnemy() const;

    VehicleById vehicles_;
    MoveQueue moveQueue_;
    Context ctx_;
};

#endif
