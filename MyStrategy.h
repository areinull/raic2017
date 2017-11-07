#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include <unordered_map>
#include <queue>
#include "Strategy.h"

class MyStrategy : public Strategy {
public:
    void move(const model::Player& me, const model::World& world, const model::Game& game, model::Move& move) override;

private:
    model::VehicleType getPreferredTargetType(model::VehicleType vehicleType, const model::Player &me, const model::World &world, const model::Game &game) const;
    void initializeStrategy(const model::Game &game);
    void initializeTick(const model::Player &me, const model::World &world, const model::Game &game, const model::Move &move);
    bool executeDelayedMove(model::Move& move);
    void move(const model::Player& me, const model::World& world, const model::Game& game);

    std::unordered_map<int, model::Vehicle> vehicles_;
    std::unordered_map<long long, int> updateTickByVehicleId_;
    std::queue<model::Move> moveQueue_;
};

#endif
