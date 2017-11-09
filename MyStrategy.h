#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include <queue>
#include "Strategy.h"
#include "Context.h"
#include "Blob.h"

class MyStrategy : public Strategy {
    using UpdateTickByVehicleId = std::unordered_map<long long, int>;
    using MoveQueue = std::queue<model::Move>;

public:
    MyStrategy();
    void move(const model::Player& me, const model::World& world, const model::Game& game, model::Move& move) override;

private:
    model::VehicleType getPreferredTargetType(model::VehicleType vehicleType, const model::Player &me, const model::World &world, const model::Game &game) const;
    void initializeStrategy(const model::Game &game);
    void initializeTick(const model::Player &me, const model::World &world, const model::Game &game, const model::Move &move);
    bool executeDelayedMove(model::Move& move);
    void move(const model::Player& me, const model::World& world, const model::Game& game);

    VehicleById vehicles_;
    UpdateTickByVehicleId updateTickByVehicleId_;
    MoveQueue moveQueue_;
    Context ctx_;
    Blob blobs_[(int)model::VehicleType::_COUNT_];
};

#endif
