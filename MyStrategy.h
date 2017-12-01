#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include <map>
#include <unordered_set>
#include "Strategy.h"
#include "Context.h"

class MyStrategy : public Strategy {
    using MoveQueue = std::multimap<int, model::Move>;

public:
    MyStrategy();
    void move(const model::Player& me, const model::World& world, const model::Game& game, model::Move& move) override;

private:
    void initializeStrategy(const model::Game &game);
    void initializeTick(const model::Player &me, const model::World &world, const model::Game &game, const model::Move &move);
    void queueMove(int delay, const model::Move &m);
    bool executeDelayedMove(model::Move& move);
    void move();
    V2d target() const;
    void nuke(const V2d &grPos, V2d &nukePos, VId &strikeUnit);
    double getWeatherVisibility(model::WeatherType w) const;
    double getTerrainVisibility(model::TerrainType t) const;
    std::pair<bool, V2d> getCenter(bool isAerial) const;

    bool startupGroundFormation();
    bool startupAirFormation();
    bool mainGround();
    bool mainAir();
    double clampX(double x) const;
    double clampY(double y) const;
    bool antiNuke();
    bool nukeStriker();

    VehicleById vehicles_;
    MoveQueue moveQueue_;
    Context ctx_;
    double slowestGroundSpeed_;
};

#endif
