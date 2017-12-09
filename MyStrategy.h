#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include <map>
#include <unordered_set>
#include <functional>
#include "Strategy.h"
#include "Context.h"

namespace Clusterize {
    class ClusterLists;
}

class MyStrategy : public Strategy {
    using MoveQueue = std::multimap<int, std::pair<model::Move, std::function<void(void)>>>;

public:
    MyStrategy();
    void move(const model::Player& me, const model::World& world, const model::Game& game, model::Move& move) override;

private:
    void initializeStrategy(const model::Game &game);
    void initializeTick(const model::Player &me, const model::World &world, const model::Game &game, const model::Move &move);
    void queueMove(int delay, const model::Move &m, std::function<void(void)> &&f = std::function<void(void)>());
    bool executeDelayedMove(model::Move& move);
    void move();
    V2d target(const V2d &c, bool acceptFacility, bool isMainForce = false) const;
    void nuke(const V2d &grPos, V2d &nukePos, VId &strikeUnit);
    double getWeatherVisibility(model::WeatherType w) const;
    double getTerrainVisibility(model::TerrainType t) const;
    std::pair<bool, V2d> getCenter(int group) const;

    bool startupFormation();
    bool mainForce();
    double clampX(double x) const;
    double clampY(double y) const;
    V2d clamp4main(V2d c, V2d t) const;
    bool antiNuke();
    bool nukeStriker();
    bool onlyGroupSelected(int g) const;
    void manageFacilities();
    void manageClusters();

    model::VehicleType recommendVehicleType() const;

    VehicleById vehicles_;
    MoveQueue moveQueue_;
    Context ctx_;
    double slowestGroundSpeed_;
    double slowestAirSpeed_;
    Clusterize::ClusterLists *clist_;
};

#endif
