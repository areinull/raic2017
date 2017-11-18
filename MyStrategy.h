#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include <map>
#include <unordered_set>
#include "Strategy.h"
#include "Context.h"

class MyStrategy : public Strategy {
    using UpdateTickByVehicleId = std::unordered_map<long long, int>;
    using MoveQueue = std::multimap<int,model::Move>;
    using VehicleSet = std::unordered_set<long long>;

public:
    MyStrategy();
    void move(const model::Player& me, const model::World& world, const model::Game& game, model::Move& move) override;

private:
    void initializeStrategy(const model::Game &game);
    void initializeTick(const model::Player &me, const model::World &world, const model::Game &game, const model::Move &move);
    void queueMove(int delay, const model::Move &m);
    bool executeDelayedMove(model::Move& move);
    void move();
    std::pair<double, double> center() const;
    std::pair<double,double> span() const;
    std::pair<double, double> target() const;
    void nuke(const std::pair<double, double> &grPos, std::pair<double, double> &nukePos, long long &strikeUnit);
    double getWeatherVisibility(model::WeatherType w) const;
    double getTerrainVisibility(model::TerrainType t) const;
    VehicleSet detectRecon(bool isAir);

    bool startupGroundFormation();
    bool mainGround();
    bool mainHeli();
    bool mainFighter();
    double clampX(double x) const;
    double clampY(double y) const;

    VehicleById vehicles_;
    UpdateTickByVehicleId vehiclesUpdateTick_;
    MoveQueue moveQueue_;
    Context ctx_;
    double slowestGroundSpeed_;
};

#endif
