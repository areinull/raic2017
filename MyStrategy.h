#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include <map>
#include <unordered_set>
#include "Strategy.h"
#include "Context.h"

class MyStrategy : public Strategy {
    using UpdateTickByVehicleId = std::unordered_map<long long, int>;
    using VehiclePrevPos = std::unordered_map<long long, std::pair<double, double>>;
    using VehicleMoved = std::unordered_map<long long, bool>;
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
    std::pair<double, double> target() const;
    void nuke(const std::pair<double, double> &grPos, std::pair<double, double> &nukePos, long long &strikeUnit);
    double getWeatherVisibility(model::WeatherType w) const;
    double getTerrainVisibility(model::TerrainType t) const;
    VehicleSet detectRecon(bool isAir);
    std::pair<bool, std::pair<double, double>> getCenter(bool isAerial) const;

    bool startupGroundFormation();
    bool startupAirFormation();
    bool mainGround();
    bool mainAir();
    bool mainHeli();
    bool mainFighter();
    double clampX(double x) const;
    double clampY(double y) const;
    bool antiNuke();
    bool nukeStriker();

    VehicleById vehicles_;
    UpdateTickByVehicleId vehiclesUpdateTick_;
    MoveQueue moveQueue_;
    Context ctx_;
    double slowestGroundSpeed_;
    VehiclePrevPos vehiclePrevPos_;
    VehicleMoved vehicleMoved_;
};

#endif
