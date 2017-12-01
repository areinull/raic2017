#pragma once

#ifndef MYSTRATEGY_CONTEXT_H
#define MYSTRATEGY_CONTEXT_H

#include <unordered_map>
#include "model/Vehicle.h"
#include "V2d.h"

namespace model {
    class Game;
    class Player;
    class World;
}

using VId = long long;

struct VehicleExt {
    model::Vehicle v;
    V2d prevPos;
    V2d pos;
    int lastUpdateTick;
    bool isMine;

    bool hasMoved() const {
        return prevPos != pos;
    }
};
using VehicleById = std::unordered_map<VId, VehicleExt>;

struct Context {
    const model::Player *me;
    const model::World *world;
    const model::Game *game;
    VehicleById *vehicleById;
};


#endif //MYSTRATEGY_CONTEXT_H
