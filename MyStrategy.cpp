#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <algorithm>

using namespace model;
using namespace std;

MyStrategy::MyStrategy() {
    ctx_.vehicleById = &vehicles_;
}

void MyStrategy::move(const Player& me, const World& world, const Game& game, Move& move) {
    initializeStrategy(game);
    initializeTick(me, world, game, move);

    if (me.getRemainingActionCooldownTicks() > 0) {
        return;
    }

    if (executeDelayedMove(move)) {
        return;
    }

    this->move(me, world, game);

    executeDelayedMove(move);
}

void MyStrategy::initializeStrategy(const model::Game &game) {
    static bool firstTime = true;
    if (firstTime) {
        srand(game.getRandomSeed());
    }
}

void MyStrategy::initializeTick(const Player &me, const World &world, const Game &game, const Move &) {
    ctx_.me = &me;
    ctx_.world = &world;
    ctx_.game = &game;

    for (const auto &vehicle : world.getNewVehicles()) {
        vehicles_[vehicle.getId()] = vehicle;
    }

    for (const auto &vehicleUpdate : world.getVehicleUpdates()) {
        const auto &vehicleId = vehicleUpdate.getId();

        if (vehicleUpdate.getDurability() == 0) {
            vehicles_.erase(vehicleId);
        } else {
            vehicles_[vehicleId] = Vehicle(vehicles_[vehicleId], vehicleUpdate);
        }
    }
}

bool MyStrategy::executeDelayedMove(Move& move) {
    if (moveQueue_.empty())
        return false;

    if (moveQueue_.front().first > 0) {
        --moveQueue_.front().first;
    } else {
        move = moveQueue_.front().second;
        moveQueue_.pop();
    }

    return true;
}

std::pair<double, double> MyStrategy::center() const {
    double x = 0.;
    double y = 0.;
    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            x += v.second.getX();
            y += v.second.getY();
            ++cnt;
        }
    }
    if (cnt) {
        x /= cnt;
        y /= cnt;
    }
    return {x, y};
}

std::pair<double, double> MyStrategy::target() const {
    double x = 0.;
    double y = 0.;
    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId()) {
            x += v.second.getX();
            y += v.second.getY();
            ++cnt;
        }
    }
    if (cnt) {
        x /= cnt;
        y /= cnt;
    }
    return {x, y};
}

std::pair<double,double> MyStrategy::span() const {
    double x_min = ctx_.world->getWidth();
    double x_max = 0.;
    double y_min = ctx_.world->getHeight();
    double y_max = 0.;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            x_min = std::min(x_min, v.second.getX());
            x_max = std::max(x_max, v.second.getX());
            y_min = std::min(y_min, v.second.getY());
            y_max = std::max(y_max, v.second.getY());
        }
    }
    return {x_max-x_min, y_max-y_min};
}

void MyStrategy::congregate() {
    constexpr double angle = PI/2.;
    const auto c = center();
    const auto s = span();
    Move m;

    // сдвиг в центр
    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(0);
    m.setTop(0);
    m.setRight(c.first);
    m.setBottom(c.second);
    moveQueue_.emplace(0, m);
    m.setAction(ActionType::MOVE);
    m.setX(s.first/2);
    m.setY(s.second/2);
    moveQueue_.emplace(0, m);

    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(c.first);
    m.setTop(0);
    m.setRight(ctx_.world->getWidth());
    m.setBottom(c.second);
    moveQueue_.emplace(0, m);
    m.setAction(ActionType::MOVE);
    m.setX(-s.first/2);
    m.setY(s.second/2);
    moveQueue_.emplace(0, m);

    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(c.first);
    m.setTop(c.second);
    m.setRight(ctx_.world->getWidth());
    m.setBottom(ctx_.world->getHeight());
    moveQueue_.emplace(0, m);
    m.setAction(ActionType::MOVE);
    m.setX(-s.first/2);
    m.setY(-s.second/2);
    moveQueue_.emplace(0, m);

    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(0);
    m.setTop(c.second);
    m.setRight(c.first);
    m.setBottom(ctx_.world->getHeight());
    moveQueue_.emplace(0, m);
    m.setAction(ActionType::MOVE);
    m.setX(s.first/2);
    m.setY(-s.second/2);
    moveQueue_.emplace(0, m);

    // вращение
    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(0);
    m.setTop(0);
    m.setRight(ctx_.world->getWidth());
    m.setBottom(ctx_.world->getHeight());
    moveQueue_.emplace(120, m);
    m.setAction(ActionType::ROTATE);
    m.setX(c.first);
    m.setY(c.second);
    m.setAngle(angle);
    moveQueue_.emplace(0, m);
}

double MyStrategy::distToEnemy() const {
    double d = 2.*std::max(ctx_.world->getHeight(), ctx_.world->getWidth()) * 2.*std::max(ctx_.world->getHeight(), ctx_.world->getWidth());
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            for (const auto &t: vehicles_) {
                if (t.second.getPlayerId() != ctx_.me->getId()) {
                    d = std::min(d, v.second.getSquaredDistanceTo(t.second));
                }
            }
        }
    }
}


void MyStrategy::move(const Player& me, const World& world, const Game& game) {
    if (world.getTickIndex() == 0) {
        congregate();
    }
    if (world.getTickIndex() == 240) {
        congregate();
    }
    if (world.getTickIndex() == 480) {
        congregate();
    }

    if (world.getTickIndex() >= 720 &&
        world.getTickIndex() % 120) {
        if (distToEnemy() > 20.) {
            constexpr double dd_max = 100. * 100.;
            const auto c = center();
            const auto t = target();
            double dx = t.first - c.first;
            double dy = t.second - c.second;
            const double dd = dx * dx + dy * dy;
            if (dd > dd_max) {
                const double k = dd_max / dd;
                dx *= k;
                dy *= k;
            }

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            moveQueue_.emplace(0, m);
            m.setAction(ActionType::MOVE);
            m.setX(dx);
            m.setY(dy);
            m.setMaxSpeed(game.getTankSpeed());
            moveQueue_.emplace(0, m);
        } else {
            congregate();
        }
    }

}
