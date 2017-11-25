#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "MoveField.h"

using namespace model;
using namespace std;

namespace {
    constexpr int LAND_GROUP = 1;
    constexpr int AIR_GROUP = 2;
    constexpr int NUKE_GROUP = 3;
    static long long vId = -1;
}

std::ostream& operator<<(std::ostream &s, VehicleType vt) {
    switch (vt) {
        case VehicleType::_UNKNOWN_:  s << "VehicleType::_UNKNOWN_"; break;
        case VehicleType::ARRV:       s << "VehicleType::ARRV"; break;
        case VehicleType::IFV:        s << "VehicleType::IFV"; break;
        case VehicleType::TANK:       s << "VehicleType::TANK"; break;
        case VehicleType::HELICOPTER: s << "VehicleType::HELICOPTER"; break;
        case VehicleType::FIGHTER:    s << "VehicleType::FIGHTER"; break;
        case VehicleType::_COUNT_:    s << "VehicleType::_COUNT_"; break;
    }
    return s;
}

MyStrategy::MyStrategy() {
    ctx_.vehicleById = &vehicles_;
}

double MyStrategy::getWeatherVisibility(WeatherType w) const {
    switch (w) {
        case WeatherType::CLEAR: return ctx_.game->getClearWeatherVisionFactor();
        case WeatherType::CLOUD: return ctx_.game->getCloudWeatherVisionFactor();
        case WeatherType::RAIN:  return ctx_.game->getRainWeatherVisionFactor();
        default: return 0.;
    }
}

double MyStrategy::getTerrainVisibility(TerrainType t) const {
    switch (t) {
        case TerrainType::FOREST: return ctx_.game->getForestTerrainVisionFactor();
        case TerrainType::PLAIN:  return ctx_.game->getPlainTerrainVisionFactor();
        case TerrainType::SWAMP:  return ctx_.game->getSwampTerrainVisionFactor();
        default: return 0.;
    }
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

    this->move();

    executeDelayedMove(move);
}

void MyStrategy::initializeStrategy(const model::Game &game) {
    static bool firstTime = true;
    if (firstTime) {
        firstTime = false;
        srand(game.getRandomSeed());
        slowestGroundSpeed_ = game.getTankSpeed() * game.getSwampTerrainSpeedFactor();
    }
}

void MyStrategy::initializeTick(const Player &me, const World &world, const Game &game, const Move &) {
    ctx_.me = &me;
    ctx_.world = &world;
    ctx_.game = &game;

    for (auto &v: vehicleMoved_) {
        v.second = false;
    }

    for (const auto &vehicle : world.getNewVehicles()) {
        vehicles_[vehicle.getId()] = vehicle;
        vehiclesUpdateTick_[vehicle.getId()] = world.getTickIndex();
        vehiclePrevPos_[vehicle.getId()] = {vehicle.getX(), vehicle.getY()};
        vehicleMoved_[vehicle.getId()] = false;
    }

    for (const auto &vehicleUpdate : world.getVehicleUpdates()) {
        const auto &vehicleId = vehicleUpdate.getId();

        if (vehicleUpdate.getDurability() == 0) {
            vehicles_.erase(vehicleId);
            vehiclesUpdateTick_.erase(vehicleId);
            vehiclePrevPos_.erase(vehicleId);
            vehicleMoved_.erase(vehicleId);
        } else {
            vehicles_[vehicleId] = Vehicle(vehicles_[vehicleId], vehicleUpdate);
            vehiclesUpdateTick_[vehicleId] = world.getTickIndex();
            vehicleMoved_[vehicleId] = vehicleUpdate.getX() != vehiclePrevPos_[vehicleId].first ||
                                       vehicleUpdate.getY() != vehiclePrevPos_[vehicleId].second;
            vehiclePrevPos_[vehicleId] = {vehicleUpdate.getX(), vehicleUpdate.getY()};
        }
    }
}

bool MyStrategy::executeDelayedMove(Move& move) {
    if (moveQueue_.empty())
        return false;

    if (moveQueue_.begin()->first <= ctx_.world->getTickIndex()) {
        move = moveQueue_.begin()->second;
        moveQueue_.erase(moveQueue_.begin());
    }

    return true;
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

void MyStrategy::nuke(const std::pair<double, double> &c, std::pair<double, double> &nukePos, long long &strikeUnit) {
    if (ctx_.me->getRemainingNuclearStrikeCooldownTicks() > 0) {
        return;
    }

    const auto t = target();

    auto eDir = std::make_pair(t.first - c.first, t.second - c.second);
    const auto eDirL = std::sqrt(eDir.first*eDir.first + eDir.second*eDir.second);
    eDir.first /= eDirL;
    eDir.second /= eDirL;

    const auto quarterSpan = 32.;
    const auto pointA = std::make_pair(c.first + eDir.first*quarterSpan, c.second + eDir.second*quarterSpan);

    // choose strike vehicle near A
    long long vehicleId = -1;
    double best_score = std::numeric_limits<double>::max();
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId())
            continue;

        const double visionCoeff = v.second.isAerial()?
                                   getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)v.second.getX()%32][(int)v.second.getY()%32]):
                                   getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)v.second.getX()%32][(int)v.second.getY()%32]);
        const double score = v.second.getSquaredDistanceTo(pointA.first, pointA.second) - v.second.getVisionRange()*visionCoeff*v.second.getVisionRange()*visionCoeff;
        if (score < best_score) {
            best_score = score;
            vehicleId = v.first;
        }
    }
    const auto &v = vehicles_[vehicleId];

    // choose strike point
    const double strikeDistance = v.getVisionRange()*0.9;
    std::pair<double, double> pointB = t;
    if (v.getSquaredDistanceTo(t.first, t.second) > strikeDistance*strikeDistance) {
        auto tDir = std::make_pair(t.first - v.getX(), t.second - v.getY());
        const auto tDirL = std::sqrt(tDir.first*tDir.first + tDir.second*tDir.second);
        tDir.first /= tDirL;
        tDir.second /= tDirL;
        pointB.first = v.getX() + tDir.first*strikeDistance;
        pointB.second = v.getY() + tDir.second*strikeDistance;
    }

    // estimate damage in strike zone
    double balanceDamage = 0.;
    for (const auto &v: vehicles_) {
        const auto d = v.second.getDistanceTo(pointB.first, pointB.second);
        if ( d > ctx_.game->getTacticalNuclearStrikeRadius())
            continue;
        const double dmg = ctx_.game->getMaxTacticalNuclearStrikeDamage() * (1 - d/ctx_.game->getTacticalNuclearStrikeRadius());
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            balanceDamage -= dmg;
        } else {
            balanceDamage += dmg;
        }
    }

    // strike!
    if (balanceDamage > 10.*ctx_.game->getMaxTacticalNuclearStrikeDamage()) {
        nukePos = pointB;
        strikeUnit = vehicleId;
    }

    // try closest enemy
    long long closestEnemy = -1;
    double minDistSq = 1024.*1024.;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId())
            continue;
        const double d = v.second.getSquaredDistanceTo(c.first, c.second);
        if (d < minDistSq) {
            minDistSq = d;
            closestEnemy = v.first;
        }
    }
    if (closestEnemy < 0)
        return;

    pointB.first = vehicles_[closestEnemy].getX();
    pointB.second = vehicles_[closestEnemy].getY();

    // find closest friendly unit
    long long closestFriend = -1;
    minDistSq = 1024.*1024.;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId())
            continue;
        const double visionCoeff = v.second.isAerial()?
                                   getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)v.second.getX()%32][(int)v.second.getY()%32]):
                                   getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)v.second.getX()%32][(int)v.second.getY()%32]);
        const double d = v.second.getSquaredDistanceTo(pointB.first, pointB.second) -
                v.second.getSquaredVisionRange()*visionCoeff*visionCoeff;
        if (d < minDistSq) {
            minDistSq = d;
            closestFriend = v.first;
        }
    }
    if (closestFriend < 0)
        return;

    const double minDist = vehicles_[closestFriend].getDistanceTo(vehicles_[closestEnemy]);
    const double visionCoeff = vehicles_[closestFriend].isAerial()?
                               getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)vehicles_[closestFriend].getX()%32][(int)vehicles_[closestFriend].getY()%32]):
                               getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)vehicles_[closestFriend].getX()%32][(int)vehicles_[closestFriend].getY()%32]);
    const double curVisionRange = vehicles_[closestFriend].getVisionRange()*visionCoeff;
    if (minDist > ctx_.game->getTacticalNuclearStrikeRadius() && minDist < curVisionRange*0.95) {
        nukePos = pointB;
        strikeUnit = closestFriend;
    }
}


void MyStrategy::move() {
    nukeStriker();

    const bool isGroundStartup = startupGroundFormation();
    const bool isAirStartup = startupAirFormation();

    if (!isGroundStartup && !isAirStartup && antiNuke()) {
        return;
    }

    if (!isGroundStartup)
        mainGround();
    if (!isAirStartup)
        mainAir();
}

MyStrategy::VehicleSet MyStrategy::detectRecon(bool isAir) {
    VehicleSet res;
    constexpr int maxRecon = 20;
    std::pair<double, double> c;
    {
        double x = 0.;
        double y = 0.;
        int cnt = 0;
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial()) {
                x += v.second.getX();
                y += v.second.getY();
                ++cnt;
            }
        }
        if (cnt) {
            x /= cnt;
            y /= cnt;
        }
        c.first = x;
        c.second = y;
    }

    const double leftBorder = c.first - 50. - ctx_.game->getFighterVisionRange()*2.;
    const double rightBorder = c.first + 50. + ctx_.game->getFighterVisionRange()*2.;
    const double topBorder = c.second - 50. - ctx_.game->getFighterVisionRange()*2.;
    const double bottomBorder = c.second + 50/2. + ctx_.game->getFighterVisionRange()*2.;

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId() || (isAir && !v.second.isAerial()) || (!isAir && v.second.isAerial()))
            continue;
        if (v.second.getX() > leftBorder && v.second.getX() < rightBorder && v.second.getY() > topBorder && v.second.getY() < bottomBorder) {
            res.insert(v.first);
        }
    }

    if (res.size() > maxRecon)
        res.clear();

    return res;
}

void MyStrategy::queueMove(int delay, const Move &m){
    moveQueue_.emplace(ctx_.world->getTickIndex() + delay, m);
}

bool MyStrategy::mainGround() {
    enum class State {
        Idle,
        PreRotate,
        Rotate,
        PostRotate,
        Move,
        NukeStrike,
        NukeStrikeWait,
        End
    };
    static State state{State::Idle};
    static pair<double, double> pos{0., 0.},
                                rot{1., 0.},
                                t{0., 0.},
                                d{0., 0.};
    constexpr double dd_max = 64.;
    static double ang = 0.;

    if (state == State::End)
        return false;

    std::pair<double, double> nukePos;
    long long strikeUnit = -1;
    nuke(pos, nukePos, strikeUnit);
    if (strikeUnit >= 0 && !vehicles_[strikeUnit].isAerial()) {
        state = State::NukeStrike;
    }

    if (state != State::NukeStrike) {
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial() && vehicleMoved_[v.first]) {
                // юниты ещё перемещаются
                return true;
            }
        }
    }

    bool haveUnits;
    std::tie(haveUnits, pos) = getCenter(false);
    if (!haveUnits) {
        state = State::End;
        return false;
    }

    switch (state) {
        case State::Idle: {
            t = target();
            d.first = t.first - pos.first;
            d.second = t.second - pos.second;
            double dd = std::sqrt(d.first * d.first + d.second * d.second);
            ang = std::asin((rot.first * d.second - rot.second * d.first) / dd);
            if (dd > dd_max) {
                const double k = dd_max / dd;
                d.first *= k;
                d.second *= k;
                dd = dd_max;
            }
            if (std::abs(ang) > M_PI / 12.) {
                rot.first = d.first / dd;
                rot.second = d.second / dd;
                state = State::PreRotate;
            } else {
                state = State::Move;
            }
            return mainGround();
        }

        case State::PreRotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setFactor(1.1);
            queueMove(0, m);

            state = State::Rotate;
        }
        break;

        case State::Rotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::ROTATE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setAngle(ang);
            m.setMaxSpeed(slowestGroundSpeed_);
            queueMove(0, m);

            state = State::PostRotate;
        }
        break;

        case State::PostRotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setFactor(0.1);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::Move: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(d.first);
            m.setY(d.second);
            m.setMaxSpeed(slowestGroundSpeed_);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::NukeStrike: {
            Move m;
            m.setAction(ActionType::TACTICAL_NUCLEAR_STRIKE);
            m.setX(nukePos.first);
            m.setY(nukePos.second);
            m.setVehicleId(strikeUnit);
            queueMove(0, m);

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(0);
            m.setY(0);
            queueMove(0, m);

            state = State::NukeStrikeWait;
        }
        break;

        case State::NukeStrikeWait: {
            if (ctx_.me->getNextNuclearStrikeTickIndex() < 0) {
                state = State::Idle;
                return mainGround();
            }
        }
        break;

        case State::End:
            return false;
    }
    return true;
}

bool MyStrategy::mainHeli() {
    enum class State {
        Idle,
        Converge,
        Move,
        AntiRecon,
        End
    };
    static auto state = State::Idle;
    static std::pair<double, double> pos{0., 0.};
    static VehicleSet enemyRecon;

    if (state == State::End)
        return false;

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == VehicleType::HELICOPTER && vehicleMoved_[v.first]) {
            // юниты ещё перемещаются
            return true;
        }
    }

    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId() || v.second.getType() != VehicleType::HELICOPTER)
            continue;
        pos.first += v.second.getX();
        pos.second += v.second.getY();
        ++cnt;
    }
    if (!cnt) {
        state = State::End;
        return false;
    }
    pos.first /= cnt;
    pos.second /= cnt;

    switch (state) {
        case State::Idle: {
            enemyRecon = detectRecon(false);
            if (!enemyRecon.empty()) {
                state = State::AntiRecon;
            } else {
                double x_min = ctx_.world->getWidth(),
                        x_max = 0.,
                        y_min = ctx_.world->getHeight(),
                        y_max = 0.;
                for (const auto &v: vehicles_) {
                    if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == VehicleType::HELICOPTER) {
                        x_min = std::min(x_min, v.second.getX());
                        x_max = std::max(x_max, v.second.getX());
                        y_min = std::min(y_min, v.second.getY());
                        y_max = std::max(y_max, v.second.getY());
                    }
                }
                const double x_span = x_max - x_min,
                        y_span = y_max - y_min;
                if (x_span > 100. || y_span > 100.) {
                    state = State::Converge;
                } else {
                    state = State::Move;
                }
            }
            return mainHeli();
        }

        case State::Converge: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::ROTATE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setAngle((rand()%2 == 0? 1.: -1.) * M_PI_4);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setFactor(0.1);
            queueMove(20, m);

            state = State::Idle;
        }
        break;

        case State::Move: {
            std::pair<double, double> ally{0., 0.},
                                      foe{0., 0.},
                                      dest{0., 0.};
            int cnt_ally = 0, cnt_foe = 0;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial()) {
                    ally.first += v.second.getX();
                    ally.second += v.second.getY();
                    ++cnt_ally;
                } else if (v.second.getPlayerId() != ctx_.me->getId() && !v.second.isAerial()) {
                    foe.first += v.second.getX();
                    foe.second += v.second.getY();
                    ++cnt_foe;
                }
            }
            if (cnt_foe) {
                foe.first /= cnt_foe;
                foe.second /= cnt_foe;
            }
            if (cnt_ally) {
                ally.first /= cnt_ally;
                ally.second /= cnt_ally;
            } else {
                // наземных юнитов не осталось, движемся к врагу
                ally = foe;
            }

            const double toFoeX = foe.first - ally.first;
            const double toFoeY = foe.second - ally.second;
            const double toFoeD = std::sqrt(toFoeX*toFoeX + toFoeY*toFoeY);

            dest.first = clampX(ally.first + toFoeX/toFoeD*32. + (rand()%200)-100);
            dest.second = clampY(ally.second + toFoeY/toFoeD*32. + (rand()%200)-100);


            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(dest.first - pos.first);
            m.setY(dest.second - pos.second);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::AntiRecon: {
            long long closest = -1;
            double minSqDist = std::numeric_limits<double>::max();
            for (auto vid: enemyRecon) {
                const double sqDist = vehicles_[vid].getSquaredDistanceTo(pos.first, pos.second);
                if (sqDist < minSqDist) {
                    minSqDist = sqDist;
                    closest = vid;
                }
            }

            const double dist = std::sqrt(minSqDist);
            double dx = vehicles_[closest].getX() - pos.first;
            double dy = vehicles_[closest].getY() - pos.second;
            if (dist > 200.) {
                const double k = 200./dist;
                dx *= k;
                dy *= k;
            }

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(dx);
            m.setY(dy);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::End:
            return false;
    }

    return true;
}

bool MyStrategy::mainFighter() {
    enum class State {
        Idle,
        Converge,
        Move,
        AntiRecon,
        End
    };
    static auto state = State::Idle;
    static std::pair<double, double> pos{0., 0.};
    static VehicleSet enemyRecon;

    if (state == State::End)
        return false;

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == VehicleType::FIGHTER && vehicleMoved_[v.first]) {
            // юниты ещё перемещаются
            return true;
        }
    }

    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId() || v.second.getType() != VehicleType::FIGHTER)
            continue;
        pos.first += v.second.getX();
        pos.second += v.second.getY();
        ++cnt;
    }
    if (!cnt) {
        state = State::End;
        return false;
    }
    pos.first /= cnt;
    pos.second /= cnt;

    switch (state) {
        case State::Idle: {
            enemyRecon = detectRecon(true);
            if (!enemyRecon.empty()) {
                state = State::AntiRecon;
            } else {
                double x_min = ctx_.world->getWidth(),
                        x_max = 0.,
                        y_min = ctx_.world->getHeight(),
                        y_max = 0.;
                for (const auto &v: vehicles_) {
                    if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == VehicleType::FIGHTER) {
                        x_min = std::min(x_min, v.second.getX());
                        x_max = std::max(x_max, v.second.getX());
                        y_min = std::min(y_min, v.second.getY());
                        y_max = std::max(y_max, v.second.getY());
                    }
                }
                const double x_span = x_max - x_min,
                        y_span = y_max - y_min;
                if (x_span > 100. || y_span > 100.) {
                    state = State::Converge;
                } else {
                    state = State::Move;
                }
            }
            return mainFighter();
        }

        case State::Converge: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::ROTATE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setAngle((rand()%2 == 0? 1.: -1.) * M_PI_4);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setFactor(0.1);
            queueMove(20, m);

            state = State::Idle;
        }
            break;

        case State::Move: {
            std::pair<double, double> ally{0., 0.},
                    foe{0., 0.},
                    dest{0., 0.};
            int cnt_ally = 0, cnt_foe = 0;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial()) {
                    ally.first += v.second.getX();
                    ally.second += v.second.getY();
                    ++cnt_ally;
                } else if (v.second.getPlayerId() != ctx_.me->getId() && v.second.isAerial()) {
                    foe.first += v.second.getX();
                    foe.second += v.second.getY();
                    ++cnt_foe;
                }
            }
            if (cnt_foe) {
                foe.first /= cnt_foe;
                foe.second /= cnt_foe;
            }
            if (cnt_ally) {
                ally.first /= cnt_ally;
                ally.second /= cnt_ally;
            } else {
                // наземных юнитов не осталось, движемся к врагу
                ally = foe;
            }

            const double toFoeX = foe.first - ally.first;
            const double toFoeY = foe.second - ally.second;
            const double toFoeD = std::sqrt(toFoeX*toFoeX + toFoeY*toFoeY);

            dest.first = clampX(ally.first + toFoeX/toFoeD*32. + (rand()%200)-100);
            dest.second = clampY(ally.second + toFoeY/toFoeD*32. + (rand()%200)-100);

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(dest.first - pos.first);
            m.setY(dest.second - pos.second);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::AntiRecon: {
            long long closest = -1;
            double minSqDist = std::numeric_limits<double>::max();
            for (auto vid: enemyRecon) {
                const double sqDist = vehicles_[vid].getSquaredDistanceTo(pos.first, pos.second);
                if (sqDist < minSqDist) {
                    minSqDist = sqDist;
                    closest = vid;
                }
            }

            const double dist = std::sqrt(minSqDist);
            double dx = vehicles_[closest].getX() - pos.first;
            double dy = vehicles_[closest].getY() - pos.second;
            if (dist > 200.) {
                const double k = 200./dist;
                dx *= k;
                dy *= k;
            }

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(dx);
            m.setY(dy);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::End:
            return false;
    }

    return true;
}

double MyStrategy::clampX(double x) const {
    return std::max(0., std::min(ctx_.world->getWidth(), x));
}

double MyStrategy::clampY(double y) const {
    return std::max(0., std::min(ctx_.world->getHeight(), y));
}

bool MyStrategy::antiNuke() {
    enum class State {
        Idle,
        Spread,
        Converge
    };
    static auto state = State::Idle;
    static int moveCnt = 0;
    static std::pair<double, double> nukePos{-1., -1.};

    switch (state) {
        case State::Idle: {
            if (ctx_.world->getOpponentPlayer().getNextNuclearStrikeTickIndex() < 0) {
                return false;
            }
            bool vehicleInRange = false;
            const double strikeX = ctx_.world->getOpponentPlayer().getNextNuclearStrikeX();
            const double strikeY = ctx_.world->getOpponentPlayer().getNextNuclearStrikeY();
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() != ctx_.me->getId() || v.first == vId)
                    continue;
                const auto d = v.second.getSquaredDistanceTo(strikeX, strikeY);
                if (d < ctx_.game->getTacticalNuclearStrikeRadius()*ctx_.game->getTacticalNuclearStrikeRadius()) {
                    vehicleInRange = true;
                    break;
                }
            }
            if (vehicleInRange) {
                state = State::Spread;
                return antiNuke();
            } else {
                return false;
            }
        }

        case State::Spread: {
            if (ctx_.world->getOpponentPlayer().getNextNuclearStrikeTickIndex() > 0) {
                if (!moveCnt) {
                    nukePos.first = ctx_.world->getOpponentPlayer().getNextNuclearStrikeX();
                    nukePos.second = ctx_.world->getOpponentPlayer().getNextNuclearStrikeY();

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(0);
                    m.setTop(0);
                    m.setRight(ctx_.world->getWidth());
                    m.setBottom(ctx_.world->getHeight());
                    queueMove(0, m);

                    m.setAction(ActionType::SCALE);
                    m.setX(nukePos.first);
                    m.setY(nukePos.second);
                    m.setFactor(10);
                    queueMove(0, m);
                }
                ++moveCnt;
            } else {
                state = State::Converge;
                return antiNuke();
            }
        }
        break;

        case State::Converge: {
            if (!moveCnt) {
                state = State::Idle;
                return false;
            }
            if (nukePos.first >= 0) {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                queueMove(0, m);
                
                m.setAction(ActionType::DESELECT);
                m.setGroup(NUKE_GROUP);
                queueMove(0, m);

                m.setAction(ActionType::SCALE);
                m.setX(nukePos.first);
                m.setY(nukePos.second);
                m.setFactor(0.1);
                queueMove(0, m);

                nukePos = {-1., -1.};
            }
            --moveCnt;
        }
        break;
    }

    return true;
}

bool MyStrategy::startupGroundFormation() {
    enum class State {
        Init,
        XArrange,
        YArrange,
        Merge,
        Scale,
        End
    };
    static auto state{State::Init};

    static std::array<VehicleType, 3> grNum{VehicleType::TANK, VehicleType::IFV, VehicleType::ARRV};
    static std::map<VehicleType, int> type2idx{
            {VehicleType::TANK, 0},
            {VehicleType::IFV,  1},
            {VehicleType::ARRV, 2}
    };
    static std::array<std::pair<double, double>, 3> grPos;

    if (state == State::End)
        return false;

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial() && vehicleMoved_[v.first]) {
            // юниты ещё перемещаются
            return true;
        }
    }

    switch (state) {
        case State::Init: {
            grPos.fill(std::make_pair(0., 0.));

            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() != ctx_.me->getId())
                    continue;
                switch (v.second.getType()) {
                    case VehicleType::TANK:
                    case VehicleType::IFV:
                    case VehicleType::ARRV:
                        break;
                    default:
                        continue;
                }
                auto &pos = grPos[type2idx[v.second.getType()]];
                pos.first += v.second.getX() / 100.;
                pos.second += v.second.getY() / 100.;
            }


            {
                int firstGr = 0;
                double minX = grPos[0].first;
                for (int i: {1, 2}) {
                    if (grPos[i].first < minX) {
                        minX = grPos[i].first;
                        firstGr = i;
                    }
                }
                std::swap(grNum[0], grNum[firstGr]);
                std::swap(grPos[0], grPos[firstGr]);

                if (grPos[1].first > grPos[2].first) {
                    std::swap(grNum[1], grNum[2]);
                    std::swap(grPos[1], grPos[2]);
                }

                for (int i = 0; i < 3; ++i) {
                    type2idx[grNum[i]] = i;
                }
            }

            state = State::XArrange;
            return startupGroundFormation();
        }

        case State::XArrange: {
            const double xMid = std::max(grPos[1].first, 110.);
            const double xCenter[] = {xMid - 70., xMid, xMid + 70.};

            for (int i: {0, 1, 2})  {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNum[i]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(xCenter[i] - grPos[i].first);
                m.setY(0);
                queueMove(0, m);
                grPos[i].first = xCenter[i];
            }

            state = State::YArrange;
        }
        break;

        case State::YArrange: {
            double yLines[10] = {10.};
            for (unsigned i=1; i<10; ++i) {
                yLines[i] = yLines[i-1] + 15.;
            }
            const double yShift[] = {-5., 0., 5.};

            for (int i=0; i<3; ++i) {
                double yMax = 0.;
                double yMin = 1024.;
                for (const auto &v: vehicles_) {
                    if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == grNum[i]) {
                        yMax = std::max(yMax, v.second.getY());
                        yMin = std::min(yMin, v.second.getY());
                    }
                }
                const double ySpan = yMax - yMin;
                const double yStride = ySpan/9.;
                for (int j=0; j<10; ++j) {
                    const double yCur = yMin + j*yStride;
                    const double yTarget = yLines[j] + yShift[i];
                    if (yTarget > yCur)
                        break;

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(0);
                    m.setTop(yCur - 2.);
                    m.setRight(ctx_.world->getWidth());
                    m.setBottom(yCur + 2.);
                    m.setVehicleType(grNum[i]);
                    queueMove(0, m);

                    m.setAction(ActionType::MOVE);
                    m.setX(0);
                    m.setY(yTarget - yCur);
                    queueMove(0, m);
                }

                for (int j=9; j>=0; --j) {
                    const double yCur = yMin + j*yStride;
                    const double yTarget = yLines[j] + yShift[i];
                    if (yTarget < yCur)
                        break;

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(0);
                    m.setTop(yCur - 2.);
                    m.setRight(ctx_.world->getWidth());
                    m.setBottom(yCur + 2.);
                    m.setVehicleType(grNum[i]);
                    queueMove(0, m);

                    m.setAction(ActionType::MOVE);
                    m.setX(0);
                    m.setY(yTarget - yCur);
                    queueMove(0, m);
                }
            }

            state = State::Merge;
        }
        break;

        case State::Merge: {
            for (int i: {0, 2}) {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNum[i]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(grPos[1].first - grPos[i].first);
                m.setY(0);
                queueMove(0, m);
            }
            state = State::Scale;
        }
        break;

        case State::Scale: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::TANK);
            queueMove(0, m);

            m.setAction(ActionType::ADD_TO_SELECTION);
            m.setVehicleType(VehicleType::IFV);
            queueMove(0, m);

            m.setVehicleType(VehicleType::ARRV);
            queueMove(0, m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            double x = 0., y = 0.;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial()) {
                    x += v.second.getX()/300.;
                    y += v.second.getY()/300.;
                }
            }

            m.setAction(ActionType::SCALE);
            m.setX(x);
            m.setY(y);
            m.setFactor(0.1);
            queueMove(0, m);

            state = State::End;
        }
        break;

        case State::End:
            return false;
    }
    return true;
}

bool MyStrategy::startupAirFormation() {
    enum class State {
        Init,
        XArrange,
        YArrange,
        Merge,
        Scale,
        End
    };
    static auto state{State::Init};

    static std::array<VehicleType, 2> grNum{VehicleType::FIGHTER, VehicleType::HELICOPTER};
    static std::map<VehicleType, int> type2idx{
            {VehicleType::FIGHTER, 0},
            {VehicleType::HELICOPTER,  1},
    };
    static std::array<std::pair<double, double>, 2> grPos;

    if (state == State::End)
        return false;

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId() && v.second.isAerial() && v.first != vId && vehicleMoved_[v.first]) {
            // юниты ещё перемещаются
            return true;
        }
    }

    switch (state) {
        case State::Init: {
            grPos.fill(std::make_pair(0., 0.));

            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() != ctx_.me->getId())
                    continue;
                switch (v.second.getType()) {
                    case VehicleType::FIGHTER:
                    case VehicleType::HELICOPTER:
                        break;
                    default:
                        continue;
                }
                auto &pos = grPos[type2idx[v.second.getType()]];
                pos.first += v.second.getX() / 100.;
                pos.second += v.second.getY() / 100.;
            }


            {
                if (grPos[0].first > grPos[1].first) {
                    std::swap(grNum[0], grNum[1]);
                    std::swap(grPos[0], grPos[1]);
                }

                for (int i = 0; i < 2; ++i) {
                    type2idx[grNum[i]] = i;
                }
            }

            state = State::XArrange;
            return startupAirFormation();
        }

        case State::XArrange: {
            const double xMid = std::max(grPos[0].first, 110.);
            const double xCenter[] = {xMid, xMid + 70.};

            for (int i: {0, 1})  {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNum[i]);
                queueMove(0, m);

                if (grNum[i] == VehicleType::FIGHTER) {
                    m.setAction(ActionType::DESELECT);
                    m.setGroup(NUKE_GROUP);
                    queueMove(0, m);
                }

                m.setAction(ActionType::MOVE);
                m.setX(xCenter[i] - grPos[i].first);
                m.setY(0);
                queueMove(0, m);
                grPos[i].first = xCenter[i];
            }

            state = State::YArrange;
        }
            break;

        case State::YArrange: {
            double yLines[10] = {10.};
            for (unsigned i=1; i<10; ++i) {
                yLines[i] = yLines[i-1] + 15.;
            }
            const double yShift[] = {0., 5.};

            for (int i=0; i<2; ++i) {
                double yMax = 0.;
                double yMin = 1024.;
                for (const auto &v: vehicles_) {
                    if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == grNum[i] && v.first != vId) {
                        yMax = std::max(yMax, v.second.getY());
                        yMin = std::min(yMin, v.second.getY());
                    }
                }
                const double ySpan = yMax - yMin;
                const double yStride = ySpan/9.;
                for (int j=0; j<10; ++j) {
                    const double yCur = yMin + j*yStride;
                    const double yTarget = yLines[j] + yShift[i];
                    if (yTarget > yCur)
                        break;

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(0);
                    m.setTop(yCur - 2.);
                    m.setRight(ctx_.world->getWidth());
                    m.setBottom(yCur + 2.);
                    m.setVehicleType(grNum[i]);
                    queueMove(0, m);

                    m.setAction(ActionType::MOVE);
                    m.setX(0);
                    m.setY(yTarget - yCur);
                    queueMove(0, m);
                }

                for (int j=9; j>=0; --j) {
                    const double yCur = yMin + j*yStride;
                    const double yTarget = yLines[j] + yShift[i];
                    if (yTarget < yCur)
                        break;

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(0);
                    m.setTop(yCur - 2.);
                    m.setRight(ctx_.world->getWidth());
                    m.setBottom(yCur + 2.);
                    m.setVehicleType(grNum[i]);
                    queueMove(0, m);

                    m.setAction(ActionType::MOVE);
                    m.setX(0);
                    m.setY(yTarget - yCur);
                    queueMove(0, m);
                }
            }

            state = State::Merge;
        }
            break;

        case State::Merge: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::DESELECT);
            m.setGroup(NUKE_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(grPos[type2idx[VehicleType::HELICOPTER]].first - grPos[type2idx[VehicleType::FIGHTER]].first);
            m.setY(0);
            queueMove(0, m);

            state = State::Scale;
        }
            break;

        case State::Scale: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);
            /*
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::ADD_TO_SELECTION);
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::DESELECT);
            m.setGroup(NUKE_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);
*/
            double x = 0., y = 0.;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == ctx_.me->getId() && v.second.isAerial() && v.first != vId) {
                    x += v.second.getX()/199.;
                    y += v.second.getY()/199.;
                }
            }

            m.setAction(ActionType::SCALE);
            m.setX(x);
            m.setY(y);
            m.setFactor(0.1);
            queueMove(0, m);

            state = State::End;
        }
            break;

        case State::End:
            return false;
    }
    return true;
}

bool MyStrategy::mainAir() {
    enum class State {
        Idle,
        PreRotate,
        Rotate,
        PostRotate,
        Move,
        NukeStrike,
        NukeStrikeWait,
        End
    };
    static State state{State::Idle};
    static pair<double, double> pos{0., 0.},
            rot{1., 0.},
            t{0., 0.},
            d{0., 0.};
    constexpr double dd_max = 10.;
    static double ang = 0.;

    if (state == State::End)
        return false;

    std::pair<double, double> nukePos;
    long long strikeUnit = -1;
    nuke(pos, nukePos, strikeUnit);
    if (strikeUnit >= 0 && vehicles_[strikeUnit].isAerial()) {
        state = State::NukeStrike;
    }

    if (state != State::NukeStrike) {
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == ctx_.me->getId() && v.second.isAerial() && v.first != vId && vehicleMoved_[v.first]) {
                // юниты ещё перемещаются
                return true;
            }
        }
    }

    bool haveUnits;
    std::tie(haveUnits, pos) = getCenter(true);
    if (!haveUnits) {
        state = State::End;
        return false;
    }

    switch (state) {
        case State::Idle: {
            bool haveGroundUnits;
            std::pair<double, double> groundPos;
            std::tie(haveGroundUnits, groundPos) = getCenter(false);
            t = target();
            double dd;
            if (haveGroundUnits &&
                    (groundPos.first - pos.first)*(groundPos.first - pos.first)+(groundPos.second - pos.second)*(groundPos.second - pos.second) > 16*16) {
                d.first = t.first - groundPos.first;
                d.second = t.second - groundPos.second;
                dd = std::sqrt(d.first * d.first + d.second * d.second);
                if (dd > dd_max) {
                    const double k = dd_max / dd;
                    d.first *= k;
                    d.second *= k;
                    dd = dd_max;
                }
                d.first += groundPos.first - pos.first;
                d.second += groundPos.second - pos.second;
                state = State::Move;
            } else {
                d.first = t.first - pos.first;
                d.second = t.second - pos.second;
                dd = std::sqrt(d.first * d.first + d.second * d.second);
                ang = std::asin((rot.first * d.second - rot.second * d.first) / dd);
                if (dd > dd_max) {
                    const double k = dd_max / dd;
                    d.first *= k;
                    d.second *= k;
                    dd = dd_max;
                }
                if (std::abs(ang) > M_PI / 12.) {
                    rot.first = d.first / dd;
                    rot.second = d.second / dd;
                    state = State::PreRotate;
                } else {
                    state = State::Move;
                }
            }

            return mainAir();
        }

        case State::PreRotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setFactor(1.1);
            queueMove(0, m);

            state = State::Rotate;
        }
            break;

        case State::Rotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::ROTATE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setAngle(ang);
            queueMove(0, m);

            state = State::PostRotate;
        }
            break;

        case State::PostRotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setFactor(0.1);
            queueMove(0, m);

            state = State::Idle;
        }
            break;

        case State::Move: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(d.first);
            m.setY(d.second);
            queueMove(0, m);

            state = State::Idle;
        }
            break;

        case State::NukeStrike: {
            Move m;
            m.setAction(ActionType::TACTICAL_NUCLEAR_STRIKE);
            m.setX(nukePos.first);
            m.setY(nukePos.second);
            m.setVehicleId(strikeUnit);
            queueMove(0, m);

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(0);
            m.setY(0);
            queueMove(0, m);

            state = State::NukeStrikeWait;
        }
            break;

        case State::NukeStrikeWait: {
            if (ctx_.me->getNextNuclearStrikeTickIndex() < 0) {
                state = State::Idle;
                return mainAir();
            }
        }
            break;

        case State::End:
            return false;
    }
    return true;
}

std::pair<bool, std::pair<double, double>> MyStrategy::getCenter(bool isAerial) const {
    int cnt = 0;
    std::pair<double, double> res{0., 0.};
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId() && v.second.isAerial() == isAerial && v.first != vId) {
            res.first += v.second.getX();
            res.second += v.second.getY();
            ++cnt;
        }
    }
    if (!cnt) {
        return {false, res};
    }
    res.first /= cnt;
    res.second /= cnt;
    return {true, res};
}

bool MyStrategy::nukeStriker() {
    enum class State {
        Init,
        Idle,
        Move,
        Nuke,
        End
    };

    static auto state = State::Init;

    if (state == State::End) {
        return false;
    }

    if (state != State::Init) {
        if (!vehicles_.count(vId)) {
            bool gotFighter = false;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == VehicleType::FIGHTER) {
                    gotFighter = true;
                    break;
                }
            }
            if (gotFighter) {
                state = State::Init;
            } else {
                state = State::End;
            }
        } else if (vehicleMoved_[vId]) {
            return true;
        }
    }

    switch (state) {
        case State::Init: {
            double x_max = 0., y_max = 0.;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() != ctx_.me->getId() || v.second.getType() != VehicleType::FIGHTER)
                    continue;
                if (v.second.getX() >= x_max && v.second.getY() >= y_max) {
                    x_max = v.second.getX();
                    y_max = v.second.getY();
                    vId = v.first;
                }
            }
//            std::cout << "vId " << vId << ' ' << x_max << ' ' << y_max << std::endl;

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(x_max - 2);
            m.setTop(y_max - 2);
            m.setRight(x_max + 2);
            m.setBottom(y_max + 2);
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(NUKE_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::DISMISS);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(32);
            m.setY(32);
            queueMove(0, m);

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::FIGHTER);
            m.setGroup(0);
            queueMove(0, m);

            m.setAction(ActionType::ADD_TO_SELECTION);
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::DESELECT);
            m.setGroup(NUKE_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::Idle: {
            MoveField f;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() != ctx_.me->getId()) {
                    f.addUnit(v.second.getX(), v.second.getY());
                } else if (v.second.isAerial() && v.first != vId) {
                    f.addObstacle(v.second.getX(), v.second.getY());
                }
            }

            const double x = vehicles_[vId].getX();
            const double y = vehicles_[vId].getY();
            const auto path = f.pathToNeg(x, y);

            if (path.empty()) {
//                std::cout << "empty" << std::endl;
                state = State::Nuke;
                return nukeStriker();
            }
            const auto dest = path[std::min(8, (int)path.size()-1)];

            bool onlyStrikeSelected = vehicles_[vId].isSelected();
            if (onlyStrikeSelected) {
                for (const auto &v: vehicles_) {
                    if (v.second.getPlayerId() == ctx_.me->getId() &&
                        v.first != vId && v.second.isSelected()) {
                        onlyStrikeSelected = false;
                        break;
                    }
                }
            }

            Move m;

            if (!onlyStrikeSelected) {
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setGroup(NUKE_GROUP);
                queueMove(0, m);
            }

            m.setAction(ActionType::MOVE);
            m.setX(dest.first - x);
            m.setY(dest.second - y);
            queueMove(0, m);

            state = State::Move;

//            std::cout << "move " << dest.first << ' ' << dest.second << std::endl;
        }
        break;

        case State::Move: {
            state = State::Idle;
            return nukeStriker();
        }

        case State::Nuke: {
            if (ctx_.me->getNextNuclearStrikeTickIndex() > 0) {
                if (ctx_.me->getNextNuclearStrikeVehicleId() == vId) {
                    break;
                } else {
                    state = State::Idle;
                    return false;
                }
            }
            if (ctx_.me->getRemainingNuclearStrikeCooldownTicks() > 0) {
                state = State::Idle;
                return false;
            }

            const double x = vehicles_[vId].getX();
            const double y = vehicles_[vId].getY();
            const double visionCoeff = getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)x%32][(int)y%32]);
            const double visionSq = (vehicles_[vId].getVisionRange()*visionCoeff)*(vehicles_[vId].getVisionRange()*visionCoeff);
            double tx = 0.;
            double ty = 0.;
            int cnt = 0;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == ctx_.me->getId() || v.second.getSquaredDistanceTo(x, y) > visionSq)
                    continue;
                tx += v.second.getX();
                ty += v.second.getY();
                ++cnt;
            }
            if (!cnt) {
                return false;
            }
            tx /= cnt;
            ty /= cnt;

            const double nSq = ctx_.game->getTacticalNuclearStrikeRadius()*ctx_.game->getTacticalNuclearStrikeRadius();
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() != ctx_.me->getId())
                    continue;
                if (v.second.getSquaredDistanceTo(tx, ty) < nSq) {
                    return false;
                }
            }

            Move m;
            m.setAction(ActionType::TACTICAL_NUCLEAR_STRIKE);
            m.setX(tx);
            m.setY(ty);
            m.setVehicleId(vId);
            queueMove(0, m);
        }
        break;

        case State::End:
            return false;
    }

    return true;
}
