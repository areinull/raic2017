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

namespace {
    constexpr int LAND_GROUP = 1;
    constexpr int AIR_GROUP = 2;
    constexpr int NUKE_GROUP = 3;
    constexpr int ANTINUKE_GROUP = 4;
    static VId vId = -1;
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
        slowestAirSpeed_ = game.getHelicopterSpeed() * game.getRainWeatherSpeedFactor();
        ctx_.vehicleById = &vehicles_;
    }
}

void MyStrategy::initializeTick(const Player &me, const World &world, const Game &game, const Move &) {
    ctx_.me = &me;
    ctx_.world = &world;
    ctx_.game = &game;

    for (auto &v: vehicles_) {
        v.second.prevPos = v.second.pos;
    }

    for (const auto &vehicle : world.getNewVehicles()) {
        vehicles_[vehicle.getId()].v = vehicle;
        vehicles_[vehicle.getId()].lastUpdateTick = world.getTickIndex();
        vehicles_[vehicle.getId()].pos = {vehicle.getX(), vehicle.getY()};
        vehicles_[vehicle.getId()].prevPos = vehicles_[vehicle.getId()].pos;
        vehicles_[vehicle.getId()].isMine = vehicle.getPlayerId() == me.getId();
    }

    for (const auto &vehicleUpdate : world.getVehicleUpdates()) {
        const auto &vehicleId = vehicleUpdate.getId();

        if (vehicleUpdate.getDurability() == 0) {
            vehicles_.erase(vehicleId);
        } else {
            vehicles_[vehicleId].v = Vehicle(vehicles_[vehicleId].v, vehicleUpdate);
            vehicles_[vehicleId].lastUpdateTick = world.getTickIndex();
            vehicles_[vehicleId].pos = {vehicleUpdate.getX(), vehicleUpdate.getY()};
        }
    }
}

bool MyStrategy::executeDelayedMove(Move& move) {
    if (moveQueue_.empty())
        return false;

    if (moveQueue_.begin()->first <= ctx_.world->getTickIndex()) {
        const auto &elt = moveQueue_.begin()->second;
        move = elt.first;
        if (elt.second) {
            elt.second();
        }
        moveQueue_.erase(moveQueue_.begin());

        if (move.getAction() == ActionType::CLEAR_AND_SELECT && move.getGroup()) {
            if (onlyGroupSelected(move.getGroup())) {
                executeDelayedMove(move);
            }
        }
    }

    return true;
}

V2d MyStrategy::target() const {
    V2d t{0., 0.};
    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (!v.second.isMine) {
            t += v.second.pos;
            ++cnt;
        }
    }
    if (cnt) {
        t /= cnt;
    }
    return t;
}

void MyStrategy::nuke(const V2d &c, V2d &nukePos, VId &strikeUnit) {
    if (ctx_.me->getRemainingNuclearStrikeCooldownTicks() > 0) {
        return;
    }

    const auto t = target();

    const V2d eDir = (t - c).unit();

    const auto quarterSpan = 32.;
    const auto pointA = c + eDir*quarterSpan;

    // choose strike vehicle near A
    VId vehicleId = -1;
    double best_score = std::numeric_limits<double>::max();
    for (const auto &v: vehicles_) {
        if (!v.second.isMine)
            continue;

        const double visionCoeff = v.second.v.isAerial()?
                                   getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)v.second.pos.x%32][(int)v.second.pos.y%32]):
                                   getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)v.second.pos.x%32][(int)v.second.pos.y%32]);
        const double score = v.second.v.getSquaredDistanceTo(pointA.x, pointA.y) - v.second.v.getVisionRange()*visionCoeff*v.second.v.getVisionRange()*visionCoeff;
        if (score < best_score) {
            best_score = score;
            vehicleId = v.first;
        }
    }
    const auto &vext = vehicles_[vehicleId];

    // choose strike point
    const double strikeDistance = vext.v.getVisionRange()*0.9;
    V2d pointB = t;
    if (vext.v.getSquaredDistanceTo(t.x, t.y) > strikeDistance*strikeDistance) {
        const auto tDir = (t - vext.pos).unit();
        pointB = vext.pos + tDir*strikeDistance;
    }

    // estimate damage in strike zone
    double balanceDamage = 0.;
    for (const auto &v: vehicles_) {
        const auto d = v.second.v.getDistanceTo(pointB.x, pointB.y);
        if ( d > ctx_.game->getTacticalNuclearStrikeRadius())
            continue;
        const double dmg = ctx_.game->getMaxTacticalNuclearStrikeDamage() * (1 - d/ctx_.game->getTacticalNuclearStrikeRadius());
        if (v.second.isMine) {
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
    VId closestEnemy = -1;
    double minDistSq = 1024.*1024.;
    for (const auto &v: vehicles_) {
        if (v.second.isMine)
            continue;
        const double d = v.second.v.getSquaredDistanceTo(c.x, c.y);
        if (d < minDistSq) {
            minDistSq = d;
            closestEnemy = v.first;
        }
    }
    if (closestEnemy < 0)
        return;

    pointB = vehicles_[closestEnemy].pos;

    // find closest friendly unit
    VId closestFriend = -1;
    minDistSq = 1024.*1024.;
    for (const auto &v: vehicles_) {
        if (!v.second.isMine || v.first == vId)
            continue;
        const double visionCoeff = v.second.v.isAerial()?
                                   getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)v.second.pos.x%32][(int)v.second.pos.y%32]):
                                   getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)v.second.pos.x%32][(int)v.second.pos.y%32]);
        const double d = v.second.v.getSquaredDistanceTo(pointB.x, pointB.y) -
                v.second.v.getSquaredVisionRange()*visionCoeff*visionCoeff;
        if (d < minDistSq) {
            minDistSq = d;
            closestFriend = v.first;
        }
    }
    if (closestFriend < 0)
        return;

    const double minDist = vehicles_[closestFriend].v.getDistanceTo(vehicles_[closestEnemy].v);
    const double visionCoeff = vehicles_[closestFriend].v.isAerial()?
                               getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)vehicles_[closestFriend].pos.x%32][(int)vehicles_[closestFriend].pos.y%32]):
                               getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)vehicles_[closestFriend].pos.x%32][(int)vehicles_[closestFriend].pos.y%32]);
    const double curVisionRange = vehicles_[closestFriend].v.getVisionRange()*visionCoeff;
    if (minDist > ctx_.game->getTacticalNuclearStrikeRadius() && minDist < curVisionRange*0.95) {
        nukePos = pointB;
        strikeUnit = closestFriend;
    }
}


void MyStrategy::move() {
    if (ctx_.world->getTickIndex() == 0) {
        Move m;
        m.setAction(ActionType::CLEAR_AND_SELECT);
        m.setLeft(0);
        m.setTop(0);
        m.setRight(ctx_.world->getWidth());
        m.setBottom(ctx_.world->getHeight());
        queueMove(0, m);

        m.setAction(ActionType::ASSIGN);
        m.setGroup(ANTINUKE_GROUP);
        queueMove(0, m);
    }

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

void MyStrategy::queueMove(int delay, const Move &m, std::function<void(void)> &&f) {
    moveQueue_.emplace(ctx_.world->getTickIndex() + delay, std::make_pair(m, f));
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
    static V2d pos{0., 0.},
               rot{1., 0.},
               t{0., 0.},
               d{0., 0.};
    constexpr double dd_max = 64.;
    static double ang = 0.;
    static bool waitingMoveQueue = false;

    if (state == State::End)
        return false;

    V2d nukePos;
    VId strikeUnit = -1;
    nuke(pos, nukePos, strikeUnit);
    if (strikeUnit >= 0 && !vehicles_[strikeUnit].v.isAerial()) {
        state = State::NukeStrike;
    }

    if (state != State::NukeStrike) {
        for (const auto &v: vehicles_) {
            if (v.second.isMine && !v.second.v.isAerial() && v.second.hasMoved()) {
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
            d = t - pos;
            double dd = d.getNorm();
            ang = std::asin((rot.x * d.y - rot.y * d.x) / dd);
            if (dd > dd_max) {
                const double k = dd_max / dd;
                d *= k;
                dd = dd_max;
            }
            if (std::abs(ang) > PI / 12.) {
                rot = d/dd;
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
            m.setX(pos.x);
            m.setY(pos.y);
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
            m.setX(pos.x);
            m.setY(pos.y);
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
            m.setX(pos.x);
            m.setY(pos.y);
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
            m.setX(d.x);
            m.setY(d.y);
            m.setMaxSpeed(slowestGroundSpeed_);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::NukeStrike: {
            Move m;
            m.setAction(ActionType::TACTICAL_NUCLEAR_STRIKE);
            m.setX(nukePos.x);
            m.setY(nukePos.y);
            m.setVehicleId(strikeUnit);
            queueMove(0, m, [&waitingMoveQueue](){ waitingMoveQueue = false; });
            waitingMoveQueue = true;

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
            if (!waitingMoveQueue && ctx_.me->getNextNuclearStrikeTickIndex() < 0) {
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
    static V2d nukePos{-1., -1.};

    switch (state) {
        case State::Idle: {
            if (ctx_.world->getOpponentPlayer().getNextNuclearStrikeTickIndex() < 0) {
                return false;
            }
            bool vehicleInRange = false;
            const double strikeX = ctx_.world->getOpponentPlayer().getNextNuclearStrikeX();
            const double strikeY = ctx_.world->getOpponentPlayer().getNextNuclearStrikeY();
            for (const auto &v: vehicles_) {
                if (!v.second.isMine || v.first == vId)
                    continue;
                const auto d = v.second.v.getSquaredDistanceTo(strikeX, strikeY);
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
                    nukePos.x = ctx_.world->getOpponentPlayer().getNextNuclearStrikeX();
                    nukePos.y = ctx_.world->getOpponentPlayer().getNextNuclearStrikeY();

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setGroup(ANTINUKE_GROUP);
                    queueMove(0, m);

                    m.setAction(ActionType::SCALE);
                    m.setX(nukePos.x);
                    m.setY(nukePos.y);
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
            if (nukePos.x >= 0) {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setGroup(ANTINUKE_GROUP);
                queueMove(0, m);

                m.setAction(ActionType::SCALE);
                m.setX(nukePos.x);
                m.setY(nukePos.y);
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
    static std::array<V2d, 3> grPos;

    if (state == State::End)
        return false;

    for (const auto &v: vehicles_) {
        if (v.second.isMine && !v.second.v.isAerial() && v.second.hasMoved()) {
            // юниты ещё перемещаются
            return true;
        }
    }

    switch (state) {
        case State::Init: {
            grPos.fill(V2d{0., 0.});

            for (const auto &v: vehicles_) {
                if (!v.second.isMine)
                    continue;
                switch (v.second.v.getType()) {
                    case VehicleType::TANK:
                    case VehicleType::IFV:
                    case VehicleType::ARRV:
                        break;
                    default:
                        continue;
                }
                auto &pos = grPos[type2idx[v.second.v.getType()]];
                pos += v.second.pos / 100.;
            }


            {
                int firstGr = 0;
                double minX = grPos[0].x;
                for (int i: {1, 2}) {
                    if (grPos[i].x < minX) {
                        minX = grPos[i].x;
                        firstGr = i;
                    }
                }
                std::swap(grNum[0], grNum[firstGr]);
                std::swap(grPos[0], grPos[firstGr]);

                if (grPos[1].x > grPos[2].x) {
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
            const double xMid = std::max(grPos[1].x, 110.);
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
                m.setX(xCenter[i] - grPos[i].x);
                m.setY(0);
                queueMove(0, m);
                grPos[i].x = xCenter[i];
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
                    if (v.second.isMine && v.second.v.getType() == grNum[i]) {
                        yMax = std::max(yMax, v.second.pos.y);
                        yMin = std::min(yMin, v.second.pos.y);
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
                m.setX(grPos[1].x - grPos[i].x);
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

            V2d c{0., 0.};
            for (const auto &v: vehicles_) {
                if (v.second.isMine && !v.second.v.isAerial()) {
                    c += v.second.pos/300.;
                }
            }

            m.setAction(ActionType::SCALE);
            m.setX(c.x);
            m.setY(c.y);
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
    static std::array<V2d, 2> grPos;

    if (state == State::End)
        return false;

    for (const auto &v: vehicles_) {
        if (v.second.isMine && v.second.v.isAerial() && v.first != vId && v.second.hasMoved()) {
            // юниты ещё перемещаются
            return true;
        }
    }

    switch (state) {
        case State::Init: {
            grPos.fill(V2d{0., 0.});

            for (const auto &v: vehicles_) {
                if (!v.second.isMine)
                    continue;
                switch (v.second.v.getType()) {
                    case VehicleType::FIGHTER:
                    case VehicleType::HELICOPTER:
                        break;
                    default:
                        continue;
                }
                auto &pos = grPos[type2idx[v.second.v.getType()]];
                pos += v.second.pos/100.;
            }


            {
                if (grPos[0].x > grPos[1].x) {
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
            const double xMid = std::max(grPos[0].x, 110.);
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
                m.setX(xCenter[i] - grPos[i].x);
                m.setY(0);
                queueMove(0, m);
                grPos[i].x = xCenter[i];
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
                    if (v.second.isMine && v.second.v.getType() == grNum[i] && v.first != vId) {
                        yMax = std::max(yMax, v.second.pos.y);
                        yMin = std::min(yMin, v.second.pos.y);
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
            m.setX(grPos[type2idx[VehicleType::HELICOPTER]].x - grPos[type2idx[VehicleType::FIGHTER]].x);
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

            V2d c{0., 0.};
            for (const auto &v: vehicles_) {
                if (v.second.isMine && v.second.v.isAerial() && v.first != vId) {
                    c += v.second.pos/199.;
                }
            }

            m.setAction(ActionType::SCALE);
            m.setX(c.x);
            m.setY(c.y);
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
    static V2d pos{0., 0.},
               rot{1., 0.},
               t{0., 0.},
               d{0., 0.};
    constexpr double dd_max = 10.;
    static double ang = 0.;
    static bool waitingMoveQueue = false;

    if (state == State::End)
        return false;

    V2d nukePos;
    VId strikeUnit = -1;
    nuke(pos, nukePos, strikeUnit);
    if (strikeUnit >= 0 && vehicles_[strikeUnit].v.isAerial()) {
        state = State::NukeStrike;
    }

    if (state != State::NukeStrike) {
        for (const auto &v: vehicles_) {
            if (v.second.isMine && v.second.v.isAerial() && v.first != vId && v.second.hasMoved()) {
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
            V2d groundPos;
            std::tie(haveGroundUnits, groundPos) = getCenter(false);
            t = target();
            double dd;
            if (haveGroundUnits && (groundPos - pos).getNormSq() > 16*16) {
                d = t - groundPos;
                dd = d.getNorm();
                if (dd > dd_max) {
                    const double k = dd_max / dd;
                    d *= k;
                    dd = dd_max;
                }
                d += groundPos - pos;
                state = State::Move;
            } else {
                d = t - pos;
                dd = d.getNorm();
                ang = std::asin((rot.x * d.y - rot.y * d.x) / dd);
                if (dd > dd_max) {
                    const double k = dd_max / dd;
                    d *= k;
                    dd = dd_max;
                }
                if (std::abs(ang) > PI / 12.) {
                    rot = d/dd;
                    state = State::PreRotate;
                } else {
                    state = State::Move;
                }
            }

            /*
            if (haveGroundUnits) {
                d = t - groundPos;
                dd = d.getNorm();
                ang = std::asin((rot.x * d.y - rot.y * d.x) / dd);
                if (std::abs(ang) > M_PI / 12.) {
                    rot = d/dd;
                    state = State::PreRotate;
                } else {
                    d = groundPos - pos + d*12./dd;
                    state = State::Move;
                }
            } else {
                d = t - pos;
                dd = d.getNorm();
                ang = std::asin((rot.x * d.y - rot.y * d.x) / dd);
                if (std::abs(ang) > M_PI / 12.) {
                    rot = d/dd;
                    state = State::PreRotate;
                } else {
                    state = State::Move;
                }
            }
             */

            return mainAir();
        }

        case State::PreRotate: {
            Move m;

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setX(pos.x);
            m.setY(pos.y);
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
            m.setX(pos.x);
            m.setY(pos.y);
            m.setAngle(ang);
            m.setMaxSpeed(slowestAirSpeed_);
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
            m.setX(pos.x);
            m.setY(pos.y);
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
            m.setX(d.x);
            m.setY(d.y);
            m.setMaxSpeed(slowestAirSpeed_);
            queueMove(0, m);

            state = State::Idle;
        }
            break;

        case State::NukeStrike: {
            Move m;
            m.setAction(ActionType::TACTICAL_NUCLEAR_STRIKE);
            m.setX(nukePos.x);
            m.setY(nukePos.y);
            m.setVehicleId(strikeUnit);
            queueMove(0, m, [&waitingMoveQueue](){ waitingMoveQueue = false; });
            waitingMoveQueue = true;

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
            if (!waitingMoveQueue && ctx_.me->getNextNuclearStrikeTickIndex() < 0) {
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

std::pair<bool, V2d> MyStrategy::getCenter(bool isAerial) const {
    int cnt = 0;
    V2d res{0., 0.};
    for (const auto &v: vehicles_) {
        if (v.second.isMine && v.second.v.isAerial() == isAerial && v.first != vId) {
            res += v.second.pos;
            ++cnt;
        }
    }
    if (!cnt) {
        return {false, res};
    }
    res /= cnt;
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
                if (v.second.isMine && v.second.v.getType() == VehicleType::FIGHTER) {
                    gotFighter = true;
                    break;
                }
            }
            if (gotFighter) {
                state = State::Init;
            } else {
                state = State::End;
            }
        } else if (vehicles_[vId].hasMoved()) {
            return true;
        }
    }

    switch (state) {
        case State::Init: {
            V2d pos_max{0., 0.};
            for (const auto &v: vehicles_) {
                if (!v.second.isMine || v.second.v.getType() != VehicleType::FIGHTER)
                    continue;
                if (v.second.pos.x >= pos_max.x && v.second.pos.y >= pos_max.y) {
                    pos_max = v.second.pos;
                    vId = v.first;
                }
            }
//            std::cout << "vId " << vId << ' ' << x_max << ' ' << y_max << std::endl;

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(pos_max.x - 2);
            m.setTop(pos_max.y - 2);
            m.setRight(pos_max.x + 2);
            m.setBottom(pos_max.y + 2);
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(NUKE_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::DISMISS);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            m.setGroup(ANTINUKE_GROUP);
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
                if (!v.second.isMine) {
                    f.addUnit(v.second.pos);
                } else if (v.second.v.isAerial() && v.first != vId) {
                    f.addObstacle(v.second.pos);
                }
            }

            const auto path = f.pathToNeg(vehicles_[vId].pos);

            if (path.empty()) {
//                std::cout << "empty" << std::endl;
                state = State::Nuke;
                return nukeStriker();
            }
            const auto dest = path[std::min(8, (int)path.size()-1)];

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(NUKE_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(dest.x - vehicles_[vId].pos.x);
            m.setY(dest.y - vehicles_[vId].pos.y);
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
            static bool waitingMoveQueue = false;
            if (waitingMoveQueue)
                break;
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

            const double visionCoeff = getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)vehicles_[vId].pos.x%32][(int)vehicles_[vId].pos.y%32]);
            const double visionSq = (vehicles_[vId].v.getVisionRange()*visionCoeff)*(vehicles_[vId].v.getVisionRange()*visionCoeff);
            V2d t{0., 0.};
            int cnt = 0;
            for (const auto &v: vehicles_) {
                if (v.second.isMine || v.second.v.getSquaredDistanceTo(vehicles_[vId].pos.x, vehicles_[vId].pos.y) > visionSq)
                    continue;
                t += v.second.pos;
                ++cnt;
            }
            if (!cnt) {
                return false;
            }
            t /= cnt;

            const double nSq = ctx_.game->getTacticalNuclearStrikeRadius()*ctx_.game->getTacticalNuclearStrikeRadius();
            for (const auto &v: vehicles_) {
                if (!v.second.isMine)
                    continue;
                if (v.second.v.getSquaredDistanceTo(t.x, t.y) < nSq) {
                    return false;
                }
            }

            Move m;
            m.setAction(ActionType::TACTICAL_NUCLEAR_STRIKE);
            m.setX(t.x);
            m.setY(t.y);
            m.setVehicleId(vId);
            queueMove(0, m, [&waitingMoveQueue](){ waitingMoveQueue = false; });
            waitingMoveQueue = true;
        }
        break;

        case State::End:
            return false;
    }

    return true;
}

bool MyStrategy::onlyGroupSelected(int g) const {
    for (const auto &vext: vehicles_) {
        if (!vext.second.isMine)
            continue;
        const auto &groups = vext.second.v.getGroups();
        if (std::find(groups.begin(), groups.end(), g) == groups.end()) {
            if (vext.second.v.isSelected()) {
                return false;
            }
        } else if (!vext.second.v.isSelected()) {
            return false;
        }
    }
    return true;
}
