#include <vector>
#include <numeric>
#include <unordered_map>
#include "Clusterize.h"

using namespace model;

namespace {
    Clusterize::ClusterList clusterizeImpl(const Context &ctx, unsigned cnt, VehicleType vt, bool isMine, double dist) {
        const double distSq = dist*dist;
        std::vector<VId> ids;
        ids.reserve(cnt);
        for (const auto &vext: *ctx.vehicleById) {
            if (vext.second.isMine == isMine && vext.second.v.getType() == vt) {
                ids.push_back(vext.first);
            }
        }

        std::vector<int> clnum(cnt);
        std::iota(clnum.begin(), clnum.end(), 1);
        for (unsigned i = 0; i < cnt; ++i) {
            for (unsigned j = i+1; j < cnt; ++j) {
                if (clnum[j] == clnum[i]) {
                    continue;
                }
                if ((*ctx.vehicleById)[ids[i]].v.getSquaredDistanceTo((*ctx.vehicleById)[ids[j]].v) < distSq) {
                    const int oldidx = clnum[j];
                    for (unsigned k = 0; k < cnt; ++k) {
                        if (clnum[k] == oldidx) {
                            clnum[k] = clnum[i];
                        }
                    }
                }
            }
        }

        std::unordered_map<int, int> clnum_map;
        for (int n: clnum) {
            if (!clnum_map.count(n)) {
                clnum_map.emplace(n, clnum_map.size());
            }
        }
        Clusterize::ClusterList clusterList(clnum_map.size());
        for (auto &c: clusterList) {
            c.type = vt;
        }
        for (unsigned int i=0; i < cnt; ++i) {
            clusterList[clnum_map[clnum[i]]].set.insert(ids[i]);
        }

        return clusterList;
    }
}

namespace Clusterize {

    ClusterLists* clusterize(const Context &ctx, double dist)
    {
        ClusterLists *res = new ClusterLists;

        // count units of each type
        unsigned my_cnt[(int)VehicleType::_COUNT_] = {0},
                 en_cnt[(int)VehicleType::_COUNT_] = {0};
        for (const auto &vext: *ctx.vehicleById) {
            if (vext.second.isMine) {
                ++my_cnt[(int)vext.second.v.getType()];
            } else {
                ++en_cnt[(int)vext.second.v.getType()];
            }
        }

        // clusterize for each type
        for (int i=0; i < (int)VehicleType::_COUNT_; ++i) {
            auto cl = clusterizeImpl(ctx, my_cnt[i], (VehicleType)i, true, dist);
            std::copy(std::make_move_iterator(cl.begin()),
                      std::make_move_iterator(cl.end()),
                      std::back_inserter(res->myClusters));
            cl = clusterizeImpl(ctx, en_cnt[i], (VehicleType)i, false, dist);
            std::copy(std::make_move_iterator(cl.begin()),
                      std::make_move_iterator(cl.end()),
                      std::back_inserter(res->enemyClusters));
        }

        return res;
    }

}
