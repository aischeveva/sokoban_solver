#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "FeatureSpaceCell.hpp"

class Solver{
    private:
        BoardState* initial_state;
        std::vector<BoardState*> move_tree;
        std::vector<FeatureSpaceCell> feature_space;
    public:
        Solver(){}

        // advisors
        bool PackingAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        bool ConnectivityAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        bool RoomConnectivityAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        bool HotspotsAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        bool ExplorerAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        bool OpenerAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
};

#endif