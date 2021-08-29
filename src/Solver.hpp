#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "FeatureSpaceCell.hpp"

/** 
 * \class Solver
 * \brief A solver instance to solve the level as a whole.
 * 
 * A template for a solver class that needs to be filled with actual functionality.
 * The algorithm could be an adopted and more complex version of the ComputePackingOrder function in FeatureSpaceCell.cpp
 * (an outline of the FESS algorithm can be found from FeatureSpaceCell.hpp as well).
 * 
 * These header and implementation files are currently not included in the build and not tested to any extent.
 * Connectivity, Room Connectivity and Out of Plan advisors are written, other advisors (Packing, Hotspots, Explorer, Opener) have placeholders.
 * Solver routine is missing completely.
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 25/08/2021
 *
 */ 

class Solver{
    private:
        BoardState* initial_state_;
        std::vector<BoardState*> move_tree_;
        std::vector<FeatureSpaceCell> feature_space_;
        std::vector<std::vector<bool>> hotspots_; // to compute hotspots in the future -- might be good to move it to the feature space cell instead
    public:
        Solver(){}

        /* Advisors */
        // picks a move that delivers a box to the current goal block in the packing plan
        bool PackingAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        // picks a move that reduces the number of disconnected areas on the board
        bool ConnectivityAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        // picks a move that restores a "broken edge" between two rooms reducing the number of disconnected rooms 
        bool RoomConnectivityAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        // picks a move that removes a hotspot: a box that is preventing other boxes from being delivered
        bool HotspotsAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        // picks a move that openes a path to a free square (in majority of cases will agree with connectivity advisor)
        bool ExplorerAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        // picks a move that opens up a hotspot by pushing away other boxes around it if it doesn't worsen the connectivity
        bool OpenerAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
        // picks a move that reduces the number of out of plan boxes
        bool OutOfPlanAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
};

#endif