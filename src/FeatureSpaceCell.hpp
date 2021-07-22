#ifndef FEATURESPACECELL_HPP
#define FEATURESPACECELL_HPP

#include <algorithm>
#include <stack>
#include <vector>
#include <utility>

#include "Board.hpp"

/** 
 * \class FeatureSpaceCell
 * \brief Feature space to solve sokoban problem.
 * Feature space is the key element of the FESS algorithm.
 * Following the Festival solver, this implementation also utilizes a four-dimensional feature space.
 * Feature space includes packing, connectivity, room connectivity and out-of-plan features.
 * 
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 16/06/2021
 *
 */ 

/*TODO:
    * change board to a vector of projected domain states
    * packing plan: keep the order of the targets (i.e., which targets should be filled in first)
    *               packing advisor prioritizes moves that pack a box on a target that is first on the packing order list
    *               once box is packed there, the target is removed from the list to make way for the next top priority target
    * how feature space works: you initiate with root and corresponding active feature cell
    *                          expand the root by finding all possible moves
    *                          pick the best move and find a feature cell corresponding to the domain state after this move
    *                               it can be viewed as a vector of feature cells, each cell has a vector of domain states corresponding to it, each domain state has a reference to its parent state
    *                               root parent reference is null
    *                               once you found solution, rebuild the path by parent links
    *                          after that pick a feature cell from the active cells
    *                          and expand all unexpanded moves that correspond to this cell -- repeat with finding the best move 
    * about macro moves: pick a box, and treat it as an only moving object as long as you can move it
    *                    that would create a macro move basically
*/
class FeatureSpaceCell{
    private:
        Board board_;
        std::vector<std::vector<int>> rooms_;
        int packing_number_;
        std::vector<std::pair<int, Block>> packing_order_;
        int connectivity_;
        int room_connectivity_;
        int out_of_plan_;

    public:
        FeatureSpaceCell(){}
        FeatureSpaceCell(Board &board): board_(board), connectivity_(0), room_connectivity_(0), out_of_plan_(0) {}

        /* getters */
        int GetPacking() const {return packing_number_;}
        int GetConnectivity() const {return connectivity_;}
        int GetRoomConnectivity() const {return room_connectivity_;}
        int GetOutOfPlan() const {return out_of_plan_;}

        /* compute heuristics */
        void ComputePacking();
        void ComputeConnectivity();
        void ComputeOutOfPlan();

        /* preparational functions */
        void FindSinkRoom();
        void PrintRooms();

};

#endif