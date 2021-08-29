#ifndef PACKINGPLANFEATURESPACECELL_HPP
#define PACKINGPLANFEATURESPACECELL_HPP

#include <climits>
#include <cmath>
#include <vector>

#include "BoardState.hpp"
#include "Block.hpp"

/** 
 * \class PackingPlanFeatureSpaceCell
 * \brief A cell in the packing plan feature space.
 * Packing order computations utilize their own feature space 
 * that consists of boxes on boards, boxes on targets (goals) and the sum of shortest distances from each box to its closest goal block.
 * 
 * The only advisor used checks that the move doesn't worsen connectivity and it is either possible to push box in this position or there is a box there at the start of the level.
 * Although the advisor is described here, it is implemented in FeatureSpaceCell.cpp, where the packing plan is computed.
 * 
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 22/07/2021
 *
 */ 

class PackingPlanFeatureSpaceCell
{
private:
    int boxes_on_board_;
    int boxes_on_target_; 
    int distance_to_targets_; ///I don't know if it's fine to use Manhattan distance, but I'll start with it anyway
    std::vector<BoardState*> corresponding_boards_;
public:
    PackingPlanFeatureSpaceCell(){}
    PackingPlanFeatureSpaceCell(BoardState* board);
    ~PackingPlanFeatureSpaceCell(){}

    int GetBoxesOnBoard() const {return boxes_on_board_;}
    int GetBoxesOnTarget() const {return boxes_on_target_;}
    int GetDistance() const {return distance_to_targets_;}
    std::vector<BoardState*> GetStates() const {return corresponding_boards_;}

    void AddBoard(BoardState* board);
    void UpdateBoxesOnBoard();
    void UpdateBoxesOnTarget();
    void UpdateDistance();

    // two cells in the feature space are equal if all their feature values are equal
    friend bool operator== (const PackingPlanFeatureSpaceCell &p1, const PackingPlanFeatureSpaceCell &p2);



    
};


#endif