#ifndef PACKINGPLANFEATURESPACECELL_HPP
#define PACKINGPLANFEATURESPACECELL_HPP

#include <cmath>
#include <vector>

#include "Board.hpp"
#include "Block.hpp"

class PackingPlanFeatureSpaceCell
{
private:
    int boxes_on_board_;
    int boxes_on_target_; 
    int distance_to_targets_; ///I don't know if it's fine to use Manhattan distance, but I'll start with it anyway
    Board current_board_;
public:
    PackingPlanFeatureSpaceCell(){}
    PackingPlanFeatureSpaceCell(Board &board):current_board_(board){}
    ~PackingPlanFeatureSpaceCell(){}

    int GetBoxesOnBoard() const {return boxes_on_board_;}
    int GetBoxesOnTarget() const {return boxes_on_target_;}
    int GetDistance() const {return distance_to_targets_;}

    void UpdateBoxesOnBoard();
    void UpdateBoxesOnTarget();
    void UpdateDistance();



    
};


#endif