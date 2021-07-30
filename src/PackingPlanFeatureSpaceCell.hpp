#ifndef PACKINGPLANFEATURESPACECELL_HPP
#define PACKINGPLANFEATURESPACECELL_HPP

#include <climits>
#include <cmath>
#include <vector>

#include "BoardState.hpp"
#include "Block.hpp"

class PackingPlanFeatureSpaceCell
{
private:
    int boxes_on_board_;
    int boxes_on_target_; 
    int distance_to_targets_; ///I don't know if it's fine to use Manhattan distance, but I'll start with it anyway
    std::vector<BoardState> corresponding_boards_;
public:
    PackingPlanFeatureSpaceCell(){}
    PackingPlanFeatureSpaceCell(BoardState &board);
    ~PackingPlanFeatureSpaceCell(){}

    int GetBoxesOnBoard() const {return boxes_on_board_;}
    int GetBoxesOnTarget() const {return boxes_on_target_;}
    int GetDistance() const {return distance_to_targets_;}
    std::vector<BoardState> GetStates() const {return corresponding_boards_;}

    void AddBoard(BoardState board);
    void UpdateBoxesOnBoard();
    void UpdateBoxesOnTarget();
    void UpdateDistance();

    friend bool operator== (const PackingPlanFeatureSpaceCell &p1, const PackingPlanFeatureSpaceCell &p2);



    
};


#endif