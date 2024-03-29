#include "PackingPlanFeatureSpaceCell.hpp"

PackingPlanFeatureSpaceCell::PackingPlanFeatureSpaceCell(BoardState* board){
    corresponding_boards_.push_back(board);
    boxes_on_board_ = 0;
    boxes_on_target_ = 0;
    distance_to_targets_ = 0;
    UpdateBoxesOnBoard();
    UpdateBoxesOnTarget();
    UpdateDistance();
}

void PackingPlanFeatureSpaceCell::AddBoard(BoardState* board){
    corresponding_boards_.push_back(board);
}

void PackingPlanFeatureSpaceCell::UpdateBoxesOnBoard(){
    boxes_on_board_ = corresponding_boards_[0]->GetBoxes().size();
}

void PackingPlanFeatureSpaceCell::UpdateBoxesOnTarget(){
    for(auto goal : corresponding_boards_[0]->GetGoals()){
        if(goal.IsOccupied()) boxes_on_target_++;
    }
}

void PackingPlanFeatureSpaceCell::UpdateDistance(){
    std::vector<Block> targets = corresponding_boards_[0]->GetGoals();
    for(auto box : corresponding_boards_[0]->GetBoxes()){
        int min_distance = INT_MAX;
        for(auto target : targets){
            if(!target.IsOccupied() || (box.GetX() == target.GetX() && box.GetY() == target.GetY())){
                int distance = abs(box.GetX() - target.GetX()) + abs(box.GetY() - target.GetY());
                if (distance < min_distance) min_distance = distance;
            }
        }
        distance_to_targets_ += min_distance;

    }
}

bool operator== (const PackingPlanFeatureSpaceCell &p1, const PackingPlanFeatureSpaceCell &p2){
    return (p1.GetBoxesOnBoard() == p2.GetBoxesOnBoard() && p1.GetBoxesOnTarget() == p2.GetBoxesOnTarget() && p1.GetDistance() == p2.GetDistance());
}