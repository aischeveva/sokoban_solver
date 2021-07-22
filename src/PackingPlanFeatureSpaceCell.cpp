#include "PackingPlanFeatureSpace.hpp"


void PackingPlanFeatureSpace::UpdateBoxesOnBoard(){
    boxes_on_board_ = current_board_.GetBoxes().size();
}

void PackingPlanFeatureSpace::UpdateBoxesOnTarget(){
    for(auto box : current_board_.GetBoxes()){
        if(box.IsDelivered()) boxes_on_target_++;
    }
}

void PackingPlanFeatureSpace::UpdateDistance(){
    std::vector<Block> targets = current_board_.GetGoals();
    for(auto box : current_board_.GetBoxes()){
        int min_distance = INT_MAX;
        for(auto target : targets){
            int distance = abs(box.GetX() - target.GetX()) + abs(box.GetY() - target.GetY());
            if (distance < min_distance) min_distance = distance;
        }
        distance_to_targets_ += min_distance;

    }
}