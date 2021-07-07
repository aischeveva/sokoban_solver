#ifndef GAMESTATE_HPP
#define GAMESTATE_HPP

#include <vector>

#include "Box.hpp"
#include "Pusher.hpp"

class GameState{
    private:
        std::vector<Box> current_boxes_;
        GameState* previous_state_;
        GameState* next_state_;
    
    public:
        GameState(){}

        std::vector<Box> GetBoxes() const {return current_boxes_;}
        GameState* GetPreviousState() const {return previous_state_;}
        GameState* GetNextState() const {return next_state_;}
};
#endif