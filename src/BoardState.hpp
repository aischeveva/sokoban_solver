#ifndef BOARDSTATE_HPP
#define BOARDSTATE_HPP

#include <fstream>
#include <iostream>
#include <set>
#include <stack>
#include <string>
#include <vector>

#include "Block.hpp"
#include "Box.hpp"
#include "Pusher.hpp"

/** 
 * \class BoardState
 * \brief Level map layout.
 * This class manages level map layout, including blocks and available boxes.
 * It represents a state of the board at some stage of the game, as well as keeps information about previous game state.
 * It also keeps track of the weight of the moves -- if the current state was picked by advisors, its weight is the previous state weight + 1. Otherwise, +100.
 * Initial board state doesn't have a previous state, so the pointer is null and its weight is 0.
 * When a board state created, by default it's not picked by the advisor. If this state is picked by some advisor later, the weight can be updated.
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 02/06/2021
 *
 */ 

class BoardState {
    private:
        unsigned int nRows_, nCols_;
        std::vector<std::vector<Block>> blocks_;
        std::vector<Box> boxes_;
        Pusher pusher_;
        BoardState* previous_board_;
        int weight_; //the weight of the move determined by the weight of the parent + 1 if it was picked by advisor or 100 otherwise
    
    public:
        BoardState(){}
        BoardState(std::vector<std::vector<Block>> blocks, std::vector<Box> boxes, Pusher pusher);
        BoardState(std::vector<std::vector<Block>> blocks, std::vector<Box> boxes, BoardState* previous, Pusher pusher, bool picked = false);

        /*Getters*/
        unsigned int GetRows() const {return nRows_;}
        unsigned int GetColumns() const {return nCols_;}
        int GetWeight() const {return weight_;}
        std::vector<std::vector<Block>> GetBlocks() const {return blocks_;}
        std::vector<Box> GetBoxes() const {return boxes_;}
        Pusher GetPusher() const {return pusher_;}
        std::vector<Block> GetGoals();
        BoardState GetPreviousState() const {return *previous_board_;}
        //returns a set of coordinates that can be reached by player from its current position with the current boxes' locations
        std::set<std::pair<int, int>> BlocksAvailableByPusher() const;

        /* Setting up the initial state when reading from file*/
        bool AddBlock(Block block);
        void AddBox(Box box);
        void AddPusher(Pusher pusher);
        void AddNeighbours();

        /* Reading from file and printing board state */
        void ReadBoardState(std::ifstream& file);
        void ReadBoardState(std::string filename);
        // function to read a level from file that contains more than one level
        void ReadBoardState(std::string filename, char level);
        void PrintBoardState();

        /* Updating weight of the state if needed */
        void UpdateWeight(bool selected);

        // meaningless comparison function required by queues
        bool operator< (const BoardState& right) const { return boxes_.size() < right.GetBoxes().size();}
        // currently two board states are considered identical if their boxes occupy the same positions on the boad
        // it should also check that the sets of locations available to players are the same
        friend bool operator== (const BoardState& b1, const BoardState& b2);

};

#endif