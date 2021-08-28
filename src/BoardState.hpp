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
 * 
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

        unsigned int GetRows() const {return nRows_;}
        unsigned int GetColumns() const {return nCols_;}
        int GetWeight() const {return weight_;}
        std::vector<std::vector<Block>> GetBlocks() const {return blocks_;}
        std::vector<Box> GetBoxes() const {return boxes_;}
        Pusher GetPusher() const {return pusher_;}
        std::vector<Block> GetGoals();
        BoardState GetPreviousState() const {return *previous_board_;}
        std::set<std::pair<int, int>> BlocksAvailableByPusher();

        bool AddBlock(Block block);
        void AddBox(Box box);
        void AddPusher(Pusher pusher);
        void AddNeighbours();
        void ReadBoardState(std::ifstream& file);
        void ReadBoardState(std::string filename);
        void ReadBoardState(std::string filename, char level);
        void PrintBoardState();
        void UpdateWeight(bool selected);

        bool operator< (const BoardState& right) const { return boxes_.size() < right.GetBoxes().size();}
        friend bool operator== (const BoardState& b1, const BoardState& b2);

};

#endif