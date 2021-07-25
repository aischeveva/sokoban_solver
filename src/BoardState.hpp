#ifndef BOARDSTATE_HPP
#define BOARDSTATE_HPP

#include <fstream>
#include <iostream>
#include <set>
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
    
    public:
        BoardState(){}
        BoardState(const std::vector<std::vector<Block>>& blocks, const std::vector<Box>& boxes);
        BoardState(const std::vector<std::vector<Block>>& blocks, const std::vector<Box>& boxes, BoardState* previous);

        unsigned int GetRows() const {return nRows_;}
        unsigned int GetColumns() const {return nCols_;}
        std::vector<std::vector<Block>> GetBlocks() const {return blocks_;}
        std::vector<Box> GetBoxes() const {return boxes_;}
        std::vector<Block> GetGoals();

        bool AddBlock(Block block);
        void AddBox(Box box);
        void AddPusher(Pusher pusher);
        void AddNeighbours();
        void ReadBoardState(std::ifstream& file);
        void ReadBoardState(std::string filename);
        void ReadBoardState(std::string filename, char level);
        void PrintBoardState();

        bool operator< (const BoardState& right) const { return boxes_.size() < right.GetBoxes().size();}
        friend bool operator== (const BoardState& b1, const BoardState& b2);

};

#endif