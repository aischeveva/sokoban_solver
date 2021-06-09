#ifndef BOARD_HPP
#define BOARD_HPP

#include "Block.hpp"
#include "Box.hpp"
#include <vector>
#include <iostream>
#include <string>

/** 
 * \class Board
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

class Board {
    private:
        int nRows_, nCols_;
        std::vector<std::vector<Block>> blocks_;
        std::vector<Box> boxes_;
    
    public:
        Board();
        Board(int nRows, int nCols, std::vector<std::vector<Block>>& blocks, std::vector<Box>& boxes);

        int GetRows() const {return nRows_;}
        int GetColumns() const {return nCols_;}
        std::vector<std::vector<Block>> GetBlocks() const {return blocks_;}
        std::vector<Box> GetBoxes() const {return boxes_;}

        bool addBlock(Block block);
        void addBox(Box box);
        void readBoard(std::string filename, int level);

};

#endif