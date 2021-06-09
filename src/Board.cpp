#include "Board.hpp"

bool Board::addBlock(Block block){
    int x = block.GetX();
    int y = block.GetY();
    if (blocks_.size() < x + 1){
        std::vector<Block> newRow = {block};
        nRows_++;
        blocks_.push_back(newRow);
        return true;
    } else if (blocks_[x].size() < y + 1){
        blocks_[x].push_back(block);
        if (nCols_ < blocks_[x].size()){
            nCols_++;
        }
        return true;
    }
    return false;
}

void Board::addBox(Box box){
    boxes_.push_back(box);
}

void Board::readBoard(std::string filename, int level){}