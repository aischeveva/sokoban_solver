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

void Board::addPusher(Pusher pusher){
    pusher_ = pusher; // add some error management here, if a pusher already exist
}

void Board::readBoard(std::string filename, char level){
    std::ifstream file;
    file.open(filename);
    if(!file) {
        std::cerr<<"Error: file could not be opened"<<std::endl;
        std::exit(1);
    }
    std::string line;
    std::getline(file, line);
    while(line[0] != level) std::getline(file, line);
    std::getline(file, line);
    int x = 0;
    while((line[0] == ' ' || line[0] == '#') && line.length() > 1){
        for (int i = 0; i < line.length(); i++){
            switch(line[i]) {
                case ' ': {
                    if(i==0 || 
                       blocks_[x][i - 1].GetType() == Outer ||
                       blocks_[x][i - 1].GetType() == Wall) addBlock(Block(x, i, Outer, false)); 
                    else addBlock(Block(x, i, Floor, false)); 
                    break;}
                case '#': addBlock(Block(x, i, Wall, false)); break;
                case '@': addBlock(Block(x, i, Floor, true)); addPusher(Pusher(x, i)); break;
                case '+': addBlock(Block(x, i, Goal, true)); addPusher(Pusher(x, i)); break;
                case '$': addBlock(Block(x, i, Floor, true)); addBox(Box(x, i)); break;
                case '*': addBlock(Block(x, i, Goal, true)); addBox(Box(x, i, true)); break;
                case '.': addBlock(Block(x, i, Goal, false));
            }
        }
        x++;
        std::getline(file, line);
    }
}