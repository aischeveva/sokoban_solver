#include "Board.hpp"

Board::Board(int nRows, int nCols, std::vector<std::vector<Block>>& blocks, std::vector<Box>& boxes){
    nRows_ = nRows; nCols_ = nCols;
    blocks_ = blocks; boxes_ = boxes;
}

bool Board::AddBlock(Block block){
    unsigned int x = block.GetX();
    unsigned int y = block.GetY();
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

void Board::AddBox(Box box){
    boxes_.push_back(box);
}

void Board::AddPusher(Pusher pusher){
    pusher_ = pusher; // add some error management here, if a pusher already exist
}

void Board::AddNeighbours(){
    for(unsigned int i = 0; i < nRows_; i++){
        for(unsigned int j = 0; j < nCols_; j++){
            //std::cout<<"Processing block "<<i<<" "<<j<<std::endl;
            BlockType type = blocks_[i][j].GetType();
            if(type == Floor || type == Goal){
                if(i > 0) blocks_[i][j].AddNeighbour(blocks_[i-1][j]);
                if(i < (nRows_ - 2)) blocks_[i][j].AddNeighbour(blocks_[i+1][j]);
                if(j > 0) blocks_[i][j].AddNeighbour(blocks_[i][j-1]);
                if(j < (nCols_ - 2)) blocks_[i][j].AddNeighbour(blocks_[i][j+1]);
            }
        }
    }
}

void Board::ReadBoard(std::ifstream& file){
    std::string line;
    std::getline(file, line);
    int x = 0;

    // read the file
    while(line.length() > 1){
        for (unsigned int i = 0; i < line.length(); i++){
            switch(line[i]) {
                case ' ': {
                    if(i==0 || x==0 ||
                       blocks_[x][i - 1].GetType() == Outer) AddBlock(Block(x, i, Outer, false)); 
                    else AddBlock(Block(x, i, Floor, false)); 
                    break;}
                case '#': AddBlock(Block(x, i, Wall, false)); break;
                case '@': AddBlock(Block(x, i, Floor, true)); AddPusher(Pusher(x, i)); break;
                case '+': AddBlock(Block(x, i, Goal, true)); AddPusher(Pusher(x, i)); break;
                case '$': AddBlock(Block(x, i, Floor, true)); AddBox(Box(x, i)); break;
                case '*': AddBlock(Block(x, i, Goal, true)); AddBox(Box(x, i, true)); break;
                case '.': AddBlock(Block(x, i, Goal, false));
            }
        }
        x++;
        std::getline(file, line);
    }

    // fill in row by row outer spaces on the right if any
    for(unsigned int i = 0; i < blocks_.size(); i++){
        unsigned int j = blocks_[i].size();
        while (j < nCols_){
            AddBlock(Block(i, j, Outer, false));
            j++;
        }
    }

    // fix floor tiles into outer space: first top to bottom, then bottom to top
    // would be nice to come up with a smarter way to do it
    for(unsigned int i = 1; i < blocks_.size(); i++){
        for(unsigned int j = 0; j < blocks_[i].size(); j++){
            if(blocks_[i-1][j].GetType() == Outer && blocks_[i][j].GetType() == Floor){
                blocks_[i][j].ChangeType(Outer);
            }
            if(i == (nRows_ - 1) && blocks_[i][j].GetType() == Floor){
                blocks_[i][j].ChangeType(Outer);
            }
        }
    }
    for(int i = nRows_ - 2; i >= 0; i--){
        for(unsigned int j = 0; j < blocks_[i].size(); j++){
            if(blocks_[i+1][j].GetType() == Outer && blocks_[i][j].GetType() == Floor){
                blocks_[i][j].ChangeType(Outer);
            }
        }
    }
}

void Board::ReadBoard(std::string filename){
    std::ifstream file;
    file.open(filename);
    if(!file) {
        std::cerr<<"Error: file could not be opened"<<std::endl;
        std::exit(1);
    }
    ReadBoard(file);
    
}

void Board::ReadBoard(std::string filename, char level){
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
        for (unsigned int i = 0; i < line.length(); i++){
            switch(line[i]) {
                case ' ': {
                    if(i==0 || 
                       blocks_[x][i - 1].GetType() == Outer ||
                       line.find_last_of('#') < i ) AddBlock(Block(x, i, Outer, false)); 
                    else AddBlock(Block(x, i, Floor, false)); 
                    break;}
                case '#': AddBlock(Block(x, i, Wall, false)); break;
                case '@': AddBlock(Block(x, i, Floor, true)); AddPusher(Pusher(x, i)); break;
                case '+': AddBlock(Block(x, i, Goal, true)); AddPusher(Pusher(x, i)); break;
                case '$': AddBlock(Block(x, i, Floor, true)); AddBox(Box(x, i)); break;
                case '*': AddBlock(Block(x, i, Goal, true)); AddBox(Box(x, i, true)); break;
                case '.': AddBlock(Block(x, i, Goal, false));
            }
        }
        x++;
        std::getline(file, line);
    }
}

void Board::PrintBoard(){
    for(unsigned int i = 0; i < blocks_.size(); i++){
        for(unsigned int j = 0; j < blocks_[i].size(); j++){
            std::cout<<blocks_[i][j];
        }
        std::cout<<std::endl;
    }
}