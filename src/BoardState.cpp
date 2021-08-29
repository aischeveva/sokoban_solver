#include "BoardState.hpp"

BoardState::BoardState(std::vector<std::vector<Block>> blocks, std::vector<Box> boxes, Pusher pusher){
    blocks_ = std::move(blocks);
    boxes_ = std::move(boxes);
    nRows_ = blocks_.size();
    nCols_ = blocks_[0].size();
    previous_board_ = NULL;
    weight_ = 0;
    pusher_=pusher;
}

//by default the state is not picked by the advisor unless stated otherwise
BoardState::BoardState(std::vector<std::vector<Block>> blocks, std::vector<Box> boxes, BoardState* previous, Pusher pusher, bool picked){
    blocks_ = std::move(blocks);
    boxes_ = std::move(boxes);
    nRows_ = blocks_.size();
    nCols_ = blocks_[0].size();
    previous_board_ = previous;
    pusher_=pusher;
    if (picked) weight_ = previous_board_->GetWeight() + 1;
    else weight_ = previous_board_->GetWeight() + 100;
}

std::vector<Block> BoardState::GetGoals(){
    std::vector<Block> targets;
    for(int i = 0; i < nRows_; i++){
        for(int j = 0; j < nCols_; j++){
            if (blocks_[i][j].GetType() == Goal) targets.push_back(blocks_[i][j]);
        }
    }
    return targets;
}

std::set<std::pair<int, int>> BoardState::BlocksAvailableByPusher() const{
    // runs a dfs to find all locations accessible to the player from the current position with the current boxes on board
    std::stack<std::pair<int, int>> stack;
    std::vector<std::vector<int>> visited(nRows_, std::vector<int>(nCols_));
    std::set<std::pair<int, int>> accessible;

    stack.push(std::make_pair(pusher_.GetX(), pusher_.GetY()));
    while(!stack.empty()){
        int x = stack.top().first;
        int y = stack.top().second;
        stack.pop();
        if(visited[x][y] == 0){
            visited[x][y] = 1;
            std::vector<Block> neighbours = blocks_[x][y].GetNeighbours();
            for(auto neighbour = neighbours.begin(); neighbour != neighbours.end(); neighbour++){
                auto type = (*neighbour).GetType();
                int x = (*neighbour).GetX();
                int y = (*neighbour).GetY();
                if ((type == Floor || type == Goal) && !blocks_[x][y].IsOccupied()){
                    accessible.insert(std::make_pair(x, y));
                    stack.push(std::make_pair(x, y));
                }
            }
        }
    }

    return accessible;
}

bool BoardState::AddBlock(Block block){
    unsigned int x = block.GetX();
    unsigned int y = block.GetY();
    // if the current number of rows is less than the x coordinate of the block
    // add a new row with the block being the first one in it
    if (blocks_.size() < x + 1){
        std::vector<Block> newRow = {block};
        nRows_++;
        blocks_.push_back(newRow);
        return true;
    } else if (blocks_[x].size() < y + 1){
        // when adding to an existing row, check that it's size is smaller than the y coordiante of the block
        // if it is, add a new block to the end of the row
        blocks_[x].push_back(block);
        if (nCols_ < blocks_[x].size()){
            nCols_++;
        }
        return true;
    }
    // otherwise exit with error, because this block already exist
    // normally shouldn't happen
    return false;
}

void BoardState::AddBox(Box box){
    boxes_.push_back(box);
}

void BoardState::AddPusher(Pusher pusher){
    pusher_ = pusher; // add some error management here, if a pusher already exist
}

void BoardState::AddNeighbours(){
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

void BoardState::ReadBoardState(std::ifstream& file){
    std::string line;
    std::getline(file, line);
    int x = 0;
    nRows_ = 0; nCols_ = 0;
    // read the file
    while(line.length() > 1){
        for (unsigned int i = 0; i < line.length(); i++){
            switch(line[i]) {
                case ' ': {
                    if(i==0 || x==0 ||
                       blocks_[x][i - 1].GetType() == Outer) AddBlock(Block(x, i, Outer, false)); 
                    else AddBlock(Block(x, i, Floor, false)); 
                    break;}
                case '#': AddBlock(Block(x, i, Wall, true)); break;
                case '@': AddBlock(Block(x, i, Floor, false)); AddPusher(Pusher(x, i)); break;
                case '+': AddBlock(Block(x, i, Goal, false)); AddPusher(Pusher(x, i)); break;
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

void BoardState::ReadBoardState(std::string filename){
    std::ifstream file;
    file.open(filename);
    if(!file) {
        std::cerr<<"Error: file could not be opened"<<std::endl;
        std::exit(1);
    }
    ReadBoardState(file);
    
}

void BoardState::ReadBoardState(std::string filename, char level){
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

void BoardState::PrintBoardState(){
    /* for(unsigned int i = 0; i < blocks_.size(); i++){
        for(unsigned int j = 0; j < blocks_[i].size(); j++){
            std::cout<<blocks_[i][j];
        }
        std::cout<<std::endl;
    } */
    for(auto box : boxes_){
        std::cout<<"Box at ("<<box.GetX()<<", "<<box.GetY()<<")"<<std::endl;
    }
}

void BoardState::UpdateWeight(bool selected){
    if(selected){
        weight_ = previous_board_->GetWeight() + 1;
    }else{
        weight_ = previous_board_->GetWeight() + 100;
    }
}

bool operator== (const BoardState& b1, const BoardState& b2){
    std::set<std::pair<int, int>> b1_boxes;
    std::set<std::pair<int, int>> b2_boxes;
    std::vector<Box> b1b = b1.GetBoxes();
    std::vector<Box> b2b = b2.GetBoxes();
    if (b1b.size() != b2b.size()) return false;
    for(unsigned int i = 0; i < b1b.size(); i++){
        b1_boxes.insert(std::make_pair(b1b[i].GetX(), b1b[i].GetY()));
        b2_boxes.insert(std::make_pair(b2b[i].GetX(), b2b[i].GetY()));
    }
    return (b1_boxes == b2_boxes) && (b1.BlocksAvailableByPusher() == b2.BlocksAvailableByPusher());
}