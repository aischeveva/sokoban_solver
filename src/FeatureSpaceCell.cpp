#include "FeatureSpaceCell.hpp"

void FeatureSpaceCell::ComputeConnectivity(){
    std::stack<std::pair<int, int>> stack;
    std::vector<std::vector<int>> visited(board_.GetRows(), std::vector<int>(board_.GetColumns()));
    std::vector<std::vector<int>> room(board_.GetRows(), std::vector<int>(board_.GetColumns()));
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();
    std::vector<Box> boxes = board_.GetBoxes();
    int nRows = board_.GetRows();
    int nCols = board_.GetColumns();

    for(unsigned int i = 1; i < nRows - 1; i++){
        for(unsigned int j = 1; j < nCols - 1; j++){
            BlockType type = blocks[i][j].GetType();
            if(visited[i][j] == 0 && (type == Floor || type == Goal) && !blocks[i][j].IsOccupied()){
                connectivity_++;
                stack.push(std::make_pair(i,j));
                while(!stack.empty()){
                    int x = stack.top().first;
                    int y = stack.top().second;
                    room[x][y] = connectivity_;
                    stack.pop();
                    if(visited[x][y] == 0){
                        visited[x][y] = 1;
                        std::vector<Block> neighbours = blocks[x][y].GetNeighbours();
                        for(auto neighbour = neighbours.begin(); neighbour != neighbours.end(); neighbour++){
                            type = (*neighbour).GetType();
                            if ((type == Floor || type == Goal) && !(*neighbour).IsOccupied()){
                                stack.push(std::make_pair((*neighbour).GetX(), (*neighbour).GetY()));
                            }
                        }
                    }
                }
            }
        }
    }

    rooms_ = room;
}

void FeatureSpaceCell::FindSinkRoom(){
    std::vector<int> candidate_rooms(connectivity_, 0);
    std::vector<Box> boxes = board_.GetBoxes();

    // find a sink room -- a basin with the most boxes in it at the beginning of the level
    for(auto box = boxes.begin(); box != boxes.end(); box++){
        int x = (*box).GetX();
        int y = (*box).GetY();
        std::vector<int> checked_rooms;
        if(rooms_[x-1][y] > 0) {
            checked_rooms.push_back(rooms_[x-1][y]);
            candidate_rooms[rooms_[x-1][y] - 1]++;
            }
        if(rooms_[x+1][y] > 0 && 
        std::find(checked_rooms.begin(), checked_rooms.end(), rooms_[x+1][y]) == checked_rooms.end()) {
            checked_rooms.push_back(rooms_[x+1][y]);
            candidate_rooms[rooms_[x+1][y] - 1]++;
            }
        if(rooms_[x][y-1] > 0 && 
        std::find(checked_rooms.begin(), checked_rooms.end(), rooms_[x][y-1]) == checked_rooms.end()) {
            checked_rooms.push_back(rooms_[x][y-1]);
            candidate_rooms[rooms_[x][y-1] - 1]++;
            }
        if(rooms_[x][y+1] > 0 && 
        std::find(checked_rooms.begin(), checked_rooms.end(), rooms_[x][y+1]) == checked_rooms.end()) {
            candidate_rooms[rooms_[x][y+1] - 1]++;
            }
    }
    int sink_room = 1 + std::max_element(candidate_rooms.begin(), candidate_rooms.end()) - candidate_rooms.begin();

    //update room numbers for box positions -- initially set to 0
    for(auto box = boxes.begin(); box != boxes.end(); box++){
        int x = (*box).GetX();
        int y = (*box).GetY();
        std::vector<int> rooms_around(connectivity_, 0);
        if(rooms_[x-1][y] > 0) {
            rooms_around[rooms_[x-1][y] - 1] = candidate_rooms[rooms_[x-1][y] - 1];
            }
        if(rooms_[x+1][y] > 0) {
            rooms_around[rooms_[x+1][y] - 1] = candidate_rooms[rooms_[x+1][y] - 1];
            }
        if(rooms_[x][y-1] > 0) {
            rooms_around[rooms_[x][y-1] - 1] = candidate_rooms[rooms_[x][y-1] - 1];
            }
        if(rooms_[x][y+1] > 0) {
            rooms_around[rooms_[x][y+1] - 1] = candidate_rooms[rooms_[x][y+1] - 1];
            }
        rooms_[x][y] = 1 + std::max_element(rooms_around.begin(), rooms_around.end()) - rooms_around.begin();
    }
    std::cout<<"Our suggested sink room is room number "<<sink_room<<std::endl;
}

void FeatureSpaceCell::PrintRooms(){
    for(unsigned int i = 0; i < rooms_.size(); i++){
        for(unsigned int j = 0; j < rooms_[i].size(); j++){
            std::cout<<rooms_[i][j];
        }
        std::cout<<std::endl;
    }
}

void FeatureSpaceCell::ComputePacking(){
    /// TODO
}

void FeatureSpaceCell::ComputeOutOfPlan(){
    /// TODO
}