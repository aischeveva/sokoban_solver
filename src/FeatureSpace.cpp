#include "FeatureSpace.hpp"

void FeatureSpace::ComputeConnectivity(){
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
    std::cout<<"hoops"<<std::endl;
    for(auto box = boxes.begin(); box != boxes.end(); box++){
        int x = (*box).GetX();
        int y = (*box).GetY();
        std::vector<int> rooms;
        if(room[x-1][y] > 0) rooms.push_back(room[x-1][y]);
        if(room[x+1][y] > 0) rooms.push_back(room[x+1][y]);
        if(room[x][y-1] > 0) rooms.push_back(room[x][y-1]);
        if(room[x][y+1] > 0) rooms.push_back(room[x][y+1]);

        if(rooms.size() >= 1 && !std::equal(rooms.begin() + 1, rooms.end(), rooms.begin())){
            room_connectivity_++;
        }
    }

}