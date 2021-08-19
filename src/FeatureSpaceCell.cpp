#include "FeatureSpaceCell.hpp"

void FeatureSpaceCell::ComputePackingNumber(){
    auto goals = board_.GetGoals();
    packing_number_ = 0;
    for(auto goal : goals){
        if(goal.IsOccupied()) packing_number_++;
    }
}

void FeatureSpaceCell::ComputePackingOrder(){
    std::vector<std::vector<Block>> starting_blocks = board_.GetBlocks();

    //set up initial position, all boxes on targets
    std::vector<Block> goals = board_.GetGoals();
    std::vector<Box> boxes_on_goals;
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();
    // free initially occupied blocks
    Pusher pusher = board_.GetPusher();
    blocks[pusher.GetX()][pusher.GetY()].Free();
    for(auto box : board_.GetBoxes()){
        blocks[box.GetX()][box.GetY()].Free();
    }
    // put new boxes on goal squares, occupy those squares
    for (auto goal : goals){
        boxes_on_goals.push_back(Box(goal.GetX(), goal.GetY(), true));
        blocks[goal.GetX()][goal.GetY()].Occupy();
    }
    BoardState initial_board = BoardState(blocks, boxes_on_goals);
    FeatureSpaceCell initial_fsc = FeatureSpaceCell(initial_board);
    initial_fsc.ComputeConnectivity();
    int initial_connectivity = initial_fsc.GetConnectivity();
    //set up list of feature cells in feature space
    std::vector<PackingPlanFeatureSpaceCell> feature_space;
    //compute initial feature space cell and add it to the list
    PackingPlanFeatureSpaceCell initial_cell = PackingPlanFeatureSpaceCell(&initial_board);
    feature_space.push_back(initial_cell);

    // set up the move "tree" -- a map (state -> weight) should be sufficient probably
    // push back the initial domain state
    std::vector<BoardState> move_tree;
    move_tree.push_back(initial_board);

    // vector to hold all states corresponding to current cell in feature space
    std::vector<BoardState*> current_states; 
    current_states = initial_cell.GetStates();
    PackingPlanFeatureSpaceCell current_cell = initial_cell;

    //custom compare function to keep track of best moves:
    //1) weight
    //2) if the box can be removed, pick this move. If more than one box can be removed, pick the one with higher distance
    //3) otherwise if a box can be moved from target, move it. Same as before, with several moves pick the one with higher distance
    //4) if 1) and 2) are not possible, just try to maximize the distance.
	auto cmp = [](std::pair<BoardState*, PackingPlanFeatureSpaceCell> left, std::pair<BoardState*, PackingPlanFeatureSpaceCell> right) { 
		if(left.first->GetWeight() > right.first->GetWeight()) return true;
        else if(left.second.GetBoxesOnBoard() < right.second.GetBoxesOnBoard()) return false; 
		else if(left.second.GetBoxesOnTarget() < right.second.GetBoxesOnTarget()) return false;
		else if(left.second.GetDistance() > right.second.GetDistance()) return false;
		return true;

	 };

    // TODO: have better definition on when the situation is solved
    //search feature space while all the boxes are not deleted
    //or the maximum number of iterations is not reached (NOT ADDED YET)
    int iteration_count = 0;
    while(current_cell.GetBoxesOnBoard() != 0 && iteration_count < 150){
        iteration_count++;
        // get all states that correspond to the current cell of the feature space
        current_states = current_cell.GetStates();
        //find all possible moves from the current states
        std::priority_queue<std::pair<BoardState*, PackingPlanFeatureSpaceCell>, std::vector<std::pair<BoardState*, PackingPlanFeatureSpaceCell>>, decltype(cmp)> possible_moves(cmp);
        
        //TODO: check that local current state and current state are used correctly
        for(auto &current_state : current_states){
            //look at moving one box at a time
            //auto parent_state = std::find(move_tree.begin(), move_tree.end(), current_state);
            unsigned int number_of_boxes = current_state->GetBoxes().size();
            for(unsigned int i = 0; i < number_of_boxes; i++){
                // moves available for i_th box only
                // tries to find all available macro moves for this box
                //keep locally explored moves
                std::vector<BoardState> explored_moves;
                bool box_removed = false;
                // TODO: should take into account if the box can be pushed in this position or if it's its starting position
                // keep a pair -- a state and if it can be pushed in this position
                std::stack<std::pair<BoardState*, bool>> available_moves;
                BoardState* current_macro_move = current_state;
                available_moves.push(std::make_pair(current_macro_move, true));
                while(!available_moves.empty() && !box_removed){
                    std::pair<BoardState*, bool> top = available_moves.top();
                    bool possible_to_push;
                    std::tie(current_macro_move, possible_to_push) = top;
                    available_moves.pop();
                    //if the move hasn't been explored yet
                    if(std::find(explored_moves.begin(), explored_moves.end(), *current_macro_move) == explored_moves.end()){
                        // mark move as checked
                        explored_moves.push_back(*current_macro_move);
                        blocks = current_macro_move->GetBlocks();
                        int x = current_macro_move->GetBoxes()[i].GetX();
                        int y = current_macro_move->GetBoxes()[i].GetY();
                        // if a box reached a sink room, remove it from the board and stop exploration for this box
                        // for now let's assume that is the only move that would be picked by advisor
                        // TODO: but in reality need to add connectivity check
                        std::pair<int, int> coord = std::make_pair(x, y);
                        if(room_by_coord_.count(coord) && room_by_coord_[coord] == sink_room_){
                            std::vector<Box> new_boxes = current_macro_move->GetBoxes();
                            std::vector<std::vector<Block>> new_blocks = current_macro_move->GetBlocks();
                            new_boxes.erase(new_boxes.begin() + i);
                            new_blocks[x][y].Free();
                            current_macro_move = new BoardState(new_blocks, new_boxes, current_state);
                            // add it to the possible moves if it's not in the move tree already
                            if(std::find(move_tree.begin(), move_tree.end(), *current_macro_move) == move_tree.end()){
                                //check if the move would be selected by advisor
                                FeatureSpaceCell connectivity_check = FeatureSpaceCell(*current_macro_move);
                                connectivity_check.ComputeConnectivity();
                                if(connectivity_check.GetConnectivity() <= initial_connectivity && possible_to_push) current_macro_move->UpdateWeight(true);
                                
                                possible_moves.push(std::make_pair(current_macro_move, PackingPlanFeatureSpaceCell(current_macro_move)));
                            }
                            box_removed = true;
                        } else {
                            // add current move to moves possible from the current state if it's not in the move tree already
                            if(std::find(move_tree.begin(), move_tree.end(), *current_macro_move) == move_tree.end()){
                                //check if the move would be selected by advisor
                                FeatureSpaceCell connectivity_check = FeatureSpaceCell(*current_macro_move);
                                connectivity_check.ComputeConnectivity();
                                if(connectivity_check.GetConnectivity() <= initial_connectivity && possible_to_push) current_macro_move->UpdateWeight(true);
                                
                                possible_moves.push(std::make_pair(current_macro_move, PackingPlanFeatureSpaceCell(current_macro_move)));
                            }
                            //otherwise check if there is enough space to pull the box in each four directions
                            std::vector<Box> new_boxes;
                            std::vector<std::vector<Block>> new_blocks;
                            //check North
                            if(!blocks[x-1][y].IsOccupied() && !blocks[x-2][y].IsOccupied()){
                                new_boxes = current_macro_move->GetBoxes();
                                new_blocks = current_macro_move->GetBlocks();
                                new_blocks[x][y].Free();
                                new_blocks[x-1][y].Occupy();
                                new_boxes[i].Move(North);
                                BoardState* new_state = new BoardState(new_blocks, new_boxes, current_state);
                                bool possible = false;
                                //check if it's possible to push box in this position
                                //do that only if the current move is actually possible
                                if(possible_to_push){
                                    if(!blocks[x+1][y].IsOccupied()) possible = true;
                                }
                                //otherwise check if the new position has a box in the starting position
                                if(starting_blocks[x-1][y].IsOccupied()){
                                    possible = true;
                                }
                                available_moves.push(std::make_pair(new_state, possible));
                            }
                            //check South
                            if(!blocks[x+1][y].IsOccupied() && !blocks[x+2][y].IsOccupied()){
                                new_boxes = current_macro_move->GetBoxes();
                                new_blocks = current_macro_move->GetBlocks();
                                new_blocks[x][y].Free();
                                new_blocks[x+1][y].Occupy();
                                new_boxes[i].Move(South);
                                BoardState* new_state = new BoardState(new_blocks, new_boxes, current_state);
                                bool possible = false;
                                //check if it's possible to push box in this position
                                //do that only if the current move is actually possible
                                if(possible_to_push){
                                    if(!blocks[x-1][y].IsOccupied()) possible = true;
                                }
                                //otherwise check if the new position has a box in the starting position
                                if(starting_blocks[x+1][y].IsOccupied()){
                                    possible = true;
                                }
                                available_moves.push(std::make_pair(new_state, possible));
                            }
                            //check East
                            if(!blocks[x][y+1].IsOccupied() && !blocks[x][y+2].IsOccupied()){
                                new_boxes = current_macro_move->GetBoxes();
                                new_blocks = current_macro_move->GetBlocks();
                                new_blocks[x][y].Free();
                                new_blocks[x][y+1].Occupy();
                                new_boxes[i].Move(East);
                                BoardState* new_state = new BoardState(new_blocks, new_boxes, current_state);
                                bool possible = false;
                                //check if it's possible to push box in this position
                                //do that only if the current move is actually possible
                                if(possible_to_push){
                                    if(!blocks[x][y-1].IsOccupied()) possible = true;
                                }
                                //otherwise check if the new position has a box in the starting position
                                if(starting_blocks[x][y+1].IsOccupied()){
                                    possible = true;
                                }
                                available_moves.push(std::make_pair(new_state, possible));
                            }
                            //check West
                            if(!blocks[x][y-1].IsOccupied() && !blocks[x][y-2].IsOccupied()){
                                new_boxes = current_macro_move->GetBoxes();
                                new_blocks = current_macro_move->GetBlocks();
                                new_blocks[x][y].Free();
                                new_blocks[x][y-1].Occupy();
                                new_boxes[i].Move(West);
                                BoardState* new_state = new BoardState(new_blocks, new_boxes, current_state);
                                bool possible = false;
                                //check if it's possible to push box in this position
                                //do that only if the current move is actually possible
                                if(possible_to_push){
                                    if(!blocks[x][y+1].IsOccupied()) possible = true;
                                }
                                //otherwise check if the new position has a box in the starting position
                                if(starting_blocks[x][y-1].IsOccupied()){
                                    possible = true;
                                }
                                available_moves.push(std::make_pair(new_state, possible));
                            }
                        }
                    }
                }
            }
        }
        //pick the best move according to features and distance:
        // if the queue was defined correctly (hopefully) the best move is the first one in the queue of possible moves
        BoardState* best_move = possible_moves.top().first;
        PackingPlanFeatureSpaceCell best_cell = possible_moves.top().second;

        //add a move to the tree and parent connection
        move_tree.push_back(*best_move);
 
        //check if the cell in feature space was already discovered:
        //if yes, add the move to the list of the domain states that project on this cell
        //if no, add new feature cell to the feature space
        auto existing_cell = std::find(feature_space.begin(), feature_space.end(), best_cell);
        if(existing_cell != feature_space.end()){
            existing_cell->AddBoard(best_move);
        } else {
            feature_space.push_back(best_cell);
        }

        //pick a new cell
        //if the current cell is the last cell in the array, start from the beginning
        //otherwise, move on to the next cell
        auto it = std::find(feature_space.begin(), feature_space.end(), current_cell);
        if(it == feature_space.end()-1) current_cell = feature_space[0];
        else current_cell = *(it+1);
    }

    if(iteration_count == 150){
        std::cout<<"Failed to finish computations"<<std::endl;
        return;
    }
    //retrieve the path from move tree
    std::vector<BoardState> path;
    BoardState next = move_tree.back();
    while(!(next == initial_board)){
        next = next.GetPreviousState();
        path.push_back(next);
    }

    int i = 0;
    for(auto state : path){
        std::cout<<"State "<<i<<":"<<std::endl;
        state.PrintBoardState();
        i++;
    }

    //get the order of goal squares from the path of board states
    for(auto state : path){
        for(auto goal : goals){
            int x = goal.GetX();
            int y = goal.GetY();
            auto b = state.GetBlocks();
            if(b[x][y].IsOccupied() && (std::find(packing_order_.begin(), packing_order_.end(), goal) == packing_order_.end())){
                packing_order_.push_back(goal);
            }
        }
    }
}

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
            if(visited[i][j] == 0 && (type == Floor) && !blocks[i][j].IsOccupied()){
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
                            int x = (*neighbour).GetX();
                            int y = (*neighbour).GetY();
                            if ((type == Floor || type == Goal) && !blocks[x][y].IsOccupied()){
                                stack.push(std::make_pair(x, y));
                            }
                        }
                    }
                }
            }
        }
    }
}

void FeatureSpaceCell::ComputeRoomConnectivity(){
    //go room by room through the upper triangle of the adjacency matrix
    //check if any edges that are supposed to be there obstructed by boxes
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();
    int broken_edges = 0;
    for(int i = 0; i < room_number_; i++){
        for(int j = i + 1; j < room_number_; j++){
            if(adjacency_[i][j]){
                std::vector<Block> room_blocks = blocks_by_room_[i];
                std::set<std::pair<int, int>> visited;
                bool reached = false;
                //check paths from all the blocks if necessary
                for(auto block : room_blocks){
                    std::pair<int, int> coord = std::make_pair(block.GetX(), block.GetY());
                    if(!reached && !block.IsOccupied() && visited.find(coord) == visited.end()){
                        std::queue<std::pair<int, int>> bfs_queue;
                        bfs_queue.push(coord);
                        while(!reached && !bfs_queue.empty()){
                            std::pair<int, int> current_block = bfs_queue.front();
                            bfs_queue.pop();
                            if(visited.find(current_block) == visited.end()){
                                visited.insert(current_block);
                                if(!room_by_coord_.count(current_block) || room_by_coord_[current_block] == i){
                                    std::vector<Block> neighbours = blocks[current_block.first][current_block.second].GetNeighbours();
                                    for(Block neighbour : neighbours){
                                        if( !(neighbour.GetType() == Wall || neighbour.GetType() == Outer || neighbour.IsOccupied()) ){
                                            bfs_queue.push(std::make_pair(neighbour.GetX(), neighbour.GetY()));
                                        }
                                    }
                                } else if (room_by_coord_[current_block] == j){
                                    reached = true;
                                }
                            }
                        }
                    }
                }
                if(!reached) {
                    broken_edges++;
                    std::cout<<"Broken edge between rooms "<<i<<" and "<<j<<std::endl;
                }
            }
        }
    }
    room_connectivity_ = broken_edges;
}

void FeatureSpaceCell::ComputeOutOfPlan(){
    /// TODO
    std::vector<Box> boxes = board_.GetBoxes();
    for(auto box : boxes){
        std::pair<int, int> coord = std::make_pair(box.GetX(), box.GetY());
        auto basin = basin_by_coord_.find(coord);
        if(basin == basin_by_coord_.end()) {
            out_of_plan_++;
        } else if(basin->second != sink_room_basin_) {
            out_of_plan_++;
        }
    }
}

std::map<std::pair<int,int>, int> FeatureSpaceCell::FindRooms(){
    //TODO: add that if only 1 square overlaps in two rooms, mark this square not belonging to any room
    std::map<std::pair<int,int>, int> rooms;
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();

    int current_room = 0;

    //first go through the level map with 2x3 rectangles
    for(unsigned int i = 0; i < blocks.size()-1; i++){
        for(unsigned int j = 0; j < blocks[i].size()-2; j++){
            bool one_room = true;
            std::vector<std::pair<int,int>> potential_room;

            int counter_height = 0;
            while(counter_height < 2){
                int counter_width = 0;
                while(counter_width < 3){
                    //if one of the blocks is not a floor block, exit the check for the current rectangle
                    if(blocks[i+counter_height][j+counter_width].GetType() != Floor && blocks[i+counter_height][j+counter_width].GetType() != Goal){
                        one_room = false;
                        counter_height = 2;
                        counter_width = 3;
                    } else {
                        potential_room.push_back(std::make_pair(blocks[i+counter_height][j+counter_width].GetX(), blocks[i+counter_height][j+counter_width].GetY()));
                    }
                    counter_width++;
                }
                counter_height++;
            }
            //if all blocks in rectangle are floor, mark them as one room
            if(one_room){
                //first check if two (or more) blocks from the upper row/left-most column overlap with previously found room
                //don't have to check below, because we go top down left to right
                int room = current_room;
                std::pair<int, int> coord1 = potential_room[0];
                std::pair<int, int> coord2 = potential_room[1];
                std::pair<int, int> coord3 = potential_room[2];
                std::pair<int, int> coord4 = potential_room[3];
                if(rooms.count(coord1) && rooms.count(coord2) && rooms[coord1] == rooms[coord2]){
                    room = rooms[coord1];
                } else if (rooms.count(coord2) && rooms.count(coord3) && rooms[coord2] == rooms[coord3]){
                    room = rooms[coord2];
                } else if (rooms.count(coord1) && rooms.count(coord4) && rooms[coord1] == rooms[coord4]){
                    room = rooms[coord1];
                }
                for(auto coordinate : potential_room){
                    rooms[coordinate] = room;
                }
                if(room == current_room) current_room++;
            }

        }
    }

    // go through the level map with 3x2 rectangles
    for(unsigned int i = 0; i < blocks.size()-2; i++){
        for(unsigned int j = 0; j < blocks[i].size()-1; j++){
            bool one_room = true;
            std::vector<std::pair<int,int>> potential_room;

            int counter_height = 0;
            while(counter_height < 3){
                int counter_width = 0;
                while(counter_width < 2){
                    //if one of the blocks is not a floor block, exit the check for the current rectangle
                    if(blocks[i+counter_height][j+counter_width].GetType() != Floor && blocks[i+counter_height][j+counter_width].GetType() != Goal){
                        one_room = false;
                        counter_height = 3;
                        counter_width = 2;
                    } else {
                        potential_room.push_back(std::make_pair(blocks[i+counter_height][j+counter_width].GetX(), blocks[i+counter_height][j+counter_width].GetY()));
                    }
                    counter_width++;
                }
                counter_height++;
            }
            //if all blocks in rectangle are floor, mark them as one room
            if(one_room){
                // save coordinates for easier checking for overlapping rooms
                int room = current_room;
                std::set<int> overlapping_rooms;

                auto end = rooms.end();
                //check by row
                for(int i = 0; i < 6; i += 2){
                    std::pair<int, int> coord1 = potential_room[i];
                    std::pair<int, int> coord2 = potential_room[i+1];
                    if(rooms.count(coord1) && rooms.count(coord2) && rooms[coord1] == rooms[coord2]){
                        overlapping_rooms.insert(rooms[coord1]);
                    }
                }
                //check pairs by columns
                for(int j = 0; j < 4; j += 2){
                    for(int i = 0; i < 2; i++){
                        std::pair<int, int> coord1 = potential_room[j+i];
                        std::pair<int, int> coord2 = potential_room[j+2+i];
                        if(rooms.count(coord1) && rooms.count(coord2) && rooms[coord1] == rooms[coord2]){
                            overlapping_rooms.insert(rooms[coord1]);
                        }
                    }
                }

                //if the resulting set has 1 room or more, update those
                if(overlapping_rooms.size() > 0){
                    //select the room with smallest number
                    room = *overlapping_rooms.begin();
                    //if there are more than one room, update it to be equal to the first room
                    for(auto it = ++overlapping_rooms.begin(); it != overlapping_rooms.end(); it++){
                        for(auto& r : rooms){
                            if(r.second == *it){
                                r.second = room;
                            }
                        }
                    }
                }

                for(auto coordinate : potential_room){
                    rooms[coordinate] = room;
                }
                if(room == current_room) current_room++;
            }

        }
    }

    for(auto room : rooms){
        blocks_by_room_[room.second].push_back(blocks[room.first.first][room.first.second]);
    }

    //adjusting correct room numbering, so there wouldn't be, for example, rooms 0,1,3 instead of 0,1,2 
    int correct_room = 0;
    for(auto& room : blocks_by_room_){
        if (room.first != correct_room){
            for(auto block : room.second){
                std::pair<int, int> coord = std::make_pair(block.GetX(), block.GetY());
                rooms[coord] = correct_room;
            }
        }
        correct_room++;
    }

    blocks_by_room_.clear();
    for(auto room : rooms){
        blocks_by_room_[room.second].push_back(blocks[room.first.first][room.first.second]);
    }

    room_by_coord_ = rooms;
    room_number_ = blocks_by_room_.size();
    return rooms;
}

void FeatureSpaceCell::FindBasins(){
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();

    //first find goal room
    int goal_room;
    bool found = false;
    for(auto room : blocks_by_room_){
        for(auto block : room.second){
            if(block.GetType() == Goal){
                goal_room = room.first;
                found = true;
                break;
            }
        }
        if(found) break;
    }
    std::cout<<"Goal room: "<<goal_room<<std::endl;

    //find exits from the goal room
    std::vector<Block> exits;
    for(auto block : blocks_by_room_[goal_room]){
        for(auto neighbour : block.GetNeighbours()){
            std::pair<int, int> coord = std::make_pair(neighbour.GetX(), neighbour.GetY());
            if(neighbour.GetType() == Floor && (room_by_coord_.find(coord) == room_by_coord_.end() || room_by_coord_[coord] != goal_room)){
                exits.push_back(blocks[neighbour.GetX()][neighbour.GetY()]);
            }
        }
    }
    
    std::map<int, std::vector<Block>> basins;
    int basin_count = 0;
    //start exploring basins corresponding to each exit
    for(auto exit : exits){
        std::set<std::pair<int, int>> visited;
        std::queue<std::pair<int, int>> bfs_queue;
        bfs_queue.push(std::make_pair(exit.GetX(), exit.GetY()));
        while(!bfs_queue.empty()){
            std::pair<int, int> current = bfs_queue.front();
            bfs_queue.pop();
            if(!visited.count(current) && (!room_by_coord_.count(current) || room_by_coord_[current] != goal_room)){
                visited.insert(current);
                int x, y; std::tie(x, y) = current;
                basins[basin_count].push_back(blocks[x][y]);
                //check if box can be pushed to the current square from all four directions
                //basically that means that both the neighbour square and square behind it are of type Floor

                //check North
                if(blocks[x-1][y].GetType() == Floor && blocks[x-2][y].GetType() == Floor){
                    bfs_queue.push(std::make_pair(x-1, y));
                }
                //check South
                if(blocks[x+1][y].GetType() == Floor && blocks[x+2][y].GetType() == Floor){
                    bfs_queue.push(std::make_pair(x+1, y));
                }
                //check East
                if(blocks[x][y+1].GetType() == Floor && blocks[x][y+2].GetType() == Floor){
                    bfs_queue.push(std::make_pair(x, y+1));
                }
                //check West
                if(blocks[x][y-1].GetType() == Floor && blocks[x][y-2].GetType() == Floor){
                    bfs_queue.push(std::make_pair(x, y-1));
                }
            }
        }
        basin_count++;
    }

    //update basin by room, so we could check to which basin a room belongs -- some blocks in the basin don't belong to any room
    //and basin by coordinate, so we could find which basin this coordinate (x, y) belongs to.
    for(auto basin : basins){
        for(auto block : basin.second){
            std::pair<int, int> coord = std::make_pair(block.GetX(), block.GetY());
            basin_by_coord_[coord] = basin.first;
            auto room = room_by_coord_.find(coord);
            if(room != room_by_coord_.end()){
                basin_by_room_[room->second] = basin.first;
            }
        }
    }

    //find the basin with the maximum number of boxes
    int maximum_boxes = 0;
    int best_basin = -1;
    for(auto basin : basins){
        int current_boxes = 0;
        for(auto block : basin.second){
            if(block.IsOccupied()) current_boxes++;
        }
        if(current_boxes>maximum_boxes){
            maximum_boxes = current_boxes;
            best_basin = basin.first;
        }
    }

    //find sink room -- the closest room of the best basin to the goal room
    int sink_room = -1;
    for(auto block : basins[best_basin]){
        if(room_by_coord_.count(std::make_pair(block.GetX(), block.GetY()))){
            sink_room = room_by_coord_[std::make_pair(block.GetX(), block.GetY())];
            break;
        }
    }
    /*std::cout<<"Sink room: "<<sink_room<<std::endl;
    std::cout<<"Best basin: "<<best_basin<<std::endl;
    for(auto basin : basins){
        std::cout<<"Basin "<<basin.first<<":"<<std::endl;
        for(auto block : basin.second){
            block.Print();
        }
    }

    std::cout<<"Rooms by basin:"<<std::endl;
    for(auto room : basin_by_room_){
        std::cout<<"Room "<<room.first<<" belongs to basin "<<room.second<<std::endl;
    }*/
    sink_room_basin_ = best_basin;
    sink_room_=sink_room;
    basins_=basins;
}

std::vector<std::vector<bool>> FeatureSpaceCell::ComputeAdjacency(){
    std::vector<std::vector<bool>> adjacency(room_number_, std::vector<bool>(room_number_, false));
     
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();

    //compute connectivity from the perspective of each room
    for(int i = 0; i < room_number_; i++){
        std::vector<Block> room_blocks = blocks_by_room_[i];
        std::set<std::pair<int, int>> visited;
        std::queue<std::pair<int, int>> bfs_queue;
        bfs_queue.push(std::make_pair(room_blocks[0].GetX(), room_blocks[0].GetY()));
        while(!bfs_queue.empty()){
            std::pair<int, int> current_block = bfs_queue.front();
            bfs_queue.pop();
            if(!visited.count(current_block)){
                visited.insert(current_block);
                if(!room_by_coord_.count(current_block) || room_by_coord_[current_block] == i){
                    std::vector<Block> neighbours = blocks[current_block.first][current_block.second].GetNeighbours();
                    for(Block neighbour : neighbours){
                        if( !(neighbour.GetType() == Wall || neighbour.GetType() == Outer) ){
                            bfs_queue.push(std::make_pair(neighbour.GetX(), neighbour.GetY()));
                        }
                    }
                } else {
                    int j = room_by_coord_[current_block];
                    adjacency[i][j] = true;
                    adjacency[j][i] = true;
                }
            }

        }
    }

    adjacency_ = adjacency;
    return adjacency;
}

void FeatureSpaceCell::PrintRooms(){
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();
    for(auto row : blocks){
        for(auto block : row){
            std::pair<int, int> coord = std::make_pair(block.GetX(), block.GetY());
            auto found = room_by_coord_.find(coord);
            if(found == room_by_coord_.end()){
                std::cout<<"_";
            } else {
                std::cout<<found->second;
            }
        }
        std::cout<<std::endl;
    }
}