#include <chrono>
#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <utility>

#include "BoardState.hpp"
#include "FeatureSpaceCell.hpp"

/** 
 * Main function, currently mostly used for testing.
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 02/06/2021
 *
 */ 

int main(int argc, char *argv[])
{
	// mostly just testing all the helper functions currently
	BoardState testBoard = BoardState();
	std::cout<<"Loading file. "<<std::endl;

	std::string filename = "../src/levels/xsokoban/screen.15";

	//takes filename as the first argument
	//if none were passed, tries to load the first level located at "./src/levels/xsokoban/screen.1"
	if(argc>1){
		filename = argv[1];
	}
	
	testBoard.ReadBoardState(filename);
	std::cout<<"Printing Board: "<<std::endl;
	testBoard.PrintBoardState();
	std::cout<<"Total number of rows: "<<testBoard.GetRows()<<", total number of columns: "<<testBoard.GetColumns()<<std::endl;
	testBoard.AddNeighbours();
	FeatureSpaceCell testSpace = FeatureSpaceCell(&testBoard);
	//testSpace.ComputeConnectivity();
	//std::cout<<"The current number of disconnected rooms is: "<<testSpace.GetConnectivity()<<std::endl;
	//std::cout<<"The current number of boxes blocking tunnels is: "<<testSpace.GetRoomConnectivity()<<std::endl;

	std::map<std::pair<int,int>, int> r = testSpace.FindRooms();
	/*for(auto const& room : r){
		std::cout<<"("<<room.first.first<<", "<<room.first.second<<") -- "<<room.second<<std::endl;
	}*/
	std::cout<<"Number of rooms: "<<testSpace.GetRoomNumber()<<std::endl;
	testSpace.FindBasins();
	testSpace.PrintRooms();
	//testSpace.ComputeOutOfPlan();
	//std::cout<<"Number out of plan boxes: "<<testSpace.GetOutOfPlan()<<std::endl;

	testSpace.ComputePackingOrder();
	for(auto block : testSpace.GetPackingOrder()){
		std::cout<<block.GetX()<<", "<< block.GetY()<<std::endl;
	}

	//testSpace.ComputeAdjacency();
	//testSpace.ComputeRoomConnectivity();
	//std::cout<<"Room connectivity (broken edges): "<<testSpace.GetRoomConnectivity()<<std::endl;
	
	
	//custom compare function
	/*auto cmp = [](std::pair<BoardState, PackingPlanFeatureSpaceCell> left, std::pair<BoardState, PackingPlanFeatureSpaceCell> right) { 
		if(left.second.GetBoxesOnBoard() < right.second.GetBoxesOnBoard()) return false; 
		else if(left.second.GetBoxesOnTarget() < right.second.GetBoxesOnTarget()) return false;
		else if(left.second.GetDistance() > right.second.GetDistance()) return false;
		return true;

	 };
	//test queue
	std::priority_queue<std::pair<BoardState, PackingPlanFeatureSpaceCell>, std::vector<std::pair<BoardState, PackingPlanFeatureSpaceCell>>, decltype(cmp)> q3(cmp);

	//set up initial position, all boxes on targets
    std::vector<Block> goals = testBoard.GetGoals();
    std::vector<Box> boxes_on_goals;
    for (auto goal : goals){
		Box new_box = Box(goal.GetX(), goal.GetY(), true);
        //boxes_on_goals.push_back(Box(goal.GetX(), goal.GetY(), true));
		boxes_on_goals.push_back(new_box);
    }
	std::cout<<"Testing:"<<std::endl;
	std::vector<std::vector<Block>> blocks = testBoard.GetBlocks();
    BoardState initial_board = BoardState(blocks, boxes_on_goals);

	//push the initial position in the queue
	q3.push(std::make_pair(initial_board, PackingPlanFeatureSpaceCell(initial_board)));
	//move a box from the target square
	boxes_on_goals[0].Move(West);
	boxes_on_goals[0].Undeliver();
	BoardState moved = BoardState(testBoard.GetBlocks(), boxes_on_goals);
	q3.push(std::make_pair(moved, PackingPlanFeatureSpaceCell(moved)));
	//move it further away
	boxes_on_goals[0].Move(West);
	BoardState moved2 = BoardState(testBoard.GetBlocks(), boxes_on_goals);
	q3.push(std::make_pair(moved2, PackingPlanFeatureSpaceCell(moved2)));
	//remove one box
	boxes_on_goals.pop_back();
	BoardState pos_without_box = BoardState(testBoard.GetBlocks(), boxes_on_goals);
	q3.push(std::make_pair(pos_without_box, PackingPlanFeatureSpaceCell(pos_without_box)));
	//move another box from the target square
	boxes_on_goals[2].Move(West);
	boxes_on_goals[2].Undeliver();
	BoardState moved1 = BoardState(testBoard.GetBlocks(), boxes_on_goals);
	q3.push(std::make_pair(moved1, PackingPlanFeatureSpaceCell(moved1)));

	std::cout<<"Printing priority queues:"<<std::endl;
	while(!q3.empty()) {
		PackingPlanFeatureSpaceCell current = q3.top().second;
        std::cout << current.GetBoxesOnBoard() << " " << current.GetBoxesOnTarget()<<" "<<current.GetDistance()<<std::endl;
        q3.pop();
    }

	std::cout<<"Test map with board states:"<<std::endl;
	//do I have to write my own compare function, equality is not enough?
	std::map<BoardState, int> move_tree;
	move_tree[testBoard] = 0;
	move_tree[moved] = 1;
	move_tree[moved1] = 2;
	//move_tree[moved2] = 3;
	std::vector<BoardState> test_vector;
	test_vector.push_back(moved);
	test_vector.push_back(testBoard);
	
	test_vector.push_back(moved1);
	test_vector.push_back(moved2);

	auto it = std::find(test_vector.begin(), test_vector.end(), testBoard);
	std::cout<<"Index found: "<<(it - test_vector.begin())<<std::endl;
	std::cout<<"Random value for moved "<<move_tree[moved]<<std::endl;
	std::cout<<"Random value for moved1 "<<move_tree[moved1]<<std::endl;*/
}