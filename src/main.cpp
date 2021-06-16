#include <iostream>

#include "Board.hpp"

int main(int argc, char *argv[])
{
	/*Block testBlock = Block(0, 0, Floor, true);
	std::cout << "Position of the test block: " << testBlock.GetX() << " " << testBlock.GetY() << std::endl;
	std::cout << "Block type: " << testBlock.GetType() << std::endl;
	std::cout << "Is block free? " << testBlock.isOccupied() << std::endl;
	testBlock.free();
	std::cout << "Is block free now? " << testBlock.isOccupied() << std::endl;*/
	Board testBoard = Board();
	testBoard.readBoard("C:\\aalto\\sokoban_solver\\src\\levels\\Brian Damgaard YASGen.sok", '1');
	testBoard.printBoard();
}