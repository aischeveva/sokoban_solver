#include "Object.hpp"

void Object::Move(Direction dir){
    switch(dir){
        case North: yCoordinate_++; break;
        case South: yCoordinate_--; break;
        case East: xCoordinate_++; break;
        case West: xCoordinate_--; break;
    }
}