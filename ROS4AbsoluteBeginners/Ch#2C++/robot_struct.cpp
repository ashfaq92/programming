#include <iostream>
#include <string>

using namespace std;

struct RobotStruct {
    int id;
    int wheelsNum;
    string name;
};

int main() {
    RobotStruct myRobot;
    myRobot.id = 23;
    myRobot.wheelsNum = 4;
    myRobot.name = "Tungston";
    return 0;
}