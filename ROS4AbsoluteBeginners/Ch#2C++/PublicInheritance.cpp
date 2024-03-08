#include <iostream>
#include <string>

using namespace std;

class GeneralRobot {
    public:
    int id;
    int wheelsNum;
    string name;
    void greet();
    void move();
};

void GeneralRobot::greet() {
    cout<<GeneralRobot::name<<" says you hello!"<<endl;
}

void GeneralRobot::move() {
    cout<<GeneralRobot::name<<" is moving!"<<endl;
}

class SpecificRobot : public GeneralRobot  {
    public:
    void moveLeft();
    void moveRight();
};

void SpecificRobot::moveLeft() {
    cout<<SpecificRobot::name<<" is moving left..."<<endl;
}

void SpecificRobot::moveRight() {
    cout<<SpecificRobot::name<<" is moving right..."<<endl;
}

int main() {
    SpecificRobot myRobot;
    myRobot.name = "🤖";
    myRobot.id = 32;
    myRobot.wheelsNum = 0;

    myRobot.move();
    myRobot.greet();
    myRobot.moveLeft();
    myRobot.moveRight();
    return 0;
}