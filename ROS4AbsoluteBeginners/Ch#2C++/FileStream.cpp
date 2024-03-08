#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main() {
    ofstream outFile;
    string data = "RobotID=0";
    cout<<"Write data: "<<data<<endl;
    outFile.open("config.txt");
    outFile<<data<<endl;
    outFile.close();

    string inputData;
    ifstream inFile;
    inFile.open("config.txt");
    inFile>>inputData;
    cout<<"Data: "<<inputData<<endl;
    inFile.close();

    return 0;
}