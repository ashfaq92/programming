#include "calculator.h"

using namespace std;

int main() {
    int a = 13;
    int b = 332;

    Calculator myCalc;
    int sum = myCalc.add(a, b);
    cout<<"Sum: "<<sum<<endl;
}