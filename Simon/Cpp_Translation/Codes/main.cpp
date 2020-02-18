#include <iostream>
#include <cmath> // For cos/sin
#include <vector>
#include <tuple>
#include "C:\Users\Simon\AppData\Local\Programs\Python\Python38-32\include\Python.h"
//#include <Mathplotlibcpp-Master\matplotlib-cpp>

using namespace std;

tuple<vector<vector<float>>, vector<vector<float>>> Linearized_discrete_DD_model(vector<float> x,vector<float> u,float dt) 
{
    vector<vector<float>> Amat
    {
       { 1, 0, -dt*u[0]*sin(x[2]) },
       { 0, 1,  dt*u[0]*cos(x[2]) },
       { 0, 0,    1               }
    };

    vector<vector<float>> Bmat
    {
       { dt*cos(x[2]), 0},
       { dt*cos(x[2]), 0},
       { 0           , dt}
    };    

    return {Amat,Bmat} ;
}

int main()
{
    vector<float> x{1, 0 ,4}, u{1, 1};
    float dt = 0.5;
    auto [A , B] = Linearized_discrete_DD_model(x, u, dt);
        // Displaying the 2D vector 
    for (int i = 0; i < A.size(); i++) { 
        for (int j = 0; j < A[i].size(); j++) 
            cout << A[i][j] << " "; 
        cout << endl; 
    } 
    for (int i = 0; i < B.size(); i++) { 
        for (int j = 0; j < B[i].size(); j++) 
            cout << B[i][j] << " "; 
        cout << endl; 
    }

    return 0;
}
