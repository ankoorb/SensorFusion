#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;

int main() {

    // Vector
    VectorXd v(2);
    v << 10, 20;
    cout << v << endl;
    cout << endl;

    // Matrix
    MatrixXd m(2, 2);
    m << 10, 20, 30, 40;
    cout << m << endl;
    cout << endl;

    // Transpose
    MatrixXd mt = m.transpose();
    cout << mt << endl;
    cout << endl;

    // Inverse
    MatrixXd mi = m.inverse();
    cout << mi << endl;
    cout << endl;

    // Matrix x Vector
    MatrixXd p = m * m;
    cout << p << endl;
    cout << endl;

    // Matrix sum, product and mean
    cout << m.sum() << endl;
    cout << m.prod() << endl;
    cout << m.mean() << endl;

    // Matrix trace
    cout << m.trace() << endl;

    // Matrix diagonal
    cout << m.diagonal() << endl;

    // Matrix size
    cout << m.size() << endl;



    return 0;
}