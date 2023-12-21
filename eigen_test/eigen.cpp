#include <iostream>
#include <Eigen/Dense>

int main() {
    // Define a 4x3 matrix (replace this with your matrix)
    Eigen::MatrixXd originalMatrix(4, 3);
    originalMatrix << 1, 2, 3,
                      4, 5, 6,
                      7, 8, 9,
                      10, 11, 12;

    // Compute the pseudo-inverse
    Eigen::MatrixXd pseudoInverse = originalMatrix.completeOrthogonalDecomposition().pseudoInverse();

    // Display the original and pseudo-inverse matrices
    std::cout << "Original Matrix:\n" << originalMatrix << "\n\n";
    std::cout << "Pseudo-Inverse Matrix:\n" << pseudoInverse << "\n";

    return 0;
}

