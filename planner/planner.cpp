#include <qpOASES.hpp>
#include <chrono>
#include "eigen3/Eigen/Core"
#include <iostream>
#include <vector>

template<int M, int N>
using QPMatrix = Eigen::Matrix<qpOASES::real_t, M, N, N != 1 ? Eigen::RowMajor : Eigen::ColMajor>;

class PathPlanner {
    public:
        PathPlanner(QPMatrix<4, 3> world_to_wheel_space, double a_max) : problem(12, 8), world_to_wheel(world_to_wheel_space) {
            qpOASES::Options options;
            options.printLevel = qpOASES::PL_NONE;
            problem.setOptions(options);

            lb << -a_max, -a_max, -a_max, -a_max, -a_max, -a_max, -a_max, -a_max;
            ub << a_max, a_max, a_max, a_max, a_max, a_max, a_max, a_max;

            init();
        }

        // `target` is [px0, px1, vx0, vx1, py0, py1, vy0, vy1, pth0, pth1, vth0, vth1]
        QPMatrix<12, 1> plan(QPMatrix<12, 1> target, double ti, double tf) {
            set_up_problem(target, ti, tf);
            qpOASES::int_t nWSR = 100;
            problem.hotstart(cost_quadratic.data(), cost_linear.data(), constraint.data(), NULL, NULL, lb.data(), ub.data(), nWSR);
            QPMatrix<12, 1> result = QPMatrix<12, 1>::Zero();
            problem.getPrimalSolution(result.data());
            return result;
        }

    private:
        void init() {
            QPMatrix<12, 1> target;
            target << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            set_up_problem(target, 0.0, 1.0);
            qpOASES::int_t nWSR = 100;

            problem.init(cost_quadratic.data(), cost_linear.data(), constraint.data(), NULL, NULL, lb.data(), ub.data(), nWSR);

            QPMatrix<12, 1> result;
            problem.getPrimalSolution(result.data());
        }

        void set_up_problem(QPMatrix<12, 1> target, double ti, double tf) {
            // M is the matrix such that M*coeffs = target
            QPMatrix<4, 4> M_1d;
            M_1d << 1.0, ti, ti * ti, ti * ti * ti,
                 1.0, tf, tf * tf, tf * tf * tf,
                 0.0, 1.0, 2 * ti, 3 * ti * ti,
                 0.0, 1.0, 2 * tf, 3 * tf * tf;
            QPMatrix<12, 12> M = QPMatrix<12, 12>::Zero();
            M.block<4, 4>(0, 0) = M_1d;
            M.block<4, 4>(4, 4) = M_1d;
            M.block<4, 4>(8, 8) = M_1d;

            // Let `a` be the coefficients of the polynomial. Then:
            // We're minimizing (Ma - target)^2 = a^T(M^T M)a - 2target^TMa + target^T target
            // Equivalently: 1/2 a^T(M^T M)a - target^T M a
            cost_quadratic = M.transpose() * M;
            cost_linear = -target.transpose() * M;

            // The matrix A giving the terminal world-space accelerations for given coefficients
            // A(coeffs) = (accel_begin, accel_end)^T
            QPMatrix<2, 4> min_max_acceleration_matrix_1d;
            min_max_acceleration_matrix_1d << 0.0, 0.0, 2.0, 6.0 * ti,
                                              0.0, 0.0, 2.0, 6.0 * tf;
            QPMatrix<6, 12> min_max_acceleration_matrix = QPMatrix<6, 12>::Zero();
            min_max_acceleration_matrix.block<1, 4>(0, 0) = min_max_acceleration_matrix_1d.block<1, 4>(0, 0);
            min_max_acceleration_matrix.block<1, 4>(1, 4) = min_max_acceleration_matrix_1d.block<1, 4>(0, 0);
            min_max_acceleration_matrix.block<1, 4>(2, 8) = min_max_acceleration_matrix_1d.block<1, 4>(0, 0);
            min_max_acceleration_matrix.block<1, 4>(3, 0) = min_max_acceleration_matrix_1d.block<1, 4>(1, 0);
            min_max_acceleration_matrix.block<1, 4>(4, 4) = min_max_acceleration_matrix_1d.block<1, 4>(1, 0);
            min_max_acceleration_matrix.block<1, 4>(5, 8) = min_max_acceleration_matrix_1d.block<1, 4>(1, 0);

            // Make a matrix that is two stacked copies of world_to_wheel to
            // constrain both the start and the end acceleration.
            QPMatrix<8, 6> world_to_wheel_twice;
            world_to_wheel_twice.block<4, 3>(0, 0) = world_to_wheel;
            world_to_wheel_twice.block<4, 3>(4, 3) = world_to_wheel;
            constraint = world_to_wheel_twice * min_max_acceleration_matrix;
        }

        QPMatrix<12, 12> cost_quadratic;
        QPMatrix<1, 12> cost_linear;

        QPMatrix<8, 12> constraint;
        QPMatrix<8, 1> lb, ub;
        qpOASES::SQProblem problem;
        QPMatrix<4, 3> world_to_wheel;
};

// NOTE TO SELF: Eigen matrices are row-major

int main() {
    // Convert world space to wheel space.
    //
    //  1/------\4
    //   |  y   |
    //   |  |_x |
    //   |      |
    //  2\------/3
    //
    // Wheel angles, measured from the x-axis.
    double theta_1 = M_PI * 0.25,
           theta_2 = M_PI * 0.75,
           theta_3 = M_PI * 0.25,
           theta_4 = M_PI * 0.75;

    double r = 0.10;
    double a_max = 10.0;

    Eigen::Matrix<qpOASES::real_t, 4, 3> world_to_wheel_space;
    world_to_wheel_space << 
        std::cos(theta_1), std::sin(theta_1), r,
        std::cos(theta_2), std::sin(theta_2), r,
        std::cos(theta_3), std::sin(theta_3), r,
        std::cos(theta_4), std::sin(theta_4), r;

    PathPlanner planner(world_to_wheel_space, a_max);

    QPMatrix<12, 1> target;
    target << 0, 4, 0, 0, 0, 8, 3, 0, 0, 3.1415, 0, 0;
    QPMatrix<12, 1> coeffs = planner.plan(target, 0, 5);

    std::cout << std::fixed;
    std::cout.precision(6);
    // Print the coefficients as a polynomial
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            int idx = i * 4 + j;
            std::cout << coeffs(idx);
            for (int k = 0; k < j; k++) {
                std::cout << " * t";
            }
            if (j != 3) {
                std::cout << " + ";
            }
        }
        if (i != 2) {
            std::cout << "," << std::endl;
        }
    }
    std::cout << std::endl;
}
