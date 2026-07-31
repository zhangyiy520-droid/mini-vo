#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <vector>

namespace {

struct Sample {
    double x = 0.0;
    double y = 0.0;
};

struct GnOptions {
    int max_iterations = 10;
    double damping = 1e-9;
    double stop_step = 1e-12;
};

struct GnResult {
    Eigen::Vector2d state = Eigen::Vector2d::Zero();
    double final_cost = 0.0;
    int iterations = 0;
    bool converged = false;
};

double computeCost(const std::vector<Sample>& samples,
                   const Eigen::Vector2d& state) {
    double cost = 0.0;
    for (const auto& sample : samples) {
        const double predicted = state[0] * sample.x + state[1];
        const double residual = sample.y - predicted;
        cost += residual * residual;
    }
    return 0.5 * cost;
}

GnResult fitLineWithGaussNewton(const std::vector<Sample>& samples,
                                const Eigen::Vector2d& initial_state,
                                const GnOptions& options) {
    GnResult result;
    result.state = initial_state;

    for (int iter = 0; iter < options.max_iterations; ++iter) {
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        const double cost = computeCost(samples, result.state);

        for (const auto& sample : samples) {
            const double predicted = result.state[0] * sample.x +
                                     result.state[1];
            const double residual = sample.y - predicted;
            Eigen::RowVector2d J;
            J << -sample.x, -1.0;

            H += J.transpose() * J;
            b += -J.transpose() * residual;
        }

        H += options.damping * Eigen::Matrix2d::Identity();
        const Eigen::Vector2d dx = H.ldlt().solve(b);
        result.state += dx;
        result.iterations = iter + 1;

        std::cout << "[GN] iter=" << iter
                  << " cost=" << cost
                  << " dx=[" << dx.transpose() << "]"
                  << " state=[" << result.state.transpose() << "]"
                  << " detH=" << H.determinant() << '\n';

        if (dx.norm() < options.stop_step) {
            result.converged = true;
            break;
        }
    }

    result.final_cost = computeCost(samples, result.state);
    return result;
}

}  // namespace

int main() {
    const std::vector<Sample> samples{
        {-2.0, -3.0},
        {-1.0, -1.0},
        { 0.0,  1.0},
        { 1.0,  3.0},
        { 2.0,  5.0}
    };
    const GnOptions options;
    const GnResult result = fitLineWithGaussNewton(
        samples, Eigen::Vector2d(-5.0, 10.0), options);

    if ((result.state - Eigen::Vector2d(2.0, 1.0)).norm() > 1e-9) {
        std::cerr << "[FAIL] expected state near [2 1], got ["
                  << result.state.transpose() << "]\n";
        return 1;
    }

    std::cout << "[PASS] gauss-newton state=["
              << result.state.transpose() << "]"
              << " cost=" << result.final_cost
              << " iterations=" << result.iterations << '\n';
    return 0;
}
