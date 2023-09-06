#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <formatter.hpp>

#define lamda 0.5
#define delta 0.01
#define number 2214111253LL
// #define number 2181411945LL

int main()
{
    using Eigen::Matrix2d;
    using Eigen::Vector2d;
    using spdlog::logger;
    Vector2d x0(number%827, number%1709);
    Matrix2d hessian;
    hessian << 2, 0, 0, 2;
    Matrix2d hessian_inv = hessian.inverse();
    // compute minimum of x^2+y^2, using newton method
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
	auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>("optimizer.log", true);
	spdlog::logger optimizer("optimizer", {console_sink, file_sink});
    optimizer.set_level(spdlog::level::debug);
    while(1)
    {
        optimizer.debug("({}, {})", x0(0), x0(1));
        Vector2d gradient(2*x0(0), 2*x0(1));
        Vector2d delta_x = lamda*hessian_inv*gradient;
        if(delta_x.norm() < delta)
            break;
        x0 -= delta_x;
    }
    double f=x0(0)*x0(0)+x0(1)*x0(1);
    optimizer.info("{}", f);
    return 0;
}
