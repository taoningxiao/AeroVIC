#include "args.h"
#include <iostream>
#include <fmt/format.h>
#include "aero.h"
#include "timer.h"
#include "viewer.h"
#include "tqdm.hpp"

int main(int argc, char * argv[]) {
    if (argc < 2) {
        std::cout << "./program [config_file]\n";
        exit(0);
    } else load_config(argv[1]);
    
    initOutput();
    initMyClock();
    double spacing = config["spacing"].as<double>();
    Vec2i size(int(config["width"].as<double>()/spacing), int(config["height"].as<double>()/spacing));
    Vec2d origin;
    for (int i = 0; i < 2; i++) origin[i] =  - size[i] * spacing / 2;
    origin.x() /= 2;
    Aero sim(origin, size, spacing);

    const int FRAMES   = config["simulation-time"].as<double>() / config["time-step"].as<double>();
    int       plot_ord = 0;

    myClock["total"].start();
    for (int i : tq::trange(FRAMES)) {
        // if (i == 51) {
        //     std::cout << fmt::format("current frame = {}\n", i);
        // }
        sim.simulate(config["time-step"].as<double>());
        if (double(i) * config["time-step"].as<double>() >= double(plot_ord) / config["fps"].as<double>()) {
            plot(sim, plot_ord);
            plot_ord++;
        }
    }

    reportMyClock();
    myClock["total"].stop();
    return 0;
}