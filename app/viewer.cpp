#include "viewer.h"
#include "args.h"

void initOutput() {
    std::filesystem::remove_all("output");
    std::filesystem::create_directory("output");
    std::filesystem::create_directory("output/frames");
    std::filesystem::create_directory("output/store");
    std::ofstream fout("output/end_frame.txt");
    fout << int(config["simulation-time"].as<double>() * config["fps"].as<double>());
    fout.close();

    writeDescription();
}

void addDescription(YAML::Node & root) {
    { // description for root
        root["dimension"] = config["dimension"].as<int>();
        // root["width"] = radius;
        // root["height"] = radius;
        root["fps"]    = config["fps"].as<int>();
        root["frames"] = int(config["simulation-time"].as<double>() * config["fps"].as<double>());

        if (config["show-vortex-particle"].as<bool>()) {
            YAML::Node obj;
            obj["name"]                 = "vortex_particles";
            obj["data_mode"]            = "dynamic";
            obj["primitive_type"]       = "point_list";
            obj["indexed"]              = false;
            obj["color_map"]["enabled"] = true;
            // obj["color_map"]["normalized"] = true;
            root["objects"].push_back(obj);
        }
        if (config["show-grid"].as<bool>()) {
            YAML::Node obj;
            obj["name"]                 = "triangle_mesh";
            obj["data_mode"]            = "dynamic";
            obj["primitive_type"]       = "line_list";
            obj["indexed"]              = false;
            obj["color_map"]["enabled"] = true;
            // obj["color_map"]["normalized"] = true;
            root["objects"].push_back(obj);
        }
        if (config["show-velX-grid"].as<bool>()) {
            YAML::Node obj;
            obj["name"]                    = "velx_grid";
            obj["data_mode"]               = "dynamic";
            obj["primitive_type"]          = "point_list";
            obj["indexed"]                 = false;
            obj["color_map"]["enabled"]    = true;
            // obj["color_map"]["normalized"] = true;
            root["objects"].push_back(obj);
        }
        if (config["show-velY-grid"].as<bool>()) {
            YAML::Node obj;
            obj["name"]                    = "vely_grid";
            obj["data_mode"]               = "dynamic";
            obj["primitive_type"]          = "point_list";
            obj["indexed"]                 = false;
            obj["color_map"]["enabled"]    = true;
            // obj["color_map"]["normalized"] = true;
            root["objects"].push_back(obj);
        }
    }
}

void plot(const Aero & data, int ord) {
    std::filesystem::create_directory(fmt::format("output/frames/{}", ord));

    if (config["show-vortex-particle"].as<bool>()) {
        // if (config["debug-mode"].as<bool>()) std::cout << "show vortex particle\n";
        std::string   particle_dir = fmt::format("output/frames/{}/vortex_particles.mesh", ord);
        std::ofstream fout(particle_dir, std::ios::binary);

        int num = data.vortex_particles.size();
        fout.write((char *) (&num), 4);

        for (const auto & particle : data.vortex_particles) {
            for (int i = 0; i < 2; i++) {
                float v = (float) (particle.position[i]);
                fout.write((char *) (&v), 4);
            }
        }

        for (const auto & particle : data.vortex_particles) {
            float v = particle.vortex;
            // std::cout << v << std::endl;
            fout.write((char *) (&v), 4);
        }
    }

    if (config["show-velX-grid"].as<bool>()) {
        std::string mesh_dir =
            fmt::format("output/frames/{}/velx_grid.mesh", ord);
        std::ofstream fout(mesh_dir, std::ios::binary);

        int num = data.velocityX.size.x() * data.velocityX.size.y();
        // int num = (data.size.y() - 1) * data.size.x();
        fout.write((char *) (&num), 4);

        for (int i = 0; i < data.velocityX.size.x(); i++) {
            for (int j = 0; j < data.velocityX.size.y(); j++) {
                auto pos = data.velocityX.tri2pos(Vec2i(i, j));
                for (int k = 0; k < 2; k++) {
                    float v = (float) (pos[k]);
                    fout.write((char *) (&v), 4);
                }
            }
        }

        for (int i = 0; i < data.velocityX.size.x(); i++) {
            for (int j = 0; j < data.velocityX.size.y(); j++) {
                float v = data.velocityX[Vec2i(i, j)];
                fout.write((char *) (&v), 4);
            }
        }
    }

    if (config["show-velY-grid"].as<bool>()) {
        std::string mesh_dir =
            fmt::format("output/frames/{}/vely_grid.mesh", ord);
        std::ofstream fout(mesh_dir, std::ios::binary);

        int num = data.velocityY.size.x() * data.velocityY.size.y();
        // int num = (data.size.y() - 1) * data.size.x();
        fout.write((char *) (&num), 4);

        for (int i = 0; i < data.velocityY.size.x(); i++) {
            for (int j = 0; j < data.velocityY.size.y(); j++) {
                auto pos = data.velocityY.tri2pos(Vec2i(i, j));
                for (int k = 0; k < 2; k++) {
                    float v = (float) (pos[k]);
                    fout.write((char *) (&v), 4);
                }
            }
        }

        for (int i = 0; i < data.velocityY.size.x(); i++) {
            for (int j = 0; j < data.velocityY.size.y(); j++) {
                float v = data.velocityY[Vec2i(i, j)];
                fout.write((char *) (&v), 4);
            }
        }
    }

    if (config["show-grid"].as<bool>()) {
        std::string mesh_dir =
            fmt::format("output/frames/{}/triangle_mesh.mesh", ord);
        std::ofstream fout(mesh_dir, std::ios::binary);

        int num = ((data.size.x()) * (data.size.y() + 1) + (data.size.y()) * (data.size.x() + 1)) * 2;
        // int num = (data.size.y() - 1) * data.size.x();
        fout.write((char *) (&num), 4);

        for (int i = 0; i < data.size.x(); i++) {
            for (int j = 0; j < data.size.y() + 1; j++) {
                Vec2i start_tri(i, j);
                Vec2d start_pos = data.vortexNode.tri2pos(start_tri);
                for (int k = 0; k < 2; k++) {
                    float v = (float) (start_pos[k]);
                    fout.write((char *) (&v), 4);
                }
                Vec2i end_tri(i + 1, j);
                Vec2d end_pos = data.vortexNode.tri2pos(end_tri);
                for (int k = 0; k < 2; k++) {
                    float v = (float) (end_pos[k]);
                    fout.write((char *) (&v), 4);
                }
            }
        }

        for (int i = 0; i < data.size.x() + 1; i++) {
            for (int j = 0; j < data.size.y(); j++) {
                Vec2i start_tri(i, j);
                Vec2d start_pos = data.vortexNode.tri2pos(start_tri);
                for (int k = 0; k < 2; k++) {
                    float v = (float) (start_pos[k]);
                    fout.write((char *) (&v), 4);
                }
                Vec2i end_tri(i, j + 1);
                Vec2d end_pos = data.vortexNode.tri2pos(end_tri);
                for (int k = 0; k < 2; k++) {
                    float v = (float) (end_pos[k]);
                    fout.write((char *) (&v), 4);
                }
            }
        }

        for (int i = 0; i < data.size.x(); i++) {
            for (int j = 0; j < data.size.y() + 1; j++) {
                Vec2i start_tri(i, j);
                float v = (float) (data.psi[data.vortexNode.getIdx(start_tri)]);
                fout.write((char *) (&v), 4);
                Vec2i end_tri(i + 1, j);
                v = (float) (data.psi[data.vortexNode.getIdx(end_tri)]);
                fout.write((char *) (&v), 4);
            }
        }

        for (int i = 0; i < data.size.x() + 1; i++) {
            for (int j = 0; j < data.size.y(); j++) {
                Vec2i start_tri(i, j);
                float v = (float) (data.psi[data.vortexNode.getIdx(start_tri)]);
                fout.write((char *) (&v), 4);
                Vec2i end_tri(i, j + 1);
                v = (float) (data.psi[data.vortexNode.getIdx(end_tri)]);
                fout.write((char *) (&v), 4);
            }
        }
    }
    updateEndFrame(ord + 1);
}

void writeDescription() {
    std::ofstream fout("output/Description.yaml");
    YAML::Node    config;
    addDescription(config);
    fout << config;
}

void updateEndFrame(int new_ord) {
    std::ofstream fout("output/end_frame.txt");
    fout << new_ord;
    fout.close();
}