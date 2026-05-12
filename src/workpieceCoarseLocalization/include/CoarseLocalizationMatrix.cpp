#include "CoarseLocalizationMatrix.h"

ViewPlanningConfig* ViewPlanningConfig::instance = nullptr;

std::mutex ViewPlanningConfig::mutex_;

ViewPlanningConfig& ViewPlanningConfig::getInstance() {
    if (instance == nullptr) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (instance == nullptr) {
            instance = new ViewPlanningConfig();
        }
    }

    return *instance;
}
void ViewPlanningConfig::readConfig() {
    std::ifstream is(configPath);

    if (!is.is_open()) {
        std::cout << "ViewPlanningConfig open failed: " << configPath << std::endl;
        return;
    }

    cereal::JSONInputArchive inputArchive(is);
    inputArchive(*this);

    std::cout << "ViewPlanningConfig read success" << std::endl;
}

void ViewPlanningConfig::writeConfig() {
    std::ofstream os(configPath);

    if (!os.is_open()) {
        std::cout << "ViewPlanningConfig write open failed: " << configPath << std::endl;
        return;
    }

    cereal::JSONOutputArchive outputArchive(os);
    outputArchive(cereal::make_nvp("ViewPlanningConfig", *this));

    std::cout << "ViewPlanningConfig write success" << std::endl;
}

void ViewPlanningConfig::printConfig() {
    cereal::JSONOutputArchive jsonOutputArchive(std::cout);
    jsonOutputArchive(cereal::make_nvp("ViewPlanningConfig", *this));
    std::cout << std::endl;
}
void ViewPlanningConfig::initDefaultConfig() {
    classConfigs.clear();

    // =========================================================
    // Plate_Plate_F0
    // =========================================================
    {
        ViewClassConfig cfg;

        cfg.classId = 0;
        cfg.className = "Plate_Plate_F0";

        // left
        {
            ViewTransformConfig trans;

            trans.name = "left";

            trans.offsetXYZ = {0.0, 0.0, 0.0};

            trans.T = {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};

            cfg.transforms.emplace_back(trans);
        }

        // right
        {
            ViewTransformConfig trans;

            trans.name = "right";

            trans.offsetXYZ = {0.0, 0.0, 0.0};

            trans.T = {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};

            cfg.transforms.emplace_back(trans);
        }

        classConfigs[cfg.classId] = cfg;
    }

    // =========================================================
    // Tube_Plate_F1
    // =========================================================
    {
        ViewClassConfig cfg;

        cfg.classId = 1;
        cfg.className = "Tube_Plate_F";

        {
            ViewTransformConfig trans;

            trans.name = "default";

            trans.offsetXYZ = {0.0, 0.0, 0.0};

            trans.T = {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};

            cfg.transforms.emplace_back(trans);
        }

        classConfigs[cfg.classId] = cfg;
    }

    // =========================================================
    // TubeSide_Plate_F2
    // =========================================================
    {
        ViewClassConfig cfg;

        cfg.classId = 2;
        cfg.className = "TubeSide_Plate_F";

        {
            ViewTransformConfig trans;

            trans.name = "default";

            trans.offsetXYZ = {0.0, 0.0, 0.0};

            trans.T = {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};

            cfg.transforms.emplace_back(trans);
        }

        classConfigs[cfg.classId] = cfg;
    }

    // =========================================================
    // Tube_Tube_F3
    // =========================================================
    {
        ViewClassConfig cfg;

        cfg.classId = 3;
        cfg.className = "Tube_Tube_F";

        {
            ViewTransformConfig trans;

            trans.name = "default";

            trans.offsetXYZ = {0.0, 0.0, 0.0};

            trans.T = {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};

            cfg.transforms.emplace_back(trans);
        }

        classConfigs[cfg.classId] = cfg;
    }
}
