#ifndef DEFINE_POSITIONS_H
#define DEFINE_POSITIONS_H
#include <cmath>
// Posições das estantes
/*
1  |-----| 2     3 |-----| 4 
5  |-----| 6     7 |-----| 8 
9  |-----| 10   11 |-----| 12 
13 |-----| 14   15 |-----| 16 
17 |-----| 18   19 |-----| 20 */
const std::vector<float> DOCK_POSITION_1 = {-3.5, 0.850};
const std::vector<float> DOCK_POSITION_2 = {-3.5, 0.85};
const std::vector<float> DOCK_POSITION_3 = {3.5, 0.850};
const std::vector<float> DOCK_POSITION_4 = {3.5, 0.850};
const std::vector<float> DOCK_POSITION_5 = {-3.5, -0.465};
const std::vector<float> DOCK_POSITION_6 = {-3.5, -0.465};
const std::vector<float> DOCK_POSITION_7 = {3.5, -0.465};
const std::vector<float> DOCK_POSITION_8 = {3.5, -0.465};
const std::vector<float> DOCK_POSITION_9 = {-3.5, -1.780};
const std::vector<float> DOCK_POSITION_10 = {-3.5, -1.780};
const std::vector<float> DOCK_POSITION_11 = {3.5, -1.780};
const std::vector<float> DOCK_POSITION_12 = {3.5, -1.780};
const std::vector<float> DOCK_POSITION_13 = {-3.5, -3.095};
const std::vector<float> DOCK_POSITION_14 = {-3.5, -3.095};
const std::vector<float> DOCK_POSITION_15 = {3.5, -3.095};
const std::vector<float> DOCK_POSITION_16 = {3.5, -3.095};
const std::vector<float> DOCK_POSITION_17 = {-3.5, -4.410};
const std::vector<float> DOCK_POSITION_18 = {-3.5, -4.410};
const std::vector<float> DOCK_POSITION_19 = {3.5, -4.410};
const std::vector<float> DOCK_POSITION_20 = {3.5, -4.410};


// Define um vetor constante que contém todas as posições
const std::vector<std::vector<float>> POSITIONS = {
    DOCK_POSITION_1, DOCK_POSITION_2, DOCK_POSITION_3, DOCK_POSITION_4, DOCK_POSITION_5,
    DOCK_POSITION_6, DOCK_POSITION_7, DOCK_POSITION_8, DOCK_POSITION_9, DOCK_POSITION_10,
    DOCK_POSITION_11, DOCK_POSITION_12, DOCK_POSITION_13, DOCK_POSITION_14, DOCK_POSITION_15,
    DOCK_POSITION_16, DOCK_POSITION_17, DOCK_POSITION_18, DOCK_POSITION_19, DOCK_POSITION_20
};

#define LEFT_ORIENTATION 0
#define RIGHT_ORIENTATION M_PI

const std::vector<float> PARK1_POSITION = {-4.0, 6.1};
const std::vector<float> PARK2_POSITION = {4.0, 6.1};

const std::vector<float> MIDDLE_PARKS = {0, 6.1};
const std::vector<float> MIDDLE_CORRIDOR_FRONT = {0, 4.5};
const std::vector<float> LEFT_CORRIDOR_FRONT = {-2.0, 2.7};
const std::vector<float> RIGHT_CORRIDOR_FRONT = {2.0, 2.7};
const std::vector<float> MIDDLE_CORRIDOR_FRONT_DD = {0.0, 3.5};
const std::vector<float> MIDDLE_CORRIDOR_BACK = {0, -5.5};
const std::vector<float> LEFT_CORRIDOR_BACK = {-2.0, -6.0};
const std::vector<float> RIGHT_CORRIDOR_BACK = {2.0, -6.0};

const std::vector<float> ROUTE_PARK1_TO_PARK2 = {0, 6.1,4.0, 6.1};
const std::vector<float> ROUTE_PARK2_TO_PARK1 = {0, 6.1,-4.0, 6.1};


#endif // POSITIONS_H

