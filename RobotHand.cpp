#include <iostream>
#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <queue>
#include <random>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

// Structs, Enumerations and Classes
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

struct hand {
    Eigen::Vector2d arm; // [bicep, forearm] [m]
    Eigen::Vector3d palm; // [depth, width, height] [m]
    Eigen::Vector3d thumb; // [first, second, third] [m]
    Eigen::Vector3d index; // [first, second, third] [m]
    Eigen::Vector3d middle; // [first, second, third] [m]
    Eigen::Vector3d ring; // [first, second, third] [m]
    Eigen::Vector3d little; // [first, second, third] [m]
    Eigen::VectorXd initAngle; // [..., knuckle, middle, tip, ...] [rad]
    Eigen::VectorXd finalAngle; // [..., knuckle, middle, tip, ...] [rad]
    int numFingers;
    // Creates struct of fingers, palm and arm to store necessary dimensions
    // The joint lengths are from the base to tip joint in the finger
    // The initial position vectors are the inital positions of each fingertip
    // The numFingers value defines how many fingers we wish to simulate
    // If you wish to add more fingers:
    //                                  - Define them here
    //                                  - Define finger lengths in "handDimensions"
    //                                  - Define initial angles in "handAngles"
    //                                  - Define final angles in "handAngles"
    //                                  - Define their value in "fingers"
    //                                  - Change numFingers in "handDimensions"
    //                                  - Define state space vector in "inSS"
    //                                  - Add new switch case in "getJointPosition"

};

struct ss {
    Eigen::Vector2d theta_s;
    Eigen::Vector2d theta_e;
    Eigen::Vector4d theta_b;
    Eigen::Vector4d theta_k;
    Eigen::Vector4d theta_m;
    Eigen::Vector4d theta_t;
    // Creates struct of allowable joint angles
    // The last 4 have 4 elements where the first 2 are for the thumb and
    // the last 2 are for any other finger...
    // More joints are not easily added without significant changes to code

};

enum fingers {
    thumb = 0,
    index = 1,
    middle = 2,
    ring = 3,
    little = 4
    // Assignment of a number to each finger for mathematical ease
    // If we wish to add more fingers please see "hand" struct!

};

struct JointInfo {
    fingers finger;
    int joint;
    std::string name;
};

class RRT {
    public:
        std::map<int, std::vector<Eigen::Vector3d>> allJoints;
        std::map<int, Eigen::VectorXd> tree;
        std::map<int, std::vector<int>> edges;
        // Creates an unordered map in order to quickly access the fingertip
        // positions in global coordinates and node position in state space...
        // Also creates map of connected edges in the tree...

        // How to add:
        //             allJoints[int].push_back(fingertipPos);
        //             tree[nodeNum] = nodePos;
        //             edges[nodeNum].push_back(connectedNode);

        // How to get:
        //             fingerPaths.at(int).at(nodeNum);

        std::vector<int> search(int start, int end);
        void Algorithm(hand &hand, fingers finger, ss &ss);
        // Defines RRT Algorithm as a member function of "RRT" class

};

// Functions
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

void handDimensions(hand &hand) {
    // Inputs: Hand structure
    // Output: none

    // Description: Assigns lengths for each joint in the hand and arm to the "hand" struct
    //              that we manually defined above.

    Eigen::Vector2d arm_lengths(32, 29);
    Eigen::Vector3d palm_dimensions(2.5, 6, 11);
    Eigen::Vector3d thumb_lengths(7, 4, 3.5);
    Eigen::Vector3d index_lengths(5, 3, 2.5);
    Eigen::Vector3d middle_lengths(5.5, 3.5, 2.5);
    Eigen::Vector3d ring_lengths(4.5, 3.5, 2.5);
    Eigen::Vector3d little_lengths(4, 2.5, 2.25);
    // Defines variables for length of joints in each finger
    // Sets values for finger lengths (measured from my own hand)
    // Starts from base joint and goes to tip joint

    hand.arm = arm_lengths/100;
    hand.palm = palm_dimensions/100;
    hand.thumb = thumb_lengths/100;
    hand.index = index_lengths/100;
    hand.middle = middle_lengths/100;
    hand.ring = ring_lengths/100;
    hand.little = little_lengths/100;
    hand.numFingers = 5;
    // Assigns finger lengths to "hand" struct

};

void jointAngles(ss &ss) {
    // Inputs: State Space structure
    // Output: none

    // Description: Assigns allowable joint angles to structure for definition
    //              of the allowable state space.

    Eigen::Vector2d shoulder(-M_PI/2, M_PI/2);
    Eigen::Vector2d elbow(0, 3*M_PI/4);
    Eigen::Vector4d base(0, M_PI/4, -M_PI/9, M_PI/9);
    Eigen::Vector4d knuckle(-M_PI/4, -M_PI/8, 0, M_PI/2);
    Eigen::Vector4d middle(0, M_PI/4, 0, 3*M_PI/4);
    Eigen::Vector4d tip(0, M_PI/2, 0, M_PI/2);
    // Defines allowable angle range for each joint along the arm and finger

    ss.theta_s = shoulder;
    ss.theta_e = elbow;
    ss.theta_b = base;
    ss.theta_k = knuckle;
    ss.theta_m = middle;
    ss.theta_t = tip;
    // Assigns allowable angle range to state space structure

};

Eigen::Matrix3d rotateAndTranslate3D(double z, double y, double x) {
    // Inputs: Rotation angles about x, y and z axes [radians]
    // Output: 3D Rotation Matrix

    // Description: Determine Rotation Matrix of a point around the origin (or some
    //              other point centered there). Does NOT calculate end point after
    //              rotation, just the Rotation Matrix needed to get there given the
    //              rotation angles...

    Eigen::Matrix3d Rx;
    Eigen::Matrix3d Ry;
    Eigen::Matrix3d Rz;
    Rx << 1, 0, 0,
          0, std::cos(x), -std::sin(x),
          0, std::sin(x), std::cos(x);
    Ry << std::cos(y), 0, std::sin(y),
          0, 1, 0,
          -std::sin(y), 0, std::cos(y);
    Rz << std::cos(z), -std::sin(z), 0,
          std::sin(z), std::cos(z), 0,
          0, 0, 1;
    // Calculates Rotation Matrix

    return Rz*(Ry*Rx);
    // Returns Rotation Matrix

};

void handAngles(hand &hand, int config) {
    // Inputs: Hand structure
    //         Configuration
    // Output: none

    // Description: Assigns initial positions of each fingertip in the "hand" struct above.

    Eigen::VectorXd initial(4*hand.numFingers + 2);
    Eigen::VectorXd final(4*hand.numFingers + 2);
    // Variable Initialization

    initial << 0, 0,
                0, -3*M_PI/16, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;
    // Defines variables for final angles
    // First two are the shoulder and elbow angles, next are the 4 angles for
    // each finger in series (thumb, index, middle, etc.)...

    switch (config) {
        case 0:
            final << 0, M_PI/2,
                     0, -M_PI/8, M_PI/4, M_PI/2,
                     -M_PI/9, 0, 0, 0,
                     M_PI/9, 0, 0, 0,
                     0, M_PI/2, 3*M_PI/4, 0,
                     0, M_PI/2, 3*M_PI/4, 0;
            break;
            // Defines variables for final angles
            // First two are the shoulder and elbow angles, next are the 4 angles for
            // each finger in series (thumb, index, middle, etc.)...

        case 1:
            final << M_PI/2, 0,
                     M_PI/4, -M_PI/4, M_PI/4, M_PI/2,
                     0, M_PI/2, 3*M_PI/4, M_PI/2,
                     0, M_PI/2, 3*M_PI/4, M_PI/2,
                     0, M_PI/2, 3*M_PI/4, M_PI/2,
                     0, M_PI/2, 3*M_PI/4, M_PI/2;
            break;
            // Defines variables for final angles
            // First two are the shoulder and elbow angles, next are the 4 angles for
            // each finger in series (thumb, index, middle, etc.)...

        default: 
            std::cerr << "Invalid configuration" << std::endl;
            std::exit(EXIT_FAILURE);
            // Throw error for invalid configuration

    }
    // Changes inital and final angles based on selection

    hand.initAngle = initial;
    hand.finalAngle = final; 
    // Assigns initial and final angles to "hand" struct

};

Eigen::Matrix4d createTransformMatrix(double z, double y, double x, double l) {
    // Inputs: Angles to consider when defining Transformation Matrix
    //         Length of joint segment
    // Output: Transformation Matrix

    // Description: Builds Homogeneous Transformation Matrix

    Eigen::Matrix4d T;
    // Initializes Transformation Matrix

    T << rotateAndTranslate3D(z, y, x), Eigen::Vector3d(0, 0, l),
         0, 0, 0, 1;
    // Defines Transformation Matrix

    return T;
    // Returns Transformation Matrix

};

Eigen::Vector3d getJointPosition(Eigen::VectorXd totAngles, fingers finger, int joint, hand &hand) {
    // Inputs: Total system angles
    //         Finger to find the tip position of
    //         Joint of the finger you want the position of (knuckle, middle, tip)
    //         Finger length struct
    // Output: Position in global coordinates of the tip of the selected finger

    // Description: Calculates the tip position of the selected finger in the
    //              global coordinate system. This is done using a series of
    //              Homogeneous Transformation Matrices. The base position is
    //              initially assumed to be the origin but can be changed to
    //              arbitrary position in the global frame. This is assumed to
    //              be the RIGHT hand, not the left hand. Changes must be made
    //              if we desire to simulate the left hand intead.

    Eigen::VectorXd angles(6);
    angles << totAngles[0], totAngles[1], totAngles.segment<4>(4*static_cast<int>(finger) + 2);
    // Defines angles for specific finger

    Eigen::Vector3d result;
    Eigen::Vector4d base;
    Eigen::Vector4d pos;
    base << 0, 0, 0, 1;
    // Defines starting, tip and palm translation positions in homogeneous
    // coordinates. The first three elements of the starting position are
    // the origin.

    double special;
    Eigen::Matrix4d T_1;
    Eigen::Matrix4d T_2;
    Eigen::Matrix4d T_3;
    Eigen::Matrix4d T_4;
    Eigen::Matrix4d T_5;
    Eigen::Matrix4d T_6;
    Eigen::Matrix4d T_7;
    Eigen::Matrix4d T_special;
    // Initializes Transformation Matrices for each joint in the hand and arm

    T_7 = createTransformMatrix(0, 0, -angles[0], 0); // Rotate shoulder but dont translate frame
    T_6 = createTransformMatrix(0, 0, -angles[1], hand.arm[0]); // Rotate elbow and translate frame to shoulder
    T_5 = createTransformMatrix(0, 0, 0, hand.arm[1]); // Dont rotate wrist but translate frame to elbow
    // Defines Transformation Matrices for getting to the wrist of the hand

    switch(finger) {
        case fingers::thumb:
            T_4 << rotateAndTranslate3D(0, angles[2], angles[3]), Eigen::Vector3d(hand.palm[0]/2, hand.palm[1]/2, 0),
                   0, 0, 0, 1; // Rotate base and translate frame to wrist
            T_3 = createTransformMatrix(0, 0, angles[4], hand.thumb[0]); // Rotate knuckle and translate frame to base
            T_2 = createTransformMatrix(0, 0, angles[5], hand.thumb[1]); // Rotate middle and translate frame to knuckle
            T_1 = createTransformMatrix(0, 0, 0, hand.thumb[2]); // Dont rotate tip and translate frame to middle
            // Defines Transformation Matrices for each joint of the thumb

            if (joint <= 1){
                special = hand.thumb[joint];
            }
            break;
            // Break out of the switch statement

        case fingers::index:
            T_4 << rotateAndTranslate3D(0, angles[3], angles[2]), Eigen::Vector3d(0, 3*hand.palm[1]/8, hand.palm[2]),
                   0, 0, 0, 1; // Rotate base and translate frame to wrist
            T_3 = createTransformMatrix(0, angles[4], 0, hand.index[0]); // Rotate knuckle and translate frame to base
            T_2 = createTransformMatrix(0, angles[5], 0, hand.index[1]); // Rotate middle and translate frame to knuckle
            T_1 = createTransformMatrix(0, 0, 0, hand.index[2]); // Dont rotate tip and translate frame to middle
            // Defines Transformation Matrices for each joint of the index finger

            if (joint <= 1) {
                special = hand.index[joint];
            }
            break;
            // Break out of the switch statement

        case fingers::middle:
            T_4 << rotateAndTranslate3D(0, angles[3], angles[2]), Eigen::Vector3d(0, hand.palm[1]/8, hand.palm[2]),
                   0, 0, 0, 1; // Rotate base and translate frame to wrist
            T_3 = createTransformMatrix(0, angles[4], 0, hand.middle[0]); // Rotate knuckle and translate frame to base
            T_2 = createTransformMatrix(0, angles[5], 0, hand.middle[1]); // Rotate middle and translate frame to knuckle
            T_1 = createTransformMatrix(0, 0, 0, hand.middle[2]); // Dont rotate tip and translate frame to middle
            // Defines Transformation Matrices for each joint of the middle finger

            if (joint <= 1) {
                special = hand.middle[joint];
            }
            break;
            // Break out of the switch statement

        case fingers::ring:
            T_4 << rotateAndTranslate3D(0, angles[3], angles[2]), Eigen::Vector3d(0, -hand.palm[1]/8, hand.palm[2]),
                   0, 0, 0, 1; // Rotate base and translate frame to wrist
            T_3 = createTransformMatrix(0, angles[4], 0, hand.ring[0]); // Rotate knuckle and translate frame to base
            T_2 = createTransformMatrix(0, angles[5], 0, hand.ring[1]); // Rotate middle and translate frame to knuckle
            T_1 = createTransformMatrix(0, 0, 0, hand.ring[2]); // Dont rotate tip and translate frame to middle
            // Defines Transformation Matrices for each joint of the middle finger

            if (joint <= 1) {
                special = hand.ring[joint];
            }
            break;
            // Break out of the switch statement

        case fingers::little:
            T_4 << rotateAndTranslate3D(0, angles[3], angles[2]), Eigen::Vector3d(0, -3*hand.palm[1]/8, hand.palm[2]),
                   0, 0, 0, 1; // Rotate base and translate frame to wrist
            T_3 = createTransformMatrix(0, angles[4], 0, hand.little[0]); // Rotate knuckle and translate frame to base
            T_2 = createTransformMatrix(0, angles[5], 0, hand.little[1]); // Rotate middle and translate frame to knuckle
            T_1 = createTransformMatrix(0, 0, 0, hand.little[2]); // Dont rotate tip and translate frame to middle
            // Defines Transformation Matrices for each joint of the middle finger

            if (joint <= 1) {
                special = hand.little[joint];
            }
            break;
            // Break out of the switch statement

        default:
            std::cout << "Invalid Finger" << std::endl;
            std::exit(EXIT_FAILURE);
            // Throw error for invalid finger

    }
    // Switches through selected finger and calculates Transfomation Matrices
    // and palm translations

    switch(joint) {
        case 0:
            T_special = createTransformMatrix(0, 0, 0, special); // Dont rotate knuckle and translate frame to base
            pos = T_7*(T_6*(T_5*(T_4*(T_special*base))));
            break;
            // Knuckle joint position

        case 1:
            T_special = createTransformMatrix(0, 0, 0, special); // Dont rotate middle and translate frame to knuckle
            pos = T_7*(T_6*(T_5*(T_4*(T_3*(T_special*base)))));
            break;
            // Middle joint position

        case 2:
            pos = T_7*(T_6*(T_5*(T_4*(T_3*(T_2*(T_1*base))))));
            break;
            // Tip joint position

        case 3:
            pos = T_7*(T_6*(T_5*(T_4*base)));
            break;
            // Base joint position

        case 4:
            pos = T_7*(T_6*(T_5*base));
            break;
            // Wrist joint position

        case 5:
            pos = T_7*(T_6*base);
            break;
            // Elbow joint position

        default:
            std::cerr << "Invalid joint position requested" << std::endl;
            std::exit(EXIT_FAILURE);
            // Throw error for invalid joint

    }
    // Defines tip position of the selected finger from its base

    result = pos.head<3>();
    return result;
    // Return position in global coordinates of selected finger

};

Eigen::VectorXd getAngularDistance(Eigen::VectorXd q1, Eigen::VectorXd q2) {
    // Inputs: Two angular composed space vectors
    // Output: Non-Euclidean "distance" vector between them

    // Description: 

    Eigen::VectorXd dist(q1.size());
    // Initializes distance vector

    for (int i = 0; i < q1.size(); ++i) {
        double diff = std::fmod(q2[i] - q1[i], 2*M_PI);
        if (diff > M_PI) diff -= 2*M_PI;
        if (diff < -M_PI) diff += 2*M_PI;
        dist[i] = diff;
        // Assigns Non-Euclidean distance to distance vector for elements i

    }
    // Builds Non-Euclidean distance vector

    return dist;
    // Return distance vector

};

bool inObstacle(Eigen::VectorXd point, hand &hand) {
    // Inputs: Point in composed space to check for finger collision in
    //         Hand structure
    // Output: Logical if collision between fingers is detected

    // Description: 

    int m = hand.numFingers;
    // Defines number of fingers to check for

    double minDist = 0.015; // [m]
    // Minium distance allowed between joints in each finger
    
    for (int i = 0; i < m; ++i) {
        Eigen::Vector3d basePos1 = getJointPosition(point, static_cast<fingers>(i), 0, hand);
        Eigen::Vector3d midPos1 = getJointPosition(point, static_cast<fingers>(i), 1, hand);
        Eigen::Vector3d tipPos1 = getJointPosition(point, static_cast<fingers>(i), 2, hand);
        // Get positions of all joints for finger i

        for (int j = i + 1; j < m; ++j) {

            Eigen::Vector3d basePos2 = getJointPosition(point, static_cast<fingers>(j), 0, hand);
            Eigen::Vector3d midPos2 = getJointPosition(point, static_cast<fingers>(j), 1, hand);
            Eigen::Vector3d tipPos2 = getJointPosition(point, static_cast<fingers>(j), 2, hand);
            // Get positions of all joints for finger j

            if ((basePos1 - basePos2).norm() < minDist ||
                (basePos1 - midPos2).norm() < minDist ||
                (basePos1 - tipPos2).norm() < minDist ||
                (midPos1 - basePos2).norm() < minDist ||
                (midPos1 - midPos2).norm() < minDist ||
                (midPos1 - tipPos2).norm() < minDist ||
                (tipPos1 - basePos2).norm() < minDist ||
                (tipPos1 - midPos2).norm() < minDist ||
                (tipPos1 - tipPos2).norm() < minDist) {
                return true;
                // If any joint distance is less than 1.5 cm then a collision is detected

            }
            // Check all possible joint combinations between fingers

        }
        // Loops through finger j and compares with finger i

    }
    // Loops through finger i
    
    return false;
    // If no collision is detected return false

};

bool inSS (ss &ss, Eigen::VectorXd point) {
    // Inputs: State Space
    //         Point in state space
    // Output: Logical if the point is a valid point in the state space

    // Description: 

    Eigen::VectorXd ssvec(2*point.size());
    ssvec << ss.theta_s, ss.theta_e,
             ss.theta_b.segment<2>(0), ss.theta_k.segment<2>(0), ss.theta_m.segment<2>(0), ss.theta_t.segment<2>(0),
             ss.theta_b.segment<2>(2), ss.theta_k.segment<2>(2), ss.theta_m.segment<2>(2), ss.theta_t.segment<2>(2),
             ss.theta_b.segment<2>(2), ss.theta_k.segment<2>(2), ss.theta_m.segment<2>(2), ss.theta_t.segment<2>(2),
             ss.theta_b.segment<2>(2), ss.theta_k.segment<2>(2), ss.theta_m.segment<2>(2), ss.theta_t.segment<2>(2),
             ss.theta_b.segment<2>(2), ss.theta_k.segment<2>(2), ss.theta_m.segment<2>(2), ss.theta_t.segment<2>(2);
    // Defines state space vector (currently hardcoded)

    for (int i = 0; i < point.size(); ++i) {
        if (point[i] < ssvec[2*i] || point[i] > ssvec[2*i + 1]) {
            return false;
        }
    }
    // Checks to make sure that the point in our composed space is inside the state space

    return true;
    // If all points are valid then return true

};

bool isValid(Eigen::VectorXd start, Eigen::VectorXd end, hand &hand, ss &ss) {
    // Inputs: Start configuration in composed space
    //         End configuration in composed space
    //         Hand structure
    // Output: Logical if line is collision free

    // Description: 

    int n = 100;
    // Number of points to check along the line

    Eigen::VectorXd point(start.size());
    // Initalizes point in composed space

    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i)/static_cast<double>(n - 1);
        for (int j = 0; j < start.size(); ++j) {
            double diff = std::fmod(end[j] - start[j], 2*M_PI);
            if (diff > M_PI) diff -= 2*M_PI;
            if (diff < -M_PI) diff += 2*M_PI;
            point[j] = start[j] + t*diff;
        }
        // Defines point on line between start and end points

        if (inObstacle(point, hand) || !inSS(ss, point)) {
            return false;
        }
        // If any point along the line is in an obstacle, the line is not valid

    }
    // Loops through all n points along line between start and end

    return true;
    // Returns true if no collision is found

};

void saveVectorToFile(const std::vector<Eigen::Vector3d>& data, const std::string& filename) {
    std::string fullPath = "your directory" + filename;
    // Specify your desired path

    std::ofstream file(fullPath);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << fullPath << std::endl;
        return;
    }

    for (const auto& vec : data) {
        file << vec.x() << ", " << vec.y() << ", " << vec.z() << std::endl;
    }
    file.close();
};

std::vector<int> RRT::search(int start, int end) {
    // Inputs: Start node
    //         End node
    // Output: Node Path to final node

    // Description: 

    std::vector<int> result;
    // Initializes result path variable

    std::unordered_set<int> closed_set;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> open_set;
    // Creates min-heap structure of NodeCostPair's and the vector of other pair's
    // The greater argument makes it a min-heap (smallest at top) and not a max-heap (largest at top)

    std::unordered_map<int, double> edge_cost;
    // Creates mapping to store (node, lowest cost to reach that node)
    // Can get edge_cost <double> (value) by using node <amp::Node> (key)

    std::unordered_map<int, int> parent;
    // Creates structure to store (node, parent node with lowest cost)
    // Can get parent node <amp::Node> (value) by using node <amp::Node> (key)

    open_set.push({0, start});
    // Starts off open set with start node and its cost of 0

    edge_cost[start] = 0;
    // Sets start node to having a cost of 0 (for each node we update this if we find a lower cost)

    while (!open_set.empty()) {

        int currNode = open_set.top().second;
        open_set.pop();
        closed_set.insert(currNode);
        // Sets node at top of open set as current node to explore
        // Moves that node to closed set and removes from open set

        if (currNode == end) {

            int prevNode = currNode;
            // Sets previous node to current node
            // We start backtracking (via parent nodes) from the current node (which is the goal node at this point)

            while (prevNode != start) {
                result.push_back(prevNode);
                prevNode = parent[prevNode];
            }
            // Places previous node in path result (building path in reverse order)
            // Repeats until previous node is start node

            result.push_back(start);
            std::reverse(result.begin(), result.end());
            // Places start node in result path last
            // Reverses order so it goes from start to goal node

            return result;
            // Sets path cost as total cost to get to goal node

        }
        // Outputs result if goal found before all nodes have been searched

        std::vector<int> children = edges.at(currNode);
        // Gets children of current node

        for (int i = 0; i < children.size(); ++i) {

            if (closed_set.find(children[i]) != closed_set.end()) {
                continue;
            }
            // If the child is in the closed set, then skip this iteration of the for loop

            double edge_cost_tot = edge_cost[currNode] + 1;
            // Gets cost from start to child (cost from start to parent + cost from parent to child)

            if (edge_cost.find(children[i]) == edge_cost.end() || edge_cost_tot < edge_cost[children[i]]) {

                edge_cost[children[i]] = edge_cost_tot;
                // Defines edge cost for child as the total cost to get to that child (excluding heuristic)

                parent[children[i]] = currNode;
                // (Re)define parent for that child

                open_set.push(std::make_pair(edge_cost_tot, children[i]));
                // Add child and its total cost to open set

            }
            // Checks to see if child is in open set (by extension of having an associated cost) or if its current cost is higher than new cost
            // Updates cost and adds to open set if it wasnt already there

        }
        // Loops through all children of current node

    }
    // Loop until open set is empty (all nodes checked)

    std::cerr << "A_star: No path found..." << std::endl;
    std::exit(EXIT_FAILURE);
    // Stops if no path is found

    return result;
    // If we exit the while loop without finding the goal, return failure

};

void RRT::Algorithm(hand &hand, fingers finger, ss &ss) {
    // Inputs: Hand Structure
    //         Finger Enumeration
    //         State Space structure
    // Output: Prints (global x, y, z) path of each fingertip

    // Description: This is achieved by creating a composed cspace and then
    //              navigating said cspace. The composed cspace has size m*q
    //              (m = number of fingers and q = configurations per agent).
    //              Use GoalBiasedRRT to determine path of multidimensional
    //              cspace. After each step, confirm each agent is not in
    //              collision with either an onstacle or another agent.

    double p_goal = 0.5;
    double r = 0.05;
    double maxIterations = 1000000;
    double epsilon = 0.01;
    // Tuning variables

    int m = hand.numFingers;
    // Defines number of agents

    Eigen::VectorXd q_start(4*m + 2);
    Eigen::VectorXd q_end(4*m + 2);
    Eigen::VectorXd pos_start(3*m);
    Eigen::VectorXd pos_end(3*m);
    // Creates start and end angle vectors for composed state space
    // Creates start and end position vectors in global coordinates

    q_start = hand.initAngle;
    // Populates q_start with angles for start region

    q_end = hand.finalAngle;
    // Populates q_end with angles for end region

    pos_start.segment<3>(0) << getJointPosition(q_start, static_cast<fingers>(0), 2, hand);
    pos_start.segment<3>(3) << getJointPosition(q_start, static_cast<fingers>(1), 2, hand);
    pos_start.segment<3>(6) << getJointPosition(q_start, static_cast<fingers>(2), 2, hand);
    pos_start.segment<3>(9) << getJointPosition(q_start, static_cast<fingers>(3), 2, hand);
    pos_start.segment<3>(12) << getJointPosition(q_start, static_cast<fingers>(4), 2, hand);
    // Populates pos_start in global coordinates with initial finger positions

    pos_end.segment<3>(0) << getJointPosition(q_end, static_cast<fingers>(0), 2, hand);
    pos_end.segment<3>(3) << getJointPosition(q_end, static_cast<fingers>(1), 2, hand);
    pos_end.segment<3>(6) << getJointPosition(q_end, static_cast<fingers>(2), 2, hand);
    pos_end.segment<3>(9) << getJointPosition(q_end, static_cast<fingers>(3), 2, hand);
    pos_end.segment<3>(12) << getJointPosition(q_end, static_cast<fingers>(4), 2, hand);
    // Populates pos_end in global coordinates with final finger positions

    std::vector<JointInfo> jointSequence = {
        {static_cast<fingers>(0), 5, "Elbow"},
        {static_cast<fingers>(0), 4, "Wrist"},
        {static_cast<fingers>(0), 3, "Base thumb"},
        {static_cast<fingers>(1), 3, "Base index"},
        {static_cast<fingers>(2), 3, "Base middle"},
        {static_cast<fingers>(3), 3, "Base ring"},
        {static_cast<fingers>(4), 3, "Base little"},
        {static_cast<fingers>(0), 0, "Knuckle thumb"},
        {static_cast<fingers>(1), 0, "Knuckle index"},
        {static_cast<fingers>(2), 0, "Knuckle middle"},
        {static_cast<fingers>(3), 0, "Knuckle ring"},
        {static_cast<fingers>(4), 0, "Knuckle little"},
        {static_cast<fingers>(0), 1, "Middle thumb"},
        {static_cast<fingers>(1), 1, "Middle index"},
        {static_cast<fingers>(2), 1, "Middle middle"},
        {static_cast<fingers>(3), 1, "Middle ring"},
        {static_cast<fingers>(4), 1, "Middle little"},
        {static_cast<fingers>(0), 2, "Tip thumb"},
        {static_cast<fingers>(1), 2, "Tip index"},
        {static_cast<fingers>(2), 2, "Tip middle"},
        {static_cast<fingers>(3), 2, "Tip ring"},
        {static_cast<fingers>(4), 2, "Tip little"}
    };
    // Joint info

    for (size_t i = 0; i < jointSequence.size(); ++i) {
        allJoints[i].push_back(getJointPosition(q_start, jointSequence[i].finger, jointSequence[i].joint, hand));
    }
    tree[0] = q_start;
    // Defines root node in tree as start composed configuration

    Eigen::VectorXd q_rand(4*m + 2);
    int nearest_node;
    Eigen::VectorXd q_near(4*m + 2);
    Eigen::VectorXd dir(4*m + 2);
    Eigen::VectorXd new_point(4*m + 2);
    // Defines random, nearest, directional and new composed space vectors

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_s(ss.theta_s[0], ss.theta_s[1]);
    std::uniform_real_distribution<> dis_e(ss.theta_e[0], ss.theta_e[1]);
    std::uniform_real_distribution<> dis_bt(ss.theta_b[0], ss.theta_b[1]);
    std::uniform_real_distribution<> dis_kt(ss.theta_k[0], ss.theta_k[1]);
    std::uniform_real_distribution<> dis_mt(ss.theta_m[0], ss.theta_m[1]);
    std::uniform_real_distribution<> dis_tt(ss.theta_t[0], ss.theta_t[1]);
    std::uniform_real_distribution<> dis_b(ss.theta_b[2], ss.theta_b[3]);
    std::uniform_real_distribution<> dis_k(ss.theta_k[2], ss.theta_k[3]);
    std::uniform_real_distribution<> dis_m(ss.theta_m[2], ss.theta_m[3]);
    std::uniform_real_distribution<> dis_t(ss.theta_t[2], ss.theta_t[3]);
    // Defines random number joint distribution range for each finger
    // Note the thumb has a different allowable range than the rest of the fingers

    std::vector<bool> agentReachedGoal(m, false);
    // Track goal status for each agent

    for (int i = 0; i < maxIterations; ++i) {
        q_rand.setZero();
        q_near.setZero();
        dir.setZero();
        new_point.setZero();
        // Sets q_rand to 0 upon new loop or at the start

        if (std::uniform_real_distribution<>(0.0, 1.0)(gen) <= p_goal) {
            q_rand = q_end;
        }
        else {
            q_rand.segment<2>(0) << dis_s(gen), dis_e(gen);
            q_rand.segment<20>(2) << dis_bt(gen), dis_kt(gen),
                                     dis_mt(gen), dis_tt(gen),
                                     dis_b(gen), dis_k(gen),
                                     dis_m(gen), dis_t(gen),
                                     dis_b(gen), dis_k(gen),
                                     dis_m(gen), dis_t(gen),
                                     dis_b(gen), dis_k(gen),
                                     dis_m(gen), dis_t(gen),
                                     dis_b(gen), dis_k(gen),
                                     dis_m(gen), dis_t(gen);
        }
        // Either samples end configuration or random configuration
        
        int nearest_node;
        double smallestDist = std::numeric_limits<double>::max();
        for (const auto& [node_id, position] : tree) {
            double distance = getAngularDistance(position, q_rand).norm();
            if (distance < smallestDist) {
                smallestDist = distance;
                nearest_node = node_id;
                q_near = position;
            }
        }
        // Finds position and node number of the nearest node in the current tree
        // to q_rand...

        dir = (q_rand - q_near).normalized();
        for (int j = 0; j < m; ++j) {
            if (agentReachedGoal[j]) {
                dir.segment<4>(4*j + 2) = Eigen::Vector4d(0, 0, 0, 0);
            }
        }
        // This ensures fingers who have reached their goal stay at their goal
        // positions...
        // Set direction vector to move fingers that arent in their goal
        // configurations...

        new_point = q_near + r*dir;
        bool got = false;
        bool atGoal = false;
        // Move 1 step size towards random node

        while (isValid(q_near, new_point, hand, ss)) {
            tree[tree.size()] = new_point;
            for (size_t i = 0; i < jointSequence.size(); ++i) {
                allJoints[i].push_back(getJointPosition(new_point, jointSequence[i].finger, jointSequence[i].joint, hand));
            }
            // Adds new node to tree
            // Adds position of agent to fingersPath

            edges[nearest_node].push_back(tree.size() - 1);
            // Connects new node to nearest node in tree

            bool allGoalsReached = true;
            // Track if all agents reached their goals

            for (int j = 0; j < m; ++j) {
                if (!agentReachedGoal[j]) {
                    Eigen::Vector3d finger_pos(getJointPosition(new_point, static_cast<fingers>(j), 2, hand));
                    Eigen::Vector3d finger_goal(pos_end.segment<3>(3*j));
                    // If agent is not at goal, check distance to goal

                    if ((finger_pos - finger_goal).norm() <= epsilon) {
                        agentReachedGoal[j] = true;
                        got = true;
                    }
                    else {
                        allGoalsReached = false;
                    }
                }
                // If agent j has reached its end configuration, set its reached goal to true

            }
            // Check if each agent has reached its end configuration

            if (allGoalsReached) {
                atGoal = true;
                nearest_node = tree.size() - 1;
                tree[tree.size()] = q_end;
                edges[nearest_node].push_back(tree.size() - 1);
                for (size_t i = 0; i < jointSequence.size(); ++i) {
                    allJoints[i].push_back(getJointPosition(q_end, jointSequence[i].finger, jointSequence[i].joint, hand));
                }
                break;
            }
            if (got) {
                break;
            }
            // If all agents have reached their goals, set goal state to true
            // Add the composed end configuration to nodes and connect the new point and q_end

            nearest_node = tree.size() - 1;
            new_point += r*dir;
            // Moves again towards q_rand and repeat the above process

        }
        // Continuously adds configuration to tree along path between q_near and q_rand
        // Stops when configuration is invalid or at end configuration

        if (atGoal) {
            break;
        }
        // Once composed end configuration has been reached, break the for loop

        if (i >= maxIterations) {
            std::cout << std::endl;
            std::cout << std::endl;
            std::cerr << "Goal not reached in Maximum alloted iterations" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        // Stops function if max iterations is reached

    }
    // Loops for maximum number of iterations
    // Essentially the algorithm can generate maxIterations number of random nodes to expand tree

    std::vector<int> pathDijkstra = search(0, tree.size() - 1);
    // Uses Dijkstra's Algorithm on the tree to find a path and return it

    for (int i = 0; i < 4*m + 2; ++i) {
        std::ostringstream oss;
        oss << "Joint_" << static_cast<int>(i) << "_Movement.csv";
        std::string filename = oss.str();
        saveVectorToFile(allJoints.at(i), filename);
    }
    // Loops through all fingers and writes their contents to a file

};

// Main
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

int main() {
    auto start_time = std::chrono::high_resolution_clock::now();
    // Starts runtime clock

    hand myHand;
    ss stateSpace;
    fingers finger;
    RRT rrt;
    // Defines needed variables for structs, enumerations and classes

    int config = 1; // [0, 1]
    // Select configuration for starting and ending configuration

    handDimensions(myHand);
    handAngles(myHand, config);
    jointAngles(stateSpace);
    // Calls necessary functions for initialization of variables to be passed into later functions

    for (int i = 0; i < 20; ++i) {
        auto start_time = std::chrono::high_resolution_clock::now();
        // Starts runtime clock

        rrt.Algorithm(myHand, finger, stateSpace);
        // Calls RRT function for our problem

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> runtime = end_time - start_time;
        std::cout << runtime.count() << " ";
        // Ends clock and prints runtime

    }
    std::cout << std::endl;
    // Benchmarking

    rrt.Algorithm(myHand, finger, stateSpace);
    // Calls RRT function for our problem

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> runtime = end_time - start_time;
    std::cout << "\nRuntime: " << runtime.count() << " seconds\n" << std::endl;
    // Ends clock and prints runtime

    return 0;
    // Return exit code 0

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
