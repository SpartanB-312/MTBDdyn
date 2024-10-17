#include "Solver.h"

// Constructor
Solver::Solver() {}

// Destructor
Solver::~Solver() {}

void Solver::EEulerBaumgarte() {
    for (const auto& rpcf : rpcfObjects) {
        // Perform operations on each RPCF object
        std::cout << "Position Matrix:\n" << rpcf.getPos() << std::endl;
    }
}

// Function to set RPCF objects
void Solver::setRPCFObjects(const std::vector<RPCF>& objects) {
    rpcfObjects = objects;
}

// Function to get RPCF objects
std::vector<RPCF> Solver::getRPCFObjects() const {
    return rpcfObjects;
}