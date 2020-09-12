#include "LCPSolverBase.h"
#include "NativeLemke/NativeLemkeLCPSolver.h"
#include "btPath/btPath.h"
#include "odeDantzig/odeDantzigLCPSolver.h"
#include <iostream>

cLCPSolverBase *BuildLCPSolver(const std::string &type)
{
    cLCPSolverBase *solver = nullptr;
    if (type == "NativeLemke")
    {
        solver = new cNativeLemkeLCPSolver();
    }
    else if (type == "btPath")
    {
        solver = new cBulletPathLCPSolver();
    }
    else if (type == "odeDantzig")
    {
        solver = new cODEDantzigLCPSolver();
    }
    else
    {
        std::cout << "build LCP solver failed for type " << type << std::endl;
        exit(1);
    }

    return solver;
}