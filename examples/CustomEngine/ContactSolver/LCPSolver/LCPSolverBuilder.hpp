#include "LCPSolverBase.h"
#include "NativeLemke/NativeLemkeLCPSolver.h"
#include "btLemke/btLemke.h"
#include "btDantzig/btDantzig.h"
#include "btPath/btPath.h"
#include <iostream>

cLCPSolverBase * BuildLCPSolver(const std::string & type)
{
    cLCPSolverBase * solver = nullptr;
    if(type == "NativeLemke")
    {
        solver = new cNativeLemkeLCPSolver();
    }
    else if (type == "btLemke")
    {
        solver = new cBulletLemkeLCPSolver();
    }
    else if (type == "btDantzig")
    {
        solver = new cBulletDantzigLCPSolver();
    }
    else if (type == "btPath")
    {
        solver = new cBulletPathLCPSolver();
    }
    else
    {
        std::cout <<"build LCP solver failed for type " << type << std::endl;
        exit(1);
    }
    
    return solver;
}