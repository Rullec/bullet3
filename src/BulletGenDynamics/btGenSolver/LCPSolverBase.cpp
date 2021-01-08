#include "LCPSolverBase.h"

cLCPSolverBase::cLCPSolverBase(eLCPSolverType type) { mType = type; }

eLCPSolverType cLCPSolverBase::GetType() { return mType; }
