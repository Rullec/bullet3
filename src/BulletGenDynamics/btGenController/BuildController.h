#pragma once
#include <string>
class btGenControllerBase;
class btGeneralizeWorld;
btGenControllerBase *BuildController(btGeneralizeWorld *world,
                                     const std::string &path);