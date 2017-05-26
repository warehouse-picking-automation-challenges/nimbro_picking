// Joint subclass containing hw information
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <drc_interface/joint.h>

namespace drc_interface
{

Joint::Joint(const std::string& name)
 : id("joints/" + name + "/id", 1, 1, 253, 0)
 , encoderTicks("joints/" + name + "/encoderTicks", 1, 1, 1000000, 1)
 , maxTorque("joints/" + name + "/maxTorque", 0, 1, 32767, 0)
 , isTTL("joints/" + name + "/isTTL", false)
 , offset("joints/" + name + "/offset", -1000000, 1, 1000000, 0)
 , invert("joints/" + name + "/invert", false)
 , NmPerAmp("joints/" + name + "/NmPerAmp", 0.0, 0.1, 5.5, 4.5)
 , velocityMode("joints/" + name + "/velocityMode", false)
 , gearboxRatio("joints/" + name + "/gearboxRatio", 0.0, 1.0, 1000.0, 1.0)
 , maxVelocity("joints/" + name + "/maxVelocity", 0, 1, 32768, 1000)
 , fadeInVelocity("joints/" + name + "/fadeInVelocity", 1, 1, 32768, 2000)
 , link(nullptr)
 , temperature(0.0)
 , timeouts(0)
 , error(0)
 , timeoutConfidence(0.0)
{
}

}
