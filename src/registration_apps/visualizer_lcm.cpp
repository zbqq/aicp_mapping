#include "registration_apps/visualizer_lcm.hpp"

using namespace std;

namespace aicp {

LCMVisualizer::LCMVisualizer(boost::shared_ptr<lcm::LCM>& lcm) : lcm_vis_(lcm)
{}

}
