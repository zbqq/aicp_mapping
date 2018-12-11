#include "registration_apps/yaml_configurator.hpp"
#include "aicp_common_utils/common.hpp"
#include "aicp_common_utils/fileIO.h"

using namespace std;

namespace aicp {

    bool YAMLConfigurator::parse(const std::string& path){
        yn_ = YAML::LoadFile(path);
        if(yn_.IsNull()){
            return false;
        }
        YAML::Node registrationNode = yn_["AICP"]["Registration"];
        for(YAML::const_iterator it=registrationNode.begin();it != registrationNode.end();++it) {

          const string key = it->first.as<string>();

          if(key.compare("type") == 0) {
            registration_params.type = it->second.as<string>();
          }
          else if(key.compare("sensorRange") == 0) {
            registration_params.sensorRange =  it->second.as<float>();
          }
          else if(key.compare("sensorAngularView") == 0) {
            registration_params.sensorAngularView =  it->second.as<float>();
          }
          else if(key.compare("loadPosesFrom") == 0) {
            registration_params.loadPosesFrom = it->second.as<string>();
          }
          else if(key.compare("initialTransform") == 0) {
            registration_params.initialTransform = it->second.as<string>();
          }
          else if(key.compare("saveCorrectedPose") == 0) {
            registration_params.saveCorrectedPose =  it->second.as<bool>();
          }
          else if(key.compare("saveInitializedReadingCloud") == 0) {
            registration_params.saveInitializedReadingCloud =  it->second.as<bool>();
          }
          else if(key.compare("saveRegisteredReadingCloud") == 0) {
            registration_params.saveRegisteredReadingCloud =  it->second.as<bool>();
          }
          else if(key.compare("enableLcmVisualization") == 0) {
            registration_params.enableLcmVisualization =  it->second.as<bool>();
          }
        }
        if(registration_params.type.compare("Pointmatcher") == 0) {

          YAML::Node pointmatcherNode = registrationNode["Pointmatcher"];

          for(YAML::const_iterator it=pointmatcherNode.begin();it != pointmatcherNode.end();++it) {
            const string key = it->first.as<string>();

            if(key.compare("configFileName") == 0) {
              registration_params.pointmatcher.configFileName.append(FILTERS_CONFIG_LOC);
              registration_params.pointmatcher.configFileName.append(PATH_SEPARATOR);
              registration_params.pointmatcher.configFileName = FILTERS_CONFIG_LOC + PATH_SEPARATOR + it->second.as<string>();
            }
            else if(key.compare("printOutputStatistics") == 0) {
              registration_params.pointmatcher.printOutputStatistics =  it->second.as<bool>();
            }
          }
        }
        else if(registration_params.type.compare("GICP") == 0) {

          YAML::Node gicpNode = registrationNode["GICP"];

          for(YAML::const_iterator it=gicpNode.begin();it != gicpNode.end();++it) {
            const string key = it->first.as<string>();
            const float val = it->second.as<float>();
          }
        }
        YAML::Node overlapNode = yn_["AICP"]["Overlap"];
        for(YAML::const_iterator it=overlapNode.begin();it != overlapNode.end();++it) {

          const string key = it->first.as<string>();

          if(key.compare("type") == 0) {
            overlap_params.type = it->second.as<string>();
          }
        }
        if(overlap_params.type.compare("OctreeBased") == 0) {

          YAML::Node octreeBasedNode = overlapNode["OctreeBased"];

          for(YAML::const_iterator it=octreeBasedNode.begin();it != octreeBasedNode.end();++it) {
            const string key = it->first.as<string>();

            if(key.compare("octomapResolution") == 0) {
              overlap_params.octree_based.octomapResolution = it->second.as<float>();
            }
          }
        }
        YAML::Node classificationNode = yn_["AICP"]["Classifier"];
        for (YAML::const_iterator it = classificationNode.begin(); it != classificationNode.end(); ++it) {
          const std::string key = it->first.as<std::string>();

          if (key.compare("type") == 0) {
            classification_params.type = it->second.as<std::string>();
          }
        }

        if (classification_params.type.compare("SVM") == 0) {

          YAML::Node svmNode = classificationNode["SVM"];

          for(YAML::const_iterator it=svmNode.begin();it != svmNode.end();++it) {
            const std::string key = it->first.as<std::string>();

            if(key.compare("threshold") == 0) {
              classification_params.svm.threshold = it->second.as<double>();
            }
            else if(key.compare("trainingFile") == 0) {
              classification_params.svm.trainingFile = expandEnvironmentVariables(it->second.as<std::string>());
            }
            else if(key.compare("testingFile") == 0) {
                classification_params.svm.testingFile = expandEnvironmentVariables(it->second.as<std::string>());
            }
            else if(key.compare("saveFile") == 0) {
              classification_params.svm.saveFile = expandEnvironmentVariables(it->second.as<std::string>());
            }
            else if(key.compare("saveProbs") == 0) {
              classification_params.svm.saveProbs = expandEnvironmentVariables(it->second.as<std::string>());
            }
            else if(key.compare("modelLocation") == 0) {
              classification_params.svm.modelLocation = expandEnvironmentVariables(it->second.as<std::string>());
            }
          }
        }
        YAML::Node experimentsNode = yn_["AICP"]["Experiments"];
        for(YAML::const_iterator it=experimentsNode.begin();it != experimentsNode.end();++it) {

          const string key = it->first.as<string>();

          if(key.compare("type") == 0) {
            experiments_param = it->second.as<string>();
          }
        }
        return true;
    }

    const RegistrationParams& YAMLConfigurator::getRegistrationParams() {
        return registration_params;
    }
    const OverlapParams& YAMLConfigurator::getOverlapParams() {
        return overlap_params;
    }

    const ClassificationParams& YAMLConfigurator::getClassificationParams() {
        return classification_params;
    }

    const std::string& YAMLConfigurator::getExperimentParams(){
        return experiments_param;
    }

    void YAMLConfigurator::printParams() {
        cout << "============================" << endl
             << "Parsed YAML Config" << endl
             << "============================" << endl;

        cout << "[Main] Registration Type: "                 << registration_params.type                          << endl;
        cout << "[Main] Sensor Range: "                      << registration_params.sensorRange                   << endl;
        cout << "[Main] Sensor Angular View: "               << registration_params.sensorAngularView             << endl;
        cout << "[Main] Load Poses from: "                   << registration_params.loadPosesFrom                 << endl;
        cout << "[Main] Initial Transform: "                 << registration_params.initialTransform              << endl;
        cout << "[Main] Save Corrected Pose: "               << registration_params.saveCorrectedPose             << endl;
        cout << "[Main] Save Initialized Reading Cloud: "    << registration_params.saveInitializedReadingCloud   << endl;
        cout << "[Main] Save Registered Reading Cloud: "     << registration_params.saveRegisteredReadingCloud    << endl;
        cout << "[Main] Enable Lcm Visualization: "          << registration_params.enableLcmVisualization        << endl;

        if(registration_params.type.compare("Pointmatcher") == 0) {
            cout << "[Pointmatcher] Config File Name: "                << registration_params.pointmatcher.configFileName        << endl;
            cout << "[Pointmatcher] Print Registration Statistics: "   << registration_params.pointmatcher.printOutputStatistics << endl;
        }
        else if(registration_params.type.compare("GICP") == 0) {
        }

        cout << "[Main] Overlap Type: "                   << overlap_params.type                             << endl;

        if(overlap_params.type.compare("OctreeBased") == 0) {
            cout << "[OctreeBased] Octomap Resolution: "    << overlap_params.octree_based.octomapResolution   << endl;
        }

        cout << "[Main] Classification Type: "       << classification_params.type                    << endl;

        if(classification_params.type.compare("SVM") == 0) {
              cout << "[SVM] Acceptance Threshold: "    << classification_params.svm.threshold           << endl;
              cout << "[SVM] Training File: "           << classification_params.svm.trainingFile        << endl;
              cout << "[SVM] Testing File: "            << classification_params.svm.testingFile         << endl;
              cout << "[SVM] Saving Model To: "         << classification_params.svm.saveFile            << endl;
              cout << "[SVM] Saving Probs To: "         << classification_params.svm.saveProbs           << endl;
              cout << "[SVM] Loading Model From: "      << classification_params.svm.modelLocation       << endl;
        }

        cout << "[Main] Experiments Type: "               << experiments_param                               << endl;
        cout << "============================" << endl;
    }
}

