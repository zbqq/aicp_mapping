#ifndef FILE_UTILS_HPP_
#define FILE_UTILS_HPP_

#include <fstream>
#include <iostream>
#include <regex>

#include <Eigen/Dense>

using namespace std;


/* Get the line whose index is given as argument from file. */
string readLineFromFile(string& filename, int line_number);

Eigen::Matrix4f parseTransformationQuaternions(string transform);
Eigen::Matrix4f parseTransformationDeg(string& transform);

void write3DTransformToFile(Eigen::Matrix4f &transform, string out_file, int id_cloud_A, int id_cloud_B);
void writeTransformToFile(Eigen::MatrixXf &transformations, string out_file, int num_clouds);
void writeLineToFile(Eigen::MatrixXf &values, string out_file, int line_number);

void replaceRatioConfigFile(string in_file, string out_file, float ratio);

// Expand Environment Variables.
void autoExpandEnvironmentVariables( std::string & text );
std::string expandEnvironmentVariables( const std::string & input );

#endif
