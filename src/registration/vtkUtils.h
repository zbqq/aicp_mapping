#ifndef __vtkMultisenseUtils_h
#define __vtkMultisenseUtils_h


#include <cstdio>
#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

//#include <lcm/lcm-cpp.hpp>

//#include <bot_frames/bot_frames.h>
//#include <bot_param/param_client.h>

//#include <lcmtypes/bot_core.hpp>
//#include <lcmtypes/multisense.hpp>

#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkFloatArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkIdTypeArray.h>
#include <vtkCellArray.h>
#include <vtkTimerLog.h>

#include "pointmatcher/PointMatcher.h"


//----------------------------------------------------------------------------
namespace
{

class DataArrays
{
public:

  vtkSmartPointer<vtkPolyData> Dataset;

  vtkPoints* Points;
  vtkFloatArray* Distance;
};

//----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts)
{
  vtkSmartPointer<vtkIdTypeArray> cells = vtkSmartPointer<vtkIdTypeArray>::New();
  cells->SetNumberOfValues(numberOfVerts*2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i)
    {
    ids[i*2] = 1;
    ids[i*2+1] = i;
    }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}

//-----------------------------------------------------------------------------
DataArrays createData(vtkIdType numberOfPoints)
{
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  // points
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToFloat();
  points->Allocate(numberOfPoints);
  polyData->SetPoints(points.GetPointer());
  polyData->SetVerts(NewVertexCells(numberOfPoints));

  // distance between matching points (ICP alignment)
  vtkSmartPointer<vtkFloatArray> distance = vtkSmartPointer<vtkFloatArray>::New();
  distance->SetName("distance_between_matches");
  distance->Allocate(numberOfPoints);
  polyData->GetPointData()->AddArray(distance.GetPointer());

  DataArrays arrays;
  arrays.Dataset = polyData;
  arrays.Points = points.GetPointer();
  arrays.Distance = distance.GetPointer();

  return arrays;
}

void savePointCloudVTK(const char *filename, PointMatcher<float>::DataPoints data, PM::Matrix distance = PM::Matrix::Zero(1,1))
{
  const int nbPoints = data.getNbPoints();

  DataArrays dataArrays = createData(nbPoints);

  if ((distance.rows() == 1) && (distance.cols() == 1))// If filled, distance is (1 X nbPoints).
  {
    PM::Matrix distance(1, nbPoints);  
    for (int i = 0; i < nbPoints; i++)
    {  
      distance(0, i) = -1;
    }
  }

  for ( unsigned int i = 0; i < nbPoints; ++i )
  {
    dataArrays.Points->InsertNextPoint( data.features(0,i), data.features(1,i), data.features(2,i) );

    dataArrays.Distance->InsertNextValue(distance(i));
  }

  dataArrays.Dataset->SetVerts(NewVertexCells(nbPoints));

  // Write to file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =  vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(filename);
  #if VTK_MAJOR_VERSION <= 5
    writer->SetInput(dataArrays.Dataset);
  #else
    writer->SetInputData(dataArrays.Dataset);
  #endif
 
  // Optional - set the mode. The default is binary.
  //writer->SetDataModeToBinary();
  //writer->SetDataModeToAscii();
  
  std::cout << "Writing to " << filename << "..." << std::endl;
  writer->Write();
}


} // end namespace



#endif
