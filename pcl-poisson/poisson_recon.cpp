#include <iostream>
#include <thread>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace std;

//http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

void
printUsage (const char* progName)
{
    std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
    << "Options:\n"
    << "-------------------------------------------\n"
    << "-h           this help\n"
    << "-p           Point cloud visualisation\n"
    << "-r           Reconstruction visualisation\n"
    << "-n           Normals visualisation\n"
    << "\n\n";
}

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr normalsVis (
                                                   pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    //viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr meshVis (
                                                   pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                PolygonMesh mesh
                                                )
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and mesh-----
    // --------------------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPolygonMesh(mesh, "mesh");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

unsigned int text_id = 0;
unsigned int id = 0;

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
    
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
    pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *> (viewer_void);
    if (event.getKeySym () == "r" && event.keyDown ())
    {
        //
    }else if(event.getKeySym () == "p" && event.keyDown ()){
        id = 1;
        //viewer->addPolygonMesh(mesh, "my");
    }else if(event.getKeySym () == "n" && event.keyDown ()){
        id = 2;
        //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    }
}

int
  main (int argc, char** argv)
{
    if (pcl::console::find_argument (argc, argv, "-h") >= 0)
    {
        printUsage (argv[0]);
        return 0;
    }
    bool simple(false), recon(false), normals(false);
    if (pcl::console::find_argument (argc, argv, "-p") >= 0)
    {
        simple = true;
        std::cout << "Point cloud visualisation\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-r") >= 0)
    {
        recon = true;
        std::cout << "Reconstruction visualisation\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-n") >= 0)
    {
        normals = true;
        std::cout << "Normals visualisation\n";
    }
    else
    {
        printUsage (argv[0]);
        return 0;
    }

   PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
   if(io::loadPLYFile<PointXYZ> (argv[2], *cloud) == -1){
      cout << "fail" << endl;

   } else {

      cout << "loaded" << endl;

      cout << "begin passthrough filter" << endl;
      PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
      PassThrough<PointXYZ> filter;
      filter.setInputCloud(cloud);
      filter.filter(*filtered);
      cout << "passthrough filter complete" << endl;

      // cout << "begin moving least squares" << endl;
      // MovingLeastSquares<PointXYZ, PointXYZ> mls;
      // mls.setInputCloud(filtered);
      // mls.setSearchRadius(0.01);
      // mls.setPolynomialFit(true);
      // mls.setPolynomialOrder(2);
      // mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
      // mls.setUpsamplingRadius(0.005);
      // mls.setUpsamplingStepSize(0.003);

      // PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ>());
      // mls.process(*cloud_smoothed);
      // cout << "MLS complete" << endl;

      cout << "begin normal estimation" << endl;
      NormalEstimationOMP<PointXYZ, Normal> ne;
      ne.setNumberOfThreads(8);
      ne.setInputCloud(filtered);
      ne.setRadiusSearch(0.01);
      Eigen::Vector4f centroid;
      compute3DCentroid(*filtered, centroid);
      ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

      PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
      ne.compute(*cloud_normals);
      cout << "normal estimation complete" << endl;
      cout << "reverse normals' direction" << endl;

      for(size_t i = 0; i < cloud_normals->size(); ++i){
      	cloud_normals->points[i].normal_x *= -1;
      	cloud_normals->points[i].normal_y *= -1;
      	cloud_normals->points[i].normal_z *= -1;
      }

      cout << "combine points and normals" << endl;
      PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
      concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

      cout << "begin poisson reconstruction" << endl;
      Poisson<PointNormal> poisson;
      poisson.setDepth(9); //default
       
      poisson.setInputCloud(cloud_smoothed_normals);
      PolygonMesh mesh;
      poisson.reconstruct(mesh);

      io::savePLYFile(argv[3], mesh);
       
       // viewer
       pcl::visualization::PCLVisualizer::Ptr viewer;
       //pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
       //viewer->setBackgroundColor (0, 0, 0);
       
       // initialize with point cloud
       viewer = simpleVis(cloud);
       
       // register events
       viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
       //viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);
       
       if(normals){
           viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
       }
       if(recon){
           viewer->addPolygonMesh(mesh, "mesh");
       }
       /*** TODO: interactive??
       if(id == 0){
           std::cout << "point cloud" << std::endl;
           viewer = simpleVis(cloud);
       }else if(id == 1){
           std::cout << "normals" << std::endl;
           viewer = normalsVis(cloud, cloud_normals);
       }else if(id == 2){
           std::cout << "mesh" << std::endl;
           viewer = meshVis(cloud, mesh);
       }
       ***/
       while (!viewer->wasStopped ())
       {
           viewer->spinOnce (100);
           boost::this_thread::sleep (boost::posix_time::microseconds (100000));
       }
       
   }
  return (0);
}



int
view (int argc, char** argv) {
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);
    
    return 0;
}
