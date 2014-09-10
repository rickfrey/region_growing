//#include "PointCloudMatcher.h"// ????

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>        // für surface
#include <pcl/surface/gp3.h>                // für surface
#include <pcl/io/vtk_io.h>                  // Surface/Mesh als VTK abspeichern
#include <pcl/io/vtk_lib_io.h>              // Mesh als STL abspeichern
#include <pcl/common/pca.h>

#include <vtkUnstructuredGrid.h>
#include <vtkPoints.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkQuad.h>
#include <vtkAssembly.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <vtkPlaneSource.h>
#include <vtkPlaneCollection.h>
#include <vtkSTLWriter.h>

#include <vtkPolyData.h>

#include <vtkAppendPolyData.h>

// NEU: VTK PLANES ERZEUGEN UND ALS STL SPEICHERN

using namespace Eigen;

void background_color (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 1.0, 1.0);
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unfiltered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile <pcl::PointXYZ> ("Arena_24_7_2014(2).pcd",*cloud_unfiltered) == -1)
    {
        std::cout<<"Cloud reading failed"<<std::endl;
        return (-1);
    }
    pcl::PCDWriter writer;

    //Filter für Höhen- und Tiefenbegrenzung:
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_unfiltered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.2,0.15);//Für Stativ ca. (-0.9,-0.2)
    pass.filter(*cloud);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(100);// Ursprünglich:50
    // http://pointclouds.org/documentation/tutorials/normal_estimation.php
    normal_estimator.compute(*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (200);
    //reg.setMaxClusterSize (10000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (50); // Ursprünglich 50 (Bei ca. 150 werden die Kanten schön glatt!)
    // aber es wird nach zu vielen Nachbarn gecheckt-> Mehrere Banden fallen in ein Cluster
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (4.0 / 180.0 * M_PI);    // Ursprünglich: 7.0/180*M_PI
    // je kleiner desto weniger punkte sind "smooth" genug ->Klares Abgrenzen der Cluster!
    reg.setCurvatureThreshold (.4);//Ursprünglich:1.0   // je kleiner desto geringer darf sich das Cluster krümmen

    // Anwendung des Cluster-Filters auf Input-Wolke "cloud"
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr planes_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    /*NUR FÜR STATIV!!!
    //Suchen des größten Clusters ->Bodenebene ->Berechnung der Ebene
    //Dieser Schritt ist nur bis zur Extraktion und Speicherung der Bodenebene geschrieben,
    // da durch Anpassung der Number of Neigbors alle vertikalen Ebenen bereits erkannt wurden
    int maxcluster=0;//Stelle des größten Clusters im Vektor "clusters"
    for (int c=0;c<clusters.size();c++)
    {
        if(clusters[c].indices.size()>clusters[maxcluster].indices.size())
        {
        maxcluster=c;
        }
    }
    std::cout<<"Größtes Cluster ist Cluster Nr. "<<maxcluster<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr boden_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IndicesPtr bodenindices_ptr (new std::vector<int> (clusters[maxcluster].indices.size()));
    for (int j=0;j<bodenindices_ptr->size();j++)
    {
        (*bodenindices_ptr)[j]=clusters[maxcluster].indices[j];
    }
    extract.setIndices(bodenindices_ptr);
    extract.setNegative(false);
    extract.filter(*boden_cloud);
    writer.write("Test_waegelchen_xtion_langeskabel.pcd",*boden_cloud,false);
*/
    //SCHLEIFE ÜBER ALLE CLUSTER:////////////////////////
    //(In Cluster_cloud ist immer nur ein Cluster!)//////

    // In vtkPlaneCollection sollen alle planes gespeichert werden
    //vtkSmartPointer<vtkPlaneCollection> PlaneCollection =
    //  vtkSmartPointer<vtkPlaneCollection>::New();
    //    vtkPlaneCollection *PlaneCollection=vtkPlaneCollection::New();
    //    vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();

    //    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    //    vtkSmartPointer<vtkAppendPolyData> appender = vtkSmartPointer<vtkAppendPolyData>::New();
    //    vtkSmartPointer<vtkRenderer> renderer;

    vtkSmartPointer<vtkAssembly> appendFilter =
            vtkSmartPointer<vtkAssembly>::New();

    vtkSmartPointer<vtkPolyData> polyplane =
            vtkSmartPointer<vtkPolyData>::New();

    vtkSmartPointer<vtkPolyData> plane1 = vtkSmartPointer<vtkPolyData>::New();

    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    int b=0;
    for(int a=0;a<clusters.size();a++)
    {
        if(clusters[a].indices.size() >50 )
        {
            pcl::IndicesPtr indices_ptr (new std::vector<int> (clusters[a].indices.size ()));
            for (int i=0;i<indices_ptr->size();i++)
            {
                (*indices_ptr)[i]=clusters[a].indices[i];   // http://www.pcl-users.org/Removing-a-cluster-Problem-with-pointer-td4023699.html
            }                                               // Indizes des jeweiligen Clusters werden alle in indices_ptr gespeichert

            // Punkte des Clusters werden in cluster_cloud geschrieben
            extract.setIndices(indices_ptr);
            extract.setNegative(false);
            extract.filter(*cluster_cloud);// Punkte des Cluster a werden in cluster_cloud geschrieben
            std::cout<<"cluster_cloud["<<a<<"] hat "<<cluster_cloud->width*cluster_cloud->height<<" Punkte."<<std::endl;

            //Erzeugen einer Ebene aus Cluster_cloud
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.1);
            seg.setInputCloud(cluster_cloud);
            seg.segment(*inliers, *coefficients);

            // Wenn Ebene vertikal: Abspeichern in Cluster_i.pcd
            if(coefficients->values[2]<.9 && coefficients->values[2]>(-.9))//ax+by+cz+d=0 (wenn c=0 => Ebene parallel zur z-Achse)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr planes_projected (new pcl::PointCloud<pcl::PointXYZ>);
                //Inliers auf Ebene projizieren:
                pcl::PCA<pcl::PointXYZ> pca2;
                pcl::ProjectInliers<pcl::PointXYZ> proj2;
                proj2.setModelType(pcl::SACMODEL_PLANE);
                proj2.setIndices(inliers);
                proj2.setInputCloud(cluster_cloud);
                proj2.setModelCoefficients(coefficients);
                proj2.filter(*planes_projected);

                // PCL ONLINE TUTORIAL
                // http://pointclouds.org/documentation/tutorials/moment_of_inertia.php
                /*                  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
                  feature_extractor.setInputCloud (planes_projected);
                  feature_extractor.compute ();

                  std::vector <float> moment_of_inertia;
                  std::vector <float> eccentricity;
                  pcl::PointXYZ min_point_AABB;
                  pcl::PointXYZ max_point_AABB;
                  pcl::PointXYZ min_point_OBB;
                  pcl::PointXYZ max_point_OBB;
                  pcl::PointXYZ position_OBB;
                  Matrix3f rotational_matrix_OBB;
                  float major_value, middle_value, minor_value;
                  Vector3f major_vector, middle_vector, minor_vector;
                  Vector3f mass_center;

                  feature_extractor.getMomentOfInertia (moment_of_inertia);
                  feature_extractor.getEccentricity (eccentricity);
                  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
                  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
                  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
                  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
                  feature_extractor.getMassCenter (mass_center);

                  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                  viewer->setBackgroundColor (0, 0, 0);
                  viewer->addCoordinateSystem (1.0);
                  viewer->initCameraParameters ();
                  viewer->addPointCloud<pcl::PointXYZ> (planes_projected, "sample cloud");
                  viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

                  Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
                  Quaternionf quat (rotational_matrix_OBB);
                  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

                  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
                  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
                  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
                  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
                  viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
                  viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
                  viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
*/

                /*
                // pcl-users Forum (http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html)
                // Finding oriented bounding box of a cloud
                //compute principal direction
                Vector4f centroid;
                pcl::compute3DCentroid(*planes_projected,centroid);
                Matrix3f covariance;
                pcl::computeCovarianceMatrixNormalized(*planes_projected,centroid,covariance);
                SelfAdjointEigenSolver<Matrix3f> eigen_solver(covariance,ComputeEigenvectors);
                Matrix3f eigDx = eigen_solver.eigenvectors();
                eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

                //move the points to the referance frame
                Matrix4f p2w (Matrix4f::Identity());
                p2w.block<3,3>(0,0) = eigDx.transpose();
                p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
                pcl::PointCloud<pcl::PointXYZRGB> cPoints;
                pcl::transformPointCloud(*planes_projected,cPoints,p2w);

                pcl::PointXYZRGB min_pt,max_pt;
                pcl::getMinMax3D(cPoints,min_pt,max_pt);
                const Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap()+min_pt.getVector3fMap());

                // final transform
                const Quaternionf qfinal(eigDx);
                const Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

                // draw the cloud and the box
                pcl::visualization::PCLVisualizer viewer;
                viewer.addPointCloud(planes_projected);
                viewer.addCube(tfinal,qfinal,max_pt.x-min_pt.x,max_pt.y-min_pt.y,max_pt.z-min_pt.z);
                viewer.spin();
*/
                /*
                                        // BOUNDINGBOX!!!
//                                        //calc boundingbox
//                                        pcl::PCA<pcl::PointXYZ> pca;        // Principal Component Analysis Class
//                                        pcl::PointCloud<pcl::PointXYZ> proj;

//                                        pca.setInputCloud (planes_projected);
//                                        pca.project (*planes_projected, proj);

//                                        pcl::PointXYZ proj_min;
//                                        pcl::PointXYZ proj_max;
//                                        pcl::getMinMax3D (proj, proj_min, proj_max);

//                                        pcl::PointXYZ min;
//                                        pcl::PointXYZ max;
//                                        pca.reconstruct (proj_min, min);
//                                        pca.reconstruct (proj_max, max);
//                                        std::cout << " min.x= " << min.x << " max.x= " << max.x << " min.y= " <<
//                                                     min.y << " max.y= " << max.y << " min.z= " << min.z << " max.z= " << max.z
//                                                  << std::endl;

//                                        //Rotation of PCA
//                                        Matrix3f rot_mat = pca.getEigenVectors (); // ursprünglich: Eigen::Matrix3f

//                                        //translation of PCA
//                                        Vector3f cl_translation = pca.getMean().head(3); // ursprünglich: Eigen::Vector3f

//                                        Matrix4f affine_trans; // ursprünglich: Eigen::Matrix3f
//                                        std::cout << "blub" << rot_mat << std::endl;


//                                        //Reordering of principal components

//                                        // HIER LIEGT DAS PROBLEM!!!!!
//                                        affine_trans.col(0) << (rot_mat.col(0).cross(rot_mat.col(1))).normalized();
//                                        affine_trans.col(1) << rot_mat.col(0); // ursprünglich 0
//                                        affine_trans.col(2) << rot_mat.col(1); // ursprünglich 1
//                                        affine_trans.col(3) << cl_translation,1;

//                                        std::cout << affine_trans << std::endl;

//                                        Eigen::Quaternionf rotation = Eigen::Quaternionf (affine_trans);
//                                        Eigen::Vector4f t = pca.getMean();

//                                        Eigen::Vector3f translation = Eigen::Vector3f (t.x(), t.y(), t.z());

//                                        double width = fabs(proj_max.x-proj_min.x);
//                                        double height = fabs(proj_max.y-proj_min.y);
//                                        double depth = fabs(proj_max.z-proj_min.z);

//                                        //adding the bounding box to a viewer :
//                                        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
//                                        pcl::visualization::PCLVisualizer ("3D Viewer"));
//                                        viewer->setBackgroundColor (0, 0, 0);
//                                        viewer->addPointCloud<pcl::PointXYZ> (planes_projected, "NAO arm cloud");
//                                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "NAO arm cloud");

//                                        viewer->addCoordinateSystem ();
//                                        viewer->initCameraParameters ();
//                                        //viewer->addCube (min.x, max.x, min.y, max.y, min.z, max.z);
//                                        viewer->addCube (translation, rotation, width, height, depth);
//                                        while (!viewer->wasStopped ())
//                                        {
//                                            viewer->spinOnce (100);
//                                            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//                                        }
//                                        //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
*/

                // pcl-users Forum (http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html)
                // compute principal direction
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*planes_projected, centroid);
                Eigen::Matrix3f covariance;
                computeCovarianceMatrixNormalized(*planes_projected, centroid, covariance);
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
                Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
                eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

                // move the points to the that reference frame
                Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
                p2w.block<3,3>(0,0) = eigDx.transpose();
                p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
                pcl::PointCloud<pcl::PointXYZ> cPoints;
                pcl::transformPointCloud(*planes_projected, cPoints, p2w);

                pcl::PointXYZ min_pt, max_pt;
                pcl::getMinMax3D(cPoints, min_pt, max_pt);
                const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

                // final transform
                const Eigen::Quaternionf qfinal(eigDx);
                const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

                // draw the cloud and the box
                //pcl::visualization::PCLVisualizer viewer;
                boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer);
                //                viewer->addCoordinateSystem ();
                //viewer->addPointCloud(planes_projected);
                //                viewer->addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);
                std::cout << " min.x= " << min_pt.x << " max.x= " << max_pt.x << " min.y= " <<
                             min_pt.y << " max.y= " << max_pt.y << " min.z= " << min_pt.z << " max.z= " << max_pt.z<< std::endl;
                std::cout << "Punkte: " << min_pt.x <<";" << min_pt.y << ";" << min_pt.z <<std::endl;
                std::cout << min_pt.x <<";" << min_pt.y << ";" << max_pt.z <<std::endl;
                std::cout << min_pt.x <<";" << max_pt.y << ";" << min_pt.z <<std::endl;
                std::cout << min_pt.x <<";" << max_pt.y << ";" << max_pt.z <<std::endl;
                std::cout << max_pt.x <<";" << min_pt.y << ";" << min_pt.z <<std::endl;
                std::cout << max_pt.x <<";" << min_pt.y << ";" << max_pt.z <<std::endl;
                std::cout << max_pt.x <<";" << max_pt.y << ";" << min_pt.z <<std::endl;
                std::cout << max_pt.x <<";" << max_pt.y << ";" << max_pt.z <<std::endl;

                pcl::PointCloud<pcl::PointXYZ>::Ptr Test (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointXYZ P1,P2,P3,P4;
                P1.x=min_pt.x;P2.x=min_pt.x; P3.x=min_pt.x; P4.x=min_pt.x;
                P1.y=min_pt.y;P2.y=min_pt.y; P3.y=max_pt.y; P4.y=max_pt.y;
                P1.z=min_pt.z;P2.z=max_pt.z; P3.z=min_pt.z; P4.z=max_pt.z;


                float a=min_pt.x, b=min_pt.y, c=min_pt.z, d=min_pt.x, e=min_pt.y, f=max_pt.z, g=min_pt.x, h=max_pt.y, i=min_pt.z, j=min_pt.x, k=max_pt.y, l=max_pt.z;

                Test->push_back(P1);
                Test->push_back(P2);
                Test->push_back(P3);
                Test->push_back(P4);

                //                 Test->push_back (pcl::PointXYZ (min_pt.x, min_pt.y, min_pt.z)); // P1
                //                 Test->push_back (pcl::PointXYZ (min_pt.x, min_pt.y, max_pt.z)); // P2
                //                 Test->push_back (pcl::PointXYZ (min_pt.x, max_pt.y, min_pt.z)); // P3
                //                 Test->push_back (pcl::PointXYZ (min_pt.x, max_pt.y, max_pt.z)); // P4   // Enweder alle x_max oder alle x_min nehmen WARUM?????
                //                 Test->push_back (pcl::PointXYZ (max_pt.x, min_pt.y, min_pt.z));
                //                 Test->push_back (pcl::PointXYZ (max_pt.x, min_pt.y, max_pt.z));
                //                 Test->push_back (pcl::PointXYZ (max_pt.x, max_pt.y, min_pt.z));
                //                 Test->push_back (pcl::PointXYZ (max_pt.x, max_pt.y, max_pt.z));

                // Schleife, um BoundingBox-"Fläche" mit Punkten zu füllen (um ein Mesh erzeugen zu können
                int AnzahlPunktehoch = 80;
                int AnzahlPunktebreit = 400;
                for(int ii=0; ii<AnzahlPunktebreit+1; ii++){
                    Test->push_back(pcl::PointXYZ (P1.x+((P2.x-P1.x)/AnzahlPunktebreit)*ii,P1.y+((P2.y-P1.y)/AnzahlPunktebreit)*ii,P1.z+((P2.z-P1.z)/AnzahlPunktebreit)*ii));
                    for(int jj=0; jj<AnzahlPunktehoch+1; jj++){
                        Test->push_back(pcl::PointXYZ (P1.x+((P2.x-P1.x)/AnzahlPunktebreit)*ii + (P3.x-P1.x)/AnzahlPunktehoch*jj,P1.y+((P2.y-P1.y)/AnzahlPunktebreit)*ii + (P3.y-P1.y)/AnzahlPunktehoch*jj,P1.z+((P2.z-P1.z)/AnzahlPunktebreit)*ii + (P3.z-P1.z)/AnzahlPunktehoch*jj));
                    }
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr Test_transformiert (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::transformPointCloud(*Test,*Test_transformiert,tfinal,qfinal);
                //viewer->addPointCloud(Test_transformiert);
                //viewer->spin();

                *planes_cloud+=*Test_transformiert;//Alle Clusterebenen, die vertikal sind werden in planes_cloud (JETZT: Test) gespeichert
                //std::stringstream ss;
                //ss<<"Cluster_"<<a<<".pcd";
                //writer.write<pcl::PointXYZ>(ss.str(),*cluster_cloud,false);
                //std::cout<<"Cluster "<<a<<" wurde gespeichert!"<<std::endl;


                double p0[3] = {a, b, c};
                double p1[3] = {d,e,f};
                double p2[3] = {g, h, (double)max_pt.z};
                double p3[3] = {(double)min_pt.x, (double)max_pt.y, (double)min_pt.z};


                vtkSmartPointer<vtkPoints> points =
                        vtkSmartPointer<vtkPoints>::New();
                points->InsertNextPoint(p0);
                points->InsertNextPoint(p1);
                points->InsertNextPoint(p2);
                points->InsertNextPoint(p3);

                // Create a quad on the four points
                vtkSmartPointer<vtkQuad> quad =
                        vtkSmartPointer<vtkQuad>::New();
                quad->GetPointIds()->SetId(0,0);
                quad->GetPointIds()->SetId(1,1);
                quad->GetPointIds()->SetId(2,2);
                quad->GetPointIds()->SetId(3,3);

                // Create a cell array to store the quad in
                vtkSmartPointer<vtkCellArray> quads =
                        vtkSmartPointer<vtkCellArray>::New();
                quads->InsertNextCell(quad);

                // Create a polydata to store everything in
                vtkSmartPointer<vtkPolyData> polydata =
                        vtkSmartPointer<vtkPolyData>::New();

                // Add the points and quads to the dataset
                polydata->SetPoints(points);
                polydata->SetPolys(quads);

                // Setup actor and mapper
                vtkSmartPointer<vtkPolyDataMapper> mapper =
                        vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
                mapper->SetInput(polydata);
#else
                mapper->SetInputData(polydata);
#endif

                vtkSmartPointer<vtkActor> actor =
                        vtkSmartPointer<vtkActor>::New();
                actor->SetMapper(mapper);

                // Setup render window, renderer, and interactor

                renderer->AddActor(actor);



                // Add the points to a vtkPoints object




                b=b+1;
            }
        }
    }

    renderWindow->Render();
    renderWindowInteractor->Start();

    // PlanesCollection als STL abspeichern
    //    vtkSmartPointer<vtkSTLWriter> schreiber = vtkSmartPointer<vtkSTLWriter>::New();
    //    schreiber->SetInput(appendFilter);
    //    schreiber->SetFileName("stl_plane_writer_test");
    //    schreiber->Update();
    //    schreiber->Write();
    //    schreiber->SetInputConnection(plane->GetOutputPort());
    //    schreiber->SetFileName("stl_plane_writer_test");
    //    schreiber->SetInput(appender->GetOutput());
    //    //schreiber->SetFileTypeToASCII();
    //    schreiber->Write();

    std::cout<<"Es wurden "<<b<<" Flächen in Planes_cloud.pcd geschrieben"<<endl;
    writer.write("Planes_cloud.pcd",*planes_cloud,false);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    std::cout << "These are the indices of the points of the initial" <<
                 std::endl << "cloud that belong to the first cluster:" << std::endl;
    int counter = 0;
    while (counter < 5 || counter > clusters[0].indices.size ())
    {
        std::cout << clusters[0].indices[counter] << std::endl;
        counter++;
    }

    // planes_cloud zu Surface konvertieren u. als stl oder vtk speichern!
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals_triangles (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_triangles (new pcl::search::KdTree<pcl::PointXYZ>);
    tree_triangles->setInputCloud(planes_cloud);
    n.setInputCloud(planes_cloud);
    n.setSearchMethod(tree_triangles);
    n.setKSearch(20);
    n.compute(*normals_triangles);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*planes_cloud,*normals_triangles,*cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius(0.4);// Ursprünglich 0.025
    gp3.setMu(5.0); // Ursprünglich 2.5
    gp3.setMaximumNearestNeighbors(20); // davor 100
    gp3.setMaximumSurfaceAngle(M_PI/4); // ursprünglich: M_PI/4
    gp3.setMinimumAngle(M_PI/18);
    gp3.setMaximumAngle(2*M_PI/1); // ursprunglich: 2*M_PI/3
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    pcl::io::savePolygonFileSTL("mesh.stl", triangles);
    //pcl::io::saveVTKFile("mesh.vtk", triangles);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    viewer.runOnVisualizationThreadOnce(background_color);
    while (!viewer.wasStopped ())
    {
    }

    /*
    pcl::PointCloud <pcl::PointXYZ>::Ptr cluster_cloud=clusters[0].indices;
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(cluster_cloud);
    while (!viewer.wasStopped())
    {
    }
*/
    return (0);

}
