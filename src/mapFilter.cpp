/************************************************************************************************************************
Programm mapFilter:

- lädt eine vorher definierte Punktwolke
- führt einen PassThrough-Filter aus zur vertikalen Begrenzung der Punktwolke
- Punkte werden Mustern (Clustern) zugeordnet und mit RANSAC eine Ebenengleichung für jedes Muster berechnet
- Wenn Ebene annährend vertikal (c ~= 0) dann werden alle Punkte, die in unter einem Abstandsschwellwert liegen, auf diese Ebene projiziert
- Berechnung einer boudingbox für jeweiliges Muster
- Zwischen den Eckpunkten werden gleichmäßig Punkte gesetzt, sodass eine gleichmäßige Oberfläche (Polygon-Mesh( berechnet werden kann
- Abspeichern als STL

ToDo: Eckpunkte der boundingbox direkt zu einem Polygon verbinden ohne vorher Punktwolke zu berechnen (Dann wird Oberfläche vollständig dicht und Datei deutlich kleiner)
************************************************************************************************************************/

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
#include <pcl/kdtree/kdtree_flann.h> // für surface
#include <pcl/surface/gp3.h> // für surface
#include <pcl/io/vtk_io.h> // Surface/Mesh als VTK abspeichern
#include <pcl/io/vtk_lib_io.h> // Mesh als STL abspeichern
#include <pcl/common/pca.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPoints.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
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


using namespace Eigen;


void background_color (pcl::visualization::PCLVisualizer& viewer)
{
viewer.setBackgroundColor (1.0, 1.0, 1.0);
}



int main(int argc, char** argv)
{

// Punktwolke laden
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unfiltered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
if (pcl::io::loadPCDFile <pcl::PointXYZ> ("Arena_24_7_2014.pcd",*cloud_unfiltered) == -1)
{
std::cout<<"Cloud reading failed"<<std::endl;
return (-1);
}

pcl::PCDWriter writer;

//Filter für Höhen- und Tiefenbegrenzung:
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud(cloud_unfiltered);
pass.setFilterFieldName("z");
pass.setFilterLimits(-0.2,0.3);//Für Stativ ca. (-0.9,-0.2)
pass.filter(*cloud);

// Berechnen der Punktnormalen
pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
normal_estimator.setSearchMethod(tree);
normal_estimator.setInputCloud(cloud);
normal_estimator.setKSearch(100);// Ursprünglich:50
// http://pointclouds.org/documentation/tutorials/normal_estimation.php
normal_estimator.compute(*normals);

// Segmentierung mit dem RegionGrowing-Algorithmus (Setzen der Parameter)
pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
reg.setMinClusterSize (200);
//reg.setMaxClusterSize (10000);
reg.setSearchMethod (tree);
reg.setNumberOfNeighbours (50); // Ursprünglich 50 (Bei ca. 150 werden die Kanten schön glatt!)
// aber es wird nach zu vielen Nachbarn gecheckt-> Mehrere Banden fallen in ein Cluster
reg.setInputCloud (cloud);
//reg.setIndices (indices);
reg.setInputNormals (normals);
reg.setSmoothnessThreshold (4.0 / 180.0 * M_PI); // Ursprünglich: 7.0/180*M_PI
// je kleiner desto weniger punkte sind "smooth" genug ->Klares Abgrenzen der Cluster!
reg.setCurvatureThreshold (.3);//Ursprünglich:1.0 // je kleiner desto geringer darf sich das Cluster krümmen

// Anwendung des Cluster-Filters auf Input-Wolke "cloud"
std::vector <pcl::PointIndices> clusters;
reg.extract (clusters);

pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::ExtractIndices<pcl::PointXYZ> extract;
extract.setInputCloud(cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr planes_cloud (new pcl::PointCloud<pcl::PointXYZ>);

//vtkSmartPointer<vtkAppendPolyData> appendFilter =
//vtkSmartPointer<vtkAppendPolyData>::New();
//vtkSmartPointer<vtkPolyData> polyplane =
//vtkSmartPointer<vtkPolyData>::New();

int b=0;

// Schleife über alle detektierten Cluster
for(int a=0;a<clusters.size();a++)
{
if(clusters[a].indices.size() >100 )
{
pcl::IndicesPtr indices_ptr (new std::vector<int> (clusters[a].indices.size ()));
for (int i=0;i<indices_ptr->size();i++)
{
(*indices_ptr)[i]=clusters[a].indices[i]; // http://www.pcl-users.org/Removing-a-cluster-Problem-with-pointer-td4023699.html
} // Indizes des jeweiligen Clusters werden alle in indices_ptr gespeichert
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

// Punkte in Eigenraum transformieren, um bounding box zu berechnen (Quelle: pcl-users Forum (http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html)
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

// Punktwolke und bounding box im viewer anzeigen
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer);
viewer->addCoordinateSystem ();
viewer->addPointCloud(planes_projected);// Danach auskommentieren
viewer->addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);


// Ausgabe der Eckpunkte (kann gelöscht werden)
std::cout << " min.x= " << min_pt.x << " max.x= " << max_pt.x << " min.y= " << min_pt.y << " max.y= " << max_pt.y << " min.z= " << min_pt.z << " max.z= " << max_pt.z<< std::endl;
std::cout << "Punkte: " << min_pt.x <<";" << min_pt.y << ";" << min_pt.z <<std::endl;
std::cout << min_pt.x <<";" << min_pt.y << ";" << max_pt.z <<std::endl;
std::cout << min_pt.x <<";" << max_pt.y << ";" << min_pt.z <<std::endl;
std::cout << min_pt.x <<";" << max_pt.y << ";" << max_pt.z <<std::endl;
std::cout << max_pt.x <<";" << min_pt.y << ";" << min_pt.z <<std::endl;
std::cout << max_pt.x <<";" << min_pt.y << ";" << max_pt.z <<std::endl;
std::cout << max_pt.x <<";" << max_pt.y << ";" << min_pt.z <<std::endl;
std::cout << max_pt.x <<";" << max_pt.y << ";" << max_pt.z <<std::endl;

// Erzeugen einer Punktwolke mit Punkten, die zwischen den Extrema liegen (um anschließend eine dichte Oberfläche zu erzeugen)
pcl::PointCloud<pcl::PointXYZ>::Ptr Fillcloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointXYZ P1,P2,P3,P4;
P1.x=max_pt.x;P2.x=max_pt.x; P3.x=max_pt.x; P4.x=max_pt.x;
P1.y=min_pt.y;P2.y=min_pt.y; P3.y=max_pt.y; P4.y=max_pt.y;
P1.z=min_pt.z;P2.z=max_pt.z; P3.z=min_pt.z; P4.z=max_pt.z;

//P4.x=min_pt.x;P3.x=min_pt.x; P2.x=min_pt.x; P1.x=min_pt.x;
//P4.y=min_pt.y;P3.y=min_pt.y; P2.y=max_pt.y; P1.y=max_pt.y;
//P4.z=min_pt.z;P3.z=max_pt.z; P2.z=min_pt.z; P1.z=max_pt.z;

Fillcloud->push_back(P1);
Fillcloud->push_back(P2);
Fillcloud->push_back(P3);
Fillcloud->push_back(P4);
// Fillcloud->push_back (pcl::PointXYZ (min_pt.x, min_pt.y, min_pt.z)); // P1
// Fillcloud->push_back (pcl::PointXYZ (min_pt.x, min_pt.y, max_pt.z)); // P2
// Fillcloud->push_back (pcl::PointXYZ (min_pt.x, max_pt.y, min_pt.z)); // P3
// Fillcloud->push_back (pcl::PointXYZ (min_pt.x, max_pt.y, max_pt.z)); // P4 // Enweder alle x_max oder alle x_min nehmen WARUM?????
// Fillcloud->push_back (pcl::PointXYZ (max_pt.x, min_pt.y, min_pt.z));
// Fillcloud->push_back (pcl::PointXYZ (max_pt.x, min_pt.y, max_pt.z));
// Fillcloud->push_back (pcl::PointXYZ (max_pt.x, max_pt.y, min_pt.z));
// Fillcloud->push_back (pcl::PointXYZ (max_pt.x, max_pt.y, max_pt.z));
// Schleife, um BoundingBox-"Fläche" mit Punkten zu füllen (um ein Mesh erzeugen zu können


int AnzahlPunktehoch = 80;
int AnzahlPunktebreit = 400;
for( int ii = 0; ii < AnzahlPunktebreit+1; ii++ ){
    Fillcloud->push_back( pcl::PointXYZ (P1.x+((P2.x-P1.x)/AnzahlPunktebreit)*ii , P1.y+((P2.y-P1.y)/AnzahlPunktebreit)*ii , P1.z+((P2.z-P1.z)/AnzahlPunktebreit)*ii) );
    for( int jj = 0; jj < AnzahlPunktehoch+1; jj++ ){
        Fillcloud->push_back( pcl::PointXYZ (P1.x+((P2.x-P1.x)/AnzahlPunktebreit)*ii + (P3.x-P1.x)/AnzahlPunktehoch*jj , P1.y+((P2.y-P1.y)/AnzahlPunktebreit)*ii + (P3.y-P1.y)/AnzahlPunktehoch*jj , P1.z+((P2.z-P1.z)/AnzahlPunktebreit)*ii + (P3.z-P1.z)/AnzahlPunktehoch*jj));
    }
}

// Erzeugte Punktwolke, die die Extrema "verbindet" rücktransformieren
pcl::PointCloud<pcl::PointXYZ>::Ptr Fillcloud_transformiert (new pcl::PointCloud<pcl::PointXYZ>);
pcl::transformPointCloud(*Fillcloud,*Fillcloud_transformiert,tfinal,qfinal);

// Fillcloud wird zum Viewer hinzugefügt; "spin" auskommentieren wenn nicht jedes Fenster einzeln weggeklickt werden soll
//viewer->addPointCloud(Fillcloud_transformiert);
//viewer->spin();

//Alle Clusterebenen, die vertikal sind werden in planes_cloud gespeichert
*planes_cloud+=*Fillcloud_transformiert;

// Aus aktuellen Punkten vtkPlaneSource erzeugen
vtkPlaneSource *plane= vtkPlaneSource::New();
vtkPlaneSource *plane2= vtkPlaneSource::New();
plane->SetOrigin(min_pt.x,min_pt.y,min_pt.z);
plane->SetPoint1(min_pt.x,min_pt.y,max_pt.z);
plane->SetPoint2(min_pt.x,max_pt.y,min_pt.z);
plane->SetXResolution(10);
//plane->SetResolution(10,200); // Set the number of x-y subdivisions in the plane
plane->Update();
plane2->SetOrigin(min_pt.x,min_pt.y,max_pt.z);
plane2->SetPoint1(min_pt.x,max_pt.y,max_pt.z);
plane2->SetPoint2(min_pt.x,max_pt.y,min_pt.z);
//plane->SetResolution(1,1);
plane2->Update();
// appender->AddInputConnection(plane->GetOutputPort());
// appender->Update();
// actor->SetMapper(mapper);
// // Aktuelle Plane zu PlaneCollection hinzufügen
// PlaneCollection->AddItem(plane->GetOutputPort());
// PlaneCollection->AddItem(plane);
// mapper->SetInputConnection(plane->GetOutputPort());
// plane->Delete();
// mapper->SetInputConnection(plane->GetOutputPort());
// actor->SetMapper(mapper);
// renderer->AddActor(plane);
// vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
// polydata->SetVerts();
// polyplane->ShallowCopy(plane->GetOutput());
// appendFilter->AddInputConnection(polyplane->GetProducerPort());

//appendFilter->AddInput(plane->GetOutput());
//appendFilter->Update();

//appendFilter->AddInput(plane2->GetOutput());
//appendFilter->Update();
b=b+1;
}
}
}
// PlanesCollection als STL abspeichern
//vtkSmartPointer<vtkSTLWriter> schreiber = vtkSmartPointer<vtkSTLWriter>::New();
//schreiber->SetInput(appendFilter->GetOutput());
//schreiber->SetFileName("funktioniert nicht!");
//schreiber->Update();
//schreiber->Write();

// schreiber->SetInputConnection(plane->GetOutputPort());
// schreiber->SetFileName("");
// schreiber->SetInput(appender->GetOutput());
// //schreiber->SetFileTypeToASCII();
// schreiber->Write();

// Ausgabe
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

// planes_cloud zu einer Oberfläche konvertieren u. als stl oder vtk speichern!
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

// Oberfläche als STL-Modell abspeichern
pcl::io::savePolygonFileSTL("Arena_24_7_2014.stl", triangles);
//pcl::io::saveVTKFile("mesh.vtk", triangles);

// Farbliche Visualisierung der segmentierten Punktwolke
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
