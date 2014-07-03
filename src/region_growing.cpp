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

#include <vtkUnstructuredGrid.h>
#include <vtkPoints.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>


int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unfiltered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile <pcl::PointXYZ> ("mehr_abstand_perfekt.pcd",*cloud_unfiltered) == -1)
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
    int b=0;
    for(int a=0;a<clusters.size();a++)
    {
        if(clusters[a].indices.size() >50 )
        {
            pcl::IndicesPtr indices_ptr (new std::vector<int> (clusters[a].indices.size ()));
            for (int i=0;i<indices_ptr->size();i++)
            {
                (*indices_ptr)[i]=clusters[a].indices[i];//http://www.pcl-users.org/Removing-a-cluster-Problem-with-pointer-td4023699.html
            }

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
                pcl::ProjectInliers<pcl::PointXYZ> proj;
                proj.setModelType(pcl::SACMODEL_PLANE);
                proj.setIndices(inliers);
                proj.setInputCloud(cluster_cloud);
                proj.setModelCoefficients(coefficients);
                proj.filter(*planes_projected);

                //Convert to vtkPointSet to get BoundingBox
                vtkSmartPointer< vtkUnstructuredGrid > vtkPs = vtkSmartPointer<vtkUnstructuredGrid >::New();
                vtkSmartPointer< vtkPoints > vtkPts = vtkSmartPointer<vtkPoints>::New();

                for(int z = 0; z< planes_projected->size(); ++z)
                    vtkPts->InsertNextPoint(planes_projected->points[z].x, planes_projected->points[z].y, planes_projected->points[z].z);

                vtkPs->SetPoints(vtkPts);

                //Debug **********************************************************************************************************

                vtkPs->ComputeBounds();
                double* bb = vtkPs->GetBounds();

                pcl::PointCloud <pcl::PointXYZRGB>::Ptr boundingBox(new pcl::PointCloud<pcl::PointXYZRGB> );
                pcl::PointXYZRGB pt(255,0,0);
                pt.x= bb[0];
                pt.y= bb[1];
                pt.z= bb[2];
                boundingBox->push_back(pt);
                pt.x= bb[3];
                pt.y= bb[1];
                pt.z= bb[2];
                boundingBox->push_back(pt);
                pt.x= bb[0];
                pt.y= bb[1];
                pt.z= bb[5];
                boundingBox->push_back(pt);
                pt.x= bb[3];
                pt.y= bb[1];
                pt.z= bb[5];
                boundingBox->push_back(pt);
                pt.x= bb[0];
                pt.y= bb[4];
                pt.z= bb[2];
                boundingBox->push_back(pt);
                pt.x= bb[3];
                pt.y= bb[4];
                pt.z= bb[2];
                boundingBox->push_back(pt);
                pt.x= bb[0];
                pt.y= bb[4];
                pt.z= bb[5];
                boundingBox->push_back(pt);
                pt.x= bb[3];
                pt.y= bb[4];
                pt.z= bb[5];
                boundingBox->push_back(pt);



                pcl::visualization::CloudViewer viewer ("Bounding Box viewer");

                viewer.showCloud(planes_projected);
                viewer.showCloud(boundingBox);
                while (!viewer.wasStopped ())
                {
                }
                //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

                *planes_cloud+=*planes_projected;//Alle Clusterebenen, die vertikal sind werden in planes_cloud gespeichert
                //std::stringstream ss;
                //ss<<"Cluster_"<<a<<".pcd";
                //writer.write<pcl::PointXYZ>(ss.str(),*cluster_cloud,false);
                //std::cout<<"Cluster "<<a<<" wurde gespeichert!"<<std::endl;
                b=b+1;
            }
        }
    }
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

    //TEST: planes_cloud zu Surface konvertieren u. als stl oder vtk speichern!
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
    gp3.setSearchRadius(0.5);// Ursprünglich 0.025
    gp3.setMu(25.0); // Ursprünglich 2.5
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI/4);
    gp3.setMinimumAngle(M_PI/18);
    gp3.setMaximumAngle(2*M_PI/3);
    gp3.setNormalConsistency(false);


    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    pcl::io::savePolygonFileSTL("mesh.stl", triangles);
    //pcl::io::saveVTKFile("mesh.vtk", triangles);

    //hier weitermachen!!!


    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
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
