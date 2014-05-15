#include <ros/ros.h>
// PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/time.h>

#include <stdlib.h>

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_vector;

// PCL Viewer
bool showUI = true;
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer 2"));

//Function declarations
pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_object_from_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,pcl::PointIndices object_indices);


void printToPCLViewer(){
    pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,"source cloud");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");

    for(int i=0; i < object_vector.size(); i++){
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> randColor;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = object_vector[i];
        std::string pc_name = "object_" + itoa(i);
        pclViewer->addPointCloud<pcl::PointXYZRGB>(pc,randColor,pc_name);
        pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, pc_name);
    }


//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(segmented_cloud, 255, 0, 0);
//    pclViewer->addPointCloud<pcl::PointXYZRGB>(segmented_cloud,red_color,"segmented cloud");
//    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segmented cloud");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color(objects_cloud, 0, 0, 255);
//    pclViewer->addPointCloud<pcl::PointXYZRGB>(objects_cloud,blue_color,"objects cloud");
//    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "objects cloud");
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> euclidean_object_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,std::vector<pcl::PointIndices> &cluster_indices){
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (500);
    ec.setMaxClusterSize (15000);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_input);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_input);
    ec.extract (cluster_indices);

    // returns a vector of all the objects
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_vector_temp;
    for (int i=0; i<cluster_indices.size(); i++){
        pcl::PointIndices cloud_indices = cluster_indices.at(i);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_cluster = extract_object_from_indices(cloud_input,cloud_indices);
        object_vector_temp.push_back(cloud_cluster);
    }

    return object_vector_temp;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_object_from_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,pcl::PointIndices object_indices){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int j=0; j<object_indices.indices.size(); j++){
        cloud_cluster->points.push_back (cloud_input->points[object_indices.indices[j]]);
    }
    return cloud_cluster;
}


// Callback Function for the subscribed ROS topic
void cloud_callback (const pcl::PCLPointCloud2ConstPtr& input){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*input,*objects);
    std::vector<pcl::PointIndices> cluster_indices;
    cloud = objects;

    //Extract objects
    if(!object_vector.empty()){object_vector.clear();}
    object_vector = euclidean_object_segmentation(objects,cluster_indices);

    //Update viewer
    printToPCLViewer();
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "object_extraction");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = n.subscribe ("/custom/not_planes", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    //pub = n.advertise<sensor_msgs::PointCloud2> ("/extracted_planes", 1);


    // Load parameters from launch file
   // nh.param("pcl_visualizer",showUI,true);
    showUI = true;

    //PCL Viewer
    bool loop_condition = true;
    if(showUI){
        pclViewer->setBackgroundColor (0, 0, 0);
        pclViewer->initCameraParameters ();
        pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
        vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
        renderWindow->SetSize(800,450);
        renderWindow->Render();
    }
    else{ // Not very clean, but only solution I found to not show pclViewer
        pclViewer->setBackgroundColor (0, 0, 0);
        pclViewer->initCameraParameters ();
        vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
        renderWindow->SetSize(1,1);
        renderWindow->Render();
    }

    ros::Rate r(30);
    while (loop_condition) {
        ros::spinOnce();
        if(showUI){
            pclViewer->spinOnce (100);
            loop_condition = ros::ok() && !pclViewer->wasStopped();
        }
        else{
            loop_condition = ros::ok();
        }
        r.sleep();
    }
    return 0;
}
