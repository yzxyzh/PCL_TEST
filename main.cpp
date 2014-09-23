#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/pca.h>
#include <vtkSmartPointer.h>
#include <vtkDataArray.h>

#include "UserColor.h"
#include <time.h>
// This function displays the help
void
showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

void IsInEllipse(const pcl::PointCloud<pcl::PointXYZ>& inPointCloud,
                 const double a, const double b, const double c,
                 const Eigen::Matrix3f& coorT,
                 const Eigen::Vector3f& translate,
                 vector<bool>& outResult)
{
    int cloudSize=inPointCloud.size();
    outResult.resize(cloudSize, true);

    //要建立椭球判断矩阵A
    //椭球可以写成$xAx^t = 1$,若坐标变换后，假设坐标变换矩阵为$Q,Q^tQ=I$,那么
    //椭球变为$xBx^t=1,B=QAQ^t$,若加上平移，椭球变为：
    //$(x-v)B(x-v)^t = 1$，椭球内部就是$ (x-v)B(x-v)^t \leq 1$
    Eigen::Matrix3f Gamma=Eigen::Matrix3f::Zero();
    Gamma(0,0)=1.0/(a*a);
    Gamma(1,1)=1.0/(b*b);
    Gamma(2,2)=1.0/(c*c);

    Eigen::Matrix3f A=coorT*Gamma*coorT.transpose();

    for (int i=0; i<cloudSize; i++) {
        
        pcl::PointXYZ p=inPointCloud[i];
        Eigen::Vector3f vec(p.x-translate[0],p.y-translate[1],p.z-translate[2]);
        double val = (vec.transpose()*A*vec)(0,0);
        if(val>1) outResult[i]=false;
    }
}


/**
 *  绘制一个斜椭球
 *
 *  @param a                         a轴长度
 *  @param b                         b轴长度
 *  @param c                         c轴长度
 *  @param outCloud                  输出的椭球点云
 *  @param coorT                     坐标变换矩阵（一定要$A^T \cdot A=I$)
 *  @param translate                 平移量
 */
void EclipseGenerator(double a, double b, double c, pcl::PointCloud<pcl::PointXYZ>& outCloud,
                      const Eigen::Matrix3f& coorT=Eigen::Matrix3f::Identity(),
                      const Eigen::Vector3f& translate=Eigen::Vector3f::Zero())
{
    //生成一个椭圆
    /**
     x = a*cos(\theta)*cos(\phi)
     y = b*cos(\theta)*sin(\phi)
     z = c*sin(\theta)
     */
    const double PI = 3.1415926535897;
    
    
    
    double theta_max = PI*2;
    double phi_max=PI/2;
    
    double iterval=PI/60;
    
    outCloud.clear();
    
    Eigen::Vector3f coor;
    
    for (double phi = -PI/2;phi<=phi_max;phi+=iterval) {
        for (double theta = 0.0; theta<=theta_max; theta+=iterval) {
            
            double e_x = a* cos(theta) * cos(phi);
            double e_y = b* cos(theta) * sin(phi);
            double e_z = c* sin(theta);
            Eigen::Vector3f coor(e_x,e_y,e_z);
            Eigen::Vector3f res=coorT*coor+translate;
            
            
            pcl::PointXYZ p(res[0],res[1],res[2]);
            outCloud.push_back(p);
            
        }
    }
    
    cout<<"out cloud size = "<<outCloud.size()<<endl;
    
    
}

// This is the main function
int
main (int argc, char** argv)
{
    srand(time(NULL));
    
    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    std::vector<int> filenames;
    
    // Load file | Works with PCD and PLY files
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    
    if (pcl::io::loadPLYFile ("manhead-2.ply", *source_cloud) < 0)  {
        return -1;
    }
    
//    EclipseGenerator(1.0, 0.5, 0.5, *source_cloud);
    
    cout<<"source cloud points: "<<source_cloud->size()<<endl;
    
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(source_cloud);
    
    Eigen::Matrix3f e_vector=pca.getEigenVectors();
    Eigen::Vector3f e_value=pca.getEigenValues();
    Eigen::Vector4f mean=pca.getMean();
    Eigen::Vector3f mean_3f(mean[0],mean[1],mean[2]);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr showCloud (new pcl::PointCloud<pcl::PointXYZ> ());
    
    EclipseGenerator(1.0, 0.5, 0.5, *showCloud, e_vector, mean_3f);
    
    
    cout<<"the orthogonity of engen vectors: "<<e_vector*e_vector.transpose()<<endl;
    
    
    Eigen::Vector3f x_axis=e_vector.col(0);
    Eigen::Vector3f y_axis=e_vector.col(1);
    Eigen::Vector3f z_axis=e_vector.col(2);

    pcl::PointXYZ origin(mean[0],mean[1],mean[2]);
    pcl::PointXYZ x(x_axis[0],x_axis[1],x_axis[2]);
    pcl::PointXYZ y(y_axis[0],y_axis[1],y_axis[2]);
    pcl::PointXYZ z(z_axis[0],z_axis[1],z_axis[2]);
    
    
    cout<<"Eigen values : "<<e_value<<" Eigen vectors : "<<e_vector<<endl;
    cout<<" Means = "<<mean<<endl;
    
    // Visualization
    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
           "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
    
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> show_e_handler (showCloud, 255, 0, 0);
    
    user_color<pcl::PointXYZ> userColor(source_cloud);
    int showSize=source_cloud->size();
    vector<rgb_s> colors(showSize);
    vector<bool> res;
    IsInEllipse(*source_cloud, 1.0, 0.5, 0.5, e_vector, mean_3f, res);

    for (int i=0; i<res.size(); i++) {
        if(res[i])
        {
            colors[i].r=255;
            colors[i].g=255;
            colors[i].b=255;
        }else{
            colors[i].r=0;
            colors[i].g=0;
            colors[i].b=255;
        }
    }
    
    userColor.SetUserColorField(colors);


    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (source_cloud, userColor, "original_cloud");
    viewer.addPointCloud (showCloud, show_e_handler, "e_cloud");
    
    //viewer.addCoordinateSystem (1.0,0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.addLine(origin, x, 255, 0 , 0, "line_x");
    viewer.addLine(origin, y, 0, 255 , 0, "line_y");
    viewer.addLine(origin, z, 0, 0 , 255, "line_z");
    //viewer.setPosition(800, 400); // Setting visualiser window position
    
    vtkSmartPointer<vtkDataArray> colorArray;
    source_cloud_color_handler.getColor(colorArray);
    
    
    
    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
    
    return 0;
}

//#include <iostream>
//#include <pcl/common/projection_matrix.h>
//#include <pcl/common/common.h>
//#include <time.h>

//using namespace pcl;
//using namespace std;
//int main()
//{
//    srand(time(NULL));
//    PointCloud<PointXYZ> pCloud;
//
//    for (int i=0; i<1000; i++) {
//        PointXYZ p;
//        p.x=rand()%100;
//        p.y=rand()%100;
//        p.z=rand()%100;
//        pCloud.push_back(p);
//    }
//
//    cout<<pCloud.size()<<endl;
//
//
//
//
//    return EXIT_SUCCESS;
//}