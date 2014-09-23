#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/visualization/common/common.h>
// VTK includes
#include <vtkSmartPointer.h>
#include <vtkDataArray.h>
#include <vtkFloatArray.h>
#include <vtkUnsignedCharArray.h>
#include <vector>
#include <iostream>
using namespace pcl;
using namespace visualization;
using namespace std;
/**
 *  这个类是一个rgb颜色的结构类
 */
typedef struct rgb_s{
    unsigned char r;
    unsigned char g;
    unsigned char b;
    
} rgb_s;

//typedef double rgbColor[3];
/**
 *  这个类集成了pcl::PointCloudColorHandler，但是可以设定所有点的任意颜色
 *  方法是给这个类传入一个颜色场vector<rgb_s>，颜色场的大小应该与点云大小一致
 */
template<typename PointT>
class user_color : public pcl::visualization::PointCloudColorHandler<PointT>
{
    typedef typename user_color<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    
public:
    typedef boost::shared_ptr<user_color<PointT> > Ptr;
    typedef boost::shared_ptr<const user_color<PointT> > ConstPtr;
    
    
    user_color (const PointCloudConstPtr &cloud) :
    PointCloudColorHandler<PointT> (cloud)
    {
        this->capable_ = true;
    }
    /**
     *  设定逐点的颜色场
     *
     *  @param inColorField 进入的颜色场
     */
    void SetUserColorField(const vector<rgb_s>& inColorField);
    
    virtual std::string
    getName () const { return ("PointCloudColorHandlerUser"); }
    
    /** \brief Get the name of the field used. */
    virtual std::string
    getFieldName () const { return ("[user]"); }
    
    virtual bool
    getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

protected:
    
    vector<rgb_s> Color;
};

//#include "UserColor.h"

template <typename PointT> bool
user_color<PointT>::getColor (vtkSmartPointer<vtkDataArray> &scalars) const
{
    if (!this->capable_ || !this->cloud_)
        return (false);
    
    if (!scalars) scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    
    scalars->SetNumberOfComponents (3);
    
    vtkIdType nr_points = this->cloud_->points.size ();
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

    unsigned char* colors = new unsigned char[nr_points * 3];
    int nCounter=0;
    for (vtkIdType cp = 0; cp < nr_points; ++cp,++nCounter)
    {
        colors[cp * 3 + 0] = Color[nCounter].r;
        colors[cp * 3 + 1] = Color[nCounter].g;
        colors[cp * 3 + 2] = Color[nCounter].b;
    }
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3 * nr_points, 0);
    return (true);
}
template <typename PointT>
void user_color<PointT>::SetUserColorField(const vector<rgb_s>& inColorField)
{
    if(inColorField.size() != this->cloud_->points.size())
    {
        cout<<"进入色彩个数与点云个数不符"<<endl;
        return;
    }
    
    int ColorSize=inColorField.size();
    Color.resize(ColorSize);
    
    for (int i=0; i<ColorSize; i++)
    {
        Color[i].r=inColorField[i].r;
        Color[i].g=inColorField[i].g;
        Color[i].b=inColorField[i].b;
    }
    
}


//#include "UserColor.cpp"