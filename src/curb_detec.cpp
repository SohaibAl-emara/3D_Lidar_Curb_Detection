#include <sensor_msgs/PointCloud2.h>
#include "ros/ros.h"
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <vector>
using namespace std;


// Help sort point clouds.
// Ascending order. (For points where y is greater than 0)

bool comp_up(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.y < b.y;
}

// Descending. (For points where y is less than 0)
bool comp_down(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.y > b.y;
}

// This class is used to detect curb. 
// Input a point cloud and return a point cloud consisting of detected curb points.
// The main execution function is detection.
class curbDetector
{
    public:
    curbDetector(){}

    pcl::PointCloud<pcl::PointXYZ> detector(const std::vector<pcl::PointCloud<pcl::PointXYZ> > input)
    {
        pc_in = input;
        pcl::PointCloud<pcl::PointXYZ> look_test;


        // Process and detect each layer. Since we took 10 layers of 64 lidar here, the number of layers here is 10.
        for (int i = 0; i < 10; i++)
        {
            pcl::PointCloud<pcl::PointXYZ> pointsInTheRing = pc_in[i]; // Save the points on this line.
            pcl::PointCloud<pcl::PointXYZ> pc_left; // Store the point where y is greater than 0 (the left point).
            pcl::PointCloud<pcl::PointXYZ> pc_right; // Store the point where y is less than 0 (the right point).

            pcl::PointCloud<pcl::PointXYZ> cleaned_pc_left; 
            pcl::PointCloud<pcl::PointXYZ> cleaned_pc_right; 

            pcl::PointXYZ point; 
            size_t numOfPointsInTheRing = pointsInTheRing.size(); 
            
            // Separate the left and right points and save them to the corresponding point cloud.           
            for (int idx = 0; idx < numOfPointsInTheRing; idx++)
                {
                point = pointsInTheRing[idx];
                if (point.y >= 0)
                {pc_left.push_back(point);}
                else
                {pc_right.push_back(point);}
            }

            // Sort. (In ascending order of absolute value)
            sort(pc_left.begin(), pc_left.end(), comp_up);
            sort(pc_right.begin(), pc_right.end(), comp_down);


            slideForGettingPoints(pc_left, true);
            slideForGettingPoints(pc_right, false);

        }
    return curb_left + curb_right;

    }

    int slideForGettingPoints(pcl::PointCloud<pcl::PointXYZ> points, bool isLeftLine)
    {
        int w_0 = 10;
        int w_d = 30;
        int i = 0;

        // some important parameters influence the final performance.
        float xy_thresh = 0.1;
        float z_thresh = 0.06;

        int points_num = points.size();

        while((i + w_d) < points_num)
        {
            float z_max = points[i].z;
            float z_min = points[i].z;

            int idx_ = 0;
            float z_dis = 0;

            for (int i_ = 0; i_ < w_d; i_++)
            {
                float dis = fabs(points[i+i_].z - points[i+i_+1].z);
                if (dis > z_dis) {z_dis = dis; idx_ = i+i_;}
                if (points[i+i_].z < z_min){z_min = points[i+i_].z;}
                if (points[i+i_].z > z_max){z_max = points[i+i_].z;}
            }

            if (fabs(z_max - z_min) >= z_thresh)
            {
                for (int i_ = 0; i_ < (w_d - 1); i_++)
                {
                    float p_dist = sqrt(((points[i + i_].y - points[i + 1 + i_].y) * (points[i + i_].y - points[i + 1 + i_].y)) 
                    + ((points[i + i_].x - points[i + 1 + i_].x) *(points[i + i_].x - points[i + 1 + i_].x)));
                    if (p_dist >= xy_thresh)
                    {
                        if (isLeftLine) {curb_left.push_back(points[i_ + i]);return 0;}
                        else {curb_right.push_back(points[i_ + i]);return 0;}
                    }
                }
                if (isLeftLine) {curb_left.push_back(points[idx_]);return 0;}
                else {curb_right.push_back(points[idx_]);return 0;}
            }
            i += w_0;
        }
    }

    private:
    std::vector<pcl::PointCloud<pcl::PointXYZ> > pc_in;
    pcl::PointCloud<pcl::PointXYZ> curb_left;
    pcl::PointCloud<pcl::PointXYZ> curb_right;
};


class rosTransPoints 
{
    public:
    rosTransPoints()
    {
        //Set the parameters of the area of interest. The four parameters set the rectangle of this region of interest.
        // X is used to define the distance of the detection infront of the car.
        // Y is used to define the distance from the car to the curb.
        // x_up means infront of the car. x_down behid the car.
        // y_up means how far from the middle of the car to the left
        // y_down means how far from the middle of the car to the right
        // z, i am not very sure about it, but it is most likely the hight of the curb 
        // ring_idx is the layer id. The lidar that they are using has 64 layers, and they selected 10 layers for detection. 
        x_range_up = 30;
        x_range_down = 0;
        y_range_up = 20;
        y_range_down = -10;
        z_range_down = -1.5;
        int ring_idx_[10] = {25, 27, 30, 32, 35, 38, 40, 41, 42, 43};
        memcpy(ring_idx, ring_idx_, sizeof(ring_idx_));

        // define the publisher and the subscriber
        pub = nh.advertise<sensor_msgs::PointCloud2>("/curb_detection_result", 1);
        sub = nh.subscribe("/points_raw",3,&rosTransPoints::call_back,this);
    }

    void call_back(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
    {
        // this fuction is the callback for the subscriber, it gets excuted everytime a point cloud data is obtained through topic /points_raw
        clock_t startTime,endTime;
        startTime = ros::Time::now().toNSec();
        curbDetector cd;

        pcl::fromROSMsg(*input, cloud_in);

        // Here to add the code for processing the pc(pointcloud).
        cloud_cleaned = cleanPoints(cloud_in);
        cloud_out = cd.detector(cloud_cleaned);

        pcl::toROSMsg(cloud_out, cloud_final);
        cloud_final.header.stamp = ros::Time::now();
        cloud_final.header.frame_id = "velodyne";
        pub.publish (cloud_final);

        endTime = ros::Time::now().toNSec();
        cout << "The run time is:" << (double)(endTime - startTime) / 10e6 << "ms" << endl;
    }

    int hiPoints_WhereAreYouFrom(pcl::PointXYZ p)
    {
        // find which to which layer the point belongs
        // refer to this code to understand it：https://github.com/luhongquan66/loam_velodyne/blob/master/src/lib/MultiScanRegistration.cpp
        double angle;
        int scanID;
        angle = atan(p.z / sqrt(p.x * p.x + p.y * p.y));
        // double test = sqrt(p.x * p.x + p.y * p.y);
        // cout << test << endl;
        scanID = (int)(angle * 134.18714161056457 + 58.81598513011153);

        return scanID;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ> > cleanPoints(pcl::PointCloud<pcl::PointXYZ> pc)
    {
        //  this function cleans the points, this way you can have only the 10 layers we want 
        // 1. gets rid of unwanted layers
        // 2. takes the points according to the layer
        // 3. save the new layers in a new pointcloud mesage and return them
        size_t cloudSize = pc.size();
        // size_t ringSize;
        pcl::PointXYZ point;
        int scanID_;
        // pcl::PointCloud<pcl::PointXYZ> _laserCloud;
        std::vector<pcl::PointCloud<pcl::PointXYZ> > laserCloudScans(10);

        for (int i = 0; i < cloudSize; i++)
        {
            point.x = pc[i].x;
            point.y = pc[i].y;
            point.z = pc[i].z;

            if (!pcl_isfinite(point.x) || 
            !pcl_isfinite(point.y) || 
            !pcl_isfinite(point.z))
            {continue;}
            if ((point.x < x_range_down) || (point.x > x_range_up) || (point.y < y_range_down) || (point.y > y_range_up) || (point.z > z_range_down))
            {continue;}

            scanID_ = hiPoints_WhereAreYouFrom(point);

            for (int ring_num = 0;ring_num < 10; ring_num++)
            {
                if (scanID_ == ring_idx[ring_num])
                {laserCloudScans[ring_num].push_back(point);}
            }
        }

        return laserCloudScans;
    }

    private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    std::vector<pcl::PointCloud<pcl::PointXYZ> > cloud_cleaned;
    sensor_msgs::PointCloud2 cloud_final;

    // parameters help to select the detection scope.
    int x_range_up;
    int x_range_down;
    int y_range_up;
    int y_range_down;
    float z_range_down;
    int ring_idx[10];

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "curb_detector");
    rosTransPoints start_detec;
    ros::spin();
}

