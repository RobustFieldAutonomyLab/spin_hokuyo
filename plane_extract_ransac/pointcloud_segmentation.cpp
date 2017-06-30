#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <unordered_set>

using namespace std;

typedef pcl::PointXYZI  PointType;

struct index_t{
    int column;
    int row;
};

class PointcloudSegmentation{

private:

    /*
        Sensor Configuration: Velodyne Puck
        */
    static const int N_SCAN = 16;
    static const int Horizon_SCAN = 1800;

     float max_range = 100;
     float min_range = 0.4;

    float ang_res_x = 0.2;
    float ang_res_y = 2.0;

     float ang_start_x = 270;
     float ang_start_y = 15;

    float factor;
    float offset;

    /*
        ROS node handle, publisher, subscriber
        */
    ros::NodeHandle nh;
    image_transport::ImageTransport it;

    image_transport::Publisher  pubImage;
    ros::Subscriber             subCloud;
    ros::Publisher              pubCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr laserCloudOut;
    pcl::PointCloud<PointType>::Ptr laserCloudMatrix[N_SCAN];

    cv_bridge::CvImagePtr rangeImageFilter;

    // Filter
    int labelCount;
    float theta = 15;
    vector< vector<float> > rangeMatrix;
    vector< vector<int> >   labelMatrix;
    vector<int> labelNumPoints; // saves the number of points that belong to a label

public:
    PointcloudSegmentation():
        nh("~"),
        it(nh){

        factor = 1.0f / (max_range - min_range);
        offset = -min_range;

        subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &PointcloudSegmentation::cloudHandler, this);
        pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud", 1);  

        pubImage = it.advertise("/image_out", 1);

        allocateMemory();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudOut.reset(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCAN; ++i)
            laserCloudMatrix[i].reset(new pcl::PointCloud<PointType>());

        rangeMatrix.resize(Horizon_SCAN);
        for (int i = 0; i < N_SCAN; ++i)
            rangeMatrix[i].resize(Horizon_SCAN);

        labelMatrix.resize(Horizon_SCAN);
        for (int i = 0; i < N_SCAN; ++i)
            labelMatrix[i].resize(Horizon_SCAN);
    }

    ~PointcloudSegmentation(){}
    /*
        cloudHadler:
        Receive velodyne point cloud and convert it to range image
        */
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        // 1. Convert ros message to pcl point cloud
        laserCloudIn->clear();
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn,*laserCloudIn, indices);

        PointType thisPoint;

        // 2. Point cloud to range image (full resolution)
        cv::Mat rangeImage;
        rangeImage = cv::Mat::zeros(N_SCAN, Horizon_SCAN, cv_bridge::getCvType("mono16"));

        int cloudSize = laserCloudIn->points.size();

        for (int i = 0; i < cloudSize; ++i){

            thisPoint.x =  laserCloudIn->points[i].x;
            thisPoint.y = -laserCloudIn->points[i].y;
            thisPoint.z =  laserCloudIn->points[i].z;

            /**
                2.1 Find the row and column index in the iamge for this point
                */
            float verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            int rowIdn = -(verticalAngle - 15.1) / ang_res_y;

            int columnIdn;
            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            if (horizonAngle <= -90)
                columnIdn = -int(horizonAngle / ang_res_x) - 450; 
            else if (horizonAngle >= 0)
                columnIdn =  -int(horizonAngle / ang_res_x) + 1350;
            else
                columnIdn =  1350 - int(horizonAngle / ang_res_x);

            /**
                2.2 Normalize the range
                */
            float range = sqrt(thisPoint.x * thisPoint.x + 
                               thisPoint.y * thisPoint.y + 
                               thisPoint.z * thisPoint.z);

            range = (!std::isinf(range)) ? 
                    std::max(0.0f, std::min(1.0f, factor * (range + offset))) : 
                    0.0;

            rangeImage.at<ushort>(rowIdn, columnIdn) = static_cast<ushort>((range) * std::numeric_limits<ushort>::max());
        }

        // 3. Visualize Range Image
        sensor_msgs::ImagePtr msgImage;
        msgImage = cv_bridge::CvImage(std_msgs::Header(), "mono16", rangeImage).toImageMsg();
        msgImage->header = laserCloudMsg->header;
        pubImage.publish(msgImage);

        // 4. Process Range Image
        imageHandler(msgImage);
    }

    /*
        imageHandler:
        Receive the range image and process it
        */
    void imageHandler(const sensor_msgs::ImageConstPtr& msgImage){
        // 0. Initialize labelCount to zero for a new range image
        labelCount = 1;

        // 1. Create range image and get row column information
        rangeImageFilter = cv_bridge::toCvCopy(msgImage, msgImage->encoding);

        int cols = rangeImageFilter->image.cols;
        int rows = rangeImageFilter->image.rows;

        PointType point;
        // 2. Convert from range image to point cloud
        for (int i = 0; i < rows; ++i){
            for (int j = 0; j < cols; ++j){   
                // 2.1 Calculate range value
                ushort range_img = rangeImageFilter->image.at<ushort>(i, j);
                float range = rescaleRange(range_img); // Rescale range
                ////////////////////////////////////////////////////////////////// 
                point.x =  sin(pcl::deg2rad(ang_start_x - j * ang_res_x)) * range;
                point.y = -cos(pcl::deg2rad(ang_start_x - j * ang_res_x)) * range;
                point.z =  sin(pcl::deg2rad(ang_start_y - i * ang_res_y)) * range;
                // 2.2 No valid range value or ground, set this point is invalid point (-1 flag)
                if (range_img == 0 || point.z <= -0.3 || point.z > 3.0)
                    point.intensity = -1; 
                else
                    point.intensity = 0;
                // 2.3 Save point and range information
                laserCloudMatrix[i]->push_back(point);
                rangeMatrix[i][j] = range;
            }
        }

        // // 3. Initialize Label Matrix
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)  
                labelMatrix[i][j] = 0;

        // // 4. Label Range Image
        int numPoints;
        labelNumPoints.clear();

        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < cols; ++j){
                if (labelMatrix[i][j] == 0 && 
                    laserCloudMatrix[i]->points[j].intensity != -1){
                    numPoints = labelComponents(i, j);
                    labelNumPoints.push_back(numPoints);
                    ++labelCount;
                }
            }
        }
        // cout << labelCount << " segments detected!" << endl; 
        // 5. Publish Labeled Point Cloud
        publishCloud();
    }



    int labelComponents(int r, int c){
        std::queue<index_t> Queue;
        index_t thisIndex;

        thisIndex.row = r;
        thisIndex.column = c;
        Queue.push(thisIndex);

        int objectPointsCount = 0;

        while(Queue.size() > 0){
            // Pop point
            index_t fromIndex = Queue.front();
            Queue.pop();
            // Mark popped point
            labelMatrix[fromIndex.row][fromIndex.column] = labelCount;

            // Loop through all the neighboring grids of popped grid
            for (int i = -1; i <= 1; ++i){
                for (int j = -1; j <= 1; ++j){
                    // Continue if the checked grid is the popped grid itself or diagonal neighbor
                    if ((i == 0 && j == 0) || (i == -1 && j == -1) || (i == -1 && j == 1) || (i == 1 && j == 1) || (i == 1 && j == -1))
                        continue;

                    thisIndex.row = fromIndex.row + i;
                    thisIndex.column = fromIndex.column + j;
                    // index should be within the boundary
                    if (thisIndex.row < 0 || thisIndex.row >= N_SCAN || thisIndex.column < 0 || thisIndex.column >= Horizon_SCAN)
                        continue;
                    // prevent infinite loop (put already examined point back)
                    if (labelMatrix[thisIndex.row][thisIndex.column] > 0)
                        continue;
                    // if a neighbor point has invalid range value, treat it as labeled (it can not be used for further examination)
                    if (laserCloudMatrix[thisIndex.row]->points[thisIndex.column].intensity == -1){
                        labelMatrix[thisIndex.row][thisIndex.column] = labelCount;
                        continue;
                    }
                    // paper code
                    float d1 = std::max(rangeMatrix[fromIndex.row][fromIndex.column], 
                                        rangeMatrix[thisIndex.row][thisIndex.column]);
                    float d2 = std::min(rangeMatrix[fromIndex.row][fromIndex.column], 
                                        rangeMatrix[thisIndex.row][thisIndex.column]);

                    float alpha;
                    if (i == 0)
                        alpha = pcl::deg2rad(ang_res_x);
                    else
                        alpha = pcl::deg2rad(ang_res_y);

                    float angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                    if (angle > pcl::deg2rad(theta)){
                        Queue.push(thisIndex);
                        labelMatrix[thisIndex.row][thisIndex.column] = labelCount;
                        laserCloudMatrix[thisIndex.row]->points[thisIndex.column].intensity = labelCount;
                    }
                }
            }
            // if a point is isolated, after one loop, there should be no points pushed into Queue.
            // discard this point since one point can not be regarded as an object
            if (objectPointsCount == 0 && Queue.size() == 0){
                laserCloudMatrix[fromIndex.row]->points[fromIndex.column].intensity == -1;
                return 1;
            }
            ++objectPointsCount;
        }
        return objectPointsCount;
    }


    void publishCloud(){
        // 1. Save point to laserCloud from laserCloudMatrix  
        unsigned long cnt = 0;
        std::unordered_set<float> label_set;

        for (int i = 0; i < N_SCAN; ++i){     
            for (int j = 0; j < Horizon_SCAN; ++j){
            
                // 1.1 point that has no range information or isolated is not published
                //      or label that has number of points less than xx is discarded
                if (laserCloudMatrix[i]->points[j].intensity <= 0
                    || labelNumPoints[labelMatrix[i][j]-1] < 30 )
                    // || labelNumPoints[labelMatrix[i][j]-1] > 500 )
                    continue;
                laserCloudOut->push_back(laserCloudMatrix[i]->points[j]);
                label_set.insert(laserCloudMatrix[i]->points[j].intensity);
            }
        }
        cout << label_set.size() << " segments publishing!" << endl;
        // 2. Publish message
        sensor_msgs::PointCloud2 laserCloudFull;
        pcl::toROSMsg(*laserCloudOut, laserCloudFull);

        laserCloudFull.header.stamp = ros::Time();
        laserCloudFull.header.frame_id = "/velodyne";

        pubCloud.publish(laserCloudFull);
        // 3. Clear cloud
        laserCloudOut->clear();
        for (int i = 0; i < N_SCAN; ++i)
            laserCloudMatrix[i]->clear();
    }

    float rescaleRange(ushort range_img){
        float range = static_cast<float>(range_img) /
                      static_cast<float>(std::numeric_limits<ushort>::max());
        return (range - offset*factor) / factor;
    }




};




int main(int argc, char** argv){

    ros::init(argc, argv, "pointcloud_segmentation");
    
    PointcloudSegmentation PS;

    ROS_INFO("----> Point Cloud Segmentation Started.");

    ros::spin();
    return 0;
}





