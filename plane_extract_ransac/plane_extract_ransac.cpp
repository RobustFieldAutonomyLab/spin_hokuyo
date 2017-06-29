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

class PlaneExtractionRANSAC{

private:

    /* Sensor Configuration: Velodyne Puckl   */
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

    float plane_dist = 0.05;

public:
    PlaneExtractionRANSAC():
        nh("~"),
        it(nh){

        factor = 1.0f / (max_range - min_range);
        offset = -min_range;

        subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/assembled_cloud", 1, &PlaneExtractionRANSAC::cloudHandler, this);
        pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/planes_pointcloud", 1);  

        pubImage = it.advertise("/image_out", 1);

        allocateMemory();
    }

     void allocateMemory(){

         laserCloudIn.reset(new pcl::PointCloud<PointType>());
         laserCloudOut.reset(new pcl::PointCloud<PointType>());

    }

    ~PlaneExtractionRANSAC(){}


vector<float> crossProduct(vector<float> a, vector<float> b) {
	vector<float> res{0,0,0};
	if(a.size()!=3 && b.size()!=3) return res;
	res[0] = a[1]*b[2] - a[2]*b[1];
	res[1] = a[2]*b[0] - a[0]*b[2];
	res[2] = a[0]*b[1] - a[1]*b[0];
	return res;
}

float distPointPlane(vector<float> origin, vector<float> norm, vector<float> point) {
	float res;
	vector<float> a(3,0), b(3,0);
	float length = sqrt(norm[0]*norm[0] + norm[1]*norm[1] + norm[2]*norm[2]);

	for (int i=0;i<3;i++) {
		a[i] = point[i] - origin[i];
		b[i] = norm[i] / length;
	}

	res = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
	return abs(res);
}

vector<vector<float>> planeExtraction_RANSAC(vector<vector<float>> points, float d) {
	// generate index
	vector<int> idx(points.size(), 0);
	for(int i=0;i<points.size();i++) {
		idx[i] = i;
	}
	// ransac
	long iter_ransac = 50, ptr_best_model = 0, num_points_max = 0;
	vector<vector<int>> models(iter_ransac, std::vector<int>(3,0));
	vector<long> pts_each_model(iter_ransac, 0);
	vector<float> planeNorm(3, 0), tmpO(3, 0), tmpA(3, 0), tmpB(3, 0);   //dists(points.size(), 0),

	for(int i=0;i<iter_ransac;i++) {
		// shuffle points
		std::random_shuffle(idx.begin(), idx.end());

		// pick a model
		models[i][0] = idx[0];
		models[i][1] = idx[1];
		models[i][2] = idx[2];
		
		// calculate norm
		for (int j=0;j<3;j++) {
			tmpA[j] = points[idx[0]][j] - points[idx[1]][j];
			tmpB[j] = points[idx[2]][j] - points[idx[1]][j];
			tmpO[j] = points[idx[1]][j];
		}
		planeNorm = crossProduct(tmpA, tmpB);
		
		// calculate number of points within the plane for current model 
		for(auto t : idx) {
			if(distPointPlane(tmpO, planeNorm, points[t]) <= d) {
				pts_each_model[i]++;
			}
		}

		// update the pointer to best model
		
		if (num_points_max < pts_each_model[i]) {
			ptr_best_model = i;
			num_points_max = pts_each_model[i];
		}
	}

	// pack all points in the best model and return
	
	// selected model: points[ model[ptr_best_model] [0-2]
	// calculate norm
	for (int j=0;j<3;j++) {
		tmpA[j] = points[models[ptr_best_model][0]][j] - points[models[ptr_best_model][1]][j];
		tmpB[j] = points[models[ptr_best_model][2]][j] - points[models[ptr_best_model][1]][j];
		tmpO[j] = points[models[ptr_best_model][1]][j];
	}
	planeNorm = crossProduct(tmpA, tmpB);

	vector<vector<float>> res;
	for(auto p : points) {
		if(distPointPlane(tmpO, planeNorm, p) <= d) {
			res.push_back(p);
		}
	}

	return res;
}
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

        vector<vector<float>> points;
        vector<vector<float>> planes_points;
        // fill in points
        for ( auto p : laserCloudIn->points) {
        	points.push_back({p.x, p.y, p.z});
        }
        double t_start = ros::Time::now().toSec();
        planes_points = planeExtraction_RANSAC( points, plane_dist);
        double t_dur = ros::Time::now().toSec() - t_start;

        laserCloudOut->clear();
        for (auto p : planes_points) {
        	PointType t;
        	t.x = p[0]; t.y = p[1]; t.z = p[2]; t.intensity = 1;
        	laserCloudOut->push_back(t);
        }
        
        cout << "extracted plane with " << planes_points.size() << "/" << points.size() << "points" 
             << " in " << t_dur << "Secs" << endl;
        // 2. Publish message
        sensor_msgs::PointCloud2 laserCloudFull;
        pcl::toROSMsg(*laserCloudOut, laserCloudFull);

        laserCloudFull.header.stamp = ros::Time();
        laserCloudFull.header.frame_id = laserCloudMsg->header.frame_id;
        // ROS_INFO("publishing point cloud in the largest plane");
        pubCloud.publish(laserCloudFull);

    }


};




int main(int argc, char** argv){

    ros::init(argc, argv, "plane_extract_ransac");
    
    PlaneExtractionRANSAC PS;

    ROS_INFO(" Extracting planes using RANSAC...");

    ros::spin();
    return 0;
}




