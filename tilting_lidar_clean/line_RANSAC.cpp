#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<time.h>

//created by viewing and modifying code from https://github.com/sanmukh/ransac/blob/master/ransac.c

using namespace std;

int rand_angle;
float rand_x[2];
float rand_y[2];
float m;
float b;
float b_test;
float threshold = 1;
int total_good;
float b_best;
float m_best;
int best_good;
float inf = numeric_limits<float>::infinity();
ros::Publisher pub;

void RANSAC(const sensor_msgs::LaserScan msg)
{
   ros::NodeHandle nh;
   srand(time(NULL));
   sensor_msgs::LaserScan aux;
   best_good = 0;

   //transfer constant message properties
   aux.header = msg.header;
   aux.angle_min = msg.angle_min;
   aux.angle_max = msg.angle_max;
   aux.angle_increment = msg.angle_increment;
   aux.time_increment = msg.time_increment;
   aux.scan_time = msg.scan_time;
   aux.range_min = msg.range_min;
   aux.range_max = msg.range_max;
   aux.intensities[920];

   aux.ranges.resize(920);
   rand_x[1] = inf;
   rand_y[1] = inf;
   int counter = 0;

     //convert first random point in scan to Cartesean
     while((rand_x[1] == inf || rand_y[1] == inf) && counter < 50)
     {
       rand_angle = rand() % 920 + 1;      
       if (msg.ranges[rand_angle] < 3)
         {
           rand_x[1] = msg.ranges[rand_angle] * cos(rand_angle * msg.angle_increment + msg.angle_min);
           rand_y[1] = msg.ranges[rand_angle] * sin(rand_angle * msg.angle_increment + msg.angle_min);
         }
       else
         {
           counter++;
         }
     }

     if (counter == 50)
     {
        return;
     }

else {
   //peform 50 times
   for (int i = 0; i < 50; i++)
   {
     total_good = 0;
     rand_x[2] = inf;
     rand_y[2] = inf;
     counter = 0;
     m = 0;
     int bad = 1;

     while (counter < 50 && bad == 1)
     {

     //convert second random point in scan to Cartesean
     while(rand_x[2] == inf || rand_y[2] == inf)
     {
       rand_angle = rand() % 920 + 1;
       rand_x[2] = msg.ranges[rand_angle] * cos(rand_angle * msg.angle_increment + msg.angle_min);
       rand_y[2] = msg.ranges[rand_angle] * sin(rand_angle * msg.angle_increment + msg.angle_min);
     }
    
     //determine slope
     m = (rand_y[1] - rand_y[2]) / (rand_x[1] - rand_x[2]);

     //determine b
     b = msg.ranges[rand_angle] * (sin(rand_angle * msg.angle_increment + msg.angle_min) - m * cos(rand_angle * msg.angle_increment + msg.angle_min));

     /*if (m > 1 || m < -1)
     {
       counter ++;
     }

     else
     {*/
       bad = 0;
    // }
}

     if (counter == 50)
     { return; }

     //count number of good points
     for(int n = 0; n < 921; n++)
     {
        if(msg.ranges[n] != inf && msg.ranges[n] != -inf)
        {
          b_test = msg.ranges[n] * (sin((n * msg.angle_increment) + msg.angle_min) - m * cos((n * msg.angle_increment) + msg.angle_min));

          if (b_test > (b-threshold) && b_test < (b+threshold) && b_test != inf && b_test != -inf && b_test != NAN && b_test != -NAN)
          {
            total_good++;
          }
        }
     }
     
     if (total_good >= best_good)
     {
       b_best = b;
       m_best = m;
       best_good = total_good;
     }
   }

   //compile best laser scan
   for (int n = 0; n < 921; n++)
   {
      b_test = msg.ranges[n] * (sin((n * msg.angle_increment) + msg.angle_min) - m * cos((n * msg.angle_increment) + msg.angle_min));
      if (b_test > (b-threshold) && b_test < (b+threshold) && b_test != inf && b_test != -inf && b_test != NAN && b_test != -NAN)
        {
            aux.ranges[n] = msg.ranges[n];
        }
      else
        {
            aux.ranges[n] = inf;
        }
   }  
    
    ROS_INFO_STREAM(m_best);
    //publish best laser scan
    pub.publish(aux);
}
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "line_extract_ransac");
   ros::NodeHandle nh;
   ros::Subscriber sub=nh.subscribe<sensor_msgs::LaserScan>("/hokuyo_filtered", 1, &RANSAC);
   pub=nh.advertise<sensor_msgs::LaserScan>("/line_ransac", 1);
   ros::spin();
   return 0;
} 
