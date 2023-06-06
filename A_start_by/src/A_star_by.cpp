#include "include/headfile.h"
using namespace std;
struct MAP_MSG{
     int width;
     int height;
     int orign_x;
     int orign_y;
     float resolution;
     int half_x;
     int half_y;
};
MAP_MSG map_msg;
void  rviz_road5(ros::Publisher marker_pub, vector<Point> point){
         visualization_msgs::Marker points;
         points.header.frame_id =  "map";
         points.header.stamp = ros::Time::now();
         points.ns = "points_and_lines";
         points.action =  visualization_msgs::Marker::ADD;
         points.pose.orientation.w =  1.0;
         points.id = 0;
         points.type = visualization_msgs::Marker::LINE_STRIP;
         // POINTS markers use x and y scale for width/height respectively
         points.scale.x = map_msg.resolution;
         points.scale.y = map_msg.resolution;
         // Points are green
         points.color.g = 0.0f;
         points.color.r = 0;
         points.color.b = 1;
         points.color.a = 1.0;
         // Create the vertices for the points 
          for (size_t j = 0; j < point.size(); j++){
               geometry_msgs::Point p1;
               p1.y =  point[j].y*map_msg.resolution+map_msg.orign_x+map_msg.resolution/2;
               p1.x =  point[j].x*map_msg.resolution+map_msg.orign_y+map_msg.resolution/2; 
               p1.z = 0;
               points.points.push_back(p1);   
          }
          marker_pub.publish(points);
     }
void  rviz_road3(ros::Publisher marker_pub, vector<Point> rem_point_vec){
         visualization_msgs::Marker points;
         points.header.frame_id =  "map";
         points.header.stamp = ros::Time::now();
         points.ns = "points_and_lines";
         points.action =  visualization_msgs::Marker::ADD;
         points.pose.orientation.w =  1.0;
         points.id = 0;
         points.type = visualization_msgs::Marker::POINTS;
         // POINTS markers use x and y scale for width/height respectively
         points.scale.x = map_msg.resolution;
         points.scale.y = map_msg.resolution;
         // Points are green
         points.color.r = 1.0f;
         points.color.b = 1.0f;
         points.color.a = 0.1f;
         // Create the vertices for the points 
          for (size_t j = 0; j < rem_point_vec.size(); j++){
                    geometry_msgs::Point p1;   
                         p1.x =  rem_point_vec[j].x*map_msg.resolution+map_msg.orign_y;
                         p1.y =  rem_point_vec[j].y*map_msg.resolution+map_msg.orign_x;
                         p1.z = 0;
                         points.points.push_back(p1);  
          }
          marker_pub.publish(points);
     }
void  rviz_road2(ros::Publisher marker_pub, vector<Point> point){
          visualization_msgs::Marker points;
         points.header.frame_id =  "map";
         points.header.stamp = ros::Time::now();
         points.ns = "points_and_lines";
         points.action =  visualization_msgs::Marker::ADD;
         points.pose.orientation.w =  1.0;
         points.id = 0;
         points.type = visualization_msgs::Marker::POINTS;
         // POINTS markers use x and y scale for width/height respectively
         points.scale.x = map_msg.resolution;
         points.scale.y = map_msg.resolution;
         // Points are green
         points.color.g = 0.5f;
         points.color.r = 0.5f;
         points.color.b = 1.0f;
         points.color.a = 1.0;
         // Create the vertices for the points 
          for (size_t j = 0; j < point.size(); j++){
               geometry_msgs::Point p1;
               p1.x =  point[j].y*map_msg.resolution+map_msg.orign_y+map_msg.resolution/2;
               p1.y =  point[j].x*map_msg.resolution+map_msg.orign_x+map_msg.resolution/2; 
               p1.z = 0;
               points.points.push_back(p1);   
          }
          marker_pub.publish(points);
     }
void  rviz_road4(ros::Publisher marker_pub, vector<Point> point){
         visualization_msgs::Marker points;
         points.header.frame_id =  "map";
         points.header.stamp = ros::Time::now();
         points.ns = "points_and_lines";
         points.action =  visualization_msgs::Marker::ADD;
         points.pose.orientation.w =  1.0;
         points.id = 0;
         points.type = visualization_msgs::Marker::LINE_STRIP;
         // POINTS markers use x and y scale for width/height respectively
         points.scale.x = map_msg.resolution;
         points.scale.y = map_msg.resolution;
         // Points are green
         points.color.g = 0.0f;
         points.color.r = 1;
         points.color.b = 0;
         points.color.a = 1.0;
         // Create the vertices for the points 
          for (size_t j = 0; j < point.size(); j++){
               geometry_msgs::Point p1;
               p1.y =  point[j].y* map_msg.resolution+map_msg.orign_y+ map_msg.resolution/2;
               p1.x =  point[j].x* map_msg.resolution+map_msg.orign_x+ map_msg.resolution/2; 
               p1.z = 0;
               points.points.push_back(p1);   
          }
          marker_pub.publish(points);
     }
class A_star{
public:
     ros::Publisher marker_pub5; // 展示平滑处理后的曲线
     ros::Publisher marker_pub3; // 展示自己的地图
     ros::Publisher marker_pub2; // 展示A星搜索的路径
     ros::Publisher marker_pub4; ////展示平滑处理后的曲线

     vector<vector<int>> map;
     vector<vector<int>> map_final;

     vector<Point> rem_point_vec;
     vector<Point> path_points;

     int pz = 5;
     bool init_flag = 0;
     pair<int, int> start_by = {0, 0}; // 起点
     pair<int, int> over_by = {0, 0};  // 终点
     pair<int, int> target;            // 终点
     pair<int, int> start;             // 起点点
     Point end_point;
     Point vehicle_pose;

     void Node_Start(int argc, char **argv);
     void initMap(int map_width, int map_height);
     void GetMapCallBack(const nav_msgs::OccupancyGrid msg);
     void end_odom_callback(const geometry_msgs::PoseStamped::ConstPtr msg);
     void get_end_point_msgs(geometry_msgs::PoseStamped::ConstPtr msg)
     {
          over_by.first = msg->pose.position.x;
          over_by.second = msg->pose.position.y;
          end_point.x = msg->pose.position.x;
          end_point.y = msg->pose.position.y;

          float min_dis = 1000;
          int x_index = 0,y_index=0;

           for (int i = 0; i < map_msg.width; i++)
           {
                for (int j = 0; j < map_msg.height; j++)
                {
                     if ((pow((i * map_msg.resolution) + map_msg.orign_x - end_point.x, 2) + pow((j * map_msg.resolution) + map_msg.orign_y - end_point.y, 2)) < min_dis)
                     {
                          x_index = j;
                          y_index = i;
                          min_dis = pow((i * map_msg.resolution) + map_msg.orign_x- end_point.x, 2) + pow((j * map_msg.resolution) + map_msg.orign_y - end_point.y, 2);
                     }
                }
          }
          target = {x_index, y_index};
     }
     void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
};
void A_star::odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg){
// 得到车辆的定位信息和速度信息
    double raw, pitch, theta;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(raw, pitch, theta);
    vehicle_pose.theta = theta;
    vehicle_pose.x = odometry_msg->pose.pose.position.x;
    vehicle_pose.y = odometry_msg->pose.pose.position.y;
    start.first = ( vehicle_pose.y- map_msg.orign_y) / map_msg.resolution;
    start.second = ( vehicle_pose.x- map_msg.orign_x) / map_msg.resolution;
}
void A_star::end_odom_callback(const geometry_msgs::PoseStamped::ConstPtr msg){
     ros::Rate loop_rate(10);
     get_end_point_msgs(msg); // 找到鼠标点击地图所对应的map的位置
     loop_rate.sleep();

     AStar star(map);       // 构造函数读取地图
     opnlist_point.clear(); // 清除A星检索过的点的容器
     path_points.clear();   //
   
     star.PrintAStarPath(start, target); // A星算法

     for(int i=0;i<final_path.size();i+=10){//将路径放入path以便进行平滑处理
          Point path;
          path.y=final_path[i].first;
          path.x=final_path[i].second;
          path_points.push_back(path);
     }
     rviz_road5(marker_pub5, path_points);             // 没有经过平滑处理的路径
     path_points=B_spline_optimization(path_points);//B样条平滑处理
     rviz_road2(marker_pub2, opnlist_point);     // 探索路径显示
     rviz_road3(marker_pub3, rem_point_vec);           // 膨胀地图显示
     rviz_road4(marker_pub4, path_points); // 经过平滑处理的路径
    }
void A_star::initMap(int map_width,int  map_height){
     for (int i = 0; i < map_width; i++) {
     vector<int> t(map_height, 0);
          map.push_back(t);
     }
     opnlist_point.clear();
}
void A_star::GetMapCallBack(const nav_msgs::OccupancyGrid msg){
     map_msg.width = msg.info.width;       // 读取宽度384
     map_msg.height = msg.info.height;     // 读取高度384
     map_msg.orign_x = msg.info.origin.position.x; // 读取地图源点-10
     map_msg.orign_y = msg.info.origin.position.y;//-10
     map_msg.half_x =  map_msg.width / 2;
     map_msg.half_y =  map_msg.height / 2;
     map_msg.resolution = msg.info.resolution; // 读取地图分辨率0.05
     initMap( map_msg.width,  map_msg.height );  // 创建地图（此时地图只给了大小还未赋值）

     rem_point_vec.clear();
     for (int i = 0; i <  map_msg.width; i++)
     {
          for (int j = 0; j <  map_msg.height; j++)
          {
               if (msg.data[i * map_msg.width + j] == 0)
               {
                    map[i][j] = 0;
               } // 0表示可通过路径
               else if (msg.data[i * map_msg.width + j] == 100)
               {
                    map[i][j] = 1;
               } // 1表示障碍物
               else
               {
                    map[i][j] = 2;
               } // 2表示未探测的路径

               if (map[i][j] == 1)
               {
                    for (int k = i - pz; k < (i + pz +1); k++)
                    {
                         for (int z = j - pz; z < (j + pz +1); z++)
                         {
                              if (k > map_msg.width - 1 || z > map_msg.height - 1 || k < 0 || z < 0)
                              {
                                   continue;
                              } // 防止越界
                              if (map[k][z] != 0)
                                   continue;
                              Point rem_point;
                              rem_point.x = z;
                              rem_point.y = k;
                              rem_point_vec.push_back(rem_point); // 将膨胀的点放入容器
                         }
                    }
               }
          }
     }
          //膨胀处理
             for(int i=0;i<rem_point_vec.size();i++)  {
                    int z=rem_point_vec[i].x;
                    int k=rem_point_vec[i].y;
                    map[k][z]=1;
             }

          cout<<"road1 map is  ok"<<endl;
          init_flag=1;//地图读取完毕标志位
      }
void A_star::Node_Start(int argc,char **argv){
     ros::init(argc,argv,"A_start_by");
     ros::NodeHandle nc;

     marker_pub5 = nc.advertise<visualization_msgs::Marker>("road_rviz5", 1);   // 展示未经平滑处理后的曲线
     marker_pub3 = nc.advertise<visualization_msgs::Marker>("road_rviz3", 1);   // 展示膨胀的地图
     marker_pub2 = nc.advertise<visualization_msgs::Marker>("road_rviz2", 1);   // 展示A星搜索过的路径
     marker_pub4 = nc.advertise<visualization_msgs::Marker>("road2_rviz4", 1); // 展示平滑处理后的曲线

     ros::Subscriber sub_map = nc.subscribe("/robot_1/map", 1, &A_star::GetMapCallBack, this);                   // 只运行一次，读取地图数据
     ros::Subscriber sub_end_odom = nc.subscribe("/move_base_simple/goal", 1, &A_star::end_odom_callback, this); // 订阅终点位置，并运行A星算法
     ros::Subscriber sub_odom = nc.subscribe("/odom", 1, &A_star::odometryGetCallBack, this);                    // 获取小车当前位置
     ros::spin();
}  
int main(int argc, char  *argv[])
{
     A_star a_star;
     a_star.Node_Start(argc, argv);
     return 0;
}
