#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include<pcl/io/pcd_io.h>
#include<boost/timer.hpp>
#include<ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/Odometry.h>
using namespace std;
int clientSocket;
int pocket_number;
bool send_sign;
double cloud_time,odom_time;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr laserMsg_ptr(new pcl::PointCloud<pcl::PointXYZI>);
Eigen::Matrix4f transform_matrix=Eigen::Matrix4f::Identity();

Eigen::Matrix4f odom2matrix4f(const nav_msgs::Odometry odom_msg) 
{
  geometry_msgs::Quaternion orientation = odom_msg.pose.pose.orientation;
  geometry_msgs::Point position = odom_msg.pose.pose.position;
  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;
//   printf("x:%f\n",position.x);

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry.matrix().cast<float>();
} 

void cloud_send(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr,Eigen::Matrix4f odom_matrix, double cloud_time, double odom_time)
{
    pcl::PointCloud<pcl::PointXYZI> inMap;
    while(1)
    {
        if (send_sign == true)
        {
            send_sign = false;
            inMap = *pointcloud_ptr;
            std::string buf,str,time_buf,odom_buf;
            pcl::PointCloud<pcl::PointXYZI> map;

            if(inMap.size()>0)
            {
                map = inMap;
                inMap.clear();
                int pocket = 2;
                int left_num = map.height * map.width;
                int it_num,start_index,end_index;
                int pocket_id=0,pocket_num,pocket_size;

                //分包，使每个包的点数小于2000
                if(left_num > 0){
                    do{
                        it_num = left_num / pocket;
                        if(it_num > 2000)
                        {
                            pocket++;
                        }
                    }while(it_num > 2000);

                    if(left_num!=it_num*pocket)
                    {pocket++;}

                    for(int i = 0;left_num >0;i++)
                    {
                        if(left_num >0 && left_num < it_num)
                        {
                            it_num = left_num;
                            start_index = end_index;
                            end_index += it_num;
                        }
                        else
                        {
                            start_index = i*it_num;
                            end_index = (i+1)*it_num;
                        }

                        //点云时间
                        char th[8];
                        memcpy(th,&cloud_time,sizeof(double));
                        time_buf.push_back(th[0]);
                        time_buf.push_back(th[1]);
                        time_buf.push_back(th[2]);
                        time_buf.push_back(th[3]);
                        time_buf.push_back(th[4]);
                        time_buf.push_back(th[5]);
                        time_buf.push_back(th[6]);
                        time_buf.push_back(th[7]);  
                        //odom时间
                        memcpy(th,&odom_time,sizeof(double));
                        time_buf.push_back(th[0]);
                        time_buf.push_back(th[1]);
                        time_buf.push_back(th[2]);
                        time_buf.push_back(th[3]);
                        time_buf.push_back(th[4]);
                        time_buf.push_back(th[5]);
                        time_buf.push_back(th[6]);
                        time_buf.push_back(th[7]); 
                        // cout<<"timebufsize:"<<time_buf.size();

                        //odom
                        char odom_ch[64];
                        memcpy(odom_ch,&odom_matrix,sizeof(odom_matrix));
                        for(int p = 0;p<64;p++)
                        {
                            odom_buf.push_back(odom_ch[p]);
                        }

                        for(int j = start_index; j< end_index; j++)   //一个包的所有点
                        {
                            char ch[4];
                            memcpy(ch,&map.points.at(j).x,sizeof(float));
                            buf.push_back(ch[0]);
                            buf.push_back(ch[1]);
                            buf.push_back(ch[2]);
                            buf.push_back(ch[3]);

                            memcpy(ch,&map.points.at(j).y,sizeof(float));
                            buf.push_back(ch[0]);
                            buf.push_back(ch[1]);
                            buf.push_back(ch[2]);
                            buf.push_back(ch[3]);

                            memcpy(ch,&map.points.at(j).z,sizeof(float));
                            buf.push_back(ch[0]);
                            buf.push_back(ch[1]);
                            buf.push_back(ch[2]);
                            buf.push_back(ch[3]);

                            memcpy(ch,&map.points.at(j).intensity,sizeof(float));
                            buf.push_back(ch[0]);
                            buf.push_back(ch[1]);
                            buf.push_back(ch[2]);
                            buf.push_back(ch[3]);
                        }

                        unsigned head_1,head_2,end;
                        int be32;

                        head_1 = 0x55aa;
                        head_2 = 0x55aa;
                        // pocket_id = i + 1;    //包id定义为每个包的起始点云的次序
                        pocket_id++;
                        pocket_num = pocket;
                        pocket_size = it_num;
                        pocket_number = pocket_num;
                        end = 0xaa55;

                        //header
                        be32 = htonl(head_1);    //主机转网络字节，转换为32位,四个字节
                        str.append(reinterpret_cast<char *>(&be32),sizeof(be32));
                        be32 = htonl(head_2);
                        str.append(reinterpret_cast<char *>(&be32),sizeof(be32));
                        //pocket_id
                        be32 = htonl(pocket_id);
                        str.append(reinterpret_cast<char *>(&be32),sizeof(be32));
                        //pocket_num
                        be32 = htonl(pocket_num);
                        str.append(reinterpret_cast<char *>(&be32),sizeof(be32));
                        //pocket_size
                        be32 = htonl(pocket_size);
                        str.append(reinterpret_cast<char *>(&be32),sizeof(be32));
                        //time
                        str.append(time_buf);
                        //odom
                        str.append(odom_buf);
                        //cloud_data
                        str.append(buf);
                        //end
                        be32 = htonl(end);
                        str.append(reinterpret_cast<char *>(&be32),sizeof(be32));
                        send(clientSocket,str.data(),str.size(),0);
                        // cout<<"point_num "<<pocket_size<<"  odomSize "<<sizeof(odom_ch);
                        // printf("  pocket_size:%d\n",str.size());
                        buf.clear();
                        str.clear();
                        time_buf.clear();
                        odom_buf.clear();
                        left_num -= it_num;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else
        {
            break;
        }
    }
}


void cloudRecvCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloud)
{
    pcl::fromROSMsg(*laserCloud,*laserMsg_ptr);
    cloud_time = laserCloud->header.stamp.toSec();
}

void callback_odom(const nav_msgs::Odometry odom)
{
    transform_matrix = odom2matrix4f(odom);
    odom_time = odom.header.stamp.toSec();
}




int main(int argc, char *argv[])
{
    ros::init(argc,argv,"pointCloud_client");
    ros::NodeHandle n;

    //1. 创建socket  
	clientSocket = socket(AF_INET,SOCK_STREAM,0);
	if(-1 == clientSocket) printf("创建socket失败:%m\n"),exit(-1);
	printf("创建socket成功!\n");
 
	//2. 创建服务器协议地址簇
	struct sockaddr_in cAddr = {0};
	cAddr.sin_family = AF_INET; 
	cAddr.sin_addr.s_addr = inet_addr("10.42.0.1"); 
	cAddr.sin_port = htons(9527);  
 
	//3.连接服务器
	int r = connect(clientSocket,(struct sockaddr*)&cAddr,sizeof cAddr);
	if(-1 == r) printf("连接服务器失败:%m\n"),close(clientSocket),exit(-2);
	printf("连接服务器成功!\n");

    //4.订阅并发送
    ros::Subscriber pointCloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 5, &cloudRecvCallback);
    ros::Subscriber odom_sub = n.subscribe("/Odometry",5,callback_odom);
    ros::Rate loop_rate(2);
    int sendTimes = 1;

    while(ros::ok())
    {
        send_sign = true;
        // cout<<"odomMatrix:"<<transform_matrix<<"--done"<<endl;
        // cout<<"sizeof:"<<sizeof(transform_matrix)<<endl;
        if(laserMsg_ptr->size()>0 & odom_time>0)
        {
            cloud_send(laserMsg_ptr,transform_matrix,cloud_time,odom_time);
            cout<<"第"<<sendTimes<<"次发送，共"<<pocket_number<<"个包,size:"<<laserMsg_ptr->size();
            printf(",cloudtime:%f,odomtime:%f\n",cloud_time,odom_time);
            sendTimes++;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    //5.关闭发送
    close(clientSocket);
    printf("数据发送完毕！\n");

    return 0;

}
