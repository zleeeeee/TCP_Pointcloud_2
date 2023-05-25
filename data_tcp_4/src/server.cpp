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
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/QuadWord.h>
#include <pthread.h>
using namespace std;
int serverSocket, clientSocket;
int pocketMinSize = 33000;//要大于最大的pocket_size
pcl::PointCloud<pcl::PointXYZI> points;
sensor_msgs::PointCloud2 output_msg;
Eigen::Matrix4f odom_matrix;
nav_msgs::Odometry odom;
bool recv_sign = false;


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"pointCloud_server");
    ros::NodeHandle n; 
    ros::Publisher pubRecvdata = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points_recv", 5);
    ros::Publisher pubOdom = n.advertise<nav_msgs::Odometry>("/odom_recv",5);

    //1. 创建socket  
	serverSocket = socket(AF_INET,SOCK_STREAM,0);
	if(-1 == serverSocket) printf("创建socket失败:%m\n"),exit(-1);
	printf("创建socket成功!\n");
 
	//2. 创建服务器协议地址簇
	struct sockaddr_in sAddr = {0};
	sAddr.sin_family = AF_INET; 
	sAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
	sAddr.sin_port = htons(9527);  
 
	//3.绑定
	int r = bind(serverSocket,(struct sockaddr*)&sAddr,sizeof sAddr);
	if(-1 == r) printf("绑定失败:%m\n"),close(serverSocket),exit(-2);
	printf("绑定成功!\n");
 
	//4.监听
	r = listen(serverSocket,10);
	if(-1 == r) printf("监听失败:%m\n"),close(serverSocket),exit(-2);
	printf("监听成功!\n");
 
	//5.等待客户端连接
	struct sockaddr_in cAddr = {0};
	socklen_t len = sizeof(cAddr);
	clientSocket = accept(serverSocket,
		(struct sockaddr*)&cAddr,&len);
	if(-1 == clientSocket) printf("服务器崩溃:%m\n"),close(serverSocket),exit(-3);
	printf("有客户端连接上服务器了:%s\n",inet_ntoa(cAddr.sin_addr));

    //6.接收数据并发布话题
    char buf[75000] = {}; //临时存储
    std::string str;
    int pocket_num,pocket_size,pocket_id,pointSize;
    int recvTimes = 1;
    double cloud_time, odom_time;
    while(ros::ok())
    {
        r = recv(clientSocket,buf,pocketMinSize,0);
        // printf("r:%d\n",r);
        if(r>0)
        {
            recv_sign = true;
            str.append(buf, r);
            // cout<<"str"<<str.length()<<endl;

            while ( str.length()>= pocketMinSize && 0x55 == (unsigned char)str.at(2) && 0xaa == (unsigned char)str.at(3) && 0x55 == (unsigned char)str.at(6) 
            && 0xaa == (unsigned char)str.at(7) || pocket_id == pocket_num && 0x55 == (unsigned char)str.at(2) && 0xaa == (unsigned char)str.at(3) && str.length()>=pocket_size &&!(recvTimes==1))
                {
                    //提取包的大小参数
                    std::string pocketRealsize;
                    pocketRealsize = str.substr(0,100);  //前100个字节
                    char *head_buf;
                    head_buf = (char*)pocketRealsize.c_str();
                    char *ptr1, *ptr2, *ptr3, *ptr4;
                    int p1, p2, p3, p4;

                    //计算包id
                    ptr1 = head_buf + 8;   //第4个字节（前4个是头id）
                    ptr2 = head_buf + 9;
                    ptr3 = head_buf + 10;
                    ptr4 = head_buf + 11;
                    p1 = (unsigned char)*ptr1 << 24;
                    p2 = (unsigned char)*ptr2 << 16;
                    p3 = (unsigned char)*ptr3 << 8;
                    p4 = (unsigned char)*ptr4;
                    pocket_id = p1 | p2 | p3 | p4;
                    // printf("pocket_id:%d\n",pocket_id);

                    //计算包总数
                    ptr1 = head_buf + 12;
                    ptr2 = head_buf + 13;
                    ptr3 = head_buf + 14;
                    ptr4 = head_buf + 15;
                    p1 = (unsigned char)*ptr1 << 24;
                    p2 = (unsigned char)*ptr2 << 16;
                    p3 = (unsigned char)*ptr3 << 8;
                    p4 = (unsigned char)*ptr4;
                    pocket_num = p1 | p2 | p3 | p4;
                    // printf("pocket_num:%d\n",pocket_num);

                    //计算包内点云总数
                    ptr1 = head_buf + 16;
                    ptr2 = head_buf + 17;
                    ptr3 = head_buf + 18;
                    ptr4 = head_buf + 19;
                    p1 = (unsigned char)*ptr1 << 24;
                    p2 = (unsigned char)*ptr2 << 16;
                    p3 = (unsigned char)*ptr3 << 8;
                    p4 = (unsigned char)*ptr4;
                    pocket_size = p1 | p2 | p3 | p4;
                    // printf("cloudpoint_num:%d\n",pocket_size);

                    //点云时间
                    char thr[8];
                    thr[0] = head_buf[20];
                    thr[1] = head_buf[21];
                    thr[2] = head_buf[22];
                    thr[3] = head_buf[23];
                    thr[4] = head_buf[24];
                    thr[5] = head_buf[25];
                    thr[6] = head_buf[26];
                    thr[7] = head_buf[27];
                    memcpy(&cloud_time, thr, sizeof(double));
                    //odom时间
                    thr[0] = head_buf[28];
                    thr[1] = head_buf[29];
                    thr[2] = head_buf[30];
                    thr[3] = head_buf[31];
                    thr[4] = head_buf[32];
                    thr[5] = head_buf[33];
                    thr[6] = head_buf[34];
                    thr[7] = head_buf[35];
                    memcpy(&odom_time, thr, sizeof(double));
                    // printf(",cloudtime:%f,odomtime:%f",cloud_time,odom_time);

                    //odom
                    char odom_ch[64];
                    for(int j=0; j<64; j++)
                    {
                        odom_ch[j]=head_buf[36+j];
                    }
                    memcpy(&odom_matrix,odom_ch,sizeof(odom_ch));

                    //提取包
                    std::string str_pocket;
                    char *recv_buf;
                    // std::cout<<str.size()<<std::endl;
                    str_pocket = str.substr(0, pocket_size*16+104);
                    // std::cout<<"str_pocket_size:"<<str_pocket.size()<<std::endl;
                    //重置
                    if(pocket_id==1)
                    {
                        points.points.resize(pocket_size);
                        pointSize = 0;
                    }

                    pointSize += pocket_size;

                    if(pocket_id==pocket_num)
                    {
                        recv_buf = (char*)str.c_str();
                        // cout<<"第"<<recvTimes<<"次收到，共"<<pocket_id<<"个包,size:"<<pointSize;
                        // printf(",cloudtime:%f,odomtime:%f\n",cloud_time,odom_time);
                        recvTimes++;
                        pocket_id = -1; //重置下
                        pocket_num = -2;

                    }
                    else{
                        recv_buf = (char*)str_pocket.c_str();
                    }
                    str.erase(0, pocket_size*16+104);
                    // cout<<"dec:"<<pocket_size*16+104<<endl;

                    char *point_data = recv_buf + 100;

                    //遍历包内点云
                    for(int i = 0; i< pocket_size; i++)
                    {
                        //初始化点云变量
                        pcl::PointXYZI p;
                        float x,y,z,intensity;
                        char ch[4];
                        ch[0] = point_data[0 + (i * 4 * sizeof(float))] & 0xff;
                        ch[1] = point_data[1 + (i * 4 * sizeof(float))] & 0xff;
                        ch[2] = point_data[2 + (i * 4 * sizeof(float))] & 0xff;
                        ch[3] = point_data[3 + (i * 4 * sizeof(float))] & 0xff;
                        memcpy(&x,ch,sizeof(float));

                        ch[0] = point_data[4 + (i * 4 * sizeof(float))];
                        ch[1] = point_data[5 + (i * 4 * sizeof(float))];
                        ch[2] = point_data[6 + (i * 4 * sizeof(float))];
                        ch[3] = point_data[7 + (i * 4 * sizeof(float))];
                        memcpy(&y, ch, sizeof(float));

                        ch[0] = point_data[8  + (i * 4 * sizeof(float))];
                        ch[1] = point_data[9  + (i * 4 * sizeof(float))];
                        ch[2] = point_data[10 + (i * 4 * sizeof(float))];
                        ch[3] = point_data[11 + (i * 4 * sizeof(float))];
                        memcpy(&z, ch, sizeof(float));

                        ch[0] = point_data[12  + (i * 4 * sizeof(float))];
                        ch[1] = point_data[13  + (i * 4 * sizeof(float))];
                        ch[2] = point_data[14 + (i * 4 * sizeof(float))];
                        ch[3] = point_data[15 + (i * 4 * sizeof(float))];
                        memcpy(&intensity, ch, sizeof(float));

                        //保存点云
                        p.x = x;
                        p.y = y;
                        p.z = z;
                        p.intensity = intensity;

                        points.push_back(p);
                    }

                    //lidar
                    pcl::toROSMsg(points,output_msg);
                    output_msg.header.frame_id = "camera_init2";
                    output_msg.header.stamp = ros::Time().fromSec(cloud_time);
                    pubRecvdata.publish(output_msg);

                    //odom
                    odom.header.frame_id = "camera_init2";
                    odom.child_frame_id = "cloud_registered";
                    odom.header.stamp = ros::Time().fromSec(odom_time);
                    //set the position
                    odom.pose.pose.position.x = odom_matrix(0,3);
                    odom.pose.pose.position.y = odom_matrix(1,3);
                    odom.pose.pose.position.z = odom_matrix(2,3);
                    Eigen::Quaternionf q;
                    q = odom_matrix.block<3,3>(0,0);
                    odom.pose.pose.orientation.x = q.x();
                    odom.pose.pose.orientation.y = q.y();
                    odom.pose.pose.orientation.z = q.z();
                    odom.pose.pose.orientation.w = q.w();
                    pubOdom.publish(odom);
                    // printf("x:%f\n",odom.pose.pose.position.x);


                    //tf
                    static tf2_ros::TransformBroadcaster broadcaster;
                    geometry_msgs::TransformStamped tfs;
                    tfs.header.frame_id = "camera_init2";
                    tfs.header.stamp = ros::Time().fromSec(odom_time);
                    tfs.child_frame_id = "cloud_registered";
                    tfs.transform.translation.x = odom_matrix(0,3);
                    tfs.transform.translation.y = odom_matrix(1,3);
                    tfs.transform.translation.z = odom_matrix(2,3);
                    tfs.transform.rotation.x = q.x();
                    tfs.transform.rotation.y = q.y();
                    tfs.transform.rotation.z = q.z();
                    tfs.transform.rotation.w = q.w();
                    broadcaster.sendTransform(tfs);

                }
        }
        else
        {
            break;
        }
    }

    //7.关闭连接
    close(serverSocket);
    close(clientSocket);
    printf("关闭连接！\n");
    
    return 0;

}

