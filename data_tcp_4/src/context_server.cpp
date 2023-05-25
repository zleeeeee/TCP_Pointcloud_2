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
int pocketMinSize = 7000;//要大于最大的pocket_size
int n_row=20,n_col=80;
Eigen::MatrixXf recv_Matrix(n_row,n_col);
void *send_thread(void *arg);
bool recv_sign = false;



void *send_thread(void *arg)
{
	int client = *(int*)arg;
    int sendTimes = 1;
    while(ros::ok() && client!=-1)
    {
        sleep(1);
        Eigen::MatrixXf context_mat = Eigen::MatrixXf::Ones(n_row,n_col);
        double context_time = 123456;
        string buff;
        unsigned head_1,head_2,end;
        int be32;
        head_1 = 0x55aa;
        head_2 = 0x55aa;
        end = 0xaa55;
        //header
        be32 = htonl(head_1);    //主机转网络字节，转换为32位,四个字节
        buff.append(reinterpret_cast<char *>(&be32),sizeof(be32));
        be32 = htonl(head_2);
        buff.append(reinterpret_cast<char *>(&be32),sizeof(be32));
        //data
        char tth[8];
        memcpy(tth,&context_time,sizeof(double));
        buff.push_back(tth[0]);
        buff.push_back(tth[1]);
        buff.push_back(tth[2]);
        buff.push_back(tth[3]);
        buff.push_back(tth[4]);
        buff.push_back(tth[5]);
        buff.push_back(tth[6]);
        buff.push_back(tth[7]);  

        char  ah[4];
        float ma;
        for(int i=0;i<n_row;++i)
        {
            for(int j=0;j<n_col;++j)
            {
                ma=context_mat(i,j);
                memcpy(ah,&ma,sizeof(float));
                buff.push_back(ah[0]);
                buff.push_back(ah[1]);
                buff.push_back(ah[2]);
                buff.push_back(ah[3]);
            }
        }
        //预留标志位
        int a =1;
        char ch[4];
        memcpy(ch,&a,sizeof(int));
        buff.push_back(ch[0]);
        buff.push_back(ch[1]);
        buff.push_back(ch[2]);
        buff.push_back(ch[3]);

        //end
        be32 = htonl(end);
        buff.append(reinterpret_cast<char *>(&be32),sizeof(be32));
        send(client,buff.data(),buff.size(),0);
        cout<<"第"<<sendTimes<<"次发送"<<endl;
        sendTimes++;
        // cout<<"context size:"<<buff.size()<<endl;
        buff.clear();
    }
    close(client);

}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"context_server");
    ros::NodeHandle n; 
    // ros::Publisher pubRecvdata = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points_recv", 5);

    //1. 创建socket  
	serverSocket = socket(AF_INET,SOCK_STREAM,0);
	if(-1 == serverSocket) printf("创建socket失败:%m\n"),exit(-1);
	printf("创建socket成功!\n");
 
	//2. 创建服务器协议地址簇
	struct sockaddr_in sAddr = {0};
	sAddr.sin_family = AF_INET; 
	sAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
	sAddr.sin_port = htons(9530);  
 
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
    pthread_t t_send;
	pthread_create(&t_send,NULL,send_thread,(void*)&clientSocket);
	pthread_detach(t_send);

    char buf[15000] = {}; //临时存储
    std::string str;
    int recvTimes = 1;
    double scan_time;
    while(ros::ok())
    {
        r = recv(clientSocket,buf,pocketMinSize,0);
        // printf("r:%d\n",r);
        if(r>0)
        {
            str.append(buf, r);
            // cout<<"str"<<str.length()<<endl;

            while ( str.length()>= pocketMinSize && 0x55 == (unsigned char)str.at(2) && 0xaa == (unsigned char)str.at(3) && 0x55 == (unsigned char)str.at(6) 
            && 0xaa == (unsigned char)str.at(7))
                {
                    cout<<"receive_times:"<<recvTimes<<endl;
                    recvTimes++;
                    //提取包的大小参数
                    std::string pocketRealsize;
                    pocketRealsize = str.substr(0,n_col*n_row+8+3*4+4);
                    char *head_buf;
                    head_buf = (char*)pocketRealsize.c_str();
                    
                    //时间
                    char thr[8];
                    thr[0] = head_buf[8];
                    thr[1] = head_buf[9];
                    thr[2] = head_buf[10];
                    thr[3] = head_buf[11];
                    thr[4] = head_buf[12];
                    thr[5] = head_buf[13];
                    thr[6] = head_buf[14];
                    thr[7] = head_buf[15];
                    memcpy(&scan_time, thr, sizeof(double));

                    //context
                    char context_ch[n_col*n_row*4];
                    for(int j=0; j<n_col*n_row*4; j++)
                    {
                        context_ch[j]=head_buf[16+j];
                    }
                    char  ch[4];
                    float mb;
                    for(int i=0;i<n_row;++i)
                    {
                        for(int j=0;j<n_col;++j)
                        {
                            ch[0]=context_ch[(i*n_col+j)*4+0];
                            ch[1]=context_ch[(i*n_col+j)*4+1];
                            ch[2]=context_ch[(i*n_col+j)*4+2];
                            ch[3]=context_ch[(i*n_col+j)*4+3];
                            memcpy(&mb, ch,sizeof(float));
                            recv_Matrix(i,j)=mb;
                        }
                    }
                    // cout<<recv_Matrix<<endl;
                    //预留
                    char aah[4];
                    int recv_a;
                    aah[0] = head_buf[n_col*n_row*4+16];
                    aah[1] = head_buf[n_col*n_row*4+17];
                    aah[2] = head_buf[n_col*n_row*4+18];
                    aah[3] = head_buf[n_col*n_row*4+19];
                    memcpy(&recv_a, aah, sizeof(int));

                    str.erase(0, n_col*n_row*4+24);

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

