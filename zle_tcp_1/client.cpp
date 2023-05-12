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

int clientSocket;
bool send_sign;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
int pocket_number;


void cloud_send(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZI> inMap;
    while(1)
    {
        // usleep(80);
        if (send_sign == true)
        {
            send_sign = false;
            if((*pointcloud_ptr).size()>0)
            {
                inMap = *pointcloud_ptr;
            }
            std::string buf,str;
            pcl::PointCloud<pcl::PointXYZI> map;

            if(inMap.size()>10)
            {
                map = inMap;
                inMap.clear();
                int pocket = 5;
                int left_num = map.height * map.width;
                int it_num,start_index,end_index;
                int pocket_id=0,pocket_num,pocket_size;


                //分包，使每个包的点数小于2000
                if(left_num > 50){
                    do{
                        it_num = left_num / pocket;
                        if(it_num > 2000)
                        {
                            pocket++;
                        }
                    }while(it_num > 2000);

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
                        for(int j = start_index; j< end_index; j++)   //一个包的所有点
                        {
                            char ch[4];
                            memcpy(ch,&map.points.at(j).x,sizeof(float));
                            buf.push_back(ch[0]);
                            buf.push_back(ch[1]);
                            buf.push_back(ch[2]);
                            buf.push_back(ch[3]);
                            printf("p.x:%f\n",map.points.at(j).x);

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
                        // int pocket_id,pocket_num,pocket_size;
                        int be32;

                        head_1 = 0x55aa;
                        head_2 = 0x55aa;
                        // pocket_id = i + 1;    //包id定义为每个包的起始点云的次序
                        pocket_id++;
                        pocket_num = pocket + 1;
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
                        // printf("pocket_size:%d\n",pocket_size);
                        str.append(reinterpret_cast<char *>(&be32),sizeof(be32));
                        //data
                        str.append(buf);
                        //end
                        be32 = htonl(end);
                        str.append(reinterpret_cast<char *>(&be32),sizeof(be32));
                        send(clientSocket,str.data(),str.size(),0);
                        // printf("str_size:%d\n",str.size());
                        buf.clear();
                        str.clear();
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


int main(){
    std::string pcd_path = "/home/zle/Documents/pointcloud_tcp/zle_tcp_1/pcd/scans.pcd";
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path,*cloud_ptr)==-1)
    {
        std::cerr<<"can't load point cloud file"<<std::endl;
        return -1;
    }
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

    //4.发送数据
    boost::timer t;
    send_sign = true;
    cloud_send(cloud_ptr);

    //5.关闭发送
    sleep(0.5);
    close(clientSocket);
    printf("数据发送完毕！\n");
    printf("发送包的个数：%d\n",pocket_number);
    printf("发送所用时间：%lfs\n",t.elapsed());

    return 0;
}

