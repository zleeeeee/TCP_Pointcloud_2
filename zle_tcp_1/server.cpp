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

int serverSocket, clientSocket;
int pocketMinSize = 32024;
pcl::PointCloud<pcl::PointXYZI> points;


void cloud_receive(int& r, int& ServerSocket, int& ClientSocket)
{
    char buf[65000] = {}; //临时存储
    std::string str;
    int pocket_num,pocket_size,pocket_id;

    while(1){
        r = recv(clientSocket,buf,pocketMinSize,0);
        // printf("r:%d\n",r);
        if(r>0)
        {
            str.append(buf, r);
            // std::cout<<int((unsigned char)str.at(2))<<int((unsigned char)str.at(3))<<std::endl;

            if ( str.length()>= pocketMinSize && 0x55 == (unsigned char)str.at(2) && 0xaa == (unsigned char)str.at(3) && 0x55 == (unsigned char)str.at(6) 
            && 0xaa == (unsigned char)str.at(7) || pocket_id+1 == pocket_num && 0x55 == (unsigned char)str.at(2) && 0xaa == (unsigned char)str.at(3))
                {
                    //提取包的大小参数
                    std::string pocketRealsize;
                    pocketRealsize = str.substr(0,20);
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
                    printf("pocket_id:%d\n",pocket_id);

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
                    printf("pocket_size:%d\n",pocket_size);

                    //提取包
                    std::string str_pocket;
                    char *recv_buf;
                    // std::cout<<str.size()<<std::endl;
                    str_pocket = str.substr(0, pocket_size*16+24);
                    // std::cout<<"str_pocket_size:"<<str_pocket.size()<<std::endl;
                    if(pocket_id+1==pocket_num)
                    {
                        recv_buf = (char*)str.c_str();
                        printf("接收包的个数：%d\n",pocket_id+1);
                    }
                    else{
                        recv_buf = (char*)str_pocket.c_str();
                    }
                    str.erase(0, pocket_size*16+24);

                    char *point_data = recv_buf + 5 * sizeof(int);

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
                        printf("p.x:%f\n",x);

                        points.push_back(p);
                    }
                }
        }
        else
        {
            break;
        }
    }
}

int main(){
    //1. 创建socket  
	serverSocket = socket(AF_INET,SOCK_STREAM,0);
	if(-1 == serverSocket) printf("创建socket失败:%m\n"),exit(-1);
	printf("创建socket成功!\n");
 
	//2. 创建服务器协议地址簇
	struct sockaddr_in sAddr = {0};
	sAddr.sin_family = AF_INET; 
	sAddr.sin_addr.s_addr = inet_addr("10.42.0.1"); 
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

    //6.通信
    cloud_receive(r,serverSocket,clientSocket);

    //7.关闭连接
    close(serverSocket);
    close(clientSocket);
    printf("关闭连接！\n");

    //8.保存点云
    printf("保存点云!\n");
    if(pcl::io::savePCDFile<pcl::PointXYZI>("/home/zle/Documents/pointcloud_tcp/zle_tcp_1/pcd/cloud_receive.pcd",points) == -1)
    {
        printf("点云保存失败！\n");
    }
    else{
        printf("点云保存成功！\n");
    }

    return 0;
}