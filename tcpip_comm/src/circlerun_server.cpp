#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <vector>

using namespace std;

void error(const char *msg) {
    perror(msg);
    exit(1);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "circlerun_server_node");
  bool connection_made = false;

  ros::NodeHandle circlerun_ball_acc_x_node;
  ros::Publisher circlerun_ball_acc_x_pub = circlerun_ball_acc_x_node.advertise<std_msgs::Float64>("/circlerun_ball_acc_x/", 1);
  ros::NodeHandle circlerun_ball_acc_y_node;
  ros::Publisher circlerun_ball_acc_y_pub = circlerun_ball_acc_y_node.advertise<std_msgs::Float64>("/circlerun_ball_acc_y/", 1);
  ros::NodeHandle circlerun_ball_acc_z_node;
  ros::Publisher circlerun_ball_acc_z_pub = circlerun_ball_acc_z_node.advertise<std_msgs::Float64>("/circlerun_ball_acc_z/", 1);

  ros::NodeHandle circlerun_ball_x_node;
  ros::Publisher circlerun_ball_x_pub = circlerun_ball_x_node.advertise<std_msgs::Float64>("/circlerun_ball_x/", 1);
  ros::NodeHandle circlerun_ball_y_node;
  ros::Publisher circlerun_ball_y_pub = circlerun_ball_y_node.advertise<std_msgs::Float64>("/circlerun_ball_y/", 1);

  ros::NodeHandle circlerun_ball_state_node;
  ros::Publisher circlerun_ball_state_pub = circlerun_ball_state_node.advertise<std_msgs::Float64MultiArray>("/circlerun_ball_state/", 1);

  vector<std_msgs::Float64> ball(5);

  int sockfd, newsockfd, portno; //Socket file descriptors and port number
  socklen_t clilen; //object clilen of type socklen_t
  char buffer[256]; //buffer array of size 256
  struct sockaddr_in serv_addr, cli_addr; ///two objects to store client and server address
  std::stringstream ss;
  int n;
  ros::Duration d(0.01); // 100Hz
  if (argc < 2) {
    fprintf(stderr,"ERROR, no port provided\n");
    exit(1);
  }
  portno = atoi(argv[1]);
  cout << "The node is listening on port " << portno << " for incoming connections" << endl;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
      error("ERROR opening socket");
  int enable = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
      error("setsockopt(SO_REUSEADDR) failed");
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);
  if (bind(sockfd, (struct sockaddr *) &serv_addr,
            sizeof(serv_addr)) < 0)
            error("ERROR on binding");
  listen(sockfd,5);
  clilen = sizeof(cli_addr);
  newsockfd = accept(sockfd,
              (struct sockaddr *) &cli_addr,
              &clilen);
  if (newsockfd < 0)
       error("ERROR on accept");
  while(ros::ok())
  {
    ss.str(std::string()); //Clear contents of string stream
    bzero(buffer,256);
    n = read(newsockfd,buffer,255);
    if (n < 0)
      error("ERROR reading from socket");
    else if (n == 0)
    {
      close(newsockfd);
      close(sockfd);
      cout << "connection lost" << endl;
      connection_made = false;
      cout << "establishing a new connection" << endl;
      sockfd = socket(AF_INET, SOCK_STREAM, 0);
      if (sockfd < 0)
          error("ERROR opening socket");
      int enable = 1;
      if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
          error("setsockopt(SO_REUSEADDR) failed");
      bzero((char *) &serv_addr, sizeof(serv_addr));
      serv_addr.sin_family = AF_INET;
      serv_addr.sin_addr.s_addr = INADDR_ANY;
      serv_addr.sin_port = htons(portno);
      if (bind(sockfd, (struct sockaddr *) &serv_addr,
                sizeof(serv_addr)) < 0)
                error("ERROR on binding");
      listen(sockfd,5);
      clilen = sizeof(cli_addr);
      newsockfd = accept(sockfd,
                  (struct sockaddr *) &cli_addr,
                  &clilen);
      if (newsockfd < 0)
           error("ERROR on accept");
    }
    else
    {
      if(connection_made == false)
      {
        connection_made = true;
        cout << "connection is made!" << endl;
      }
      ss << buffer;
      string datain = ss.str();

      int p = datain.find("DATA");
      if(p != string::npos)
        datain = datain.substr(p+5, string::npos);

      for(int i = 0; i < 5; i++)
      {
        p = datain.find(" ");
        if(p != string::npos)
        {
          //cout << datain.substr(0, p).c_str() << endl;
          ball[i].data= atof(datain.substr(0, p).c_str());
          datain = datain.substr(p+1, string::npos);
        }
      }
      ball[0].data *= -1;
      ball[3].data *= -1;
      circlerun_ball_acc_x_pub.publish(ball[1]);
      circlerun_ball_acc_y_pub.publish(ball[0]);
      circlerun_ball_acc_z_pub.publish(ball[2]);
      circlerun_ball_x_pub.publish(ball[4]);
      circlerun_ball_y_pub.publish(ball[3]);

      std_msgs::Float64MultiArray array;
      array.data.resize(4);
      array.data[0]=ball[4].data;
      array.data[1]=ball[3].data;
      array.data[2]=ball[1].data;
      array.data[3]=ball[0].data;
      circlerun_ball_state_pub.publish(array);
    }
    usleep(10000);
  }
  return 0;
}


//n = write(newsockfd,"I got your message",18);
//if (n < 0) error("ERROR writing to socket");
//close(newsockfd);
//close(sockfd);
//ros::spinOnce();
//d.sleep();
