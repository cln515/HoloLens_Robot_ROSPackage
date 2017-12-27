/*
Copyright (c) 2017 Ryoichi Ishikawa. All rights reserved.

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/


#include "HololensManager.h"




  
  
  namespace hololensManager
  {
    //initialize
    hololensNode::hololensNode(char* ip)
    {
      int TCP_PORT=1234;
      sock= socket(AF_INET, SOCK_STREAM,0);
      holo.sin_addr.s_addr = inet_addr(ip);
      holo.sin_family = AF_INET;
      holo.sin_port=htons(TCP_PORT);
      initx=0;
      inity=0;
      yaw=0;
      scale=12.0/512;//scale of received image from HoloLens
    }

    //send message to HoloLens
    void hololensNode::sendMessage(unsigned int val){
                unsigned int n=val;
                char* buffer,buffer_[4];
                buffer=(char*)&n;
                buffer_[0]=buffer[3];      buffer_[1]=buffer[2];      buffer_[2]=buffer[1];      buffer_[3]=buffer[0];
                send(sock,buffer_,sizeof(n),0);
    }

    //receive message from HoloLens
    void hololensNode::recvMessage(unsigned int size,char* pointer){
	  int total=0;
	  while(total<size){
	    int n=recv(sock,pointer,size-total,0);
	    total+=n;
	    pointer+=n;
	  }
	}
    
    //projecting 3D pose -> 2D pose on floor, 
    void hololensNode::dimensionRemover(tf::StampedTransform transform, double& _x,double& _y, double& _yaw,tf::Quaternion& _initq){
	    tf::Quaternion q=transform.getRotation();
	    tf::Quaternion dirq(0,0,-1,0);
	    
	    dirq=q*dirq*q.inverse();
	    
	    double qz=dirq.getX();
	    double qw=dirq.getY();double rad=sqrt(qz*qz+qw*qw);
	    qz=qz/rad;
	    qw=qw/rad;
	    
	    double theta=acos(qz);
	    if(qw<0)theta=-theta;
	    _initq=tf::Quaternion(0,0,sin((theta)/2),cos((theta)/2));
	    
	    tf::Matrix3x3 mat(_initq);
	    tfScalar yaw_, pitch, roll;
	    mat.getEulerYPR(yaw_, pitch, roll, 1);
	    _yaw=yaw_;
	    
	    double px=(256)*cos(yaw)-(256)*sin(yaw);
	    double py=(256)*sin(yaw)+(256)*cos(yaw);
	    
	    _x=transform.getOrigin().getX()-px*scale;
	    _y=transform.getOrigin().getY()-py*scale;
	    
	    
}
    
    //HoloLens Initial Pose callback
    void hololensNode::callbackInitPoseStamped(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initPose){
      cout<<"pose subscrived!!"<<endl;  
      cout<<initPose->pose.pose.position<<endl;
      cout<<initPose->pose.pose.orientation<<endl;
      bPoseSubscribed=true;
      transLookUp.clear();
      initq=tf::Quaternion(
	initPose->pose.pose.orientation.x,
	initPose->pose.pose.orientation.y,
	initPose->pose.pose.orientation.z,
	initPose->pose.pose.orientation.w);      
      tfScalar yaw_, pitch, roll;
      tf::Matrix3x3 mat(initq);
      mat.getEulerYPR(yaw_, pitch, roll, 1);
      yaw=yaw_;
      
      double px=(256)*cos(yaw)-(256)*sin(yaw);
      double py=(256)*sin(yaw)+(256)*cos(yaw);
      initx=initPose->pose.pose.position.x-px*scale;
      inity=initPose->pose.pose.position.y-py*scale;
      sendMessage(CMD_IMG_REQ);
    }
    


    //continuously receiving Depth map 
    void hololensNode::reqHoloMap(const std_msgs::Bool::ConstPtr& call){
      if(call->data){
        sendMessage(DEPTH_STREAM_REQ);
      }else{
	sendMessage(DEPTH_STREAM_OFF_REQ);
      }
    }
    
  
    //get floormap and create rtree
    void hololensNode::callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& subscribedmap){
      if(rtree.size()>0)return;
      int width=subscribedmap->info.width;
      int height=subscribedmap->info.height;
      double scale=subscribedmap->info.resolution;
      unsigned int cnt=0;
      for(int i=0;i<width;i++){
	    for(int j=0;j<height;j++){
	      if(subscribedmap->data[i+j*width]>50){
		point p=point(i*scale+subscribedmap->info.origin.position.x,j*scale+subscribedmap->info.origin.position.y);
		rtree.insert(std::make_pair(p,++cnt));
	      }
	    }
	  }
      cout<<"tree size"<<cnt<<endl;
    }
    
    //initialize
    void hololensNode::start()
    {
      //set publisher
      obst_pub_=nh.advertise<sensor_msgs::PointCloud>("/holo/map", 1000);
      holoGrid_pub_=nh.advertise<nav_msgs::OccupancyGrid>("/holo/vmap", 1000);
      holomapviz_pub_= nh.advertise<nav_msgs::OccupancyGrid>("/holo/vizmap", 1000);
      floor2holo_pub_= nh.advertise<geometry_msgs::PoseStamped>("/holo/floor", 1000);
      pos_lost_pub_= nh.advertise<std_msgs::Bool>("/holo/position_lost", 1000);
      
      //set subscriber
      initScriber= nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/holo/initposestamped", 1, &hololensNode::callbackInitPoseStamped, this);
      mapSubscriber=nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &hololensNode::callbackMap, this);
      switchCallSub_=nh.subscribe<std_msgs::Bool>("/holo/modeswitch", 1, &hololensNode::reqHoloMap, this);

      //set tf
      tf_anchor_to_hololens_.frame_id_ = std::string("spatialAnchor");
      tf_anchor_to_hololens_.child_frame_id_ = std::string("hololens");
      tf_originmap_to_holomap.frame_id_=std::string("map");
      tf_originmap_to_holomap.child_frame_id_=std::string("localmap");
      	  tf_map_to_anchor_.setOrigin(tf::Vector3(0, 0, 0));
	  tf::Matrix3x3 rot(1.0,0,0,
			0,1.0,0,
			0,0,1.0);
	  tf::Quaternion q;
	  rot.getRotation(q);
	  tf_map_to_anchor_.setRotation(q);
      
      tf_map_to_anchor_.frame_id_=std::string("map");
      tf_map_to_anchor_.child_frame_id_=std::string("spatialAnchor");
      
      //connection
      connect(sock,(struct sockaddr *)&holo, sizeof(holo));
      cout<<"Connection"<<endl;          
      run();

    }
    
    //receive HoloLens pose and position in Spatial Anchor coordinates
    void hololensNode::recvPosition(){
	  char* pbuf;
	    float posdata[16];
	  pbuf=(char*)posdata;
	  unsigned int recvsize=sizeof(float)*16;
	  recvMessage(recvsize,pbuf);
	  tf_anchor_to_hololens_.setOrigin(tf::Vector3(posdata[12], posdata[13], posdata[14]));
	  tf::Matrix3x3 rot(posdata[0],posdata[4],posdata[8],
			posdata[1],posdata[5],posdata[9],
			posdata[2],posdata[6],posdata[10]);
	  tf::Quaternion q;
	  rot.getRotation(q);
	  tf_anchor_to_hololens_.setRotation(q);
	  tf_anchor_to_hololens_.stamp_=ros::Time::now();
	  if(bSpatialAnchorUpdate){
		tf_map_to_anchor_.stamp_=tf_anchor_to_hololens_.stamp_;
		tf_br_.sendTransform(tf_map_to_anchor_);
		bSpatialAnchorUpdate=false;
		}
	  tf_br_.sendTransform(tf_anchor_to_hololens_);	
	  ros::spinOnce();
}

	//receive rendered map from top view
	void hololensNode::recvMap(cv::Mat& dst,double upper,double lower){
		char buffer[4];
		char buffer_[4];
		char* pbuf;
		pbuf=buffer;
          recvMessage(4,pbuf);
	  buffer_[0]=buffer[3];      buffer_[1]=buffer[2];      buffer_[2]=buffer[1];      buffer_[3]=buffer[0];
	  unsigned int rowpitch=((unsigned int*)buffer_)[0];
	  cout<<rowpitch<<endl;
	  pbuf=buffer;
	  recvMessage(4,pbuf);
	  buffer_[0]=buffer[3];      buffer_[1]=buffer[2];      buffer_[2]=buffer[1];      buffer_[3]=buffer[0];
	  unsigned int height=((unsigned int*)buffer_)[0];
	  float* imageData=(float*)malloc(height*rowpitch);
	  pbuf=(char *)imageData;
	  cout<<height<<endl;
	  bool positionLost=false;
	  recvMessage(rowpitch*height,pbuf);	
	  cv::Mat m=cv::Mat(height,rowpitch/4,CV_8UC1);//get wall data
	  for(int i=0;i<height*rowpitch/4;i++){
	    m.data[i]=(imageData[i]>lower&&imageData[i]<upper)?20:100;	    
	  }
	    cv::Canny(m,dst,25,75,3);
	    cv::flip(dst,dst,1);
	  free(imageData);
	}
	
	//registration
	bool hololensNode::registration(cv::Mat dst,int iteration_start,int iteration_time){
	 bool positionLost=false;
	 for(int itr=iteration_start;itr<iteration_time;itr++){
	    vector<cv::Point2f> querypoints;
	    int rad=128+128;
	    for(int i=0;i<dst.cols;i++){
	      for(int j=0;j<dst.rows;j++){
		if(dst.data[i+j*dst.cols]>200&&(i-256)*(i-256)+(j-256)*(i-256)<rad*rad){
		  float px,py;
		  px=(i)*cos(yaw)-(j)*sin(yaw);
		  px*=scale;
		  py=(i)*sin(yaw)+(j)*cos(yaw);
		  py*=scale;
		  querypoints.push_back(cv::Point2f(px+initx,py+inity));
		}
	      }
	    }
	    for(int i=0;i<dst.cols*dst.rows;i++){grid.data[i]=dst.data[i]==255?100:0;}
	    vector<int> idx;
	    vector<cv::Point2f> dstpoints;
	    if(querypoints.size()<500){
		if(bPositionTracked)cout<<"data shortage!!"<<endl;
		positionLost=true;
	      break;
	    }
	    
	    for(int i=0;i<querypoints.size();){
	      vector<value> result_n;
	      rtree.query(bgi::nearest(point(querypoints[i].x,querypoints[i].y),1),std::back_inserter(result_n));
	      float x = result_n[0].first.get<0>();
	      float y = result_n[0].first.get<1>();
	      
	      if((x-querypoints[i].x)*(x-querypoints[i].x)+(y-querypoints[i].y)*(y-querypoints[i].y)>2.0/(itr+1)){
		querypoints.erase(querypoints.begin()+i);
		continue;
	      }
	      dstpoints.push_back(cv::Point2f(x,y));
	      i++;
	    }
	    Eigen::Matrix2d R;
	    Eigen::Vector2d T;
	    linearCompute(querypoints,dstpoints,R,T);
	    Eigen::Matrix2d Ryaw;
	    Ryaw<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
	    double new_originx=R(0,0)*initx+R(0,1)*inity,
	      new_originy=R(1,0)*initx+R(1,1)*inity;
	    R=R*Ryaw;
	    yaw=acos(R(0,0));
	    if(R(0,1)>0)yaw=-yaw;
	    tf::Matrix3x3 mat(R(0,0),R(0,1),0,
				R(1,0),R(1,1),0,
				0,0,1
			    );
	    mat.getRotation(initq);
	    initx=new_originx+T(0);
	    inity=new_originy+T(1); 
	  }
	  return positionLost;
	}
    
    
    //main roop
    void hololensNode::run()
    {
      ros::Rate loop_rate(100);

       while (ros::ok()){
	char buffer[4];
	char* pbuf=buffer;
	recvMessage(4,pbuf);
	char buffer_[4];
	buffer_[0]=buffer[3];      buffer_[1]=buffer[2];      buffer_[2]=buffer[1];      buffer_[3]=buffer[0];
	unsigned int headernum=((unsigned int*)buffer_)[0];
	if(headernum==HEADER_CAMERA){
		if(!bPositionTracked){
			bPositionTracked=true;
			std_msgs::Bool posLost;
			posLost.data=false;
			pos_lost_pub_.publish(posLost);
		}
		//receive sensor position
	  recvPosition();
	}
	else if(headernum==HEADER_IMAGE){
		//spatial anchor localization
		pbuf=buffer;
		//receive anchor name length
	  recvMessage(4,pbuf);
	  buffer_[0]=buffer[3];      buffer_[1]=buffer[2];      buffer_[2]=buffer[1];      buffer_[3]=buffer[0];
	  unsigned int strLength=((unsigned int*)buffer_)[0];
	  char* strBuf=(char*)malloc(sizeof(char)*strLength+1);

	  //string length
	  pbuf=(char*)strBuf;
	  recvMessage(sizeof(char)*strLength,pbuf);
	  strBuf[sizeof(char)*strLength]=0;
	  string str(strBuf);
	  free(strBuf);

	  //check receive map including enough information for alignment
	  bool isSendAck=false;

	  
	  if(!bPoseSubscribed){
		  //get current hololens position & direction
	    tf::StampedTransform transform;
	    try{
	    listener.lookupTransform(std::string("map"),std::string("hololens"),
				   ros::Time(0),transform);
	    }catch (exception e){
	      
	    }
	    dimensionRemover(transform,initx,inity,yaw,initq);
	    isSendAck=true;
	  }else bPoseSubscribed=false;
	  cv::Mat dst;
	   pbuf=buffer;
	   
	   //receive map and detect edge
	 recvMap(dst,0.17, 0.0);
	 grid.data.resize(dst.cols*dst.rows);  
	 
	 int iteration_time=30;
	 //initial: Itaretion times, 30 times
	 //other: 10 times
	 int iteration_start=bPoseSubscribed?0:20;
	 bool positionLost=registration(dst,iteration_start,iteration_time);
//	  cout<<"(x,y,yaw) ("<<initx<<","<<inity<<","<<yaw<<")"<<endl;
	  if(!positionLost){
		  //publish map and publish anchor position
	    nav_msgs::MapMetaData info;
	    std_msgs::Header header;
	    //header.frame_id="localmap";
	    info.resolution =scale;
	    info.map_load_time=ros::Time();
	    info.width=dst.cols;
	    info.height=dst.rows;
	    info.origin.orientation.x=initq.x();
	    info.origin.orientation.y=initq.y();
	    info.origin.orientation.z=initq.z();
	    info.origin.orientation.w=initq.w();
	    info.origin.position.x=initx;
	    info.origin.position.y=inity;
	    info.origin.position.z=0;
	    
	    grid.info=info;
	    //grid.header=header;
	    holomapviz_pub_.publish(grid);
	    //set spatial anchor tf
	    //origin center of local map 
	    float px,py;
	    px=(256)*cos(yaw)-(256)*sin(yaw);
	    px*=scale;
	    py=(256)*sin(yaw)+(256)*cos(yaw);
	    py*=scale;

	    //rotation
	    Eigen::Matrix4d m1,m2,m3,m4;
	    m1<<0,0,-1,0,
	      -1,0,0,0,
	      0,1,0,0,
	      0,0,0,1;
	    m2<<1,0,0,256*scale,
	      0,1,0,256*scale,
	      0,0,1,0,
	      0,0,0,1;
	    m3<<cos(yaw),-sin(yaw),0,initx,
	      sin(yaw),cos(yaw),0,inity,
	      0,0,1,0,
	      0,0,0,1;
	    m4=m3*m2*m1;
	    cout<<m4<<endl;
	    tf::Matrix3x3 rot(   m4(0,0),          m4(0,1),  m4(0,2),
			      m4(1,0),  m4(1,1),    m4(1,2),
			      m4(2,0),  m4(2,1),    m4(2,2));
	    tf_map_to_anchor_=tf::StampedTransform();
	    tf_map_to_anchor_.setOrigin(tf::Vector3(m4(0,3), m4(1,3), holoheight));
	    tf::Quaternion q;
	    rot.getRotation(q);
	    tf_map_to_anchor_.setRotation(q);
	    tf_map_to_anchor_.frame_id_=std::string("map");
	    tf_map_to_anchor_.child_frame_id_=std::string("spatialAnchor");
	    bSpatialAnchorUpdate=true;
	    transLookUp.insert(MTF::value_type(str,tf_map_to_anchor_));
	  }else{
	    sendMessage(LOC_FAILED);
	    isSendAck=false;
	  }
	  if(isSendAck){
	    sendMessage(FINISH_ALIGNMENT);
	  }
	}else if(headernum==HEADER_CHANC){
		//change spatial anchor
		pbuf=buffer;
		recvMessage(4,pbuf);
		buffer_[0]=buffer[3];      buffer_[1]=buffer[2];      buffer_[2]=buffer[1];      buffer_[3]=buffer[0];
		unsigned int strLength=((unsigned int*)buffer_)[0];
		char* strBuf=(char*)malloc(sizeof(char)*strLength+1);
		pbuf=(char*)strBuf;
		recvMessage(sizeof(char)*strLength,pbuf);
		strBuf[sizeof(char)*strLength]=0;
		string str(strBuf);
		cout<<str<<endl;
		cout<<transLookUp.size();
		tf_map_to_anchor_=transLookUp.at(str);
		bSpatialAnchorUpdate=true;
		free(strBuf);
	}else if(headernum==HEADER_POSLOST || headernum==HEADER_TRACKLOST){
		if(bPositionTracked){
			bPositionTracked=false;
			std_msgs::Bool posLost;
			posLost.data=true;
			pos_lost_pub_.publish(posLost);
		}
	}
	else if(headernum==HEADER_DEPTHSTREAM){
			
		//calculate map potision from sensor position
		tf::StampedTransform transform;
		try{
			listener.lookupTransform(std::string("map"),std::string("hololens"),
					ros::Time(0),transform);
		}catch (exception e){
		
		}

		tf::Quaternion q=transform.getRotation();
		tf::Quaternion invq=q.inverse();
		tf::Quaternion initq_;
		double initx_,inity_,yaw_;
		
		dimensionRemover(transform,initx_,inity_,yaw_,initq_);
		cv::Mat dst;
		std::cout<<"map recv"<<std::endl;
		recvMap(dst,0.66, 0.33);
		grid.data.resize(dst.cols*dst.rows);
		sensor_msgs::PointCloud pointcloud;
		tf::Matrix3x3 invmat(invq);
		double invx=invmat[0][0]*transform.getOrigin().getX()+invmat[0][1]*transform.getOrigin().getY()+invmat[0][2]*transform.getOrigin().getZ(),
			invy=invmat[1][0]*transform.getOrigin().getX()+invmat[1][1]*transform.getOrigin().getY()+invmat[1][2]*transform.getOrigin().getZ(),
			invz=invmat[2][0]*transform.getOrigin().getX()+invmat[2][1]*transform.getOrigin().getY()+invmat[2][2]*transform.getOrigin().getZ();
		for(int i=0;i<dst.cols*dst.rows/4;i++){
			grid.data[i]=dst.data[i]==255?100:0;
			int x,y;
			x=i%dst.cols;
			y=i/dst.cols;
			if(dst.data[i]==255){
				geometry_msgs::Point32 pt,pt_;
				float px,py;
				px=(x)*cos(yaw_)-(y)*sin(yaw_);px*=scale;px+=initx_;
				py=(x)*sin(yaw_)+(y)*cos(yaw_);py*=scale;py+=inity_;
				pt.x=px;
				pt.y=py;
				pt.z=0.5;
				
				pt_.x=pt.x*invmat[0][0]+pt.y*invmat[0][1]+pt.z*invmat[0][2]-invx;				
				pt_.y=pt.x*invmat[1][0]+pt.y*invmat[1][1]+pt.z*invmat[1][2]-invy;
				pt_.z=pt.x*invmat[2][0]+pt.y*invmat[2][1]+pt.z*invmat[2][2]-invz;				
				
				pointcloud.points.push_back(pt_);
			}
		}
		nav_msgs::MapMetaData info;
		std_msgs::Header header;
		info.resolution =scale;
		info.map_load_time=ros::Time();
		info.width=dst.rows;
		info.height=dst.cols;
		info.origin.orientation.x=initq_.x();
		info.origin.orientation.y=initq_.y();
		info.origin.orientation.z=initq_.z();
		info.origin.orientation.w=initq_.w();
		info.origin.position.x=initx_;
		info.origin.position.y=inity_;
		info.origin.position.z=0;
		grid.info=info;
		if(mapdate)holoGrid_pub_.publish(grid);
		pointcloud.header.frame_id="hololens";
		obst_pub_.publish(pointcloud);
		{
			char* buffer;
			sendMessage(FINISH_RECV_MESH);
		}
 	}else if(headernum==HOLOLENS_HEIGHT){
	  float dat[4];
	  pbuf=(char*)dat;
	  unsigned int recvsize=sizeof(float)*4;
	  recvMessage(recvsize, pbuf);
	  holoheight=dat[0];
	  floorpoint<<dat[1],dat[2],dat[3];
	  std::cout<<holoheight<<std::endl;
	  std::cout<<floorpoint.transpose()<<std::endl;
	  
	  tf::StampedTransform transform;
		try{
			listener.lookupTransform(std::string("map"),std::string("hololens"),
					ros::Time(0),transform);
		}catch (exception e){
		
		}
		geometry_msgs::PoseStamped floor2holo;
		floor2holo.header.stamp=ros::Time::now();
		floor2holo.header.frame_id="map";
		floor2holo.pose.position.x=transform.getOrigin().getX()+floorpoint(0);
		floor2holo.pose.position.y=transform.getOrigin().getY()+floorpoint(1);
		floor2holo.pose.position.z=transform.getOrigin().getZ()+floorpoint(2);
		//rotation (1,0,0) to bestn
		Eigen::Vector3d stdVec,rotAx;stdVec<<1,0,0;
		Eigen::Vector3d bestn=floorpoint/(-holoheight);
		rotAx=(stdVec.cross(bestn)).normalized();
		double angle=acos(stdVec.dot(bestn));
		floor2holo.pose.orientation.x=rotAx(0)*sin(angle/2);
		floor2holo.pose.orientation.y=rotAx(1)*sin(angle/2);
		floor2holo.pose.orientation.z=rotAx(2)*sin(angle/2);
		floor2holo.pose.orientation.w=cos(angle/2);
		floor2holo_pub_.publish(floor2holo);
	}
	else{
		std::cout<<"header: "<<headernum<<std::endl;	
		
	}
	
	if(!bSpatialAnchorUpdate){
		tf_map_to_anchor_.stamp_=ros::Time::now();
		tf_br_.sendTransform(tf_map_to_anchor_);
	}
	loop_rate.sleep();
    }
    }

  }

