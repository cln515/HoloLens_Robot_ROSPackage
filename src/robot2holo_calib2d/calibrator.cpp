#include "calibrator.h"

calibrator::calibrator(std::string holoLinkedFrame_, std::string odomFrame_, std::string robotFootFrame_)
{
	holoLinkedFrame=holoLinkedFrame_;
	odomFrame=odomFrame_;
	robotFootFrame=robotFootFrame_;
	//ros::Rate rate(publish_rate_);
	//loop_rate=rate;
	R<<1,0,0,0,1,0,0,0,1;
	T<<0,0,0;
	tf_map_to_odom_.frame_id_ = std::string(holoLinkedFrame);
	tf_map_to_odom_.child_frame_id_ = std::string("hololens_p");

	//boost::thread t{publish_thread};
	tf_map_to_odom_.setOrigin(tf::Vector3(T(0), T(1), T(2)));
	tf::Matrix3x3 rot(R(0,0),R(0,1),R(0,2),
			R(1,0),R(1,1),R(1,2),
			R(2,0),R(2,1),R(2,2));
	tf::Quaternion q;
	rot.getRotation(q);
	tf_map_to_odom_.setRotation(q);
	horizontalCalibMode=false;
	//holo_floor_sub_= nh.subscribe<geometry_msgs::PoseStamped>("/holo/floor", 1, &calibrator::poseStampedCB, this);
	
	boost::thread* thr = new boost::thread(boost::bind(&calibrator::publish_thread, this));
        //run();
}

void calibrator::poseStampedCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	//std::cout<<"subscribed!!"<<std::endl;
	latestPoseStamped=*msg;
	std::cout<<(*msg)<<std::endl;
}


calibrator::calibrator(std::string holoLinkedFrame_, std::string odomFrame_, std::string robotFootFrame_,std::string fpath_)
{
	holoLinkedFrame=holoLinkedFrame_;
	odomFrame=odomFrame_;
	robotFootFrame=robotFootFrame_;	
	fpath=fpath_;
	R<<1,0,0,0,1,0,0,0,1;
	T<<0,0,0;
	tf_map_to_odom_.frame_id_ = std::string(holoLinkedFrame);
	tf_map_to_odom_.child_frame_id_ = std::string("hololens_p");
	
        std::ifstream ifs(fpath, std::ios::binary);
        if(ifs){
            float dat[12];
            ifs.read((char*)dat,sizeof(float)*12);
            R<<dat[0],dat[1],dat[2]
                ,dat[3],dat[4],dat[5]
                ,dat[6],dat[7],dat[8];
            T<<dat[9],dat[10],dat[11];
            ifs.close();
        }else{
            std::ofstream ofs(fpath);
            float dat[12];
            dat[0]=R(0,0);dat[1]=R(0,1);dat[2]=R(0,2);
            dat[3]=R(1,0);dat[4]=R(1,1);dat[5]=R(1,2);
            dat[6]=R(2,0);dat[7]=R(2,1);dat[8]=R(2,2);
            dat[9]=T(0);dat[10]=T(1);dat[11]=T(2);
            ofs.write((char*)dat,sizeof(float)*12);
            ofs.close();            
        }
        	tf_map_to_odom_.setOrigin(tf::Vector3(T(0), T(1), T(2)));
	tf::Matrix3x3 rot(R(0,0),R(0,1),R(0,2),
			R(1,0),R(1,1),R(1,2),
			R(2,0),R(2,1),R(2,2));
	tf::Quaternion q;
	rot.getRotation(q);
	tf_map_to_odom_.setRotation(q);
	


	
        boost::thread* thr = new boost::thread(boost::bind(&calibrator::publish_thread, this));
        //run();
}

void calibrator::run()
{
    // main loopbtMatrix3x3 btmp1(pep_posa.getRotation());
    holo_floor_sub_= nh.subscribe<geometry_msgs::PoseStamped>("/holo/floor", 1, &calibrator::poseStampedCB, this);
    std::cout<<"subscriver open"<<std::endl;
	
    while (ros::ok())
    {
        ros::spinOnce();
	int c=getch();
        ros::spinOnce();
	try{
	  listener.lookupTransform(std::string(odomFrame),std::string(holoLinkedFrame),
				   ros::Time(0),transform);
	  listener.lookupTransform(std::string("spatialAnchor"),std::string("hololens"),
				   ros::Time(0),transform2);
	  
	}catch (tf::TransformException ex){
	  std::cout<<"listener error!!"<<std::endl;
          continue;
	}
	//std::cout<<transform.getOrigin().getX()<<","<<transform.getOrigin().getY()<<","<<transform.getOrigin().getZ()<<std::endl;
	
	if(c==' '){
		if(horizontalCalibMode){
			try{
				listener.lookupTransform(std::string("map"),std::string("hololens"),
					latestPoseStamped.header.stamp,transform3);
				listener.lookupTransform(std::string(holoLinkedFrame),std::string(robotFootFrame),
					latestPoseStamped.header.stamp,transform4);
			}catch (tf::TransformException ex){
				std::cout<<"listener error!!"<<std::endl;
				continue;
			}
			Eigen::Vector3d floor2holoVec,head2footVec;			
			floor2holoVec<<transform3.getOrigin().getX()-latestPoseStamped.pose.position.x
				,transform3.getOrigin().getY()-latestPoseStamped.pose.position.y
				,transform3.getOrigin().getZ()-latestPoseStamped.pose.position.z;
			//map2holo
			Matrix4d map2holo=btTrans2EigMat4d(transform3);
			floor2holoVec=map2holo.block(0,0,3,3).inverse()*floor2holoVec;
			floor2holo.push_back(floor2holoVec);
			head2footVec<<transform4.getOrigin().getX(),transform4.getOrigin().getY(),transform4.getOrigin().getZ();
			head2foot.push_back(head2footVec);
			std::cout<<"vertical vec: "<<floor2holoVec<<std::endl;
		}
		pep_pos.push_back(transform);
		hol_pos.push_back(transform2);
		tf::Quaternion q=transform.getRotation();
		std::cout<<"count "<<pep_pos.size();
		
		std::cout<<holoLinkedFrame<<transform.getOrigin().getX()<<","<<transform.getOrigin().getY()<<","<<transform.getOrigin().getZ()<<std::endl
			<<q.getX()<<","<<q.getY()<<","<<q.getZ()<<","<<q.getW()<<std::endl;;
		q=transform2.getRotation();
		std::cout<<"Hololens:"<<transform2.getOrigin().getX()<<","<<transform2.getOrigin().getY()<<","<<transform2.getOrigin().getZ()<<std::endl
			<<q.getX()<<","<<q.getY()<<","<<q.getZ()<<","<<q.getW()<<std::endl;

	}else if(c=='c'){
		if(pep_pos.size()<3){
			std::cout<<"Calibration Error!! The number of recorded positions must be >=3 !!"<<std::endl;
			continue;
		} 
		if(!horizontalCalibMode){
			calibration(pep_pos,hol_pos,R,T);
		}else{
			horizontalCalibration(pep_pos,hol_pos,floor2holo,head2foot,R,T);
		}
		std::cout<<"calibration result! translation"<<std::endl
			<<T<<std::endl
			<<"rotation"<<std::endl
			<<R<<std::endl;
		tf_map_to_odom_.setOrigin(tf::Vector3(T(0), T(1), T(2)));
		tf::Matrix3x3 rot(R(0,0),R(0,1),R(0,2),
				R(1,0),R(1,1),R(1,2),
				R(2,0),R(2,1),R(2,2));
		tf::Quaternion q;
		rot.getRotation(q);
		tf_map_to_odom_.setRotation(q);
	}else if(c=='q'){
		test_solve();
		break; 
	}else if(c=='w'){
            std::ofstream ofs(fpath);
            float dat[12];
            dat[0]=R(0,0);dat[1]=R(0,1);dat[2]=R(0,2);
            dat[3]=R(1,0);dat[4]=R(1,1);dat[5]=R(1,2);
            dat[6]=R(2,0);dat[7]=R(2,1);dat[8]=R(2,2);
            dat[9]=T(0);dat[10]=T(1);dat[11]=T(2);
            ofs.write((char*)dat,sizeof(float)*12);
            ofs.close();
            std::cout<<"file saved: "<<fpath<<std::endl;
        }else if(c=='d'){
          pep_pos.pop_back();
          hol_pos.pop_back();
        }else if(c=='z'){
	  std::ofstream ofs(fpath+".log");
	  transition_log(pep_pos,hol_pos,ofs);
	  std::cout<<"Param R"<<std::endl;
	  std::cout<<R<<std::endl;
	  std::cout<<"Param t"<<std::endl;
	  std::cout<<T<<std::endl;
	  ofs<<"Param R"<<std::endl;
	  ofs<<R<<std::endl;
	  ofs<<"Param t"<<std::endl;
	  ofs<<T<<std::endl;
	}else if(c=='h'){
		horizontalCalibMode=!horizontalCalibMode;
		std::cout<<"horizontal calibration mode: "<<horizontalCalibMode<<std::endl;
		pep_pos.clear();
		hol_pos.clear();
		floor2holo.clear();
		head2foot.clear();
	}else{
	  tele.operation(c);
	}
    }
}

void calibrator::publish_thread()
{
	ros::Rate loop_rate(100);
    while (ros::ok())
    {
      // broadcast transform
        tf_map_to_odom_.stamp_ = ros::Time::now();
        tf_br_.sendTransform(tf_map_to_odom_);

        loop_rate.sleep();
    }
}

void calibrator::calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d& R, Vector3d& T){
	linear_calibration(pep_pos,hol_pos,R,T);
	nonlinear_calibration(pep_pos,hol_pos,R,T);
}
void calibrator::horizontalCalibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> floor_holo, std::vector<Eigen::Vector3d> head_foot, Matrix3d& R, Vector3d& T){
	horizontal_initialization(pep_pos,hol_pos,floor_holo,head_foot,R,T);
	nonlinear_horizontal_calibration(pep_pos,hol_pos,floor_holo,head_foot,R,T);
}
void calibrator::linear_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d& R, Vector3d& T){
		std::vector<Matrix4d> pepMat,pepdMat;   
		std::vector<Matrix4d> holMat,holdMat;
		for(int i=0;i<pep_pos.size();i++){
			Matrix4d mp=btTrans2EigMat4d(pep_pos.at(i));
			Matrix4d mh=btTrans2EigMat4d(hol_pos.at(i));		
			pepMat.push_back(mp);
			holMat.push_back(mh);
			if(i>0){
				pepdMat.push_back(pepMat.at(i-1).inverse()*pepMat.at(i));
				holdMat.push_back(holMat.at(i-1).inverse()*holMat.at(i));
			}
		}
		//Rotation calibration
		//obtain axis
		std::vector<Vector3d> pepAxis,holAxis;
		MatrixXd KA(3,pepdMat.size()),KB(3,pepdMat.size());
		std::cout<<"axis of rotation matrix"<<std::endl;
		for(int i=0;i<pepdMat.size();i++){
			Vector3d pepax=mat2axis(pepdMat.at(i));
			Vector3d holax=mat2axis(holdMat.at(i));
			KA.col(i)=pepax;
			KB.col(i)=holax;
			std::cout<<pepax<<std::endl;
			std::cout<<holax<<std::endl<<std::endl;
		}
		std::cout<<"KA,KB"<<std::endl;
		std::cout<<KA<<std::endl;
		std::cout<<KB<<std::endl;
		MatrixXd KBKA=KB*KA.transpose();
		//calc rotation
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(KBKA, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3d Hm;
		Eigen::Matrix3d uvt=svd.matrixU()*svd.matrixV().transpose();
		Hm<<1,0,0,
			0,1,0,
			0,0,uvt.determinant();
		R=svd.matrixV()*Hm*svd.matrixU().transpose();
		std::cout<<"Rotation"<<std::endl<<R<<std::endl;
		//t
		//solve least square problem
		MatrixXd A(pepdMat.size()*2,3);
		VectorXd B(pepdMat.size()*2);
		for(int i=0;i<pepdMat.size();i++){
			Vector3d tpep,thol;
			tpep<<pepdMat.at(i)(0,3),pepdMat.at(i)(1,3),pepdMat.at(i)(2,3);
			thol<<holdMat.at(i)(0,3),holdMat.at(i)(1,3),holdMat.at(i)(2,3);
			Vector3d rightt=tpep-R*thol;
			Matrix3d leftm=Matrix3d::Identity()-pepdMat.at(i).block(0,0,3,3);
			A.block(i*2,0,2,3)=leftm.block(0,0,2,3);
			B(i*2)=rightt(0);
			B(i*2+1)=rightt(1);
		}
		std::cout<<"Matrix A"<<std::endl<<A<<std::endl;
		std::cout<<"Voctor B"<<std::endl<<B<<std::endl;
		T=A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);
		std::cout<<"Translation"<<std::endl<<T<<std::endl;	
}

void calibrator::transition_log(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos,std::ofstream& ofs){
		std::vector<Matrix4d> pepMat,pepdMat;   
		std::vector<Matrix4d> holMat,holdMat;
		for(int i=0;i<pep_pos.size();i++){
			Matrix4d mp=btTrans2EigMat4d(pep_pos.at(i));
			Matrix4d mh=btTrans2EigMat4d(hol_pos.at(i));		
			pepMat.push_back(mp);
			holMat.push_back(mh);
			if(i>0){
				pepdMat.push_back(pepMat.at(i-1).inverse()*pepMat.at(i));
				holdMat.push_back(holMat.at(i-1).inverse()*holMat.at(i));
			}
		}
		//Rotation calibration
		//obtain axis
		std::vector<Vector3d> pepAxis,holAxis;
		MatrixXd KA(3,pepdMat.size()),KB(3,pepdMat.size());
		MatrixXd TA(3,pepdMat.size()),TB(3,pepdMat.size());
		std::cout<<"axis of rotation matrix"<<std::endl;
		for(int i=0;i<pepdMat.size();i++){
			Vector3d tpep,thol;
			tpep<<pepdMat.at(i)(0,3),pepdMat.at(i)(1,3),pepdMat.at(i)(2,3);
			thol<<holdMat.at(i)(0,3),holdMat.at(i)(1,3),holdMat.at(i)(2,3);
			TA.col(i)=tpep;
			TB.col(i)=thol;
			Vector3d pepax=mat2axis(pepdMat.at(i));
			Vector3d holax=mat2axis(holdMat.at(i));
			KA.col(i)=pepax;
			KB.col(i)=holax;
			//std::cout<<pepax<<std::endl;
			//std::cout<<holax<<std::endl<<std::endl;
		}
		std::cout<<"KA(robot)"<<std::endl;
		std::cout<<KA<<std::endl;
		std::cout<<"KB(sensor)"<<std::endl;
		std::cout<<KB<<std::endl;
		std::cout<<"Trans A(robot)"<<std::endl;
		std::cout<<TA<<std::endl;
		std::cout<<"Trans B(sensor)"<<std::endl;
		std::cout<<TB<<std::endl;		
		ofs<<"KA(robot)"<<std::endl;
		ofs<<KA<<std::endl;
		ofs<<"KB(sensor)"<<std::endl;
		ofs<<KB<<std::endl;
		ofs<<"Trans A(robot)"<<std::endl;
		ofs<<TA<<std::endl;
		ofs<<"Trans B(sensor)"<<std::endl;
		ofs<<TB<<std::endl;		
}


void calibrator::nonlinear_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d& R, Vector3d& T){
	//decompose
	double opt[6];
	R2axisRot(R,opt[3],opt[4],opt[5]);
	opt[0]=T(0);
	opt[1]=T(1);
	opt[2]=T(2);
	//optimize
	Problem problem;
	
	std::vector<Matrix4d> pepMat,pepdMat;   
	std::vector<Matrix4d> holMat,holdMat;
	for(int i=0;i<pep_pos.size();i++){
		Matrix4d mp=btTrans2EigMat4d(pep_pos.at(i));
		Matrix4d mh=btTrans2EigMat4d(hol_pos.at(i));		
		pepMat.push_back(mp);
		holMat.push_back(mh);
		if(i>0){
			pepdMat.push_back(pepMat.at(i-1).inverse()*pepMat.at(i));
			holdMat.push_back(holMat.at(i-1).inverse()*holMat.at(i));
		}
	}
	
	for(int i=0;i<pepdMat.size();i++){
		
		Matrix4d A = pepdMat.at(i);
		Matrix4d B = holdMat.at(i);	
		std::cout<<A<<std::endl<<B<<std::endl;
		CostFunction* cost_function = new NumericDiffCostFunction<simple_costfunctor,ceres::CENTRAL,12,6>(new simple_costfunctor(A,B));
		problem.AddResidualBlock(cost_function, NULL, opt);
	}
	
	Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	std::cout << "finished optimization"<< "\n";
	std::cout << summary.BriefReport() << "\n";
	
	R=axisRot2R(opt[3],opt[4],opt[5]);
	T<<opt[0],opt[1],opt[2];
}

void calibrator::horizontal_initialization(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> verticalVecs, std::vector<Eigen::Vector3d> normVecs, Matrix3d& R, Vector3d& T){
		std::vector<Matrix4d> pepMat,pepdMat;   
		std::vector<Matrix4d> holMat,holdMat;
		for(int i=0;i<pep_pos.size();i++){
			Matrix4d mp=btTrans2EigMat4d(pep_pos.at(i));
			Matrix4d mh=btTrans2EigMat4d(hol_pos.at(i));		
			pepMat.push_back(mp);
			holMat.push_back(mh);
			if(i>0){
				pepdMat.push_back(pepMat.at(0).inverse()*pepMat.at(i));
				holdMat.push_back(holMat.at(0).inverse()*holMat.at(i));
			}
		}
		//Rotation calibration
		//obtain axis
		std::vector<Vector3d> pepAxis,holAxis;
		MatrixXd KA(3,pepdMat.size()),KB(3,pepdMat.size());
		std::cout<<"axis of rotation matrix"<<std::endl;
		Vector3d aveAxisPep;aveAxisPep<<0,0,0;
		Vector3d aveAxisHol;aveAxisHol<<0,0,0;
		for(int i=0;i<pepdMat.size();i++){
			double anglea,angleb;
			Vector3d pepax;mat2axis_angle(pepdMat.at(i),pepax,anglea);
			Vector3d holax;mat2axis_angle(holdMat.at(i),holax,angleb);
			if(anglea<0.1||angleb<0.1)continue;
			KA.col(i)=pepax;
			aveAxisPep=pepax+aveAxisPep;
			KB.col(i)=holax;
			aveAxisHol=holax+aveAxisHol;
			std::cout<<pepax<<std::endl;
			std::cout<<holax<<std::endl<<std::endl;
		}
		std::cout<<"average Axis, robot & hololens"<<std::endl;
		aveAxisPep=aveAxisPep.normalized();
		aveAxisHol=aveAxisHol.normalized();
		std::cout<<aveAxisPep<<std::endl;
		std::cout<<aveAxisHol<<std::endl;
		Eigen::Vector3d AxA=aveAxisHol.cross(aveAxisPep);
		double angle1=asin(AxA.norm());
		if(aveAxisPep.dot(aveAxisHol)<0)angle1=M_PI-angle1;
		std::cout<<"angle\n"<<angle1<<std::endl;

		AxA=AxA.normalized();	
		std::cout<<"axis\n"<<AxA<<std::endl;		
		Eigen::Matrix3d RA,M,Minv,I=Eigen::Matrix3d::Identity();
		RA<<0,-AxA(2),AxA(1),
			AxA(2),0,-AxA(0),
			-AxA(1),AxA(0),0;
		M=I+sin(angle1)*RA+(1-cos(angle1))*RA*RA;
		std::cout<<"initial rot mat\n"<<M<<std::endl;
		std::cout<<"chack ta\n"<<aveAxisPep<<std::endl;
		std::cout<<"chack Rtb\n"<<M*aveAxisHol<<std::endl;			
		
		//get vertical distance
		Vector3d vert;vert<<0,0,0;
		Minv=M.inverse();
		for(int i=0;i<verticalVecs.size();i++){
			Eigen::Vector3d floor2holo=verticalVecs.at(i);
			Eigen::Vector3d head2foot=normVecs.at(i);
			Eigen::Vector3d t_=floor2holo+Minv*head2foot;//in hololens flame
			double alpha=t_.dot(floor2holo)/floor2holo.norm();
			std::cout<<"head2foot\n"<<head2foot<<std::endl;
			
			std::cout<<"t and alpha\n"<<t_<<std::endl;
			std::cout<<alpha<<std::endl;
			vert=vert+alpha*((1.0)/floor2holo.norm())*floor2holo;
		}
		vert=((1.0)/verticalVecs.size())*vert;
		std::cout<<"vertical vec (in hololens frame): "<<vert<<std::endl;
		
		//calc lest parameters from RA*t+tA=R*tb+t
		//solve least square problem
		//get good rotational parameter
		double mscore=0;
		int bestidx=-1;		
		for(int i=pepMat.size()-1;i>=0;i--){
			Matrix4d mp=btTrans2EigMat4d(pep_pos.at(pepMat.size()-1));
			Matrix4d mh=btTrans2EigMat4d(hol_pos.at(holMat.size()-1));		
			Matrix4d mp2=btTrans2EigMat4d(pep_pos.at(i));
			Matrix4d mh2=btTrans2EigMat4d(hol_pos.at(i));		
			Matrix4d mh3,mp3;
			mp3=mp2.inverse()*mp;
			mh3=mh2.inverse()*mh;
			double anglea,angleb;
			Vector3d pepax;mat2axis_angle(mp3,pepax,anglea);
			Vector3d holax;mat2axis_angle(mh3,holax,angleb);
			if(abs(anglea)>0.1)continue;
			Vector3d ta=mp3.block(0,3,3,1);
			double score=ta.norm();
			if(mscore<score){
				mscore=score;
				bestidx=i;
			}
		}
		if(bestidx<0){
			std::cout<<"initialization failed!! (need large rotaion)\nPlease check how to move..."<<std::endl;
			return;			
		}
		//non rotation (RA nearly I): RA*t+tA=R*tb+t --> t+tA=R*tb+t --> tA=R'*M*tb
		std::cout<<"bestidx: "<<bestidx<<std::endl;		
		Matrix4d mp_1=btTrans2EigMat4d(pep_pos.at(pepMat.size()-1));
		Matrix4d mh_1=btTrans2EigMat4d(hol_pos.at(holMat.size()-1));		
		Matrix4d mp_2=btTrans2EigMat4d(pep_pos.at(bestidx));
		Matrix4d mh_2=btTrans2EigMat4d(hol_pos.at(bestidx));		
		Matrix4d mh_3,mp_3;
		mp_3=mp_2.inverse()*mp_1;
		mh_3=mh_2.inverse()*mh_1;
		std::cout<<"chack matrix robot\n"<<mp_3<<std::endl;
		std::cout<<"chack matrix hololens\n"<<mh_3<<std::endl;
		Vector3d ta_=mp_3.block(0,3,3,1);
		Vector3d tb_=mh_3.block(0,3,3,1);
		tb_=M*tb_;
		ta_=ta_.normalized();
		tb_=tb_.normalized();
		AxA=tb_.cross(ta_);
		double angle2=asin(AxA.norm());
		if(ta_.dot(tb_)<0)angle2=M_PI-angle2;
		AxA=AxA.normalized();	
		Eigen::Matrix3d R2;
		R2<<0,-AxA(2),AxA(1),
			AxA(2),0,-AxA(0),
			-AxA(1),AxA(0),0;
		R=I+sin(angle2)*R2+(1-cos(angle2))*R2*R2;		
		R=R*M;
		std::cout<<"rotation mat\n"<<R<<std::endl;
		
		vert=R*vert;
		std::cout<<"vertical vec (in robot frame): "<<vert<<std::endl;
		
		mscore=0;
		double offset=0.01;
		bestidx=-1;
		for(int i=0;i<pepdMat.size();i++){
			double anglea,angleb;
			Vector3d pepax;mat2axis_angle(pepdMat.at(i),pepax,anglea);
			Vector3d holax;mat2axis_angle(holdMat.at(i),holax,angleb);
			if(abs(anglea)<0.1)continue;
			Vector3d ta=pepdMat.at(i).block(0,3,3,1);
			Vector3d tb=holdMat.at(i).block(0,3,3,1);
			double score=abs(anglea)/(ta.norm()+offset);
			if(mscore<score){
				mscore=score;
				bestidx=i;
			}
		}
		if(bestidx<0){
			std::cout<<"initialization failed!! (need less rotaion)\nPlease check how to move..."<<std::endl;
			return;			
		}		
		std::cout<<"bestidx: "<<bestidx<<std::endl;
		//solve (I-RA)t=Rtb-tA
		Vector3d robax=mat2axis(pepdMat.at(bestidx));
		//xz plane, yzplane
		Vector3d v_c1,v_c2;
		if(robax(1)!=1){
			v_c1<<robax(2),0,-robax(0);
			v_c1=v_c1.normalized();
			v_c2=v_c1.cross(robax);
		}else{
			v_c1<<0,robax(2),robax(1);
			v_c1=v_c1.normalized();
			v_c2=v_c1.cross(robax);
		}
		//solve (I-RA)(a*t_c1+b*t_c2)=tA-Rtb
		
		MatrixXd A(2,2);
		VectorXd B(2);
		Vector3d tpep,thol;
		tpep<<pepdMat.at(bestidx)(0,3),pepdMat.at(bestidx)(1,3),pepdMat.at(bestidx)(2,3);
		thol<<holdMat.at(bestidx)(0,3),holdMat.at(bestidx)(1,3),holdMat.at(bestidx)(2,3);
		Vector3d rightt=tpep-R*thol;
		Matrix3d leftM=Matrix3d::Identity()-pepdMat.at(bestidx).block(0,0,3,3);
		Vector3d t_c1=leftM*v_c1,t_c2=leftM*v_c2;
		A.block(0,0,2,1)=t_c1.block(0,0,2,1);
		A.block(0,1,2,1)=t_c2.block(0,0,2,1);
		B(0)=rightt(0);
		B(1)=rightt(1);
		
		std::cout<<"Matrix A"<<std::endl<<A<<std::endl;
		std::cout<<"Voctor B"<<std::endl<<B<<std::endl;
		
		VectorXd xv=A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);
		T=vert+xv(0)*t_c1+xv(1)*t_c2;
		std::cout<<"Translation"<<std::endl<<T<<std::endl;	




}

void calibrator::nonlinear_horizontal_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> floor_holo, std::vector<Eigen::Vector3d> head_foot, Matrix3d& R, Vector3d& T){
	//decompose
	double opt[6];
	R2axisRot(R,opt[3],opt[4],opt[5]);
	opt[0]=T(0);
	opt[1]=T(1);
	opt[2]=T(2);
	//optimize
	Problem problem;
	
	std::vector<Matrix4d> pepMat,pepdMat;   
	std::vector<Matrix4d> holMat,holdMat;
	for(int i=1;i<pep_pos.size();i++){//rotation estimation by horizontal rotation 
		Matrix4d mp=btTrans2EigMat4d(pep_pos.at(0));
		Matrix4d mh=btTrans2EigMat4d(hol_pos.at(0));	
		Matrix4d mp2=btTrans2EigMat4d(pep_pos.at(i));
		Matrix4d mh2=btTrans2EigMat4d(hol_pos.at(i));		
		Matrix4d mh3,mp3;
		mp3=mp2.inverse()*mp;
		mh3=mh2.inverse()*mh;
		double anglea,angleb;
		Vector3d pepax;mat2axis_angle(mp3,pepax,anglea);
		Vector3d holax;mat2axis_angle(mh3,holax,angleb);
		Vector3d ta=mp3.block(0,3,3,1);
		double score=ta.norm();
		if(abs(anglea)>0.1){
			CostFunction* cost_function1 = new NumericDiffCostFunction<F1,ceres::CENTRAL,1,6>(new F1(pepax,holax));
			problem.AddResidualBlock(cost_function1, NULL, opt);		
			std::cout<<"terma"<<std::endl;
			if(score<0.1){
				CostFunction* cost_function2 = new NumericDiffCostFunction<F2,ceres::CENTRAL,1,6>(new F2(mp3,mh3));
				problem.AddResidualBlock(cost_function2, NULL, opt);
			}	
		}
	}
	
	for(int i=pep_pos.size()-2;i>=0;i--){//direction
		Matrix4d mp=btTrans2EigMat4d(pep_pos.at(pep_pos.size()-1));
		Matrix4d mh=btTrans2EigMat4d(hol_pos.at(pep_pos.size()-1));		
		Matrix4d mp2=btTrans2EigMat4d(pep_pos.at(i));
		Matrix4d mh2=btTrans2EigMat4d(hol_pos.at(i));		
		Matrix4d mh3,mp3;
		mp3=mp2.inverse()*mp;
		mh3=mh2.inverse()*mh;
		double anglea,angleb;
		Vector3d pepax;mat2axis_angle(mp3,pepax,anglea);
		Vector3d holax;mat2axis_angle(mh3,holax,angleb);
		Vector3d ta=mp3.block(0,3,3,1);
		Vector3d tb=mh3.block(0,3,3,1);		
		double score=ta.norm();
		if(abs(anglea)<0.1 && score>0.1){
			Vector3d ta_n=ta.normalized();
			Vector3d tb_n=tb.normalized();
			CostFunction* cost_function1 = new NumericDiffCostFunction<F1,ceres::CENTRAL,1,6>(new F1(ta_n,tb_n));
			problem.AddResidualBlock(cost_function1, NULL, opt);	
		}
	}

	
	for(int i=0;i<floor_holo.size();i++){
		Eigen::Vector3d fh=floor_holo.at(i);
		Eigen::Vector3d hf=head_foot.at(i);
		CostFunction* cost_function3 = new NumericDiffCostFunction<F3,ceres::CENTRAL,1,6>(new F3(fh,hf));
		problem.AddResidualBlock(cost_function3, NULL, opt);
	}
	
	Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);
	std::cout << "finished optimization"<< "\n";
	std::cout << summary.BriefReport() << "\n";
	
	R=axisRot2R(opt[3],opt[4],opt[5]);
	T<<opt[0],opt[1],opt[2];
}

