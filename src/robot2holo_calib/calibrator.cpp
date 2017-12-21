#include "calibrator.h"

calibrator::calibrator(std::string holoLinkedFrame_, std::string odomFrame_, std::string robotFootFrame_)
{
	holoLinkedFrame=holoLinkedFrame_;
	odomFrame=odomFrame_;
	robotFootFrame=robotFootFrame_;
	R<<1,0,0,0,1,0,0,0,1;
	T<<0,0,0;
	tf_map_to_odom_.frame_id_ = std::string(holoLinkedFrame);
	tf_map_to_odom_.child_frame_id_ = std::string("hololens_p");

	tf_map_to_odom_.setOrigin(tf::Vector3(T(0), T(1), T(2)));
	tf::Matrix3x3 rot(R(0,0),R(0,1),R(0,2),
			R(1,0),R(1,1),R(1,2),
			R(2,0),R(2,1),R(2,2));
	tf::Quaternion q;
	rot.getRotation(q);
	tf_map_to_odom_.setRotation(q);
	horizontalCalibMode=false;
	
	boost::thread* thr = new boost::thread(boost::bind(&calibrator::publish_thread, this));
        //run();
}

void calibrator::poseStampedCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	latestPoseStamped=*msg;
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
}

void calibrator::run()
{
    holo_floor_sub_= nh.subscribe<geometry_msgs::PoseStamped>("/holo/floor", 1, &calibrator::poseStampedCB, this);
    std::cout<<"subscriver open"<<std::endl;
	
    while (ros::ok())
    {
        int c;
	if(kbhit()){
		std::cout<<"\r \r";	
		c=getch();
	
	}else{
		ros::spinOnce();
		continue;
	}
	
	try{
	  listener.lookupTransform(std::string(odomFrame),std::string(holoLinkedFrame),
				   ros::Time(0),transform);
	  listener.lookupTransform(std::string("spatialAnchor"),std::string("hololens"),
				   ros::Time(0),transform2);
	  
	}catch (tf::TransformException ex){
	  std::cout<<"listener error!!"<<std::endl;
          continue;
	}
	
	if(c==' '){
		if(horizontalCalibMode){
			try{
				listener.waitForTransform("/map", "/hololens",
					latestPoseStamped.header.stamp, ros::Duration(3.0));
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
			//std::cout<<"vertical vec: "<<floor2holoVec<<std::endl;
		}
		pep_pos.push_back(transform);
		hol_pos.push_back(transform2);
		tf::Quaternion q=transform.getRotation();
		std::cout<<"count "<<pep_pos.size()<<std::endl;
		
		std::cout<<holoLinkedFrame<<":("<<transform.getOrigin().getX()<<","<<transform.getOrigin().getY()<<","<<transform.getOrigin().getZ()<<"),("<<q.getX()<<","<<q.getY()<<","<<q.getZ()<<","<<q.getW()<<")"<<std::endl;;
		q=transform2.getRotation();
		std::cout<<"HoloLens:("<<transform2.getOrigin().getX()<<","<<transform2.getOrigin().getY()<<","<<transform2.getOrigin().getZ()<<"),("<<q.getX()<<","<<q.getY()<<","<<q.getZ()<<","<<q.getW()<<")"<<std::endl;

	}else if(c=='c'){
		if(pep_pos.size()<3){
			std::cout<<"Calibration Error!! The number of recorded positions must be >=3 !!"<<std::endl;
			continue;
		} 
		std::cout<<"=======calibration start========"<<std::endl;
		if(!horizontalCalibMode){
			calibration(pep_pos,hol_pos,R,T);
		}else{
			horizontalCalibration(pep_pos,hol_pos,floor2holo,head2foot,R,T);
		}
		std::cout<<"=========finish=========="<<std::endl;
		std::cout<<"calibration result!\n translation"<<std::endl
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
		if(pep_pos.size()>=1){
			pep_pos.pop_back();
			hol_pos.pop_back();
		}
        }else if(c=='z'){
	  std::ofstream ofs(fpath+".log");
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
	std::cout<<"========finished initial parameter computation========"<<std::endl;
	nonlinear_calibration(pep_pos,hol_pos,R,T);
}
void calibrator::horizontalCalibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> floor_holo, std::vector<Eigen::Vector3d> head_foot, Matrix3d& R, Vector3d& T){
	double bestScores[2];
	horizontal_initialization(pep_pos,hol_pos,floor_holo,head_foot,R,T,bestScores);
	std::cout<<"========finished initial parameter computation========"<<std::endl;
	nonlinear_horizontal_calibration(pep_pos,hol_pos,floor_holo,head_foot,R,T,bestScores);
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
		for(int i=0;i<pepdMat.size();i++){
			Vector3d pepax=mat2axis(pepdMat.at(i));
			Vector3d holax=mat2axis(holdMat.at(i));
			KA.col(i)=pepax;
			KB.col(i)=holax;
		}
		MatrixXd KBKA=KB*KA.transpose();
		//calc rotation
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(KBKA, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3d Hm;
		Eigen::Matrix3d uvt=svd.matrixU()*svd.matrixV().transpose();
		Hm<<1,0,0,
			0,1,0,
			0,0,uvt.determinant();
		R=svd.matrixV()*Hm*svd.matrixU().transpose();
		
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
		T=A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);
		std::cout<<"Rotation (Initial)"<<std::endl<<R<<std::endl;
		std::cout<<"Translation (Initial)"<<std::endl<<T<<std::endl;	
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

void calibrator::horizontal_initialization(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> verticalVecs, std::vector<Eigen::Vector3d> normVecs, Matrix3d& R, Vector3d& T, double* bestScores){
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
		}
		aveAxisPep=aveAxisPep.normalized();
		aveAxisHol=aveAxisHol.normalized();
		Eigen::Vector3d AxA=aveAxisHol.cross(aveAxisPep);
		double angle1=asin(AxA.norm());
		if(aveAxisPep.dot(aveAxisHol)<0)angle1=M_PI-angle1;
		
		AxA=AxA.normalized();	
		Eigen::Matrix3d RA,M,Minv,I=Eigen::Matrix3d::Identity();
		RA<<0,-AxA(2),AxA(1),
			AxA(2),0,-AxA(0),
			-AxA(1),AxA(0),0;
		M=I+sin(angle1)*RA+(1-cos(angle1))*RA*RA;
		
		//get vertical distance
		Vector3d vert;vert<<0,0,0;
		Minv=M.inverse();
		for(int i=0;i<verticalVecs.size();i++){
			Eigen::Vector3d floor2holo=verticalVecs.at(i);
			Eigen::Vector3d head2foot=normVecs.at(i);
			Eigen::Vector3d t_=floor2holo+Minv*head2foot;//in hololens flame
			double alpha=t_.dot(floor2holo)/floor2holo.norm();
			//std::cout<<"head2foot\n"<<head2foot<<std::endl;
			
			//std::cout<<"t and alpha\n"<<t_<<std::endl;
			//std::cout<<alpha<<std::endl;
			vert=vert+alpha*((1.0)/floor2holo.norm())*floor2holo;
		}
		
		
		
		vert=((1.0)/verticalVecs.size())*vert;
		
		//calc lest parameters from RA*t+tA=R*tb+t
		//solve least square problem
		//get good rotational parameter
		double mscore=0;
		int bestidx=-1;		
		/*for(int i=pepMat.size()-1;i>=0;i--){
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
			if(fabs(anglea)>0.1)continue;
			Vector3d ta=mp3.block(0,3,3,1);
			double score=ta.norm();
			if(mscore<score){
				mscore=score;
				bestidx=i;
			}
		}*/
		double offset_trans=0.1;
		double offset=0.01;		
		for(int i=0;i<pepdMat.size();i++){
			double anglea,angleb;
			Vector3d pepax;mat2axis_angle(pepdMat.at(i),pepax,anglea);
			Vector3d holax;mat2axis_angle(holdMat.at(i),holax,angleb);
			Vector3d ta=pepdMat.at(i).block(0,3,3,1);
			double score=(sqrt(ta.norm())-sqrt(offset_trans))/(fabs(anglea)+offset);
			if(mscore<score){
				mscore=score;
				bestidx=i;
			}
		}
		if(bestidx<0){
			std::cout<<"initialization failed!! (need large rotaion)\nPlease check how to move..."<<std::endl;
			return;			
		}
		bestScores[0]=mscore;
		//non rotation (RA nearly I): RA*t+tA=R*tb+t --> t+tA=R*tb+t --> tA=R'*M*tb
		
		//Matrix4d mp_1=btTrans2EigMat4d(pep_pos.at(pepMat.size()-1));
		//Matrix4d mh_1=btTrans2EigMat4d(hol_pos.at(holMat.size()-1));		
		//Matrix4d mp_2=btTrans2EigMat4d(pep_pos.at(bestidx));
		//Matrix4d mh_2=btTrans2EigMat4d(hol_pos.at(bestidx));		
		Matrix4d mh_3,mp_3;
		//mp_3=mp_2.inverse()*mp_1;
		//mh_3=mh_2.inverse()*mh_1;
		mp_3=pepdMat.at(bestidx);
		mh_3=holdMat.at(bestidx);
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
		
		
		vert=R*vert;
		
		mscore=0;

		bestidx=-1;
		double offset_angle=0.2;
		for(int i=0;i<pepdMat.size();i++){
			double anglea,angleb;
			Vector3d pepax;mat2axis_angle(pepdMat.at(i),pepax,anglea);
			Vector3d holax;mat2axis_angle(holdMat.at(i),holax,angleb);
			//if(fabs(anglea)<0.1)continue;
			Vector3d ta=pepdMat.at(i).block(0,3,3,1);
			Vector3d tb=holdMat.at(i).block(0,3,3,1);
			double score=(sqrt(fabs(anglea))-sqrt(offset_angle))/(ta.norm()+offset);
			if(mscore<score){
				mscore=score;
				bestidx=i;
			}
		}
		if(bestidx<0){
			std::cout<<"initialization failed!! (need less rotaion)\nPlease check how to move..."<<std::endl;
			return;			
		}		
		bestScores[1]=mscore;
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
		
		MatrixXd A(3,2);
		VectorXd B(3);
		Vector3d tpep,thol;
		tpep<<pepdMat.at(bestidx)(0,3),pepdMat.at(bestidx)(1,3),pepdMat.at(bestidx)(2,3);
		thol<<holdMat.at(bestidx)(0,3),holdMat.at(bestidx)(1,3),holdMat.at(bestidx)(2,3);
		Vector3d rightt=tpep-R*thol;
		Matrix3d leftM=Matrix3d::Identity()-pepdMat.at(bestidx).block(0,0,3,3);
		Vector3d t_c1=leftM*v_c1,t_c2=leftM*v_c2;
		A.block(0,0,3,1)=t_c1;
		A.block(0,1,3,1)=t_c2;
		B(0)=rightt(0);
		B(1)=rightt(1);
		B(2)=rightt(2);		
		
		VectorXd xv=A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);
		T=vert+xv(0)*v_c1+xv(1)*v_c2;
		std::cout<<"Rotation (Initial)\n"<<R<<std::endl;
		std::cout<<"Translation (Initial)"<<std::endl<<T<<std::endl;	




}

void calibrator::nonlinear_horizontal_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> floor_holo, std::vector<Eigen::Vector3d> head_foot, Matrix3d& R, Vector3d& T, double* bestScores){
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
	double offset_trans=0.1;
	double offset_angle=0.2;
	double offset=0.01;
	for(int i=1;i<pep_pos.size();i++){//rotation estimation by horizontal rotation 
		Matrix4d mp=btTrans2EigMat4d(pep_pos.at(i-1));
		Matrix4d mh=btTrans2EigMat4d(hol_pos.at(i-1));	
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
		double scorea=(sqrt(ta.norm())-sqrt(offset_trans))/(fabs(anglea)+offset);
		double scoreb=(sqrt(fabs(anglea))-sqrt(offset_angle))/(ta.norm()+offset);
		if(scoreb>=bestScores[1]*0.95){
			CostFunction* cost_function1 = new NumericDiffCostFunction<F1,ceres::CENTRAL,1,6>(new F1(pepax,holax));
			problem.AddResidualBlock(cost_function1, NULL, opt);		
			CostFunction* cost_function2 = new NumericDiffCostFunction<F2,ceres::CENTRAL,1,6>(new F2(mp3,mh3));
			problem.AddResidualBlock(cost_function2, NULL, opt);
				
		}
		
		if(scorea>=bestScores[0]*0.95){
			Vector3d ta_n=ta.normalized();
			Vector3d tb_n=tb.normalized();
			CostFunction* cost_function1 = new NumericDiffCostFunction<F1,ceres::CENTRAL,1,6>(new F1(ta_n,tb_n));
			problem.AddResidualBlock(cost_function1, NULL, opt);	
		}
	}
	/*
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
		if(fabs(anglea)<0.1 && score>0.1){
			Vector3d ta_n=ta.normalized();
			Vector3d tb_n=tb.normalized();
			CostFunction* cost_function1 = new NumericDiffCostFunction<F1,ceres::CENTRAL,1,6>(new F1(ta_n,tb_n));
			problem.AddResidualBlock(cost_function1, NULL, opt);	
		}
	}
	*/
	
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

