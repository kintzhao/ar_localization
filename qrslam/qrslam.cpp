/* read me :
 *1. the num of qrcode N :we init as the static..    --->>> the method of vector add.
 *
 *2.
 *
 *
*/
// ***ros 里程订阅的修改
//   标记  //rosOdom

#include "qrslam.h"

ofstream state_miu("result.txt");
ofstream frobot("frobot.txt");
ofstream fmiu("miu.txt");
ofstream fvel("vel.txt");
ofstream fre("re.txt");

ofstream fQrData("qr_data_recorder.txt");

QrSlam::QrSlam(char* addr):transport_( ar_handle)
{
    is_odom_update = false;
    is_img_update  = false;
    qr_detect_init_num = 0;
    odom_init_num = 0;
    observed_mark_num_old = 0;
    init_start_localization_ = true ;
    coordinate_init_finished_ = false;
    coordinate_x_ = 0;
    coordinate_y_ = 0;
    coordinate_angle_ = 0;

    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();
    odometer_sub_ = ar_handle.subscribe("/odom",10, &QrSlam::getOdomterCallback,this);
    qr_image_sub_ = transport_.subscribe("/usb_cam/image_raw",10, &QrSlam::qrDetectCallback,this );
    raw_img_cvt_   =  new ImageConverter("/slam/raw_flip_image");
    robot_img_cvt_ =  new ImageConverter("/slam/robot_image");
    slam_img_cvt_  =  new ImageConverter("/slam/qrslam/slam_map");

    raw_global_map_ =  cv::imread("./data/bl.png", CV_LOAD_IMAGE_COLOR); //for display
    if(! raw_global_map_.data )
    {
        ROS_INFO("Could not open or find the image ./data/bl.png");
    }
    else
    {
        cv::line(raw_global_map_,cv::Point(raw_global_map_.cols/2,0),cv::Point(raw_global_map_.cols/2,raw_global_map_.rows),CV_RGB(255,0,0),3,8);
        cv::line(raw_global_map_,cv::Point(0,raw_global_map_.rows/2),cv::Point(raw_global_map_.cols,raw_global_map_.rows/2),CV_RGB(255,0,0),3,8);
    }
    pQrDetect_ = new DetctQrcode(addr);
    delta_t_=0.0;
    ftime(&TimeOld_); //init the value of delta_T .
    odom_i = 0 ;
    init_state_ = true ;
    odom_init_ = 0;

    init_EKF_value_ = true ;   //miu_SLAM的初始化标志，只有第一次进来时需要初始化
    robot_info_.X = robot_info_.Y = robot_info_.Vx = robot_info_.W = robot_info_.Theta = 0.0;
    last_time_ = ros::Time::now();
    fQrData <<"size id c0x y c1x y c2x y c3x y center_x y "<<endl;
}

QrSlam::~QrSlam()
{
    if (!raw_img_cvt_)
    {
        delete raw_img_cvt_;
    }
    if (!slam_img_cvt_)
    {
        delete slam_img_cvt_;
    }
    if (!pQrDetect_)
    {
        delete pQrDetect_;
    }

}
/****  里程处理回调函数
 *  getOdomterCallback(const nav_msgs::Odometry::ConstPtr& msg)
 *  主要用来订阅turtlebot发布的Topic：   sensor_msgs::ImageConstPtr& img_msg *
 * 1. 单位为：长度为cm 角度为 弧度
 * 2. 图像更新速率为： 发布速率 / ODOMETRY_FREQUENCE_DIFF_
 */
void QrSlam::getOdomterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(init_state_)
    {
        odom_init_++;
        if(odom_init_ == 5 ) init_state_ = false ;
    }
    odom_init_num++;
    if (odom_init_num == ODOMETRY_FREQUENCE_DIFF_ )  // odom frequence = 50 /ODOMETRY_FREQUENCE_DIFF_
    {
        odom_init_num = 0;
        current_time_ = ros::Time::now();
        double dt = (current_time_ - last_time_).toSec();
        last_time_ = current_time_;

        geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);
        cout<<"----------"<<yaw<<"---------"<<endl;

        robot_info_.X = msg->pose.pose.position.x*100 + coordinate_x_;
        robot_info_.Y = msg->pose.pose.position.y*100 + coordinate_y_;    // ****坐标系反转  逆转实际为负  测量为正

        robot_info_.V  = msg->twist.twist.linear.x*100;
        robot_info_.W  = msg->twist.twist.angular.z;     //****坐标系反转  逆转实际为负 测量为正  角速度积分计算要注意
        //        robot_info_.Theta += robot_info_.W*dt;
        //        robot_info_.Theta = robot_info_.Theta + coordinate_angle_ ;
        robot_info_.Theta = yaw + coordinate_angle_  ;

        ROS_INFO("start odom");
        cout<<"dt(秒) x y theta v w "<<dt<<" "<<" "<<robot_info_.X<<" "<<robot_info_.Y<<" "<<robot_info_.Theta<<" "<<robot_info_.V<<" "<<robot_info_.W<<std::endl;
        fvel<<" "<<robot_info_.V<<"   "<<robot_info_.W<<"   "<<endl;
        is_odom_update = true;
    }

}

/****  图像处理回调函数
 *  qrDetectCallback(const sensor_msgs::ImageConstPtr& img_msg)
 *
 * 主要用来订阅相机采集的图像发出的Topic：   sensor_msgs::ImageConstPtr& img_msg *
 * 1. 起始原点的设计： 以20号qr 作为原点或者以里程数据作为原点
 * 2. 图像更新速率为： 发布速率 / CAMERA_FREQUENCE_DIFF_
 * 3. 图像传入qrdetect，提取相应的2d codes‘s 位置。
 *    注：提取的位置信息是利用几何模型结合实际物理参数焦距求得的实际相对尺寸
 *
 */
void QrSlam::qrDetectCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    cv_bridge::CvImagePtr  cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(cv_camera_);
    cv::flip(cv_camera_,cv_camera_,-1);       //display has flip with x

    landmark5_vector_.clear();
    qr_detect_init_num++;
    //get observe image //detect frequence = 20 / CAMERA_FREQUENCE_DIFF_
    if(qr_detect_init_num == CAMERA_FREQUENCE_DIFF_ ) //
    {
        landmark5_vector_ = pQrDetect_->detectLandmarks(cv_ptr->image,visual_mark_num_); //获取的观测值是实际物理距离值
        cout<<"visual_mark_num_  "<<visual_mark_num_<<" landmark5_vector_.size()    "<<landmark5_vector_.size()<<"   "<<endl;
        qr_detect_init_num = 0;
        is_img_update = true;
        qrDetectDataStore(landmark5_vector_); //保存detect 数据
        //是否静止初始化 -->提取20的mark
        if( init_start_localization_ == true && abs(robot_info_.V)<0.05 && abs(robot_info_.W)<0.005)
        {   
            for(int i=0; i<visual_mark_num_;++i )
            {
                CPointsFour mark_2d = landmark5_vector_.at(i);
                if(mark_2d.ID == SELECT_MARK_FOR_INIT_ )
                {
                    mark5_init_vector_.push_back(mark_2d);
                    cout<<"-----please wait for init mark 20:"<<mark5_init_vector_.size()<<"-----"<<endl;
                }
            }
            // 静止采集的数据达到 INIT_LOCALIZATION ，触发初始化部分。
            if( mark5_init_vector_.size() == INIT_LOCALIZATION )  // 缓存大小
            {
                init_start_localization_  = false;  // 用于排除一直初始静态下定位
                coordinate_init_finished_ = true;
                cout<<"-----init vector of mark 20 have full :"<<mark5_init_vector_.size()<<"-----"<<endl;
            }
        }
    }
}
/*
 * 将detected的数据进行原始保存
 *    数据都是物理距离数据  单位 :cm
 * 数据保存在 ofstream fQrData("qr_data_recorder.txt") 中
 * 分别是 ：  size id c0x y c1x y c2x y c3x y center_x y …… id c0x y c1x y c2x y c3x y center_x y
 */
void QrSlam::qrDetectDataStore( vector<CPointsFour> vector_data )
{
    //fQrData.open(fileName);   //文件打开
    fQrData << vector_data.size();
    for (int i=0; i< vector_data.size();i++)
    {
        fQrData <<" "<< vector_data[i].ID <<" "<< vector_data[i].corn0.X <<" "<<vector_data[i].corn0.Y  <<" "<< vector_data[i].corn1.X<<" "<<vector_data[i].corn1.Y
               <<" "<< vector_data[i].corn2.X <<" "<<vector_data[i].corn2.Y  <<" "<< vector_data[i].corn3.X<<" "<<vector_data[i].corn3.Y
              <<" "<< vector_data[i].center.X<<" "<<vector_data[i].center.Y;
    }
    fQrData <<" "<< endl;
}
/*
 * 初始位置偏移补偿程序
 * 1 物理位置获取（观测向量表均值化）   mark_2d
 * 2 世界位置（配置信息）获取          mark_2d_world
 * 3 计算偏移                       diff_data
 * 4 偏移赋值
 * 关闭补偿
 */
void QrSlam::initLocalization( )
{
    CPointsFour mark_2d = pQrDetect_->averageCPointsFour(mark5_init_vector_,10.0,0.0,2.0,6420);
    CPointsFourWorld mark_2d_world = pQrDetect_->getInitMarkMessage(SELECT_MARK_FOR_INIT_);

    Point3f diff_data = diffCoordinate(mark_2d, mark_2d_world);
    coordinate_x_ = diff_data.x ;
    coordinate_y_ = diff_data.y ;
    coordinate_angle_ = diff_data.z;
    coordinate_init_finished_ = false;
}
/*
 * Point3f QrSlam::diffCoordinate(CPointsFour mark_2d,CPointsFourWorld mark_2d_world)
 *    初始位置运算，
 * 输入：
 *     为均值化后的mark， 起始物理位置与世界位置
 * 输出：
 *     相对偏移值
 *
 */
Point3f QrSlam::diffCoordinate(CPointsFour mark_2d,CPointsFourWorld mark_2d_world)
{
    Point3f coordinate_data;
    float d_x ,d_y,d_theta ;
    d_x =(   (-mark_2d.corn0.X - mark_2d_world.corn0.X) +(-mark_2d.corn1.X - mark_2d_world.corn1.X)
            +(-mark_2d.corn2.X - mark_2d_world.corn2.X) +(-mark_2d.corn3.X - mark_2d_world.corn3.X)
            +(-mark_2d.center.X - mark_2d_world.center.X)   )/5 ;
    d_y =(   (-mark_2d.corn0.Y - mark_2d_world.corn0.Y) +(-mark_2d.corn1.Y - mark_2d_world.corn1.Y)
            +(-mark_2d.corn2.Y - mark_2d_world.corn2.Y) +(-mark_2d.corn3.Y - mark_2d_world.corn3.Y)
            +(-mark_2d.center.Y - mark_2d_world.center.Y)  )/5 ;
    d_theta = -1.0*(  atan2( (mark_2d.corn3.Y - mark_2d.corn0.Y ),(mark_2d.corn3.X - mark_2d.corn0.X ))
                      +atan2( (mark_2d.corn2.Y - mark_2d.corn1.Y ),(mark_2d.corn2.X - mark_2d.corn1.X ))
                   )/2;
    coordinate_data.x = d_x;
    coordinate_data.y = d_y;
    coordinate_data.z = d_theta;
    return  coordinate_data;
}
/*
 * ekf_slam 算法设计与实现
 * 利用： 里程速度信息 + 观测landmark信息 -->  更新的机器人状态与地图信息
 *
 * 步骤
 *     1. 状态初始化，初始化时间
 *     2. 运动模型： 速度向量表（t-1） + dt
 *     3. 观测模型： 新状态：miu + odservation
 *                 已有状态：代入融合更新
 *     4. 数据融合：  预测与观测偏差-->卡尔曼增益
 *        状态更新 --> robot pos +  map postions
 * 变量说明：
 *     1. miu_state（3+2n,1）   系统状态量                  miu_convar_p 系统状态协方差量（3+2n,3+2n）
 *     2. observed_mark_num    观测到landmark总数量（n）    observed_mark_num_old   landmark已有储存量 -->状态扩维
 *     3.
 */
void QrSlam::ekfSlam(float V, float W)
{
    cout<<"ekfslam start !"<<endl;
    if(init_EKF_value_)
    {
        ftime(&Time_);
        miu_state = Mat::zeros(3,1,CV_32FC1);       //已去掉跟s有关的项，原来是3+3*

        miu_state.at<float>(0) = robot_info_.X + coordinate_x_;
        miu_state.at<float>(1) = robot_info_.Y + coordinate_y_;      //  取y  标准正向
        miu_state.at<float>(2) = robot_info_.Theta + coordinate_angle_ ;

        miu_convar_p =  Mat::zeros(3,3,CV_32FC1);    //这个可以放到初始化函数中去  ？？初值
        miu_convar_p.at<float>(0,0) = 0.1;//0.1;//1;//100;//0.1;
        miu_convar_p.at<float>(1,1) = 0.10;//0.1;//1;//100;//0.1;
        miu_convar_p.at<float>(2,2) = 0.38;//0.1;//0.38;

        init_EKF_value_ = false;
        TimeOld_ = Time_;
        velocities_.push_back(Point2f(V,W));
    }
    else
    {
        ftime(&Time_);
        delta_t_ = (Time_.time - TimeOld_.time) + (Time_.millitm - TimeOld_.millitm)/1000.0; //  秒
        /////----------------------------------------------------------------------------------------------------------
        // landmark_vector_ = pQrDetect_->QrCodeMapping(visual_mark_num_,robot_pose); //获取的观测值是实际物理距离值
        cout<<"observation start !"<<endl;
        genObservations();
        getNumQrcode();
        /////----------------------------------------------------------------------------------------------------------
        int observed_mark_num = observed_landmark_num.size();
        cout<<"----------------"<<observed_mark_num_old<<endl;

        if(observed_mark_num > observed_mark_num_old)     //状态miu_SLAM 扩维 一个码两个量（x,y）加入系统状态   ？？？加入 mark(r,seita) 坐标值  扩维与加数。
        {
            //miu_SLAM扩维       后面将其改造成成员函数
            cv::Mat miu_state_new = Mat::zeros(3+2*observed_mark_num,1,CV_32FC1);   //已去掉跟s有关的项，
            for(uint i=0;i<3+2*observed_mark_num_old;i++)
            {
                miu_state_new.at<float>(i) = miu_state.at<float>(i);
            }
            miu_state = Mat::zeros(3+2*observed_mark_num,1,CV_32FC1);   //已去掉跟s有关的项，原来是3+3*
            for(uint i=0;i<3+2*observed_mark_num_old;i++)
            {
                miu_state.at<float>(i) = miu_state_new.at<float>(i);
            }
            displayMatrix(miu_state);
            //  xP_SLAM 扩维
            cv::Mat miu_convar_p_new =  Mat::zeros(3+2*observed_mark_num,3+2*observed_mark_num,CV_32FC1);  //这个可以放到初始化函数中去  ？？初值???
            displayMatrix(miu_convar_p_new);
            displayMatrix(miu_convar_p);

            for(uint i=0;i<3+2*observed_mark_num_old ;i++)
            {
                for(uint j=0;j<3+2*observed_mark_num_old;j++)
                {
                    miu_convar_p_new.at<float>(i,j) = miu_convar_p.at<float>(i,j);
                }
            }

            miu_convar_p = Mat::zeros(3+2*observed_mark_num,3+2*observed_mark_num,CV_32FC1);   //已去掉跟s有关的项，原来是3+3*
            for(uint i=0;i<3+2*observed_mark_num_old;i++)
            {
                for(uint j=0;j<3+2*observed_mark_num_old;j++)
                    miu_convar_p.at<float>(i,j) = miu_convar_p_new.at<float>(i,j);
            }

            for(uint i=3+2*observed_mark_num_old;i<3+2*observed_mark_num;i++)
            {
                miu_convar_p.at<float>(i,i) = 1000000;   //对角方差要大1000000
            }
        }
        observed_mark_num_old = observed_mark_num;
        // xPred_SLAM 与 xPPred_SLAM 传入值的问题
        Mat miu_prediction = Mat::zeros(3+2*observed_mark_num, 1, CV_32FC1); //原来是3+3*N
        //Point3f xLast; //已经不需要，第一次进来时用初始化的
        Mat x_convar_p_prediction = Mat::zeros(3+2*observed_mark_num, 3+2*observed_mark_num, CV_32FC1);//协方差矩阵的估计矩阵
        Mat Fx = Mat::zeros(3, 3+2*observed_mark_num, CV_32FC1);
        Mat I_SLAM = Mat::eye(3+2*observed_mark_num, 3+2*observed_mark_num, CV_32FC1); //算法中的单位矩阵
        Mat Gt = Mat::zeros(3+2*observed_mark_num, 3+2*observed_mark_num, CV_32FC1);
        Mat Gt_temp = Mat::zeros(3, 3, CV_32FC1); //用来计算Gt的3*3矩阵
        Mat Ht = Mat::zeros(2, 3+2*observed_mark_num, CV_32FC1);
        Mat Kt = Mat::zeros(3+2*observed_mark_num, 2, CV_32FC1); //


        Mat Fj;
        Mat cal_temp = Mat::zeros(3, 1, CV_32FC1); //算法第三行计算xPred_SLAM时的3*1矩阵
        Mat Qt = Mat::zeros(2, 2, CV_32FC1);
        Mat Ht_temp = Mat::zeros(2, 5, CV_32FC1); //用来计算Ht的2*5矩阵
//        Mat miu_temp_sum=Mat::zeros(3+2*observed_mark_num,1,CV_32FC1); //用来计算Ht的2*5矩阵
//        Mat Kt_i_Ht_sum=Mat::zeros(3+2*observed_mark_num,3+2*observed_mark_num,CV_32FC1);
//        Mat delta_z_observed =Mat::zeros(3+2*observed_mark_num,3+2*observed_mark_num,CV_32FC1);



        Mat St = Mat::zeros(2,2,CV_32FC1); //vv
        Mat Rt = Mat::zeros(3,3,CV_32FC1); //vv
        Mat Vt = Mat::zeros(3,2,CV_32FC1);
        Mat Mt = Mat::zeros(2,2,CV_32FC1);


        landmark_miu_conver_p_ = Mat::zeros(2,2,CV_64FC1); //协方差椭圆有关
        ////////////////////////////////////////////////////////////////////////////////////
        //     motionModelIndex==0 //Velocity模式
        velocities_.push_back(Point2f(V,W));
        Point2f VEL = velocities_.at(velocities_.size()-2);  //数组从0开始  又存入一值

        Vd_ = VEL.x;
        Wd_ = VEL.y;  //  取y  标准正向
        cout<<"Vd: "<<Vd_<<"   "<<"Wd"<<Wd_<<endl;
        if( Wd_ < 0.00006 && Wd_ >=0)  Wd_ = 0.00006;
        if( Wd_ > -0.00006 && Wd_ <0)  Wd_ = -0.00006;
        cout<<"vd/wd"<<Vd_/Wd_<<endl;
        //        float Vr_noise, Wr_noise, Sigv, Sigw;
        //        Sigv = a1*Vd_*Vd_+a2*Wd_*Wd_;
        //        Sigw = a3*Vd_*Vd_+a4*Wd_*Wd_;
        //        Vr_noise = genGaussianValue(Sigv);
        //        Wr_noise = genGaussianValue(Sigw);

        //        Vd_ = VEL.x + Vr_noise;
        //        Wd_ = VEL.y + Wr_noise;
        //        if( Wd_ <0.00006 && Wd_ >=0)  Wd_ =0.0001;
        //        if( Wd_ >-0.00006 && Wd_ <0)  Wd_ =-0.0001;

        ////基于EKF的SLAM方法， 条件有当前观测量Observations， 上一时刻估计所得机器人位置
        //计算Ft
        Fx.at<float>(0,0) = 1.0;
        Fx.at<float>(1,1) = 1.0;
        Fx.at<float>(2,2) = 1.0;
        //计算cal_temp，预测不加扰动
        float last_miu_theta = miu_state.at<float>(2) ;//上一个miu_SLAM的角度
        angleWrap(last_miu_theta);


        //speed mode motion increase   Wd 不能为 0
        cal_temp.at<float>(0) =  -Vd_/Wd_*sin(last_miu_theta) + Vd_/Wd_*sin(last_miu_theta+Wd_*delta_t_);
        cal_temp.at<float>(1) =   Vd_/Wd_*cos(last_miu_theta) - Vd_/Wd_*cos(last_miu_theta+Wd_*delta_t_);
        cal_temp.at<float>(2) =  Wd_*delta_t_;
        cout<<"cal_temp"<<cal_temp<<endl;

        ///prediction
        miu_prediction = miu_state + Fx.t()*cal_temp; // X'= X +Jaci_f(x)*delt(x)   predicted mean
        angleWrap(miu_prediction.at<float>(2));
        fre<<cal_temp.at<float>(0)<<"  "<<cal_temp.at<float>(1)<<" "<< cal_temp.at<float>(2)<<endl;
        // cout<<"miu_prediction"<<miu_prediction<<endl;

        //计算Gt   Jacibi_x(x,y,theita)
        Gt_temp.at<float>(0,2) = -Vd_/Wd_*cos(last_miu_theta) + Vd_/Wd_*cos(last_miu_theta+Wd_*delta_t_);
        Gt_temp.at<float>(1,2) = -Vd_/Wd_*sin(last_miu_theta) + Vd_/Wd_*sin(last_miu_theta+Wd_*delta_t_);
        Gt=I_SLAM+Fx.t()*Gt_temp*Fx ;
        //change()
        //        Gt_temp.at<float>(0,2) = Vd_/Wd_*cos(last_miu_theta) - Vd_/Wd_*cos(last_miu_theta+Wd_*delta_t_);
        //        Gt_temp.at<float>(1,2) = Vd_/Wd_*sin(last_miu_theta) - Vd_/Wd_*sin(last_miu_theta+Wd_*delta_t_);
        //        Gt=I_SLAM+Fx.t()*Gt_temp*Fx ;


        //计算Vt   Jacibi_u(v,w)
        Vt.at<float>(0,0) = (-sin(last_miu_theta)+sin(last_miu_theta+Wd_*delta_t_))/Wd_; Vt.at<float>(0,1)=Vd_*(sin(last_miu_theta)-sin(last_miu_theta+Wd_*delta_t_))/Wd_/Wd_+Vd_*cos(last_miu_theta+Wd_*delta_t_)*delta_t_/Wd_;
        Vt.at<float>(1,0) = (cos(last_miu_theta)-cos(last_miu_theta+Wd_*delta_t_))/Wd_;  Vt.at<float>(1,1)=-Vd_*(cos(last_miu_theta)-cos(last_miu_theta+Wd_*delta_t_))/Wd_/Wd_+Vd_*sin(last_miu_theta+Wd_*delta_t_)*delta_t_/Wd_;
        Vt.at<float>(2,0) = 0;                          	                                Vt.at<float>(2,1)=delta_t_;

        //计算Mt   motion noise ;  why add the motion noise   ?????
        Mt.at<float>(0,0) = a1*Vd_*Vd_+a2*Wd_*Wd_;
        Mt.at<float>(1,1) = a3*Vd_*Vd_+a4*Wd_*Wd_;
        Rt = Vt*Mt*Vt.t();//计算Rt

        //计算预测方差矩阵miu_convar_p
        cout<<"----------------------------------------"<<endl;
        // cout<<"miu_convar_p"<<miu_convar_p<<endl;
        x_convar_p_prediction = Gt*miu_convar_p*Gt.t() + Fx.t()*Rt*Fx; //计算预测方差 Px  ??????????  xP_SLAM  与 xPred_SLAM

        //计算Qt
        Qt.at<float>(0,0) = sigma_r*sigma_r;
        Qt.at<float>(1,1) = sigma_phi*sigma_phi;
        /////----------------------------------------------------------------------------------------------------------
        //    LandmarkVector = pLocalizer->QrCodeMapping(MarkVisualNum);
        //    GenObservations();

        ///？？观测后面要改成 x y 坐标系
        //    /////----------------------------------------------------------------------------------------------------------
        /////----------------------------------------------------------------------------------------------------------
        //更新
        int   j = 0;  // the order of Qrmark  表示观察到的 特征标记      与状态量中landmark有关
        int   Qid = 0; //  the Id of Qrmark
        int   observed_flag = 0 ;//是否观察到过的标志  0表示从未出现过
        float dst;
        float theta;
        float q;
        Point2f delta;
        Point2f z,zp;
        Mat delta_z;
        for(int i=0; i< observations_.size(); i++)
        {
            Qid = observations_.at(i).z;
            z = Point2f(observations_.at(i).x,observations_.at(i).y); //观测值  是极坐标
            for(int c=0; c<Lm_observed_Num.size(); c++)
            {
                if(Qid == Lm_observed_Num[c])   //   出现过   逐一比对已经观察到的mark列表
                {
                    observed_flag = 1 ;
                    j = c;                     //  选取第j个landmark导入观察模型并进行状态更新。
                }
            }
            if(observed_flag == 0) //从未出现过
            {
                //           landmark_observed.at<float>(i)=j ;//保存住这次观测到的路标号
                Lm_observed_Num.push_back(Qid) ;
                j = Lm_observed_Num.size()-1 ;                // 注意 数组从0开始  -1 ？？？？？？？？？？
                // 在观测函数 x y 极坐标系下的值。
                miu_prediction.at<float>(2*j+3) = miu_prediction.at<float>(0) + z.x*cos( z.y + miu_prediction.at<float>(2) );  //第j个landmark的y坐标
                miu_prediction.at<float>(2*j+4) = miu_prediction.at<float>(1) + z.x*sin( z.y + miu_prediction.at<float>(2) );  //第j个landmark的x坐标
            }
            observed_flag=0 ;
            //预测测量值   ？？？？？？反向查找运动预测中 Point2f(xPred_SLAM.at<float>(2*j+3),xPred_SLAM.at<float>(2*j+4))
            //             不然就为加入地图mark  无变化     xPred_SLAM  都为世界坐标系下值
            //这里的z相当于landmark的标号  （全局坐标下： 地图值- 预测机器人值）与 观测值 z ： 表示mark与robot的相对距离
            delta = Point2f(miu_prediction.at<float>(2*j+3),miu_prediction.at<float>(2*j+4))-Point2f(miu_prediction.at<float>(0),miu_prediction.at<float>(1));
            q = delta.x*delta.x+delta.y*delta.y ;
            dst = sqrt(q);
            theta = atan2(delta.y, delta.x) - miu_prediction.at<float>(2);      //偏离robot的方向角方向的角度   相对xy坐标系下值
            zp.x= dst;
            angleWrap(theta);
            zp.y= theta;

            //计算Fj
            Fj=Mat::zeros(5,3+2*observed_mark_num,CV_32FC1);  //已降维，本来是6行
            Fj.at<float>(0,0) = 1;
            Fj.at<float>(1,1) = 1;
            Fj.at<float>(2,2) = 1;
            Fj.at<float>(3,2*j+3) = 1;
            Fj.at<float>(4,2*j+4) = 1;

            //计算Htjaccibi
            Ht_temp.at<float>(0,0) = -delta.x*dst;
            Ht_temp.at<float>(0,1) = -delta.y*dst;
            Ht_temp.at<float>(0,3) = delta.x*dst;
            Ht_temp.at<float>(0,4) = delta.y*dst;

            Ht_temp.at<float>(1,0) = delta.y;
            Ht_temp.at<float>(1,1) = -delta.x;
            Ht_temp.at<float>(1,2) = -q;
            Ht_temp.at<float>(1,3) = -delta.y;
            Ht_temp.at<float>(1,4) = delta.x;

            ///~~~~~~~~~~~~~~~~~~
            Ht=(1/q)*Ht_temp*Fj ;

            St = Ht*x_convar_p_prediction*Ht.t()+Qt;
            Kt = x_convar_p_prediction*Ht.t()*St.inv();

            z = z-zp;       //  更新的处理
            angleWrap(z.y);
            delta_z = Mat::zeros(2,1,CV_32FC1);
            delta_z.at<float>(0) = z.x;
            delta_z.at<float>(1) = z.y;
            //××                        //xPred_SLAM为xy坐标系    delta_z 为极坐标系   存在点问题 观测模型是以  极坐标建立的   H阵描述的也就是 极值与极径的关系  即K表述成立
            //                        miu_temp_sum = miu_temp_sum + Kt*delta_z ;
            //                        Kt_i_Ht_sum = Kt_i_Ht_sum + Kt*Ht;
            //                    }
            //                    miu_prediction =miu_prediction + miu_temp_sum ;  // xPred_SLAM 关于landmark为极坐标值
            //                    angleWrap(miu_prediction.at<float>(2));
            //                    x_convar_p_prediction= (I_SLAM-Kt_i_Ht_sum)*x_convar_p_prediction;
            //                    miu_state=miu_prediction;
            //                    miu_convar_p=x_convar_p_prediction;
            //×××
            ////××
            miu_prediction = miu_prediction +Kt*delta_z;  // xPred_SLAM 关于landmark为极坐标值
            angleWrap(miu_prediction.at<float>(2));
            x_convar_p_prediction= (I_SLAM-Kt*Ht)*x_convar_p_prediction;
        }
        miu_state=miu_prediction;
        miu_convar_p=x_convar_p_prediction;
        ////×
        ///
        Point3f xPred;
        xPred.x=miu_prediction.at<float>(0);
        xPred.y=miu_prediction.at<float>(1);
        xPred.z=miu_prediction.at<float>(2);
        angleWrap(xPred.z);

        est_path_points_.push_back(xPred);
        miu_convar_p= 0.5*(miu_convar_p+miu_convar_p.t());
        //    xP_SLAM(Rect(0,0,3,3)).copyTo(xP);   //分离出位置的协方差，用于图形显示不确定椭圆

        //draw_mark();
        for(int t=0;t<3+2*Lm_observed_Num.size();t++)
        {
            cout <<" "<<miu_state.at<float>(t)<<" ";
        }
        cout <<" "<<endl;
        //   cvWaitKey(10);
        TimeOld_ = Time_;
    }
}

void QrSlam::genObservations()
{
    // QrLandMark landmarktemp;
    Point3ffi  mark_temp;
    CPointsFour landmark5_temp;
    observations_.clear();  //清空观测量。只保存当前时刻的观测量
    for(int i=0;i < visual_mark_num_; i++)
    {
        landmark5_temp = landmark5_vector_.at(i);  //观测实际值
        mark_temp = addObservationPos(landmark5_temp.corn0,5*landmark5_temp.ID + 0) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中
        mark_temp = addObservationPos(landmark5_temp.corn1,5*landmark5_temp.ID + 1) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中
        mark_temp = addObservationPos(landmark5_temp.corn2,5*landmark5_temp.ID + 2) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中
        mark_temp = addObservationPos(landmark5_temp.corn3,5*landmark5_temp.ID + 3) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中

        mark_temp = addObservationPos(landmark5_temp.center,5*landmark5_temp.ID + 4) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中
    }
}
/**
 * @brief QrSlam::addObservationPos
 * 添加landmark 点（x,y） 到观测量中
 * @param landmarktemp     2d mark角点
 * @param id               2d mark ID
 * @return                 点集（x,y,id）  id = ID*5 + j (j=01234)
 */
Point3ffi QrSlam::addObservationPos(ConerPoint landmarktemp ,int id )
{
    //Generate observations.假设传感器能观察到机器人周围sd米内的所有特征   @param[in] sd
    Point2f delta;
    float dst;   //特征距离
    float theta; //特征角w.r.t.robot frame
    Point3ffi mark_temp;
    delta = Point2f(landmarktemp.X,landmarktemp.Y) ;//-Point2f(RobotPos.X,RobotPos.Y); //已经是delta值了
    dst = norm(delta);
    theta = atan2(delta.y, delta.x) ;//- robot_info_.Theta ;//    ??? the true path of vehicle
    //    if(delta.x < 0)
    //        theta = 3.14159 + theta;   //- robot_info_.Theta ;//    ??? the true path of vehicle
    //        dst+= genGaussianValue(sigma_r*sigma_r);
    //        theta+= genGaussianValue(sigma_phi*sigma_phi); //给测量量添加噪声
    angleWrap(theta);
    mark_temp.x = dst;
    mark_temp.y = theta;
    mark_temp.z = id;//temp value
    if( (id%5==4) && (id/5 ==19))
    {
        std::string text_id ="id: "+ int2str(id/5 );
        cv::putText(cv_camera_,text_id,cvPoint(20,100),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));

        std::string text1 ="d: "+ float2str(mark_temp.x );
        cv::putText(cv_camera_,text1,cvPoint(20,150),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));

        std::string text2 ="theta: "+ float2str(mark_temp.y*180/ 3.14159);
        cv::putText(cv_camera_,text2,cvPoint(20,200),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    }
    return mark_temp;

}
/**
 * @brief QrSlam::getNumQrcode
 * observations_        一帧图像管测量
 * observed_landmark_num   所有观测到的landmark id 向量表
 *
 * 逐步筛选observations_中的量，与已有的observed_landmark_num向量表匹配，
 * observations_中第一次观测到的角点加入observed_landmark_num向量表中
 */
void QrSlam::getNumQrcode(void)
{
    int Qid;
    int flag=0 ;
    for(int i=0;i<observations_.size();i++)
    {
        Qid = observations_.at(i).z;
        for(int c = 0; c < observed_landmark_num.size(); c++)
        {
            if(Qid == observed_landmark_num[c])//出现过
            {
                flag=1 ;
            }
        }
        if(flag==0) //从未出现过
        {
            observed_landmark_num.push_back(Qid) ;
        }
        flag=0 ;
    }
}
/**
 * @brief QrSlam::genGaussianValue
 * 高斯扰动量生成
 * @param Sigma2
 * @return
 */
float QrSlam::genGaussianValue(float Sigma2)
{///Generate a Gaussian value with variance sigma
    return rng.gaussian(sqrt(Sigma2));
}
/**
 * @brief QrSlam::displayMatrix
 * Mat 矩阵打印输出
 * @param matrix
 */
void QrSlam::displayMatrix(Mat matrix)
{
    for(int ii=0;ii<matrix.rows;ii++)
    {
        for(int jj=0;jj<matrix.cols;jj++)
        {
            std::cout<<"  "<<matrix.at<float>(ii,jj);
        }
        std::cout<<""<<std::endl;
    }
}
/**
 * @brief QrSlam::angleWrap
 * 角度归一化，规范到（-pi,pi）范围内
 * @param angle
 */
void QrSlam::angleWrap(float& angle)
{
    ///这个函数用来实现把角度规划到-pi至pi之间
    if(angle>=CV_PI)
        while(angle >= CV_PI)
        { angle=angle-2*CV_PI;}
    else if(angle<-1.0*CV_PI)
        while(angle < -1.0*CV_PI)
        { angle = angle+2*CV_PI;}

}
/**
 * @brief QrSlam::int2str
 * int 转换成 string
 * @param num
 * @return
 */
std::string  QrSlam::int2str(int num)
{
    std::stringstream ss;
    ss << num;
    std::string text = ss.str();
    return text;
}
/*
bool QrSlam::DrawCovarianceEllipseI(Mat image, Point2f center, Mat covMatrix)
{///draw covariance ellipse on the display image
    double angle;
    Mat eigenvalues;
    Mat	eigenvectors;
    CvSize size;

    if(!eigen(covMatrix,eigenvalues,eigenvectors))   //注意eigenvalues和eigenvectors的数据格式与covMatrix一致
        return false;

    //DisplayMatrix("covMatrix", covMatrix);
    //DisplayMatrix("eigenvalues", eigenvalues);
    //DisplayMatrix("eigenvectors", eigenvectors);

    size.width =  int(sqrt(eigenvalues.at<float>(0))/3000*640);   //如果covMatrix为CV_64FC1型，那么应把float改成double，如下相同
    size.height = int(sqrt(eigenvalues.at<float>(1))/300*480);

    size.width = max(1, size.width)*4;
    size.height = max(1, size.height) *4;


    angle = atan2(eigenvectors.at<float>(0,1),eigenvectors.at<float>(0,0))*180.0/CV_PI;

    //CoordinateTransformImageReal(center,center,0);
    ellipse(image, center, size, angle, 0, 360, CV_RGB(255,0,255));
    return true;
}
*/
double QrSlam::odom_aver(double*  Vodom,int n)
{
    double sum=0;
    for(int i=0;i< sizeof(Vodom);i++)
    {
        sum+=Vodom[i];
    }
    double pvg = sum/sizeof(Vodom);
    return  pvg;
}

/**
 * @brief QrSlam::showImage
 * 过程显示
 * 显示机器人位置  landmark  坐标系
 */
void  QrSlam::showImage()
{
    cout<<"robot_info "<<robot_info_.X<<"  "<<robot_info_.Y<<endl;
    showRobotTriangle(raw_global_map_, robot_info_,CV_RGB(0,0,0)) ;

    //robot_img_cvt_->convertOnce(raw_global_map_);
    showLandmark(raw_global_map_,CV_RGB(0,0,255));

    // flip(raw_global_map_,raw_global_map_flip0_,0);
    drawCoordinate(raw_global_map_);
    slam_img_cvt_->convertOnce(raw_global_map_);
}
/**
 * @brief QrSlam::showRobot
 * 在mat 图片上绘制机器人位置
 * @param map          图片
 * @param robot_info   机器人信息（x,y,theta）
 * @param rgb          绘制时颜色选择
 */
void QrSlam::showRobot(cv::Mat& map, RobotInfo robot_info,Scalar rgb)
{
    const int ROBOT_DEFAULT_RADIUS = 2;
    const int ROBOT_DEFAULT_ARROW_LEN = 10;

    Point start, end;
    start.x = robot_info.X + MAP_BASE_X_;
    start.y = robot_info.Y + MAP_BASE_Y_;
    start.y = map.rows - start.y;

    int thickness = 1;
    int lineType = 8;

    // int radius = 10;
    circle(map,start,ROBOT_DEFAULT_RADIUS,rgb,0,lineType );

    end.x = start.x + ROBOT_DEFAULT_ARROW_LEN * cos(robot_info.Theta);
    //    end.y = start.y + ROBOT_DEFAULT_ARROW_LEN * sin(robot_info.Theta);  //display  y  convert ..
    end.y = start.y - ROBOT_DEFAULT_ARROW_LEN * sin(robot_info.Theta);  //display  y  convert ..

    line( map,start,end,rgb,thickness,lineType );
}
/**
 * @brief QrSlam::showRobotTriangle
 * 在mat 图片上绘制机器人位置，以三角形式表示        -->单独做到一个截框界面显示 每步机器人信息（x,y,theta）
 * @param map          图片
 * @param robot_info   机器人信息（x,y,theta）
 * @param rgb          绘制时颜色选择
 */
void QrSlam::showRobotTriangle(cv::Mat& map, RobotInfo robot_info, Scalar rgb)
{
    float width =3;
    Point robot_pos;
    robot_pos.x = robot_info.X + MAP_BASE_X_;
    robot_pos.y = map.rows-(robot_info.Y + MAP_BASE_Y_);

    Point Pa,Pb,Pc,Pd,Pe;
    float width_cos= width*cos(robot_info.Theta);
    float width_sin= width*sin(robot_info.Theta);

    Pa.x = robot_pos.x + 1.7*width_cos; Pa.y = robot_pos.y + 1.7*width_sin;
    Pd.x = robot_pos.x - 1.7*width_cos; Pd.y = robot_pos.y - 1.7*width_sin;

    Pb.x = Pd.x + 2*width_sin; Pb.y = Pd.y + 2*width_cos;
    Pc.x = Pd.x - 2*width_cos; Pc.y = Pd.y - 2*width_sin;

    Pe.x = robot_pos.x + 3.7*width_cos; Pe.y = robot_pos.y - 3.7*width_sin;

    int thickness = 1;
    int lineType = 8;

    // int radius = 10;
    line( map,Pa,Pe,rgb,thickness,lineType );
    line( map,Pa,Pb,rgb,thickness,lineType );
    line( map,Pa,Pc,rgb,thickness,lineType );
    line( map,Pb,Pc,rgb,thickness,lineType );
}
/**
 * @brief QrSlam::showLandmark
 * 在map上绘制landmark的位置信息。
 * @param map
 * @param rgb
 */
void QrSlam::showLandmark(cv::Mat& map, Scalar rgb)
{
    // cv::Mat map_copy ;
    static bool first = false;
    cout<<miu_state.cols<<" "<<miu_state.rows<<endl;
    int temp_X= miu_state.at<float>(0)+ MAP_BASE_X_;
    int temp_Y= miu_state.at<float>(1)+ MAP_BASE_Y_;
    temp_Y = map.rows - temp_Y ;
    frobot<<" "<<temp_X<<" "<<temp_Y<<endl;
    cv::circle(map,cvPoint( temp_X,temp_Y),3,CV_RGB(0, 255,0),2); //绘制 robot

    for(int t=0;t<observed_landmark_num.size();t++)
    {
        float X= miu_state.at<float>(3+t*2)+ MAP_BASE_X_;
        float Y= miu_state.at<float>(4+t*2)+ MAP_BASE_Y_;
        Y = map.rows - Y ;
        state_miu<<"  "<<t<<" "<<X<<"  "<<Y;
        cv::circle(map,cvPoint( X,Y),1,rgb,2); //绘制mark位置
        std::string text = int2str(observed_landmark_num.at(t)/5);
        if(!first)
        {
            cv::putText(map,text,cvPoint(X,Y+20),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0,0) );
            first = true ; //首次出现作标记
        }
        //        map.copyTo(map_copy);

        //        std::string  X_text = float2str(X);
        //        cv::putText(cv_camera_,X_text,cvPoint( 80*(t+1),80),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 0,255) );

        //        std::string  Y_text = float2str(Y);
        //        cv::putText(cv_camera_,Y_text,cvPoint( 120,80*(t+1)),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 0,255) );

    }
    std::string  num_text = int2str(observed_landmark_num.size());
    cv::putText(cv_camera_,num_text,cvPoint( 40,40),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 0,255) );

    state_miu<<" "<<endl;

    raw_img_cvt_->convertOnce(cv_camera_);

    // map_copy.copyTo(map);
}

void QrSlam::drawCoordinate(cv::Mat& mat)
{
    std::string text ="Y";
    cv::putText(mat,text,cvPoint(20,20),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::line(mat,cv::Point(1,1),cv::Point(1,mat.rows),CV_RGB(255,0,0),1,8);

    text ="O";
    cv::putText(mat,text,cvPoint(20,mat.rows-20),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));

    text ="X";
    cv::putText(mat,text,cvPoint(mat.cols-20,mat.rows-20),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::line(mat,cv::Point(1,mat.rows-1),cv::Point(mat.cols,mat.rows-1),CV_RGB(255,0,0),1,8);
}
/**
 * @brief QrSlam::storeData
 * 保存系统状态量到文件  ofstream fmiu("miu.txt")   中
 */
void QrSlam::storeData(void )
{
    for(int i=0;i<miu_state.rows;++i)
    {
        fmiu<<miu_state.at<float>(i)<<"     " ;
    }
    fmiu<<endl;
}
/**
 * @brief QrSlam::float2str
 * 数据类型转换 float 转换到 string
 * @param num
 * @return
 */
std::string  QrSlam::float2str(float num)
{
    std::stringstream ss;
    ss << num;
    std::string text = ss.str();
    return text;
}
