
#include <ros/ros.h>
#include "./class/detectqrcode.h"
#include "./qrslam.h"
#define FIRST_ROBOT_POS_AS_ORIGIN  1 // 0 表示20 mark 为原点
int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_slam");
    QrSlam qr_slam(argv[1]);
    ros::Rate loop_rate(50);
    bool init_finihed_ = false;
    while (ros::ok())
    {
        if( qr_slam.coordinate_init_finished_)
        {
            qr_slam.initLocalization();
            init_finihed_ = true;
        }
#if  FIRST_ROBOT_POS_AS_ORIGIN
        init_finihed_ = true;
        qr_slam.coordinate_x_ = 0.0 ;
        qr_slam.coordinate_y_ = 0.0 ;
        qr_slam.coordinate_angle_ = 0.0 ;
#endif
        if(init_finihed_ && !qr_slam.init_state_)
        {
//          if(!qr_slam.init_state_ && qr_slam.is_odom_update && qr_slam.is_img_update)
            if( qr_slam.is_odom_update && qr_slam.is_img_update_)
            {
                qr_slam.ekfSlam(qr_slam.robot_info_.V,qr_slam.robot_info_.W);
                qr_slam.showImage();
                qr_slam.storeData();
                qr_slam.is_odom_update = false ;
                qr_slam.is_img_update_  = false;
            }
        }
        else
           cout<<"..";
        ros::spinOnce();
        loop_rate.sleep(); ///change
    }
    return 0;
}











