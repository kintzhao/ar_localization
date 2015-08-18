#include <ros/ros.h>
#include "./class/detectqrcode.h"
#include "./qrslam.h"
#define SELECT_ROBOT_POS_AS_ORIGIN  0 // 0 表示20 mark 为原点
ofstream fmain("main.txt");
int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_slam");
    QrSlam qr_slam(argv[1]);
 //   ros::Rate loop_rate(50);  // 系统频率放开
    bool init_finihed_ = false;
    while (ros::ok())
    {
        if( qr_slam.coordinate_init_finished_)
        {
            qr_slam.initLocalization();
            init_finihed_ = true;
        }
#if  SELECT_ROBOT_POS_AS_ORIGIN
        init_finihed_ = true;
        qr_slam.coordinate_x_ = 0.0 ;
        qr_slam.coordinate_y_ = 0.0 ;
        qr_slam.coordinate_angle_ = 0.0 ;
#endif
        if(init_finihed_ && !qr_slam.init_state_)
        {
            //          if(!qr_slam.init_state_ && qr_slam.is_odom_update && qr_slam.is_img_update)
            // 里程速率大于image   采用缓存的方式
             //    if( qr_slam.is_odom_update)
            fmain<< "odom :  1       "<< (qr_slam.odom_stamp -qr_slam.odom_init_stamp).toSec()
                 <<" img :  2  "<<(qr_slam.image_stamp -qr_slam.odom_init_stamp).toSec()<< endl;
            if( qr_slam.is_img_update_)
            {
                //qr_slam.ekfSlam_old(qr_slam.robot_info_.V,qr_slam.robot_info_.W);
                //qr_slam.ekfSlam(qr_slam.robot_info_.V,qr_slam.robot_info_.W);
                if (qr_slam.data_coordinate())
                {
                    // qr_slam.ekfSlam_record_bag(qr_slam.robot_info_.V,qr_slam.robot_info_.W);
                    qr_slam.ekfSlam_record_bag(qr_slam.best_odom.vel,qr_slam.best_odom.angleVel);
                    qr_slam.storeData();
                    qr_slam.showImage();
                    qr_slam.is_odom_update  = false ;
                    qr_slam.is_img_update_  = false;
                }
            }
        }
        else
           cout<<".";
        ros::spinOnce();
       // loop_rate.sleep(); ///change
    }
    return 0;
}











