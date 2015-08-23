/*
    局部处理与全局处理的不同是针对的处理图像大小不一样。
    一个是在截取的小块图像，一个是在全幅图像中处理。。小块是通过找到最优Ｍark，利用已知位置列表估计所有mark与Mark的相对位置
    反映到实际像素值中。。在截取相对位置可能出现在同一幅图像中的mark作为visualbal_mark.分别截取可见的mark再进行trace.
*/

#include "detectqrcode.h"
ofstream fthreshold("threshold.txt");
ofstream fPixPos("pixpos.txt");


DetctQrcode::DetctQrcode(char * mapFile)
{
    useBCH = false;
    // display = cv::Mat::zeros(640,480,CV_32F);
    for (int i=0;i<20;i++)
    {
        mark_arrys[i].Id = -1;
    }
    //      用于后面显示地图数据作提前处理
    if ( mapFile != NULL)
    {
        printf("%s <configuration filename>", mapFile);
    }
    createMap( input_data_, mapFile);
    //迭代操作  初始化vextor[0-599]:  ht orients xoff yoff sideSizes
    if (!readConfigureMessage(60))
    {
        cout<<"read configure erro"<<endl;
        waitKey();
    }
    else
        cout<<"configure message read finished!"<<endl;   //
    if (!readMarkForLocalization(20))
    {
        cout<<"check the message of mark 20 in configure"<<endl;
        waitKey();
    }
    cv::FileStorage fs2(input_data_[std::string("cvCalibFile")].c_str(), cv::FileStorage::READ);
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    distcoeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    //获取标定参数
    fs2["Camera_Matrix"] >> camera_matrix_;               //相机矩阵
    fs2["Distortion_Coefficients"] >> distcoeffs_;        //畸变系数
    //相机焦距相关
    camInnerPara.fx = camera_matrix_.at<double>(0,0)/100.0; camInnerPara.fy = camera_matrix_.at<double>(1,1)/100.0;
    camInnerPara.dx = camera_matrix_.at<double>(0,2); camInnerPara.dy = camera_matrix_.at<double>(1,2);
    cv::initUndistortRectifyMap( camera_matrix_,distcoeffs_,cv::Mat(),
                                 cv::getOptimalNewCameraMatrix(camera_matrix_, distcoeffs_, cv::Size(img_W_,img_H_), 1, cv::Size(img_W_,img_H_), 0),
                                 cv::Size(img_W_,img_H_),CV_16SC2,map1_,map2_);

    p_tracker_marks_ = new TrackerSingleMarker(ar_mode_width, ar_mode_height, 5, 6, 6, 6, 0);
    qr_landmark_cvt_ = new ImageConverter("/detect_qr/qr_img");
    fPixPos <<"id corn0_x y corn1_x y coen2_x y corn3_x y center_x y  …… "<<endl;
    if ((p_tracker_marks_ == NULL) || (qr_landmark_cvt_ == NULL))
    {
        exit(1);
    }
}
DetctQrcode::~DetctQrcode()
{
    cv::imwrite("./2d_mark",show_landmark_img_);
    if (!qr_landmark_cvt_)
    {
        delete qr_landmark_cvt_;
    }
    if (!p_tracker_marks_)
    {
        delete p_tracker_marks_;
    }
}
bool DetctQrcode::readConfigureMessage(int max_num)
{
    bool flag = false;

    std::string imgW = "imgW";
    std::string imgH = "imgH";

    MapType::iterator it; //输出相机成像尺寸信息
    it = input_data_.find(imgW.c_str());
    if ( it != input_data_.end() )
        img_W_ = atoi( it->second.c_str() ) ; // Add data to the end of the %vector.

    it = input_data_.find(imgH.c_str());
    if ( it != input_data_.end() )
        img_H_ = atoi( it->second.c_str() ) ; // Add data to the end of the %vector.

    for (int hti = 0 ; hti < max_num ; hti++)
    {
        //配置参数字符等价
        std::string htstr = arrToStr("ht",hti);
        std::string orientsstr = arrToStr("orients",hti);
        std::string xoffstr = arrToStr("xoff",hti);
        std::string yoffstr = arrToStr("yoff",hti);
        std::string sideSizesstr = arrToStr("sideSizes",hti);

        MapType::iterator it;
        it = input_data_.find(htstr.c_str());
        if ( it != input_data_.end() )
            ht_.push_back( atof( it->second.c_str() ) ); // Add data to the end of the %vector.
        else
            ht_.push_back(0.0);

        it = input_data_.find(orientsstr.c_str());
        if ( it != input_data_.end() )
            orients_.push_back( atoi( it->second.c_str() ) );
        else
            orients_.push_back(0.0);

        it = input_data_.find(xoffstr.c_str());
        if ( it != input_data_.end() )
            x_off_.push_back( atoi( it->second.c_str() ) );
        else
            x_off_.push_back(0.0);

        it = input_data_.find(yoffstr.c_str());
        if ( it != input_data_.end() )
            y_off_.push_back( atoi( it->second.c_str() ) );
        else
            y_off_.push_back(0.0);

        it = input_data_.find(sideSizesstr.c_str());
        if ( it != input_data_.end() )
            side_sizes_.push_back( atoi( it->second.c_str() ) );
        else
            side_sizes_.push_back(0.0);
    }
    if (side_sizes_.empty()||ht_.empty()||x_off_.empty()||y_off_.empty())
        flag = false;
    else //configure no empty
    {
        flag = true;
        for ( int hti = 0 ; hti < max_num ; hti++)
        {
            if (ht_[hti]!=0)
            {
                cout<<" Mark Id: "<<hti<<"have been marked.\n"<<endl;
                cout<<" ht["<<hti<<"]="<<ht_[hti]<<endl;
            }
        }
        cout<<"camera width*heigh:"<<img_W_<<" "<<img_H_<<endl;
    }
    return flag;
}
/*
 * bool DetctQrcode::readMarkForLocalization(int id)
 *  将用于初始化的mark（ID=id）点作为起始的原点,
 *  将相应的自定义坐标作为起始世界坐标放到 mark5_init_world_ 中。
 *
 */

bool DetctQrcode::readMarkForLocalization(int id)
{
    bool flag = false;
    std::string corn0x = arrToStr("corn0x",id);
    std::string corn0y = arrToStr("corn0y",id);
    std::string corn0z = arrToStr("corn0z",id);

    std::string corn1x = arrToStr("corn1x",id);
    std::string corn1y = arrToStr("corn1y",id);
    std::string corn1z = arrToStr("corn1z",id);

    std::string corn2x = arrToStr("corn2x",id);
    std::string corn2y = arrToStr("corn2y",id);
    std::string corn2z = arrToStr("corn2z",id);

    std::string corn3x = arrToStr("corn3x",id);
    std::string corn3y = arrToStr("corn3y",id);
    std::string corn3z = arrToStr("corn3z",id);

    std::string centerx = arrToStr("centerx",id);
    std::string centery = arrToStr("centery",id);
    std::string centerz = arrToStr("centerz",id);

    MapType::iterator it = input_data_.find(corn0x.c_str());
    if ( it != input_data_.end() )
    {
        mark5_init_world_.corn0.X = atof( it->second.c_str() ) ;
        flag = true;
    }
    else{
        mark5_init_world_.corn0.X = 0.0;
        flag = false;
        cout<<"the file jx does not have the mark for init,check the message of mark : "<<id<<endl;
        cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "<<endl;
    }
    it = input_data_.find(corn0y.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn0.Y = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn0.Y = 0.0;
    it = input_data_.find(corn0z.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn0.Z = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn0.Z = 0.0;


    it = input_data_.find(corn1x.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn1.X = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn1.X = 0.0;
    it = input_data_.find(corn1y.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn1.Y = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn1.Y = 0.0;
    it = input_data_.find(corn1z.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn1.Z = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn1.Z = 0.0;

    it = input_data_.find(corn2x.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn2.X = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn2.X = 0.0;
    it = input_data_.find(corn2y.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn2.Y = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn2.Y = 0.0;
    it = input_data_.find(corn2z.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn2.Z = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn2.Z = 0.0;


    it = input_data_.find(corn3x.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn3.X = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn3.X = 0.0;
    it = input_data_.find(corn3y.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn3.Y = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn3.Y = 0.0;
    it = input_data_.find(corn3z.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.corn3.Z = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.corn3.Z = 0.0;


    it = input_data_.find(centerx.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.center.X = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.center.X = 0.0;
    it = input_data_.find(centery.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.center.Y = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.center.Y = 0.0;
    it = input_data_.find(centerz.c_str());
    if ( it != input_data_.end() )
        mark5_init_world_.center.Z = atof( it->second.c_str() ) ;
    else
        mark5_init_world_.center.Z = 0.0;
    mark5_init_world_.ID = id;
    return flag ;
}

/*
 * CPointsFourWorld DetctQrcode::getInitMarkMessage(int id)
 *    回调id的二维码初始配置坐标信息
 */
CPointsFourWorld DetctQrcode::getInitMarkMessage(const int id)
{
    return mark5_init_world_;
}

/***
 * 将输入image通过artoolkitplus提取轮廓。ar的功能是对符合2D code的编码轮廓附上
 * 相应的id信息，返回的是像素值，通过成像与实际物理坐标系方向的转化，和成像模型的映射
 * 转化成实际相对尺寸输出。 *
 * *
 * 输入：
 *   cv::Mat image  原始image
 *   int &MarkNum   找到2D Mark数量
 * 返回：
 *   vector<CPointsFour>   关于corners组的vector
 ***
*/
vector<CPointsFour> DetctQrcode::detectLandmarks(cv::Mat image, int &MarkNum)
{
    found_code = false;
    loopClear();
    if (undistort)          //    #define undistort 1
       cv::remap(image, image, map1_, map2_, cv::INTER_LINEAR);
    gray = image.clone();
    cv::cvtColor(image,gray,CV_BGR2GRAY);
    show_landmark_img_ = image.clone();
    cv::rectangle(show_landmark_img_,Point(319,239),Point(321,241),CV_RGB(0,255,0),1,8); //圈取图像中心点
    cv::line(show_landmark_img_,Point(320,0),Point(320,480),CV_RGB(0,0,0),1,8);
    cv::line(show_landmark_img_,Point(0,240),Point(640,240),CV_RGB(0,0,0),1,8);
    //TrackerSingleMarker* ptrackerWhole = new TrackerSingleMarker(ar_mode_width, ar_mode_height, 5, 6, 6, 6, 0);

    // trackerFunction(p_tracker_marks_); // 下面qr提取前期一系列的操作的封装　　ARToolkitplus 处理二维码信息  1000次自动track
    p_tracker_marks_->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    if (!p_tracker_marks_->init("./data/no_distortion.cal", 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR:p_tracker_marks_->init（） failed\n");
    }
    p_tracker_marks_->setPatternWidth(2.0);
    p_tracker_marks_->setBorderWidth(useBCH ? 0.125 : 0.25);    //  const bool useBCH = false;
    p_tracker_marks_->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
    p_tracker_marks_->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
    /// Turns automatic threshold calculation on/off
    p_tracker_marks_->activateAutoThreshold(false);
    //p_tracker_marks_->setNumAutoThresholdRetries(1000);

    int tryCount = 50;
    // int  qr_valid_num =0;
    while(tryCount < 253 && !found_code )
//    while(tryCount < 200)
    {
//        if ( input_data_[std::string("processingType")] == std::string("1") || input_data_[std::string("processingType")] == std::string("3") )
//        {
//            p_tracker_marks_->setThreshold(tryCount+1);
//        }
//        else
//        {
//            tryCount = 300;
//        }
        //Returns:detected markers in image
        /*
          * int 	area
            double 	cf
            int 	dir
            int 	id
            double 	line [4][3]
            double 	pos [2]
            double 	vertex [4][2]
            Detailed Description
            main structure for detected marker.
            Store information after contour detection (in idea screen coordinate, after distorsion compensated).

            Remarks:
                lines are represented by 3 values a,b,c for ax+by+c=0
            Parameters:
                area	number of pixels in the labeled region
                id	marker identitied number
                dir	Direction that tells about the rotation about the marker (possible values are 0, 1, 2 or 3). This parameter makes it possible to tell about the line order of the detected marker (so which line is the first one) and so find the first vertex. This is important to compute the transformation matrix in arGetTransMat().
                cf	confidence value (probability to be a marker)
                pos	center of marker (in ideal screen coordinates)
                line	line equations for four side of the marker (in ideal screen coordinates)
                vertex	edge points of the marker (in ideal screen coordinates)
        */
        p_tracker_marks_->setThreshold(tryCount);
        std::vector<int> markerId = p_tracker_marks_->calc(gray.data, &nMarker_info, &nNumMarkers);
        cout<<"nNumMarkers"<<nNumMarkers<<"  "<<markerId.size()<<endl;
        int value_thre =  p_tracker_marks_->getThreshold();
        fthreshold<<"  "<< value_thre; // 二值化阈值
        tryCount++;
        tryCount = tryCount + 5;
        for (int i = 0 ; i < nNumMarkers ; i++)
        {
            cout<<"nMarker_info[i].id "<<nMarker_info[i].id<<endl;
            if ( (nMarker_info[i].id != -1) && (ht_[nMarker_info[i].id] > 0.1) )
            {
                found_code = true;
                cout<<" found_code = true "<< nMarker_info[i].id<<endl;
                ARMarkerInfo  Mark_info = nMarker_info[i];
                //double robot_theta = calcPan(Mark,orients_) ;
                bool id_was_detected = false;
                for (int ii = 0 ; ii < detectedID.size(); ii++)
                {
/**
 ** 这用 landmark的记录，起始 id_was_detected 为未观测到，将id保存在detectedID中，在vector<vector()>中开辟新vector
 ** 后面再观察有mark,逐个与detectID比较：
 **     出现在里面就放入之前开辟的vector中；
 **     未出现过就将id保存在detectedID中，在vector<vector()>中开辟新vector
**/
                    if (Mark_info.id == detectedID[ii])
                    {
                        id_was_detected = true;
                        ConerPoint  center,corner0,corner1,corner2,corner3 ;
                        center.init(Mark_info.pos[0],Mark_info.pos[1]) ;
                        corner0.init(Mark_info.vertex[(4-Mark_info.dir+0)%4][0],Mark_info.vertex[(4-Mark_info.dir+0)%4][1]) ;
                        corner1.init(Mark_info.vertex[(4-Mark_info.dir+1)%4][0],Mark_info.vertex[(4-Mark_info.dir+1)%4][1]) ;
                        corner2.init(Mark_info.vertex[(4-Mark_info.dir+2)%4][0],Mark_info.vertex[(4-Mark_info.dir+2)%4][1]) ;
                        corner3.init(Mark_info.vertex[(4-Mark_info.dir+3)%4][0],Mark_info.vertex[(4-Mark_info.dir+3)%4][1]) ;

                        CPointsFour points_temp;
                        points_temp.ID = Mark_info.id ;
                        points_temp.init(corner0,corner1,corner2,corner3,center);
                        coners[ii].push_back(points_temp);
                    }
                }
                if (id_was_detected == false)   //主要是为了多阈值处理下（在循环 while(tryCount < 253)下只进来一次）先填充满ID_X容器，确定好detectedID的数量。
                {                       // X_arr的填充顺序是nMarker_info[i]中i的顺序。。
                    detectedID.push_back(Mark_info.id);

                    ConerPoint  center,corner0,corner1,corner2,corner3 ;
                    center.init(Mark_info.pos[0],Mark_info.pos[1]) ;
                    corner0.init(Mark_info.vertex[(4-Mark_info.dir+0)%4][0],Mark_info.vertex[(4-Mark_info.dir+0)%4][1]) ;
                    corner1.init(Mark_info.vertex[(4-Mark_info.dir+1)%4][0],Mark_info.vertex[(4-Mark_info.dir+1)%4][1]) ;
                    corner2.init(Mark_info.vertex[(4-Mark_info.dir+2)%4][0],Mark_info.vertex[(4-Mark_info.dir+2)%4][1]) ;
                    corner3.init(Mark_info.vertex[(4-Mark_info.dir+3)%4][0],Mark_info.vertex[(4-Mark_info.dir+3)%4][1]) ;

                    CPointsFour points_temp;
                    points_temp.init(corner0,corner1,corner2,corner3,center);
                    points_temp.ID = Mark_info.id;
                    vector<CPointsFour> coners_temp;
                    coners_temp.push_back(points_temp);
                    coners.push_back(coners_temp);
                }
            }
        }
        cout<<" found_code = end "<<endl;
    }//mark有效区间结束
    fthreshold<<" "<< endl; // 二值化阈值
    cout<<"coners.size() "<< coners.size() <<endl;
    //average   将填充的vector< vector > 均值化  // vector< vector > 均值化 成 vector
    for (int i = 0 ; i < coners.size(); i++)
    {
        CPointsFour  c_temp;
        c_temp =  averageCPointsFour(coners[i], 10.0,0.0,2.0,6420);
        //c_temp.ID = detectedID[i];
        cout<<"c_temp.ID "<< c_temp.ID <<endl;
        fPixPos <<" "<<c_temp.ID
                <<" "<<c_temp.corn0.X<<" "<<c_temp.corn0.Y<<" "<<c_temp.corn1.X<<" "<<c_temp.corn1.Y<<" "
                <<" "<<c_temp.corn2.X<<" "<<c_temp.corn2.Y<<" "<<c_temp.corn3.X<<" "<<c_temp.corn3.Y<<" "
                <<" "<<c_temp.center.X<<" "<<c_temp.center.Y<<" ";
        imgCornerToWorld(c_temp);                //图像点像素值转换成物理距离值
        coners_aver.push_back(c_temp);
    }
    fPixPos <<" "<<endl;
    cout<<"coners_aver.size() "<< coners_aver.size() <<endl;
    MarkNum = coners_aver.size() ;

    drawCoordinate(show_landmark_img_);
    drawQrcode() ;    //draw the square-qrcode_fourSide

    std::string text1 = "coners_aver_" + int2str(coners_aver.size() );
    cv::putText(show_landmark_img_,text1,Point(60,50),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    std::string text2 = "coners_" + int2str(coners.size());
    cv::putText(show_landmark_img_,text2,Point(60,100),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));

    qr_landmark_cvt_->convertOnce(show_landmark_img_);  //display
    //  return QrMarks;  //返回的值是实际距离值
    return coners_aver;
}


//返回原始像素点坐标
vector<CPointsFour> DetctQrcode::detectRawLandmarks(cv::Mat image, int &MarkNum)
{
    Point2i middle(image.rows,image.rows);
    found_code = false;
    loopClear();
    if (undistort)          //    #define undistort 1
       cv::remap(image, image, map1_, map2_, cv::INTER_LINEAR);
    gray = image.clone();
    cv::cvtColor(image,gray,CV_BGR2GRAY);
    show_landmark_img_ = image.clone();
    cv::rectangle(show_landmark_img_,Point(319,239),Point(321,241),CV_RGB(0,255,0),1,8); //圈取图像中心点
    cv::line(show_landmark_img_,Point(320,0),Point(320,480),CV_RGB(0,0,0),1,8);
    cv::line(show_landmark_img_,Point(0,240),Point(640,240),CV_RGB(0,0,0),1,8);
    //TrackerSingleMarker* ptrackerWhole = new TrackerSingleMarker(ar_mode_width, ar_mode_height, 5, 6, 6, 6, 0);

    // trackerFunction(p_tracker_marks_); // 下面qr提取前期一系列的操作的封装　　ARToolkitplus 处理二维码信息  1000次自动track
    p_tracker_marks_->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    if (!p_tracker_marks_->init("./data/no_distortion.cal", 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR:p_tracker_marks_->init（） failed\n");
    }
    p_tracker_marks_->setPatternWidth(2.0);
    p_tracker_marks_->setBorderWidth(useBCH ? 0.125 : 0.25);    //  const bool useBCH = false;
    p_tracker_marks_->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
    p_tracker_marks_->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
    /// Turns automatic threshold calculation on/off
    p_tracker_marks_->activateAutoThreshold(false);
    //p_tracker_marks_->setNumAutoThresholdRetries(1000);

    int tryCount = 50;
    // int  qr_valid_num =0;
    while(tryCount < 253 && !found_code )
//    while(tryCount < 200)
    {
//        if ( input_data_[std::string("processingType")] == std::string("1") || input_data_[std::string("processingType")] == std::string("3") )
//        {
//            p_tracker_marks_->setThreshold(tryCount+1);
//        }
//        else
//        {
//            tryCount = 300;
//        }
        //Returns:detected markers in image
         p_tracker_marks_->setThreshold(tryCount);
        std::vector<int> markerId = p_tracker_marks_->calc(gray.data, &nMarker_info, &nNumMarkers);
        cout<<"nNumMarkers"<<nNumMarkers<<"  "<<markerId.size()<<endl;
        int value_thre =  p_tracker_marks_->getThreshold();
        fthreshold<<"  "<< value_thre; // 二值化阈值
        tryCount++;
        tryCount = tryCount + 5;
        for (int i = 0 ; i < nNumMarkers ; i++)
        {
            cout<<"nMarker_info[i].id "<<nMarker_info[i].id<<endl;
            if ( (nMarker_info[i].id != -1) && (ht_[nMarker_info[i].id] > 0.1) )
            {
                found_code = true;
                cout<<" found_code = true "<< nMarker_info[i].id<<endl;
                ARMarkerInfo  Mark_info = nMarker_info[i];
                //double robot_theta = calcPan(Mark,orients_) ;
                bool id_was_detected = false;
                for (int ii = 0 ; ii < detectedID.size(); ii++)
                {
                    if (Mark_info.id == detectedID[ii])
                    {
                        id_was_detected = true;
                        if( 20 == Mark_info.id)
                        {
                           Mark_info.dir = Mark_info.dir +1;
                        }

                        ConerPoint  center,corner0,corner1,corner2,corner3 ;
                        center.init(Mark_info.pos[0],Mark_info.pos[1]) ;
                        corner0.init(Mark_info.vertex[(4-Mark_info.dir+0)%4][0],Mark_info.vertex[(4-Mark_info.dir+0)%4][1]) ;
                        corner1.init(Mark_info.vertex[(4-Mark_info.dir+1)%4][0],Mark_info.vertex[(4-Mark_info.dir+1)%4][1]) ;
                        corner2.init(Mark_info.vertex[(4-Mark_info.dir+2)%4][0],Mark_info.vertex[(4-Mark_info.dir+2)%4][1]) ;
                        corner3.init(Mark_info.vertex[(4-Mark_info.dir+3)%4][0],Mark_info.vertex[(4-Mark_info.dir+3)%4][1]) ;

                        CPointsFour points_temp;
                        points_temp.ID = Mark_info.id ;
                        points_temp.init(corner0,corner1,corner2,corner3,center);
                        coners[ii].push_back(points_temp);
                    }
                }
                if (id_was_detected == false)   //主要是为了多阈值处理下（在循环 while(tryCount < 253)下只进来一次）先填充满ID_X容器，确定好detectedID的数量。
                {                       // X_arr的填充顺序是nMarker_info[i]中i的顺序。。
                    detectedID.push_back(Mark_info.id);
                    if( 20 == Mark_info.id)
                    {
                       Mark_info.dir = Mark_info.dir +1;
                    }
                    ConerPoint  center,corner0,corner1,corner2,corner3 ;
                    center.init(Mark_info.pos[0],Mark_info.pos[1]) ;
                    corner0.init(Mark_info.vertex[(4-Mark_info.dir+0)%4][0],Mark_info.vertex[(4-Mark_info.dir+0)%4][1]) ;
                    corner1.init(Mark_info.vertex[(4-Mark_info.dir+1)%4][0],Mark_info.vertex[(4-Mark_info.dir+1)%4][1]) ;
                    corner2.init(Mark_info.vertex[(4-Mark_info.dir+2)%4][0],Mark_info.vertex[(4-Mark_info.dir+2)%4][1]) ;
                    corner3.init(Mark_info.vertex[(4-Mark_info.dir+3)%4][0],Mark_info.vertex[(4-Mark_info.dir+3)%4][1]) ;

                    CPointsFour points_temp;
                    points_temp.init(corner0,corner1,corner2,corner3,center);
                    points_temp.ID = Mark_info.id;
                    vector<CPointsFour> coners_temp;
                    coners_temp.push_back(points_temp);
                    coners.push_back(coners_temp);
                }
            }
        }
        cout<<" found_code = end "<<endl;
    }//mark有效区间结束
    fthreshold<<" "<< endl; // 二值化阈值
    cout<<" the num of codes : "<< coners.size() <<endl;
    //average   将填充的vector< vector > 均值化  // vector< vector > 均值化 成 vector
    for (int i = 0 ; i < coners.size(); i++)
    {
        CPointsFour  c_temp;
        c_temp =  averageCPointsFour(coners[i], 10.0,0.0,2.0,6420);
        //c_temp.ID = detectedID[i];
        cout<<"c_temp.ID "<< c_temp.ID <<endl;
        fPixPos <<" "<<c_temp.ID
                <<" "<<c_temp.corn0.X<<" "<<c_temp.corn0.Y<<" "<<c_temp.corn1.X<<" "<<c_temp.corn1.Y<<" "
                <<" "<<c_temp.corn2.X<<" "<<c_temp.corn2.Y<<" "<<c_temp.corn3.X<<" "<<c_temp.corn3.Y<<" "
                <<" "<<c_temp.center.X<<" "<<c_temp.center.Y<<" ";
        //imCornerToWorld(c_temp);                //图像点像素值转换成物理距离值
        coners_aver.push_back(c_temp);
        float line0 = sqrt( (c_temp.corn0.X - c_temp.corn1.X) * (c_temp.corn0.X - c_temp.corn1.X) + (c_temp.corn0.Y - c_temp.corn1.Y) * (c_temp.corn0.Y - c_temp.corn1.Y) );
        float line1 = sqrt( (c_temp.corn1.X - c_temp.corn2.X) * (c_temp.corn1.X - c_temp.corn2.X) + (c_temp.corn1.Y - c_temp.corn2.Y) * (c_temp.corn1.Y - c_temp.corn2.Y) );
        float line2 = sqrt( (c_temp.corn2.X - c_temp.corn3.X) * (c_temp.corn2.X - c_temp.corn3.X) + (c_temp.corn2.Y - c_temp.corn3.Y) * (c_temp.corn2.Y - c_temp.corn3.Y) );
        float line3 = sqrt( (c_temp.corn3.X - c_temp.corn0.X) * (c_temp.corn3.X - c_temp.corn0.X) + (c_temp.corn3.Y - c_temp.corn0.Y) * (c_temp.corn3.Y - c_temp.corn0.Y) );
    cout<<" === "<<line0<<" "<<line1 <<" "<<line2 <<" "<<line3 <<" === "<<endl;
    }
    fPixPos <<" "<<endl;
    cout<<"output the num of codes :  "<< coners_aver.size() <<endl;
    MarkNum = coners_aver.size() ;

    drawCoordinate(show_landmark_img_);
    drawQrcode() ;    //draw the square-qrcode_fourSide

    std::string text1 = "coners_aver_" + int2str(coners_aver.size() );
    cv::putText(show_landmark_img_,text1,Point(60,50),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    std::string text2 = "coners_" + int2str(coners.size());
    cv::putText(show_landmark_img_,text2,Point(60,100),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));

    qr_landmark_cvt_->convertOnce(show_landmark_img_);  //display
    //  return QrMarks;  //返回的值是实际距离值
    return coners_aver;
}


void DetctQrcode::drawQrcode(void)   //draw the square-qrcode_fourSide
{
    for (int count = 0 ; count < nNumMarkers; count++)
    {
        Mark_ = nMarker_info[count];
        if (Mark_.id != -1 && ht_[Mark_.id] > 0.4 )
        {
            cv::line(show_landmark_img_,cv::Point(Mark_.vertex[0][0],Mark_.vertex[0][1]),cv::Point(Mark_.vertex[1][0],Mark_.vertex[1][1]),CV_RGB(255,0,0),1,8);
            cv::line(show_landmark_img_,cv::Point(Mark_.vertex[1][0],Mark_.vertex[1][1]),cv::Point(Mark_.vertex[2][0],Mark_.vertex[2][1]),CV_RGB(255,0,0),1,8);
            cv::line(show_landmark_img_,cv::Point(Mark_.vertex[2][0],Mark_.vertex[2][1]),cv::Point(Mark_.vertex[3][0],Mark_.vertex[3][1]),CV_RGB(255,0,0),1,8);
            cv::line(show_landmark_img_,cv::Point(Mark_.vertex[3][0],Mark_.vertex[3][1]),cv::Point(Mark_.vertex[0][0],Mark_.vertex[0][1]),CV_RGB(255,0,0),1,8);

            string dir_str = int2str(Mark_.dir);
            string tx0 = "0";//int2str((4-Mark_.dir+0)%4);
            string tx1 = "1";//int2str((4-Mark_.dir+1)%4);
            string tx2 = "2";//int2str((4-Mark_.dir+2)%4);
            string tx3 = "3";//int2str((4-Mark_.dir+3)%4);
            if( 20 == Mark_.id)
            {
               Mark_.dir = Mark_.dir +1;
               dir_str = int2str(Mark_.dir);
               tx0 = "0";//int2str((4-Mark_.dir+0)%4);
               tx1 = "1";//int2str((4-Mark_.dir+1)%4);
               tx2 = "2";//int2str((4-Mark_.dir+2)%4);
               tx3 = "3";//int2str((4-Mark_.dir+3)%4);
            }
            cv::putText(show_landmark_img_,dir_str,cv::Point(400,400),CV_FONT_HERSHEY_COMPLEX,2,CV_RGB(0,0,255));
            cv::putText(show_landmark_img_,tx0,cv::Point(Mark_.vertex[(4-Mark_.dir+0)%4][0],Mark_.vertex[(4-Mark_.dir+0)%4][1]),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
            cv::putText(show_landmark_img_,tx1,cv::Point(Mark_.vertex[(4-Mark_.dir+1)%4][0],Mark_.vertex[(4-Mark_.dir+1)%4][1]),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
            cv::putText(show_landmark_img_,tx2,cv::Point(Mark_.vertex[(4-Mark_.dir+2)%4][0],Mark_.vertex[(4-Mark_.dir+2)%4][1]),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
            cv::putText(show_landmark_img_,tx3,cv::Point(Mark_.vertex[(4-Mark_.dir+3)%4][0],Mark_.vertex[(4-Mark_.dir+3)%4][1]),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));

            std::string text ="ID_"+ int2str(Mark_.id);
            cv::putText(show_landmark_img_,text,Point(Mark_.pos[0]+80,Mark_.pos[1]),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));

            cv::rectangle(show_landmark_img_,Point(Mark_.pos[0]-1,Mark_.pos[1]-1),Point(Mark_.pos[0]+1,Mark_.pos[1]+1),CV_RGB(0,255,0),1,8); //圈取图像中心点

            //        cv::line(show_landmark_img_,cvPoint(Mark_.pos[0],0),cvPoint(Mark_.pos[0],460),CV_RGB(255,0,0),1,8);
            //        cv::line(show_landmark_img_,cvPoint(0,Mark_.pos[1]),cvPoint(620,Mark_.pos[1]),CV_RGB(255,0,0),1,8);
        }
        count++;
    }
    //qr_landmark_cvt_->convertOnce(show_landmark_img_);
}

void DetctQrcode::drawCoordinate(cv::Mat& mat)
{
    std::string text ="X";
    cv::putText(mat,text,Point(20,mat.rows-20),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::line(mat,cv::Point(1,1),cv::Point(1,mat.rows),CV_RGB(255,0,0),1,8);

    text ="O";
    cv::putText(mat,text,Point(20,20),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));

    text ="Y";
    cv::putText(mat,text,Point(mat.cols-20,20),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::line(mat,cv::Point(1,1),cv::Point(mat.cols,1),CV_RGB(255,0,0),1,8);
}

std::string  DetctQrcode::int2str(int num)
{
    std::stringstream ss;
    ss << num;
    std::string text = ss.str();
    return text;
}

void DetctQrcode::createMap( MapType &inputData, char * filename)
{
    std::string line;
    std::string startBraceEnd(">");
    std::string endBraceStart("</");
    std::ifstream infile(filename);
    while (std::getline(infile, line))
    {
        bool readFine = false;
        if (line.size() > 0 && line[0] == '<' && line[line.size()-1] == '>' )
        {
            std::size_t pSBE = line.find(startBraceEnd);
            std::size_t pEBS = line.find(endBraceStart);
            if ( pSBE!=std::string::npos && pEBS!=std::string::npos && pSBE < pEBS)
            {
                std::string marker1 = line.substr(1,pSBE-1);
                std::string marker2 = line.substr(pEBS+2,line.size()-pEBS-3);
                if ( marker1 == marker2 )
                {
                    std::string content = line.substr(pSBE+1,pEBS-pSBE-1);
                    inputData.insert( pair<string, string>(marker1, content) );
                    readFine = true;
                }
            }
        }
        if ( readFine == true )
        {
            cout<<line<<" READ FINE "<<endl;
        }
        else
        {
            cout<<line<<" ERROR ERROR ERROR !!!!!!!!!!!!! "<<endl;
        }
    }
}
void DetctQrcode::undistortImage( IplImage* frame, cv::Mat & map1,cv::Mat & map2)
{
    cout << "frame: " << frame->width << " x " << frame->height << endl;
    cv::Mat viewDistorted(frame,false);
    cv::Mat viewRectified;
    // warps the image using the precomputed maps.
    cv::remap(viewDistorted, viewRectified, map1, map2, cv::INTER_LINEAR);
    cv::Mat FM(frame);
    viewRectified.copyTo(FM);
}
std::string DetctQrcode::arrToStr(const char * c, int i )
{
    std::stringstream ss;
    ss << c << i ;
    std::string res;
    ss >> res;
    return res;
}
void   DetctQrcode::gradientNormalizePic(cv::Mat &cutout)
{
    double rowsDiff = 0;
    for ( int x = 0; x < cutout.cols; x++ )
    {
        int diffCounter = 0;
        int diffAcc = 0;
        for ( int y = 1; y < cutout.rows; y++ )
        {
            int diff = cutout.at<uchar>(y,x) - cutout.at<uchar>(y-1,x) ;
            if ( abs(diff) < 5 )
            {
                diffAcc += diff;
                diffCounter++;
            }
        }
        rowsDiff += (double) diffAcc / (double) diffCounter;
    }

    rowsDiff = (double) rowsDiff / (double) cutout.cols;


    double colsDiff = 0;
    for ( int y = 0; y < cutout.rows; y++ )
    {
        int diffCounter = 0;
        int diffAcc = 0;
        for ( int x = 1; x < cutout.cols; x++ )
        {
            int diff = cutout.at<uchar>(y,x) - cutout.at<uchar>(y,x-1) ;
            if ( abs(diff) < 5 )
            {
                diffAcc += diff;
                diffCounter++;
            }
        }
        colsDiff += (double) diffAcc / (double) diffCounter;
    }
    colsDiff = (double) colsDiff / (double) cutout.rows ;
    // 均值行列差分作为起始项
    double startdr = rowsDiff * cutout.rows / 2.0 ;
    double startdc = colsDiff * cutout.cols / 2.0 ;

    for ( int x = 0; x < cutout.cols; x++ )
    {
        double adjr = startdr;
        for ( int y = 0; y < cutout.rows; y++ )
        {
            int newVal = ( (double) cutout.at<uchar>(y,x) ) + adjr;
            newVal = newVal >= 0  ? newVal : 0;
            newVal = newVal < 256 ? newVal :255;
            cutout.at<uchar>(y,x) = newVal;
            adjr -= rowsDiff;
        }
    }

    for ( int y = 0; y < cutout.rows; y++ )
    {
        double adjc = startdc;
        for ( int x = 1; x < cutout.cols; x++ )
        {
            int newVal = ( (double) cutout.at<uchar>(y,x) ) + adjc;
            newVal = newVal >= 0  ? newVal : 0;
            newVal = newVal < 256 ? newVal :255;
            cutout.at<uchar>(y,x) = newVal;
            adjc -= colsDiff;

        }
    }
}
int    DetctQrcode::flipv(int y)
{
    return (y-240) * -1.0 + 240;
}
int    DetctQrcode::fliph(int x)
{
    return (x-320) * -1.0 + 320;
}
double DetctQrcode::getPan(double a1, double b1,double a2, double b2, int dirNum, int side)
{
    double pan = atan(-1.0 * a1 / b1 ) + 1.57079632679;


    if (dirNum == 0 || dirNum == 1 )
        pan = pan + 3.141592653589793;

    if ( (pan > 2.7 || pan < .4 )  && ( dirNum == 0 || dirNum == 3) )
    {
        pan = atan(-1.0 * a2 / b2 );
    }

    if ( (pan > 2.7 || pan < .4 )  && ( dirNum == 1 || dirNum == 2) )
    {
        pan = atan(-1.0 * a2 / b2 )+ 3.141592653589793;
    }

    if (dirNum == 0 && pan < 1.58 && pan > .3)
    {
        pan = -1.0 * pan;
    }

    return pan;

}
double DetctQrcode::combinePan(double pan, double pan2)
{
    if (fabs(pan-pan2) > 5 )
    {
        if (pan > pan2)
        {
            pan2 = pan2 + 3.141592653589793 * 2.0;
        }
        else
        {
            pan = pan + 3.141592653589793 * 2.0;
        }
    }

    return (pan + pan2 ) / 2.0;
}
double DetctQrcode::normalizePan(double pan)
{
    if (pan < -3.141592653589793)
        pan = pan + 3.141592653589793 * 2.0;

    if (pan > 3.141592653589793)
        pan = pan - 3.141592653589793 * 2.0;

    return pan;
}
double DetctQrcode::calcPan(ARMarkerInfo arcode /*nMarker_info*/,vector<int> orients)
{
    /*   for (int i=0;i<4;i++)
    {
       cout<<"------mark:"<<arcode.id<<"  arcode[line]a b:"<<arcode.line[i][0]<<"  "<<arcode.line[i][1]<< endl;
       cout<<"arcode.vertex x y:"<<arcode.vertex[i][0]<<"  "<<arcode.vertex[i][1]<< endl;
       cout<< endl;
    }
    cout<<"arcode"<<arcode.id<<"-----dir: =" <<arcode.dir<< endl;
*/
    double pan1A =        atan( arcode.line[0][1] / arcode.line[0][0] );
    double pan3A =        atan( arcode.line[2][1] / arcode.line[2][0] );
    double pan2A = atan( -1.0 * arcode.line[1][0] / arcode.line[1][1] );
    double pan4A = atan( -1.0 * arcode.line[3][0] / arcode.line[3][1] );

    double pan1B = atan( -1.0 * arcode.line[0][0] / arcode.line[0][1] );
    double pan3B = atan( -1.0 * arcode.line[2][0] / arcode.line[2][1] );
    double pan2B =        atan( arcode.line[1][1] / arcode.line[1][0] );
    double pan4B =        atan( arcode.line[3][1] / arcode.line[3][0] );

    //    cout<<"pan1A(b/a) "<<pan1A*57.3<<"---pan1B-(-a/b)-: =" <<pan1B*57.3<< endl;
    double absPanA = 0;
    double absPanB = 0;
    double panA = 0;
    double panB = 0;

    if ( arcode.dir % 2 == 0 )
    {
        absPanA = fabs(pan1A) + fabs(pan3A);
        absPanB = fabs(pan1B)+ fabs(pan3B);
        panA = pan1A + pan3A;
        panB = pan1B + pan3B;
    }
    else
    {
        absPanA = fabs(pan2A) + fabs(pan4A);
        absPanB = fabs(pan2B)+ fabs(pan4B);
        panA = pan2A + pan4A;
        panB = pan2B + pan4B;
    }

    panA /= -2.0;
    panB /= -2.0;
    panB += M_PI / 2.0;
    double pan = (absPanA > absPanB ? panB: panA);

    pan += (arcode.dir-3 )* M_PI / 2.0;
    pan = normalizePan(pan);
    pan = pan * -1.0;

    pan += orients[ arcode.id ] * M_PI / 2.0 - M_PI;

    while(pan > M_PI * 3.0 / 2.0)
        pan = pan - M_PI * 2.0;

    while(pan < -1.0 * M_PI / 2.0)
        pan = pan + M_PI * 2.0;
    return pan;

}

/*与标记的旋转方向有关    标记边的方向号
Direction that tells about the rotation about the marker (possible values are 0, 1, 2 or 3).
This parameter makes it possible to tell about the line order of the detected marker
(so which line is the first one) and so find the first vertex.
*/
void   DetctQrcode::normalizeOrientation(ARMarkerInfo Marker_info, vector<int> orients, int &dirNum,int &v1, int &v2, int &side)
{
    dirNum = ( Marker_info.dir - orients[ Marker_info.id ] + 3) % 4;

    v1 = 0;
    v2 = 1;
    side = 0;

    if (dirNum == 0)
    {
        v1 = 0;
        v2 = 1;
        side = 2;
    }
    else if (dirNum == 1)
    {
        v1 = 3;
        v2 = 0;
        side = 1;
    }
    else if (dirNum == 2)
    {
        v1 = 2;
        v2 = 3;
        side = 0;
    }
    else
    {
        v1 = 1;
        v2 = 2;
        side = 3;
    }
}
//均值换算  加了滤波。最大值滤波
/* 找到最大的值与相应的指数。。范围判断。累加求和
double QRCodeLocalizer::avgMode(vector<double> data, double multiplier, double shift, double range ,int bucketAmount)
{

    int buckets[bucketAmount];
    for (int i = 0;i< bucketAmount;i++)
        buckets[i]=0;

    for (int i = 0 ; i < data.size();i++)
    {
        int index = data[i]*multiplier+shift;
        buckets[index] = buckets[index]+1;
    }

    int maxBucket = -1;
    int maxBucketIndex = 0;
    for (int i = 0;i< bucketAmount;i++)
    {
        if ( maxBucket < buckets[i] )
        {
            maxBucket = buckets[i];
            maxBucketIndex = i;
        }
    }

    int count = 0;
    double accum = 0.0;
    for (int i = 0 ; i < data.size();i++)
    {
        if ( fabs ( ( (double) maxBucketIndex ) - ( data[i] * multiplier + shift ) ) < range )
        {
            count++;
            accum = accum + data[i];
        }
    }

    return  accum / ( (double) count );
}
*/

double DetctQrcode::averageVector(vector<double> data, double multiplier, double shift, double range ,int bucketAmount)
{
    int count = 0;
    double accum = 0.0;
    for (int i = 0 ; i < data.size();i++)
    {
        count++;
        accum = accum + data[i];
    }
    return  accum / count ;
}

/*边长计算  矩形四个顶点  */
double DetctQrcode::sideCalc(ARMarkerInfo Mark)
{
    double ddAcc = 0.0;
    double xd,yd,dd;
    for ( int i = 0 ; i < 4 ; i++ )
    {
        xd = Mark.vertex[i][0] - Mark.vertex[(i+1)%4][0];
        yd = Mark.vertex[i][1] - Mark.vertex[(i+1)%4][1];
        dd = sqrt(xd*xd + yd*yd);
        ddAcc = ddAcc + dd;
    }
    return ddAcc / 4.0;
}

void DetctQrcode::loopClear()
{
    detectedID.clear();
    ID_X.clear();
    ID_Y.clear();
    side_size.clear();
    ID_Theta.clear();
    //thresholds.clear();
    MarkerPrototypes.clear();
    pan_res_acc.clear();
    QrMarks.clear();
    coners.clear();
    coners_aver.clear();
}

void DetctQrcode::trackerFunction(TrackerSingleMarker *ptrackerFast)
{
    ptrackerFast->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    // load a camera file.
    if (!ptrackerFast->init("./data/no_distortion.cal", 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR: init() failed\n");
    }
    ptrackerFast->setPatternWidth(2.0);
    ptrackerFast->setBorderWidth(useBCH ? 0.125 : 0.25);    //  const bool useBCH = false;
    ptrackerFast->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
    ptrackerFast->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
    //ptrackerFast->setNumAutoThresholdRetries(1000);
    /// Turns automatic threshold calculation on/off
    ptrackerFast->activateAutoThreshold(true);
    ptrackerFast->setNumAutoThresholdRetries(1000);
}
/*
 * width (x)
 * height(y)
//旋转是相对
void DetctQrcode::imTotruePos(double &width,double &height,double theta)
{

    double centXoff = width-320;
//    double centYoff = 240-height;  //垂直镜像
    double centYoff = height - 240;  //垂直镜像


    width  = centXoff * ht_[Mark.id] / camInnerPara.fx;
    height = centYoff * ht_[Mark.id] / camInnerPara.fy;

    Q_temp.X = width ;
    Q_temp.Y = height;

    double xx= width*cos(theta) - height*sin(theta);
    double yy= width*sin(theta) + height*cos(theta);
    width = xx;
    height = yy;
}
*/
/**
 * @brief DetctQrcode::imCornerToWorld
 * 将图像提取的像素坐标值转换成实际的物理距离值
 * 相应数值的显示
 * @param point_four   五点集
 */
void DetctQrcode::imgCornerToWorld(CPointsFour &point_four)
{
    point_four.corn0 = imTotruePos( point_four.corn0.X, point_four.corn0.Y,point_four.ID);
    point_four.corn1 = imTotruePos( point_four.corn1.X, point_four.corn1.Y,point_four.ID);
    point_four.corn2 = imTotruePos( point_four.corn2.X, point_four.corn2.Y,point_four.ID);
    point_four.corn3 = imTotruePos( point_four.corn3.X, point_four.corn3.Y,point_four.ID);
    point_four.center = imTotruePos(point_four.center.X,point_four.center.Y,point_four.ID);
}

/**
 * @brief DetctQrcode::imCornerToWorld
 * 将图像提取的像素坐标值转换成实际的物理距离值
 * 相应数值的显示
 * @param point_four   五点集
 */
void DetctQrcode::imgCornerToWorld(CPointsFour& dst,CPointsFour& src)
{
    dst.corn0 = imTotruePos( src.corn0.X, src.corn0.Y,src.ID);
    dst.corn1 = imTotruePos( src.corn1.X, src.corn1.Y,src.ID);
    dst.corn2 = imTotruePos( src.corn2.X, src.corn2.Y,src.ID);
    dst.corn3 = imTotruePos( src.corn3.X, src.corn3.Y,src.ID);
    dst.center = imTotruePos(src.center.X,src.center.Y,src.ID);
    dst.ID = src.ID;
}


//旋转是相对
/**
 * @brief DetctQrcode::imTotruePos
 * 根据几何成像模型把图像像素值转换成物理值
 * @param width     宽(y向)--> x
 * @param height    高（x向）--> y
 * @param id
 */

//void DetctQrcode::imTotruePos(double &width,double &height,int id)
//{
//    double centXoff = height - camInnerPara.dy;
//    double centYoff = camInnerPara.dx - width ;          //采取内参校正数值
////    double centXoff = height -240;
////    double centYoff = 320 - width ;
//    //double camInnerFocus =sqrt(camInnerPara.fx*camInnerPara.fx + camInnerPara.fy* camInnerPara.fy) ;
//    double x_cm  = centXoff * ht_[id] / camInnerPara.fy; //camInnerFocus;
//    double y_cm =  centYoff * ht_[id] / camInnerPara.fx; //camInnerFocus;

//    width  = x_cm;
//    height = y_cm;       // X-Height   Y-Width
//}

ConerPoint DetctQrcode::imTotruePos(double width,double height,int id)
{
    double centXoff = height - camInnerPara.dy;
    double centYoff = camInnerPara.dx - width ;          //采取内参校正数值

    double x_cm  = centXoff * ht_[id] / camInnerPara.fy; //camInnerFocus;
    double y_cm =  centYoff * ht_[id] / camInnerPara.fx; //camInnerFocus;

    ConerPoint cornPoint;
    cornPoint.init(x_cm,y_cm);
   return  cornPoint;
}



void DetctQrcode::PosToXY(double &width,double &height,double delta)
{
    double xx = width*cos(delta)+height*sin(delta);
    double yy = -width*sin(delta)+height*cos(delta);
    width = xx;
    height = yy;
}

CPointsFour DetctQrcode::averageCPointsFour(vector<CPointsFour> data, double multiplier, double shift, double range ,int bucketAmount)
{
    int count = 0;
    int qr_id = -1;
    ConerPoint  center,corner0,corner1,corner2,corner3 ;
    center.init(0,0);corner0.init(0,0);corner1.init(0,0);corner2.init(0,0);corner3.init(0,0);
    for (int i = 0 ; i < data.size();i++)
    {
        count++;
        //qr_id += data[i].ID;
        qr_id = data[i].ID;
        center.X += data[i].center.X ;
        center.Y += data[i].center.Y ;
        corner0.X += data[i].corn0.X ;
        corner0.Y += data[i].corn0.Y ;
        corner1.X += data[i].corn1.X ;
        corner1.Y += data[i].corn1.Y ;
        corner2.X += data[i].corn2.X ;
        corner2.Y += data[i].corn2.Y ;
        corner3.X += data[i].corn3.X ;
        corner3.Y += data[i].corn3.Y ;
    }
    center.X =    center.X / count ;
    center.Y =    center.Y / count ;
    corner0.X =  corner0.X / count ;
    corner0.Y =  corner0.Y / count ;
    corner1.X =  corner1.X / count ;
    corner1.Y =  corner1.Y / count ;
    corner2.X =  corner2.X / count ;
    corner2.Y =  corner2.Y / count ;
    corner3.X =  corner3.X / count ;
    corner3.Y =  corner3.Y / count ;
    CPointsFour points_temp;
    points_temp.init(corner0,corner1,corner2,corner3,center);
    points_temp.ID = qr_id;
    return points_temp;
}
