#ifndef QRCODELOCALIZER_H
#define QRCODELOCALIZER_H

#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../../include/ARToolKitPlus/TrackerSingleMarker.h"
#include "../../include/ARToolKitPlus/ar.h"
#include "../../image_convert/image_converter.h"



using ARToolKitPlus::TrackerSingleMarker;
using ARToolKitPlus::ARMarkerInfo;
using namespace std;
using namespace cv;

/// struct
typedef struct ConerPoint
{
    double X;
    double Y;
    void init(double x1,double y1)
    {
        X= x1;
        Y= y1;
    }
    void init(ConerPoint x1)
    {
        X=x1.X ;
        Y=x1.Y ;
    }
}ConerPoint;

typedef struct CPointsFour
{
    int ID;
    ConerPoint corn0;
    ConerPoint corn1;
    ConerPoint corn2;
    ConerPoint corn3;
    ConerPoint center;

    void init(ConerPoint p0,ConerPoint p1,ConerPoint p2,ConerPoint p3,ConerPoint p4)
    {
        corn0.init(p0);
        corn1.init(p1);
        corn2.init(p2);
        corn3.init(p3);
        center.init(p4);
    }

}CPointsFour;

typedef struct ConerPointWorld
{
    float X;
    float Y;
    float Z;
    void init(float x1,float y1,float z1)
    {
        X= x1;
        Y= y1;
        Z= z1;
    }
    void init(ConerPointWorld x1)
    {
        X=x1.X ;
        Y=x1.Y ;
        Z=x1.Z ;
    }
}ConerPointWorld;

typedef struct CPointsFourWorld
{
    int ID;
    ConerPointWorld corn0;
    ConerPointWorld corn1;
    ConerPointWorld corn2;
    ConerPointWorld corn3;
    ConerPointWorld center;

    void init(ConerPointWorld p0,ConerPointWorld p1,ConerPointWorld p2,ConerPointWorld p3,ConerPointWorld p4)
    {
        corn0.init(p0);
        corn1.init(p1);
        corn2.init(p2);
        corn3.init(p3);
        center.init(p4);
    }

}CPointsFourWorld;

/// struct
typedef struct Pose3D
{
    double X;
    double Y;
    double Theta;
}Pose3D;

typedef struct CamraInner
{
    double fx;
    double fy;
    double dx;
    double dy;
}CamraInner;

typedef struct QrLandMark
{
int    Id   ;
int    Dir  ;
float  Theta ;
double X;
double Y;
double Side;
}QrLandMark;

typedef std::map <std::string, std::string> MapType;

class DetctQrcode
{
public:
    /// construction & destruction
    DetctQrcode(char * mapFile);
    ~DetctQrcode();

    static const int ar_mode_width  = 640;
    static const int ar_mode_height = 480;
    static const int ar_mode_bpp    = 1;
    static const int undistort  = 1   ;//   图像校正关闭0 / 1开
    int img_W_  ;
    int img_H_  ;

public:
    /// public method
//    vector<QrLandMark> detectLandmarks(cv::Mat image, int &MarkNum, Pose3D robPostion, Mat &qr_img) ;
    vector<CPointsFour> detectLandmarks(cv::Mat image, int &MarkNum) ;
    vector<CPointsFour> detectRawLandmarks(cv::Mat image, int &MarkNum) ;
    //void imTotruePos(double &ar_mode_width, double &ar_mode_height, int id);
    ConerPoint  imTotruePos(double width,double height,int id);
    std::string  int2str(int num);
    void PosToXY(double &ar_mode_width,double &ar_mode_height,double delta);
    CPointsFourWorld getInitMarkMessage(const int id);
    CPointsFourWorld mark5_init_world_;

private:
    /// private method
    std::string arrToStr( const char *c, int i );
    void   createMap( MapType &input_data_, char * filename);
    void   undistortImage( IplImage* frame, cv::Mat & map1_,cv::Mat & map2_);
    void   gradientNormalizePic(cv::Mat &cutout);
    int    flipv(int y);
    int    fliph(int x);
    double getPan(double a1, double b1,double a2, double b2, int dirNum, int side);
    double combinePan(double pan, double pan2);
    double normalizePan(double pan);
    double calcPan(ARMarkerInfo arcode ,vector<int> orients_);
    void   normalizeOrientation(ARMarkerInfo nMarker_info, vector<int> orients_, int &dirNum,int &v1, int &v2, int &side);
    double averageVector(vector<double> data, double multiplier, double shift, double range ,int bucketAmount);
    double sideCalc(ARMarkerInfo Mark_);

    void   loopClear();
    void   ValueToOld(void);
    void   trackerFunction(TrackerSingleMarker *trackerFast);
    void   drawQrcode(void)  ;
    void   drawCoordinate(cv::Mat& mat);

private:
    /// private member
    CamraInner camInnerPara;
    vector<float>   ht_;
    vector<int>     orients_;
    vector<int>     x_off_;
    vector<int>     y_off_;
    vector<int>     side_sizes_;




    MapType         input_data_;
    bool            useBCH ;

private:
    vector<double> pan_res_acc;
    double pos_pan_acc;
    cv::Mat camera_matrix_;
    cv::Mat distcoeffs_ ;
    cv::Mat  map1_,map2_;
    cv::Mat gray ;

    ARMarkerInfo* nMarker_info;
    ARMarkerInfo   Mark_;
    int nNumMarkers;
    double pan, old_pan;

    vector<int> detectedID;
    vector< vector<double> > ID_X;
    vector< vector<double> > ID_Y;
    vector< vector<double> > side_size;
    vector< vector<double> > ID_Theta;

    vector< vector<CPointsFour> > coners;
    vector<CPointsFour>  coners_aver;


    QrLandMark  QrMark;
    QrLandMark  Q_temp ;
    QrLandMark  mark_arrys[20];
    bool        found_code ;
    vector<ARMarkerInfo> MarkerPrototypes;
    vector<QrLandMark>   QrMarks;
    std::vector<int>     markerId ;

public:
    cv::Mat res;
    cv::Mat show_landmark_img_;
    ImageConverter* qr_landmark_cvt_ = NULL;
    TrackerSingleMarker* p_tracker_marks_ =NULL;

    CPointsFour averageCPointsFour(vector<CPointsFour> data, double multiplier, double shift, double range ,int bucketAmount);
    void  imgCornerToWorld(CPointsFour &point_four) ;
    void  imgCornerToWorld(CPointsFour& dst,CPointsFour& src);
    bool readMarkForLocalization(int id) ;
    bool readConfigureMessage(int max_num);
};

#endif // QRCODELOCALIZER_H
