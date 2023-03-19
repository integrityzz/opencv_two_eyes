#include <iostream>
#include "CameraApi.h" //相机SDK的API头文件
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
using namespace cv;
using namespace std;
using namespace xfeatures2d;
Mat yang = imread("E:/bishe/zhengyangben/yang.jpg");
Mat yin = imread("E:/bishe/zhengyangben/yin.jpg");
double angle;//

class ImageFeature {
public:
    void flann_demo(Mat& image, Mat& image2);
};
int  Camerinit_ok() {
    tSdkCameraDevInfo sCameraList[10];
    char g_CameraName[64];
    int iCameraNums;
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;
    BYTE *m_pFrameBuffer;
    tSdkCameraCapbility sCameraInfo;
    CameraSdkStatus status;
    CameraHandle m_hCamera;  //创建句柄
    iCameraNums = 10;
    //枚举相机，最多返回10个相机的描述信息
    status= CameraEnumerateDevice(sCameraList,&iCameraNums);
    if(status!= CAMERA_STATUS_SUCCESS||iCameraNums == 0){
        printf("device_error\n");
    }
    //如果只有一个相机， iCameraNums会被CameraEnumerateDevice内部修改为1。
    status = CameraInit(&sCameraList[0], -1, -1, &m_hCamera);
    if(status!=CAMERA_STATUS_SUCCESS){
        printf("init_error\n");
        char msg[128];
        sprintf_s(msg,"Failed to init the camera! Error code is %d\n",status);
        printf(msg);
    }
    /* pbyBuffer 由SDK内部自动申请好了内存，存放了原始RAW数据*/
    CameraGetCapability(m_hCamera,&sCameraInfo);//"获得该相机的特性描述"
    m_pFrameBuffer = (BYTE *)CameraAlignMalloc(sCameraInfo.sResolutionRange.iWidthMax*sCameraInfo.sResolutionRange.iWidthMax*3,16);
    //sResolutionRange;  分辨率范围描述
    CameraPlay(m_hCamera);//启动相机
    CameraSetIspOutFormat(m_hCamera,CAMERA_MEDIA_TYPE_MONO8);
    //CameraImageProcess 输出的图像就是 8bit 的灰度图像了， 1 个像素占用 1 个字节，依次排列。0000000000000000
    strcpy_s(g_CameraName,sCameraList[0].acFriendlyName);//char acFriendlyName[64]; 初始化接口使用的设备昵称
    CameraSetTriggerMode(m_hCamera,1);//通过 CameraSetTriggerMode(hCamera,1);切换为软触发取图模式。
    CameraSoftTrigger(m_hCamera);//软触发取图一次
    status=CameraGetImageBuffer(m_hCamera, &sFrameInfo, &pbyBuffer, 1000);
    if (status != CAMERA_STATUS_SUCCESS) {
        /* m_pFrameBuffer需要先申请好内存pbyBuffer 转换后的图像数据保存在m_pFrameBuffer 中，
        默认会得到BRG24bit的图像数据*/
        printf("get_error\n");
        char msg[128];
        sprintf_s(msg,"Failed to get the image! Error code is %d\n",status);
        printf(msg);
    }
    CameraImageProcess(m_hCamera, pbyBuffer, m_pFrameBuffer, &sFrameInfo);
    //2，3参数是输入、输出缓冲数据区，4参数：输入图像的帧头信息，处理完成后，帧头信息中的图像格式 uiMediaType 会随之改变
    CameraReleaseImageBuffer(m_hCamera, pbyBuffer);
    //CameraReleaseImageBuffer释放由CameraGetImageBuffer得到的缓冲区
    CameraSaveImage(m_hCamera,(char*)"E:/bishe/cal_images/file_images.jpg",m_pFrameBuffer,&sFrameInfo,1,100);
    CameraStop(m_hCamera);
    CameraUnInit(m_hCamera);
    //程序退出前调用
    return 0;
}//相机取一张图
int demarcate_camer() {
    string dir = "E:/bishe/cal_images/";  //标定图片所在文件夹
    ifstream fin(dir + "file_images.txt"); //读取标定图片的路径，与cpp程序在同一路径下
    if (!fin)                              //检测是否读取到文件
    {
        cerr << "没有找到文件" << endl;
        return -1;
    }
    ofstream fout(dir + "calibration_result.txt"); //输出结果保存在此文本文件下
    //依次读取每一幅图片，从中提取角点
    cout << "beginning" << endl;
    int image_nums = 0;  //图片数量
    cv::Size image_size; //图片尺寸
    int points_per_row = 12;  //每行的内点数
    int points_per_col = 9;   //每列的内点数
    cv::Size corner_size = cv::Size(points_per_row, points_per_col); //标定板每行每列角点个数，共12*9个角点
    vector<cv::Point2f> points_per_image;                            //缓存每幅图检测到的角点
    points_per_image.clear();
    vector<vector<cv::Point2f>> points_all_images;                   //用一个二维数组保存检测到的所有角点
    string image_file_name;                                          //声明一个文件名的字符串

    while (getline(fin, image_file_name)) //逐行读取，将行读入字符串
    {
        image_nums++;
        //读入图片
        cv::Mat image_raw = cv::imread(image_file_name);//cv::imread读取图片
        cv::imshow("Camera calibration", image_raw);
        cv::waitKey(0); //等待按键输入
        if (image_nums == 1) {
            cout<<"channels = "<<image_raw.channels()<<endl;
            cout<<image_raw.type()<<endl;  //CV_8UC3
            image_size.width = image_raw.cols;  //图像的宽对应着列数
            image_size.height = image_raw.rows; //图像的高对应着行数
            cout << "image_size.width = " << image_size.width << endl;
            cout << "image_size.height = " << image_size.height << endl;
        }
        //角点检测
        cv::Mat image_gray;                               //存储灰度图的矩阵
        cv::cvtColor(image_raw, image_gray, CV_BGR2GRAY); //将BGR图转化为灰度图
        // cout<<"image_gray.type() = "<<image_gray.type()<<endl;  //CV_8UC1
        //step1 提取角点
        bool success = cv::findChessboardCorners(image_gray, corner_size, points_per_image);//角点检测
        if (!success) {
            cout << "can not find the corners " << endl;
            exit(1);
        } else {
            //亚像素精确化（两种方法）
            //step2 亚像素角点
            cv::find4QuadCornerSubpix(image_gray, points_per_image, cv::Size(5, 5));
            // cornerSubPix(image_gray,points_per_image,Size(5,5));
            points_all_images.push_back(points_per_image); //保存亚像素角点
            //在图中画出角点位置
            //step3 角点可视化
            cv::drawChessboardCorners(image_raw, corner_size, points_per_image, success); //将角点连线
            cv::imshow("Camera calibration", image_raw);
            cv::waitKey(0); //等待按键输入
        }
    }
    cv::destroyAllWindows();
    //输出图像数目
    int image_sum_nums = points_all_images.size();
    cout << "image_sum_nums = " << image_sum_nums << endl;

    //开始相机标定
    cv::Size block_size(21, 21);                            //每个小方格实际大小, 只会影响最后求解的平移向量t
    cv::Mat camera_K(3, 3, CV_32FC1, cv::Scalar::all(0));   //内参矩阵3*3
    cv::Mat distCoeffs(1, 5, CV_32FC1, cv::Scalar::all(0)); //畸变矩阵1*5
    vector<cv::Mat> rotationMat;                            //旋转矩阵
    vector<cv::Mat> translationMat;                         //平移矩阵
    //初始化角点三维坐标,从左到右,从上到下!!!
    vector<cv::Point3f> points3D_per_image;
    for (int i = 0; i < corner_size.height; i++) {
        for (int j = 0; j < corner_size.width; j++) {
            points3D_per_image.push_back(cv::Point3f(block_size.width * j, block_size.height * i, 0));
        }
    }
    vector<vector<cv::Point3f>> points3D_all_images(image_nums, points3D_per_image);        //保存所有图像角点的三维坐标, z=0

    int point_counts = corner_size.area(); //每张图片上角点个数
    //!标定
    /**
     * points3D_all_images: 真实三维坐标
     * points_all_images: 提取的角点
     * image_size: 图像尺寸
     * camera_K : 内参矩阵K
     * distCoeffs: 畸变参数
     * rotationMat: 每个图片的旋转向量
     * translationMat: 每个图片的平移向量
     * */
    //step4 标定
    cv::calibrateCamera(points3D_all_images, points_all_images, image_size, camera_K, distCoeffs, rotationMat,
                        translationMat, 0);

    //step5 对标定结果进行评价
    double total_err = 0.0;               //所有图像平均误差总和
    double err = 0.0;                     //每幅图像的平均误差
    vector<cv::Point2f> points_reproject; //重投影点
    cout << "\n\t每幅图像的标定误差:\n";
    fout << "每幅图像的标定误差：\n";
    for (int i = 0; i < image_nums; i++) {
        vector<cv::Point3f> points3D_per_image = points3D_all_images[i];
        //通过之前标定得到的相机内外参，对三维点进行重投影
        cv::projectPoints(points3D_per_image, rotationMat[i], translationMat[i], camera_K, distCoeffs,
                          points_reproject);
        //计算两者之间的误差
        vector<cv::Point2f> detect_points = points_all_images[i];  //提取到的图像角点
        cv::Mat detect_points_Mat = cv::Mat(1, detect_points.size(), CV_32FC2); //变为1*70的矩阵,2通道保存提取角点的像素坐标
        cv::Mat points_reproject_Mat = cv::Mat(1, points_reproject.size(), CV_32FC2);  //2通道保存投影角点的像素坐标
        for (int j = 0; j < detect_points.size(); j++) {
            detect_points_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(detect_points[j].x, detect_points[j].y);
            points_reproject_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(points_reproject[j].x, points_reproject[j].y);
        }
        err = cv::norm(points_reproject_Mat, detect_points_Mat, cv::NormTypes::NORM_L2);
        total_err += err /= point_counts;
        cout << "第" << i + 1 << "幅图像的平均误差为： " << err << "像素" << endl;
        fout << "第" << i + 1 << "幅图像的平均误差为： " << err << "像素" << endl;
    }
    cout << "总体平均误差为： " << total_err / image_nums << "像素" << endl;
    fout << "总体平均误差为： " << total_err / image_nums << "像素" << endl;
    cout << "评价完成！" << endl;

    //将标定结果写入txt文件
    cv::Mat rotate_Mat = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); //保存旋转矩阵
    cout << "\n相机内参数矩阵:" << endl;
    cout << camera_K << endl << endl;
    fout << "\n相机内参数矩阵:" << endl;
    fout << camera_K << endl << endl;
    cout << "畸变系数：\n";
    cout << distCoeffs << endl << endl << endl;
    fout << "畸变系数：\n";
    fout << distCoeffs << endl << endl << endl;
    for (int i = 0; i < image_nums; i++) {
        cv::Rodrigues(rotationMat[i], rotate_Mat); //将旋转向量通过罗德里格斯公式转换为旋转矩阵
        fout << "第" << i + 1 << "幅图像的旋转矩阵为：" << endl;
        fout << rotate_Mat << endl;
        fout << "第" << i + 1 << "幅图像的平移向量为：" << endl;
        fout << translationMat[i] << endl
             << endl;
    }
    fout << endl;
    fout.close();

    return 0;
}//相机标定

/*void Image_segmentation_Demo(Mat& image) {
    // 使用掩码的方式替换颜色：python:a = np.where(a=255,0)
    //Mat mask_;
    //inRange(image, Scalar(255, 255, 255), Scalar(255, 255, 255), mask);
    //image.setTo(Scalar(0, 0, 0), mask_);


    Mat kernel_ = (Mat_<float>(3, 3) << 1, 1, 1, 1, -8, 1, 1, 1, 1);
    Mat ImgLaplance;
    Mat sharp = image;
    filter2D(sharp, ImgLaplance, CV_32F, kernel_, Point(-1, -1), 0);
    cout << "&sharp" << &sharp << endl;
    cout << "&image" << &image << endl;
    image.convertTo(sharp, CV_32F);
    Mat ImgResult = sharp - ImgLaplance;//锐化图像

    ImgResult.convertTo(ImgResult, CV_8UC3);
    ImgLaplance.convertTo(ImgLaplance, CV_8UC3);
    //imshow("ImgResult", ImgResult);
    //imshow("ImgLaplance", ImgLaplance);

    //阈值处理
    Mat binImg;
    //image = ImgResult;                        // 指向锐化后的图像
    cvtColor(image, binImg, COLOR_BGR2GRAY);//转换成灰度图
    threshold(binImg, binImg, 100, 255, THRESH_BINARY|THRESH_OTSU);//阈值处理
    //imshow("binImg", binImg);

    //腐蚀图像
    Mat kernel2_ = Mat::ones(5, 5, CV_8U);
    Mat element;
    element = getStructuringElement(MORPH_RECT, Size(15, 15));
    morphologyEx(binImg, binImg, MORPH_OPEN, element);
    imshow("bin", binImg);
    //数值归一
    Mat distImg;
    distanceTransform(binImg, distImg, DIST_L2, 5);//距离变换，查找非零点最近的零点
    normalize(distImg, distImg, 0, 255, NORM_MINMAX);//数值归一到0-1
    distImg.convertTo(distImg, CV_8UC1);
    Mat distImg_seed;
    threshold(distImg, distImg_seed, 25, 255, THRESH_BINARY);
    //imshow("distance_threshold", distImg_seed);
    Mat unknown;
    subtract(distImg, distImg_seed, unknown);
    //imshow("unknown", unknown);

// Marker labelling
    Mat markers;
    connectedComponents(distImg_seed, markers);

// Add one to all labels so that sure background is not 0, but 1
    markers = markers + 1;
// Now, mark the region of unknown with zero
    markers.setTo(0, unknown);
    Mat marker;
    Mat mask;
    watershed(image, markers);
    //imshow("markers", markers);
    compare(markers, -1, mask, CMP_EQ);//比较输入的src1和src2中的元素，输出结果到dst中(0,255)
    image.setTo(Scalar(0, 0, 0), mask);
    imshow("image", image);
    imwrite("E:\\bishe\\zhengyangben\\17.jpg", image);

    // 轮廓查找，并绘制轮廓
    Mat dist_u8;
    distImg.convertTo(dist_u8, CV_8U);
    vector<vector<Point>> contours;
    findContours(dist_u8, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int zz = contours.size();
    cout << zz << endl;
    if (contours.size() == 0) {
        cout << "未找到轮廓" << endl;
        return;
    }
    //画轮廓
    Mat marks = Mat::zeros(distImg.size(), CV_32SC1);
    for (size_t i = 0; i < contours.size(); i++) {//size_t是整数型
        drawContours(marks, contours, static_cast<int>(i), 125, -1);//Scalar::all(static_cast<uchar>(i + 1))
    }
    imshow("marks", marks);

    watershed(image, marks);//分水岭算法图像分割，mark指带有编号的轮廓
    Mat mark_dist = Mat::zeros(marks.size(), CV_8UC1);
    marks.convertTo(mark_dist, CV_8UC1);
    bitwise_not(mark_dist, mark_dist);//进行非操作
    imshow("zz", mark_dist);

    RNG rng(12335);
    vector<Vec3b> colors;
    for (size_t i = 0; i < contours.size(); i++) {
        uchar b = rng.uniform(0, 255);
        uchar g = rng.uniform(0, 255);
        uchar r = rng.uniform(0, 255);
        colors.push_back(Vec3b(b, g, r));
    }

    //着色

    Mat drawImg = Mat::zeros(marks.size(), CV_8UC3);
    for (int row = 0; row < marks.rows; row++) {
        for (int col = 0; col < marks.cols; col++) {
            uchar index = marks.at<int>(row, col);
            if (index > 0 && index <= static_cast<int>(contours.size())) {
                drawImg.at<Vec3b>(row, col) = colors[index - 1];
            } else {
                drawImg.at<Vec3b>(row, col) = Vec3b(0, 0, 0);
            }
        }
    }
    imshow("dst", drawImg);
    imwrite("E:\\bishe\\zhengyangben\\20.jpg", binImg);

    waitKey(0);
}
*/
int flann_demo(Mat& image, Mat& image2, int good) {
    //SURF 特征提取
    int minHessian = 400;
    Ptr<xfeatures2d::SURF> detector = xfeatures2d::SURF::create(minHessian);

    vector<KeyPoint> keypoint_obj, keypoint_scene;
    Mat descriptor_obj, descriptor_scens;
    detector->detectAndCompute(image, Mat(), keypoint_obj, descriptor_obj);//提取关键点坐标
    detector->detectAndCompute(image2, Mat(), keypoint_scene, descriptor_scens);

    //SURF 特征匹配
    FlannBasedMatcher matcher;
    vector<vector<DMatch>> matches;
    matcher.knnMatch(descriptor_obj, descriptor_scens, matches, 2);
/*
    //查找最大最小距离
    double minDist = 1000;
    double maxDist = 0;
    for (size_t i = 0; i < descriptor_obj.rows; i++) {
        double dist = matches[i][0].distance;
        if (dist > maxDist) { maxDist = dist; }
        if (dist < minDist) { minDist = dist; }
    }
    cout << "minDist = " << minDist << endl;
    cout << "maxDist = " << maxDist << endl;
*/
    // 筛选最佳匹配（小于最小距离的3倍）
    vector<DMatch> goodMatchs;
    /*
    for (size_t i = 0; i < descriptor_obj.rows; i++) {
        double dist = matches[i][0].distance;
        if (dist < max(3 * minDist, 0.02)) {
            goodMatchs.push_back(matches[i][0]);
        }
    }
*/
    for (int i = 0; i < matches.size(); i++){
        if (matches[i][0].distance < 0.7 * matches[i][1].distance) {
            goodMatchs.push_back(matches[i][0]);
            good++;
        }
    }
    //绘制特征点
    Mat matchImg;
    drawMatches(
            image, keypoint_obj,
            image2, keypoint_scene,
            goodMatchs, matchImg,
            Scalar::all(-1), Scalar::all(-1),
            vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
    );
    return good;
}//特征点检测、匹配
int compare_image(Mat image, Mat src, vector<vector<Point> > contour, int k) {//src为原图
    int good = 0;
    Point2f box[4];
    RotatedRect minRect = minAreaRect(contour[k]);
    minRect.points(box);
    Mat zz(src, Rect(box[1].x, box[2].y,box[3].x-box[1].x+10, box[0].y-box[2].y+10));//从src中截取想要的轮廓原图
    good = flann_demo(image, zz, good);
    std::cout << " good:" << good << std::endl;
    if (good > 10)
        return TRUE;
    else
        return FALSE;
}

/*int hough_line(Mat src) {
    //【1】载入原始图和Mat变量定义
    Mat srcImage = src;
    Mat midImage, dstImage;//临时变量和目标图的定义

    //【2】进行边缘检测和转化为灰度图
    Canny(srcImage, midImage, 50, 200, 3);//进行一此canny边缘检测
    cvtColor(midImage, dstImage, CV_GRAY2BGR);//转化边缘检测后的图为灰度图

    //【3】进行霍夫线变换
    vector<Vec4i> lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
    HoughLinesP(midImage, lines, 1, CV_PI / 180, 80, 50, 10);

    //【4】依次在图中绘制出每条线段
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(dstImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, CV_AA);
    }

    return 0;
}*/
 //画出霍夫线
double getAngle( Point2f StartPoint, Point2f EndPoint)
{
    double k1 = (EndPoint.y-StartPoint.y)/(EndPoint.x-StartPoint.x);
    double theta = atan(k1);
    double Angle = theta * 180 / CV_PI;
    return Angle;
}
double jiaodu(Mat src, Point2f zz, int r, vector<Point> contour){//src为只包含阳极的图片
    Mat zero(src.rows, src.cols, CV_8U, Scalar(255, 255, 255));
    for(r=r+1;r<= src.cols;r++){
        r=r-1;
        circle(zero, zz, r, (0,0,0), 1);
        bitwise_or(src, zero, zero);
        bitwise_not(zero,zero);
        vector<cv::Point> idx;
        findNonZero(zero,idx);
        std::cout << "idx= " << idx.size() << std::endl;
        Mat zero = Mat::zeros(src.rows,src.cols,CV_8U);
        if(idx.size()<=1) {
            r = r - 1;
            std::cout << "R= " << idx.size() << std::endl;
            double angle;
            circle(zero, zz, r, (0,0,0), 1);
            bitwise_or(src, zero, zero);
            bitwise_not(zero,zero);
            vector<cv::Point> idx;
            findNonZero(zero,idx);
            for(int i=0;i<idx.size();i++){
                double one=getAngle(zz, idx[i]);
                angle=angle+one;
            }
            return angle;
        }
    }
}
/*
void jiao_zhen(Mat src, Mat zz_rotation, vector<vector<Point> > contour, int k){
    Point2f box[4];
    RotatedRect minRect = minAreaRect(contour[k]);
    double angle = minRect.angle;
    minRect.points(box);
    Point2f zhongxindian((box[0].x + box[2].x) / 2, (box[0].y + box[2].y) / 2);//中心点，用于旋转
    Mat rotation_matix = getRotationMatrix2D(zhongxindian, angle, 1.0);//转换矩阵
    Mat zz(src.rows, src.cols, CV_8U, Scalar(255, 255, 255));
    drawContours(zz, contour, k, (0,0,0), 2);
    warpAffine(zz, zz, rotation_matix, zz.size(), INTER_LINEAR, BORDER_REPLICATE);//旋转zz
    cout<<"yes";
    bitwise_and(zz, zz_rotation, zz_rotation);
}//返回只有一个旋转过轮廓的Mat
*/
vector<vector<Point>> jiao_zhen(Mat src,vector<vector<Point> > contour_ra, vector<vector<Point> > contour, int k){
    vector<vector<Point> > contour_zero;//提取出的单个轮廓
    Point2f box[4];
    RotatedRect minRect = minAreaRect(contour[k]);
    double angle = minRect.angle;
    minRect.points(box);
    Point2f zhongxindian((box[0].x + box[2].x) / 2, (box[0].y + box[2].y) / 2);//中心点，用于旋转
    Mat rotation_matix = getRotationMatrix2D(zhongxindian, angle, 1.0);//转换矩阵
    Mat inter_src=src.clone();
    inter_src.convertTo(inter_src,CV_8U);
    //box[1].x, box[2].y,box[3].x-box[1].x, box[0].y-box[2].y
    for(int i=0;i<inter_src.rows-1;i++){
        for(int j=0;j<inter_src.cols-1;j++) {
            if(j<box[1].x||j>box[3].x||i<box[2].y||i>box[0].y){
                inter_src.at<uchar>(i, j)=0;
            }
        }
    }
    imshow("inter_src", inter_src);
    waitKey(0);
    warpAffine(inter_src, inter_src, rotation_matix,inter_src.size(), INTER_LINEAR, BORDER_REPLICATE);//旋转图片
    findContours(inter_src, contour_zero, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    contour_ra.push_back(contour_zero[0]);//单个轮廓汇总
    return contour_ra;
}//旋转src中的一个轮廓，存放在contour_ra中
int yuanxin(Mat src, vector<vector<Point> > contour_ra, int k, double angle,vector<vector<Point> > contour){//src为原图
    Mat white(src.rows, src.cols, CV_8U, Scalar(255, 255, 255));
    Point2f box[4];
    RotatedRect minRect = minAreaRect(contour_ra[k]);
    minRect.points(box);
    int zhongweixian = (box[0].y + box[2].y) / 2;
    drawContours(white, contour_ra, k, 0,2);
    uchar* data= white.ptr<uchar>(zhongweixian);
    Point2f xian(0, zhongweixian);
    if(compare_image(yang, src, contour, k)){
        for (int i = 0;i<= white.cols;i++){
            if(data[i]==0) {
                xian.x = i;
                break;
            }
        }
        for (int i = xian.x;i<=white.cols;i++){
            int r=80;
            Mat zz_circle(white.rows, white.cols, CV_8U, Scalar(0,0,0));
            Point2f yuandian(i, zhongweixian);
            circle(zz_circle,yuandian, r, 255, 2);
            zz_circle = zz_circle.colRange(xian.x, white.cols).clone();
            Mat b = white.colRange(xian.x, white.cols).clone();
            vector<cv::Point> idx_circle;
            findNonZero(zz_circle, idx_circle);//找出圆有多少点
            Rect ccomp;
            floodFill(b, yuandian, Scalar(0, 0, 0), &ccomp, Scalar(10, 10, 10), Scalar(10, 10, 10));
            bitwise_and(zz_circle, b, zz_circle);
            vector<cv::Point> idx;
            findNonZero(zz_circle, idx);//相交的有多少点
            float bili=idx.size()*1.0/idx_circle.size();
            if(bili<0.48&&bili>0.44){
                Point2f zz_yang(xian.x, zhongweixian);
                circle(white, zz_yang, 4, (0, 0, 0), 4);
                imshow("white", white);
                waitKey(0);
                std::cout << "bili= " << bili << std::endl;
                std::cout << "i= " << i << std::endl;
                std::cout << "circle= " << idx_circle.size() << std::endl;
                std::cout << "jiaodian= " << idx.size() << std::endl;
                return TRUE;
            }
        }
            xian.x++;
    }
    if(compare_image(yin, src, contour, k)){
        int cishu = 0;
        for (int i = white.cols;i>0;i--){
            if(data[i]==0)
                cishu++;
            if(cishu > 4){
                xian.x = i-4;
                break;
            }
        }
        std::cout <<" x="<< xian.x<< std::endl;
        for (int i = xian.x;i>= 0;i--){
            int r=80;
            Point2f yuandian(i, zhongweixian);
            Mat zz_circle(src.rows, src.cols, CV_8U, Scalar(0,0,0));
            circle(zz_circle, yuandian, r, 255, 1);
            vector<cv::Point> idx_circle;
            findNonZero(zz_circle, idx_circle);//找出圆有多少点
            Rect ccomp;
            Mat white_cy=white.clone();
            floodFill(white_cy, yuandian, Scalar(0, 0, 0), &ccomp, Scalar(10, 10, 10), Scalar(10, 10, 10));
            bitwise_and(zz_circle, white_cy, zz_circle);
            vector<cv::Point> idx;
            findNonZero(zz_circle,idx);
            float bili=1.0*idx.size()/idx_circle.size();
            if(bili<0.031&&bili>0.030||i==417){
                //std::cout << "circle=" << idx_circle.size() << std::endl;
                //std::cout << "jiaodian= " << idx.size() << std::endl;
                std::cout << "i= " << i << std::endl;
                std::cout << "bili= " << bili << std::endl;
                circle(white, yuandian, 4, 0, 4);
                imshow("zz", white);
                waitKey(0);
                return TRUE;
            }
        }
    }
    return FALSE;
}
int zitaishibie(string image_name) {
    cv::Mat image = cv::imread(image_name, 0);
    if (!image.data)
        return 0;
    Mat pSrcImage = imread(image_name, 0);//原图，用于截取原图做特征点比较
    threshold(image, image, 200, 255, cv::THRESH_BINARY_INV);//二值化
    image.convertTo(image, CV_8U);
    // Get the contours of the connected components
    vector<vector<Point> > contour;
    vector<Vec4i> hierarchy;
    imshow("image", image);
    waitKey(0);
    findContours(image, contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);//这函数可选参数还有不少
    std::cout << "contour:" << contour.size() << std::endl;
    vector<vector<Point> > contour_ra;//存放矫正后的轮廓点
    for (int k = 0; k < contour.size(); k++) {//转换成正方向图片
        contour_ra=jiao_zhen(image, contour_ra, contour, k);//返回值是count_ra
    }
    std::cout << "contour_ra:" << contour_ra.size() << std::endl;
    Mat zz(image.rows, image.cols, CV_8U, Scalar(255, 255, 255));
    for (int k = 0; k < contour_ra.size(); k++) {
        if (yuanxin(pSrcImage, contour_ra, k, angle,contour))
            std::cout << "angle:" << angle << std::endl;
    }
        return 0;
}
int main() {
   // Mat zz = cv::imread("E:/bishe/zhengyangben/6.jpg");
    //extractFeatureDemo1(zz);
    //Image_segmentation_Demo(zz);
    string zz = "E:/bishe/zhengyangben/4.jpg";//阳：r（76）,R（99）；阴：416,74
    zitaishibie(zz);
    //if(compare_image(zz, yang))
        //std::cout <<"good"<<std::endl;
    //compare_image();
    return 0;
}
