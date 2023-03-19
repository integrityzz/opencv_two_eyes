#include <iostream>
#include "CameraApi.h" //���SDK��APIͷ�ļ�
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
    CameraHandle m_hCamera;  //�������
    iCameraNums = 10;
    //ö���������෵��10�������������Ϣ
    status= CameraEnumerateDevice(sCameraList,&iCameraNums);
    if(status!= CAMERA_STATUS_SUCCESS||iCameraNums == 0){
        printf("device_error\n");
    }
    //���ֻ��һ������� iCameraNums�ᱻCameraEnumerateDevice�ڲ��޸�Ϊ1��
    status = CameraInit(&sCameraList[0], -1, -1, &m_hCamera);
    if(status!=CAMERA_STATUS_SUCCESS){
        printf("init_error\n");
        char msg[128];
        sprintf_s(msg,"Failed to init the camera! Error code is %d\n",status);
        printf(msg);
    }
    /* pbyBuffer ��SDK�ڲ��Զ���������ڴ棬�����ԭʼRAW����*/
    CameraGetCapability(m_hCamera,&sCameraInfo);//"��ø��������������"
    m_pFrameBuffer = (BYTE *)CameraAlignMalloc(sCameraInfo.sResolutionRange.iWidthMax*sCameraInfo.sResolutionRange.iWidthMax*3,16);
    //sResolutionRange;  �ֱ��ʷ�Χ����
    CameraPlay(m_hCamera);//�������
    CameraSetIspOutFormat(m_hCamera,CAMERA_MEDIA_TYPE_MONO8);
    //CameraImageProcess �����ͼ����� 8bit �ĻҶ�ͼ���ˣ� 1 ������ռ�� 1 ���ֽڣ��������С�0000000000000000
    strcpy_s(g_CameraName,sCameraList[0].acFriendlyName);//char acFriendlyName[64]; ��ʼ���ӿ�ʹ�õ��豸�ǳ�
    CameraSetTriggerMode(m_hCamera,1);//ͨ�� CameraSetTriggerMode(hCamera,1);�л�Ϊ����ȡͼģʽ��
    CameraSoftTrigger(m_hCamera);//����ȡͼһ��
    status=CameraGetImageBuffer(m_hCamera, &sFrameInfo, &pbyBuffer, 1000);
    if (status != CAMERA_STATUS_SUCCESS) {
        /* m_pFrameBuffer��Ҫ��������ڴ�pbyBuffer ת�����ͼ�����ݱ�����m_pFrameBuffer �У�
        Ĭ�ϻ�õ�BRG24bit��ͼ������*/
        printf("get_error\n");
        char msg[128];
        sprintf_s(msg,"Failed to get the image! Error code is %d\n",status);
        printf(msg);
    }
    CameraImageProcess(m_hCamera, pbyBuffer, m_pFrameBuffer, &sFrameInfo);
    //2��3���������롢���������������4����������ͼ���֡ͷ��Ϣ��������ɺ�֡ͷ��Ϣ�е�ͼ���ʽ uiMediaType ����֮�ı�
    CameraReleaseImageBuffer(m_hCamera, pbyBuffer);
    //CameraReleaseImageBuffer�ͷ���CameraGetImageBuffer�õ��Ļ�����
    CameraSaveImage(m_hCamera,(char*)"E:/bishe/cal_images/file_images.jpg",m_pFrameBuffer,&sFrameInfo,1,100);
    CameraStop(m_hCamera);
    CameraUnInit(m_hCamera);
    //�����˳�ǰ����
    return 0;
}//���ȡһ��ͼ
int demarcate_camer() {
    string dir = "E:/bishe/cal_images/";  //�궨ͼƬ�����ļ���
    ifstream fin(dir + "file_images.txt"); //��ȡ�궨ͼƬ��·������cpp������ͬһ·����
    if (!fin)                              //����Ƿ��ȡ���ļ�
    {
        cerr << "û���ҵ��ļ�" << endl;
        return -1;
    }
    ofstream fout(dir + "calibration_result.txt"); //�����������ڴ��ı��ļ���
    //���ζ�ȡÿһ��ͼƬ��������ȡ�ǵ�
    cout << "beginning" << endl;
    int image_nums = 0;  //ͼƬ����
    cv::Size image_size; //ͼƬ�ߴ�
    int points_per_row = 12;  //ÿ�е��ڵ���
    int points_per_col = 9;   //ÿ�е��ڵ���
    cv::Size corner_size = cv::Size(points_per_row, points_per_col); //�궨��ÿ��ÿ�нǵ��������12*9���ǵ�
    vector<cv::Point2f> points_per_image;                            //����ÿ��ͼ��⵽�Ľǵ�
    points_per_image.clear();
    vector<vector<cv::Point2f>> points_all_images;                   //��һ����ά���鱣���⵽�����нǵ�
    string image_file_name;                                          //����һ���ļ������ַ���

    while (getline(fin, image_file_name)) //���ж�ȡ�����ж����ַ���
    {
        image_nums++;
        //����ͼƬ
        cv::Mat image_raw = cv::imread(image_file_name);//cv::imread��ȡͼƬ
        cv::imshow("Camera calibration", image_raw);
        cv::waitKey(0); //�ȴ���������
        if (image_nums == 1) {
            cout<<"channels = "<<image_raw.channels()<<endl;
            cout<<image_raw.type()<<endl;  //CV_8UC3
            image_size.width = image_raw.cols;  //ͼ��Ŀ��Ӧ������
            image_size.height = image_raw.rows; //ͼ��ĸ߶�Ӧ������
            cout << "image_size.width = " << image_size.width << endl;
            cout << "image_size.height = " << image_size.height << endl;
        }
        //�ǵ���
        cv::Mat image_gray;                               //�洢�Ҷ�ͼ�ľ���
        cv::cvtColor(image_raw, image_gray, CV_BGR2GRAY); //��BGRͼת��Ϊ�Ҷ�ͼ
        // cout<<"image_gray.type() = "<<image_gray.type()<<endl;  //CV_8UC1
        //step1 ��ȡ�ǵ�
        bool success = cv::findChessboardCorners(image_gray, corner_size, points_per_image);//�ǵ���
        if (!success) {
            cout << "can not find the corners " << endl;
            exit(1);
        } else {
            //�����ؾ�ȷ�������ַ�����
            //step2 �����ؽǵ�
            cv::find4QuadCornerSubpix(image_gray, points_per_image, cv::Size(5, 5));
            // cornerSubPix(image_gray,points_per_image,Size(5,5));
            points_all_images.push_back(points_per_image); //���������ؽǵ�
            //��ͼ�л����ǵ�λ��
            //step3 �ǵ���ӻ�
            cv::drawChessboardCorners(image_raw, corner_size, points_per_image, success); //���ǵ�����
            cv::imshow("Camera calibration", image_raw);
            cv::waitKey(0); //�ȴ���������
        }
    }
    cv::destroyAllWindows();
    //���ͼ����Ŀ
    int image_sum_nums = points_all_images.size();
    cout << "image_sum_nums = " << image_sum_nums << endl;

    //��ʼ����궨
    cv::Size block_size(21, 21);                            //ÿ��С����ʵ�ʴ�С, ֻ��Ӱ���������ƽ������t
    cv::Mat camera_K(3, 3, CV_32FC1, cv::Scalar::all(0));   //�ڲξ���3*3
    cv::Mat distCoeffs(1, 5, CV_32FC1, cv::Scalar::all(0)); //�������1*5
    vector<cv::Mat> rotationMat;                            //��ת����
    vector<cv::Mat> translationMat;                         //ƽ�ƾ���
    //��ʼ���ǵ���ά����,������,���ϵ���!!!
    vector<cv::Point3f> points3D_per_image;
    for (int i = 0; i < corner_size.height; i++) {
        for (int j = 0; j < corner_size.width; j++) {
            points3D_per_image.push_back(cv::Point3f(block_size.width * j, block_size.height * i, 0));
        }
    }
    vector<vector<cv::Point3f>> points3D_all_images(image_nums, points3D_per_image);        //��������ͼ��ǵ����ά����, z=0

    int point_counts = corner_size.area(); //ÿ��ͼƬ�Ͻǵ����
    //!�궨
    /**
     * points3D_all_images: ��ʵ��ά����
     * points_all_images: ��ȡ�Ľǵ�
     * image_size: ͼ��ߴ�
     * camera_K : �ڲξ���K
     * distCoeffs: �������
     * rotationMat: ÿ��ͼƬ����ת����
     * translationMat: ÿ��ͼƬ��ƽ������
     * */
    //step4 �궨
    cv::calibrateCamera(points3D_all_images, points_all_images, image_size, camera_K, distCoeffs, rotationMat,
                        translationMat, 0);

    //step5 �Ա궨�����������
    double total_err = 0.0;               //����ͼ��ƽ������ܺ�
    double err = 0.0;                     //ÿ��ͼ���ƽ�����
    vector<cv::Point2f> points_reproject; //��ͶӰ��
    cout << "\n\tÿ��ͼ��ı궨���:\n";
    fout << "ÿ��ͼ��ı궨��\n";
    for (int i = 0; i < image_nums; i++) {
        vector<cv::Point3f> points3D_per_image = points3D_all_images[i];
        //ͨ��֮ǰ�궨�õ����������Σ�����ά�������ͶӰ
        cv::projectPoints(points3D_per_image, rotationMat[i], translationMat[i], camera_K, distCoeffs,
                          points_reproject);
        //��������֮������
        vector<cv::Point2f> detect_points = points_all_images[i];  //��ȡ����ͼ��ǵ�
        cv::Mat detect_points_Mat = cv::Mat(1, detect_points.size(), CV_32FC2); //��Ϊ1*70�ľ���,2ͨ��������ȡ�ǵ����������
        cv::Mat points_reproject_Mat = cv::Mat(1, points_reproject.size(), CV_32FC2);  //2ͨ������ͶӰ�ǵ����������
        for (int j = 0; j < detect_points.size(); j++) {
            detect_points_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(detect_points[j].x, detect_points[j].y);
            points_reproject_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(points_reproject[j].x, points_reproject[j].y);
        }
        err = cv::norm(points_reproject_Mat, detect_points_Mat, cv::NormTypes::NORM_L2);
        total_err += err /= point_counts;
        cout << "��" << i + 1 << "��ͼ���ƽ�����Ϊ�� " << err << "����" << endl;
        fout << "��" << i + 1 << "��ͼ���ƽ�����Ϊ�� " << err << "����" << endl;
    }
    cout << "����ƽ�����Ϊ�� " << total_err / image_nums << "����" << endl;
    fout << "����ƽ�����Ϊ�� " << total_err / image_nums << "����" << endl;
    cout << "������ɣ�" << endl;

    //���궨���д��txt�ļ�
    cv::Mat rotate_Mat = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); //������ת����
    cout << "\n����ڲ�������:" << endl;
    cout << camera_K << endl << endl;
    fout << "\n����ڲ�������:" << endl;
    fout << camera_K << endl << endl;
    cout << "����ϵ����\n";
    cout << distCoeffs << endl << endl << endl;
    fout << "����ϵ����\n";
    fout << distCoeffs << endl << endl << endl;
    for (int i = 0; i < image_nums; i++) {
        cv::Rodrigues(rotationMat[i], rotate_Mat); //����ת����ͨ���޵����˹��ʽת��Ϊ��ת����
        fout << "��" << i + 1 << "��ͼ�����ת����Ϊ��" << endl;
        fout << rotate_Mat << endl;
        fout << "��" << i + 1 << "��ͼ���ƽ������Ϊ��" << endl;
        fout << translationMat[i] << endl
             << endl;
    }
    fout << endl;
    fout.close();

    return 0;
}//����궨

/*void Image_segmentation_Demo(Mat& image) {
    // ʹ������ķ�ʽ�滻��ɫ��python:a = np.where(a=255,0)
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
    Mat ImgResult = sharp - ImgLaplance;//��ͼ��

    ImgResult.convertTo(ImgResult, CV_8UC3);
    ImgLaplance.convertTo(ImgLaplance, CV_8UC3);
    //imshow("ImgResult", ImgResult);
    //imshow("ImgLaplance", ImgLaplance);

    //��ֵ����
    Mat binImg;
    //image = ImgResult;                        // ָ���񻯺��ͼ��
    cvtColor(image, binImg, COLOR_BGR2GRAY);//ת���ɻҶ�ͼ
    threshold(binImg, binImg, 100, 255, THRESH_BINARY|THRESH_OTSU);//��ֵ����
    //imshow("binImg", binImg);

    //��ʴͼ��
    Mat kernel2_ = Mat::ones(5, 5, CV_8U);
    Mat element;
    element = getStructuringElement(MORPH_RECT, Size(15, 15));
    morphologyEx(binImg, binImg, MORPH_OPEN, element);
    imshow("bin", binImg);
    //��ֵ��һ
    Mat distImg;
    distanceTransform(binImg, distImg, DIST_L2, 5);//����任�����ҷ������������
    normalize(distImg, distImg, 0, 255, NORM_MINMAX);//��ֵ��һ��0-1
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
    compare(markers, -1, mask, CMP_EQ);//�Ƚ������src1��src2�е�Ԫ�أ���������dst��(0,255)
    image.setTo(Scalar(0, 0, 0), mask);
    imshow("image", image);
    imwrite("E:\\bishe\\zhengyangben\\17.jpg", image);

    // �������ң�����������
    Mat dist_u8;
    distImg.convertTo(dist_u8, CV_8U);
    vector<vector<Point>> contours;
    findContours(dist_u8, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int zz = contours.size();
    cout << zz << endl;
    if (contours.size() == 0) {
        cout << "δ�ҵ�����" << endl;
        return;
    }
    //������
    Mat marks = Mat::zeros(distImg.size(), CV_32SC1);
    for (size_t i = 0; i < contours.size(); i++) {//size_t��������
        drawContours(marks, contours, static_cast<int>(i), 125, -1);//Scalar::all(static_cast<uchar>(i + 1))
    }
    imshow("marks", marks);

    watershed(image, marks);//��ˮ���㷨ͼ��ָmarkָ���б�ŵ�����
    Mat mark_dist = Mat::zeros(marks.size(), CV_8UC1);
    marks.convertTo(mark_dist, CV_8UC1);
    bitwise_not(mark_dist, mark_dist);//���зǲ���
    imshow("zz", mark_dist);

    RNG rng(12335);
    vector<Vec3b> colors;
    for (size_t i = 0; i < contours.size(); i++) {
        uchar b = rng.uniform(0, 255);
        uchar g = rng.uniform(0, 255);
        uchar r = rng.uniform(0, 255);
        colors.push_back(Vec3b(b, g, r));
    }

    //��ɫ

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
    //SURF ������ȡ
    int minHessian = 400;
    Ptr<xfeatures2d::SURF> detector = xfeatures2d::SURF::create(minHessian);

    vector<KeyPoint> keypoint_obj, keypoint_scene;
    Mat descriptor_obj, descriptor_scens;
    detector->detectAndCompute(image, Mat(), keypoint_obj, descriptor_obj);//��ȡ�ؼ�������
    detector->detectAndCompute(image2, Mat(), keypoint_scene, descriptor_scens);

    //SURF ����ƥ��
    FlannBasedMatcher matcher;
    vector<vector<DMatch>> matches;
    matcher.knnMatch(descriptor_obj, descriptor_scens, matches, 2);
/*
    //���������С����
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
    // ɸѡ���ƥ�䣨С����С�����3����
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
    //����������
    Mat matchImg;
    drawMatches(
            image, keypoint_obj,
            image2, keypoint_scene,
            goodMatchs, matchImg,
            Scalar::all(-1), Scalar::all(-1),
            vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
    );
    return good;
}//�������⡢ƥ��
int compare_image(Mat image, Mat src, vector<vector<Point> > contour, int k) {//srcΪԭͼ
    int good = 0;
    Point2f box[4];
    RotatedRect minRect = minAreaRect(contour[k]);
    minRect.points(box);
    Mat zz(src, Rect(box[1].x, box[2].y,box[3].x-box[1].x+10, box[0].y-box[2].y+10));//��src�н�ȡ��Ҫ������ԭͼ
    good = flann_demo(image, zz, good);
    std::cout << " good:" << good << std::endl;
    if (good > 10)
        return TRUE;
    else
        return FALSE;
}

/*int hough_line(Mat src) {
    //��1������ԭʼͼ��Mat��������
    Mat srcImage = src;
    Mat midImage, dstImage;//��ʱ������Ŀ��ͼ�Ķ���

    //��2�����б�Ե����ת��Ϊ�Ҷ�ͼ
    Canny(srcImage, midImage, 50, 200, 3);//����һ��canny��Ե���
    cvtColor(midImage, dstImage, CV_GRAY2BGR);//ת����Ե�����ͼΪ�Ҷ�ͼ

    //��3�����л����߱任
    vector<Vec4i> lines;//����һ��ʸ���ṹlines���ڴ�ŵõ����߶�ʸ������
    HoughLinesP(midImage, lines, 1, CV_PI / 180, 80, 50, 10);

    //��4��������ͼ�л��Ƴ�ÿ���߶�
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(dstImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, CV_AA);
    }

    return 0;
}*/
 //����������
double getAngle( Point2f StartPoint, Point2f EndPoint)
{
    double k1 = (EndPoint.y-StartPoint.y)/(EndPoint.x-StartPoint.x);
    double theta = atan(k1);
    double Angle = theta * 180 / CV_PI;
    return Angle;
}
double jiaodu(Mat src, Point2f zz, int r, vector<Point> contour){//srcΪֻ����������ͼƬ
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
    Point2f zhongxindian((box[0].x + box[2].x) / 2, (box[0].y + box[2].y) / 2);//���ĵ㣬������ת
    Mat rotation_matix = getRotationMatrix2D(zhongxindian, angle, 1.0);//ת������
    Mat zz(src.rows, src.cols, CV_8U, Scalar(255, 255, 255));
    drawContours(zz, contour, k, (0,0,0), 2);
    warpAffine(zz, zz, rotation_matix, zz.size(), INTER_LINEAR, BORDER_REPLICATE);//��תzz
    cout<<"yes";
    bitwise_and(zz, zz_rotation, zz_rotation);
}//����ֻ��һ����ת��������Mat
*/
vector<vector<Point>> jiao_zhen(Mat src,vector<vector<Point> > contour_ra, vector<vector<Point> > contour, int k){
    vector<vector<Point> > contour_zero;//��ȡ���ĵ�������
    Point2f box[4];
    RotatedRect minRect = minAreaRect(contour[k]);
    double angle = minRect.angle;
    minRect.points(box);
    Point2f zhongxindian((box[0].x + box[2].x) / 2, (box[0].y + box[2].y) / 2);//���ĵ㣬������ת
    Mat rotation_matix = getRotationMatrix2D(zhongxindian, angle, 1.0);//ת������
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
    warpAffine(inter_src, inter_src, rotation_matix,inter_src.size(), INTER_LINEAR, BORDER_REPLICATE);//��תͼƬ
    findContours(inter_src, contour_zero, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    contour_ra.push_back(contour_zero[0]);//������������
    return contour_ra;
}//��תsrc�е�һ�������������contour_ra��
int yuanxin(Mat src, vector<vector<Point> > contour_ra, int k, double angle,vector<vector<Point> > contour){//srcΪԭͼ
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
            findNonZero(zz_circle, idx_circle);//�ҳ�Բ�ж��ٵ�
            Rect ccomp;
            floodFill(b, yuandian, Scalar(0, 0, 0), &ccomp, Scalar(10, 10, 10), Scalar(10, 10, 10));
            bitwise_and(zz_circle, b, zz_circle);
            vector<cv::Point> idx;
            findNonZero(zz_circle, idx);//�ཻ���ж��ٵ�
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
            findNonZero(zz_circle, idx_circle);//�ҳ�Բ�ж��ٵ�
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
    Mat pSrcImage = imread(image_name, 0);//ԭͼ�����ڽ�ȡԭͼ��������Ƚ�
    threshold(image, image, 200, 255, cv::THRESH_BINARY_INV);//��ֵ��
    image.convertTo(image, CV_8U);
    // Get the contours of the connected components
    vector<vector<Point> > contour;
    vector<Vec4i> hierarchy;
    imshow("image", image);
    waitKey(0);
    findContours(image, contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);//�⺯����ѡ�������в���
    std::cout << "contour:" << contour.size() << std::endl;
    vector<vector<Point> > contour_ra;//��Ž������������
    for (int k = 0; k < contour.size(); k++) {//ת����������ͼƬ
        contour_ra=jiao_zhen(image, contour_ra, contour, k);//����ֵ��count_ra
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
    string zz = "E:/bishe/zhengyangben/4.jpg";//����r��76��,R��99��������416,74
    zitaishibie(zz);
    //if(compare_image(zz, yang))
        //std::cout <<"good"<<std::endl;
    //compare_image();
    return 0;
}
