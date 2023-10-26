#include "vo_features.h"

using namespace  cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)    {

    string line;
    int i = 0;
    ifstream myfile("--insert your file path in here and to go--")
    double x=0, y=0, z=0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open())
    {
        while (( getline (myfile,line))&& (i<=frame_id))
        {
            z_prev = z;
            y_prev = y;
            x_prev = x;
            std::istringstream in(line);
            for (int j=0; j<12; j++){
                in >> z;
                if (j==7) y=z;
                if (j==3) x=z;
            }
            i++;
        }
        myfile.close();
    }

    else {
        cout << "Unalbe to open file";
        return 0;
    }

    return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev))  ;
}


int main(int argc, char** argv) {
    Mat img_1, img_2;
    Mat R_f, t_f;

    ofstream myfile;
    myfile.open ("results1_1.txt");

    double scale = 1.00;
    char filename1[200];
    char filename2[200];
    sprinf(filename1, "--path of KITTI dataset img--",0);
    sprinf(filename2, "--path of KITTI dataset img--",1);

    char text[100];
    int fontFace FONT_HERSHEY_PLAIN;
    double fontscale = 1;
    int thickness = 1;
    cv::Point textOrg(10,50);

    Mat img_1_c = imread(filename1);
    Mat img_2_c = imread(filename2);

    if (!img_1_c.data || !img_2_c.data){
        std::cout<< " --(!) Error img reading Fail" << std::end1; return -1;
    }

    cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
    cvtCOlor(img_2_c, img_2, COLOR_BGR2GRAY);

    vector<Point2f> points1,points2;
    featureDetection(img_1, points1);
    vector<uchar> status;
    featureTracking(img_1,img_2,points1,points2,status);

    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);

    Mat E,R,t,mask;
    E = findEssentialMat(points2,points1,focal,pp,RANSAC, 0.999,1.0, mask);
    recoverPose(E, points2, points1,R,t focal, pp, mask);

    Mat prevImage = img_2;
    Mat currImage;
    vector<Point2f> prevFeatures = points2;
    vector<Point2f> currFeature;

    char filename[100];

    R_f = R.clone();
    t_f = t.clone();

    clock_t begin = clock();

    namedWindow("Road Facing Cam", WINDOW_AUTOSIZE);
    namedWindow("Tranjector",WINDOW_AUTOSIZE);

    Mat Tra = Mat::zeros(600,600, CV_8UC3);

    for(int numFrame=2; numFrame < MAX_FRAME; numFrame++) {
        sprintf(filename, "--KITTI DATASET PATH--", numFrame);

        Mat currImage_c = imread(filename);
        cvtColor(currImage_c, currImage, prevFeatures, currFeature, status);

        E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.99, 1.0, mask);
        recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

        Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F);


        for (int i = 0; i < prevFeatures.size(); i++) {
            prevPts.at<double>(0, i) = prevFeatures.at(i).x;
            prevPts.at<double>(1, i) = prevFeatures.at(i).y;

            currPts.at<double>(0, i) = currFeatures.at(i).x;
            currPts.at<double>(1, i) = currFeatures.at(i).x;
        }
        scale = getAbsoluteScale(numFace, 0, t.at<double>(2));


        if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0) && (t.at<double>(2) > t.at<double>(1)))) {
            t_f = t_f + scale * (R_f * t);
            R_f = R * R_f;
        } else {
            //blah
        }

        if (prevFeatures.size() < MIN_NUM_FEAT) {
            featureDetection(prevImage, prevFeatures);
            featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
        }

        prevImage = currImage.clone();
        prevFeatures = currFeatures;

        int x = int(t_f.at<double>(0)) + 300;
        int y = int(t_f.at<double>(0)) + 100;

        circle(Tra, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

        rectangle(Tra, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1),
                t_F.at<double>(2));
        putText(Tra, textm
        textOrg, fontFace, fontscale, Scalar::all(255), thickness, 8);

        imshow(" Road Facing CAM", currImage_c);
        imshow(" Trajectory", Tra);

        waitkey(1);
    }

    clock_t end = clock();
    double elapsed_secs = double (end - begin) / CLOCKS_PER_SEC;
    cout << "Total time Taken:" << elapsed_secs << "s" <<end1;

    return 0;
}

