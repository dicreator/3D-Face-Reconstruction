
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <iterator>
#include <dirent.h>
#include <stdlib.h>
#include <algorithm>
#include <vector>


using namespace cv;
using namespace std;


void mouseEvent(int event, int x, int y, int flags, void* param){
  IplImage* img = (IplImage*) param;

  if (event == CV_EVENT_LBUTTONDOWN){
    printf("%d, %d \n", x, y);
  }
}
int main( int argc, char** argv){


Mat Rgb;
Mat Gray;
Mat Bin;

Rgb = imread(argv[1], WINDOW_AUTOSIZE);
cvtColor(Rgb, Gray, CV_RGB2GRAY);

vector<Point> v;
threshold(Gray, Bin, 100, 255, THRESH_BINARY);
cv::findNonZero(Bin, v);

//write to a files
// std::string fname = argv[1];
//
// std::ofstream outputFile(fname+".txt");
// std::ostream_iterator<Point> output_iterator(outputFile, "\n");
// std::copy(v.begin(), v.end(), output_iterator);
//
// //print
// for (auto vec : v){
//   std::cout << vec << '\n';
// }


IplImage *img = cvLoadImage(argv[1]);
cvNamedWindow("My img",1);
cvSetMouseCallback("My img", mouseEvent, &img);
cvShowImage("My img", img);

cvWaitKey(0);

//wait for a character to exit
}
