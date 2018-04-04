
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <stdlib.h>
#include <algorithm>
#include <vector>
//
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>

using namespace dlib;
using namespace cv;
using namespace std;

// Detect faces
// Transformation to original
// Face Alignment
// filter and blur


//////////////////////////
double distanceFound(Point2f first, Point2f second){
  double dis = norm(first-second);
  return dis;

}

Point2d pointAbove(Point2f first, Point2f second){
  Point2f midPnt = Point2f((first.x+second.x)/2,(first.y+second.y)/2);

  double gradientPerp = (second.x-first.x)/(second.y-second.x);

  Point2f pntAbove;


  pntAbove.y = midPnt.y - distanceFound(first,second) * 1.4;



  pntAbove.x = ((pntAbove.y - midPnt.y)/gradientPerp) + midPnt.x;
  return pntAbove;
}

Scalar skinTone(std::vector<Point2f> trgPoints, Mat avepic){
  std::vector<Point2f> outpt1;
  outpt1.push_back(trgPoints[15]);
  outpt1.push_back(trgPoints[53]);
  outpt1.push_back(trgPoints[35]);

  Rect r11 = boundingRect(outpt1);

  std::vector<Point> tRectInt1;

  //used in tranformation, distance inside rect
  for(int g = 0; g < 3; g++)
  {
    tRectInt1.push_back( Point((int)(outpt1[g].x - r11.x), (int)(outpt1[g].y - r11.y)) ); // for fillConvexPoly
  }////

  Mat mask1 = Mat::zeros(r11.height, r11.width, CV_8U);
  // Mat mask2 = Mat::zeros(r11.height, r11.width, avepic.type());

  fillConvexPoly(mask1, tRectInt1, Scalar(1.0, 1.0, 1.0));

  Scalar average = mean(avepic(r11), mask1);

  return average;
}
//////////////////////////////////////

static void applyFragment(std::vector<Point2f> inpt, std::vector<Point2f> outpt, Mat &txtimage, Mat imgTr){
  Rect r1 = boundingRect(inpt);
  Rect r2 = boundingRect(outpt);

  // std::cout << trIdx[i][0] << trIdx[i][1] << trIdx[i][2] << '\n';

  ////// offsetting points be left top corner
  std::vector<Point2f> t1Rect, t2Rect;
  std::vector<Point> tRectInt;
  //used in tranformation, distance inside rect
  for(int g = 0; g < 3; g++)
  {

      t1Rect.push_back( Point2f( inpt[g].x - r1.x, inpt[g].y -  r1.y) ); //rect 1
      t2Rect.push_back( Point2f( outpt[g].x - r2.x, outpt[g].y - r2.y) ); //rect2
      tRectInt.push_back( Point((int)(outpt[g].x - r2.x), (int)(outpt[g].y - r2.y)) ); // for fillConvexPoly


  }////

  Mat img1Rect;
  //crop
  imgTr(r1).copyTo(img1Rect);
  //trasnformation for the shape inside the rectangular
  Mat warpMat = getAffineTransform( t1Rect, t2Rect );
  Mat img2Rect = Mat::zeros(r2.height, r2.width,img1Rect.type());


  warpAffine(img1Rect, img2Rect, warpMat, img2Rect.size(), INTER_LINEAR, BORDER_REFLECT_101);

  // Get mask by filling triangle
  Mat mask = Mat::zeros(r2.height, r2.width, CV_32FC3);

  ////////////////////
  //image - mask
  //tRectInt  - cvertices
  //Scalar to specify color intensity
  // 16 - line type
  /////////////////////////////
  //fill rect mask
  fillConvexPoly(mask, tRectInt, Scalar(1.0, 1.0, 1.0));

  // apply traingl to the image mask first
  multiply(img2Rect, mask, img2Rect);
  multiply(txtimage(r2), Scalar(1.0,1.0,1.0) - mask, txtimage(r2));

  txtimage(r2) = txtimage(r2) + img2Rect;
}
///////////////////

Mat applySkinBg(Mat txtimage, std::vector<Point2f> basePoints, Scalar average){
  std::vector<Point2f> hull;
  std::vector<int> hullIdx;
  std::vector<Point> hullfMask;
  Mat maskHull = Mat::zeros(txtimage.rows, txtimage.cols, txtimage.depth());
  Point center;

  convexHull(basePoints, hullIdx, false, false);

  for(int v = 0; v < hullIdx.size(); v++){
    hull.push_back(basePoints[hullIdx[v]]);
    //for mask
    Point pt1(hull[v].x , hull[v].y);
    hullfMask.push_back(pt1);


  }

//Create mask1
fillConvexPoly(maskHull, &hullfMask[0], hullfMask.size(), Scalar(255,255,255));

  //Clone seamlessly
  Rect rHull = boundingRect(hull);
  center = (rHull.tl() + rHull.br()) / 2;

  Mat bgSkin(txtimage.rows, txtimage.cols, CV_8UC3, average);
  Mat finalOutput;
  seamlessClone(txtimage,bgSkin,maskHull,center,finalOutput, NORMAL_CLONE);
  return finalOutput;

}

std::vector< std::vector<int> > triangulate_delaunay(Mat& img1, std::vector<Point2f> &basePoints){
  // Keep a copy around

  // Rectangle to be used with Subdiv2D
  Size size = img1.size();
  Rect rect(0, 0, size.width, size.height);
  Subdiv2D subdiv(rect);
  int size_list = 0;
  int row = 0;
  for( std::vector<Point2f>::iterator it = basePoints.begin(); it != basePoints.end(); it++){
    subdiv.insert(*it);
  }

  std::vector<Vec6f> triangleList;
  subdiv.getTriangleList(triangleList);

  //check if points are inside
  for (size_t i = 0; i < triangleList.size(); i++){
    Vec6f trg = triangleList[i];
    if (rect.contains(Point(cvRound(trg[0]),cvRound(trg[1]))) && rect.contains(Point(cvRound(trg[2]),cvRound(trg[3]))) && rect.contains(Point(cvRound(trg[4]),cvRound(trg[5])))){
      size_list += 1;
    }
  }

  std::vector<Point2f> pit(3);
  std::vector< std::vector<int> > indaxes(size_list);

  std::cout << size_list << '\n';

//////////////////////////////////////
  //define size for 2-D vector
  for (size_t i = 0; i < triangleList.size(); i++){
    Vec6f trg = triangleList[i];

    pit[0] = Point2f(trg[0], trg[1]);
    pit[1] = Point2f(trg[2], trg[3]);
    pit[2] = Point2f(trg[4], trg[5]);

        // Identify is the point inside the rectangle
//    if ( rect.contains(pit[0]) && rect.contains(pit[1]) && rect.contains(pit[2])){
    if (rect.contains(Point(cvRound(trg[0]),cvRound(trg[1]))) && rect.contains(Point(cvRound(trg[2]),cvRound(trg[3]))) && rect.contains(Point(cvRound(trg[4]),cvRound(trg[5])))){

      for(int j = 0; j < 3; j++){
//////////////
        bool found = false;
        auto pos = std::find(basePoints.begin(), basePoints.end(), pit[j]);
        if( pos !=  basePoints.end()) found = true;
        if(found)
        {
       int index = std::distance(basePoints.begin(), pos );

       // std::cout << index << '\n';
       indaxes[row].push_back(index);

        }else{
          std::cout << "not found:\t" << *pos << '\n';
        }
      }
      row++;
    } //if inside
  }//for

  return indaxes;
}


int main( int argc, char** argv){


  //for boundary boxes
  //function returns object_detector
  frontal_face_detector detector = get_frontal_face_detector();
  //predict face landmark positions - 68 .dat import
  shape_predictor fshape;

  deserialize("shape_predictor_68_face_landmarks.dat") >> fshape; // change two lines into one

  image_window win, win_faces;
  array2d<rgb_pixel> img;
  // string fname1("Untitled Folder/avefac.jpg");
  string fname1(argv[1]);
  // string fname1("Untitled Folder/face-straight1.jpg");


  //
load_image(img,fname1);

//increase image?
// pyramid_up(img);

std::vector<dlib::rectangle> dets = detector(img);

//if more than one face was detected
if (dets.size() == 0 || dets.size() > 1) {
  cout << "Picture contains none or many faces" << endl;
  cout << "Please - use a picture with only one face." << endl;
  exit(0);
}


// std::std::vector<full_object_detection> shapes; //for each of faces can be removed
// std::vector<full_object_detection> shape;
full_object_detection ws = fshape(img,dets[0]);
cout << "number of parts: "<< ws.num_parts() << endl;

// shape.push_back(ws);

std::vector<Point2f> trgPoints;
for(size_t i = 0; i < ws.num_parts(); i++){
  trgPoints.push_back(Point2f(ws.part(i)(0), ws.part(i)(1)));
}
cout << "pixel position of first part:  " << trgPoints[18] << endl;
cout << "pixel position of second part: " << trgPoints[19] << endl;
Point2f tt = pointAbove(trgPoints[18], trgPoints[19]);
std::cout << tt<< '\n';
/////////////////////////////////////////
//SUBDIV



/////////////////////////////////////
//get the points from the file
//points to map the texture
std::vector<Point2f> basePoints;

ifstream ifs("fullface-texture.txt");
float q, w;
while(ifs >> q >> w){
  basePoints.push_back(Point2f(q, w));
}


// * split pic into triangles
// * transfer each fragment triangle to certain location

//////////////////////////////////////////

Mat imgTr = imread(fname1,1); //1 so read in all colors
//detect skin tone by mean calculation
Scalar average = skinTone(trgPoints, imgTr);
imgTr.convertTo(imgTr, CV_32FC3);

// Output image is set to white
Mat txtimage = imread("fullface-texture.jpg",1 );

txtimage.convertTo(txtimage, CV_32FC3);
// Mat txtimage = Mat::ones(Size(1024,1024), imgTr.type());
// txtimage = Scalar(1.0,1.0,1.0);

std::vector< std::vector<int> > trIdx;

//////////
//
// * apply triangulation to 68 vertices
// * apply indices to the triangles
//

std::vector<int> numbers(68);
//fill with 1s
std::iota (std::begin(numbers), std::end(numbers), 1);


trIdx = triangulate_delaunay(txtimage, basePoints);




for (size_t i = 0; i < trIdx.size(); i++){

  std::vector<Point2f> inpt;
  inpt.push_back(trgPoints[trIdx[i][0]]);
  inpt.push_back(trgPoints[trIdx[i][1]]);
  inpt.push_back(trgPoints[trIdx[i][2]]);
  std::vector<Point2f> outpt;
  outpt.push_back(basePoints[trIdx[i][0]]);
  outpt.push_back(basePoints[trIdx[i][1]]);
  outpt.push_back(basePoints[trIdx[i][2]]);

  applyFragment(inpt, outpt, txtimage, imgTr);

}


float contrast = 0.78;
int brightness = 41;
// convertTo( OutputArray m, int rtype, double alpha=1, double beta=0 )
// - rtype - depth of the output image
// m[i,j] = alfa * img[i,j] + beta
txtimage.convertTo(txtimage, CV_8UC3, contrast, brightness);

// CLONE SEAMLESSLY
Mat finalOt = applySkinBg(txtimage, basePoints, average);

imshow("Let's see", finalOt);

////////////////////////////

imwrite("final1.jpg", finalOt);
imshow("Morphed Face", txtimage);

///////////////////////////

 // cin.get();
 waitKey(0);
 return(0);
//wait for a character to exit
}
