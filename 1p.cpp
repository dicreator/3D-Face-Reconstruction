
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

std::vector< std::vector<Point> > triangulate_delaunay(Mat& img1, std::vector<Point2f> &basePoints){
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
  //change


  //define size for 2-D vector
  for (size_t i = 0; i < triangleList.size(); i++){
    Vec6f trg = triangleList[i];

    if (rect.contains(Point(cvRound(trg[0]),cvRound(trg[1]))) && rect.contains(Point(cvRound(trg[2]),cvRound(trg[3]))) && rect.contains(Point(cvRound(trg[4]),cvRound(trg[5])))){
      size_list++;
    }
  }

  std::cout << size_list << '\n';
  std::vector< std::vector<Point> > pnt(size_list);
  std::vector<Point> pit(3);

  //Loop to insert tiangle points into the 2D vector
  for (size_t i = 0; i < triangleList.size(); i++){
    Vec6f trg = triangleList[i];

    pit[0] = Point(cvRound(trg[0]), cvRound(trg[1]));
    pit[1] = Point(cvRound(trg[2]), cvRound(trg[3]));
    pit[2] = Point(cvRound(trg[4]), cvRound(trg[5]));

    // Identify is the point inside the rectangle
    if ( rect.contains(pit[0]) && rect.contains(pit[1]) && rect.contains(pit[2])){

          //insert triangle points
          pnt[row].push_back(pit[0]);
          pnt[row].push_back(pit[1]);
          pnt[row].push_back(pit[2]);
          row++;
        }
  }
  return pnt;
}


int main( int argc, char** argv){

  //set Width and Hight for an output picture
  // int w = 600;
  // int h = 600;

  //for boundary boxes
  //function returns object_detector
  frontal_face_detector detector = get_frontal_face_detector();
  //predict face landmark positions - 68 .dat import
  shape_predictor fshape;

  deserialize("shape_predictor_68_face_landmarks.dat") >> fshape; // change two lines into one

  image_window win, win_faces;
  array2d<rgb_pixel> img;
  string fname1("Untitled Folder/face-straight1.jpg");
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
cout << "pixel position of first part:  " << ws.part(0) << endl;
cout << "pixel position of second part: " << ws.part(67) << endl;
// shape.push_back(ws);

std::vector<Point2f> trgPoints;
for(size_t i = 0; i < ws.num_parts(); i++){
  trgPoints.push_back(Point2f(ws.part(i)(0), ws.part(i)(1)));
}


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
//Traingulate
// Mat img1 = imread("fullface-texture.jpg");
Mat imgTr = imread(fname1,1); //1 so read in all colors
Mat outImg;
imgTr.convertTo(imgTr, CV_32FC3);

// Output image is set to white
Mat txtimage = imread("fullface-texture.jpg",1 );
txtimage.convertTo(txtimage, CV_32FC3);
// Mat txtimage = Mat::ones(Size(1024,1024), imgTr.type());
txtimage = Scalar(1.0,1.0,1.0);


std::vector< std::vector<Point> > pnt;

pnt = triangulate_delaunay(txtimage, basePoints);

for(auto tri : pnt){
  cout << tri << '\n';
}

std::vector<Point2f> inpt;
inpt.push_back(trgPoints[37]);
inpt.push_back(trgPoints[0]);
inpt.push_back(trgPoints[3]);
std::vector<Point2f> outpt;
outpt.push_back(basePoints[37]);
outpt.push_back(basePoints[0]);
outpt.push_back(basePoints[3]);



// Rect r = boundingRect(midPt);
Rect r1 = boundingRect(inpt);
Rect r2 = boundingRect(outpt);


//////
std::vector<Point2f> t1Rect, t2Rect;
std::vector<Point> tRectInt;
for(int i = 0; i < 3; i++)
{

    t1Rect.push_back( Point2f( inpt[i].x - r1.x, inpt[i].y -  r1.y) ); //rect 1
    t2Rect.push_back( Point2f( outpt[i].x - r2.x, outpt[i].y - r2.y) ); //rect2
    tRectInt.push_back( Point((int)(outpt[i].x - r2.x), (int)(outpt[i].y - r2.y)) ); // for fillConvexPoly
}////

Mat img1Rect;
imgTr(r1).copyTo(img1Rect);





/////////////////////////
// if (0 <= r2.x
//   && 0 <= r2.width
//   && r2.x + r2.width <= txtimage.cols
//   && 0 <= r2.y
//   && 0 <= r2.height
//   && r2.y + r2.height <= txtimage.rows){
    // your code
    /////////////////////
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
fillConvexPoly(mask, tRectInt, Scalar(1.0, 1.0, 1.0), 16, 0);

// apply traingl to the image
multiply(img2Rect, mask, img2Rect);
multiply(txtimage(r2), Scalar(1.0,1.0,1.0) - mask, txtimage(r2));

txtimage(r2) = txtimage(r2) + img2Rect;

txtimage.convertTo(txtimage, CV_8UC3);

// imwrite("save-texture.jpg", txtimage);
imshow("Morphed Face", txtimage);


// }
// else{
//     return -1;
//   }


///////////////////////////

//
// // render face to a window
// win.clear_overlay();
// win.set_image(img);
// //show face landmark
// win.add_overlay(render_face_detections(ws));
//
// // crop face *don't need for now
// dlib::array2d<rgb_pixel> face_chip;
// extract_image_chip(img, get_face_chip_details(ws), face_chip);
// win_faces.set_image(face_chip);

 // cin.get();
 waitKey(0);
 return(0);
//wait for a character to exit
}
