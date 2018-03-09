
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

int main( int argc, char** argv){

  //set Width and Hight for an output picture
  int w = 600;
  int h = 600;

  //for boundary boxes
  //function returns object_detector
  frontal_face_detector detector = get_frontal_face_detector();
  //predict face landmark positions - 68 .dat import
  shape_predictor fshape;

  deserialize("shape_predictor_68_face_landmarks.dat") >> fshape; // change two lines into one

  image_window win, win_faces;
  array2d<rgb_pixel> img;
  //
load_image(img,argv[1]);

//increase image?
pyramid_up(img);

std::vector<dlib::rectangle> dets = detector(img);

//if more than one face was detected
if (dets.size() > 1) {
  cout << "More than one face was detected" << endl;
  cout << "We will proceed with the first detected face" << endl;
}


// std::std::vector<full_object_detection> shapes; //for each of faces can be removed
// std::vector<full_object_detection> shape;
full_object_detection ws = fshape(img,dets[0]);
cout << "number of parts: "<< ws.num_parts() << endl;
cout << "pixel position of first part:  " << ws.part(0)[0] << endl;
cout << "pixel position of second part: " << ws.part(67) << endl;
// shape.push_back(ws);


//render face to a window
win.clear_overlay();
win.set_image(img);
//show face landmark
win.add_overlay(render_face_detections(ws));

//crop face *don't need for now
dlib::array2d<rgb_pixel> face_chip;
extract_image_chip(img, get_face_chip_details(ws), face_chip);
win_faces.set_image(face_chip);

 cin.get();
//wait for a character to exit
}
