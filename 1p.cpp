/*
|--------------------------------------------------------------------------
| 3D Face Reconstruction
|--------------------------------------------------------------------------
|
| This project exists to create 3D face models that are generated from 2D images.
| @author Dinmukhamed Komekov <@dicreator>
| @version 1.0
*/
//import libraries
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <sstream>
#include <vector>
//
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>

#include <ctime>

using namespace dlib;
using namespace cv;
using namespace std;


// =======================================================================//
// ! Gives a distance in pixels between 2 point on an image               //
// =======================================================================//

static double distanceFound(Point2f first, Point2f second) {
   double dis = norm(first - second);
   return dis;

}

// =======================================================================//
// ! Detects extra forehead points from an image (its list of points)     //
// =======================================================================//

static void pointAbove(std::vector < Point2f > & trgPoints) {

   Point2f first = trgPoints[27];
   Point2f second = trgPoints[30];

   Point2f topPoint = Point2f(first.x - (second.x - first.x) * 0.8, first.y - (second.y - first.y) * 0.8);

   double gradientPerp = -1 / ((second.y - first.y) / (second.x - first.x));

   Point2f pntAboveL;
   Point2f pntAboveR;

   pntAboveL.x = topPoint.x - (second.y - first.y) * 0.4;
   pntAboveR.x = topPoint.x + (second.y - first.y) * 0.4;

   pntAboveL.y = (gradientPerp * (pntAboveL.x - topPoint.x)) + topPoint.y;
   pntAboveR.y = (gradientPerp * (pntAboveR.x - topPoint.x)) + topPoint.y;

   //add points
   trgPoints.push_back(pntAboveL);
   trgPoints.push_back(topPoint);
   trgPoints.push_back(pntAboveR);
}

// =======================================================================//
// ! Detect the skin tone by mean calculation                             //
// =======================================================================//

Scalar skinTone(std::vector < Point2f > trgPoints, Mat avepic) {
   //define a triangle
   std::vector < Point2f > outpt1;
   outpt1.push_back(trgPoints[69]);
   outpt1.push_back(trgPoints[24]);
   outpt1.push_back(trgPoints[19]);
   Rect r11 = boundingRect(outpt1);
   std::vector < Point > tRectInt1;

   //used in tranformation, distance inside rect
   for (int g = 0; g < 3; g++) {
      tRectInt1.push_back(Point((int)(outpt1[g].x - r11.x), (int)(outpt1[g].y - r11.y))); // for fillConvexPoly
   }
   //create a mask
   Mat mask1 = Mat::zeros(r11.height, r11.width, CV_8U);
   fillConvexPoly(mask1, tRectInt1, Scalar(1.0, 1.0, 1.0));
   //retrieve the average skin color from the triangular mask
   Scalar average = mean(avepic(r11), mask1);
   //retur the value
   return average;
}

// =======================================================================//
// ! Adds a face triangle component through Affine Tranformation          //
// =======================================================================//

static void applyFragment(std::vector < Point2f > inpt, std::vector < Point2f > outpt, Mat & txtimage, Mat imgTr) {
   Rect r1 = boundingRect(inpt);
   Rect r2 = boundingRect(outpt);

   ////// offsetting points be left top corner
   std::vector < Point2f > t1Rect, t2Rect;
   std::vector < Point > tRectInt;
   //used in tranformation, distance inside rect
   for (int g = 0; g < 3; g++) {

      t1Rect.push_back(Point2f(inpt[g].x - r1.x, inpt[g].y - r1.y)); //rect 1
      t2Rect.push_back(Point2f(outpt[g].x - r2.x, outpt[g].y - r2.y)); //rect2
      tRectInt.push_back(Point((int)(outpt[g].x - r2.x), (int)(outpt[g].y - r2.y))); // for fillConvexPoly

   }//for

   Mat img1Rect;
   //crop
   imgTr(r1).copyTo(img1Rect);
   //trasnformation for the shape inside the rectangular
   Mat warpMat = getAffineTransform(t1Rect, t2Rect);
   Mat img2Rect = Mat::zeros(r2.height, r2.width, img1Rect.type());

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
   multiply(txtimage(r2), Scalar(1.0, 1.0, 1.0) - mask, txtimage(r2));

   txtimage(r2) = txtimage(r2) + img2Rect;
}

// =======================================================================//
// ! Applies blur filter to smooth face boarders with the Background      //
// =======================================================================//

Mat applySkinBg(Mat txtimage, std::vector < Point2f > basePoints, Scalar average) {
   std::vector < Point2f > hull;
   std::vector < int > hullIdx;
   std::vector < Point > hullfMask;
   Mat maskHull = Mat::zeros(txtimage.rows, txtimage.cols, txtimage.depth());
   Point center;

   convexHull(basePoints, hullIdx, false, false);

   for (int v = 0; v < hullIdx.size(); v++) {
      hull.push_back(basePoints[hullIdx[v]]);
      //for mask
      Point pt1(hull[v].x, hull[v].y);
      hullfMask.push_back(pt1);

   }

   //Create mask1
   fillConvexPoly(maskHull, & hullfMask[0], hullfMask.size(), Scalar(255, 255, 255));

   //Clone seamlessly
   Rect rHull = boundingRect(hull);
   center = (rHull.tl() + rHull.br()) / 2;

   Mat bgSkin(txtimage.rows, txtimage.cols, CV_8UC3, average);
   Mat finalOutput;
   seamlessClone(txtimage, bgSkin, maskHull, center, finalOutput, NORMAL_CLONE);
   return finalOutput;

}

// =======================================================================//
// ! Divide image into triangles with the given list of facial landmarks  //
// =======================================================================//

std::vector < std::vector < int > > triangulate_delaunay(Mat & img1, std::vector < Point2f > & basePoints) {
   // Keep a copy around

   // Rectangle to be used with Subdiv2D
   Size size = img1.size();
   Rect rect(0, 0, size.width, size.height);
   Subdiv2D subdiv(rect);
   int size_list = 0;
   int row = 0;
   for (std::vector < Point2f > ::iterator it = basePoints.begin(); it != basePoints.end(); it++) {
      subdiv.insert( * it);
   }

   std::vector < Vec6f > triangleList;
   subdiv.getTriangleList(triangleList);

   //Find the number of the triangles inside the face region
   for (size_t i = 0; i < triangleList.size(); i++) {
      Vec6f trg = triangleList[i];
      if (rect.contains(Point(cvRound(trg[0]), cvRound(trg[1]))) && rect.contains(Point(cvRound(trg[2]), cvRound(trg[3]))) && rect.contains(Point(cvRound(trg[4]), cvRound(trg[5])))) {
         size_list += 1;
      }
   }

   std::vector < Point2f > pit(3);
   std::vector < std::vector < int > > indaxes(size_list);

   //////////////////////////////////////
   //Loop to find indexes to structure triangles
   for (size_t i = 0; i < triangleList.size(); i++) {
      Vec6f trg = triangleList[i];

      pit[0] = Point2f(trg[0], trg[1]);
      pit[1] = Point2f(trg[2], trg[3]);
      pit[2] = Point2f(trg[4], trg[5]);

      // Identify is the point inside the rectangle
      if (rect.contains(Point(cvRound(trg[0]), cvRound(trg[1]))) && rect.contains(Point(cvRound(trg[2]), cvRound(trg[3]))) && rect.contains(Point(cvRound(trg[4]), cvRound(trg[5])))) {

         for (int j = 0; j < 3; j++) {
            bool found = false;
            auto pos = std::find(basePoints.begin(), basePoints.end(), pit[j]);
            if (pos != basePoints.end()) found = true;
            if (found) {
               int index = std::distance(basePoints.begin(), pos);

               // std::cout << index << '\n';
               indaxes[row].push_back(index);

            } else {
               std::cout << "not found:\t" << * pos << '\n';
            }
         }
         row++;
      } //if inside
   } //for

   return indaxes;
}

// =======================================================================//
// ! Make a copy of the image         //
// =======================================================================//

static void makeCopy(const char * src, string dst) {
   std::ifstream srce(src, std::ios::binary);
   std::ofstream dest(dst, std::ios::binary);
   dest << srce.rdbuf();
}

// =======================================================================//
//  MAIN                                                                  //
// =======================================================================//

int main(int argc, char * * argv) {

   //for boundary boxes
   //function returns object_detector
   clock_t begin = clock();

   frontal_face_detector detector = get_frontal_face_detector();
   //predict face landmark positions - 68 .dat import
   shape_predictor fshape;

   deserialize("shape_predictor_68_face_landmarks.dat") >> fshape; // change two lines into one

   array2d < rgb_pixel > img;
   string fname1(argv[1]);
   load_image(img, fname1);

   //to increase image, uncomment the bottom line
   // pyramid_up(img);

   // * detect faces
   std::vector < dlib::rectangle > dets = detector(img);

   clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
   std::cout << elapsed_secs << '\n';
   //if more than one face was detected
   if (dets.size() == 0 || dets.size() > 1) {
      cout << "Picture contains none or many faces" << endl;
      cout << "Please - use a picture with only one face." << endl;
      exit(0);
   }

   // ==========================================
   //  Set up texture resolution
   int value;
   int txtSize = 1024;
   std::cout << "What resolution of the texture you need? 1 or 2" << '\n';
   std::cout << "(1) 1024x1024  -- recomended " << '\n';
   std::cout << "(2) 256x256" << '\n';
   std::cin >> value;
   while (std::cin.fail() || value > 2 || value <= 0) {
      std::cout << "Please provide the correct number: 1 or 2" << '\n';
      std::cin.clear();
      std::cin.ignore(256, '\n');
      std::cin >> value;
   }

   if (value == 2) {
      txtSize = 256;
      value = 4;
   }

   // ==========================================
   //  Transfer facial landmarks into vecotor of points
   full_object_detection ws = fshape(img, dets[0]);

   std::vector < Point2f > trgPoints;
   for (size_t i = 0; i < ws.num_parts(); i++) {
      trgPoints.push_back(Point2f(ws.part(i)(0), ws.part(i)(1)));
   }
   //add extre forehead points
   pointAbove(trgPoints);

   // ==========================================
   //  Store vecotor of targeted points of the UV map.
   std::vector < Point2f > basePoints;
   // read from the file
   ifstream ifs("fullface-texture.txt");
   float q, w;
   while (ifs >> q >> w) {
      basePoints.push_back(Point2f(q / value, w / value));
   }
   //forehead points
   basePoints.push_back(Point2f(325 / value, 140 / value));
   basePoints.push_back(Point2f(512 / value, 134 / value));
   basePoints.push_back(Point2f(699 / value, 140 / value));


   // ==========================================
   //  Calculate Background color + create the texture

   Mat imgTr = imread(fname1, 1); //1 so read in all colors
   //detect skin tone by mean calculation
   Scalar average = skinTone(trgPoints, imgTr);

   ////////
   // Calculate brightness and contrast
   // double min, max;
   // minMaxLoc(imgTr, &min, &max);
   // std::cout << min << max << '\n';
   // double brightness = 255/(max- min);
   // double contrast = -min * brightness;
   // std::cout << "ll "<< min << max << '\n';
   //////////////

   imgTr.convertTo(imgTr, CV_32FC3);

   // Create an empty texture matrix filled with the skin color
   Mat txtimage = Mat::ones(Size(txtSize, txtSize), imgTr.type());
   txtimage = Scalar(average);
   txtimage.convertTo(txtimage, CV_32FC3);

   // ==========================================
   // * apply triangulation to 68 vertices
   // * apply indices to the triangles
   //
   std::vector < std::vector < int > > trIdx;
   trIdx = triangulate_delaunay(txtimage, basePoints);

   for (size_t i = 0; i < trIdx.size(); i++) {

     //Face triangle -extract from
      std::vector < Point2f > inpt;
      inpt.push_back(trgPoints[trIdx[i][0]]);
      inpt.push_back(trgPoints[trIdx[i][1]]);
      inpt.push_back(trgPoints[trIdx[i][2]]);

      //UV texture triangle -target  to
      std::vector < Point2f > outpt;
      outpt.push_back(basePoints[trIdx[i][0]]);
      outpt.push_back(basePoints[trIdx[i][1]]);
      outpt.push_back(basePoints[trIdx[i][2]]);

      //Adds a face triangle component through Affine Tranformation
      //to the texture
      applyFragment(inpt, outpt, txtimage, imgTr);

   }

   // convertTo( OutputArray m, int rtype, double alpha=1, double beta=0 )
   // - rtype = CV_8UC3 - depth of the output image
   // finalM[i,j] = brightness * img[i,j] + contrast
   float contrast = 0.78;
   int brightness = 41;
   txtimage.convertTo(txtimage, CV_8UC3, contrast, brightness);

   // ==========================================
   // CLONE SEAMLESSLY -- apply skin tone to Background

   //Smooth face boarders with the Background
   Mat finalOt = applySkinBg(txtimage, basePoints, average);

   size_t dotIndex = fname1.find_last_of(".");
   string OtFiles = fname1.substr(0, dotIndex);
   string objName = OtFiles + ".obj";


   // 27.4734
   double ratio = distanceFound(trgPoints[0], trgPoints[16]) / 28.8; //decrease ratio
   // double ratio = distanceFound(trgPoints[0], trgPoints[16])/12;
   std::cout << ratio << '\n';


   // ==========================================
   // 3D Model Creation

   //read points
   std::vector < Point > faceLength;
   ifstream ist("tpoints.txt");
   int s, d;
   while (ist >> s >> d) {
      faceLength.push_back(Point(s, d));
   }

   //read vertices from 3d file
   int tempT;
   int tempY;
   string typeInObj;
   float locationX;
   double poisitionOne;
   double poisitionTwo;
   double changeLengthOne;

   std::vector < string > verticesObj;
   string readline;
   int track = 0;

   ifstream objst("orgFaceD.obj");

   while (getline(objst, readline)) {
      if (track != 0 && track <= 294) {
         verticesObj.push_back(readline);
      } else if (track > 294) {
         break;
      }
      track++;
   }

   // Adjustment of location of vertices

   for (size_t i = 0; i < 14; i++) {

      //Store vertices in 'vector' then adjust location of each vertex
      // good way cuz it copies exactly and even commenting
      // rather than storing and printing.
      // doesn't change anything else except of vertices.

      // x-axis
      istringstream iss1(verticesObj[faceLength[i].x - 1]);
      //find the point coordinate of Left side
      std::vector < string > tokens1 {
         istream_iterator < string > {
               iss1
            },
            istream_iterator < string > {}
      };
      //convert to float
      poisitionOne = atof(tokens1[1].c_str());

      //y-axis
      istringstream iss2(verticesObj[faceLength[i].y - 1]);
      //find the point coordinate of Right side
      std::vector < string > tokens2 {
         istream_iterator < string > {
               iss2
            },
            istream_iterator < string > {}
      };
      //convert to float
      poisitionTwo = atof(tokens2[1].c_str());

      tokens1[0] += ' ';
      tokens2[0] += ' ';

      // Calculte by how much we need to move the vertices
      changeLengthOne = ((distanceFound(trgPoints[faceLength[i + 14].x], trgPoints[faceLength[i + 14].y]) / ratio) - (poisitionTwo - poisitionOne)) / 4;

      //write
      std::cout << changeLengthOne << '\n';
      poisitionOne -= changeLengthOne;

      poisitionTwo += changeLengthOne;

      tokens1[1] = to_string(poisitionOne);
      tokens2[1] = to_string(poisitionTwo);

      for (int i = 0; i < 3; i++) {
         tokens1[i] += ' ';
         tokens2[i] += ' ';
      }

      verticesObj[faceLength[i].x - 1] = accumulate(tokens1.begin(), tokens1.end(), string(""));
      verticesObj[faceLength[i].y - 1] = accumulate(tokens2.begin(), tokens2.end(), string(""));

      //read te
   }


   string textureFname = OtFiles + "Texture.jpg";
   imwrite(textureFname, finalOt);

   ifstream mtlst("orgFaceD.mtl");
   ofstream mtlOt(OtFiles + ".mtl");
   track = 0;

   string writeMtl;
   while (getline(mtlst, writeMtl)) {
      if (track < 11) {
         mtlOt << writeMtl << '\n';
      } else {
         mtlOt << "  " << "map_Ka " << textureFname << '\n';
         mtlOt << "  " << "map_Kd " << textureFname << '\n';
         mtlOt.flush();
         break;
      }
      track++;
   }

   std::cout << OtFiles << '\n';
   ifstream objst1("orgFaceD.obj");
   ofstream objOt(OtFiles + ".obj");

   track = 0;
   // for (int i = 0; i < 294; i++){
   //   objOt << verticesObj[i] << '\n';
   //
   // }
   string readout;
   while (!objst1.eof()) {
      std::getline(objst1, readout);
      if (track == 0 || track > 294) {
         objOt << readout << '\n';
         objOt.flush();

      } else {
         objOt << verticesObj[track - 1] << '\n';
      }
      track++;
   }

   // std::cout << "/* message */" << '\n';
   // for (int i = 0; i<8; i++){
   //   double dist = distanceFound(trgPoints[i], trgPoints[16-i]);
   //
   //   std::cout << dist << '\n';
   // }

   //////////////////MAKE FOLDER
   //
   // const int dir_err = mkdir("foo", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
   // if (-1 == dir_err)
   // {
   //     printf("Error creating directory!n");
   //     exit(1);
   // }

   //imshow("Let's see", finalOt);

   ////////////////////////////

   //imshow("Morphed Face", txtimage);
   //imwrite("final3.jpg", txtimage);

   ///////////////////////////

   // cin.get();
   //waitKey(0);
   return (0);
   //wait for a character to exit
}
