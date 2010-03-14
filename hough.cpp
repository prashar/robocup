#include <stdio.h>
#include <cv.h>
#include <math.h>
#include <highgui.h>

#define k_lthresh 10
#define k_hthresh 100
#define k_apert 3
#define k_tol 45 

// From digital color monitor ..
// Manual Calibration with Aammir's video
#define k_BALLR 194
#define k_BALLG 22
#define k_BALLB 0

// For Line Detection .. 
#define THRESH_1 180
#define WHITE 255 
#define CANNY_THRESH_1 166
#define CANNY_THRESH_2 166*3
#define APERTURE_SIZE 3
#define MAX_THRESHOLD 80
#define MIN_LINE_LEN 30
#define MAX_GAP_BET_LINES 20
#define THICKNESS 3
#define TYPE_OF_LINE 8

IplImage * detectBall(IplImage * src){
  int w, h ; 
  CvScalar ballcolor = CV_RGB(k_BALLR,k_BALLG,k_BALLB) ; 
  CvScalar pixcolor ; 
  CvScalar white = CV_RGB(255,255,255) ; 
  CvScalar black = CV_RGB(0,0,0) ; 
  // Rectangle Coordinates .. 
  int miy=10000,may=0,mix=10000,max=0 ; 
  // For each pixel in the src image, get it's color and compare with the ball
  // color
  for(h=0 ; h < src->height ; h++){
    for(w=0 ; w < src->width ; w++){
      pixcolor = cvGet2D(src,h,w) ; 
      // Check if the current pixel color fits the range we're looking for. 
      if( ((pixcolor.val[0] - k_tol) <= ballcolor.val[0]) && 
          ((pixcolor.val[0] + k_tol) >= ballcolor.val[0]) && 
          ((pixcolor.val[1] - k_tol) <= ballcolor.val[1]) && 
          ((pixcolor.val[1] + k_tol) >= ballcolor.val[1]) && 
          ((pixcolor.val[2] - k_tol) <= ballcolor.val[2]) && 
          ((pixcolor.val[2] + k_tol) >= ballcolor.val[2]) ){
        // Record where the ball is 
        if(w>max) max = w ; 
        if(w<mix) mix = w ; 
        if(h<miy) miy = h ; 
        if(h>may) may = h ; 
        //cvSet2D(src,h,w,white);
      }else{
        //cvSet2D(src,h,w,black);
      }

    }
  }
  cvRectangle(src,cvPoint(mix,miy),cvPoint(max,may),CV_RGB(0,255,0),2,0,0) ; 
  return src ; 
}

IplImage * detectLines(IplImage *frame){

  IplImage  *grey       = NULL;
  IplImage  *white      = NULL;	
  IplImage  *edges      = NULL;
  IplImage  *color_dst  = NULL;
  CvSeq* lines = 0;
  CvMemStorage* storage = cvCreateMemStorage(0);

  /* Get the frame and also create grey frame of same size*/
  grey      = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
  white     = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
  edges     = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
  color_dst = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);

    // Convert to grayscale
    cvCvtColor(frame, grey, CV_BGR2GRAY);
    // binarize the grayscale image, i.e. if pixel(x,y) is > THRESH_1
    // then set pixel(x,y) to MAX_THRESH - This is the CV_THRESH_BINARY option. 
    cvThreshold(grey, white, THRESH_1, WHITE , CV_THRESH_BINARY);
    // Write the edges into another image edges. 
    // If pixel magnitude between CANNY_THRESH_1 and CANNY_THRESH_2, then make
    // it zero, if above CANNY_THRESH_2, then make it an edge. 
    cvCanny(white, edges, CANNY_THRESH_1 , CANNY_THRESH_2, APERTURE_SIZE);
    color_dst = frame ; 
    // Find all lines int the binary image edges. 
    lines = cvHoughLines2(edges, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, MAX_THRESHOLD, MIN_LINE_LEN, MAX_GAP_BET_LINES );
    for (int i = 0; i < lines->total; i++)
    {
      CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
      cvLine(color_dst, line[0], line[1], CV_RGB(255, 0, 0), THICKNESS, TYPE_OF_LINE);
	}

    /*
    cvNamedWindow("original", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("grey", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("white", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("edges", CV_WINDOW_AUTOSIZE);
	cvShowImage("grey", grey);
	cvShowImage("white", white);
	cvShowImage("edges",edges);
    */

    return color_dst ; 
}

void usage(){
  printf("./hough -v|-a filename\n") ; 
}


int main(int argc, char* argv[])
{
  int delay = 0, key = 0, i = 0;
  char c ; 
  char *window_name     = NULL ;
  short vidFlagOn       = 0 ;              
  int errorFlagOn       = 0 ; 
  char      * fname     = NULL ; 
  CvCapture * video     = NULL ; 
  IplImage  * pic       = NULL ; 
  
  for(int i= 0 ; i < argc ; i++){

    if(!strncmp(argv[i],"-v",2)){
      vidFlagOn = 1 ; 
      if(argv[i+1] == NULL){
        errorFlagOn = 1 ; 
      }else{
        fname = argv[i+1] ; 
      }
      i++ ; 
    }else if(!strncmp(argv[i],"-a",2)){
      vidFlagOn = 0 ; 
      if(argv[i+1] == NULL){
        errorFlagOn = 1 ; 
      }else{
        fname = argv[i+1] ; 
      }
      i++ ; 
    }

    if(errorFlagOn){
      usage() ; 
      return -1 ; 
    }
  }

  if(!vidFlagOn){
    pic = cvLoadImage( fname );
  }else{
    video = cvCaptureFromFile( fname );
    if(!video)
    {
      printf("Unable to open file\n");
      return 1;
    }
    pic = cvQueryFrame(video) ; 
    /* Display fps and calculate delay */
    printf("Video FPS: %2.2f \n", cvGetCaptureProperty(video, CV_CAP_PROP_FPS));
    delay = (int) (1000/cvGetCaptureProperty(video, CV_CAP_PROP_FPS));
  }

  window_name = "VisionSimulator" ;  
  cvNamedWindow(window_name, CV_WINDOW_AUTOSIZE);
  while(pic)
  {
    IplImage * processed = detectLines(pic);
    // Detect the ball first. 
    detectBall(processed) ; 
	/* show the loaded image with edges detected*/
    cvShowImage(window_name, processed);

    // If The user requested an image just be displayed, just wait .. 
    if(!vidFlagOn){
      while(1){
        c = cvWaitKey(33);
        if(c == 27) break;
      }
      break ; 
    }
    
    /* load and check next frame */
    pic = cvQueryFrame(video);
    if(!pic)
    {
      printf("Error loading frame\n");
      return 1;
    }

    /* wait delay and check for quit key */
    c = cvWaitKey(33);
    if(c == 27) break;
  }
}

