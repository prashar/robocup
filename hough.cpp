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

#define k_FIELDR 147
#define k_FIELDG 85
#define k_FIELDB 77

// For Line Detection .. 
#define THRESH_1 180
#define WHITE 215
#define CANNY_THRESH_1 0
#define CANNY_THRESH_2 255
#define APERTURE_SIZE 3
#define MAX_THRESHOLD 70
#define MIN_LINE_LEN 70
#define MAX_GAP_BET_LINES 5
#define THICKNESS 3
#define TYPE_OF_LINE 8
#define PIX_DIFF 50

// Camera Centre
#define CAMERA_X 310
#define CAMERA_Y 250

int global_match ;

void capImage(char * fname,IplImage * pic){
  if(!cvSaveImage(fname,pic)){
    printf("Could not save file \n") ;  
  }
}

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
}

// This function just checks whether or not all pixels are accessible or not.
// TODO: Directly access bytes instead of cvGet2D().
void checkImagePixels(IplImage * src){
	  int height=0,width=0;

	  if( src ){
	    height = src->height ;
	    width = src->width ;
	  }

	 for(int i=0 ; i < width ; i++){
		  for(int k=0 ; k < height ; k++){
			  CvScalar pix = cvGet2D(src,k,i);
			  printf("Pixel [%d][%d] has color B[%f]G[%f]R[%f][%f] \n",k,i,pix.val[0],pix.val[1],pix.val[2],pix.val[3]);
		  }
	  }
}

bool match(CvScalar pixcolor, CvScalar comparecolor){
      if( ((pixcolor.val[0] - k_tol) <= comparecolor.val[0]) && 
          ((pixcolor.val[0] + k_tol) >= comparecolor.val[0]) && 
          ((pixcolor.val[1] - k_tol) <= comparecolor.val[1]) && 
          ((pixcolor.val[1] + k_tol) >= comparecolor.val[1]) && 
          ((pixcolor.val[2] - k_tol) <= comparecolor.val[2]) && 
          ((pixcolor.val[2] + k_tol) >= comparecolor.val[2]) ){
        return true ;   
      }
        return false ; 
}

IplImage * detectCorners(IplImage * src, CvSeq * lines){

	// Need these for clamping ..
	int height=0,width=0;
	if( src ){
		height = src->height ;
		width = src->width ;
	}
	//printf("Width is %d, Height is %d\n",src->width,src->height);

  CvScalar fieldcolor = CV_RGB(k_FIELDR,k_FIELDG,k_FIELDB) ; 
  CvScalar whitecolor = CV_RGB(255,255,255) ; 
  unsigned clamp_left,clamp_right,clamp_top,clamp_bottom ;
  //printf("Total Number of Lines detected: %d\n",lines->total) ;

  // For each line
  //	For each of the 2 end points(x and y)
  //		find the (x-10,y) (x+10,y) (x,y+10) (x,y-10) and check whether they are brown or white
  //		If they fit the model, we need to draw a circle on the point.
  for (int i = 0; i < lines->total; i++)
  {
    CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
    for (int j=0 ; j < 2 ; j++){
      //printf("Line %d %d\n",line[j].x,line[j].y) ;
      //CvScalar pixcolor = cvGet2D(src,line[j].y,line[j].x) ;
      CvMat * mat = (CvMat*)src ;

      //printf("mat has %d rows, and %d cols\n",(unsigned)(mat->cols),(unsigned)(mat->rows));

      // Clamp all values to not exceed image dimensions.
      if( (line[j].x - PIX_DIFF) <= 0 )
        clamp_left = 0 ; 
      else
        clamp_left = line[j].x - PIX_DIFF ;

      if( (line[j].x + PIX_DIFF) >= width ){
    	  //printf("Hitting this \n");
    	  clamp_right = width-1 ;
      }
      else{
    	  clamp_right = line[j].x + PIX_DIFF ;
    	  //printf("clamp_right set to %d %d\n",clamp_right, width);
      }

      if( (line[j].y - PIX_DIFF) <= 0 )
        clamp_bottom = 0 ;
      else
        clamp_bottom = line[j].y - PIX_DIFF ;

      if( (line[j].y + PIX_DIFF) >= height )
        clamp_top = height-1 ;
      else
        clamp_top = line[j].y + PIX_DIFF ;

      //printf("\n ct %d, cb %d, cl %d, cr %d \n",clamp_top,clamp_bottom,clamp_left,clamp_right);

      // What is the color of (pix + 10) pixels out from the centre pixel ?
      //printf("Matching for color_left ... \n");
      CvScalar pixcolor_left = cvGet2D(src,line[j].y,clamp_left) ;
     // printf("Matching for color_right where clamp_right %u, and line[j].y is %u ... \n",clamp_right,line[j].y);
      CvScalar pixcolor_right = cvGet2D(src,line[j].y,clamp_right) ;
      //printf("Matching for color_right where clamp_top %u, and line[j].x is %u ... ... \n",clamp_top,line[j].x);
      CvScalar pixcolor_top = cvGet2D(src,clamp_top,line[j].x) ;
      //printf("Matching for color_top ... \n");
      CvScalar pixcolor_bottom = cvGet2D(src,clamp_bottom,line[j].x) ;

      //printf("Matching for color ... \n");
      if( match(pixcolor_left,whitecolor) && match(pixcolor_right,fieldcolor) 
          && match(pixcolor_top,whitecolor) && match(pixcolor_bottom,fieldcolor) ){
        cvCircle(src,line[j],5,cvScalar(0,0,255),1); 
        printf("Match found @ %d, %d\n",line[j].x,line[j].y);
        global_match=1 ;
      }
    }
	}
  return src ; 
}

IplImage * detectLines(IplImage *frame){

  IplImage  *grey       = NULL;
  IplImage  *white      = NULL;	
  IplImage  *edges      = NULL;
  IplImage  *color_dst  = NULL;
  IplImage  *edge_detect  = NULL;
  CvSeq* lines = 0;
  CvMemStorage* storage = cvCreateMemStorage(0);

  /* Get the frame and also create grey frame of same size*/
  grey      = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
  white     = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
  edges     = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
  color_dst = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
  edge_detect = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);

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
    lines = cvHoughLines2(edges, storage, CV_HOUGH_PROBABILISTIC, 9, CV_PI/180, MAX_THRESHOLD, MIN_LINE_LEN, MAX_GAP_BET_LINES );
    //printf("Total Number of Lines detected: %d\n",lines->total) ;
    for (int i = 0; i < lines->total; i++)
    {
      CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
      //printf("P and Q: %d %d\n",line[0].x,line[0].y);  
      cvLine(color_dst, line[0], line[1], CV_RGB(255, 0, 0), THICKNESS, TYPE_OF_LINE);
	  }
/*
     float rho_arr[100] ; 
     float theta_arr[100] ; 
     float par_lines[100][2] ; 
     lines = cvHoughLines2( edges, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, 75, 0, 0 );

        for( int i = 0; i < MIN(lines->total,100); i++ )
        {
            float* line = (float*)cvGetSeqElem(lines,i);
            float rho = line[0];
            float theta = line[1];
            CvPoint pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            cvLine( color_dst, pt1, pt2, CV_RGB(255,0,0), 3, 8 );
            rho_arr[i] = rho ; 
            theta_arr[i] = theta ; 
        }
        int i=0,j=0,parlines=0 ; 
        //printf("Total Lines are %d , MIN():%d\n") ; 
        for(i = 0; i < MIN(lines->total,100); i++ ){
          //printf("rho[i] %f, theta[i] %f \n",rho_arr[i], theta_arr[i]) ; 
          for(j = 0; j < MIN(lines->total,100); j++ ){
            if( i != j){
                //printf("------->: %f with %f (%lf)\n",theta_arr[i],theta_arr[j], a) ; 
              //printf("Comparing %f with %f\n",theta_arr[i],theta_arr[j]) ; 
              double a = theta_arr[i] - theta_arr[j] ; 
              if(a < 0) a*=-1 ;
              if(a < 0.02){
                par_lines[parlines][0] = rho_arr[i] ; 
                par_lines[parlines][1] = theta_arr[i] ; 
                parlines++ ; 
                par_lines[parlines][0] = rho_arr[j] ; 
                par_lines[parlines][1] = theta_arr[j] ; 
                parlines++ ; 
              }
            }
          }
        }

        printf("Parallel Lines :: \n") ; 
        for( i = 0; i < parlines ; i++ ){
                //printf("------->: %f with %f \n",par_lines[i][0],par_lines[i][0]) ; 
                printf("a) rho[i] %f, theta[i] %f \n",par_lines[parlines][0], par_lines[parlines][1]) ; 
        }
    */
    edge_detect = detectCorners(frame,lines);

    cvNamedWindow("edge_detect", CV_WINDOW_AUTOSIZE);
	  cvShowImage("edge_detect", edge_detect);
	  if(global_match)
		  while(1){}
	  global_match=0 ;
	    cvNamedWindow("grey", CV_WINDOW_AUTOSIZE);
	    //capImage("cap2.jpg",white) ;
	    cvNamedWindow("white", CV_WINDOW_AUTOSIZE);
	    cvNamedWindow("edges", CV_WINDOW_AUTOSIZE);
	  cvShowImage("grey", grey);
	  cvShowImage("white", white);
	  cvShowImage("edges",edges);

    return color_dst ; 
}

void usage(){
  printf("./hough -v|-p filename\n") ; 
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
    }else if(!strncmp(argv[i],"-p",2)){
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
    if(vidFlagOn){
    /* Display fps and calculate delay */
      printf("Video FPS: %2.2f \n", cvGetCaptureProperty(video, CV_CAP_PROP_FPS));
      delay = (int) (1000/cvGetCaptureProperty(video, CV_CAP_PROP_FPS));
    }
  }

  window_name = "VisionSimulator" ;  
  cvNamedWindow(window_name, CV_WINDOW_AUTOSIZE);
  IplImage * processed = NULL ; 
  int count = 0 ; 
  while(pic)
  {
    processed = detectLines(pic);
    // Detect the ball first. 
    detectBall(processed) ; 
    // Mark Camera
    cvCircle(processed,cvPoint(CAMERA_X,CAMERA_Y),40,cvScalar(0,0,255),1); 
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

    // Capture Frame 10..
    if(count == 180){
      capImage("cap3.jpg",pic) ; 
    }
    //printf("frame_count is %d\n",count) ;

    /* wait delay and check for quit key */
    c = cvWaitKey(33);
    if(c == 27) break;
    count++ ; 
  }
}

