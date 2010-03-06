#include <stdio.h>
#include <cv.h>
#include <math.h>
#include <highgui.h>

#define k_lthresh 10
#define k_hthresh 100
#define k_apert 3
#define k_tol 45 

// From digital color monitor ..
#define k_BALLR 168 
#define k_BALLG 69
#define k_BALLB 46

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


int main(int argc, char* argv[])
{
        int delay = 0, key = 0, i = 0;
        char *window_name;
        CvCapture *video = NULL;
        IplImage  *frame = NULL;
        IplImage  *grey  = NULL;
	      IplImage  *white = NULL;	
        IplImage  *edges = NULL;
        IplImage  *color_dst = NULL;
        CvSeq* lines = 0;
        CvMemStorage* storage = cvCreateMemStorage(0);
        IplImage  *frame_with_ball  = NULL;
	//CvVideoWriter *writer = 0;

	if(argc > 1)
        {
                video = cvCaptureFromFile(argv[1]);
        }
        else
        {
                printf("Usage: %s VIDEO_FILE\n", argv[0]);
                return 1;
        }

        if(!video)
        {
                printf("Unable to open file\n");
                return 1;
        }

        window_name = argv[1];
        cvNamedWindow(window_name, CV_WINDOW_AUTOSIZE);
	cvNamedWindow("original", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("grey", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("white", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("edges", CV_WINDOW_AUTOSIZE);
	//writer = cvCreateVideoWriter("out.avi", CV_FOURCC('M','J','P','G'), 15, cvSize(640,480));

        /* Get the frame and also create grey frame of same size*/
        frame     = cvQueryFrame(video);
	grey      = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
	white     = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
        edges     = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
        color_dst = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
        frame_with_ball = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);

        /* Display fps and calculate delay */
        printf("%2.2f FPS\n", cvGetCaptureProperty(video, CV_CAP_PROP_FPS));
        delay = (int) (1000/cvGetCaptureProperty(video, CV_CAP_PROP_FPS));

        while(frame)
        {
		//cvThreshold(frame,frame, 254, 255, CV_THRESH_BINARY);
                cvCvtColor(frame, grey, CV_BGR2GRAY);
		// Originally, 220-255. This eliminates all edges as well
                cvThreshold(grey, white, 180, 255, CV_THRESH_BINARY);
                cvCanny(white, edges, 166 , 166*3, 3);

                //cvCvtColor(grey, color_dst, CV_GRAY2BGR);
                color_dst = frame ; 

                lines = cvHoughLines2(edges, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/360, 15, 30, 10);

                for (i = 0; i < lines->total; i++)
                {
			 CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
                         cvLine(color_dst, line[0], line[1], CV_RGB(255, 0, 0), 3, 8);
		}
          // Detect the ball first. 
          frame_with_ball = detectBall(color_dst) ; 

		 /* show the loaded image with edges detected*/
                cvShowImage(window_name, color_dst);
		//cvShowImage("original", frame_with_ball);
		//cvShowImage("grey", grey);
		//cvShowImage("white", white);
		//cvShowImage("edges",edges);
		//cvWriteFrame(writer, color_dst);
                /* load and check next frame */
                frame = cvQueryFrame(video);
                if(!frame)
                {
                        printf("Error loading frame\n");
                        return 1;
                }

                /* wait delay and check for quit key */
                char c = cvWaitKey(33);
                if(c == 27) break;
        }
}

