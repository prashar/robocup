#include <cv.h>
#include <highgui.h>
#include <stdio.h>

#define k_lthresh 10
#define k_hthresh 100
#define k_apert 3
#define k_tol 45 

// From digital color monitor ..
#define k_BALLR 168 
#define k_BALLG 69
#define k_BALLB 46

void showImage(IplImage * image, const char * title) ; 

void printImageInfo(IplImage * img) {
  int img_width,img_height ; 
  CvSize img_size ; 
  img_size = cvGetSize(img) ; 
  fprintf(stderr,"Img Width : %d\n",img_size.width) ; 
  fprintf(stderr,"Img Height : %d\n",img_size.height) ; 
  fprintf(stderr,"Num channels : %d\n",img->nChannels) ; 
  fprintf(stderr,"Depth : %d\n",img->depth) ; 
}

// Induction of some blurring .. 
int smoothImage(IplImage *img){
  if(!img)
    return 0 ; 
  IplImage * smooth_image = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,3) ; 
  cvSmooth(img,smooth_image,CV_GAUSSIAN,3,3) ; 
  cvShowImage("SmoothImage",smooth_image) ; 
  cvWaitKey(0) ; 
  cvReleaseImage(&smooth_image) ; 
  cvDestroyWindow("SmoothImage") ; 
}

// Scale the image up by a certain factor ..
IplImage* scaleUp(IplImage* in, int filter=IPL_GAUSSIAN_5x5){
  IplImage * out = cvCreateImage(cvSize(in->width*2,in->height*2),in->depth,in->nChannels) ; 
  cvPyrUp(in,out,filter) ; 
  return out ; 
}

// Break the image into it's grayscale and just detect edges. 
int detEdges(IplImage * in){
  IplImage *grayimg = cvCreateImage(cvSize(in->width,in->height),in->depth,1); 
  IplImage *plainlines = cvCreateImage(cvSize(in->width,in->height),in->depth,1); 
  cvCvtColor(in,grayimg,CV_BGR2GRAY) ; 
  cvCanny(grayimg,plainlines,k_lthresh,k_hthresh,k_apert) ; 
  showImage(plainlines,"DetectedLines");
  return 1 ; 
}

// Highlight a certain part with a rectangle. 
IplImage * markROI(IplImage * src,int x,int y,int width,int height,int inc){
  cvSetImageROI(src,cvRect(x,y,width,height)) ; 
  cvAddS(src,cvScalar(inc),src) ;
  cvResetImageROI(src) ; 
  return src ; 
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
  cvRectangle(src,cvPoint(mix,miy),cvPoint(max,may),CV_RGB(0,255,0),2,0,0) ; 
  return src ; 
}

// Draw a circle on the image. 
IplImage * drawCircle(IplImage *img){
  cvCircle(img,cvPoint(300,300),90,CV_RGB(0,0,0),1,8); 
}

void showImage(IplImage * image, const char * title){
  cvShowImage(title,image) ; 
  cvWaitKey(0) ; 
  cvReleaseImage(&image) ; 
  cvDestroyWindow(title) ; 
}

int main(int argc, char * argv[]) {
  if(!argv[1]){
    fprintf(stderr,"Provide the name of the file to process\n") ; 
    return 0 ; 
  }
  IplImage * input_img = cvLoadImage( argv[1] ) ; 
  IplImage * out ; 
  printImageInfo(input_img) ; 
  cvNamedWindow("360Photo",CV_WINDOW_AUTOSIZE) ;  
  cvShowImage("360Photo",input_img) ; 

  //smoothImage(input_img) ; 
  out = scaleUp(input_img) ; 
  out = detectBall(out) ; 
  //drawCircle(out) ; 
  //out = markROI(out,100,100,300,200,100) ; 
  //detEdges(input_img) ; 
  showImage(out,"ModImage") ; 
  return 0 ; 
}
