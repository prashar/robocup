#include <stdio.h>
#include <stdlib.h>
#include <cv.h>
#include <math.h>
#include <highgui.h>

#define k_lthresh 10
#define k_hthresh 100
#define k_apert 3
#define f_tol 30
#define w_tol 75
#define b_tol 40

// BALL COLOUR - ORANGE
#define k_BALLU 196
#define k_BALLV 52

// From digital color monitor ..
// Manual Calibration with Aammir's video
#define k_BALLR 194
#define k_BALLG 22
#define k_BALLB 0

#define k_FIELDR 155
#define k_FIELDG 110
#define k_FIELDB 115

#define k_WHITER 255
#define k_WHITEG 255
#define k_WHITEB 255

// For Line Detection .. 
#define THRESH_1 180
#define WHITE 215
#define CANNY_THRESH_1 0
#define CANNY_THRESH_2 255
#define APERTURE_SIZE 3
#define MAX_THRESHOLD 70
#define MIN_LINE_LEN 5
#define MAX_GAP_BET_LINES 5
#define THICKNESS 3
#define TYPE_OF_LINE 8
#define PIX_DIFF 2
#define DIST_LEFT 8

// Camera Centre
#define CAMERA_X 310
#define CAMERA_Y 250

using namespace std;

int global_match;
float top[15];
int next_avail = 0;

struct point {
	int x;
	int y;
};

struct state {
	CvPoint corners[4];
	double angles[4];
	CvPoint pos_world;
	double angle_to_center;
	CvPoint centre_corners[2];
};

typedef struct state State;

typedef struct point Point;

void capImage(char * fname, IplImage * pic) {
	if (!cvSaveImage(fname, pic)) {
		printf("Could not save file \n");
	}
}

IplImage * detectBall(IplImage * src) {
	int w, h;
	CvScalar ballcolor = cvScalar(0, k_BALLU, k_BALLV);
	CvScalar pixcolor;
	CvScalar white = CV_RGB(255,255,255);
	CvScalar black = CV_RGB(0,0,0);
	// Convert to YUV
	IplImage * img_yuv = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
	cvCvtColor(src, img_yuv, CV_BGR2YCrCb);
	// Rectangle Coordinates ..
	int miy = 10000, may = 0, mix = 10000, max = 0;
	// For each pixel in the src image, get it's color and compare with the ball
	// color
	for (h = 0; h < src->height; h++) {
		for (w = 0; w < src->width; w++) {
			pixcolor = cvGet2D(img_yuv, h, w);
			// Check if the current pixel color fits the range we're looking for.
			if (((pixcolor.val[1] - b_tol) <= ballcolor.val[1])
					&& ((pixcolor.val[1] + b_tol) >= ballcolor.val[1])
					&& ((pixcolor.val[2] - b_tol) <= ballcolor.val[2])
					&& ((pixcolor.val[2] + b_tol) >= ballcolor.val[2])) {
				// Record where the ball is
				if (w > max)
					max = w;
				if (w < mix)
					mix = w;
				if (h < miy)
					miy = h;
				if (h > may)
					may = h;

				cvCircle(src,cvPoint(w,h),2,cvScalar(255,255,255),1);
			} else {
				//cvSet2D(src,h,w,black);
			}

		}
	}
}

// This function just checks whether or not all pixels are accessible or not.
// TODO: Directly access bytes instead of cvGet2D().
void checkImagePixels(IplImage * src) {
	int height = 0, width = 0;

	if (src) {
		height = src->height;
		width = src->width;
	}

	for (int i = 0; i < width; i++) {
		for (int k = 0; k < height; k++) {
			CvScalar pix = cvGet2D(src, k, i);
			printf("Pixel [%d][%d] has color B[%f]G[%f]R[%f][%f] \n", k, i,
					pix.val[0], pix.val[1], pix.val[2], pix.val[3]);
		}
	}
}

// flag == 1 means compare white.
bool match(CvScalar pixcolor, CvScalar comparecolor, int flag) {
	int k_tol;
	if (flag == 1) {
		k_tol = w_tol;
	} else
		k_tol = f_tol;
	if (((pixcolor.val[0] - k_tol) <= comparecolor.val[0]) && ((pixcolor.val[0]
			+ k_tol) >= comparecolor.val[0]) && ((pixcolor.val[1] - k_tol)
			<= comparecolor.val[1]) && ((pixcolor.val[1] + k_tol)
			>= comparecolor.val[1]) && ((pixcolor.val[2] - k_tol)
			<= comparecolor.val[2]) && ((pixcolor.val[2] + k_tol)
			>= comparecolor.val[2])) {
		return true;
	}
	return false;
}

int detectColorChangeY(IplImage *img, int x, int y) {
	int height = img->height;
	CvScalar fieldcolor = CV_RGB(k_FIELDR,k_FIELDG,k_FIELDB);
	CvScalar whitecolor = CV_RGB(255,255,255);
	CvScalar pixcolor = cvGet2D(img, y, x);
	while (y < height) {
		if (match(pixcolor, fieldcolor, 0)) {
			return y;
		} else {
			y--;
			pixcolor = cvGet2D(img, y, x);
		}
	}
	return -1;
}

int detectColorChangeX(IplImage *img, int x, int y) {
	int height = img->height;
	CvScalar fieldcolor = CV_RGB(k_FIELDR,k_FIELDG,k_FIELDB);
	CvScalar whitecolor = CV_RGB(255,255,255);
	CvScalar pixcolor = cvGet2D(img, y, x);
	while (y < height) {
		if (match(pixcolor, fieldcolor, 0)) {
			return x;
		} else {
			x++;
			pixcolor = cvGet2D(img, y, x);
		}
	}
	return -1;
}

double getYFromLine(CvPoint2D32f line1_pts[2], double x) {
	// Get Line 1 equation from pts.
	double a1 = line1_pts[1].y - line1_pts[0].y;
	double b1 = line1_pts[0].x - line1_pts[1].x;
	double c1 = a1 * (line1_pts[0].x) + b1 * (line1_pts[0].y);
	double y = (c1 - a1 * x) / b1;
	return y;
}

double getXFromLine(CvPoint2D32f line1_pts[2], double y) {
	// Get Line 1 equation from pts.
	double a1 = line1_pts[1].y - line1_pts[0].y;
	double b1 = line1_pts[0].x - line1_pts[1].x;
	double c1 = a1 * (line1_pts[0].x) + b1 * (line1_pts[0].y);
	double x = (c1 - b1 * y) / a1;
	return x;
}

// center - The white point that we are drawing everything off - mostly this will be camera centre.
// arc_point - The actual point to which we are trying to find theta off.
// trans - The transition point to which we are trying to find everything related to.
double getTheta(CvPoint center, CvPoint arc_point, CvPoint trans,
		IplImage * src) {
	double len_hyp, len_vleg;
	double theta = 0, centerx = (double) center.x, centery = (double) center.y,
			arcx = (double) arc_point.x, arcy = (double) arc_point.y;
	if (arcx != (double) trans.x) {
		CvPoint2D32f arr[2] = { cvPoint2D32f(centerx, centery), cvPoint2D32f(
				arcx, arcy) };
		double new_arcy = getYFromLine(arr, (double) trans.x);
		printf("Changing arcx from %f to %f\n", arcx, (double) trans.x);
		printf("Changing arcy from %f to %f\n", arcy, new_arcy);
		arcx = (double) trans.x;
		arcy = new_arcy;
		//cvLine(src, center, cvPoint((int)arcx,(int)arcy), CV_RGB(255, 255, 0), THICKNESS, TYPE_OF_LINE);
	}
	// Hypotenuse.
	len_hyp = sqrt(pow((arcx - centerx), 2) + pow(arcy - centery, 2));
	// Vertical Leg.
	len_vleg = sqrt(pow((centerx - (double) trans.x), 2) + pow(centery
			- (double) trans.y, 2));
	printf("Hypotenuse %f, Vertical Leg %f\n", len_hyp, len_vleg);
	printf("centre(%d,%d), arc_point(%d,%d), trans(%d,%d)\n", center.x,
			center.y, arc_point.x, arc_point.y, trans.x, trans.y);
	theta = acos(len_vleg / len_hyp);
	printf("Theta is %f deg and %f radians \n", theta * (180 / (CV_PI)), theta);
	return theta;
}

void pointOnStraightLine(float dist) {
	top[next_avail++] = 7.5 + dist;
}

IplImage * detectCorners(IplImage * src, CvSeq * lines) {

	// Need these for clamping ..
	int height = 0, width = 0;
	if (src) {
		height = src->height;
		width = src->width;
	}
	//printf("Width is %d, Height is %d\n",src->width,src->height);

	CvScalar fieldcolor = CV_RGB(k_FIELDR,k_FIELDG,k_FIELDB);
	CvScalar whitecolor = CV_RGB(255,255,255);
	unsigned clamp_left, clamp_right, clamp_top, clamp_bottom;
	//printf("Total Number of Lines detected: %d\n",lines->total) ;

	// For each line
	//	For each of the 2 end points(x and y)
	//		find the (x-10,y) (x+10,y) (x,y+10) (x,y-10) and check whether they are brown or white
	//		If they fit the model, we need to draw a circle on the point.
	for (int i = 0; i < lines->total; i++) {
		CvPoint* line = (CvPoint*) cvGetSeqElem(lines, i);
		for (int j = 0; j < 2; j++) {
			//printf("Line %d %d\n",line[j].x,line[j].y) ;
			//CvScalar pixcolor = cvGet2D(src,line[j].y,line[j].x) ;
			CvMat * mat = (CvMat*) src;

			//printf("mat has %d rows, and %d cols\n",(unsigned)(mat->cols),(unsigned)(mat->rows));

			// Clamp all values to not exceed image dimensions.
			if ((line[j].x - PIX_DIFF) <= 0)
				clamp_left = 0;
			else
				clamp_left = line[j].x - PIX_DIFF;

			if ((line[j].x + PIX_DIFF) >= width) {
				//printf("Hitting this \n");
				clamp_right = width - 1;
			} else {
				clamp_right = line[j].x + PIX_DIFF;
				//printf("clamp_right set to %d %d\n",clamp_right, width);
			}

			if ((line[j].y - PIX_DIFF) <= 0)
				clamp_bottom = 0;
			else
				clamp_bottom = line[j].y - PIX_DIFF;

			if ((line[j].y + PIX_DIFF) >= height)
				clamp_top = height - 1;
			else
				clamp_top = line[j].y + PIX_DIFF;

			printf("\n ct %d, cb %d, cl %d, cr %d \n", clamp_top, clamp_bottom,
					clamp_left, clamp_right);

			// What is the color of (pix + 10) pixels out from the centre pixel ?
			//printf("Matching for color_left ... \n");
			CvScalar pixcolor_left = cvGet2D(src, line[j].y, clamp_left);
			// printf("Matching for color_right where clamp_right %u, and line[j].y is %u ... \n",clamp_right,line[j].y);
			CvScalar pixcolor_right = cvGet2D(src, line[j].y, clamp_right);
			//printf("Matching for color_right where clamp_top %u, and line[j].x is %u ... ... \n",clamp_top,line[j].x);
			CvScalar pixcolor_top = cvGet2D(src, clamp_top, line[j].x);
			//printf("Matching for color_top ... \n");
			CvScalar pixcolor_bottom = cvGet2D(src, clamp_bottom, line[j].x);

			//printf("Matching for color ... \n");
			if (match(pixcolor_left, whitecolor, 1) && match(pixcolor_right,
					fieldcolor, 0) && match(pixcolor_top, fieldcolor, 0)
					&& match(pixcolor_bottom, whitecolor, 1)) {
				cvCircle(src, line[j], 5, cvScalar(0, 0, 255), 1);
				printf("Match found @ %d, %d\n", line[j].x, line[j].y);
				global_match = 1;
			}
		}
	}
	return src;
}

IplImage * detectLines(IplImage *frame) {

	IplImage *grey = NULL;
	IplImage *white = NULL;
	IplImage *edges = NULL;
	IplImage *color_dst = NULL;
	IplImage *edge_detect = NULL;
	CvSeq* lines = 0;
	CvMemStorage* storage = cvCreateMemStorage(0);

	/* Get the frame and also create grey frame of same size*/
	grey = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
	white = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
	edges = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
	color_dst = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	edge_detect = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);

	// Convert to grayscale
	cvCvtColor(frame, grey, CV_BGR2GRAY);
	// binarize the grayscale image, i.e. if pixel(x,y) is > THRESH_1
	// then set pixel(x,y) to MAX_THRESH - This is the CV_THRESH_BINARY option.
	//cvThreshold(grey, white, THRESH_1, WHITE , CV_THRESH_BINARY);
	// Write the edges into another image edges.
	// If pixel magnitude between CANNY_THRESH_1 and CANNY_THRESH_2, then make
	// it zero, if above CANNY_THRESH_2, then make it an edge.
	cvCanny(grey, edges, CANNY_THRESH_1, CANNY_THRESH_2, APERTURE_SIZE);
	color_dst = frame;
	// Find all lines int the binary image edges.
	//lines = cvHoughLines2(edges, storage, CV_HOUGH_PROBABILISTIC, 9, CV_PI/180, MAX_THRESHOLD, MIN_LINE_LEN, MAX_GAP_BET_LINES );
	//printf("Total Number of Lines detected: %d\n",lines->total) ;
	//    for (int i = 0; i < lines->total; i++)
	//    {
	//      CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
	//      printf("P and Q: %d %d\n",line[0].x,line[0].y);
	//      cvLine(color_dst, line[0], line[1], CV_RGB(255, 0, 0), THICKNESS, TYPE_OF_LINE);
	//	}

	float rho_arr[100];
	float theta_arr[100];
	float par_lines[100][2];
	lines = cvHoughLines2(edges, storage, CV_HOUGH_STANDARD, 1, CV_PI / 180,
			90, 0, 0);

	for (int i = 0; i < MIN(lines->total,100); i++) {
		float* line = (float*) cvGetSeqElem(lines, i);
		float rho = line[0];
		float theta = line[1];
		CvPoint pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		cvLine(color_dst, pt1, pt2, CV_RGB(255,0,0), 3, 8);
		rho_arr[i] = rho;
		theta_arr[i] = theta;
	}
	int i = 0, j = 0, parlines = 0;
	//printf("Total Lines are %d , MIN():%d\n") ;
	for (i = 0; i < MIN(lines->total,100); i++) {
		//printf("rho[i] %f, theta[i] %f \n",rho_arr[i], theta_arr[i]) ;
		for (j = 0; j < MIN(lines->total,100); j++) {
			if (i != j) {
				//printf("------->: %f with %f (%lf)\n",theta_arr[i],theta_arr[j], a) ;
				//printf("Comparing %f with %f\n",theta_arr[i],theta_arr[j]) ;
				double a = theta_arr[i] - theta_arr[j];
				if (a < 0)
					a *= -1;
				if (a < 0.02) {
					par_lines[parlines][0] = rho_arr[i];
					par_lines[parlines][1] = theta_arr[i];
					parlines++;
					par_lines[parlines][0] = rho_arr[j];
					par_lines[parlines][1] = theta_arr[j];
					parlines++;
				}
			}
		}
	}

	printf("Parallel Lines :: \n");
	for (i = 0; i < parlines; i++) {
		//printf("------->: %f with %f \n",par_lines[i][0],par_lines[i][0]) ;
		printf("a) rho[i] %f, theta[i] %f \n", par_lines[parlines][0],
				par_lines[parlines][1]);
	}

	/*edge_detect = detectCorners(frame,lines);

	 cvNamedWindow("edge_detect", CV_WINDOW_AUTOSIZE);
	 cvShowImage("edge_detect", edge_detect);
	 if(global_match)
	 //while(1){}
	 global_match=0 ;
	 */
	cvNamedWindow("grey", CV_WINDOW_AUTOSIZE);
	//capImage("cap2.jpg",white) ;
	cvNamedWindow("white", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("edges", CV_WINDOW_AUTOSIZE);
	cvShowImage("grey", grey);
	cvShowImage("white", white);
	cvShowImage("edges", edges);
	//cvShowImage("color_dst",color_dst);


	return color_dst;
}

double getDistance(CvPoint p1, CvPoint p2) {
	double p1x = (double) p1.x, p1y = (double) p1.y, p2x = (double) p2.x, p2y =
			(double) p2.y;
	double dist = sqrt((p2x - p1x) * (p2x - p1x) + (p2y - p1y) * (p2y - p1y));
	return dist;
}

void usage() {
	printf("./hough -v|-p filename\n");
}

IplImage * parseImage(IplImage * src) {
	// Find P1 given the centre
	int y = 0;
	y = detectColorChangeY(src, CAMERA_X, CAMERA_Y);
	printf("y is %d \n", y);
	CvPoint P1 = { CAMERA_X, y };
	printf("CAMX,CAMY : %d %d\n", P1.x, P1.y);
	cvCircle(src, P1, 5, cvScalar(255, 0, 0), 1);
	return src;
}

CvPoint getWhitePoint(IplImage * src, int rad, CvPoint white) {
	white.x = CAMERA_X;
	white.y = CAMERA_Y;
	CvPoint circum_point;
	int angle = 360, in_rad = 1, out_rad = rad;
	double angle_rad = 0;
	CvScalar whitecolor = CV_RGB(255,255,255);
	for (; angle >= 180; angle -= 20) {
		// Convert to radians
		angle_rad = ((CV_PI) / 180) * angle;
		//white.x = white.x + cos(angle_rad)*in_rad ;
		circum_point.x = white.x + cos(angle_rad) * out_rad;
		//white.y = white.y + sin(angle_rad)*in_rad ;
		circum_point.y = white.y + sin(angle_rad) * out_rad;
		//cvLine( src, white, circum_point, CV_RGB(255,0,0), 2, 8 );
		CvScalar pixcolor = cvGet2D(src, circum_point.y, circum_point.x);
		printf("b,g,r for angle %d is %f %f %f \n", angle, pixcolor.val[0],
				pixcolor.val[1], pixcolor.val[2]);
		if (match(pixcolor, whitecolor, 1)) {
			printf("White Detected .. \n");
			//cvCircle(src,cvPoint(circum_point.x,circum_point.y),5,cvScalar(255,0,255),1);
			//white.x = circum_point.x ;
			//white.y = circum_point.y ;
			return circum_point;
		}
	}
}

double getAngleToDrawLine(IplImage * src, CvPoint white) {
	// x and y are flipped.
	int opp = CAMERA_Y - white.y;
	double camx = (double) CAMERA_X, camy = (double) CAMERA_Y, whitex =
			(double) white.x, whitey = (double) white.y;
	//cvLine( src, cvPoint(CAMERA_X,CAMERA_Y), white, CV_RGB(255,0,0), 2, 8 );
	//cvLine( src, cvPoint(CAMERA_X,CAMERA_Y), cvPoint(white.x,CAMERA_Y), CV_RGB(0,255,0), 2, 8 );
	//cvLine( src, cvPoint(white.x,CAMERA_Y), white, CV_RGB(0,0,255), 2, 8 );
	double hyp = sqrt((whitex - camx) * (whitex - camx) + (whitey - camy)
			* (whitey - camy));
	double angle = asin(opp / hyp);
	printf("Angle is %f with hyp,opp,white.x,white.y: %f %d %d %d \n", angle,
			hyp, opp, white.x, white.y);
	return (angle);
}

// angle is in radians.
void drawRadialLine(IplImage * src, double angle) {
	int out_rad = 120;
	CvPoint circum_point;
	// This is the same as (0 - angle) - for adnan
	double revised_angle = 0 - angle;
	circum_point.x = CAMERA_X + cos(revised_angle) * out_rad;
	circum_point.y = CAMERA_Y + sin(revised_angle) * out_rad;
	//cvLine( src, cvPoint(CAMERA_X,CAMERA_Y), circum_point, CV_RGB(255,0,0), 2, 8 );
}

// angle is in radians.
CvPoint findTransitionPoint(IplImage * src, double angle, CvPoint white) {
	int out_rad = 2;
	CvPoint circum_point;
	CvScalar fieldcolor = CV_RGB(k_FIELDR,k_FIELDG,k_FIELDB);
	for (; out_rad < 200; out_rad += 5) {
		// This is the same as (0 - angle) - for adnan
		double revised_angle = 0 - angle;
		circum_point.x = CAMERA_X + cos(revised_angle) * out_rad;
		circum_point.y = CAMERA_Y + sin(revised_angle) * out_rad;
		CvScalar pixcolor = cvGet2D(src, circum_point.y, circum_point.x);
		if (match(pixcolor, fieldcolor, 0)) {
			//cvLine( src, white, circum_point, CV_RGB(255,0,0), 2, 8 );
			return circum_point;
		}
	}
}

int getIntersection(CvPoint2D32f line1_pts[2], CvPoint2D32f line2_pts[2],
		CvPoint2D32f * intersect) {
	// Get Line 1 equation from pts.
	printf("Line1_pts0 (%f,%f) -- Line1_pts1 %f,%f\n", line1_pts[0].x,
			line1_pts[0].y, line1_pts[1].x, line1_pts[1].y);
	printf("Line2_pts0 (%f,%f) -- Line2_pts1 %f,%f\n", line2_pts[0].x,
			line2_pts[0].y, line2_pts[1].x, line2_pts[1].y);
	double a1 = line1_pts[0].y - line1_pts[1].y;
	double b1 = line1_pts[0].x - line1_pts[1].x;
	double c1 = a1 * (line1_pts[0].x) + b1 * (line1_pts[0].y);
	printf("%f %f %f\n", a1, b1, c1);
	// Get Line 2 equation from pts.
	double a2 = line2_pts[0].y - line2_pts[1].y;
	double b2 = line2_pts[0].x - line2_pts[1].x;
	double c2 = a2 * (line2_pts[0].x) + b2 * (line2_pts[0].y);
	printf("%f %f %f\n", a2, b2, c2);
	double det = (a1 * b2) - (a2 * b1);
	printf("det %f \n", det);
	double x = 0, y = 0;
	if (det == 0) {
		return -1;
	} else {
		x = ((b2 * c1) - (b1 * c2)) / det;
		y = ((a1 * c2) - (a2 * c1)) / det;
		intersect->x = x;
		intersect->y = y;
	}
	printf("x,y is %f %f\n", x, y);
	return 1;
}

int drawRadialFromPoint(IplImage * src, int inner, int outer,
		CvPoint img_centre, CvPoint toBeRet[][2], CvPoint *perp_point) {
	CvPoint circum_point;
	int angle = 360, in_rad = inner, out_rad = outer, k = 1, inc = 0;
	double angle_rad = 0;
	CvScalar whitecolor = CV_RGB(255,255,255);
	CvScalar blackcolor = CV_RGB(0,0,0);
	CvScalar yellow = CV_RGB(255,255,0);
	CvScalar green = CV_RGB(0,255,0);
	CvScalar blue = CV_RGB(0,0,255);
	CvScalar cyan = CV_RGB(0,255,255);
	CvScalar red = CV_RGB(255,0,0);
	CvScalar pixcolor;

	// Collect the intersection point
	CvPoint2D32f pts1[2];
	CvPoint2D32f pts2[2];
	CvPoint2D32f intersection;

	for (; angle >= 0; angle -= 3) {
		// Convert to radians
		angle_rad = ((CV_PI) / 180) * angle;
		for (k = in_rad; k < out_rad; k += 1) {
			circum_point.x = img_centre.x + cos(angle_rad) * k;
			circum_point.y = img_centre.y + sin(angle_rad) * k;
			//printf("Cx,Cy : %d %d\n",circum_point.x,circum_point.y);
			pixcolor = cvGet2D(src, circum_point.y, circum_point.x);
			if (match(pixcolor, whitecolor, 1)) {
				if (k == in_rad)
					continue;
				//printf("b,g,r for angle %d is %f %f %f \n", angle,
				//		pixcolor.val[0], pixcolor.val[1], pixcolor.val[2]);
				//printf("White Detected .. \n");
				// Return the perpendicular point.
				if (angle == 180) {
					perp_point[0].x = circum_point.x;
					perp_point[0].y = circum_point.y;
				}
				cvCircle(src, cvPoint(circum_point.x, circum_point.y), 5,
						cvScalar(255, 0, 255), 1);
				toBeRet[inc][0].x = circum_point.x;
				toBeRet[inc][0].y = circum_point.y;
				inc++;
				break;
			}
		}
		if (!match(pixcolor, blackcolor, 0)) {
			if (!(angle % 90)) {
				cvLine(src, img_centre, circum_point, red, 2, 8);
			} else {
				if (angle > 0 && angle < 90)
					cvLine(src, img_centre, circum_point, yellow, 2, 8);
				if (angle >= 90 && angle < 180)
					cvLine(src, img_centre, circum_point, green, 2, 8);
				if (angle >= 180 && angle < 270)
					cvLine(src, img_centre, circum_point, blue, 2, 8);
				if (angle >= 270 && angle < 360)
					cvLine(src, img_centre, circum_point, cyan, 2, 8);
			}
		}
		// Get points and call the routines.
		if (angle == 90) {
			pts1[0].x = (double) toBeRet[inc][0].x;
			pts1[0].y = (double) toBeRet[inc][0].y;
		}
		if (angle == 110) {
			pts1[1].x = (double) toBeRet[inc][0].x;
			pts1[1].y = (double) toBeRet[inc][0].y;
		}
		if (angle == 165) {
			pts2[0].x = (double) toBeRet[inc][0].x;
			pts2[0].y = (double) toBeRet[inc][0].y;
		}
		if (angle == 180) {
			pts2[0].x = (double) toBeRet[inc][0].x;
			pts2[0].y = (double) toBeRet[inc][0].y;
		}
	}

	int rc = getIntersection(pts1, pts2, &intersection);
	if (rc == -1) {
		printf("Parallel Lines \n");
	} else {
		printf("Intersection @ ( %f, %f  ) \n", intersection.x, intersection.y);
		cvCircle(src, cvPoint(intersection.x, intersection.y), 10, cvScalar(
				255, 255, 255), 1);
	}
	return inc;
}

void occludeView(IplImage *src, CvPoint p1, CvPoint p2) {
	int height = 0, width = 0;

	if (src) {
		height = src->height;
		width = src->width;
	}

	// Construct the 2 edges that will form the bitmask
	CvPoint bot_centre = cvPoint(CAMERA_X, CAMERA_Y);
	for (int i = 0; i < width; i++) {
		for (int k = 0; k < height; k++) {
			CvScalar black = CV_RGB(0,0,0);
			CvScalar pix = cvGet2D(src, k, i);
			double edge_1 = ((bot_centre.x - p1.x) * (k - p1.y)) - ((i - p1.x)
					* (bot_centre.y - p1.y));
			double edge_2 = ((p2.x - bot_centre.x) * (k - bot_centre.y)) - ((i
					- bot_centre.x) * (p2.y - bot_centre.y));
			// For each pixel, set it to black if it's to the right of the line.
			if ((edge_1 < 0) || (edge_2 < 0)) {
				cvSet2D(src, k, i, black);
			}
		}
	}
}

void getHoughBall(IplImage * image) {
	CvMemStorage* storage = cvCreateMemStorage(0);
	cvSmooth(image, image, CV_GAUSSIAN, 5, 5);

	CvSeq* results = cvHoughCircles(image, storage, CV_HOUGH_GRADIENT, 30,
			image->width / 10);

	for (int i = 0; i < results->total; i++) {
		float* p = (float*) cvGetSeqElem(results, i);
		CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));
		cvCircle(image, pt, cvRound(p[2]), CV_RGB(0xff,0xff,0xff)
		);
	}
}

void getDRdata(IplImage *src, CvPoint arr[][2], CvPoint *perp_point, int count) {
	double hyp_world = 0, theta = 0;
	int i = 0;
	for (i = 0; i < count; i++) {
		theta = getTheta(cvPoint(CAMERA_X, CAMERA_Y), arr[i][0], perp_point[0],
				src);
		// Take stuff only less than 45 degrees ..
		//if(theta <= 1.0471){
		hyp_world = (5 / cos(theta));
		printf("%f,%f \n", hyp_world, getDistance(cvPoint(CAMERA_X, CAMERA_Y),
				arr[i][0]));
		printf("----\n");
		//}
	}
}

void findClosestCorner(CvPoint * cam_center, CvPoint * pix_corners,
		CvPoint * closest_corner) {

}

void findClosestElement(CvPoint2D32f * arr, int count) {
	// WRITE THIS ..
}

void getCorners(IplImage * pic, CvPoint img_center, CvPoint *pix_perpendicular,
		CvPoint *pix_corners, double *global_init_angles) {
	CvPoint circum_point;
	int angle = 360, in_rad = 150, out_rad = 220, k = 1, inc = 0;
	double angle_rad = 0;
	CvScalar whitecolor = CV_RGB(255,255,255);
	CvScalar blackcolor = CV_RGB(0,0,0);
	CvScalar yellow = CV_RGB(255,255,0);
	CvScalar green = CV_RGB(0,255,0);
	CvScalar blue = CV_RGB(0,0,255);
	CvScalar cyan = CV_RGB(0,255,255);
	CvScalar red = CV_RGB(255,0,0);
	CvScalar pixcolor;

	// Collect the intersection point
	CvPoint2D32f pts1[2];
	CvPoint2D32f pts2[2];
	CvPoint2D32f intersection;

	// Collect the intersection point
	CvPoint2D32f pts_right[40];
	CvPoint2D32f right_line[2];
	CvPoint2D32f pts_left[40];
	CvPoint2D32f left_line[2];
	CvPoint2D32f pts_bottom[40];
	CvPoint2D32f bottom_line[2];
	int right_count = 0, left_count = 0, bottom_count = 0, count = 0;
	int right_perp_index = 0, bottom_perp_index = 0, left_perp_index = 0,
			top_perp_index = 0;

	// We want to isolate the 4 different corners, so we shoot rays
	// at certain line combinations.
	angle = -90;
	for (; angle <= 90; angle += 3) {
		angle_rad = ((CV_PI) / 180) * angle;
		for (k = in_rad + 1; k < out_rad; k += 1) {
			circum_point.x = img_center.x + cos(angle_rad) * k;
			circum_point.y = img_center.y + sin(angle_rad) * k;
			//printf("Cx,Cy : %d %d\n",circum_point.x,circum_point.y);
			pixcolor = cvGet2D(pic, circum_point.y, circum_point.x);
			CvPoint interestedPoint = cvPoint(circum_point.x, circum_point.y);
			if (match(pixcolor, whitecolor, 1)) {
				//printf("b,g,r for angle %d is %f %f %f \n", angle,
				//		pixcolor.val[0], pixcolor.val[1], pixcolor.val[2]);
				//printf("White Detected .. \n");
				// For case 1 ---> left ray
				if (angle >= -45 && angle < 45) {
					if (angle == 0) {
						right_perp_index = count;
						pix_perpendicular[count] = interestedPoint;
						count++;
					} else {
						pts_right[right_count++]
								= cvPointTo32f(interestedPoint);
					}
					//cvCircle(pic, interestedPoint , 3,
					//	cvScalar(255,0,0), 1);
				} else {
					//cvCircle(pic, interestedPoint , 3,
					//	cvScalar(255, 255, 255), 1);
				}
				break;
			}
		}
	}

	// Get the 2nd coordinate in pixel space on the right line.
	// We already have 1, i.e the perperndicular line segment.
	// hypotenuse is the point from the camera center to another point(in our case last point) on the curvature.
	CvPoint2D32f hypotenuse[2] = { cvPointTo32f(img_center),
			pts_right[right_count - 1] };
	// We want to extend linearly, so first find this coordinate's y
	double sec_coord_y = getYFromLine(hypotenuse,
			pix_perpendicular[right_perp_index].x);
	CvPoint2D32f line_point_2 = cvPoint2D32f(
			(double) pix_perpendicular[right_perp_index].x, sec_coord_y);
	right_line[0] = cvPointTo32f(pix_perpendicular[right_perp_index]);
	right_line[1] = line_point_2;
	//cvLine(pic, cvPointFrom32f(right_line[0]), cvPointFrom32f(right_line[1]), red, 2, 8);

	printf("coord0 is %f,%f\n", right_line[0].x, right_line[0].y);
	printf("coord1 is %f,%f\n", right_line[1].x, right_line[1].y);
	printf("LinePoint2 is %f\n", sec_coord_y);
	printf("right_count is %d\n", right_count);
	printf("count is %d\n", count);
	printf("right_count pts are \n");
	int i = 0;
	for (i; i < count; i++) {
		printf("perp: (%d,%d)\n", pix_perpendicular[i].x,
				pix_perpendicular[i].y);
	}
	for (i = 0; i < right_count; i++) {
		printf("pt: (%f,%f)\n", pts_right[i].x, pts_right[i].y);
	}

	angle = 0;
	for (; angle <= 180; angle += 3) {
		angle_rad = ((CV_PI) / 180) * angle;
		for (k = in_rad + 1; k < out_rad; k += 1) {
			circum_point.x = img_center.x + cos(angle_rad) * k;
			circum_point.y = img_center.y + sin(angle_rad) * k;
			//printf("Cx,Cy : %d %d\n",circum_point.x,circum_point.y);
			pixcolor = cvGet2D(pic, circum_point.y, circum_point.x);
			CvPoint interestedPoint = cvPoint(circum_point.x, circum_point.y);
			if (match(pixcolor, whitecolor, 1)) {
				//printf("b,g,r for angle %d is %f %f %f \n", angle,
				//		pixcolor.val[0], pixcolor.val[1], pixcolor.val[2]);
				//printf("White Detected .. \n");
				// For case 2 ---> right ray
				if ((angle >= 45) && (angle < 135)) {
					if (angle == 90) {
						bottom_perp_index = count;
						pix_perpendicular[count] = interestedPoint;
						count++;
					} else {
						pts_bottom[bottom_count++] = cvPointTo32f(
								interestedPoint);
					}
					//cvCircle(pic, interestedPoint, 3, cvScalar(255, 0, 0), 1);
				} else {
					//cvCircle(pic, interestedPoint, 3, cvScalar(255, 255, 255),
					//		1);
				}
				break;
			}
		}
	}

	// Get the 2nd coordinate in pixel space on the right line.
	// We already have 1, i.e the perperndicular line segment.
	// hypotenuse is the point from the camera center to another point(in our case last point) on the curvature.
	CvPoint2D32f hypotenuse2[2] = { cvPointTo32f(img_center),
			pts_bottom[bottom_count - 1] };
	// We want to extend linearly, so first find this coordinate's x
	double sec_coord_x = getXFromLine(hypotenuse2,
			(double) pix_perpendicular[bottom_perp_index].y);
	CvPoint2D32f line_point_2_2 = cvPoint2D32f(sec_coord_x,
			(double) pix_perpendicular[bottom_perp_index].y);
	bottom_line[0] = cvPointTo32f(pix_perpendicular[bottom_perp_index]);
	bottom_line[1] = line_point_2_2;
	//cvLine(pic, cvPointFrom32f(pts_bottom[bottom_count - 1]), cvPointFrom32f(
	//		pts_bottom[bottom_count - 1]), red, 2, 8);
	//cvLine(pic, cvPointFrom32f(bottom_line[0]), cvPointFrom32f(bottom_line[1]),
	//		red, 2, 8);

	printf("coord0 is %f,%f\n", bottom_line[0].x, bottom_line[0].y);
	printf("coord1 is %f,%f\n", bottom_line[1].x, bottom_line[1].y);
	printf("sec_coord_x is %f\n", sec_coord_x);
	printf("bottom_count is %d\n", bottom_count);
	printf("pix_perpendicular[1].y %d\n", pix_perpendicular[1].y);
	printf("count is %d\n", count);
	printf("right_count pts are \n");
	i = 0;
	for (i; i < count; i++) {
		printf("perp: (%d,%d)\n", pix_perpendicular[i].x,
				pix_perpendicular[i].y);
	}
	for (i = 0; i < right_count; i++) {
		printf("pt: (%f,%f)\n", pts_right[i].x, pts_right[i].y);
	}

	// We want to isolate the 4 different corners, so we shoot rays
	// at certain line combinations.
	angle = 0;
	for (; angle <= 360; angle += 3) {
		angle_rad = ((CV_PI) / 180) * angle;
		for (k = out_rad; k > in_rad + 1; k -= 1) {
			circum_point.x = img_center.x + cos(angle_rad) * k;
			circum_point.y = img_center.y + sin(angle_rad) * k;
			//printf("Cx,Cy : %d %d\n",circum_point.x,circum_point.y);
			pixcolor = cvGet2D(pic, circum_point.y, circum_point.x);
			CvPoint interestedPoint = cvPoint(circum_point.x, circum_point.y);
			if (match(pixcolor, whitecolor, 1)) {
				//printf("b,g,r for angle %d is %f %f %f \n", angle,
				//		pixcolor.val[0], pixcolor.val[1], pixcolor.val[2]);
				//printf("White Detected .. \n");
				// For case 1 ---> left ray
				cvCircle(pic, interestedPoint, 3, cvScalar(255, 0, 0), 1);
			}
		}
	}

	// Get the 2nd coordinate in pixel space on the right line.
	// We already have 1, i.e the perperndicular line segment.
	// hypotenuse is the point from the camera center to another point(in our case last point) on the curvature.
	CvPoint2D32f hypotenuse_3[2] = { cvPointTo32f(img_center),
			pts_left[left_count - 1] };
	// We want to extend linearly, so first find this coordinate's y
	sec_coord_y = getYFromLine(hypotenuse_3,
			pix_perpendicular[left_perp_index].x);
	CvPoint2D32f line_point_3 = cvPoint2D32f(
			(double) pix_perpendicular[left_perp_index].x, sec_coord_y);
	left_line[0] = cvPointTo32f(pix_perpendicular[left_perp_index]);
	left_line[1] = line_point_3;
	//cvLine(pic, cvPointFrom32f(left_line[0]), cvPointFrom32f(left_line[1]), red, 2, 8);

	printf("coord0 is %f,%f\n", left_line[0].x, left_line[0].y);
	printf("coord1 is %f,%f\n", left_line[1].x, left_line[1].y);
	printf("LinePoint2 is %f\n", sec_coord_y);
	printf("left_count is %d\n", left_count);
	printf("count is %d\n", count);
	printf("left_count pts are \n");
	i = 0;
	for (i; i < count; i++) {
		printf("perp: (%d,%d)\n", pix_perpendicular[i].x,
				pix_perpendicular[i].y);
	}
	for (i = 0; i < left_count; i++) {
		printf("pt: (%f,%f)\n", pts_left[i].x, pts_left[i].y);
	}

}

bool pointInsideCircle(CvPoint pt, CvPoint center, double radius) {
	double distance = getDistance(pt, center);
	if (distance > radius) {
		return false;
	}
	return true;
}

void occlude(IplImage* src, CvPoint cam_center, double radius) {
	// draw the circle on the screen
	cvCircle(src, cam_center, radius, cvScalar(0, 0, 0), 4);
	int height = 0, width = 0;
	if (src) {
		height = src->height;
		width = src->width;
	}
	for (int i = 0; i < width; i++) {
		for (int k = 0; k < height; k++) {
			CvScalar black = CV_RGB(0,0,0);
			CvScalar pix = cvGet2D(src, k, i);
			// For each pixel, set it to black if it's to the right of the line.
			if (!pointInsideCircle(cvPoint(i, k), cvPoint(300, 250), 220.0)) {
				cvSet2D(src, k, i, black);
			}
		}
	}
}

void Neerajtestpoints(IplImage*pic, int in_rad, int out_rad,
		CvPoint img_center, bool top_flag, CvPoint2D32f *world) {
	double angle = -90.0, angle_rad = 0;
	CvPoint circum_point;
	CvScalar pixcolor;
	CvScalar whitecolor = cvScalar(k_WHITEB, k_WHITEG, k_WHITER);
	CvScalar fieldcolor = cvScalar(k_FIELDB, k_FIELDG, k_FIELDR);
	cvCircle(pic, img_center, in_rad, cvScalar(255, 0, 255), 3);

	// Collect the intersection point
	CvPoint2D32f pts_right[40];
	CvPoint2D32f line_right[2];
	CvPoint2D32f pts_left[40];
	CvPoint2D32f line_left[2];
	CvPoint2D32f pts_bottom[40];
	CvPoint2D32f line_bottom[2];
	CvPoint2D32f pts_top[40];
	CvPoint2D32f line_top[2];
	CvPoint2D32f top_left;
	CvPoint2D32f top_right;
	CvPoint2D32f bottom_left;
	CvPoint2D32f bottom_right;
	CvPoint2D32f side_right_angle;
	CvPoint2D32f side_left_angle;
	CvPoint2D32f side_bottom_angle;
	CvPoint2D32f side_top_angle;
	int right_count = 0, left_count = 0, bottom_count = 0, top_count = 0,
			count = 0;

	//cvCircle(pic, img_center, out_rad, cvScalar(255, 0, 255), 3);
	int k = 0, flag_white = 0;
	for (; angle <= 274.0; angle += 1.0) {
		angle_rad = ((CV_PI) / 180) * angle;
		for (k = out_rad; k > in_rad + 1; k -= 1) {
			circum_point.x = img_center.x + cos(angle_rad) * k;
			circum_point.y = img_center.y + sin(angle_rad) * k;
			//if(circum_point.x == 290 && circum_point.y == 77){
			//	printf("Hit this \n");
			//}
			//printf("Cx,Cy : %d %d\n",circum_point.x,circum_point.y);
			pixcolor = cvGet2D(pic, circum_point.y, circum_point.x);
			CvPoint interestedPoint = cvPoint(circum_point.x, circum_point.y);
			if (!flag_white) {
				if (match(pixcolor, fieldcolor, 0)) {
					flag_white = 1;
				}
			} else if (flag_white) {
				if (match(pixcolor, whitecolor, 1)) {
					//printf("b,g,r for angle %d is %f %f %f \n", angle,
					//		pixcolor.val[0], pixcolor.val[1], pixcolor.val[2]);
					//printf("White Detected .. \n");
					flag_white = 0;
					// For case 1 ---> left ray
					cvCircle(pic, interestedPoint, 3, cvScalar(255, 0, 0), 1);

					// Make youre arrays
					if ((angle > 266.0) && (angle <= 274)) {
						pts_top[top_count++] = cvPointTo32f(interestedPoint);
						printf("WE ARE HERE\n");
					} else if (-4.0 < angle && angle < 4.0) {
						pts_right[right_count++]
								= cvPointTo32f(interestedPoint);
					} else if (86 < angle && angle < 94) {
						pts_bottom[bottom_count++] = cvPointTo32f(
								interestedPoint);
					} else if (176 < angle && angle < 184) {
						pts_left[left_count++] = cvPointTo32f(interestedPoint);
					}
					break;
				}
			} else {
				printf("Bullshit\n");
			}
		}
		if (!((int) angle % 91)) {
			cvCircle(pic, circum_point, 4, cvScalar(0, 0, 0), 3);
			if (angle == 0) {
				side_right_angle = cvPointTo32f(circum_point);
			} else if (angle == 91) {
				side_bottom_angle = cvPointTo32f(circum_point);
			} else if (angle == 182) {
				side_left_angle = cvPointTo32f(circum_point);
			} else if (angle == 273) {
				side_top_angle = cvPointTo32f(circum_point);
			}
		}
	}

	// Get Lines.

	// TOP

	// Get the 2nd coordinate in pixel space on the right line.
	// We already have 1, i.e the perperndicular line segment.
	// hypotenuse is the point from the camera center to another point(in our case last point) on the curvature.
	CvPoint2D32f hypotenuse_top[2] = { cvPointTo32f(img_center),
			pts_top[top_count - 1] };
	cvCircle(pic, cvPointFrom32f(pts_top[top_count - 1]), 2, cvScalar(0, 255,
			255), 3);
	// Take the middle one ..
	CvPoint2D32f pix_perpendicular_top(pts_top[count / 2]);
	//CvPoint2D32f pix_perpendicular_top(side_top_angle);

	cvCircle(pic, cvPointFrom32f(pix_perpendicular_top), 2, cvScalar(0, 255,
			255), 3);
	// We want to extend linearly, so first find this coordinate's y
	double sec_coord_x = getXFromLine(hypotenuse_top, pix_perpendicular_top.y);
	// This is the 2nd point on the line . The first one is the perpendicular point.
	CvPoint2D32f line_point_2 = cvPoint2D32f(sec_coord_x,
			(double) pix_perpendicular_top.y);
	line_top[0] = pix_perpendicular_top;
	line_top[1] = line_point_2;
	cvLine(pic, cvPointFrom32f(line_top[0]), cvPointFrom32f(line_top[1]),
			cvScalar(0, 0, 255), 2, 8);

	printf("coord0 is %f,%f\n", line_top[0].x, line_top[0].y);
	printf("coord1 is %f,%f\n", line_top[1].x, line_top[1].y);
	printf("LinePoint2 is %f\n", sec_coord_x);
	printf("top_count is %d\n", top_count);
	printf("count is %d\n", count);
	printf("top_count pts are \n");
	int i = 0;
	printf("perp: (%f,%f)\n", pix_perpendicular_top.x, pix_perpendicular_top.y);
	printf("Dist_top: (%f)\n", getDistance(
			cvPointFrom32f(pix_perpendicular_top), img_center));
	for (i = 0; i < top_count; i++) {
		printf("pt: (%f,%f)\n", pts_top[i].x, pts_top[i].y);
	}

	// Bottom

	CvPoint2D32f hypotenuse_bottom[2] = { cvPointTo32f(img_center),
			pts_bottom[bottom_count - 1] };
	cvCircle(pic, cvPointFrom32f(pts_bottom[bottom_count - 1]), 2, cvScalar(0,
			255, 255), 3);
	// Take the middle one ..
	CvPoint2D32f pix_perpendicular_bottom(pts_bottom[count / 2]);
	//CvPoint2D32f pix_perpendicular_bottom(side_bottom_angle);
	cvCircle(pic, cvPointFrom32f(pix_perpendicular_bottom), 2, cvScalar(0, 255,
			255), 3);
	// We want to extend linearly, so first find this coordinate's y
	double sec_coord_x_bottom = getXFromLine(hypotenuse_bottom,
			pix_perpendicular_bottom.y);
	// This is the 2nd point on the line . The first one is the perpendicular point.
	CvPoint2D32f line_point_2_bottom = cvPoint2D32f(sec_coord_x_bottom,
			(double) pix_perpendicular_bottom.y);
	line_bottom[0] = pix_perpendicular_bottom;
	line_bottom[1] = line_point_2_bottom;
	cvLine(pic, cvPointFrom32f(line_bottom[0]), cvPointFrom32f(line_bottom[1]),
			cvScalar(0, 0, 255), 2, 8);
	printf("Dist_bot: (%f)\n", getDistance(cvPointFrom32f(
			pix_perpendicular_bottom), img_center));

	// Right

	CvPoint2D32f hypotenuse_right[2] = { cvPointTo32f(img_center),
			pts_right[right_count - 1] };
	cvCircle(pic, cvPointFrom32f(pts_right[right_count - 1]), 2, cvScalar(0,
			255, 255), 3);
	// Take the middle one ..
	CvPoint2D32f pix_perpendicular_right(pts_right[count / 2]);
	//CvPoint2D32f pix_perpendicular_right(side_right_angle);

	cvCircle(pic, cvPointFrom32f(pix_perpendicular_right), 2, cvScalar(0, 255,
			255), 3);
	// We want to extend linearly, so first find this coordinate's y
	double sec_coord_y_right = getYFromLine(hypotenuse_right,
			pix_perpendicular_right.x);
	// This is the 2nd point on the line . The first one is the perpendicular point.
	CvPoint2D32f line_point_2_right = cvPoint2D32f(
			(double) pix_perpendicular_right.x, sec_coord_y_right);
	line_right[0] = pix_perpendicular_right;
	line_right[1] = line_point_2_right;
	cvLine(pic, cvPointFrom32f(line_right[0]), cvPointFrom32f(line_right[1]),
			cvScalar(0, 0, 255), 2, 8);
	printf("Dist_right: (%f)\n", getDistance(cvPointFrom32f(
			pix_perpendicular_right), img_center));

	// Left

	CvPoint2D32f hypotenuse_left[2] = { cvPointTo32f(img_center),
			pts_left[left_count - 1] };
	cvCircle(pic, cvPointFrom32f(pts_left[left_count - 1]), 2, cvScalar(0, 255,
			255), 3);
	// Take the middle one ..
	CvPoint2D32f pix_perpendicular_left(pts_left[count / 2]);
	//CvPoint2D32f pix_perpendicular_left(side_left_angle);
	cvCircle(pic, cvPointFrom32f(pix_perpendicular_left), 2, cvScalar(0, 255,
			255), 3);
	// We want to extend linearly, so first find this coordinate's y
	double sec_coord_y_left = getYFromLine(hypotenuse_left,
			pix_perpendicular_left.x);
	// This is the 2nd point on the line . The first one is the perpendicular point.
	CvPoint2D32f line_point_2_left = cvPoint2D32f(
			(double) pix_perpendicular_left.x, sec_coord_y_left);
	line_left[0] = pix_perpendicular_left;
	line_left[1] = line_point_2_left;
	cvLine(pic, cvPointFrom32f(line_left[0]), cvPointFrom32f(line_left[1]),
			cvScalar(0, 0, 255), 2, 8);
	printf("Dist_left: (%f)\n", getDistance(cvPointFrom32f(
			pix_perpendicular_left), img_center));

	// Find Intersections.
	int x = getIntersection(line_top, line_right, &top_right);
	x = getIntersection(line_bottom, line_right, &bottom_right);
	x = getIntersection(line_left, line_bottom, &bottom_left);
	x = getIntersection(line_left, line_top, &top_left);
	//	printf("returned %d\n",x);
	printf("top right ( %f %f )\n", top_right.x, top_right.y);
	cvCircle(pic, cvPointFrom32f(top_right), 2, cvScalar(255, 0, 255), 3);
	cvCircle(pic, cvPointFrom32f(bottom_right), 2, cvScalar(255, 0, 255), 3);
	cvCircle(pic, cvPointFrom32f(bottom_left), 2, cvScalar(255, 0, 255), 3);
	cvCircle(pic, cvPointFrom32f(top_left), 2, cvScalar(255, 0, 255), 3);

	// Get Localization angles.
	double dist_top_left, dist_top_right, dist_bottom_right, dist_perp_top,
			dist_perp_right;
	double alpha, beta, gamma;
	dist_top_left = getDistance(img_center, cvPointFrom32f(top_left));
	dist_top_right = getDistance(img_center, cvPointFrom32f(top_right));
	dist_bottom_right = getDistance(img_center, cvPointFrom32f(bottom_right));
	dist_perp_top = getDistance(cvPointFrom32f(line_top[0]), img_center);
	dist_perp_right = getDistance(cvPointFrom32f(line_right[0]), img_center);
	alpha = asin(dist_perp_top / dist_top_left);
	beta = asin(dist_perp_top / dist_top_right);
	gamma = asin(dist_perp_right / dist_bottom_right);
	printf("alpha %f, beta %f, gamma %f\n", alpha * (180 / CV_PI), beta * (180
			/ CV_PI), gamma * (180 / CV_PI));
	double ratio = ((sin(beta) / tan(alpha) + cos(beta)) / (cos(90 - beta)
			+ sin(90 - beta / tan(gamma))));
	printf("Ratio is %f\n", ratio);
	double common_side_world = 0.0;
	double dx, dy, world_x, world_y;
	if (ratio > 1.0) {
		common_side_world = 10 / (cos(beta) + sin(beta) / tan(alpha));
	} else {
		common_side_world = 8 / (cos(beta) + sin(beta) / tan(alpha));
	}
	dx = common_side_world * cos(beta);
	dy = common_side_world * sin(beta);
	if (top_flag) {
		world_x = 10 - dx;
		world_y = 16 - dy;
	} else {
		world_x = 0 + dx;
		world_y = 0 + dy;
	}
	printf("dx,dy (%f,%f)\n", dx, dy);
	printf("common side world,pixel (%f m, %f pix) \n", common_side_world,
			dist_top_right);
	printf("===\n");
	printf("  !!!!! <<<< world_x,world_y (%f,%f) >>>  !!!!! \n", world_x,
			world_y);
	printf("===\n");
	CvFont font;
	cvInitFont(&font, CV_FONT_VECTOR0, 0.4, 0.4, 0, 1);
	char text[40];
	sprintf(text, "(alpha,beta,gamma: %f,%f %f)", alpha * (180 / CV_PI), beta
			* (180 / CV_PI), gamma * (180 / CV_PI));
	cvPutText(pic, text, cvPoint(0, 460), &font, cvScalar(255, 255, 255));
	world->x = world_x;
	world->y = world_y;
}

void shootRayAtAngle(IplImage * src, CvPoint *start_point, double angle,
		CvPoint *perp_point) {
	CvPoint circum_point;
	int in_rad = 60;
	int out_rad = (src->width / 2 > src->height / 2 ? src->height / 2
			: src->width / 2);
	int k = 1, inc = 0;
	double angle_rad = 0;
	CvScalar whitecolor = CV_RGB(255,255,255);
	CvScalar fieldcolor = CV_RGB(k_BALLR,k_BALLG,k_BALLB);
	CvScalar blackcolor = CV_RGB(0,0,0);
	CvScalar pixcolor;

	angle_rad = ((CV_PI) / 180) * angle;
	for (k = in_rad; k < out_rad; k += 1) {
		circum_point.x = start_point->x + cos(angle_rad) * k;
		circum_point.y = start_point->y + sin(angle_rad) * k;
		//printf("Cx,Cy : %d %d\n",circum_point.x,circum_point.y);
		pixcolor = cvGet2D(src, circum_point.y, circum_point.x);
		if (match(pixcolor, whitecolor, 1)) {
			//printf("b,g,r for angle %d is %f %f %f \n", angle,
			//		pixcolor.val[0], pixcolor.val[1], pixcolor.val[2]);
			//printf("White Detected .. \n");
			// Return the perpendicular point.
			perp_point->x = circum_point.x;
			perp_point->y = circum_point.y;
			double dist = getDistance(*start_point, *perp_point);
			printf("distance in pixels: %f\n", dist);
			cvCircle(src, cvPoint(circum_point.x, circum_point.y), 5, cvScalar(
					255, 0, 255), 1);
			break;
		}
	}
}

void shootRayAt90Incs(IplImage * src, CvPoint *start_point) {
	CvPoint circum_point;
	int in_rad = 50;
	int out_rad = 350;
	int k = 1, inc = 0;
	double angle = 0.0, angle_rad = 0.0;
	double bottom_dist = 0.0, right_dist = 0.0, left_dist = 0.0, top_dist = 0.0;
	CvScalar whitecolor = CV_RGB(255,255,255);
	CvScalar fieldcolor = CV_RGB(k_BALLR,k_BALLG,k_BALLB);
	CvScalar pixcolor;

	for (angle = 0.0; angle < 360.0; angle += 2.0) {
		angle_rad = ((CV_PI) / 180) * angle;
		for (k = in_rad; k < out_rad; k += 1) {
			circum_point.x = start_point->x + cos(angle_rad) * k;
			circum_point.y = start_point->y + sin(angle_rad) * k;
			if ((circum_point.x > src->width - 1) || (circum_point.x < 0)) {
				break;
			}
			if ((circum_point.y > src->height - 1) || (circum_point.y < 0)) {
				break;
			}
			pixcolor = cvGet2D(src, circum_point.y, circum_point.x);
			//printf("Cx,Cy : %d %d(%f)\n", circum_point.x, circum_point.y,angle);
			if (match(pixcolor, whitecolor, 1)) {
				if (angle == 90) {
					bottom_dist = getDistance(circum_point, *start_point);
				} else if (angle == 0) {
					right_dist = getDistance(circum_point, *start_point);
				} else if (angle == 180) {
					left_dist = getDistance(circum_point, *start_point);
				} else if (angle == 272) {
					top_dist = getDistance(circum_point, *start_point);
					//printf("CxPT,CyPT : %d %d\n", circum_point.x, circum_point.y);
				} else {
					break;
				}
				//printf("CxPT,CyPT : %d %d\n", circum_point.x, circum_point.y);
				//printf("b,g,r for angle %d is %f %f %f \n", angle,
				//		pixcolor.val[0], pixcolor.val[1], pixcolor.val[2]);
				//printf("White Detected .. \n");
				// Return the perpendicular point.
				cvCircle(src, cvPoint(circum_point.x, circum_point.y), 5,
						cvScalar(255, 0, 255), 1);
				break;
			}
		}
	}
	CvFont font;
	cvInitFont(&font, CV_FONT_VECTOR0, 0.4, 0.4, 0, 1);
	char text[40];
	sprintf(text, "(T,B: %f,%f)", top_dist, bottom_dist);
	cvPutText(src, text, cvPoint(240, 260), &font, cvScalar(255, 255, 255));
	sprintf(text, "(R,L: %f,%f)", right_dist, left_dist);
	cvPutText(src, text, cvPoint(240, 280), &font, cvScalar(255, 255, 255));
}

double getWorldDistance(double pix_dist) {
	return (.00017 * (pow(pix_dist, 2)) - .0073 * (pix_dist) + 0.2);
}

// Get a virtual axis ..
// Track the angle made by each frame with the virtual axis ..
// If the angle hits 0 then
// If the angle between the 2 bottom centers hits 0, then switch what to add/subtract from.

double vaxis(IplImage * pic) {
	CvPoint bot = cvPoint(310, 0);
	CvPoint top = cvPoint(310, 479);
	CvPoint right = cvPoint(0, 250);
	CvPoint left = cvPoint(640, 250);
	cvLine(pic, bot, top, cvScalar(0, 0, 255), 2, 8);
	cvLine(pic, right, left, cvScalar(0, 0, 255), 2, 8);
}

int main(int argc, char* argv[]) {
	int delay = 0, key = 0, i = 0;
	char c;
	char *window_name = NULL;
	short vidFlagOn = 0;
	int errorFlagOn = 0;
	char * fname = NULL;
	CvCapture * video = NULL;
	IplImage * pic = NULL;
	IplImage * gray;

	for (int i = 0; i < argc; i++) {

		if (!strncmp(argv[i], "-v", 2)) {
			vidFlagOn = 1;
			if (argv[i + 1] == NULL) {
				errorFlagOn = 1;
			} else {
				fname = argv[i + 1];
				i++;
			}
		} else if (!strncmp(argv[i], "-p", 2)) {
			vidFlagOn = 0;
			if (argv[i + 1] == NULL) {
				errorFlagOn = 1;
			} else {
				fname = argv[i + 1];
				i++;
			}
		}

		if (errorFlagOn) {
			usage();
			return -1;
		}
	}

	if (!vidFlagOn) {
		pic = cvLoadImage(fname);
	} else {
		video = cvCaptureFromFile(fname);
		if (!video) {
			printf("Unable to open file\n");
			return 1;
		}
		pic = cvQueryFrame(video);
		if (vidFlagOn) {
			/* Display fps and calculate delay */
			printf("Video FPS: %2.2f \n", cvGetCaptureProperty(video,
					CV_CAP_PROP_FPS));
			delay = (int) (1000 / cvGetCaptureProperty(video, CV_CAP_PROP_FPS));
		}
	}

	window_name = "VisionSimulator";
	cvNamedWindow(window_name, CV_WINDOW_AUTOSIZE);
	IplImage * processed = NULL;
	int frame_count = 0;
	CvPoint cam_center = cvPoint(CAMERA_X, CAMERA_Y);
	CvPoint init_coords[4] = { cvPoint(0, 8), cvPoint(10, 8), cvPoint(10, 16),
			cvPoint(0, 16) };
	double global_init_angles[4];
	CvScalar pixcolor;
	CvPoint2D32f world_pos;
	CvPoint perp_point;
	while (pic) {
		occlude(pic, cvPoint(300, 250), 220.0);
		/*
		 shootRayAtAngle(pic, &cam_center, 274, &perp_point);
		 CvFont font;
		 cvInitFont(&font, CV_FONT_VECTOR0, 0.4, 0.4, 0, 1);
		 char text[40];
		 sprintf(text, "(%d)", frame_count);
		 cvPutText(pic, text, cvPoint(0, 420), &font, cvScalar(255, 255, 255));
		 */
		/*
		 Neerajtestpoints(pic, 80, 220, cam_center, 1, &world_pos);
		 CvFont font;
		 cvInitFont(&font, CV_FONT_VECTOR0, 0.4, 0.4, 0, 1);
		 char text[40];
		 sprintf(text, "(%f,%f)", world_pos.x, world_pos.y);
		 cvPutText(pic, text, cvPoint(240, 260), &font, cvScalar(255, 255, 255));
		 */
		//vaxis(pic);
		detectBall(pic);
		//shootRayAt90Incs(pic, &cam_center);
		/*
		 printf(
		 "closest pixels in PSpace @ (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n",
		 pix_corners[0].x, pix_corners[0].y, pix_corners[1].x,
		 pix_corners[1].y, pix_corners[2].x, pix_corners[2].y,
		 pix_corners[3].x, pix_corners[3].y);
		 int i = 0;
		 CvPoint closest_corner;
		 double theta;
		 for (i = 0; i < 4; i++) {
		 findClosestCorner(&cam_center, pix_corners, &closest_corner);
		 theta = getTheta(cam_center, closest_corner,
		 pix_perpendicular[i], pic);
		 printf("closest corner @ pixel space (%d %d)\n",
		 closest_corner.x, closest_corner.y);
		 printf("theta for pix_perp[%d] is %f\n", i, theta);
		 }
		 }

		 IplImage *tpl = cvLoadImage("Videos/newpoint.jpg", 1);
		 if(tpl == NULL)
		 printf("Img is null\n");
		 CvRect rect = cvRect(0, 0, 640, 480);
		 cvSetImageROI(pic, rect);
		 printf("Circle Width is %d, %d",tpl->width,tpl->height);
		 IplImage *res = cvCreateImage(cvSize(rect.width - tpl->width + 1,rect.height - tpl->height + 1), IPL_DEPTH_32F, 1);
		 cvMatchTemplate(pic, tpl, res, CV_TM_SQDIFF);
		 // find best matches location
		 CvPoint minloc, maxloc;
		 printf("got here\n");
		 double minval, maxval;
		 cvMinMaxLoc(res, &minval, &maxval, &minloc, &maxloc, 0);
		 printf("got here\n");
		 cvRectangle(pic,cvPoint(minloc.x, minloc.y),cvPoint(minloc.x + tpl->width, minloc.y + tpl->height),CV_RGB(255, 0, 0), 1, 0, 0 );
		 cvResetImageROI(pic);
		 printf("got here\n");
		 */

		//IplImage * grey = cvCreateImage(cvGetSize(pic), IPL_DEPTH_8U, 1);
		//cvCvtColor(pic, grey, CV_BGR2GRAY);
		//getHoughBall(grey);
		/*
		 occludeView(pic, cvPoint(330, 0), cvPoint(330, 480));
		 int i = 0, count = 0;
		 CvPoint arr[300][2], perp_point[1];
		 double pixandworlddist[300][2] ;
		 count = drawRadialFromPoint(pic, 150, 220, cvPoint(CAMERA_X, CAMERA_Y),
		 arr, perp_point);
		 printf("Perp. Point (%d,%d) \n", perp_point[0].x, perp_point[0].y);
		 printf("%d Points Detected \n", count);
		 for (i = 0; i < count; i++) {
		 printf(" x,y: %d , %d \n", arr[i][0].x, arr[i][0].y);
		 }
		 getDRdata(pic,arr,perp_point,count) ;

		 // For all x's to the right of the main perp point , get
		 // thetas and add them to the global array.
		 double dist_inc = 0, theta = 0;
		 for (i = 0; i < count; i++) {
		 theta = getTheta(cvPoint(CAMERA_X, CAMERA_Y), arr[i][0],
		 perp_point[0], pic);
		 // Take stuff only less than 45 degrees ..
		 if(theta <= 0.7853){
		 dist_inc = (tan(theta) * 5);
		 printf("dist_inc initial %f\n", dist_inc);
		 if (arr[i][0].y < perp_point[0].y) {
		 printf("1 %d %d\n", arr[i][0].y, perp_point[0].y);
		 dist_inc = (double)DIST_LEFT + dist_inc ;
		 } else if (arr[i][0].y > perp_point[0].y) {
		 dist_inc = (double)DIST_LEFT - dist_inc ;
		 } else
		 dist_inc = (double)DIST_LEFT;
		 printf("dist_inc after modification %f\n", dist_inc);
		 int dist = (int) round(dist_inc);
		 CvPoint world_point = cvPoint(0, dist);
		 printf("World Coordinate is( %d %d)\n", world_point.x,
		 world_point.y);
		 arr[i][1].x = world_point.x;
		 arr[i][1].y = world_point.y;
		 printf("----\n");
		 //pointOnStraightLine(dist_inc);
		 }
		 }

		 for(i=0 ; i<count ; i++){
		 pixandworlddist[i][0] = getDistance(cvPoint(CAMERA_X,CAMERA_Y),arr[i][0]);
		 pixandworlddist[i][1] = getDistance(cvPoint(5,8),arr[i][1]);
		 }

		 // Pixel to World ->
		 for (i = 0; i < count; i++) {
		 printf("PIXEL (%d %d) --> WORLD (%d,%d)\n",arr[i][0].x,arr[i][0].y,arr[i][1].x,arr[i][1].y);
		 printf("PIX DIST(%f) --> WORLD DIST(%f)\n",pixandworlddist[i][0],pixandworlddist[i][1]) ;
		 }
		 //processed = detectLines(pic);
		 // Detect the ball first.
		 //detectBall(processed) ;
		 // Mark Camera
		 //cvCircle(pic,cvPoint(CAMERA_X,CAMERA_Y),50,cvScalar(0,0,255),1);
		 /* show the loaded image with edges detected*/
		/*
		 CvPoint white,transition,arc_point,perp_point;
		 white = getWhitePoint(pic,50,white) ;
		 double angle = getAngleToDrawLine(pic,white),theta=0 ;
		 printf("Angle is %f\n",angle * (180/(CV_PI)));
		 //drawRadialLine(pic,angle) ;
		 transition = findTransitionPoint(pic,angle,white);
		 cvCircle(pic,transition,2,cvScalar(255,255,255),1);
		 // Calculate arcpoint ;
		 //arc_point.x = white.x + 30 ;
		 //arc_point.y = detectColorChangeY(pic,arc_point.x,white.y);

		 // Theoretically, this is just y2-y1,x2-x1
		 perp_point.x = white.x + abs(transition.y - white.y) ;
		 perp_point.y = white.y + abs(white.x - transition.x) ;
		 printf("ppx,ppy,wwx,wwy: %d %d %d %d\n",perp_point.x, perp_point.y,white.x,white.y);

		 // distance from white to first transition point
		 double dist = getDistance(white,transition) ;

		 CvPoint arr[40] ;
		 int count = drawRadialFromPoint(pic,200,white,arr);
		 printf("%d Points Detected \n",count);
		 for(i=0 ; i < count ; i++ ){
		 printf(" x,y: %d , %d \n", arr[i].x,arr[i].y) ;
		 }

		 // For all x's to the right of the main perp point , get thetas and add them to the global array.
		 double dist_inc = 0 ;
		 for(i=0 ; i < count ; i++ ){
		 if(arr[i].x > transition.x && arr[i].y > white.y){
		 theta = giveTheta(white,arr[i],transition);
		 printf("Theta is %f deg and %f radians \n",theta * (180/(CV_PI)), theta);
		 dist_inc = (tan(theta)*dist);
		 pointOnStraightLine(dist_inc) ;
		 }else{
		 printf("Ignoring ..\n") ;
		 }
		 }


		 printf("Distance is %f pixels\n", dist);
		 // Set default to t_word.
		 top[next_avail++] = 7.5 ;
		 dist_inc = (tan(theta)*dist) ;
		 pointOnStraightLine(dist_inc) ;
		 //pic = parseImage(pic);
		 *
		 */

		cvShowImage(window_name, pic);
		//cvWaitKey(0);

		// If an image, wait for an escape ..
		if (!vidFlagOn) {
			cvWaitKey(0);
			break;
		}

		/* load and check next frame */
		pic = cvQueryFrame(video);
		if (!pic) {
			printf("Error loading frame\n");
			return 1;
		}

		// Capture Frame "count"
		if (frame_count == 136) {
			//capImage("newcenter.jpg", pic);
		} else if (frame_count == 146) {
			//capImage("newcenter2.jpg", pic);
		}
		//printf("frame_count is %d\n", frame_count);

		/* wait delay and check for quit or pause('a') key */
		c = cvWaitKey(33);
		if (c == 27)
			break;
		else if (c == 'a') {
			printf("Pause .. \n");
			c = cvWaitKey(0);
			if (c == 27)
				break;
			else if (c == 'c') {
				char img[40];
				sprintf(img, "img_%d.jpg", frame_count);
				int strlen_img = strlen(img);
				img[strlen_img] = '\0';
				printf("%s\n", img);
				capImage(img, pic);
			}
		}
		frame_count++;
	}
	cvDestroyWindow(window_name);
	cvReleaseImage(&pic);
	printf("Exiting .. Bye\n");
	return 1;
}

