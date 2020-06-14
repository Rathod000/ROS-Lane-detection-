#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <chapter5_tutorials/ThresholdConfig.h>
#include <cv.h>
#include <ros/ros.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;



//declaring functions
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void GaussianBlur(const cv::Mat& src, cv::Mat& srcGauss );
void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst);
void Birdeyeview(const cv::Mat src, cv::Mat &dst3, cv::Mat &dst4 );





//Mat src, dst, color_dst, src_gray, detected_edges, GaussBlur, hide3;



  				




//Create Mask 
static void ConexPolymask(const cv::Mat& srcdst, cv::Mat& srcred, cv::Mat& srccolor )
{

			Mat srcMask;
			srcMask = cv::Mat::zeros(srcdst.rows, srcdst.cols, CV_8U);

			cv::Point pts[4] = {

			    cv::Point(550,1000),
			    cv::Point(850,700),
			    cv::Point(1100,700),
			    cv::Point(1500, 1000),
					    };

					    // (0,500) , (300,150) , (700 , 500  )
					    // (0,500),(100,200),(500,200),(700,200) 

			cv::fillConvexPoly( srcMask, pts, 4, cv::Scalar(1) );



			cv::Mat srcMask1(srcdst.size(), CV_8UC3, cv::Scalar(0));


		if (srcMask.size == srcdst.size)
		{
	
			cout << "Both Image size is similar " << endl;
				
			//Now you can copy your source image to destination image with masking
			srcdst.copyTo(srcMask1,srcMask);

		}
		else
		{
			
			cout << "Both Image size is not similar " << endl;
		}


		
			cout << "Hough transform " << endl;	
			//Mat srccolor;
			
			cvtColor( srcMask1, srccolor, COLOR_GRAY2BGR );

			vector<Vec4i> lines;
			HoughLinesP( srcMask1, lines, 1, CV_PI/180, 80, 50, 500 );

			for( size_t i = 0; i < lines.size(); i++ )
   				 {
				        line( srccolor, Point(lines[i][0], lines[i][1]),
				        Point( lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
				 }

			cv::Mat srcmask2 = cv::Mat::zeros(srccolor.rows, srccolor.cols, CV_8U);
	//		Scalar color = Scalar(0,255,0);
	

			namedWindow( "hough", 1 );
			imshow( "hough", srccolor );


//}

//static void Grad(const cv::Mat& srccolor, cv::Mat& srcred )
//{

		// You can try more different parameters
 			

 			Scharr(srccolor, srccolor, CV_8U, 1, 0, 1, 1, BORDER_ISOLATED);
			


			 //Scharr(src1, dsty, CV_8U, 0, 1, 1, 0, BORDER_ISOLATED);

			//cv::Sobel(src1, dstx, CV_8U, 1, 0, 3, 1, 0, BORDER_DEFAULT);
			//cv::Sobel(src1, dsty, CV_8U, 0, 1, 3, 1, 0, BORDER_DEFAULT);

//			namedWindow( "Horizontal", 1 );
//			imshow( "Horizontal", srccolor );

//			namedWindow( "Vertical", 1 );
//			imshow( "Vertical", srcdstx );


		


// Shading area 
/*

*/




			int x=0, y=0;
			int xi=0,yj=0;
			int count, freqMax ,freqMin; 


			for (int i=900; i>=450; i--)
				{

					xi = 0;	
			   		 for (int j=350; j<=1700; j++ )
					{
				          cv::Vec3b color = srccolor.at<Vec3b>(Point(j,i));
						
						if (color[0] < 50 && color[1] < 50 && color [2] > 150 )
								{

									if (j >= 800 )
									{
										xi++;								
									}	


									xi++;
									j = j + 10;
								


								}
   							else if ( xi == 1 )
     						{		

     								color[0] = 0;
					            	color[1] = 150;
					            	color[2] = 0;
									srccolor.at<Vec3b>(Point(j,i)) = color;


							}	

							
						
										
						}


					}

//			namedWindow("Result", 1);
//			imshow("Result", srccolor);		




}



//Function which applies the Canny Filter
static void CannyThreshold(const cv::Mat& srcGauss, cv::Mat& srcCanny )
{

			int lowThreshold = 10;
			int const max_lowThreshold = 100;
			int ratio = 3 ; //An upper threshold is needed for the Canny filter, the 				recommended ratio between upper and lower thereshold is 2:1 or 3:1
			int kernel_size = 3;//Kernel size for the Sobel operations to be performed internally by the Canny function

    
			Canny( srcGauss, srcCanny, lowThreshold , lowThreshold*ratio, kernel_size );
//			Canny( gauss, dst, 50, 200, 3 );
/*
			int scale = 1;
			int delta = 0;
			int ddepth = CV_16S;


			Sobel( srcGauss, srcCanny, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
			convertScaleAbs( srcCanny, srcCanny);
/*
		//	namedWindow("GradientX",1);			
		//	imshow ("GradientX", srcCanny);

			

			Mat srcCannyY;
			Sobel( srcGauss, srcCannyY, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
			convertScaleAbs( srcCannyY, srcCannyY);

			namedWindow("GradientY",1);			
			imshow ("GradientY", srcCannyY);
*/

//			Canny( srcCanny, srcCanny, lowThreshold , lowThreshold*ratio, kernel_size );



 }

//Main Function
int main(int argc, char** argv)

{


	// Ros node for subscribing and publishing the from the network
/*
		ros::init(argc, argv, "canny_edge_node");
		ros::NodeHandle nh_;

		image_transport::ImageTransport it_(nh_);
		image_transport::Publisher pub = it_.advertise("/canny", 1);//Filtered image publisher
		image_transport::Subscriber sub = it_.subscribe("/camera/image_raw", 1, imageCallback); //custom_cv_camera subscriber

		ros::Rate loop_rate(5);
		sensor_msgs::ImagePtr msg_;//Image message for publisher
		
			while (ros::ok())
			{
				if(!src.empty()) 
			{
*/
			



			VideoCapture cap( "/home/rathod/Videos/Video_20.mp4"); //read the video file
			if (!cap.isOpened())	// check if we succeeded
				{
					cout << "Error opening video stream" << endl;
				return -1;
				}

			for (;;)
			{


			Mat src;
			cap >> src;
			if (src.empty())
				break;

			std::cout << src.channels()<< endl;

			namedWindow("InputVideo", 1);
			imshow("InputVideo",src);

/*
////////////////////////////////////////////////////////////////////////

			//Birdeyeview

		 	 Point2f src_vertices[4];
    		src_vertices[0] = Point(850,450);
    		src_vertices[1] = Point(1050, 450);
    		src_vertices[2] = Point(1550, 900);
    		src_vertices[3] = Point(100, 900);

    		Point2f dst_vertices[4];
    		dst_vertices[0] = Point(0, 0);
    		dst_vertices[1] = Point(640, 0);
    		dst_vertices[2] = Point(640, 480);
    		dst_vertices[3] = Point(0, 480);

    		Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    		Mat dst(480, 640, CV_8UC3);
    		warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

		    Mat dst2(480, 640, CV_8UC3);
		    transform(src_vertices, dst_vertices, src, dst2);

		     imshow("src", src);
		    imshow("dst", dst);
		    imshow("dst2", dst2);
////////////////////////////////////////////////////////////////////////
*/
//		    src = dst2;

			Mat srcGauss;
			GaussianBlur( src,srcGauss );	// Apply gaussian blur filter

//			src = srcGauss ;

//	   		srcGauss = dst2; 

			Mat srcCanny;
			CannyThreshold(srcGauss, srcCanny);		

//			namedWindow( "Canny", 1 );
//			imshow("Canny", srcCanny );


			Mat srcdst = srcCanny;




	//		cout << "Canny Function performed" << endl;


			Mat srcMask,srcred, srccolor;
			ConexPolymask(srcdst,srcred,srccolor); // Create Mask for ROI
		//	cout << "Mask operations performed" << endl;


			cout << "Add weight " << endl;	
			Mat srcAddW;

			std::cout <<srccolor.channels()<< endl;
			std::cout <<src.channels()<< endl;
			std::cout <<srcAddW.channels()<< endl;
//			 imwrite("srccolor.jpg" , srccolor);
//			cvtColor( srccolor, srccolor, CV_BGR2GRAY );
//			cvtColor( srccolor, srccolor, CV_GRAY2BGR );
			//Now you can copy your source image to destination image with masking
			addWeighted(srccolor, 0.5 , src, 0.5, 0.0 , srcAddW);

			namedWindow("Video", 1);
			imshow("Video",srcAddW);


	
		
//	Mat srcred;
//	Grad(srccolor,srcred ); // Gradient
//			cout << "Gradient operations performed" << endl;
//		Mat srcred;
//	LineExtend(srcAddW,srcred);






		
/*
			msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();//OpenCV filtered image to ROS Image
			
			pub.publish(msg_);//Filtered image publication
		
*/
			
			waitKey(1);
		}
         



	  	

//		ros::spinOnce();
//		loop_rate.sleep();
//	}

	return 0;
}
/*

//Callback used to transform the incoming ROS Image into an OpenCV Image
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
 {
	cv_bridge::CvImagePtr src_; //OpenCV full Image variable
	try
	{
		src_ = cv_bridge::toCvCopy(msg, enc::BGR8); //Conversion
		src= src_->image;//Assign just the image information to a Mat variable
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
 }

*/
// GaussianBlur Filter
//static 
void GaussianBlur(const cv::Mat& src, cv::Mat& srcGauss )
{
   				
   		//	int DELAY_CAPTION = 1500;
		//	int DELAY_BLUR = 100;
			int MAX_KERNEL_LENGTH = 3;
	

			srcGauss.create( src.size(), src.type() ); //Create a matrix with raw Image size for future mask creation


			cvtColor( src, srcGauss, CV_BGR2GRAY ); //RGB to Gray Scale
	
			
 			   for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
			        { GaussianBlur( srcGauss, srcGauss,  Size( i, i ), 0, 0 ) ;}

//					namedWindow( "GaussianBlur", 1 );
//                      imshow( "GaussianBlur", srcGauss );
}


void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst)
{
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

 	


