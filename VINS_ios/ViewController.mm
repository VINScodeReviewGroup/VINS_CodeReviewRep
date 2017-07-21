//
//  ViewController.m
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#import "ViewController.h"
#import "utility.hpp"
#import "CameraUtils.h"
#import "Thumbnail.h"

#if VINS_FRAMEWORK
//#import "VINSUnityAPI.h"
/*for Unity_iPhone, choose camera mode or glass mode
 if true: camera mode, visualization use alighed image and vins result, neither is the latest result
 if false: glass mode, visulization use latest vins result and glasses real image
 */
//#define CAMERA_MODE true
#define ENABLE_IMU_PRIDICT true //only enbale in (glass mode) && (Unity_iPhone)

#else
//for VINS_ios, use camera mode and disable imu predict by default
//don't change
//#define CAMERA_MODE true
#define ENABLE_IMU_PRIDICT false
//#define DATA_EXPORT true
#endif

//wrz
//#define Width_2D_Map_m (1.42*24.5/0.7)
#define Width_2D_Map_m (2960*1.53/65.0)
#define Height_2D_Map_m ((2960*1.53/65.0)*(2500.0/2960.0))
#define Width_2D_Map_pixel 2960.0
#define Height_2D_Map_pixel 2500.0

@interface ViewController ()
@property (weak, nonatomic) IBOutlet UILabel *X_label;
@property (weak, nonatomic) IBOutlet UILabel *Y_label;
@property (weak, nonatomic) IBOutlet UILabel *Z_label;
@property (weak, nonatomic) IBOutlet UILabel *buf_label;
@property (weak, nonatomic) IBOutlet UIButton *start_button;
@property (weak, nonatomic) IBOutlet UILabel *total_odom_label;
@property (weak, nonatomic) IBOutlet UILabel *loop_label;
@property (weak, nonatomic) IBOutlet UIButton *stop_button;
@property (weak, nonatomic) IBOutlet UILabel *feature_label;
@property (weak, nonatomic) IBOutlet UILabel *feature_label2;
@property (weak, nonatomic) IBOutlet UILabel *feature_label3;
@property (weak, nonatomic) IBOutlet UISlider *fovSlider;
@property (weak, nonatomic) IBOutlet UILabel *fovLabel;

@end

@implementation ViewController

@synthesize imageView;
@synthesize featureImageView;
@synthesize startButton;
@synthesize videoCamera;

FeatureTracker featuretracker;
VINS vins;

queue<ImgConstPtr> img_msg_buf;
queue<ImuConstPtr> imu_msg_buf;
queue<Vector3f> draw_buf;
int waiting_lists = 0;  //number of measurements waiting to be processed


float time_interval = 0;
int frame_cnt = 0;

/******************************* UI CONFIG *******************************/
bool ui_main = false;  // true: UI is the main view, origin image is in left bottom
// false: origin is the main view, UI is in left bottom
bool box_in_AR = false;
bool box_in_trajectory = false;
bool CAMERA_MODE = true;
bool SHOW_TRACK = true;
bool start_show = false;
UIActivityIndicatorView *indicator;
/******************************* UI CONFIG *******************************/

UIImage *image_ui;
UIImage *image_origin;
Matrix3f RcForView;

std::mutex m_buf;
std::mutex m_time;
std::condition_variable con;

NSTimeInterval current_time = -1;
NSTimeInterval lateast_imu_time = -1;
int imu_prepare = 0;
Vector3d rotation_imu;

//for pridict
double latest_time = -1;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;

CMMotionManager *motionManager;

//for save data
bool start_record = false;
bool start_playback = false;
bool start_playback_vins = false;
unsigned long imageDataIndex = 0;
unsigned long imageDataReadIndex = 0;
unsigned long imuDataIndex = 0;
unsigned long imuDataReadIndex = 0;
unsigned long vinsDataIndex = 0;
unsigned long vinsDataReadIndex = 0;
queue<IMG_DATA> imgDataBuf;
NSMutableData *imuDataBuf = [[NSMutableData alloc] init];
NSData *imuReader;
NSMutableData *vinsDataBuf = [[NSMutableData alloc] init];
NSData *vinsReader;
IMG_DATA imgData;
IMU_MSG imuData;

//for loop closure
queue<pair<cv::Mat, double>> image_buf_loop;
std::mutex i_buf;
LoopClosure *loop_closure;
KeyFrameDatabase keyframe_database;
int process_keyframe_cnt = 0;
int miss_keyframe_num = 0;
int keyframe_freq = 0;
int global_frame_cnt = 0;
int loop_check_cnt = 0;
bool voc_init_ok = false;
int old_index = -1;
UIAlertView *alertView;
vector<int> erase_index;
Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);
Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();
int segmentation_index = 0;

//for textview
int loop_old_index = -1;
float x_view_last = -5000;
float y_view_last = -5000;
float z_view_last = -5000;
float total_odom = 0;

//缩略图上绘制点的图层
//使用方法：设置x、y坐标，更新绘制图层
//[view setPixelX:i];
//[view setPixelY:j];
//[view setNeedsDisplay];
ThumbnailView *view;

- (void)viewDidLoad {
    [super viewDidLoad];
	
	/*******************************************缩略图上绘制点的图层*******************************************/
	
	CGRect  viewRect = CGRectMake(333, 236, 156, 132);
	view = [[ThumbnailView alloc] initWithFrame:viewRect];
//	[view setBackgroundColor:[UIColor clearColor]];
	
	[view setOpaque:NO];
 
	[self.view addSubview:view];
	
    /*******************************************Camera setup*******************************************/
#if VINS_FRAMEWORK
    self.videoCamera = [[CvVideoCamera alloc] init];
    ui_main = true;
#else
    self.videoCamera = [[CvVideoCamera alloc]
                        initWithParentView:imageView];
#endif
    
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition =
    AVCaptureDevicePositionBack;
    
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    self.videoCamera.defaultAVCaptureSessionPreset =
    AVCaptureSessionPreset640x480;
#ifdef DATA_EXPORT
    self.videoCamera.defaultFPS = 1;
#else
    self.videoCamera.defaultFPS = 30;
#endif
    
    isCapturing = NO;
    
    [CameraUtils setExposureOffset: -1.0f];
    [videoCamera start];
    
    /***************************************UI configuration*****************************************/
    UIPanGestureRecognizer *resultPanGestureRecognizer = [[UIPanGestureRecognizer alloc]
                                                          initWithTarget:self
                                                          action:@selector(handlePan:)];
    resultPanGestureRecognizer.minimumNumberOfTouches = 1;
    resultPanGestureRecognizer.maximumNumberOfTouches = 2;
    [self.imageView addGestureRecognizer:resultPanGestureRecognizer];
    
    UIPinchGestureRecognizer *resultPinchGestureRecognizer = [[UIPinchGestureRecognizer alloc]
                                                              initWithTarget:self
                                                              action:@selector(handlePinch:)];
    [self.imageView addGestureRecognizer:resultPinchGestureRecognizer];
    
    UITapGestureRecognizer *resultTapGestureRecognizer = [[UITapGestureRecognizer alloc]
                                                          initWithTarget:self
                                                          action:@selector(handleTap:)];
    [self.imageView addGestureRecognizer:resultTapGestureRecognizer];
    
    UILongPressGestureRecognizer *resultLongPressGestureRecognizer = [[UILongPressGestureRecognizer alloc]
                                                                      initWithTarget:self
                                                                      action:@selector(handleLongPress:)];
    [self.imageView addGestureRecognizer:resultLongPressGestureRecognizer];
    
    if (!feature_tracker)
        feature_tracker = new FeatureTracker();
    
    //give projection variance
    vins.setIMUModel();
    RcForView = MatrixXf::Identity(3,3);
    
    //UI
    startButton.layer.zPosition = 1;
    _recordButton.layer.zPosition = 1;
    _playbackButton.layer.zPosition = 1;
    startButton.enabled = YES;
    _stopButton.enabled = NO;
    alertView = [[UIAlertView alloc]initWithTitle:@"WARN" message:@"please wait for vocabulary loading!"
                                         delegate:self cancelButtonTitle:@"confirm" otherButtonTitles:@"cancel", nil];
    
    indicator = [[UIActivityIndicatorView alloc] initWithActivityIndicatorStyle:UIActivityIndicatorViewStyleWhiteLarge];
    indicator.center = CGPointMake(self.imageView.frame.size.width * 0.5, self.imageView.frame.size.height * 0.22);
    indicator.color = [UIColor darkGrayColor];
    [indicator startAnimating];
    [self.view addSubview:indicator];
    
    /****************************************Init all the thread****************************************/
    _condition=[[NSCondition alloc] init];
    mainLoop=[[NSThread alloc]initWithTarget:self selector:@selector(run) object:nil];
    [mainLoop setName:@"mainLoop"];
    
    saveData=[[NSThread alloc]initWithTarget:self selector:@selector(saveData) object:nil];
    [saveData setName:@"saveData"];
    
    if(LOOP_CLOSURE)
    {
        //loop closure thread
        loop_thread = [[NSThread alloc]initWithTarget:self selector:@selector(loop_thread) object:nil];
        [loop_thread setName:@"loop_thread"];
        [loop_thread start];
        
        globalLoopThread=[[NSThread alloc]initWithTarget:self selector:@selector(globalLoopThread) object:nil];
        [globalLoopThread setName:@"globalLoopThread"];
        [globalLoopThread start];
    }
    
    /************************************Device and iOS version check************************************/
    bool deviceCheck = setGlobalParam(deviceName());
    if(!deviceCheck)
    {
        UIAlertController *alertDevice = [UIAlertController alertControllerWithTitle:@"Error"
                                                                             message:@"Unsupported Device!" preferredStyle:UIAlertControllerStyleAlert];
        UIAlertAction *cancelAction = [UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:^(UIAlertAction * action)
                                       {exit(0);}];
        UIAlertAction *okAction = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action)
                                   {exit(0);}];
        [alertDevice addAction:cancelAction];
        [alertDevice addAction:okAction];
        dispatch_async(dispatch_get_main_queue(), ^ {
            [self presentViewController:alertDevice animated:YES completion:nil];
        });
    }
    vins.setExtrinsic();
    vins.setIMUModel();
    bool versionCheck = iosVersion();
    if(!versionCheck)
    {
        UIAlertController *alertVersion = [UIAlertController alertControllerWithTitle:@"Warn"
                                                                              message:@"Please upgrade your iOS version!" preferredStyle:UIAlertControllerStyleAlert];
        UIAlertAction *cancelAction = [UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:^(UIAlertAction * action)
                                       {exit(0);}];
        UIAlertAction *okAction = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:^(UIAlertAction * action)
                                   {exit(0);}];
        [alertVersion addAction:cancelAction];
        [alertVersion addAction:okAction];
        dispatch_async(dispatch_get_main_queue(), ^ {
            [self presentViewController:alertVersion animated:YES completion:nil];
        });
    }
    
#if !VINS_FRAMEWORK
    [self.fovLabel removeFromSuperview];
    [self.fovSlider removeFromSuperview];
#endif
    
    /*********************************************Start VINS*******************************************/
    if(versionCheck && deviceCheck)
    {
        [self imuStartUpdate];
        isCapturing = YES;
        [mainLoop start];
        motionManager = [[CMMotionManager alloc] init];
        frameSize = cv::Size(videoCamera.imageWidth,
                             videoCamera.imageHeight);
    }
    
}


- (const NSString*)httpAsynchronousRequest:(UIImage *) photo
{
 
 NSURL *url = [NSURL URLWithString:@"http://10.84.137.135:8090"];
 
 NSData *data = UIImageJPEGRepresentation(photo, 1.0);
 
 //进行base64编码
 NSString *pictureDataString=[data base64EncodedStringWithOptions:0];
 
	
 
 //进行URLEncode 对特殊符号进行编码，防止url对特殊符号的替换
 NSString *encodedString = (NSString *)CFBridgingRelease(CFURLCreateStringByAddingPercentEscapes(
                         NULL,
                         (CFStringRef)pictureDataString,
                         NULL,
                         (CFStringRef)@"!*'();:@&=+$,/?%#[]",
                         kCFStringEncodingUTF8 ));
 
 //需要传输的字符串
 NSString *post = [NSString stringWithFormat:@"image64=%@&model=%@", encodedString, @"Didi_T1_F4"];
 
 //http模板
 NSData *postData = [post dataUsingEncoding:NSASCIIStringEncoding allowLossyConversion:YES];
 
 NSMutableURLRequest *request = [NSMutableURLRequest requestWithURL:url];
 [request setHTTPMethod:@"POST"];
 [request setHTTPBody:postData];
 [request setTimeoutInterval:10.0];
 [request setValue:@"application/x-www-form-urlencoded" forHTTPHeaderField:@"Content-Type"];
 
 static NSString* resultStr=[[NSString alloc]init];

 NSOperationQueue *queue = [[NSOperationQueue alloc]init];
 [NSURLConnection sendAsynchronousRequest:request
									queue:queue
						completionHandler:^(NSURLResponse *response, NSData *data, NSError *error){
														if (error) {
								NSLog(@"Httperror:%@%d", error.localizedDescription,error.code);
							}else{
								
								NSInteger responseCode = [(NSHTTPURLResponse *)response statusCode];
								
								NSString* responseString = [[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding];
								resultStr=responseString;
								
								NSLog(@"HttpResponseCode:%d", responseCode);
								NSLog(@"HttpResponseBody %@",responseString);
							}
							
							
						}];
	return resultStr;
 
}

void dispPosIn2Dmap(){
	NSLog(@"display current position in 2D map");
	if(vins.hasInitialP0){
		Vector2f curPosInVins(lateast_P.x(),lateast_P.y());
		Vector2f curPosIn2Dmap_m=vins.Rwc_vinsTo2Dmap*curPosInVins+vins.wTcw_vinsTo2Dmap;
		Vector2i curPosIn2Dmap_pixel;
		curPosIn2Dmap_pixel.x()=int(curPosIn2Dmap_m.x()/Width_2D_Map_m*Width_2D_Map_pixel);
		curPosIn2Dmap_pixel.y()=int(curPosIn2Dmap_m.y()/Height_2D_Map_m*Height_2D_Map_pixel);
		bool isIn2Dmap=true;
		if(curPosIn2Dmap_pixel.x()>Width_2D_Map_pixel||curPosIn2Dmap_pixel.y()>Height_2D_Map_pixel)
			isIn2Dmap=false;
		if(isIn2Dmap){
			vins.curPosIn2Dmap_m=curPosIn2Dmap_m;
			vins.curPosIn2Dmap_pixel=curPosIn2Dmap_pixel;
		}
		printf("wrz05 x0:%f, x1:%f, y0:%f, y1:%f, t0:%f, t1:%f, px:%f, py:%f\n",vins.Rwc_vinsTo2Dmap(0,0),vins.Rwc_vinsTo2Dmap(1,0),vins.Rwc_vinsTo2Dmap(0,1),vins.Rwc_vinsTo2Dmap(1,1),vins.wTcw_vinsTo2Dmap.x(),vins.wTcw_vinsTo2Dmap.y(),curPosInVins.x(),curPosInVins.y());

	}
	else{
		printf("wrz05 has no initP0\n");
	}
}

void setVinsPath(){
	NSLog(@"set vins path");
	if(!vins.drawresult.hasSetPath and vins.drawresult.hasInitialPlane and vins.drawresult.tapFlag){
		
		vins.vinsDestPath.push_back(lateast_P);
		if(vins.vinsDestPath.size()>=4){
			vins.drawresult.hasSetPath=true;
			vins.drawresult.pathDestNum=vins.vinsDestPath.size();
		}
		vins.drawresult.tapFlag=false;
	}
}


/*
 Main process image thread: this thread detects and track feature between two continuous images
 and takes the newest VINS result and the corresponding image to draw AR and trajectory.
 */
queue<IMG_DATA_CACHE> image_pool;
queue<VINS_DATA_CACHE> vins_pool;
IMG_DATA_CACHE image_data_cache;
cv::Mat lateast_equa;
UIImage *lateast_image;
Vector3f lateast_P;
Matrix3f lateast_R;
bool vins_updated = false;

//主要负责对采集到的原始图像进行角点检测跟踪，显示等
- (void)processImage:(cv::Mat&)image
{
    if(isCapturing == YES)
    {
		
		
		//NSLog(@"image processing");
        float lowPart = image.at<float>(0,0);  //modify opencv library, timestamp was stored at index 0,0
        float highPart = image.at<float>(0,1);
        //image.at<float>(0,0) = image.at<float>(1,0);
        //image.at<float>(0,1) = image.at<float>(1,1);
        shared_ptr<IMG_MSG> img_msg(new IMG_MSG());
        //cout << (videoCamera->grayscaleMode) << endl;
        //img_msg->header = [[NSDate date] timeIntervalSince1970];
        img_msg->header = [[NSProcessInfo processInfo] systemUptime];
        float Group[2];
        Group[0] = lowPart;
        Group[1] = highPart;
        double* time_now_decode = (double*)Group;
        double time_stamp = *time_now_decode;
		//printf("time_Stamp:%f, lowPart:%f, highPart:%f, group0:%f,\n",time_stamp,lowPart,highPart);
		
		//如果是第一帧，不用处理
        if(lateast_imu_time <= 0)
        {
            cv::cvtColor(image, image, CV_BGRA2RGB);
            cv::flip(image,image,-1);
            return;
        }
        //img_msg->header = lateast_imu_time;
        img_msg->header = time_stamp;
        BOOL isNeedRotation = image.size() != frameSize;
        
        //for save data
        cv::Mat input_frame;
		//回放存储的图像数据
        if(start_playback)
        {
            //TS(readImg);
            bool still_play;
            still_play = [self readImageTime:imageDataReadIndex];
            [self readImage:imageDataReadIndex];
            if(!still_play)
                return;
            imageDataReadIndex++;
#ifdef DATA_EXPORT
            [self tapSaveImageToIphone:imgData.image];
#endif
            UIImageToMat(imgData.image,image);
            UIImageToMat(imgData.image,input_frame);
            img_msg->header = imgData.header;
            //TE(readImg);
#ifdef DATA_EXPORT
            printf("record play image: %lf\n",imgData.header,imageDataReadIndex);
#endif
        }
        else
        {
            input_frame = image;
        }
        //记录图像数据
        if(start_record)
        {
            imgData.header = img_msg->header;
            imgData.image = MatToUIImage(image);
            imgDataBuf.push(imgData);
            return;
        }
        else
        {
            if(!imgDataBuf.empty())
                return;
        }
        
        prevTime = mach_absolute_time();
		
		//对图像先进行直方图均值化，再进行角点检测并跟踪
        cv::Mat gray;
        cv::cvtColor(input_frame, gray, CV_RGBA2GRAY);
        cv::Mat img_with_feature;
        cv::Mat img_equa;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(3);
        clahe->apply(gray, img_equa);
        //img_equa = gray;
		gray.copyTo(vins.imageGray);
		
        TS(time_feature);
        
        vector<Point2f> good_pts;
        vector<double> track_len;
		//提取角点，光流法跟踪匹配
        featuretracker.readImage(img_equa, img_with_feature,frame_cnt, good_pts, track_len);
        TE(time_feature);
#if VINS_FRAMEWORK
#else
        //cvtColor(img_equa, img_equa, CV_GRAY2BGR);
		//显示角点
        if(SHOW_TRACK)
        {
			for (int i = 0; i < good_pts.size(); i++)
            {
                cv::circle(image, good_pts[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
            }
        }
#endif
        //image msg buf
        int is_calculate = false;
        if(featuretracker.img_cnt==0)
        {
			//每张图像的local点云（深度为单位1），存在img_msg_buf
			img_msg->point_clouds = featuretracker.image_msg;
            //img_msg callback
            m_buf.lock();
            img_msg_buf.push(img_msg);
            is_calculate = true;
            //NSLog(@"Img timestamp %lf",img_msg_buf.front()->header);
            m_buf.unlock();
            con.notify_one();
            if(CAMERA_MODE)
            {
				//当前帧的缓存
				image_data_cache.header = img_msg->header;
#if VINS_FRAMEWORK
                image_data_cache.image = MatToUIImage(image);
#else
                if(SHOW_TRACK)
                {
                    image_data_cache.image = MatToUIImage(image);
                }
                else
                {
                    image_data_cache.image = MatToUIImage(image);
                }
#endif
				//图像数据池？存了用于显示的图像？
                image_pool.push(image_data_cache);
            }
            
            if(LOOP_CLOSURE)
            {
                i_buf.lock();
                cv::Mat loop_image = gray.clone();
                image_buf_loop.push(make_pair(loop_image, img_msg->header));
                if(image_buf_loop.size() > WINDOW_SIZE)
                    image_buf_loop.pop();
                i_buf.unlock();
            }
        }
        else
        {
            is_calculate = false;
            if(CAMERA_MODE)
            {
                //image_data_cache.image = MatToUIImage(image);
                //image_data_cache.equ_image = img_equa.clone();
                
                //image_pool.push(image_data_cache);
            }
        }
        featuretracker.img_cnt = (featuretracker.img_cnt + 1) % FREQ;
		//？
        for (int i = 0; i < good_pts.size(); i++)
        {
            cv::circle(image, good_pts[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
        }
        TS(visualize);
        if(CAMERA_MODE)
        {
            //use aligned vins and image
            if(!vins_pool.empty() && !image_pool.empty())
            {
                while(vins_pool.size() > 1)
                {
                    vins_pool.pop();
                }
                while(!image_pool.empty() && image_pool.front().header < vins_pool.front().header)
                {
                    image_pool.pop();
                }
                if(!vins_pool.empty() && !image_pool.empty())
                {
                    vins_updated = true;
                    lateast_image = image_pool.front().image;
                    lateast_P = vins_pool.front().P;
                    lateast_R = vins_pool.front().R;
                    UIImageToMat(lateast_image, image);
                }
            }
            else if(!image_pool.empty())
            {
                if(image_pool.size() > 10)
                    image_pool.pop();
            }
        }
		//debug wrz
		if(CAMERA_MODE)printf("camera_mode:true\n");
		else printf("camer_mode:false\n");
        if(ui_main || start_show == false || vins.solver_flag != VINS::NON_LINEAR)  //show image and AR
        {
#if VINS_FRAMEWORK
            if (CAMERA_MODE || vins.solver_flag != VINS::NON_LINEAR) {
                //VINSUnityAPI::UpdateBackgroundTexture(image);
            }
#else
            if(CAMERA_MODE)
            {
				//debug wrz
				if(ui_main){
					static int iii=0;
					printf("ui_main=true,%d\n",iii<100?++iii:(iii=0));
				}
				
				cv::Mat tmp2;
                if(vins.solver_flag == VINS::NON_LINEAR && start_show)
                {
                    cv::Mat tmp;
                    vins.drawresult.startInit = true;
                    //vins.drawresult.drawAR(lateast_equa, vins.imageAI, vins.correct_point_cloud, lateast_P, lateast_R, vins_updated);
					//vins.drawresult.drawArrowAR(lateast_equa, vins.imageAI, vins.correct_point_cloud, lateast_P, lateast_R, vins_updated);
					//vins.drawresult.drawFixedArrowWithCameraAR(lateast_equa, vins.imageAI, vins.correct_point_cloud, lateast_P, lateast_R, vins_updated);
					//vins.drawresult.drawArrowTowardFixedPointAR(lateast_equa, vins.imageAI, vins.correct_point_cloud, lateast_P, lateast_R, vins_updated);
					vins.drawresult.drawArrowFllowedByPath(lateast_equa, vins.imageAI, vins.correct_point_cloud, lateast_P, lateast_R, vins_updated,vins.vinsDestPath);
					//vins.drawresult.drawArrowFllowedByFixedTag(lateast_equa, vins.imageAI, vins.correct_point_cloud, lateast_P, lateast_R, vins_updated,vins.vinsDestPath);
					
					setVinsPath();
					dispPosIn2Dmap();
					
                    vins_updated = false;
                    
                    cv::cvtColor(image, tmp, CV_RGBA2BGR);
                    cv::Mat mask;
                    cv::Mat imageAI = vins.imageAI;
                    if(!imageAI.empty())
                        cv::cvtColor(imageAI, mask, CV_RGB2GRAY);
                    imageAI.copyTo(tmp,mask);
                    cv::cvtColor(tmp, image, CV_BGRA2BGR);
                }
                if(DEBUG_MODE)
                {
                    cv::flip(lateast_equa, image, -1);
                }
                else
                {
                    cv::flip(image,tmp2,-1);
                    image = tmp2;
                    if(vins.solver_flag != VINS::NON_LINEAR || !start_show)
                        cv::cvtColor(image, image, CV_RGBA2BGR);
                }
            }
#endif
        }
        else //show VINS
        {
			dispPosIn2Dmap();
			if(vins.solver_flag == VINS::NON_LINEAR)
            {
                vins.drawresult.pose.clear();
                vins.drawresult.pose = keyframe_database.refine_path;
                vins.drawresult.segment_indexs = keyframe_database.segment_indexs;
                vins.drawresult.Reprojection(vins.image_show, vins.correct_point_cloud, vins.correct_Rs, vins.correct_Ps, box_in_trajectory);
            }
            cv::Mat tmp2 = vins.image_show;
            
            
            cv::Mat down_origin_image;
            cv::resize(image.t(), down_origin_image, cv::Size(200, 150));
            cv::cvtColor(down_origin_image, down_origin_image, CV_BGRA2RGB);
            cv::flip(down_origin_image,down_origin_image,0);
            cv::Mat imageROI;
            imageROI = tmp2(cv::Rect(10,COL - down_origin_image.rows- 10, down_origin_image.cols,down_origin_image.rows));
            cv::Mat mask;
            cv::cvtColor(down_origin_image, mask, CV_RGB2GRAY);
            down_origin_image.copyTo(imageROI, mask);
            
            
            cv::cvtColor(tmp2, image, CV_BGRA2BGR);
            cv::flip(image,tmp2,1);
            if (isNeedRotation)
                image = tmp2.t();
        }
        
        TE(visualize);
    } else {
        // Not capturing, means not started yet
#if VINS_FRAMEWORK
        //VINSUnityAPI::UpdateBackgroundTexture(image);
#else
        cv::cvtColor(image, image, CV_BGRA2RGB);
        cv::flip(image,image,-1);
        //BOOL isNeedRotation = image.size() != frameSize;
        //if (isNeedRotation)
        //    image = image.t();
#endif
    }
}


/*
 Send imu data and visual data into VINS
 */
std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
    while (true)
    {
		//如果没有Imu或image数据，直接返回
		if (imu_msg_buf.empty() || img_msg_buf.empty())
            return measurements;
        //如果imu_msg_buf最后一个数据早于img_msg_buf第一个数据，直接返回等待直至imu数据出现在img_msg_buf第一个数据后
        if (!(imu_msg_buf.back()->header > img_msg_buf.front()->header))
        {
            NSLog(@"wait for imu, only should happen at the beginning");
            return measurements;
        }
        //如果img_msg_buf第一个数据早于imu_msg_buf第一个数据，不断丢掉img_msg_buf第一个数据直至出现比Imu_msg_buf第一个数据晚的数据
        if (!(imu_msg_buf.front()->header < img_msg_buf.front()->header))
        {
            NSLog(@"throw img, only should happen at the beginning");
            img_msg_buf.pop();
            continue;
        }
		//将image和image前的imu数据取出来，在存到measurements内
        ImgConstPtr img_msg = img_msg_buf.front();
        img_msg_buf.pop();
        
        std::vector<ImuConstPtr> IMUs;
        while (imu_msg_buf.front()->header <= img_msg->header)
        {
            IMUs.emplace_back(imu_msg_buf.front());
            imu_msg_buf.pop();
        }
        //NSLog(@"IMU_buf = %d",IMUs.size());
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void send_imu(const ImuConstPtr &imu_msg)
{
    NSTimeInterval t = imu_msg->header;
    if (current_time < 0)
        current_time = t;
    double dt = (t - current_time);
    current_time = t;
    
    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};
    
    double dx = imu_msg->acc.x() - ba[0];
    double dy = imu_msg->acc.y() - ba[1];
    double dz = imu_msg->acc.z() - ba[2];
    
    double rx = imu_msg->gyr.x() - bg[0];
    double ry = imu_msg->gyr.y() - bg[1];
    double rz = imu_msg->gyr.z() - bg[2];
    //NSLog(@"IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);
    
    vins.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

void update()
{
    latest_time = lateast_imu_time;
    tmp_P = vins.Ps[WINDOW_SIZE];
    tmp_Q = Eigen::Quaterniond{vins.Rs[WINDOW_SIZE]};
    tmp_V = vins.Vs[WINDOW_SIZE];
    tmp_Ba = vins.Bas[WINDOW_SIZE];
    tmp_Bg = vins.Bgs[WINDOW_SIZE];
    acc_0 = vins.acc_0;
    gyr_0 = vins.gyr_0;
    //printf("predict update: x = %.3f, y = %.3f z = %.3f\n",tmp_P(0),tmp_P(1),tmp_P(2));
}



/*
 VINS thread: this thread tightly fuses the visual measurements and imu data and solves pose, velocity, IMU bias, 3D feature for all frame in WINNDOW
 If the newest frame is keyframe, then push it into keyframe database
 */
-(void)run{
    [_condition lock];
    while (![[NSThread currentThread] isCancelled])
    {
        [self process];
        [NSThread sleepForTimeInterval:0.01];
    }
    [_condition unlock];
    
}

int kf_global_index;
bool start_global_optimization = false;
-(void)process
{
    //imu和camera的数据，每一个元素为一对pair：当前帧的image以及当前帧和上帧之间所有的imu数据
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
    std::unique_lock<std::mutex> lk(m_buf);
    con.wait(lk, [&]
             {
				 //一般情况下每次调用getMeasurements()只会返回一对数据
                 return (measurements = getMeasurements()).size() != 0;
             });
    lk.unlock();
    waiting_lists = measurements.size();
	//对每对数据进行处理，即当前帧image和帧间的imu数据
    for(auto &measurement : measurements)
    {
		//处理帧间的每个imu数据
		for(auto &imu_msg : measurement.first)
        {
			//进行delta误差covarance的更新，进行midpoint integration得到帧间的预积分结果
			send_imu(imu_msg);
        }
        //得到当前帧image点云数据，暂且没有深度信息
        auto img_msg = measurement.second;
        map<int, Vector3d> image = img_msg->point_clouds;
        //NSLog(@"Image timestamp = %lf",img_msg->header);
        double header = img_msg->header;
        TS(process_image);
		//对当前帧点云数据进行处理，首先计算当前帧平均视差判断当前帧是否可为关键帧；然后如果当前帧为初始化帧，则进行初始化；如果不是则进行常规的三角化->优化sliding windows->更新回环检测状态
        vins.processImage(image,header,waiting_lists);
		
        TE(process_image);
        double time_now = [[NSProcessInfo processInfo] systemUptime];
        double time_vins = vins.Headers[WINDOW_SIZE];
        NSLog(@"vins delay %lf", time_now - time_vins);
        if(vins.solver_flag == vins.NON_LINEAR)
        {
            //Vector3d rotation_vins = Utility::R2ypr(vins.Rs[WINDOW_SIZE]);
            //printf("attitude compare\n");
            //printf("attitude vins pitch: %lf, roll: %lf\n", rotation_vins.y(), rotation_vins.z());
            //printf("attitude imu  pitch: %lf, roll: %lf\n", rotation_imu.y(), rotation_imu.z());
        }
        if(CAMERA_MODE)
        {
            //add state into vins buff for alignwith image
            if(vins.solver_flag == VINS::NON_LINEAR && start_show)
            {
				//缓存vins前一帧的位姿数据
				VINS_DATA_CACHE vins_data_cache;
                vins_data_cache.header = vins.Headers[WINDOW_SIZE-1];
                vins_data_cache.P = vins.correct_Ps[WINDOW_SIZE-1];
                vins_data_cache.R = vins.correct_Rs[WINDOW_SIZE-1];
                vins_pool.push(vins_data_cache);
            }
            else if(vins.failure_occur == true)
            {
				//显示？
				vins.drawresult.change_color = true;
                vins.drawresult.indexs.push_back(vins.drawresult.pose.size());
                segmentation_index++;
                keyframe_database.max_seg_index++;
                keyframe_database.cur_seg_index = keyframe_database.max_seg_index;
                
                while(!vins_pool.empty())
                    vins_pool.pop();
            }
        }
        /**
         *** start build keyframe database for loop closure
         **/
        if(LOOP_CLOSURE)
        {
            static bool first_frame = true;
            if(vins.solver_flag != vins.NON_LINEAR)
                first_frame = true;
            if(vins.marginalization_flag == vins.MARGIN_OLD && vins.solver_flag == vins.NON_LINEAR && !image_buf_loop.empty())
            {
                first_frame = false;
                if(!first_frame && keyframe_freq % LOOP_FREQ == 0)
                {
                    keyframe_freq = 0;
                    /**
                     ** save the newest keyframe to the keyframe database
                     ** only need to save the pose to the keyframe database
                     **/
                    Vector3d T_w_i = vins.Ps[WINDOW_SIZE - 2];
                    Matrix3d R_w_i = vins.Rs[WINDOW_SIZE - 2];
                    i_buf.lock();
                    while(!image_buf_loop.empty() && image_buf_loop.front().second < vins.Headers[WINDOW_SIZE - 2])
                    {
                        image_buf_loop.pop();
                    }
                    //assert(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second);
                    if(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second)
                    {
                        const char *pattern_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_pattern" ofType:@"yml"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
                        KeyFrame* keyframe = new KeyFrame(vins.Headers[WINDOW_SIZE - 2], global_frame_cnt, T_w_i, R_w_i, image_buf_loop.front().first, pattern_file, keyframe_database.cur_seg_index);
                        keyframe->setExtrinsic(vins.tic, vins.ric);
                        /*
                         ** we still need save the measurement to the keyframe(not database) for add connection with looped old pose
                         ** and save the pointcloud to the keyframe for reprojection search correspondance
                         */
						//计算所有特帧三维点在当前帧的投影，测量值；计算当前帧的三维点云，在世界坐标系下
                        keyframe->buildKeyFrameFeatures(vins);
						//在关键帧库内添加关键帧，如果关键帧过多，降采样，并更新关键帧路径显示
                        keyframe_database.add(keyframe);
                        erase_index.clear();
                        keyframe_database.resample(erase_index);
                        
                        global_frame_cnt++;
                    }
                    
                }
                else
                {
                    first_frame = false;
                }
                // update loop info
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(vins.Headers[i] == vins.front_pose.header)
                    {
                        KeyFrame* cur_kf = keyframe_database.getKeyframe(vins.front_pose.cur_index);
                        if (abs(vins.front_pose.relative_yaw) > 30.0 || vins.front_pose.relative_t.norm() > 10.0)
                        {
                            printf("Wrong loop\n");
                            cur_kf->removeLoop();
                            break;
                        }
                        cur_kf->updateLoopConnection(vins.front_pose.relative_t,
                                                     vins.front_pose.relative_q,
                                                     vins.front_pose.relative_yaw);
                        break;
                    }
                }
                /*
                 ** update the keyframe pose when this frame slides out the window and optimize loop graph
                 */
                int search_cnt = 0;
                for(int i = 0; i < keyframe_database.size(); i++)
                {
                    search_cnt++;
                    KeyFrame* kf = keyframe_database.getLastKeyframe(i);
                    if(kf->header == vins.Headers[0])
                    {
                        kf->updateOriginPose(vins.Ps[0], vins.Rs[0]);
                        //update edge
                        // if loop happens in this frame, update pose graph;
                        if (kf->has_loop)
                        {
                            kf_global_index = kf->global_index;
                            start_global_optimization = true;
                        }
                        break;
                    }
                    else
                    {
                        if(search_cnt > 2 * WINDOW_SIZE)
                            break;
                    }
                }
                keyframe_freq++;
                i_buf.unlock();
            }
        }
		//loop结束
        update();
        waiting_lists--;
        //finish solve one frame
        [self performSelectorOnMainThread:@selector(showInputView) withObject:nil waitUntilDone:YES];
    }
}


/*
 Loop detection thread: this thread detect loop for newest keyframe and retrieve features
 */
-(void)loop_thread{
    
    if(LOOP_CLOSURE && loop_closure == NULL)
    {
        NSLog(@"loop start load voc");
        TS(load_voc);
        const char *voc_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_k10L6" ofType:@"bin"]
                                cStringUsingEncoding:[NSString defaultCStringEncoding]];
        loop_closure = new LoopClosure(voc_file, COL, ROW);
        TE(load_voc);
        NSLog(@"loop load voc finish");
        
        voc_init_ok = true;
    }
    while(![[NSThread currentThread] isCancelled] )
    {
        if(!LOOP_CLOSURE)
        {
            [NSThread sleepForTimeInterval:0.5];
            continue;
        }
        if(!erase_index.empty() && loop_closure != NULL)
            loop_closure->eraseIndex(erase_index);
        
        if (loop_check_cnt < global_frame_cnt)
        {
            KeyFrame* cur_kf = keyframe_database.getLastUncheckKeyframe();
            assert(loop_check_cnt == cur_kf->global_index);
            loop_check_cnt++;
            cur_kf->check_loop = 1;
            
            cv::Mat current_image;
            current_image = cur_kf->image;
            
            std::vector<cv::Point2f> measurements_old;
            std::vector<cv::Point2f> measurements_old_norm;
            std::vector<cv::Point2f> measurements_cur;
            std::vector<int> features_id;
            std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;
            
            bool loop_succ = false;
            
            vector<cv::Point2f> cur_pts;
            vector<cv::Point2f> old_pts;
            cur_kf->extractBrief(current_image);
            printf("loop extract %d feature\n", cur_kf->keypoints.size());
            loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);
            if(loop_succ)
            {
                KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);
                if (old_kf == NULL)
                {
                    printf("NO such frame in keyframe_database\n");
                    assert(false);
                }
                printf("loop succ %d with %drd image\n", process_keyframe_cnt-1, old_index);
                assert(old_index!=-1);
                
                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;
                
                old_kf->getOriginPose(T_w_i_old, R_w_i_old);
                cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                   measurements_old, measurements_old_norm);
                measurements_cur = cur_kf->measurements;
                features_id = cur_kf->features_id;
                
                if(measurements_old_norm.size()>MIN_LOOP_NUM)
                {
                    
                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm;
                    retrive_data.features_ids = features_id;
                    vins.retrive_pose_data = (retrive_data);
                    printf("loop push\n");
                    
                    //cout << "old pose " << T_w_i_old.transpose() << endl;
                    //cout << "refinded pose " << T_w_i_refine.transpose() << endl;
                    // add loop edge in current frame
                    cur_kf->detectLoop(old_index);
                    keyframe_database.addLoop(old_index);
                    old_kf->is_looped = 1;
                }
            }
            cur_kf->image.release();
        }
        else
        {
            i_buf.unlock();
        }
        [NSThread sleepForTimeInterval:0.05];
    }
    //[self process_loop_detection];
}

/*
 GLobal Pose graph thread: optimize global pose graph based on realative pose from vins and update the keyframe database
 */
-(void)globalLoopThread{
    while (![[NSThread currentThread] isCancelled])
    {
        if(start_global_optimization)
        {
            start_global_optimization = false;
            TS(debug_loop_thread);
            keyframe_database.optimize4DoFLoopPoseGraph(kf_global_index,
                                                        loop_correct_t,
                                                        loop_correct_r);
            vins.t_drift = loop_correct_t;
            vins.r_drift = loop_correct_r;
            TE(debug_loop_thread);
            [NSThread sleepForTimeInterval:0.17];
        }
        [NSThread sleepForTimeInterval:0.03];
    }
}

/*
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 IMU data process and interploration
 
 */
bool imuDataFinished = false;
bool vinsDataFinished = false;
shared_ptr<IMU_MSG> cur_acc(new IMU_MSG());
vector<IMU_MSG> gyro_buf;  // for Interpolation
- (void)imuStartUpdate
{
    CMMotionManager *motionManager = [[CMMotionManager alloc] init];
    if (!motionManager.accelerometerAvailable) {
        NSLog(@"没有加速计");
    }
#ifdef DATA_EXPORT
    motionManager.accelerometerUpdateInterval = 0.1;
    motionManager.gyroUpdateInterval = 0.1;
#else
    motionManager.accelerometerUpdateInterval = 0.01;
    motionManager.gyroUpdateInterval = 0.01;
#endif
    
    [motionManager startDeviceMotionUpdates];
    //获得加速度计数据,存到cur_acc
    [motionManager startAccelerometerUpdatesToQueue:[NSOperationQueue currentQueue]
                                        withHandler:^(CMAccelerometerData *latestAcc, NSError *error)
     {
         double header = motionManager.deviceMotion.timestamp;
         rotation_imu << motionManager.deviceMotion.attitude.yaw * 180.0 / M_PI,  //yaw
         motionManager.deviceMotion.attitude.roll * 180.0 / M_PI,  //pitch for vins
         motionManager.deviceMotion.attitude.pitch * 180.0 / M_PI;  //roll for vins
         if(imu_prepare<10)
         {
             imu_prepare++;
         }
		 //xyz方向的数值均取反了？
         shared_ptr<IMU_MSG> acc_msg(new IMU_MSG());
         acc_msg->header = latestAcc.timestamp;
         acc_msg->acc << -latestAcc.acceleration.x * GRAVITY,
         -latestAcc.acceleration.y * GRAVITY,
         -latestAcc.acceleration.z * GRAVITY;
         cur_acc = acc_msg;
         //printf("imu acc update %lf %lf %lf %lf\n", acc_msg->header, acc_msg->acc.x(), acc_msg->acc.y(), acc_msg->acc.z());
         
     }];
	//获得陀螺仪的数据，存在gyro_msg
    [motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMGyroData *latestGyro, NSError *error)
     {
         //The time stamp is the amount of time in seconds since the device booted.
         NSTimeInterval header = latestGyro.timestamp;
         if(header<=0)
             return;
         if(imu_prepare < 10)
             return;
         //存储到gyro_msg
         IMU_MSG gyro_msg;
         gyro_msg.header = header;
         gyro_msg.gyr << latestGyro.rotationRate.x,
         latestGyro.rotationRate.y,
         latestGyro.rotationRate.z;
         //gyro_buf包括前后两次陀螺仪数据，只有加速度计数据时间戳在前后之间才同时存储加速度和陀螺仪，相当于同步陀螺仪和加速度计时间戳
         if(gyro_buf.size() == 0)
         {
             gyro_buf.push_back(gyro_msg);
             gyro_buf.push_back(gyro_msg);
             return;
         }
         else
         {
             gyro_buf[0] = gyro_buf[1];
             gyro_buf[1] = gyro_msg;
         }
         //interpolation
		 //只有加速度数据时间戳和陀螺仪保持一致才共同存为imu_msg，并且使用时间插值计算陀螺仪数据，即为：gyro=gyro0+(acc_t-gyro0_t)*(gyro1-gyro0)/(gyro1_t-gyro0_t)
         shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
         if(cur_acc->header >= gyro_buf[0].header && cur_acc->header < gyro_buf[1].header)
         {
             imu_msg->header = cur_acc->header;
             imu_msg->acc = cur_acc->acc;
             imu_msg->gyr = gyro_buf[0].gyr + (cur_acc->header - gyro_buf[0].header)*(gyro_buf[1].gyr - gyro_buf[0].gyr)/(gyro_buf[1].header - gyro_buf[0].header);
             //printf("imu gyro update %lf %lf %lf\n", gyro_buf[0].header, imu_msg->header, gyro_buf[1].header);
             //printf("imu inte update %lf %lf %lf %lf\n", imu_msg->header, gyro_buf[0].gyr.x(), imu_msg->gyr.x(), gyro_buf[1].gyr.x());
         }
		 //否则认为imu数据不可取，这样岂不如果imu加速度计和陀螺仪时间差异较大则会一直认为数据不可取吗？
         else
         {
             printf("imu error %lf %lf %lf\n", gyro_buf[0].header, cur_acc->header, gyro_buf[1].header);
             return;
         }
         
         //for save data
		 //回放存储的imu数据
         if(start_playback)
         {
             //TS(read_imu_buf);
             if(imuDataFinished)
                 return;
             [imuReader getBytes:&imuData range: NSMakeRange(imuDataReadIndex * sizeof(imuData), sizeof(imuData))];
             imuDataReadIndex++;
             if(imuData.header == 0)
             {
                 imuDataFinished = true;
                 return;
             }
             imu_msg->header = imuData.header;
             imu_msg->acc = imuData.acc;
             imu_msg->gyr = imuData.gyr;
             //TE(read_imu_buf);
#ifdef DATA_EXPORT
             printf("record play imu: %lf %lf %lf %lf %lf %lf %lf\n",imuData.header,imu_msg->acc.x(), imu_msg->acc.y(), imu_msg->acc.z(),
                    imu_msg->gyr.x(), imu_msg->gyr.y(), imu_msg->gyr.z());
#endif
         }
         //记录imu数据
         if(start_record)
         {
             TS(record_imu_buf);
             imuData.header = imu_msg->header;
             imuData.acc = imu_msg->acc;
             imuData.gyr = imu_msg->gyr;
             [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
             imuDataIndex++;
             TE(record_imu_buf);
             //NSLog(@"record: imu %lf, %lu",imuData.header,imuDataIndex);
         }
         
         m_time.lock();
         lateast_imu_time = imu_msg->header;
         m_time.unlock();
#if VINS_FRAMEWORK
         //predict status
		 //将上一时刻的加速度和角速度和当前时刻的进行中点插值，并预测计算下一时刻的Q，P,V积分
         if(!CAMERA_MODE && ENABLE_IMU_PRIDICT)
         {
             if(latest_time > 0)
             {
                 double t = imu_msg->header;
                 double dt = t - latest_time;
                 latest_time = t;
				 
				 //当前时刻的imu数据
                 double dx = imu_msg->acc.x();
                 double dy = imu_msg->acc.y();
                 double dz = imu_msg->acc.z();
                 Eigen::Vector3d linear_acceleration{dx, dy, dz};
                 
                 double rx = imu_msg->gyr.x();
                 double ry = imu_msg->gyr.y();
                 double rz = imu_msg->gyr.z();
                 Eigen::Vector3d angular_velocity{rx, ry, rz};
                 
                 Vector3d g{0,0,GRAVITY};
				 //上一时刻的加速度，相对惯性系
                 Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * g);
				 
				 //中点插值得到当前时刻的角速度，相对惯性系
                 Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
                 //Eigen::Vector3d un_gyr = gyr_0 - tmp_Bg;
				 //当前Q的更新
                 tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);
				 
				 //当前加速度，相对惯性系
                 Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse() * g);
                 //中点插值更新加速度，相对惯性系;加速度小于阈值，则直接设为0
                 Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
                 double acc_threshold = 0.2;
                 un_acc << ((fabs(un_acc.x())<acc_threshold)?0.0:un_acc.x()),
                 ((fabs(un_acc.y())<acc_threshold)?0.0:un_acc.y()),
                 ((fabs(un_acc.z())<acc_threshold)?0.0:un_acc.z());
                 //printf("predict: ax = %.3lf, ay = %.3lf, az = %.3lf\n",un_acc(0),un_acc(1),un_acc(2));
				 //预积分得到P,V
                 tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
                 tmp_V = tmp_V + dt * un_acc;
                 
                 //acc_0 = linear_acceleration;
                 //gyr_0 = angular_velocity;
                 //printf("predict x = %.3f, y = %.3f z = %.3f\n",tmp_P(0),tmp_P(1),tmp_P(2));
             }
         }
#endif
         //img_msg callback
         m_buf.lock();
		 //将imu存在imu_msg_buf内
         imu_msg_buf.push(imu_msg);
         //NSLog(@"IMU_buf timestamp %lf, acc_x = %lf",imu_msg_buf.front()->header,imu_msg_buf.front()->acc.x());
         m_buf.unlock();
         con.notify_one();
     }];
}

/********************************************************************UI View Controler********************************************************************/
- (void)showInputView
{
    NSString *stringView;
    static bool finish_init = false;
    if(vins.solver_flag != vins.NON_LINEAR)
    {
        finish_init = false;
        switch (vins.init_status) {
            case vins.FAIL_IMU:
                stringView = [NSString stringWithFormat:@"STA: FAIL_IMU"];
                break;
            case vins.FAIL_PARALLAX:
                stringView = [NSString stringWithFormat:@"STA: FAIL_PARA"];
                break;
            case vins.FAIL_RELATIVE:
                stringView = [NSString stringWithFormat:@"STA: FAIL_RELA"];
                break;
            case vins.FAIL_SFM:
                stringView = [NSString stringWithFormat:@"STA: FAIL_SFM"];
                break;
            case vins.FAIL_PNP:
                stringView = [NSString stringWithFormat:@"STA: FAIL_PNP"];
                break;
            case vins.FAIL_ALIGN:
                stringView = [NSString stringWithFormat:@"STA: FAIL_ALIGN"];
                break;
            case vins.FAIL_CHECK:
                stringView = [NSString stringWithFormat:@"STA: FAIL_COST"];
                break;
            case vins.SUCC:
                stringView = [NSString stringWithFormat:@"STA: SUCC!"];
                break;
            default:
                break;
        }
        [_X_label setText:stringView];
        stringView = [NSString stringWithFormat:@"FAIL: %d times", vins.fail_times];
        [_Y_label setText:stringView];
        stringView = [NSString stringWithFormat:@"PARALLAX: %d", vins.parallax_num_view];
        [_Z_label setText:stringView];
        
        stringView = [NSString stringWithFormat:@"Initializing: %d%%", vins.initProgress];
        [_feature_label2 setText:stringView];
        
        [_feature_label2 setHidden:NO];
        [_feature_label3 setHidden:NO];
        [indicator setHidden:NO];
        [featureImageView setHidden:NO];
    }
    else
    {
        if(finish_init == false)
        {
            //Hide init UI
            [_feature_label2 setHidden:YES];
            [_feature_label3 setHidden:YES];
            [indicator setHidden:YES];
            [featureImageView setHidden:YES];
            
            startButton.enabled = false;
            _stopButton.enabled = true;
            
            start_active = false;
            start_show = true;
            finish_init = true;
        }
        
        float x_view = (float)vins.correct_Ps[frame_cnt][0];
        float y_view = (float)vins.correct_Ps[frame_cnt][1];
//        float z_view = (float)vins.correct_Ps[frame_cnt][2];
		//wrz
//		float x_view=(float)vins.curPosIn2Dmap_m.x();
//		float y_view=(float)vins.curPosIn2Dmap_m.y();
//		float x_view=(float)vins.curPosIn2Dmap_pixel.x();
//		float y_view=(float)vins.curPosIn2Dmap_pixel.y();
//		float x_view=(float)vins.curTruthPos.x()/Width_2D_Map_m*Width_2D_Map_pixel;
//		float y_view=(float)vins.curTruthPos.y()/Height_2D_Map_m*Height_2D_Map_pixel;
		float z_view=(float)vins.initForwardDirecIn2Dmap.x();
		if(x_view_last == -5000)
        {
            x_view_last = x_view;
            y_view_last = y_view;
            z_view_last = z_view;
        }
        total_odom += sqrt(pow((x_view - x_view_last), 2) +
                           pow((y_view - y_view_last), 2) +
                           pow((z_view - z_view_last), 2));
        x_view_last = x_view;
        y_view_last = y_view;
        z_view_last = z_view;
        
        stringView = [NSString stringWithFormat:@"X:%.2f",x_view];
        [_X_label setText:stringView];
		float z_vins = (float)vins.correct_Ps[frame_cnt][2];
        stringView = [NSString stringWithFormat:@"z_m:%.2f",z_vins];
        //stringView = [NSString stringWithFormat:@"COST:%.2lf",vins.final_cost];
        //stringView = [NSString stringWithFormat:@"COST: %d, %.2lf",vins.visual_factor_num, vins.visual_cost];
        [_total_odom_label setText:stringView];
        stringView = [NSString stringWithFormat:@"Y:%.2f",y_view];
        [_Y_label setText:stringView];
        stringView = [NSString stringWithFormat:@"dx:%.2f",z_view];
        [_Z_label setText:stringView];
		
		float dyIn2Dmap=vins.initForwardDirecIn2Dmap.y();
		stringView = [NSString stringWithFormat:@"dy:%.2f",dyIn2Dmap];
		[_buf_label setText:stringView];
		
		float x2D=(float)vins.curTruthPosIn2Dmap_pixel.x();
		float y2D=(float)vins.curTruthPosIn2Dmap_pixel.y();
		stringView = [NSString stringWithFormat:@"x2D: %f",x2D];
		[_loop_label setText:stringView];
//		stringView = [NSString stringWithFormat:@"y2D: %f",y2D];
		int pathNumTmp=-2;
		if(vins.hasInitialP0)pathNumTmp=-1;
		if(vins.drawresult.hasInitialPlane)pathNumTmp=vins.vinsDestPath.size();
		
		stringView = [NSString stringWithFormat:@"pathNum: %d",pathNumTmp];
		[_feature_label setText:stringView];

		
		//更新当前位置在平面图上的位置
		int curPosX_2Dmap=vins.curPosIn2Dmap_pixel.x();
		int curPosY_2Dmap=vins.curPosIn2Dmap_pixel.y();
		//	int curPosX_2Dmap=vins.curTruthPos.x()/Width_2D_Map_m*Width_2D_Map_pixel;
		//	int curPosY_2Dmap=vins.curTruthPos.y()/Height_2D_Map_m*Height_2D_Map_pixel;
		printf("wrz40 curPosX:%u, curPosY:%u\n",curPosX_2Dmap,curPosY_2Dmap);
		[view setPixelX:curPosX_2Dmap];
		[view setPixelY:curPosY_2Dmap];
		//					[view setPixelX:10];
		//					[view setPixelY:10];
		[view setNeedsDisplay];

		
    }
//	float dyIn2Dmap=vins.initForwardDirecIn2Dmap.y();
//    stringView = [NSString stringWithFormat:@"dy:%.2f",dyIn2Dmap];
//    [_buf_label setText:stringView];
    //NSString *stringZ = [NSString stringWithFormat:@"Z:%.2f",z_view, vins.f_manager.getFeatureCount()];
//    if(old_index != -1)
//    {
//        stringView = [NSString stringWithFormat:@"LOOP with %d",old_index];
//        [_loop_label setText:stringView];
//    }
//    stringView = [NSString stringWithFormat:@"FEATURE: %d",vins.feature_num];
//    [_feature_label setText:stringView];
}

-(void)showOutputImage:(UIImage*)image
{
    [featureImageView setImage:image];
}


/********************************************************************UI View Controler********************************************************************/


/********************************************************************UI Button Controler********************************************************************/

bool start_active = true;

-(IBAction)startButtonPressed:(id)sender
{
    printf("start\n");
    
    if(!voc_init_ok && LOOP_CLOSURE)
    {
        [alertView show];
    }
    else if(start_active)
    {
        start_playback_vins = true;
        startButton.enabled = false;
        _stopButton.enabled = true;
        
        start_active = false;
        start_show = true;
    }
}


-(IBAction)extrange:(id)sender   //actually is stop button
{
    if(!start_active)
    {
        startButton.enabled = true;
        _stopButton.enabled = false;
        
        start_active = true;
    }
}

-(IBAction)switchUI:(UISegmentedControl *)sender
{
    switch (_switchUI.selectedSegmentIndex)
    {
        case 0:
#if VINS_FRAMEWORK
            CAMERA_MODE = false;
#else
            printf("show AR\n");
            ui_main = true;
            box_in_AR= true;
#endif
            break;
        case 1:
#if VINS_FRAMEWORK
            CAMERA_MODE = true;
#else
            ui_main = false;
            if (box_in_AR)
                box_in_trajectory = true;
            printf("show VINS\n");
#endif
            break;
        default:
            break;
    }
}
- (IBAction)fovSliderValueChanged:(id)sender {
    self.fovLabel.text = [[NSNumber numberWithFloat:self.fovSlider.value] stringValue];
    
#if VINS_FRAMEWORK
    //VINSUnityAPI::SetCameraFOV(self.fovSlider.value);
#endif
}

- (void) handlePan:(UIPanGestureRecognizer*) recognizer
{
    if(ui_main and 0)
        return;
    
    if (!ui_main)
    {
        CGPoint translation = [recognizer translationInView:self.view];
        CGFloat velocityX = [recognizer velocityInView:self.view].x;
        CGFloat velocityY = [recognizer velocityInView:self.view].y;
        //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
        static CGFloat vx_last = 0;
        static CGFloat vy_last = 0;
        
        CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_last;
        CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_last;
        vx_last = vx_smooth;
        vy_last = vy_smooth;
        if(recognizer.numberOfTouches == 2)
        {
            vins.drawresult.Y0 += vx_smooth/100.0;
            vins.drawresult.X0 += vy_smooth/100.0;
        }
        else
        {
            vins.drawresult.theta += vy_smooth/100.0;
            vins.drawresult.theta = fmod(vins.drawresult.theta, 360.0);
            vins.drawresult.phy += vx_smooth/100.0;
            vins.drawresult.phy = fmod(vins.drawresult.phy, 360.0);
        }
        
        vins.drawresult.change_view_manualy = true;
    }
    else
    {
        CGPoint translation = [recognizer translationInView:self.view];
        CGFloat velocityX = [recognizer velocityInView:self.view].x;
        CGFloat velocityY = [recognizer velocityInView:self.view].y;
        //CGFloat translationX =
        //CGFloat translationY = [recognizer translationInView:self.view].y;
        //NSLog(@"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!pipikk test x: %f y: %f", translationX, translationY);
        //NSLog(@"!!!!!!!!!!!!!!!!!!!!!!%f  %f", imageView.frame.size.height, imageView.frame.size.width);
        CGPoint point = [recognizer locationInView:self.view];
        //NSLog(@"X Location: %f", point.x);
        //NSLog(@"Y Location: %f",point.y);
        
        //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
        static CGFloat vx_lastAR = 0;
        static CGFloat vy_lastAR = 0;
        
        CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_lastAR;
        CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_lastAR;
        vx_lastAR = vx_smooth;
        vy_lastAR = vy_smooth;
        if(recognizer.numberOfTouches == 2)
        {
            
            vins.drawresult.Y0AR += vx_smooth/100.0;
            vins.drawresult.X0AR += vy_smooth/100.0;
            
            vins.drawresult.locationXT2 = point.x * 640.0 / imageView.frame.size.width;
            vins.drawresult.locationYT2 = point.y * 480.0 / imageView.frame.size.height;
            
            vins.drawresult.finger_s = 0;
            vins.drawresult.finger_p = 0;
            if ((vins.drawresult.finger_d ++) > 7)
                vins.drawresult.finger_state = 2;
			//debug wrz
			static int nnn=0;
			printf("twoTouch:%u\n",nnn=(nnn<100?++nnn:0));
        }
        else
        {
            vins.drawresult.thetaAR += vy_smooth/100.0;
            //vins.drawresult.thetaAR = fmod(vins.drawresult.thetaAR, 360.0);
            vins.drawresult.phyAR += vx_smooth/100.0;
            //vins.drawresult.phyAR = fmod(vins.drawresult.phyAR, 360.0);
            
            vins.drawresult.locationX = point.x * 640.0 / imageView.frame.size.width;
            vins.drawresult.locationY = point.y * 480.0 / imageView.frame.size.height;
            
            vins.drawresult.finger_d = 0;
            vins.drawresult.finger_p = 0;
            if ((vins.drawresult.finger_s ++) > 7)
                vins.drawresult.finger_state = 1;
			//debug_wrz
			printf("locationX: %f, locationY:%f\n",vins.drawresult.locationX,vins.drawresult.locationY);
        }
    }
    
    
}

- (void) handlePinch:(UIPinchGestureRecognizer*) recognizer
{
    if(ui_main and 0)
        return;
    
    if (!ui_main)
    {
        vins.drawresult.change_view_manualy = true;
        if(vins.drawresult.radius > 5 || recognizer.velocity < 0)
            vins.drawresult.radius -= recognizer.velocity * 0.5;
        else
        {
            vins.drawresult.Fx += recognizer.velocity * 15;
            if(vins.drawresult.Fx < 50)
                vins.drawresult.Fx = 50;
            vins.drawresult.Fy += recognizer.velocity * 15;
            if(vins.drawresult.Fy < 50)
                vins.drawresult.Fy = 50;
        }
    }
    else{
        
        vins.drawresult.finger_s = 0;
        vins.drawresult.finger_d = 0;
        if ((vins.drawresult.finger_p ++) > 7)
            vins.drawresult.finger_state = 3;
        
        CGPoint point = [recognizer locationInView:self.view];
        vins.drawresult.locationXP = point.x * 640.0 / imageView.frame.size.width;
        vins.drawresult.locationYP = point.y * 480.0 / imageView.frame.size.height;
        
        //NSLog(@"pipikk_radius: %f velocity: ", vins.drawresult.radiusAR, recognizer.velocity);
        
        //if(vins.drawresult.radiusAR > 5 || recognizer.velocity < 0)
        //{
        vins.drawresult.radiusAR -= recognizer.velocity * 0.5;
        //}
		
    }
    
}

- (void) handleTap:(UITapGestureRecognizer*) recognizer
{
    if (!ui_main)
    {
        
    }
    else{
        
        /*vins.drawresult.finger_s = 0;
         vins.drawresult.finger_d = 0;
         if ((vins.drawresult.finger_p ++) > 7)
         vins.drawresult.finger_state = 3;*/
        
        CGPoint point = [recognizer locationInView:self.view];
        vins.drawresult.locationTapX = point.x * 640.0 / imageView.frame.size.width;
        vins.drawresult.locationTapY = point.y * 480.0 / imageView.frame.size.height;
        
        vins.drawresult.tapFlag = true;
		
		//debug_wrz
		printf("tap once\n");
        
    }
    
}

- (IBAction)updatePosIn2Dmap:(UIButton *)sender {
	int curPosX_2Dmap=vins.curPosIn2Dmap_pixel.x();
	int curPosY_2Dmap=vins.curPosIn2Dmap_pixel.y();
//	int curPosX_2Dmap=vins.curTruthPos.x()/Width_2D_Map_m*Width_2D_Map_pixel;
//	int curPosY_2Dmap=vins.curTruthPos.y()/Height_2D_Map_m*Height_2D_Map_pixel;
	printf("wrz40 curPosX:%u, curPosY:%u\n",curPosX_2Dmap,curPosY_2Dmap);
	[view setPixelX:curPosX_2Dmap];
	[view setPixelY:curPosY_2Dmap];
	//					[view setPixelX:10];
	//					[view setPixelY:10];
	[view setNeedsDisplay];
}


- (IBAction)getInitPoseInMap:(UIButton *)sender {
	NSLog(@"initial transformation from vins to 2Dmap");
	
	double direction_y=0.0;
	double direction_x=0.0;
	double position_y=lateast_P.y();
	double position_x=lateast_P.x();
//	if([self requstTruthPos:direction_y requestDx:direction_x requestPy:position_y requestPx:position_x]){
	if(true){
		NSLog(@"wrz0 initialP0 dy:%f, dx:%f, py:%f, px:%f",direction_y,direction_x,position_y,position_x);
//		计算从vins到2D平面图间的转换关系
//		Vector3f vins_xAxisIn2Dmap(direction_x,direction_y,0.0);
		//for demo
		Vector3f vins_xAxisIn2Dmap(0,1.0,0.0);
		vins_xAxisIn2Dmap=vins_xAxisIn2Dmap/vins_xAxisIn2Dmap.norm();
		//平面图的z轴是垂直朝下的，vins的x轴朝前，y轴朝左，所以vins的y轴在2Dmap的表示为vins的x轴在2Dmap的表示叉乘2Dmap的z轴
		Vector3f vins_zAxisIn2Dmap(0.0,0.0,1.0);
		Vector3f vins_yAxisIn2Dmap=vins_xAxisIn2Dmap.cross(vins_zAxisIn2Dmap);
		vins_yAxisIn2Dmap=vins_yAxisIn2Dmap/vins_yAxisIn2Dmap.norm();
//		Vector2f T_vinsTo2Dmap(position_x,position_y);
		//for demo
//		float initX2Dmap=(512.0)/Width_2D_Map_pixel*Width_2D_Map_m+1.82;
//		float initY2Dmap=(760.0)/Height_2D_Map_pixel*Height_2D_Map_m-2.15;
		float initX2Dmap=(512.0)/Width_2D_Map_pixel*Width_2D_Map_m+2.02;
		float initY2Dmap=(780.0)/Height_2D_Map_pixel*Height_2D_Map_m-2.80;
		Vector2f T_vinsTo2Dmap(initX2Dmap,initY2Dmap);
		vins.Rwc_vinsTo2Dmap.block<2, 1>(0, 0)=vins_xAxisIn2Dmap.block<2, 1>(0, 0);
		vins.Rwc_vinsTo2Dmap.block<2, 1>(0, 1)=vins_yAxisIn2Dmap.block<2, 1>(0, 0);
		vins.wTcw_vinsTo2Dmap=T_vinsTo2Dmap;
		//计算从2D平面图到vins间的转换关系
		vins.Rcw_2DmapTovins=vins.Rwc_vinsTo2Dmap.transpose();
		vins.cTwc_2DmapTovins=-vins.Rwc_vinsTo2Dmap.transpose()*vins.wTcw_vinsTo2Dmap;
		//记录初始相机朝向在平面图的方向
		vins.initForwardDirecIn2Dmap.x()=direction_x;
		vins.initForwardDirecIn2Dmap.y()=direction_y;
		
		//设置目标路径
//		vector<Vector2i> pathDestIn2Dmap;
//		pathDestIn2Dmap.push_back(Vector2i(512,750));
//		pathDestIn2Dmap.push_back(Vector2i(295,760));
//		pathDestIn2Dmap.push_back(Vector2i(295,1085));
//		pathDestIn2Dmap.push_back(Vector2i(512,1085));
//		pathDestIn2Dmap.push_back(Vector2i(512,750));
		
//		pathDestIn2Dmap.push_back(Vector2i(680,796));
//		pathDestIn2Dmap.push_back(Vector2i(670,1396));
//		pathDestIn2Dmap.push_back(Vector2i(620,1918));
//		pathDestIn2Dmap.push_back(Vector2i(1684,1918));
//		pathDestIn2Dmap.push_back(Vector2i(1684,1628));
		
//		int pathDestNum=pathDestIn2Dmap.size();
//		vins.drawresult.pathDestNum=pathDestNum;
//		vins.drawresult.curDestIndex=0;
//		vins.vinsDestPath.clear();
//		for(int i=0;i<pathDestNum;++i){
//			Vector2i tmp_i=pathDestIn2Dmap[i];
//			Vector2f tmp_f;
//			tmp_f.x()=(float)tmp_i.x()/Width_2D_Map_pixel*Width_2D_Map_m;
//			tmp_f.y()=(float)tmp_i.y()/Height_2D_Map_pixel*Height_2D_Map_m;
//			tmp_f=vins.Rcw_2DmapTovins*tmp_f+vins.cTwc_2DmapTovins;
//			Vector3f pathDestInVins;
//			pathDestInVins.x()=tmp_f.x();
//			pathDestInVins.y()=tmp_f.y();
//			pathDestInVins.z()=0.0;
//			vins.vinsDestPath.push_back(pathDestInVins);
//			printf("wrz06 %u destX:%f, destY:%f, destZ:%f\n",i, pathDestInVins.x(),pathDestInVins.y(),pathDestInVins.z());
//			
//		}
		vins.vinsDestPath.clear();
//		vins.vinsDestPath.push_back(Vector3f(2.51,1.5,0));
//		vins.vinsDestPath.push_back(Vector3f(2.51,-8.01,0));
//		vins.vinsDestPath.push_back(Vector3f(9.52,-8.01,0));
//		vins.vinsDestPath.push_back(Vector3f(9.52,1.5,0));
//		vins.vinsDestPath.push_back(Vector3f(1.9,1.31,0));
//		vins.vinsDestPath.push_back(Vector3f(2.43,-7.01,0));
//		vins.vinsDestPath.push_back(Vector3f(9.0,-6.51,0));
//		vins.vinsDestPath.push_back(Vector3f(7.81,1.58,0));

//		int pathDestNum=vins.vinsDestPath.size();
//		vins.drawresult.pathDestNum=pathDestNum;

		//vins.vinsDestPath.push_back(Vector3f(2.51,1.5,0));
		
		
//		Vector3f initCurDest;
//		initCurDest.x()=2.51;
//		initCurDest.y()=1.5;
//		initCurDest.z()=0;
//		Vector3f initNextDest;
//		initNextDest.x()=2.51;
//		initNextDest.y()=-7.01;
//		initNextDest.z()=0;
//		
//		vins.drawresult.curDest=initCurDest;
//		vins.drawresult.nextDest=initNextDest;
		
		vins.hasInitialP0=true;
		
		
		
//		printf("wrz01 x0:%f, x1:%f, y0:%f, y1:%f,t0Tmp:%f, t1Tmp:%f\n",vins_xAxisIn2Dmap.x(),vins_xAxisIn2Dmap.y(),vins_yAxisIn2Dmap.x(),vins_yAxisIn2Dmap.y(),position_x,position_y);
		printf("wrz01 x0:%f, x1:%f, y0:%f, y1:%f, t0:%f, t1:%f\n",vins.Rwc_vinsTo2Dmap(0,0),vins.Rwc_vinsTo2Dmap(1,0),vins.Rwc_vinsTo2Dmap(0,1),vins.Rwc_vinsTo2Dmap(1,1),vins.wTcw_vinsTo2Dmap.x(),vins.wTcw_vinsTo2Dmap.y());
		printf("wrz010 x0:%f, x1:%f, y0:%f, y1:%f, t0:%f, t1:%f\n",vins.Rcw_2DmapTovins(0,0),vins.Rcw_2DmapTovins(1,0),vins.Rcw_2DmapTovins(0,1),vins.Rcw_2DmapTovins(1,1),vins.cTwc_2DmapTovins.x(),vins.cTwc_2DmapTovins.y());
		
	}
}

-(bool) requstTruthPos:(double&) direction_y requestDx:(double&) direction_x requestPy:(double&) position_y requestPx:(double&) position_x
{
	NSLog(@"request true positon from server");
	//send image to server
	UIImage *uiimage=MatToUIImage(vins.imageGray);
	const NSString* requestTruePathResult=[self httpAsynchronousRequest:uiimage];
	NSLog(@"wrz requestTruePathResult %@",requestTruePathResult);
	//解析出在2D平面图上的方位和坐标
	NSRange foundDirecY = [requestTruePathResult rangeOfString:@"direction_y"];
	NSRange foundDirecX = [requestTruePathResult rangeOfString:@"direction_x"];
	NSRange foundPosiY = [requestTruePathResult rangeOfString:@"position_y"];
	NSRange foundPosiX = [requestTruePathResult rangeOfString:@"position_x"];
	
	bool requestOK=true;
	if(foundDirecY.location==NSNotFound||foundDirecX.location==NSNotFound||foundPosiY.location==NSNotFound||foundPosiX.location==NSNotFound){
		requestOK=false;
	}
	if(!requestOK)printf("wrz request fail 0\n");
	
	if(requestOK){
//		NSLog(@"foundDy before %lu,length:%lu",foundDirecY.location,foundDirecY.length);
//		NSLog(@"foundDX before %lu,length:%lu",foundDirecX.location,foundDirecX.length);
//		NSLog(@"foundPy before %lu,length:%lu",foundPosiY.location,foundPosiY.length);
//		NSLog(@"foundPx before %lu,length:%lu",foundPosiX.location,foundPosiX.length);
		double direction_y_tmp=0.0;
		double direction_x_tmp=0.0;
		double position_y_tmp=0.0;
		double position_x_tmp=0.0;
		if(foundDirecY.location!=NSNotFound){
			foundDirecY.location=foundDirecY.location+foundDirecY.length+NSUInteger(2);
			foundDirecY.length=foundDirecX.location-NSUInteger(2)-foundDirecY.location;
			if(foundDirecY.length>0){
				direction_y_tmp =[[requestTruePathResult substringWithRange:foundDirecY] doubleValue];
			}
			else requestOK=false;
			if(!requestOK)printf("wrz request fail 1\n");
		}
		if(foundDirecX.location!=NSNotFound){
			foundDirecX.location=foundDirecX.location+foundDirecX.length+NSUInteger(2);
			foundDirecX.length=foundPosiY.location-NSUInteger(2)-foundDirecX.location;
			if(foundDirecX.length>0){
				direction_x_tmp =[[requestTruePathResult substringWithRange:foundDirecX] doubleValue];
			}
			else requestOK=false;
			if(!requestOK)printf("wrz request fail 2\n");
		}
		if(foundPosiY.location!=NSNotFound){
			foundPosiY.location=foundPosiY.location+foundPosiY.length+NSUInteger(2);
			foundPosiY.length=foundPosiX.location-NSUInteger(2)-foundPosiY.location;
			if(foundPosiY.length>0){
				position_y_tmp =[[requestTruePathResult substringWithRange:foundPosiY] doubleValue];
			}
			else requestOK=false;
			if(!requestOK)printf("wrz request fail 3\n");
		}
		if(foundPosiX.location!=NSNotFound){
			foundPosiX.location=foundPosiX.location+foundPosiX.length+NSUInteger(2);
			foundPosiX.length=[requestTruePathResult length]-NSUInteger(1)-foundPosiX.location;
			if(foundPosiX.length>0){
				position_x_tmp =[[requestTruePathResult substringWithRange:foundPosiX] doubleValue];
			}
			else requestOK=false;
			if(!requestOK)printf("wrz request fail 4\n");
		}
		if(direction_x_tmp==0&&direction_y_tmp==0&&position_x_tmp==0&&position_y_tmp==0)requestOK=false;
		
		if(!requestOK)printf("wrz request fail 5\n");
		
		NSLog(@"wrz dy:%f, dx:%f, py:%f, px:%f",direction_y_tmp,direction_x_tmp,position_y_tmp,position_x_tmp);
		
		if(requestOK){
			direction_y=((direction_y_tmp-position_y_tmp)/Height_2D_Map_pixel)*Height_2D_Map_m;
			direction_x=((direction_x_tmp-position_x_tmp)/Width_2D_Map_pixel)*Width_2D_Map_m;
			position_y=(position_y_tmp/Height_2D_Map_pixel)*Height_2D_Map_m;
			position_x=(position_x_tmp/Width_2D_Map_pixel)*Width_2D_Map_m;
		}
		
		
	}
	return requestOK;

}

- (IBAction)correctPath:(UIButton *)sender {
	NSLog(@"correct path!");
	if(vins.hasInitialP0){
		double direction_y=0.0;
		double direction_x=0.0;
		double position_y=0.0;
		double position_x=0.0;
		//从server端获取在2D平面图上的坐标和方位，并转换到vins坐标系中
		if([self requstTruthPos:direction_y requestDx:direction_x requestPy:position_y requestPx:position_x]){
			NSLog(@"wrz02 dy:%f, dx:%f, py:%f, px:%f",direction_y,direction_x,position_y,position_x);
			Vector2f posIn2Dmap(position_x,position_y);
			Vector2f posInVins=vins.Rcw_2DmapTovins*posIn2Dmap+vins.cTwc_2DmapTovins;
			
			vins.curTruthPosIndex=WINDOW_SIZE-1;
			vins.curTruthPos.x()=posInVins.x();
			vins.curTruthPos.y()=posInVins.y();
//			vins.curTruthPos.x()=position_x;
//			vins.curTruthPos.y()=position_y;
			vins.curTruthPos.z()=lateast_P.z();
			vins.curTruthPosIn2Dmap_pixel.x()=position_x/Width_2D_Map_m*Width_2D_Map_pixel;
			vins.curTruthPosIn2Dmap_pixel.y()=position_y/Height_2D_Map_m*Height_2D_Map_pixel;
			vins.correctFlag=true;
			
			printf("wrz02 xvins:%f, yvins:%f, x2D:%f, y2D:%f\n",posInVins.x(),posInVins.y(), posIn2Dmap.x(),posIn2Dmap.y());
			printf("latest_P:%f, %f, %f,direction:%d\n",lateast_P.x(),lateast_P.y(),lateast_P.z(),direction_y);
		}
		else{
			printf("wrz02 request failed\n");
		}
		
		
	}
	else{
		printf("wrz02 no initialP0\n");
	}
	
	
}

- (void) handleLongPress:(UILongPressGestureRecognizer*) recognizer
{
    if (!ui_main)
    {
        
    }
    {
        CGPoint point = [recognizer locationInView:self.view];
        vins.drawresult.locationLongPressX = point.x * 640.0 / imageView.frame.size.width;
        vins.drawresult.locationLongPressY = point.y * 480.0 / imageView.frame.size.height;
        vins.drawresult.longPressFlag = true;
    }
}

- (IBAction)recordButtonPressed:(id)sender {
    if(LOOP_CLOSURE)
    {
        LOOP_CLOSURE = false;
        [_recordButton setTitle:@"ENLOOP" forState:UIControlStateNormal];
    }
    else
    {
        LOOP_CLOSURE = true;
        [_recordButton setTitle:@"UNLOOP" forState:UIControlStateNormal];
    }
    /*
     start_record = !start_record;
     if(start_record)
     {
     start_playback = false;
     [_recordButton setTitle:@"Stop" forState:UIControlStateNormal];
     [saveData start];
     }
     else
     {
     TS(record_imu);
     imuData.header = 0; // as the ending marker
     imuData.acc << 0,0,0;
     imuData.gyr << 0,0,0;
     [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
     [self recordImu];
     TE(record_imu);
     [_recordButton setTitle:@"Record" forState:UIControlStateNormal];
     }
     */
}

- (IBAction)playbackButtonPressed:(id)sender {
    vins.failure_hand = true;
    vins.drawresult.change_color = true;
    vins.drawresult.indexs.push_back(vins.drawresult.pose.size());
    segmentation_index++;
    keyframe_database.max_seg_index++;
    keyframe_database.cur_seg_index = keyframe_database.max_seg_index;
    /*
     start_playback = !start_playback;
     if(start_playback)
     {
     //TS(read_imu);
     NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
     NSString *documentsPath = [paths objectAtIndex:0];
     NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name
     imuReader = [NSData dataWithContentsOfFile:filePath];
     //TE(read_imu);
     start_record = false;
     [_playbackButton setTitle:@"Stop" forState:UIControlStateNormal];
     }
     else
     [_playbackButton setTitle:@"Playback" forState:UIControlStateNormal];
     */
}

/********************************************************************UI Button Controler********************************************************************/


/***********************************************************About record and playback data for debug********************************************************/

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

-(void)saveData{
    while (![[NSThread currentThread] isCancelled])
    {
        @autoreleasepool
        {
            if(!imgDataBuf.empty())
            {
                IMG_DATA tmp_data;
                tmp_data = imgDataBuf.front();
                imgDataBuf.pop();
                [self recordImageTime:tmp_data];
                [self recordImage:tmp_data];
                imageDataIndex++;
                //NSLog(@"record: %lf %lu",tmp_data.header,imageDataIndex);
            }
        }
        [NSThread sleepForTimeInterval:0.04];
    }
}

- (void)tapSaveImageToIphone:(UIImage*)image
{
    UIImageWriteToSavedPhotosAlbum(image, self, @selector(image:didFinishSavingWithError:contextInfo:), nil);
}

- (void)image:(UIImage *)image didFinishSavingWithError:(NSError *)error contextInfo:(void *)contextInfo{
    
    if (error == nil) {
        NSLog(@"save access");
    }else{
        NSLog(@"save failed");
    }
}

- (void)checkDirectoryPath:(unsigned long)index withObject:(NSString*)directoryPath
{
    //delete already exist directory first time
    NSError *error;
    if (index == 0 && [[NSFileManager defaultManager] fileExistsAtPath:directoryPath])	//Does directory exist?
    {
        if (![[NSFileManager defaultManager] removeItemAtPath:directoryPath error:&error])	//Delete it
        {
            NSLog(@"Delete directory error: %@", error);
        }
    }
    
    //creat file directory if it does not exist
    if (![[NSFileManager defaultManager] fileExistsAtPath:directoryPath])
    {
        NSLog(@"directory does not exist");
        if (![[NSFileManager defaultManager] createDirectoryAtPath:directoryPath
                                       withIntermediateDirectories:NO
                                                        attributes:nil
                                                             error:&error])
        {
            NSLog(@"Create directory error: %@", error);
        }
    }
}

- (void)recordImu
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name
    
    [imuDataBuf writeToFile:filePath atomically:YES];
    //[msgData writeToFile:filePath atomically:YES];
}

- (void)recordVins
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"VINS"]; //Add the file name
    
    [vinsDataBuf writeToFile:filePath atomically:YES];
    //[msgData writeToFile:filePath atomically:YES];
}

- (void)recordImageTime:(IMG_DATA&)image_data
{
    double time = image_data.header;
    NSData *msgData = [NSData dataWithBytes:&time length:sizeof(time)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"];; //Get the docs directory
    
    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];
    
    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    [msgData writeToFile:filePath atomically:YES];
}

- (void)recordImage:(IMG_DATA&)image_data
{
    NSData *msgData = UIImagePNGRepresentation(image_data.image);
    //NSData *msgData = [NSData dataWithBytes:&image_data length:sizeof(image_data)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"];; //Get the docs directory
    
    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];
    
    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    [msgData writeToFile:filePath atomically:YES];
}

-(bool)readImageTime:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *file1 = [[NSData alloc] initWithContentsOfFile:filePath];
        if (file1)
        {
            double time;
            [file1 getBytes:&time length:sizeof(time)];
            imgData.header = time;
        }
        file_exist = true;
    }
    else
    {
        file_exist = false;
        //NSLog(@"File does not exist");
    }
    return file_exist;
}

-(bool)readImage:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *pngData = [NSData dataWithContentsOfFile:filePath];
        imgData.image = [UIImage imageWithData:pngData];
        file_exist = true;
    }
    else
    {
        file_exist = false;
        //NSLog(@"File does not exist");
    }
    return file_exist;
}

/**************************************************************About record and playback data for debug**********************************************************/

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
    
#if VINS_FRAMEWORK
    //VINSUnityAPI::TestNativeTexture();
#endif
}

- (void)viewDidDisappear:(BOOL)animated
{
    [super viewDidDisappear:animated];
    if (isCapturing)
    {
        [videoCamera stop];
    }
    [mainLoop cancel];
    [draw cancel];
#ifdef LOOP_CLOSURE
    [loop_thread cancel];
#endif
    
}

-(void)viewDidUnload{
    [motionManager stopAccelerometerUpdates];
    [motionManager stopDeviceMotionUpdates];
    [motionManager stopGyroUpdates];
    [motionManager stopMagnetometerUpdates];
    [super viewDidUnload];
}

- (void)dealloc
{
    videoCamera.delegate = nil;
}

/*
 Check the device
 */
DeviceType deviceName()
{
    struct utsname systemInfo;
    uname(&systemInfo);
    
    NSString *device = [NSString stringWithCString:systemInfo.machine
                                          encoding:NSUTF8StringEncoding];
    DeviceType device_type;
    if(([device compare:@"iPhone9,1"] == NSOrderedSame) ||
       ([device compare:@"iPhone9,3"] == NSOrderedSame))
    {
        printf("Device iPhone7\n");
        device_type = iPhone7;
    }
    else if(([device compare:@"iPhone9,2"] == NSOrderedSame) ||
            ([device compare:@"iPhone9,4"] == NSOrderedSame))
    {
        printf("Device iPhone7 plus\n");
        device_type = iPhone7P;
    }
    else if(([device compare:@"iPhone8,2"] == NSOrderedSame))
    {
        printf("Device iPhone6s plus\n");
        device_type = iPhone6sP;
    }
    else if(([device compare:@"iPhone8,1"] == NSOrderedSame))
    {
        printf("Device iPhone6s\n");
        device_type = iPhone6s;
    }
    else if(([device compare:@"iPad6,3"] == NSOrderedSame)||
            ([device compare:@"iPad6,4"] == NSOrderedSame))
    {
        printf("Device iPad pro 9.7\n");
        device_type = iPadPro97;
    }
    else if(([device compare:@"iPad6,7"] == NSOrderedSame)||
            ([device compare:@"iPad6,8"] == NSOrderedSame))
    {
        printf("Device iPad pro 12.9\n");
        device_type = iPadPro129;
    }
    else
    {
        printf("Device undefine\n");
        device_type = unDefine;
    }
    return device_type;
}

bool iosVersion()
{
    NSComparisonResult order = [[UIDevice currentDevice].systemVersion compare: @"10.2.1" options: NSNumericSearch];
    if (order == NSOrderedSame || order == NSOrderedDescending) {
        printf("system version >= 10.2.1\n");
        return true;
    } else {
        printf("system version < 10.2.1\n");
        return false;
    }
}



@end

/**************************************************************UNITY INTERFACE**********************************************************/
/*
#if VINS_FRAMEWORK
extern "C" {
    // Expose the current position and rotation to Unity.
    // Unity will call this function every frame at 30 FPS.
    // In Unity code, we simply assign
    //   camera.position = new Vector3(x, y, z);
    //   camera.rotation = new Quaternion(qx, qy, qz, qw);
    //
    // TODO:
    // VINS uses a right-handed system, but Unity uses a left-handed system,
    //   as in https://github.com/wandermyz/VINS_Unity_XCode/blob/master/docs/coordinate.png
    // We need to somehow transform the "position" and "rotation" below,
    //   so that the exported x,y,z,qx,qy,qz,qw is in Unity coordinate system.
    void VINSGetCurrentPose(float *x, float *y, float *z, float *qx, float *qy, float *qz, float *qw) {
        static Matrix3d initR;
        initR << 0,  0,  -1.0,
        0.0,  1.0,  0.0,
        1.0,  0.0,  0.0;
 
        if (vins.solver_flag != vins.NON_LINEAR) {
            *x = *y = *z = 0;
            *qx = *qy = *qz = 0;
            *qw = 1;
            return;
        }
        
        if(!vins.drawresult.planeInit)
        {
            Vector3f model;
            vins.drawresult.computeAR(vins.correct_point_cloud, model);
            //model << 1.76, 0, 0;
            cout << "plane for unity: "<< model.transpose() << endl;
            model = Utility::Vins2Unity(model.cast<double>()).cast<float>();
            
            VINSUnityAPI::SetCameraSeeThrough(CAMERA_MODE);
            //VINSUnityAPI::SetModelSpaceTransform(model, Vector3f(0.63, 0.63, 0.63), Quaternionf(1, 0, 0, 0));
            VINSUnityAPI::SetModelSpaceTransform(model, Vector3f(0.03, 0.03, 0.03), Quaternionf(1, 0, 0, 0));
            if(CAMERA_MODE)
            {
                VINSUnityAPI::SetCameraFOV(40.23109);    //iPhone7 camera fov
            }
            else
            {
                VINSUnityAPI::SetCameraFOV(49.7);    //eye fov??? cannot believe, need discussion
            }
            VINSUnityAPI::SetCameraOffset(Vector3f(0, 0, 0));
        }
        
        Vector3d position_vins, position;
        Quaterniond rotation_vins, rotation;
        if(CAMERA_MODE)
        {
            position_vins = lateast_P.cast<double>() + lateast_R.cast<double>() * Vector3d(TIC_X, TIC_Y, TIC_Z);
            rotation_vins = Eigen::Quaterniond(lateast_R.cast<double>()); // Eigen::Quaterniond(vins.Rs[WINDOW_SIZE]);
        }
        else
        {
            position_vins = tmp_P + tmp_Q.toRotationMatrix() * Vector3d(-0.082, 0, 0);  //where TIC should be the extrinsic between glasses center and phone IMU
            rotation_vins = tmp_Q;
            
            //position_vins = Vector3d(0, 0, 0) + Vector3d(-0.082, 0, 0);  //where TIC should be the extrinsic between glasses center and phone IMU
            //Quaterniond q_eye(Utility::ypr2R(Vector3d(0, -90, 0)));
            //rotation_vins = q_eye;
        }
        
        position = Utility::Vins2Unity(position_vins);
        *x = (float)position.x();
        *y = (float)position.y();
        *z = (float)position.z();
        
        rotation = Utility::Vins2Unity(rotation_vins);
        *qx = (float)rotation.coeffs().x();
        *qy = (float)rotation.coeffs().y();
        *qz = (float)rotation.coeffs().z();
        *qw = (float)rotation.coeffs().w();
    }
}


#endif
*/
