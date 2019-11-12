/**
* @file			HumanCounter.cpp
* @brief		Sample program for Hitachi-LG Data Storage (HLDS)'s TOF Motion Sensor
* @author		Hitachi-LG Data Storage, Inc. (HLDS)
* @date			2017.05.19
* @version		v2.1.0
* @copyright	Hitachi-LG Data Storage,Inc.
*
* @par Change History:
* - 2016.06.30 New
* - 2016.08.30 v1.0.2l
*					- Linux Convert
* - 2017.04.28 v1.1.0
*					- Add hand detection function
*					- Add edge noise reduction
*					- Support playing capture data
* - 2018.03.07 v2.0.0
*					- Support vertical position of sensor (HLS-LFOM5 is vertical in its default)
*					- Change rotation order from X-Y-Z to Z-Y-X
*					- Transpose sub display in case of vertical position
*					- Add Enable Area
* - 2018.02.13 v2.1.0
*					- Add close window function
*					- Avoid negative value for angles
*					- Change LowSignalCutoff value from 20 to 10
*					- Add Side/Front View for calibration
*/

#if defined (_WIN64) || defined(_WIN32)
#include <Windows.h>
#endif
#include <opencv2/opencv.hpp>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <time.h>
#if defined (__linux__) || defined(__linux)
#include <unistd.h>
#endif

#include "tof.h"
#include <vector>

#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h> 
#include <stdlib.h>

//TCP communication port
#define PORT 12348

using namespace std;
using namespace hlds;

#define VERSION "Ver.2.0.7"  // antes era 2.0.0

//Compile option
#define HUMAN_COLOR		//Different color for each human

#ifdef ENABLE_CONSOLE_DEBUG_LOG
#define	DOUT std::cout << __FILE__ << ":" << __LINE__ << ":" << __FUNCTION__ << ":" 
#define	WOUT std::wcout
#define	_DOUTFUNC()		DOUT << __FILE__ << ":" << __LINE__ << ":" << __FUNCTION__ << endl
#else
#define	DOUT 0 && std::cout
#define	WOUT 0 && std::wcout
#define	_DOUTFUNC()
#endif // ENABLE_CONSOLE_DEBUG_LOG
#define	CERR std::cerr << __FILE__ << ":" << __LINE__ << ":" << __FUNCTION__  << ":" 

#if defined (_WIN64) || defined(_WIN32)
#define M_PI (3.14159265358979) //Pi
#endif
#define deg2rad(d) ( (d) / 180.0 * M_PI )	//Macro function to convert deg to rad

//sub display
#define SUB_DISPLAY_X			(10)			//X-coordinate of upper left of sub display
#define SUB_DISPLAY_Y			(710)			//Y-coordinate of upper left of sub display
#define SUB_DISPLAY_WIDTH		(320)			//Width of sub display
#define SUB_DISPLAY_HEIGHT		(240)			//Height of sub display

#define ANGLE_ADJUSTMENT_DEGREE		(1)			//Angle ajustment unit(degree)

//Section Display
#define SIDE_VIEW_X				(850)			//X-coordinate of side view
#define SIDE_VIEW_Y				(100)			//Y-coordinate of side view
#define SIDE_VIEW_WIDTH			(400)			//Width of side view
#define SIDE_VIEW_HEIGHT		(300)			//Height of side view
#define FRONT_VIEW_X			(850)			//X-coordinate of front view
#define FRONT_VIEW_Y			(500)			//Y-coordinate of front view
#define FRONT_VIEW_WIDTH		(400)			//Width of front view
#define FRONT_VIEW_HEIGHT		(300)			//Height of front view
#define SIDE_VIEW_RANGE			(5000)			//Range (distance) of side view [mm]
#define FRONT_VIEW_RANGE		(6000)			//Range (width) of front view [mm]
#define SECTION_HEIGHT_MIN		(-500)			//Min height of side/front view [mm]
#define SECTION_HEIGHT_MAX		(2000)			//Max height of side/front view [mm]

#if defined (__linux__) || defined(__linux)		// Debian8
#define	CVWAITKEY_CURSOR_TOP	'u'
#define	CVWAITKEY_CURSOR_BOTTOM	'n'
#define	CVWAITKEY_CURSOR_RIGHT	'k'
#define	CVWAITKEY_CURSOR_LEFT	'j'
//#define	CVWAITKEY_CURSOR_TOP	(65362)
//#define	CVWAITKEY_CURSOR_BOTTOM	(65364)
//#define	CVWAITKEY_CURSOR_RIGHT	(65363)
//#define	CVWAITKEY_CURSOR_LEFT	(65361)
#endif
#if defined (_WIN64) || defined(_WIN32)		// Win
#define	CVWAITKEY_CURSOR_TOP	(2490368)
#define	CVWAITKEY_CURSOR_BOTTOM	(2621440)
#define	CVWAITKEY_CURSOR_RIGHT	(2555904)
#define	CVWAITKEY_CURSOR_LEFT	(2424832)
#endif

//buffer
#define BUFSIZE		(1024)

//Display image
cv::Mat img(480 * 2, 640 * 2, CV_8UC3);

//Background image
cv::Mat back(480 * 2, 640 * 2, CV_8UC3);

//Z-buffer(to understand before and behind)
float z_buffer[640 * 2][480 * 2];

//ini file
string inifilename = "./HumanCounter.ini";
string inisection = "Settings";
#define	DELIM_CHAR	'='

//Save file name
string savefile;	//Image save file

//file names to read
std::vector<std::string> directories ;
int decision = 0; //user decision
int solid = 0; //volume decision
int auxi1 = 0; //volume algorithm decision

//tracks the number of saved files
int saved_cube = 0 ;
int saved_sphere = 0 ;
int saved_pyramid = 0 ;
int saved_paralellipiped = 0 ;
int saved_cylinder = 0 ;

//pcl imports
#include <iostream>
#include <cmath>
#include <vector>
#include <numeric>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

//volume variables
int formula = 0;
float comprimento = 0;
float largura = 0;
float altura = 0;
float AB = 0;
float diametro = 0;
pcl::PointXYZ the_point ;
float volume = 0;
int VolumeType= 0;

//analysis area
int aa_height =0;
int aa_width =0;

//remove point variables
int Ktype = 0;
int MeanKtype = 0;
int removeType = 0;

//visualization
bool mesh = false;

//Initial settings
float angle_x = 90.0f;				//Angle to rotate around X-axis(degree)
float angle_y = 0.0f;				//Angle to rotate around Y-axis(degree)
float angle_z = 0.0f;				//Angle to rotate around Z-axis(degree)
float height = 1000.0f;				//Height from floor(mm)
float dx = 600.0f;					//Shift length to x-axis positive direction(mm)
float dy = 900.0f;					//Shift length to y-axis positive direction(mm)
float zoom = 0.12f;					//Zoom ratio
bool bPoint = true;					//Mode to display points
bool bBack = false;					//Mode to display footprint on background
bool bSubDisplay = true;			//Mode to display sub display
bool bTest = false;				// Mode to test codes
bool bScan = false;				// liberta a captura da pcl
bool bAnalysis = false;				// obtem a point cloud para analisar
bool bArea = false;				// desenha quadrado de analise
bool bClassification = false; 			// permite que se realize a classificacao
bool fDecision = false;				// mostra os resultados finais
bool fAlgorithm	= true;				// decides volume algorithm
bool bFixed = true;				// decides fixed analysis area or not
bool bManual = true;				// decides classification or manual
std::string final_cls1 = " ";			// classification result
float final_prob1 = 0;		// probability result
std::string final_cls2 = " ";			// classification result
float final_prob2 = 0;		// probability result
bool bAdjust = false;			//ajusts analysis area

/* Win32 API Convert */
#if defined (__linux__) || defined(__linux)
typedef unsigned long	DWORD;
typedef unsigned short	WORD;
typedef unsigned char	BYTE;
#define	TRUE	true
#define	FALSE	false
#define	BOOL	bool
#define	os_sleep(intvl)	usleep(intvl*1000);

#define	INIFILE_HEADER	"[Settings]"
static vector<string> split(const string& stLine, char delim);
static bool _WritePrivateProfileString(string stAppName, string stKeyName, string stSetString, string stFileName );
static string _GetPrivateProfileString(string stAppName, string stKeyName, string stDefault, unsigned long* rSize, string stFileName );

static vector<string> split(const string& stLine, char delim)
{
	stringstream ssBuf(stLine);
	string stBuf;
	vector<string> vResults;

	while(getline(ssBuf, stBuf, delim)) {
		vResults.push_back(stBuf);
	}

	return vResults;
}

static bool _WritePrivateProfileString(string stAppName, string stKeyName, string stSetString, string stFileName )
{
	bool ret = true;
	bool isChange = false;
	bool isFound = false;
	bool isNewCreate = false;

	_DOUTFUNC();

	ifstream ifs;

	ifs.open(stFileName, ios::in);;
	if ( ifs.rdstate() != 0 ) {
		if ( ifs.fail() ) {
			isNewCreate = true;
		} else {
			CERR << "Error:ifs.rdstate()=" << ifs.rdstate() << endl;
			ret = false;
			return ret;
		}
	}

	vector<string> vsFileStream;

	if ( !isNewCreate ) {
		string stLine;
		while(getline(ifs, stLine)) {
			vector<string> ss;
			ss = split(stLine, DELIM_CHAR);
			string sa;
			if (ss.size() == 1) {	// not ini file element.
				sa = ss[0];
			} else
			if (ss.size() == 2) {	// ini file element.
				if ( ss[0] == stKeyName ) {
					isFound = true;
					if (ss[1] != stSetString ) {
						sa = ss[0] + "=" + stSetString;
						isChange = true;
					}
				} else {
					sa = ss[0] + "=" + ss[1];
				}
			} else {				// ignore
				continue;
			}
			vsFileStream.push_back(sa);
		}

		ifs.close();
	}

	if ( isNewCreate || !isFound ) {
		if ( isNewCreate ) {
			vsFileStream.push_back(string(INIFILE_HEADER));
		}
		string sa = stKeyName + "=" + stSetString;
		vsFileStream.push_back(sa);
	} else 
	if ( !isNewCreate && !isChange ) {
		return ret;
	}

	ofstream ofs;

	ofs.open(stFileName, ios::out);
	if ( ofs.rdstate() != 0 ) {
		CERR << "Error:ofs.rdstate()=" << ofs.rdstate() << endl;
		ret = false;
		return ret;
	}

	auto itr = vsFileStream.begin();
	while(itr != vsFileStream.end()) {
		ofs << *itr <<  endl;
		++itr;
	}

	ofs.close();

	_DOUTFUNC();

	return ret;
}

static string _GetPrivateProfileString(string stAppName, string stKeyName, string stDefault, unsigned long* rSize, string stFileName )
{
	string ReturnedString;
	*rSize = 0;

	_DOUTFUNC();

	ifstream ifs;

	ifs.open(stFileName, ios::in);;
	if ( ifs.rdstate() != 0 ) {
		CERR << "Error:ifs.rdstate()=" << ifs.rdstate() << endl;
		return ReturnedString;
	}

	string stLine;
	while(getline(ifs, stLine)) {
		vector<string> ss;
		ss = split(stLine, DELIM_CHAR);
		if (ss.size() != 2) continue;	// not ini file element.

		if ( ss[0] == stKeyName ) {
			ReturnedString = ss[1];
			*rSize = ss[1].size();
		}
	}

	ifs.close();

	_DOUTFUNC();

	return ReturnedString;
}
#endif	//_LINUX
#if defined (_WIN64) || defined(_WIN32)		// Win
#define	os_sleep(intvl)	Sleep(intvl);
#endif

//Save ini file
bool SaveIniFile(void)
{
	bool ret = TRUE;
	string strBuffer;

	_DOUTFUNC();

	try {
		ret = _WritePrivateProfileString(inisection, "ANGLE_X", to_string(angle_x), inifilename);
		if (ret != TRUE){
			throw -1;
		}

		ret = _WritePrivateProfileString(inisection, "ANGLE_Y", to_string(angle_y), inifilename);
		if (ret != TRUE){
			throw -2;
		}

		ret = _WritePrivateProfileString(inisection, "ANGLE_Z", to_string(angle_z), inifilename);
		if (ret != TRUE){
			throw -3;
		}

		ret = _WritePrivateProfileString(inisection, "SHIFT_X", to_string(dx), inifilename);
		if (ret != TRUE){
			throw -4;
		}

		ret = _WritePrivateProfileString(inisection, "SHIFT_Y", to_string(dy), inifilename);
		if (ret != TRUE){
			throw -5;
		}

		ret = _WritePrivateProfileString(inisection, "HEIGHT", to_string(height), inifilename);
		if (ret != TRUE){
			throw -6;
		}

		ret = _WritePrivateProfileString(inisection, "ZOOM", to_string(zoom), inifilename);
		if (ret != TRUE){
			throw -7;
		}

	}
	catch(int errExcp) {
		cerr << __FUNCTION__ << ":error:" << errExcp << endl;
	}

	_DOUTFUNC();

	return true;
}

//Load ini file
bool LoadIniFile(void){

	DWORD ret = 0;
	string stBuffer;

	_DOUTFUNC();

	stBuffer = _GetPrivateProfileString(inisection, "ANGLE_X", to_string(angle_x), &ret, inifilename);
	if (ret != 0){
		angle_x = stof(stBuffer);
	}

	stBuffer = _GetPrivateProfileString(inisection, "ANGLE_Y", to_string(angle_y), &ret, inifilename);
	if (ret != 0){
		angle_y = stof(stBuffer);
	}

	stBuffer = _GetPrivateProfileString(inisection, "ANGLE_Z", to_string(angle_z), &ret, inifilename);
	if (ret != 0){
		angle_z = stof(stBuffer);
	}

	stBuffer = _GetPrivateProfileString(inisection, "HEIGHT", to_string(height), &ret, inifilename);
	if (ret != 0){
		height = stof(stBuffer);
	}

	stBuffer = _GetPrivateProfileString(inisection, "SHIFT_X", to_string(dx), &ret, inifilename);
	if (ret != 0){
		dx = stof(stBuffer);
	}

	stBuffer = _GetPrivateProfileString(inisection, "SHIFT_Y", to_string(dy), &ret, inifilename);
	if (ret != 0){
		dy = stof(stBuffer);
	}

	stBuffer = _GetPrivateProfileString(inisection, "ZOOM", to_string(zoom), &ret, inifilename);
	if (ret != 0){
		zoom = stof(stBuffer);
	}

	_DOUTFUNC();

	return true;
}

void DrawSection(Frame3d* pframe3d)
{
	cv::rectangle(img, cv::Point(SIDE_VIEW_X, SIDE_VIEW_Y),
		cv::Point(SIDE_VIEW_X + SIDE_VIEW_WIDTH, SIDE_VIEW_Y + SIDE_VIEW_HEIGHT), cv::Scalar(0, 0, 0), -1);

	cv::rectangle(img, cv::Point(FRONT_VIEW_X, FRONT_VIEW_Y),
		cv::Point(FRONT_VIEW_X + FRONT_VIEW_WIDTH, FRONT_VIEW_Y + FRONT_VIEW_HEIGHT), cv::Scalar(0, 0, 0), -1);

	cv::Vec3b v;
	v.val[0] = 255;
	v.val[1] = 255;
	v.val[2] = 255;

	for (int y = 0; y < pframe3d->height; y++){
		for (int x = 0; x < pframe3d->width; x++){

			TofPoint p = pframe3d->frame3d[y * pframe3d->width + x];
			int h = height - (int)p.z;
			if ((h >= SECTION_HEIGHT_MIN) && (h <= SECTION_HEIGHT_MAX)){
				//In range of height

				int px = (int)p.x;
				int py = (int)p.y;
				if (py < 0){
					py *= -1;
				}

				//Side view
				int sdx = (SIDE_VIEW_RANGE - py) * SIDE_VIEW_WIDTH / SIDE_VIEW_RANGE;
				int sdy = ((SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN) - (h - SECTION_HEIGHT_MIN)) * SIDE_VIEW_HEIGHT / (SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN);
				if ((sdx >= 0) && (sdx < SIDE_VIEW_WIDTH) &&
					(sdy >= 0) && (sdy < SIDE_VIEW_HEIGHT)){
					img.at<cv::Vec3b>(sdy + SIDE_VIEW_Y, sdx + SIDE_VIEW_X) = v;
				}

				//Front view
				int fdx = (FRONT_VIEW_RANGE / 2 + px) * FRONT_VIEW_WIDTH / FRONT_VIEW_RANGE;
				int fdy = ((SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN) - (h - SECTION_HEIGHT_MIN)) * FRONT_VIEW_HEIGHT / (SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN);
				if ((fdx >= 0) && (fdx < SIDE_VIEW_WIDTH) &&
					(fdy >= 0) && (fdy < SIDE_VIEW_HEIGHT)){
					img.at<cv::Vec3b>(fdy + FRONT_VIEW_Y, fdx + FRONT_VIEW_X) = v;
				}
			}
		}
	}

	cv::Point p0;
	cv::Point p1;

	//Ruled line
	for (int i = 1; i < 4; i++){
		p0.x = SIDE_VIEW_X;
		p0.y = SIDE_VIEW_HEIGHT * i / 4 + SIDE_VIEW_Y;
		p1.x = SIDE_VIEW_X + SIDE_VIEW_WIDTH;
		p1.y = p0.y;
		cv::line(img, p0, p1, cv::Scalar(128, 128, 128), 1, CV_AA, 0);

		p0.x = SIDE_VIEW_WIDTH * i / 4 + SIDE_VIEW_X;
		p0.y = SIDE_VIEW_Y;
		p1.x = p0.x;
		p1.y = SIDE_VIEW_Y + SIDE_VIEW_HEIGHT;
		cv::line(img, p0, p1, cv::Scalar(128, 128, 128), 1, CV_AA, 0);

		p0.x = FRONT_VIEW_X;
		p0.y = FRONT_VIEW_HEIGHT * i / 4 + FRONT_VIEW_Y;
		p1.x = FRONT_VIEW_X + FRONT_VIEW_WIDTH;
		p1.y = p0.y;
		cv::line(img, p0, p1, cv::Scalar(128, 128, 128), 1, CV_AA, 0);

		p0.x = FRONT_VIEW_WIDTH * i / 4 + FRONT_VIEW_X;
		p0.y = FRONT_VIEW_Y;
		p1.x = p0.x;
		p1.y = FRONT_VIEW_Y + FRONT_VIEW_HEIGHT;
		cv::line(img, p0, p1, cv::Scalar(128, 128, 128), 1, CV_AA, 0);
	}

	//Ground line
	p0.x = SIDE_VIEW_X;
	p0.y = SIDE_VIEW_HEIGHT - SIDE_VIEW_HEIGHT * (0 - SECTION_HEIGHT_MIN) / (SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN) + SIDE_VIEW_Y;
	p1.x = SIDE_VIEW_X + SIDE_VIEW_WIDTH;
	p1.y = p0.y;
	cv::line(img, p0, p1, cv::Scalar(0, 255, 255), 1, CV_AA, 0);

	cv::putText(img, "Height 0[mm]", cv::Point(SIDE_VIEW_X + 5, p1.y + 18),
		cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 255, 255), 1, CV_AA);

	p0.x = FRONT_VIEW_X;
	p0.y = FRONT_VIEW_HEIGHT - FRONT_VIEW_HEIGHT * (0 - SECTION_HEIGHT_MIN) / (SECTION_HEIGHT_MAX - SECTION_HEIGHT_MIN) + FRONT_VIEW_Y;
	p1.x = FRONT_VIEW_X + FRONT_VIEW_WIDTH;
	p1.y = p0.y;
	cv::line(img, p0, p1, cv::Scalar(0, 255, 255), 1, CV_AA, 0);

	cv::putText(img, "Height 0[mm]", cv::Point(FRONT_VIEW_X + 5, p1.y + 18),
		cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 255, 255), 1, CV_AA);

	cv::putText(img, "Side View", cv::Point(SIDE_VIEW_X + 5, SIDE_VIEW_Y + SIDE_VIEW_HEIGHT - 5),
		cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255, 0, 0), 1, CV_AA);

	cv::putText(img, "Front View", cv::Point(FRONT_VIEW_X + 5, FRONT_VIEW_Y + FRONT_VIEW_HEIGHT - 5),
		cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255, 0, 0), 1, CV_AA);

	cv::rectangle(img, cv::Point(SIDE_VIEW_X, SIDE_VIEW_Y),
		cv::Point(SIDE_VIEW_X + SIDE_VIEW_WIDTH, SIDE_VIEW_Y + SIDE_VIEW_HEIGHT), cv::Scalar(255, 0, 0), 2);

	cv::rectangle(img, cv::Point(FRONT_VIEW_X, FRONT_VIEW_Y),
		cv::Point(FRONT_VIEW_X + FRONT_VIEW_WIDTH, FRONT_VIEW_Y + FRONT_VIEW_HEIGHT), cv::Scalar(255, 0, 0), 2);
}

// dimension and place of analysis square
cv::Point point1;
cv::Point point2;
cv::Point point3;
cv::Point point4;
cv::Point point5;
cv::Point point6;
		
void callback(int event, int x, int y, int flags, void* userdata){

	if (event == cv::EVENT_LBUTTONDOWN){
		bScan = true;
		//estamos a clicar dentro do ecra
		fDecision = false;
		
		if(x > 60 && y > 60 ){
			if(x < 1180 && y < 900){
				
				std::cout << "Allowed !"<<std::endl;
				
				if(!bFixed){
					// Fixed analysis			
					point1.x = x - 30;
					point1.y = y - 30;
					point2.x = x + 30;
					point2.y = y + 30;
					point3.x = point1.x;
					point3.y = point1.y;
					point4.x = point2.x;
					point4.y = point2.y;
				
/*
					//retira o zoom e o shift dos limites
					point3.x = point1.x;
					point3.y = point1.y;
					point3.x -= dx;
					point3.y -= dy;
			
					point3.x /= zoom;
					point3.y /= zoom;

					point4.x = point2.x;
					point4.y = point2.y;
					point4.x -= dx;
					point4.y -= dy;

					point4.x /= zoom;
					point4.y /= zoom;*/	
				
				}
				else{	
					// Fixed analysis			
					point1.x = 747 - 30;
					point1.y = 626 - 30;
					point2.x = 747 + 30;
					point2.y = 626 + 30;
				}

       			}
		}
	}		
}

//Save screen when f key is pushed
bool SaveFile(void){

	//Make file name with current time
	char buff[16];

	_DOUTFUNC();

	time_t now = time(NULL);
	struct tm *pnow = localtime(&now);
	sprintf(buff, "%04d%02d%02d%02d%02d%02d",
		pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday,
		pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
	savefile = buff;
	savefile += ".png";

	_DOUTFUNC();

	//Save
	return cv::imwrite(savefile, img);
}

bool ChangeAttribute(Tof& tof, float x, float y, float z, float rx, float ry, float rz)
{
	_DOUTFUNC();

	if (rx < 0) rx += 360;
	if (ry < 0) ry += 360;
	if (rz < 0) rz += 360;

	if (tof.SetAttribute(x, y, z, rx, ry, rz) != Result::OK){
		return false;
	}

	_DOUTFUNC();

	return true;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////Remove points functions/////////////////////////////
///////////////////////////////////////////////////////////////////////////////


////////////////////////////
// Segments the point cloud to the analysis square
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr cuttingCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int xmin, const int xmax, const int ymin, const int ymax ){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the x layer condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, xmin))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, xmax))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);

	// build the y condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condy (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condy->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, ymin))); //Greather than
	range_condy->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, ymax))); //Less than

	// build the y filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremy;
	condremy.setCondition (range_condy);
	condremy.setInputCloud (cloud_filtered);
	condremy.setKeepOrganized(true);
	condremy.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	std::cerr << "1 - Cloud before filtering: " << cloud->width << " " <<std::endl;
	std::cerr << "2 - Points filtered: " << num << " " <<std::endl;
	std::cerr << "3 - Cloud after filtering: " << cloud_filtered->width << " " <<std::endl;

	//creates a new point cloud to store the filtered pcl

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width    = cloud_filtered->width;
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	int num2= 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		if(!isnan(cloud_filtered->points[i].x)){
		cloud_final->points[num2].x = cloud_filtered->points[i].x;
		cloud_final->points[num2].y = cloud_filtered->points[i].y;
		cloud_final->points[num2].z = cloud_filtered->points[i].z;
		num2 ++;
		}		
	
	}

	return cloud_final;

}

////////////////////////////
//K - Nearest Neighbour to remove furthest points
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  segmentCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ centre){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	//creating octree
	float resolution = 128.0f;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

	octree.setInputCloud (cloud);
	octree.addPointsFromInputCloud ();

	// K nearest neighbor search

	int K = Ktype;

	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	int num2=0;

	if (octree.nearestKSearch (centre, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){

		cloud_final->width = pointIdxNKNSearch.size();
		cloud_final->height = 1;
		cloud_final->is_dense = false;
		cloud_final->points.resize(cloud_final->width * cloud_final->height);

		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
			cloud_final->points[num2].x = cloud->points[ pointIdxNKNSearch[i] ].x ;
			cloud_final->points[num2].y = cloud->points[ pointIdxNKNSearch[i] ].y ;
			cloud_final->points[num2].z = cloud->points[ pointIdxNKNSearch[i] ].z ;
			num2++;
		}

	}

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_final);
	sor.setMeanK (MeanKtype);
	sor.setStddevMulThresh (1.25);
	sor.filter (*cloud_filtered);

	//outliers
  	//sor.setNegative (true);
  	//sor.filter (*cloud_filtered);

	std::cerr << " Final cloud: " << cloud_filtered->points.size() << " points " <<std::endl;

	return cloud_filtered;
} 

////////////////////////////
//Removes planes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  RandomSampleConsensus(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (15);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

	//obtaining outliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr the_inliers (new pcl::PointIndices());
	the_inliers->indices = inliers;

	extract.setInputCloud(cloud);
	extract.setIndices(the_inliers);
	extract.setNegative(true);
	extract.filter(*cloud_final);

	return cloud_final;
} 

////////////////////////////
//Statistical outlier removal
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  FinalCleaning(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);	

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_final);
	sor.setMeanK (MeanKtype); //aumentar retira mais pontos
	sor.setStddevMulThresh (1.25); 
	sor.filter (*cloud_filtered);

	//outliers
  	//sor.setNegative (true);
  	//sor.filter (*cloud_filtered);

	std::cerr << " Final cloud: " << cloud_filtered->points.size() << " points " <<std::endl;

	return cloud_filtered;
}

////////////////////////////
// Discovers the centre of the point cloud
////////////////////////////
pcl::PointXYZ meanPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	
	float total_x = 0;
	float total_y = 0;
	float total_z = 0;
	float mean_x = 0;
	float mean_y = 0;
	float mean_z = 0;
	float total_points = cloud->points.size ();

	for (size_t i = 0; i < cloud->points.size (); ++i){
		
		total_x += cloud->points[i].x;
		total_y += cloud->points[i].y;
		total_z += cloud->points[i].z;		
	}

	mean_x = total_x / total_points ;
	mean_y = total_y / total_points ;
	mean_z = total_z / total_points ;

	//creating the centre point
	pcl::PointXYZ centre (mean_x,mean_y,mean_z) ;
	//std::cout << "Centre x- " << centre.x << " y- " << centre.y << " z- " << centre.z << ""<<std::endl;

	return centre;
}

////////////////////////////
// Discovers the highest point
////////////////////////////
pcl::PointXYZ highestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	float maxy = cloud->points[0].y;
	float valy = 0;

	for (size_t i = 0; i < cloud->points.size (); ++i){

		//max		
		if( maxy < cloud->points[i].y ){
			maxy =cloud->points[i].y;
			valy = i;
		}

	}

	//creating the highest point
	pcl::PointXYZ searchPoint (cloud->points[valy].x,cloud->points[valy].y,cloud->points[valy].z) ;
	return searchPoint;
}

////////////////////////////
// measures distance between two points
////////////////////////////
float squaredEuclideanDistance (const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){

	float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y, diff_z = p2.z - p1.z;
	return (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
}

////////////////////////////
// Detects circular shapes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr circularCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr    model_s (new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
	ransac.setDistanceThreshold (15);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

	//obtaining outliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr the_inliers (new pcl::PointIndices());
	the_inliers->indices = inliers;

	extract.setInputCloud(cloud);
	extract.setIndices(the_inliers);
	extract.setNegative(true);
	extract.filter(*cloud_final);


	return cloud_final;

}

////////////////////////////
// Detects circular shapes with a wider threshold
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr circularCloud2(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr    model_s (new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
	ransac.setDistanceThreshold (25);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

	//obtaining outliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr the_inliers (new pcl::PointIndices());
	the_inliers->indices = inliers;

	extract.setInputCloud(cloud);
	extract.setIndices(the_inliers);
	extract.setNegative(true);
	extract.filter(*cloud_final);


	return cloud_final;

}

////////////////////////////
// Detects circular shapes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr circularCloud3(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr    model_s (new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
	ransac.setDistanceThreshold (10);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

	//obtaining outliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr the_inliers (new pcl::PointIndices());
	the_inliers->indices = inliers;

	extract.setInputCloud(cloud);
	extract.setIndices(the_inliers);
	extract.setNegative(true);
	extract.filter(*cloud_final);


	return cloud_final;

}


////////////////////////////
// Removes borders - furthest points from the centre
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int points){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	// measuring the centre of the point cloud
	float total_x = 0;
	float total_y = 0;
	float total_z = 0;
	float mean_x = 0;
	float mean_y = 0;
	float mean_z = 0;
	float total_points = cloud->points.size ();

	for (size_t i = 0; i < cloud->points.size (); ++i){
		
		total_x += cloud->points[i].x;
		total_y += cloud->points[i].y;
		total_z += cloud->points[i].z;		
	}

	mean_x = total_x / total_points ;
	mean_y = total_y / total_points ;
	mean_z = total_z / total_points ;

	//creating the centre point
	pcl::PointXYZ centre (mean_x,mean_y,mean_z) ;

	//saving all the distances to the centre
	std::vector<float> distances;
	float distance = 0;
	for (size_t i = 0; i < cloud->points.size (); ++i){
		

		distance = squaredEuclideanDistance( cloud->points[i], centre);
		distances.push_back(distance);

	}

	//discovering the farthest points until the cloud as only 2048 points

	int removing = points;
	int detected = 0;

	std::vector<int> remove_points ;
	int farthest_value = distances[0];  //minimum value is the firts known point
	int farthest_point = 0;

	int mean_distance1 = 0;
	int mean_distance = 0;

	for (size_t i = 0; i < distances.size (); ++i){

		mean_distance1 += distances[i];
	}

	mean_distance = mean_distance1 / distances.size();

	while(detected != removing){

		for (size_t i = 0; i < distances.size (); ++i){

			if(distances[i] > farthest_value){
				farthest_value = distances[i];
				farthest_point = i;
			}
		}

		// keep the index of the point to remove and removes the position of other distances 
		distances[farthest_point] = mean_distance; //with erase, the vector would substitute the deleted position by the deleted position +1
		detected++;
		remove_points.push_back(farthest_point);
		farthest_point = 0;
		farthest_value = distances[0];

	}

	// create new point cloud without the farthest points
	int newc = cloud->points.size() - remove_points.size();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width = newc;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);

	// auxiliar variables
	int position = 0; 
	bool aux = false; 
	int num= 0; 
	std::vector<int>::iterator it;
	for (size_t i = 0; i < cloud->points.size (); ++i){

		for (size_t j = 0; j < remove_points.size (); ++j){

			if (i == remove_points[j]){
				aux = false;
				break;
			}
			
			else
				aux = true;
		}
	
		// caso seja igual ao fim da lista, esse ponto nao pertence a lista de remover
		if(aux && num <newc){	
			cloud_final->points[num].x = cloud->points[i].x;
			cloud_final->points[num].y = cloud->points[i].y;
			cloud_final->points[num].z = cloud->points[i].z;
			num ++;
			aux = false;
		}
	}

	std::cout << "Final cloud: " << cloud_final->points.size() << " points !"<<std::endl;

	return cloud_final;

}


///////////////////////////////////////////////////////////////////////////////
////////////////////////////////Volume functions///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////////////////////
//Discovers the max and min values for each coordinates
////////////////////////////
std::vector<float> maxmin(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	std::vector<float> values;

	float maxx = cloud->points[0].x;
	float Mvalx = 0;
	float maxy = cloud->points[0].y;
	float Mvaly = 0;
	float maxz = cloud->points[0].z;
	float Mvalz = 0;

	float minx = cloud->points[0].x;
	float mvalx = 0;
	float miny = cloud->points[0].y;
	float mvaly = 0;
	float minz = cloud->points[0].z;
	float mvalz = 0;

	for (size_t i = 0; i < cloud->points.size (); ++i){

		//max		
		if( maxx < cloud->points[i].x ){
			maxx =cloud->points[i].x;
			Mvalx = i;
		}

		if( maxy < cloud->points[i].y ){
			maxy =cloud->points[i].y;
			Mvaly = i;
		}

		if( maxz < cloud->points[i].z ){
			maxz =cloud->points[i].z;
			Mvalz = i;
		}

		//min
		if( minx > cloud->points[i].x ){
			minx =cloud->points[i].x;
			mvalx = i;
		}

		if( miny > cloud->points[i].y){
			miny =cloud->points[i].y;
			mvaly = i;
		}

		if( minz > cloud->points[i].z ){
			minz =cloud->points[i].z;
			mvalz = i;
		}

	}

	//Saves the values in the list
	values.push_back(Mvalx);
	values.push_back(mvalx);
	values.push_back(Mvaly);
	values.push_back(mvaly);
	values.push_back(Mvalz);
	values.push_back(mvalz);

	return values;

}

////////////////////////////
// discovers the maximum and minimum points like puting an axis
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr valuePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);	 
	cloud_final->width = 7;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);
	std::vector<float> maximus;

	maximus = maxmin(cloud);

	float centrez1 = cloud->points[maximus[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud->points[maximus[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;
	
	//min plus half the distance
	float pointx = cloud->points[maximus[5]].x;
	float pointy = cloud->points[maximus[5]].y ;
	float pointz = cloud->points[maximus[5]].z ;


	//central point to start the scan
	pcl::PointXYZ the_point1 (pointx,pointy,pointz) ;
	the_point = the_point1;
	//std:cerr << the_point << std::endl;

	for(size_t i =0; i< cloud_final->points.size(); ++i){

		if(i<6){
			
			cloud_final->points[i].x = cloud->points[maximus[i]].x;
			cloud_final->points[i].y = cloud->points[maximus[i]].y;
			cloud_final->points[i].z = cloud->points[maximus[i]].z;
			
		}
		else
			cloud_final->points[i] = the_point; //central point

		//std::cerr <<  ""<< i << " x- "<< cloud->points[maximus[i]].x << " y- "<< cloud->points[maximus[i]].y << " z- "<< cloud->points[maximus[i]].z <<  std::endl;
	}

	return cloud_final;
}

//Parallelepiped

////////////////////////////
//Removes planes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  BigplaneRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the z condition -> comprimento
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condz (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, the_point.z - 30))); //Greather than
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, the_point.z + 30))); //Less than

	// build the z filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremz;
	condremz.setCondition (range_condz);
	condremz.setInputCloud (cloud);
	condremz.setKeepOrganized(true);
	condremz.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	int num2= 0;

	//Creates a new point cloud to save the filtered pcl
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width    = cloud_filtered->width;
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		if(!isnan(cloud_filtered->points[i].x)){
			cloud_final->points[num2].x = cloud_filtered->points[i].x;
			cloud_final->points[num2].y = cloud_filtered->points[i].y;
			cloud_final->points[num2].z = cloud_filtered->points[i].z;
			num2 ++;
		}		
	
	}

	std::vector<float> measures = maxmin(cloud_final);
	//distance between max and min
	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	float centrey1 = cloud_final->points[measures[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud_final->points[measures[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;

	largura = centrey /10; //to cm
	comprimento = centrex /10; //to cm

	std::cerr << "comprimento result: " << comprimento << std::endl;
	std::cerr << "largura result: " << largura << std::endl;

	return cloud_final;
} 

////////////////////////////
// Segments the cloud to the object to be scanned -- based on height position z
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr zsliceCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the x layer condition - >largura
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, the_point.x - 5))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, the_point.x + 5))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	//Creates a new point cloud to save the filtered pcl
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width    = cloud_filtered->width;
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	int num2= 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		if(!isnan(cloud_filtered->points[i].x)){
			cloud_final->points[num2].x = cloud_filtered->points[i].x;
			cloud_final->points[num2].y = cloud_filtered->points[i].y;
			cloud_final->points[num2].z = cloud_filtered->points[i].z;
			num2 ++;
		}		
	
	}

	std::vector<float> measures = maxmin(cloud_final);

	//distance between max and min
	float centrez1 = cloud_final->points[measures[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud_final->points[measures[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;

	
	altura = centrez/10; // to cm
	std::cerr << "altura result: " << altura << std::endl;

	return cloud_final;

}

////////////////////////////
//Removes planes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  planeRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (10);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);
	
	//mean value and values below
	float total_x = 0;
	float total_y = 0;
	float total_z = 0;
	float mean_x = 0;
	float mean_y = 0;
	float mean_z = 0;
	float total_points = cloud_final->points.size ();

	for (size_t i = 0; i < cloud_final->points.size (); ++i){
		
		total_x += cloud_final->points[i].x;
		total_y += cloud_final->points[i].y;
		total_z += cloud_final->points[i].z;		
	}

	mean_x = total_x / total_points ;
	mean_y = total_y / total_points ;
	mean_z = total_z / total_points ;

	std::vector<float> measures = maxmin(cloud_final);
	//distance between max and min
	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	float centrey1 = cloud_final->points[measures[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud_final->points[measures[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;

	float centrez1 = cloud_final->points[measures[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud_final->points[measures[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;

	//in case it is in a different positions, the plane is not the top
	if(centrex > centrez)
		largura = centrex/10;
	else
		largura = centrez/10;

	if(centrey > centrez)
		comprimento = centrey/10;
	else
		comprimento = centrez/10;

	//std::cerr << "zresult: " << centrez << std::endl;
	std::cerr << "comprimento result: " << comprimento << std::endl;
	std::cerr << "largura result: " << largura << std::endl;

	return cloud_final;
} 



// Cube

////////////////////////////
//Removes planes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  cubePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	// for the height
	
	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;
	
	std::vector<float> measure = maxmin(cloud);

	//distance between max and min
	
	float centrez1 = cloud->points[measure[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud->points[measure[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);	 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	//std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;
	
	//mean height and values below
	float total_x = 0;
	float total_y = 0;
	float total_z = 0;
	float mean_x = 0;
	float mean_y = 0;
	float mean_z = 0;
	float total_points = cloud->points.size ();

	for (size_t i = 0; i < cloud->points.size (); ++i){
		
		total_x += cloud->points[i].x;
		total_y += cloud->points[i].y;
		total_z += cloud->points[i].z;		
	}

	mean_x = total_x / total_points ;
	mean_y = total_y / total_points ;
	mean_z = total_z / total_points ;

	// build the z condition -> comprimento
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condz (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, mean_z - 30))); //Greather than
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, mean_z ))); //Less than

	// build the z filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremz;
	condremz.setCondition (range_condz);
	condremz.setInputCloud (cloud);
	condremz.setKeepOrganized(true);
	condremz.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	std::cerr << "1 - Cloud before filtering: " << cloud->width << " " <<std::endl;
	std::cerr << "2 - Points filtered: " << num << " " <<std::endl;
	std::cerr << "3 - Cloud after filtering: " << cloud_filtered->width << " " <<std::endl;


	//Creates a new point cloud to save the filtered pcl
	cloud_final->width    = cloud_filtered->width;
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	int num2= 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		if(!isnan(cloud_filtered->points[i].x)){
		cloud_final->points[num2].x = cloud_filtered->points[i].x;
		cloud_final->points[num2].y = cloud_filtered->points[i].y;
		cloud_final->points[num2].z = cloud_filtered->points[i].z;
		num2 ++;
		}		
	
	}

	std::vector<float> measures = maxmin(cloud_final);
	//distance between max and min
	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	float centrey1 = cloud_final->points[measures[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud_final->points[measures[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;


	//measure with the middle value
	if(centrey > centrex){
		if(centrex > centrez )
			largura = centrex/10;
		else{
			if(centrez > centrey )
				largura = centrey/10;
			else
				largura = centrez/10;
		}
	}
	else{
		if(centrey > centrez )
			largura = centrey/10;
		else{
			if(centrez > centrex )
				largura = centrex/10;
			else
				largura = centrez/10;
		}
	}

	std::cerr << "xresult: " << centrex/10 << std::endl;
	std::cerr << "yresult: " << centrey/10 << std::endl;
	std::cerr << "zresult: " << centrez/10 << std::endl;
	//std::cerr << "largura result: " << largura << std::endl;

	return cloud_final;
} 

//Sphere

////////////////////////////
// discovers the maximum and minimum point
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr spherePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width = 7;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);
	std::vector<float> maximus;

	maximus = maxmin(cloud);

	//distance between max and min
	float centrex1 = cloud->points[maximus[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud->points[maximus[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	float centrey1 = cloud->points[maximus[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud->points[maximus[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;
	
	float centrez1 = cloud->points[maximus[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud->points[maximus[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;

	
	//min plus half the distance
	float pointx = cloud->points[maximus[5]].x ;
	float pointy = cloud->points[maximus[5]].y ;
	float pointz = cloud->points[maximus[4]].z - centrez/2;


	//scanning by the highest point
	pcl::PointXYZ the_point1 (pointx,pointy,pointz) ;
	the_point = the_point1;
	std:cerr << the_point << std::endl;

	for(size_t i =0; i< cloud_final->points.size(); ++i){

		if(i<6){
			cloud_final->points[i].x = cloud->points[maximus[i]].x;
			cloud_final->points[i].y = cloud->points[maximus[i]].y;
			cloud_final->points[i].z = cloud->points[maximus[i]].z;
			

		}
		else
			cloud_final->points[i] = the_point; //central point
		std::cerr <<  ""<< i << " x- "<< cloud->points[maximus[i]].x << " y- "<< cloud->points[maximus[i]].y << " z- "<< cloud->points[maximus[i]].z <<  std::endl;
		
	}

	return cloud_final;
}

////////////////////////////
// Segments the cloud to the object to be scanned 
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr sphereCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the x layer condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, the_point.x - 5))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, the_point.x + 5))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	//Creates a new point cloud to save the filtered pcl
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width    = cloud_filtered->width;
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	int num2= 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		if(!isnan(cloud_filtered->points[i].x)){
			cloud_final->points[num2].x = cloud_filtered->points[i].x;
			cloud_final->points[num2].y = cloud_filtered->points[i].y;
			cloud_final->points[num2].z = cloud_filtered->points[i].z;
			num2 ++;
		}		
	
	}
	
	std::vector<float> measures = maxmin(cloud_final);

	//distance between max and min
	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	float centrez1 = cloud_final->points[measures[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud_final->points[measures[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;


	largura = centrez/10 ; // to cm
	std::cerr << "largura result: " << largura << std::endl;

	return cloud_final;

}

//Cylinder

////////////////////////////
// discovers the highest point
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr cylPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width = 7;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);
	std::vector<float> maximus;

	maximus = maxmin(cloud);

	float centrez1 = cloud_final->points[maximus[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud_final->points[maximus[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;
	
	
	//min plus half the distance
	float pointx = cloud->points[maximus[5]].x ;
	float pointy = cloud->points[maximus[5]].y ;
	float pointz = cloud->points[maximus[4]].z - centrez/2;


	//scanning point is the highest
	pcl::PointXYZ the_point1 (pointx,pointy,pointz) ;
	the_point = the_point1;
	std:cerr << the_point << std::endl;

	for(size_t i =0; i< cloud_final->points.size(); ++i){

		if(i<6){
			cloud_final->points[i].x = cloud->points[maximus[i]].x;
			cloud_final->points[i].y = cloud->points[maximus[i]].y;
			cloud_final->points[i].z = cloud->points[maximus[i]].z;
			

		}
		else
			cloud_final->points[i] = the_point; //central point
			//std::cerr <<  ""<< i << " x- "<< cloud->points[maximus[i]].x << " y- "<< cloud->points[maximus[i]].y << " z- "<< cloud->points[maximus[i]].z <<  std::endl;
		
	}

	return cloud_final;
}

////////////////////////////
// Segments the cloud to the object to be scanned
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr cylsliceCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the x layer condition - >largura
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, the_point.z - 50))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, the_point.z))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	//Creates a new point cloud to save the filtered pcl
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width    = cloud_filtered->width;
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	int num2= 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		if(!isnan(cloud_filtered->points[i].x)){
			cloud_final->points[num2].x = cloud_filtered->points[i].x;
			cloud_final->points[num2].y = cloud_filtered->points[i].y;
			cloud_final->points[num2].z = cloud_filtered->points[i].z;
			num2 ++;
		}		
	
	}

	std::vector<float> measures = maxmin (cloud_final);

	//distance between max and min
	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;


	diametro = centrex/10; //to cm
	
	// build the x layer condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condy (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condy->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, the_point.y - 5))); //Greather than
	range_condy->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, the_point.y + 5))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremy;
	condremy.setCondition (range_condy);
	condremy.setInputCloud (cloud);
	condremy.setKeepOrganized(true);
	condremy.filter (*cloud_filtered);

	//number of filtered points
	num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	//Creates a new point cloud to save the filtered pcl
	cloud_final->width    = cloud_filtered->width;
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	num2= 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		if(!isnan(cloud_filtered->points[i].x)){
			cloud_final->points[num2].x = cloud_filtered->points[i].x;
			cloud_final->points[num2].y = cloud_filtered->points[i].y;
			cloud_final->points[num2].z = cloud_filtered->points[i].z;
			num2 ++;
		}		
	
	}

	measures = maxmin (cloud_final);

	float centrez1 = cloud_final->points[measures[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud_final->points[measures[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;

	altura = centrez/10; //to cm

	std::cerr << "altura result: " << altura <<  "diametro result: "<< diametro <<std::endl;
	return cloud_final;

}

//Pyramid

////////////////////////////
//Removes planes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  groundRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);	 
	cloud_final->width = 7;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);
/*
	//std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr  model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (10);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

	//obtaining outliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr the_inliers (new pcl::PointIndices());
	the_inliers->indices = inliers;

	extract.setInputCloud(cloud);
	extract.setIndices(the_inliers);
	extract.setNegative(true);
	extract.filter(*cloud_final);*/
	std::vector<float> maximus;

	maximus = maxmin(cloud);

	float centrez1 = cloud->points[maximus[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud->points[maximus[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;
	
	//min plus half the distance
	float pointx = cloud->points[maximus[5]].x;
	float pointy = cloud->points[maximus[5]].y ;
	float pointz = cloud->points[maximus[4]].z - centrez/2;


	//Scanning by the highest point
	pcl::PointXYZ the_point1 (pointx,pointy,pointz) ;
	the_point = the_point1;
	std:cerr << the_point << std::endl;

	for(size_t i =0; i< cloud_final->points.size(); ++i){

		if(i<6){
			cloud_final->points[i].x = cloud->points[maximus[i]].x;
			cloud_final->points[i].y = cloud->points[maximus[i]].y;
			cloud_final->points[i].z = cloud->points[maximus[i]].z;
			
		}
		else
			cloud_final->points[i] = the_point; //central point
		
		std::cerr <<  ""<< i << " x- "<< cloud->points[maximus[i]].x << " y- "<< cloud->points[maximus[i]].y << " z- "<< cloud->points[maximus[i]].z <<  std::endl;
		
	}

	return cloud_final;
} 


////////////////////////////
// cuts two slices to the pyr
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr pyrsliceCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the z condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condz (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, the_point.y - 80))); //Greather than
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, the_point.y + 80))); //Less than

	// build the z filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremz;
	condremz.setCondition (range_condz);
	condremz.setInputCloud (cloud);
	condremz.setKeepOrganized(true);
	condremz.filter (*cloud_filtered);

	// build the x layer condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, the_point.x - 30))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, the_point.x + 20))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud_filtered);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	//Creates a new point cloud to save the filtered pcl
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width    = cloud_filtered->width;
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	int num2= 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		if(!isnan(cloud_filtered->points[i].x)){
			cloud_final->points[num2].x = cloud_filtered->points[i].x;
			cloud_final->points[num2].y = cloud_filtered->points[i].y;
			cloud_final->points[num2].z = cloud_filtered->points[i].z;
			num2 ++;
		}		
	
	}

	//obtains max
	std::vector<float> measures = maxmin(cloud_final);

	float centrey1 = cloud_final->points[measures[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud_final->points[measures[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;

	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	float centrez1 = cloud_final->points[measures[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud_final->points[measures[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;

	//uses the biggest
	if(centrey < centrex)
		comprimento = centrex/10; //to cm
	else
		comprimento = centrey/10;
	
	altura = centrez/10; // to cm
	std::cerr << "altura : " << altura << " comprimento: " << comprimento <<  std::endl;
	return cloud_final;

}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////Mesh functions///////////////////////////////
///////////////////////////////////////////////////////////////////////////////


////////////////////////////
// Generates point cloud triangle mesh
////////////////////////////
pcl::PolygonMesh meshingCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (10);   // posso trocar o search radius aumentar ou diminuir

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	return triangles;
}

////////////////////////////
// Measures triangle volume
////////////////////////////
float SignedVolumeOfTriangle(const pcl::PointXYZ p1,const  pcl::PointXYZ p2, const pcl::PointXYZ p3) {
    float v321 = p3.x*p2.y*p1.z;
    float v231 = p2.x*p3.y*p1.z;
    float v312 = p3.x*p1.y*p2.z;
    float v132 = p1.x*p3.y*p2.z;
    float v213 = p2.x*p1.y*p3.z;
    float v123 = p1.x*p2.y*p3.z;
    return (1.0f/6000.0f)*(-v321 + v231 + v312 - v132 - v213 + v123); // dividin for 1000 because it's cm^3
}

////////////////////////////
// Generates volume of point cloud
////////////////////////////
float VolumeOfMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PolygonMesh mesh) {

	std::vector<float> vols ;
	float volume = 0;
	float final_volume = 0;
	int ix = 0;
	int iy = 0;
	int iz = 0;

	for (size_t i = 0 ; i< mesh.polygons.size(); ++i){
		ix = mesh.polygons[i].vertices[0];
		iy = mesh.polygons[i].vertices[1];
		iz = mesh.polygons[i].vertices[2];
		
		volume =  SignedVolumeOfTriangle(cloud->points[ix], cloud->points[iy], cloud->points[iz]);
		vols.push_back(volume);

	}

	//sums all the triangles
	final_volume = accumulate(vols.begin(),vols.end(),0);
	final_volume = sqrt (final_volume * final_volume);
	return final_volume;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////Classification functions///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////
//Obtains prediction result and saves it to a buffer
////////////////////////////////////////////////////////
char* ClassificationRequest(){

	int sock = 0, valread; 
	struct sockaddr_in serv_addr; 
	char *hello = "Analyse this pcd"; 
	char buffer[1024] = {0}; 
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0){ 
	printf("\n Socket creation error \n"); 
	return buffer; 
	} 

	serv_addr.sin_family = AF_INET; 
	serv_addr.sin_port = htons(PORT); 

	// Convert IPv4 and IPv6 addresses from text to binary form 
	if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0){ 
		printf("\nInvalid address/ Address not supported \n"); 
		return buffer; 
	} 

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){ 
		printf("\nConnection Failed \n"); 
		return buffer; 
	} 
	send(sock , hello , strlen(hello) , 0 ); 
	printf("Predict request sent\n"); 
	valread = read( sock , buffer, 1024); 
	char *result = buffer;
	//message
	printf("%s\n",buffer );
	return result;

}

////////////////////////////////////////////////////////
//Converts the message received to float values
////////////////////////////////////////////////////////
std::vector<float> messagetoFloat(const char* classification){

	std::vector<float> finals ;

	char firstprob[4] = {0};
	char secondprob[4] = {0};
	char otherprob[4] = {0};
	char firstclass[3] = {0};
	char secondclass[3] = {0};

	//gets individual values
	int j = 0;
	
	for(size_t i = 0; i< 4 ; ++i){
		firstprob[j] = classification[i];
		j+=1;
	}
	j= 0;

	for(size_t i= 5; i< 9 ; ++i){
		secondprob[j] = classification[i];
		j+=1;
	}
	j = 0;

	for(size_t i = 10; i< 13 ; ++i){
		otherprob[j] = classification[i];
		j+=1;
	}
	j = 0;

	for(size_t i = 14; i< 17 ; ++i){
		firstclass[j] = classification[i];
		j+=1;
	}
	j = 0;

	for(size_t i = 18; i< 21 ;++i){
		secondclass[j] = classification[i];
		j+=1;
	}
	j = 0;

	//converts char* to float
	finals.push_back(atof(firstprob));
	finals.push_back(atof(secondprob));
	finals.push_back(atof(otherprob));
	finals.push_back(atof(firstclass));
	finals.push_back(atof(secondclass));

	return finals;

}


////////////////////////////
// Removes the ground
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr groundRemoving(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZ>);

	// Segment the ground
	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);

	// Make room for a plane equation (ax+by+cz+d=0)
	plane->values.resize (4);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (9);  // aumentar caso o ground seja ondulado diminiur caso seja mais simples

	seg.setInputCloud (cloud);
	seg.segment (*inliers_plane, *plane);

	if (inliers_plane->indices.size () == 0) 
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");

	// Extract inliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);			// Extract the inliers
	extract.filter (*cloud_inliers);		// cloud_inliers contains the plane
	extract.setNegative (true);				// Extract the outliers
	  extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane

	//criar a point cloud para guardar a nova e filtrada pcl

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width    = cloud_outliers->points.size ();
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	int num2= 0;

	for (size_t i = 0; i < cloud_outliers->points.size (); ++i){
	
		cloud_final->points[num2].x = cloud_outliers->points[i].x;
		cloud_final->points[num2].y = cloud_outliers->points[i].y;
		cloud_final->points[num2].z = cloud_outliers->points[i].z;
		num2 ++;
			
	}

	return cloud_final;

}

////////////////////////////
// Removes the excedent points
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr pointsRemoving(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	// measuring the centre of the point cloud
	float total_x = 0;
	float total_y = 0;
	float total_z = 0;
	float mean_x = 0;
	float mean_y = 0;
	float mean_z = 0;
	float total_points = cloud->points.size ();

	for (size_t i = 0; i < cloud->points.size (); ++i){
		
		total_x += cloud->points[i].x;
		total_y += cloud->points[i].y;
		total_z += cloud->points[i].z;		
	}

	mean_x = total_x / total_points ;
	mean_y = total_y / total_points ;
	mean_z = total_z / total_points ;

	//creating the centre point
	pcl::PointXYZ centre (mean_x,mean_y,mean_z) ;
	//std::cout << "Centre x- " << centre.x << " y- " << centre.y << " z- " << centre.z << ""<<std::endl;

	//saving all the distances to the centre
	std::vector<float> distances;
	float distance = 0;
	for (size_t i = 0; i < cloud->points.size (); ++i){
		

		distance = squaredEuclideanDistance(  cloud->points[i], centre);
		distances.push_back(distance);

	}

	//discovering the farthest points until the cloud as only 2048 points

	int removing = total_points - 2048;
	int detected = 0;

	std::vector<int> remove_points ;
	int farthest_value = distances[0];  //valor minimo e' o primeiro ponto conhecido
	int farthest_point = 0;

	int mean_distance1 = 0;
	int mean_distance = 0;

	for (size_t i = 0; i < distances.size (); ++i){

		mean_distance1 += distances[i];
	}

	mean_distance = mean_distance1 / distances.size();

	while(detected != removing){

		for (size_t i = 0; i < distances.size (); ++i){

			if(distances[i] > farthest_value){
				farthest_value = distances[i];
				farthest_point = i;
			}
		}

		//guardo o indice do ponto a retirar e retiro a posicao das outras distancias 
		distances[farthest_point] = mean_distance; //with erase, the vector would substitute the deleted position by the deleted position +1
		detected++;
		remove_points.push_back(farthest_point);
		//std::cout << "point " << farthest_point << " removed !"<<std::endl;
		farthest_point = 0;
		farthest_value = distances[0];

	}

	// create new point cloud without the farthest points

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width = 2048;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);

	int num= 0; // var auxiliar

	for (size_t i = 0; i < cloud->points.size (); ++i){
		
		// ve se e um ponto a eliminar
		std::vector<int>::iterator it = std::find(remove_points.begin(), remove_points.end(), i );
	
		// caso seja igual ao fim da lista, esse ponto nao pertence a lista de remover
		if(it == remove_points.end() && num <2048){ //in some pcds removes more than the necessary	
			cloud_final->points[num].x = cloud->points[i].x;
			cloud_final->points[num].y = cloud->points[i].y;
			cloud_final->points[num].z = cloud->points[i].z;
			num ++;
		}
	}

	return cloud_final;

}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////Decision functions/////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////////////////////
// Decides scanning method and its parameteres
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr SlicesVolume (const int solido, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final1 (new pcl::PointCloud<pcl::PointXYZ>);	//resultado do remove
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final2 (new pcl::PointCloud<pcl::PointXYZ>);	//resultado do volume
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux1 (new pcl::PointCloud<pcl::PointXYZ>); //permite alteracoes na cloud original
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux2 (new pcl::PointCloud<pcl::PointXYZ>); //apenas aux
  
	if (solido == 0){

		std::cerr << "Big parallelepiped scanning" << std::endl;
		Ktype = 2500;
		MeanKtype = 1500;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_aux1, centre);

		std::cerr << " Big parallelepiped volume" << std::endl;
		//retira caracteristicas
		cloud_aux = valuePoint(cloud_final1);
		cloud_aux = zsliceCloud(cloud_final1);
		cloud_final2 = BigplaneRemove(cloud_final1);
		volume = comprimento * altura * largura;
		comprimento = 0;
		altura = 0;
		largura = 0;
	}

	if (solido == 1){

		std::cerr << "Big sphere scanning" << std::endl;
		removeType = 130;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_aux1 = circularCloud(cloud_aux1);
		//removes outliers
		cloud_final1 = FinalCleaning(cloud_aux1);
		//removes points in the borders
		cloud_final1 = borderCloud(cloud_final1, removeType);
		//removes the rest of the ground
		cloud_final1 = RandomSampleConsensus(cloud_final1);

		std::cerr << " Big Sphere volume" << std::endl;
		//detecta esfera
		cloud_aux = spherePoint(cloud_final1);
		//obtem diametro
		cloud_final2 = sphereCloud(cloud_final1);
		diametro = largura;
		float raio = diametro /2;
		float r3 = pow (raio, 3.0);
		volume = (4*M_PI* r3) /3 ;
	
	}

	if (solido == 2){

		std::cerr << "Cube scanning" << std::endl;
		//grande
		//Ktype = 1000;
		//MeanKtype = 650;
		Ktype = 700;
		MeanKtype = 450;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_aux1, centre);

		std::cerr << "Cube volume" << std::endl;
		cloud_final2 = cubePlane (cloud_final1);
		float c3 = pow(largura, 3.0);
		volume = c3;
		largura = 0;

	}

	if (solido == 3){

		std::cerr << "Cylinder scanning" << std::endl;
		removeType = 130;
		Ktype = 300;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final1 = circularCloud(cloud_aux1);
		//removes the rest of the ground
		cloud_final1 = RandomSampleConsensus(cloud_final1);
		//removes outliers
		cloud_final1 = FinalCleaning(cloud_final1);

		std::cerr << "Cylinder volume" << std::endl;
		cloud_final2 = spherePoint(cloud_final1);
		cloud_final2 = cylsliceCloud(cloud_final1);
		float raio = diametro /2;
		float r2 = pow (raio, 2.0);
		volume = M_PI*r2* altura;

	}

	if (solido == 4){

		std::cerr << "Pyramid scanning" << std::endl;
		removeType = 130;
		Ktype = 300;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final1 = circularCloud(cloud_aux1);
		//removes the rest of the ground
		cloud_final1 = RandomSampleConsensus(cloud_final1);
		//removes outliers
		cloud_final1 = FinalCleaning(cloud_final1);

		std::cerr << "Pyramid volume" << std::endl;
		cloud_aux = groundRemove (cloud_final1);
		cloud_final2 = pyrsliceCloud(cloud_final1);
		AB = comprimento * comprimento;
		volume = (AB * altura) / 3;
		
	}

	if (solido == 5){

		std::cerr << "Small parallelepiped scanning" << std::endl;
		Ktype = 2500;
		MeanKtype = 1500;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_aux1, centre);
		//apenas para o mais pequeno
		MeanKtype = 750;
		cloud_final1 = FinalCleaning(cloud_final1);
		Ktype = 900;
		//measures the centre point on top 
		centre = meanPoint(cloud_final1);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_final1, centre);

		//retira caracteristicas
		cloud_final2 = planeRemove(cloud_final1);
		cloud_aux = valuePoint(cloud_final2);
		cloud_aux = zsliceCloud(cloud_final1);
		volume = comprimento * altura * largura;
		comprimento = 0;
		altura = 0;
		largura = 0;
		
	}

	if (solido == 6){

		std::cerr << "Small sphere scanning" << std::endl;
		removeType = 130;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final1 = circularCloud2(cloud_aux1);

		std::cerr << "Small sphere volume" << std::endl;
		//detecta esfera
		cloud_aux = spherePoint(cloud_final1);
		//obtem diametro trocar para altura e nao largura 
		cloud_final2 = sphereCloud(cloud_final1);
		diametro = largura;
		float raio = diametro /2;
		float r3 = pow (raio, 3.0);
		volume = (4*M_PI* r3) /3 ;
		
	}

	//output cloud
	return cloud_final2;
}

////////////////////////////
// Decides mesh scanning method and its parameteres
////////////////////////////
pcl::PolygonMesh MeshVolume (int solido, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux1 (new pcl::PointCloud<pcl::PointXYZ>); //permite alteracoes na cloud original
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux2 (new pcl::PointCloud<pcl::PointXYZ>); //apenas aux
	pcl::PolygonMesh mesh_final ;
	
	float factor =0;
	float factor2 = 0;
	float factor3 = 0;

	if (solido == 7){

		std::cerr << "Big parallelepiped scanning" << std::endl;
		Ktype = 2500;
		MeanKtype = 1500;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_aux1, centre);

		std::cerr << "Parallelepiped volume" << std::endl;
		factor =40;
		factor2 = 1.15;
		factor3 = 53,6;
		mesh = true;

	}

	if (solido == 8){

		std::cerr << "Big sphere scanning" << std::endl;
		removeType = 130;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_aux1 = circularCloud(cloud_aux1);
		//removes outliers
		cloud_final1 = FinalCleaning(cloud_aux1);
		//removes points in the borders
		cloud_final1 = borderCloud(cloud_final1, removeType);
		//removes the rest of the ground
		cloud_final1 = RandomSampleConsensus(cloud_final1);

		std::cerr << "Big Sphere volume" << std::endl;
		factor = 50;
		factor2 = 2.5;
		factor3 = 60;
		mesh = true;
	
	}

	if (solido == 9){

		std::cerr << "Cube scanning" << std::endl;
		//grande
		//Ktype = 1000;
		//MeanKtype = 650;
		Ktype = 700;
		MeanKtype = 450;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_aux1, centre);

		std::cerr << "Cube volume" << std::endl;
		factor = 6.45 ;
		factor3 = 30;
		mesh = true;

	}

	if (solido == 10){

		std::cerr << "Cylinder scanning" << std::endl;
		removeType = 130;
		Ktype = 300;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final1 = circularCloud(cloud_aux1);
		//removes the rest of the ground
		cloud_final1 = RandomSampleConsensus(cloud_final1);
		//removes outliers
		cloud_final1 = FinalCleaning(cloud_final1);

		//removes outliers
		MeanKtype = 100;
		cloud_final1 = FinalCleaning(cloud_final1);

		std::cerr << "Cylinder volume" << std::endl;
		factor = 9.54 ;
		factor3 = 271;
		mesh = true;

	}

	if (solido == 11){

		std::cerr << "Pyramid scanning" << std::endl;
		removeType = 130;
		Ktype = 300;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final1 = circularCloud(cloud_aux1);
		//removes the rest of the ground
		cloud_final1 = RandomSampleConsensus(cloud_final1);
		//removes outliers
		cloud_final1 = FinalCleaning(cloud_final1);


		std::cerr << "Pyramid volume" << std::endl;
		factor = 8 ;
		factor2 = 20;
		factor3 = 131;
		mesh = true;

	}

	if (solido == 12){

		std::cerr << "Small parallelepiped scanning" << std::endl;
		Ktype = 2500;
		MeanKtype = 1500;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_aux1, centre);
		//apenas para o mais pequeno
		MeanKtype = 750;
		cloud_final1 = FinalCleaning(cloud_final1);
		Ktype = 900;
		//measures the centre point on top 
		centre = meanPoint(cloud_final1);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_final1, centre);

		std::cerr << "Small Parallelepipe volume" << std::endl;
		factor =20;
		factor2 = 85,2;
		mesh = true;

	}

	if (solido == 13){

		std::cerr << "Small sphere scanning" << std::endl;
		removeType = 130;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final1 = circularCloud2(cloud_aux1);

		std::cerr << "Small Sphere volume" << std::endl;
		factor = 20;
		factor2 = 836;
		mesh = true;
	

	}

	if(mesh){
		mesh_final = meshingCloud(cloud);
		std::cerr << "Polygons: " << mesh_final.polygons.size() << " " <<std::endl;
		volume = VolumeOfMesh(cloud, mesh_final);
		volume = volume * factor;
		volume = volume * factor3;
	}

	return mesh_final;
}


int main(void)
{
	_DOUTFUNC();

	//キー入力モード
	int mode = -1;

	//Load ini file
	LoadIniFile();

	//Create TofManager
	TofManager tofm;

	//Open TOF Manager (Read tof.ini file)
	if (tofm.Open() != Result::OK){
		std::cout << "TofManager Open Error (may not be tof.ini file)" << endl;
		return -1;
	}

	//Get number of TOF sensor and TOF information list
	const TofInfo * ptofinfo = nullptr;
	int numoftof = tofm.GetTofList(&ptofinfo);

	if (numoftof == 0){
		std::cout << "No TOF Sensor" << endl;
		return -1;
	}

	//Create Tof instance for a TOF sensor
	Tof tof;

	//Open Tof instance (Set TOF information)
	if (tof.Open(ptofinfo[0]) != Result::OK){
		std::cout << "TOF ID " << ptofinfo[0].tofid << " Open Error" << endl;
		return -1;
	}

	//Once Tof instances are started, TofManager is not necessary and closed
	if (tofm.Close() != Result::OK){
		std::cout << "TofManager Close Error" << endl;
		return -1;
	}

	//Set camera mode as Depth mode
	if (tof.SetCameraMode(CameraMode::CameraModeDepth) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Mode Error" << endl;
		return -1;
	}

	//Set camera pixel
	if (tof.SetCameraPixel(CameraPixel::w320h240) != Result::OK){
		//	if (tof.SetCameraPixel(CameraPixel::w160h120) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Pixel Error" << endl;
		return -1;
	}

	//Set TOF sensor angle and height
	if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Position Error" << endl;
		return -1;
	}

	//Noise reduction(Low signal cutoff)
	if (tof.SetLowSignalCutoff(10) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Low Signal Cutoff Error" << endl;
		return -1;
	}

	//Edge noise reduction
	if (tof.SetEdgeSignalCutoff(EdgeSignalCutoff::Enable) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Edge Noise Reduction Error" << endl;
		return -1;
	}

	//Start human detection
	if (tof.Run(RunMode::HumanDetect) != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Run Error" << endl;
		return -1;
	}
	std::cout << "TOF ID " << tof.tofinfo.tofid << " Run OK" << endl;

	//Create instances for reading frames
	FrameDepth frame;

	//Create instances for 3D data after conversion
	Frame3d frame3d;

	//Create instances for reading human frames
	FrameHumans framehumans;

	//Create color table
	frame.CreateColorTable(0, 65530);

	//Create display window as changeable size
	cv::namedWindow("Volume App", CV_WINDOW_NORMAL);

	//Sub display
	cv::Mat subdisplay(SUB_DISPLAY_WIDTH, SUB_DISPLAY_HEIGHT, CV_8UC3);

	//Initialize background
	back = cv::Mat::zeros(480 * 2, 640 * 2, CV_8UC3);

	//number of saved point clouds
	int num = 0;
	int num2=0;	//number of saved points
	int total_p = 0;
	//directories to save point clouds
	directories.push_back("Originals/Cubes/");
	directories.push_back("Originals/Spheres/");
	directories.push_back("Originals/Pyramids/");
	directories.push_back("Originals/Parallelepipeds/");
	directories.push_back("Originals/Cylinders/");


	//analysis area
	point3.x = 187;
	point3.y = 787;
	point4.x = 84;
	point4.y = 790;
	point5.x = 88;
	point5.y = 715;
	point6.x = 180;
	point6.y = 712;

	//principal cicle
	bool brun = true;
	while (brun){

		//Get the latest frame number
		long frameno;
		TimeStamp timestamp;
		tof.GetFrameStatus(&frameno, &timestamp);

		if (frameno != frame.framenumber){
			//Read a new frame only if frame number is changed(Old data is shown if it is not changed.)

			//Read a frame of humans data
			Result ret = Result::OK;
			ret = tof.ReadFrame(&framehumans);
			if (ret != Result::OK) {
				std::cout << "read frame error" << endl;
				break;
			}

			//Read a frame of depth data
			ret = Result::OK;
			ret = tof.ReadFrame(&frame);  
			if (ret != Result::OK) {
				std::cout << "read frame error" << endl;
				break;
			}

			//3D conversion(with lens correction) - lens e como se endireitasse as linhas, ficando paralelas as margens
			frame3d.Convert(&frame);

			//3D rotation in Z,Y,X order(to top view)
			frame3d.RotateZYX(angle_x, angle_y, angle_z);

			//Initialize Z-buffer
			memset(z_buffer, 0, sizeof(z_buffer));

			if (bBack){
				img = back.clone();
			}
			else {
				img = cv::Mat::zeros(480 * 2, 640 * 2, CV_8UC3);
			}

			if (bSubDisplay){
				//Initialize sub display
				subdisplay = cv::Mat::zeros(SUB_DISPLAY_HEIGHT, SUB_DISPLAY_WIDTH, CV_8UC3);
			}

			// creating pcl 
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			cloud->width    = frame3d.width;
			cloud->height   = frame3d.height;
			cloud->is_dense = false;
			cloud->points.resize (cloud->width * cloud->height);

			//apenas para tratamento dos dados e a sua amostragem na img
			for (int y = 0; y < frame3d.height; y++){
				for (int x = 0; x < frame3d.width; x++){
					
					if ((frame.CalculateLength(frame.databuf[y * frame.width + x]) >= framehumans.distance_min) &&
						(frame.CalculateLength(frame.databuf[y * frame.width + x]) <= framehumans.distance_max)){
						//Valid data(Only data in specific distance for sensor is valid)
						// define z distance where the point cloud can be captured					
	
						TofPoint point;		//Coordinate after rotation
						
						//Get coordinates after 3D conversion
						point.x = frame3d.frame3d[y * frame3d.width + x].x;
						point.y = frame3d.frame3d[y * frame3d.width + x].y;
						point.z = frame3d.frame3d[y * frame3d.width + x].z;

						//storing points in pcl
						if(bScan){
							if(bFixed){
								if(point.x != 0 && point.y !=0 && point.z != 0){
									total_p++;							
									cloud->points[y * frame3d.width + x].x = frame3d.frame3d[y * frame3d.width + x].x;
									cloud->points[y * frame3d.width + x].y = frame3d.frame3d[y * frame3d.width + x].y;
									cloud->points[y * frame3d.width + x].z = frame3d.frame3d[y * frame3d.width + x].z;
								}
							}
							else{
								if(point.x > point3.x && point.x < point4.x){
								if(point.y > point3.y && point.y < point4.y){
									if(point.x != 0 && point.y !=0 && point.z != 0){
										total_p++;							
										cloud->points[y * frame3d.width + x].x = frame3d.frame3d[y * frame3d.width + x].x;
										cloud->points[y * frame3d.width + x].y = frame3d.frame3d[y * frame3d.width + x].y;
										cloud->points[y * frame3d.width + x].z = frame3d.frame3d[y * frame3d.width + x].z;
									}}}
							
							}
						}

						//Zoom
						point.x *= zoom;
						point.y *= zoom;

						//Shift to X/Y direction on display
						point.x += dx;
						point.y += dy;

						if ((point.x >= 0) && (point.x < img.size().width) &&
							(point.y >= 0) && (point.y < img.size().height)){

							if ((point.z >= framehumans.z_min) && (point.z < framehumans.z_max)){
								//Within range of Z direction

								if ((z_buffer[(int)point.x][(int)point.y] == 0) ||
									(z_buffer[(int)point.x][(int)point.y] > point.z)){
									//Front than data already registered in Z-buffer

									//Register to Z-buffer
									z_buffer[(int)point.x][(int)point.y] = point.z;

									//Register color to display image based on distance of Z direction
									long color = (long)(65530 * (point.z - framehumans.z_min) / ((framehumans.z_max - framehumans.z_min)));
									 //ColorTable 1 e rgb 2 e a distancia z q atribui valor de cor
									cv::Vec3b v;
									v.val[0] = frame.ColorTable[0][color];
									v.val[1] = frame.ColorTable[1][color];
									v.val[2] = frame.ColorTable[2][color];
									

									//draws the points in the middle of the screen
									if (bPoint){
										img.at<cv::Vec3b>((int)point.y, (int)point.x) = v;
									}
								}
								//draws the subdisplay with colours
								if (bSubDisplay){
									//Sub display
									cv::Vec3b v;
									v.val[0] = frame.ColorTable[0][frame.databuf[y * frame.width + x]];
									v.val[1] = frame.ColorTable[1][frame.databuf[y * frame.width + x]];
									v.val[2] = frame.ColorTable[2][frame.databuf[y * frame.width + x]];
									//desenha um ponto na posiçaõ em que esta na point cloud no subdisplay
									subdisplay.at<cv::Vec3b>(y * SUB_DISPLAY_HEIGHT / frame3d.height,
										x * SUB_DISPLAY_WIDTH / frame3d.width) = v;

								}
								
							}
						}
					}
					else {
						//Invalid point is (x,y,z) = (0,0,0)
						frame3d.frame3d[y * frame3d.width + x].x = 0;
						frame3d.frame3d[y * frame3d.width + x].y = 0;
						frame3d.frame3d[y * frame3d.width + x].z = 0;
					}
				}
			} 

			//Draw human counter
			if ((mode == 'a') || (mode == 'h')){
				//Draw side/front view
				DrawSection(&frame3d);
				
			}

			//Saves pcd for dataset generation				
			if(bTest){
				std::string file, end, result, directory;
				directory = directories[decision];
				file = "cloud";
				end = ".pcd";

				//int conversion to string
  				std::string number;
  				std::stringstream ss;

				// number decision according to object type
				if(decision == 0){
					saved_cube++;
					ss << saved_cube;
				}
				if(decision == 1){
					saved_sphere++;
					ss << saved_sphere;
				}
				if(decision == 2){
					saved_pyramid++;
					ss << saved_pyramid;
				}
				if(decision == 3){
					saved_paralellipiped++;
					ss << saved_paralellipiped;
				}
				if(decision == 4){
					saved_cylinder++;
					ss << saved_cylinder;
				}
  					
  				number = ss.str();
				result =  directory + file + number + end;

				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
				cloud_final->width = total_p;
				cloud_final->height = 1;
				cloud_final->is_dense = false;
				cloud_final->points.resize(cloud_final->width * cloud_final->height);

				//remove points with value 0 in its coordinates
				for (size_t i = 0; i < cloud_final->points.size (); ++i){
					if(cloud->points[i].x != 0 && cloud->points[i].y != 0){
		        			cloud_final->points[num2].x = cloud->points[i].x;
						cloud_final->points[num2].y = cloud->points[i].y;
						cloud_final->points[num2].z = cloud->points[i].z;
						num2 ++;		
					}
				}

				//point cloud at the moment in cloud_final
   				pcl::io::savePCDFileASCII (result, *cloud_final);
   				std::cerr << "Saved " << cloud_final->points.size () <<" points to " << result << " !" << std::endl;
				bTest = !bTest;
				fDecision = true;
				bScan = false;
				num2 = 0;

			}

			if(bClassification){
			
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
				cloud_final->width = total_p;
				cloud_final->height = 1;
				cloud_final->is_dense = false;
				cloud_final->points.resize(cloud_final->width * cloud_final->height);

				//remove points with value 0 in its coordinates
				for (size_t i = 0; i < cloud_final->points.size (); ++i){
					if(cloud->points[i].x != 0 && cloud->points[i].y != 0){
		        			cloud_final->points[num2].x = cloud->points[i].x;
						cloud_final->points[num2].y = cloud->points[i].y;
						cloud_final->points[num2].z = cloud->points[i].z;
						num2 ++;		
					}
				}

				//reduces the cloud to the desired size 2048 points
				cloud_final = cuttingCloud(cloud_final, -310, 310, -1020, -400);
				cloud_final = groundRemoving(cloud_final);
				cloud_final = pointsRemoving(cloud_final);

				//point cloud at the moment in cloud_final
   				pcl::io::savePCDFileASCII ("cloud.pcd", *cloud_final);
   				std::cerr << "Saved " << cloud_final->points.size () <<" points to cloud.pcd !" << std::endl;
				//bTest = !bTest;
				//fDecision = true;
				bScan = false;
				num2 = 0;

				//obtains the prediction results
				char* classification = ClassificationRequest();
				//transforms message to float
				std::vector<float> result = messagetoFloat(classification);
				std::cerr << "0 " << result[0] << "1 " << result[1] << "2 " << result[2] << "3 " << result[3] << "4 " << result[4] << "5 " << result[5]<< std::endl;
				int auxi2 = (int) result[3] ;
				if(auxi2 == 0){
					final_cls1 = "Big Parallelepiped";
					final_prob1 = result[0];
					solid = 0; 
				}
				if(auxi2 == 1){
					final_cls1 = "Small Parallelepiped";
					final_prob1 = result[0];
					solid = 5; 
				}
				if(auxi2 == 2){
					final_cls1 = "Small Sphere";
					final_prob1 = result[0];
					solid = 1; 
				}
				if(auxi2 == 3){
					final_cls1 = "Big Sphere";
					final_prob1 = result[0];
					solid = 6; 
				}
				if(auxi2 == 4){
					final_cls1 = "Cube";
					final_prob1 = result[0];
					solid = 2; 
				}
				if(auxi2 == 5){
					final_cls1 = "Pyramid";
					final_prob1 = result[0];
					solid = 3; 
				}
				if(auxi2 == 6){
					final_cls1 = "Cylinder";
					final_prob1 = result[0];
					solid = 4; 
				}

				//second probability
				int auxi3 = (int) result[4] ;
				if(auxi3 == 0){
					final_cls2 = "Big Parallelepiped";
					final_prob2 = result[1];
					solid = 0; 
				}
				if(auxi3 == 1){
					final_cls2 = "Small Parallelepiped";
					final_prob2 = result[1];
					solid = 5; 
				}
				if(auxi3 == 2){
					final_cls2 = "Small Sphere";
					final_prob2 = result[1];
					solid = 1; 
				}
				if(auxi3 == 3){
					final_cls2 = "Big Sphere";
					final_prob2 = result[1];
					solid = 6; 
				}
				if(auxi3 == 4){
					final_cls2 = "Cube";
					final_prob2 = result[1];
					solid = 2; 
				}
				if(auxi3 == 5){
					final_cls2 = "Pyramid";
					final_prob2 = result[1];
					solid = 3; 
				}
				if(auxi3 == 6){
					final_cls2 = "Cylinder";
					final_prob2 = result[1];
					solid = 4; 
				}

				bClassification = !bClassification;
				fDecision = true;
			}

			if(bAnalysis){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux2 (new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final2 (new pcl::PointCloud<pcl::PointXYZ>);
				cloud_final2->width = total_p;
				cloud_final2->height = 1;
				cloud_final2->is_dense = false;
				cloud_final2->points.resize(cloud_final2->width * cloud_final2->height);

				//remove points with value 0 in its coordinates
				for (size_t i = 0; i < cloud_final2->points.size (); ++i){
					if(cloud->points[i].x != 0 && cloud->points[i].y != 0){
		        			cloud_final2->points[num2].x = cloud->points[i].x;
						cloud_final2->points[num2].y = cloud->points[i].y;
						cloud_final2->points[num2].z = cloud->points[i].z;
						num2 ++;		
					}
				}

	
				pcl::PolygonMesh mesh_final ;

				if(fAlgorithm){
					cloud_aux2  = SlicesVolume (solid, cloud_final2);
					// stops the normal application execution but works separately
					/*
					//normal visualization
					pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
					viewer.addPointCloud (cloud_final2, "cloud");
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb (cloud_aux2, 255 ,0, 0);
					viewer.addPointCloud (cloud_aux2 , rgb, "cloud outliers");
					viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud outliers");

					while (!viewer.wasStopped ()) {
						viewer.spinOnce ();
						boost::this_thread::sleep_for(boost::chrono::seconds(12));
						viewer.close();
					}
					*/
				}
				else{
					mesh_final = MeshVolume(solid, cloud_final2);
					/*
					//mesh visualization
					boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
					viewer->addPolygonMesh(mesh_final,"meshes",0);

					while (!viewer->wasStopped ()){
						viewer->spinOnce (100);
						boost::this_thread::sleep_for(boost::chrono::seconds(12));
						viewer->close();
					}
					*/
				}
				
				std::cerr << "Volume result: " << volume << std::endl;
				fDecision = true;
				bAnalysis = !bAnalysis;
				bScan = false;
				num2 = 0;

			}

			//Display information
			int tx = 10;
			int ty = 80;
			int tdy = 40;
			cv::Scalar white(255, 255, 255);
			cv::Scalar blue(255, 0, 0);
			cv::Scalar red(0, 0, 255);
			cv::Scalar gray(128, 128, 128);
			cv::Scalar color = white;
			cv::Scalar color2 = white;
			string text = VERSION;
			

			cv::putText(img, text, cv::Point(1000, 30), cv::FONT_HERSHEY_TRIPLEX, 1.0, blue, 2, CV_AA);

			text = "q key for Quit, m key for Menu";
			cv::putText(img, text, cv::Point(tx, 30), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
			switch (mode){
			case 'a':
				text = "Angle x=" + std::to_string((int)angle_x);
				if (180 < angle_x){
					text += "(" + std::to_string((int)angle_x - 360) + ")";
				}
				text += "[degree]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Angle y=" + std::to_string((int)angle_y);
				if (180 < angle_y){
					text += "(" + std::to_string((int)angle_y - 360) + ")";
				}
				text += "[degree]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Angle z=" + std::to_string((int)angle_z);
				if (180 < angle_z){
					text += "(" + std::to_string((int)angle_z - 360) + ")";
				}
				text += "[degree]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				color = red;
				text = "Angle x u:up / n:down | Angle z k:up / j:down";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 's':
				text = "Shift x=" + std::to_string((int)dx) + "[mm]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Shift y=" + std::to_string((int)dy) + "[mm]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				color = red;
				text = "u:up / n:down / j:left / k:right";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 'z':
				text = "Zoom=" + std::to_string((int)(zoom * 100)) + "%";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "u:up / n:down";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 'h':
				text = "Height from Floor=" + std::to_string((int)height) + "[mm]";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				color = red;
				text = "u:up / n:down";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 'p':
				text = "Display Key 1: Points ";
				if (bPoint){
					text += "ON";
				}
				else {
					text += "OFF";
				}
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Display Key 2: Sub Display ";
				if (bSubDisplay){
					text += "ON";
				}
				else {
					text += "OFF";
				}
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				break;
			case 'f':
				if (savefile != ""){
					text = "Saved to " + savefile;
				}
				else {
					text = "Save Failed !";
				}
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 'q':
				text = "Quit ? y: yes";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				break;
			case 't':
				//obtem o clique do rato
				cv::setMouseCallback("Volume App", callback);
				text = "Click in the left mouse button to allow a new scan  ";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Place the object in the middle of the blue square  ";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;	
				text = "1 - Save as a Cube ";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;	
				text = "2 - Save as a Sphere ";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;	
				text = "3 - Save as a Pyramid ";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;	
				text = "4 - Save as a Parallelepiped ";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;	
				text = "5 - Save as a Cylinder ";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;	
				if (fDecision){
					text = "Point cloud saved ! ";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
				}
				break;
			case 'd':	
				cv::setMouseCallback("Volume App", callback);
				if(bFixed){
					text = "0 - Analysis area: Fixed";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "Click in the left mouse button to allow a new scan  ";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "Place the object in the middle of the blue square  ";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "1 - Analyse point cloud";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					if (fDecision){
						text = "The object as " + std::to_string((int)(final_prob1 * 100)) + " %";
						text = text + " of being a ";
						text = text + final_cls1;
						cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
						ty += tdy;
						text = "The object as " + std::to_string((int)(final_prob2 * 100)) + " %";
						text = text + " of being a ";
						text = text + final_cls2;
						cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
						ty += tdy;
					}
					break;

				}
				else{
					text = "0 - Analysis area: Movel";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "Click in the place to scan with the left mouse button";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "1 - Analyse point cloud";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					if (fDecision){
						text = "The object as " + std::to_string((int)(final_prob1 * 100)) + " %";
						text = text + " of being a ";
						text = text + final_cls1;
						cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
						ty += tdy;
						text = "The object as " + std::to_string((int)(final_prob2 * 100)) + " %";
						text = text + " of being a ";
						text = text + final_cls2;
						cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
						ty += tdy;
						
					}
					break;
				}
			case 'g':	
				cv::setMouseCallback("Volume App", callback);
				if(bFixed){
					text = "0 - Analysis area: Fixed";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "Click in the left mouse button to allow a new scan  ";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "Place the object in the middle of the blue square  ";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
				}
				else{
					text = "0 - Analysis area: Movel";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "Click in the place to scan with the left mouse button";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
				}
				text = "1 - Volume algorithm : ";
				if (fAlgorithm){
					text += "Slices Algorithm";
					auxi1 = 0;
				}
				else {
					text += "Mesh Algorithm";
					auxi1 = 7;
				}
				if (bManual){
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "2 - Classification: Manual";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;	
					text = "3 - Parallelepiped volume";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "4 - Sphere volume";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "5 - Cube volume";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "6 - Cylinder volume";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "7 - Pyramid volume";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "8 - Small Parallelepiped volume";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "9 - Small Sphere volume";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					if (fDecision){
						std::ostringstream sf;
						sf << volume;
						std::string vol(sf.str());
						text = "The object's volume is " + vol;
						text = text + " cm3 ";
						cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
						ty += tdy;
					}
					break;
				}
				else {
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "2 - Classification: Automatic";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					text = "3 - Analyse point cloud";
					cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
					ty += tdy;
					if (fDecision){
						text = "The object is a " + final_cls1;
						cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
						ty += tdy;
						std::ostringstream sf;
						sf << volume;
						std::string vol(sf.str());
						text = "The volume is " + vol ;
						cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
						ty += tdy;
					}
					break;
				}
			case 'w':
				text = "To change analysis area position use:";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "u - move to the top";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "n - move to the bottom";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "j - move to the left";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "k - move to the right";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				break;	
				
					
			case 'm':
				text = "Key q: Quit";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key p: Display Key";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key a: Angle";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key s: Shift";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key z: Zoom";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key h: Height from Floor";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key f: File Save";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key m: Menu";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key t: Get Point Cloud for dataset";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key d: Object Classification";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key g: Volume estimation";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
				text = "Key w: Adjust Analysis area";
				cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_TRIPLEX, 1.0, color, 2, CV_AA);
				ty += tdy;
			}

			if (bSubDisplay){
				//Sub display
				if ((angle_y == 0) && ((angle_z < 45) || (angle_z > 270))){ 
					cv::Mat roi = img(cv::Rect(SUB_DISPLAY_X, SUB_DISPLAY_Y, SUB_DISPLAY_WIDTH, SUB_DISPLAY_HEIGHT));
					cv::resize(subdisplay, roi, roi.size(), cv::INTER_LINEAR);
				}
				else {
					for (int y = 0; y < SUB_DISPLAY_HEIGHT; y++){
						for (int x = 0; x < SUB_DISPLAY_WIDTH; x++){						
							img.at<cv::Vec3b>(SUB_DISPLAY_Y - (SUB_DISPLAY_WIDTH - SUB_DISPLAY_HEIGHT) + x,
								SUB_DISPLAY_X + SUB_DISPLAY_HEIGHT - y) =
								subdisplay.at<cv::Vec3b>(y, x);
						}
					}
				}

				
				//draws the rectangle where the point cloud is captured
				if(bScan){
					if(bFixed){
						cv::rectangle(img,point1,point2,cv::Scalar(0,128,255),2,CV_AA,0);
						cv::line(img,point3,point4,cv::Scalar(255,0,0),2,0);
						cv::line(img,point3,point6,cv::Scalar(255,0,0),2,0);
						cv::line(img,point6,point5,cv::Scalar(255,0,0),2,0);
						cv::line(img,point5,point4,cv::Scalar(255,0,0),2,0);
					}
					else{
						cv::rectangle(img,point1,point2,cv::Scalar(0,128,255),2,CV_AA,0);
					}
				}
				
			}

			if (NULL == cvGetWindowHandle("Volume App")){
				brun = false;
			}
			else{
				cv::imshow("Volume App", img);
			}

		total_p = 0; // reset the number of points stored
		}

		int key = cv::waitKey(10);
#if defined (__linux__) || defined(__linux)
		key &= 0xffff;	// Linux cv::waitKey issue. (0x100000 map)
#endif	//_LINUX
		switch (key){
		case 'a':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 's':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'z':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'h':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'd':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 't':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'g':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'p':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'f':
			if (mode == key){
				mode = 0;
			}
			else {
				//Save file
				if (!SaveFile()){
					//Failed
					savefile = "";
				}
				mode = key;
			}
			break;
		case 'm':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;

			}
			break;
		case 'w':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case 'l':
			break;
		//actions
		case '1':
			if (mode == 't'){
				bTest = !bTest;
				decision = 0;
			}
			if (mode == 'p'){
				bPoint = !bPoint;
			}
			if (mode == 'g'){
				fAlgorithm = !fAlgorithm;
			}
			if (mode == 'd'){
				bClassification = !bClassification;
				
			}
			break;
		case '2':
			if (mode == 't'){
				bTest = !bTest;
				decision = 1;
			}
			if (mode == 'g'){
				bManual= !bManual;
			}
			break;
		case '3':
			if (mode == 't'){
				bTest = !bTest;
				decision = 2;
			}
			if (mode == 'g' && !bManual){
				bClassification = !bClassification;
				bAnalysis = !bAnalysis;
				
			}
			if (mode == 'g' && bManual){
				bAnalysis = !bAnalysis;
				solid = 0 + auxi1;
			}
			break;
		case '4':
			if (mode == 't'){
				bTest = !bTest;
				decision = 3;
			}
			if (mode == 'g'){
				bAnalysis = !bAnalysis;
				solid = 1 + auxi1;
			}
			break;
		case '5':
			if (mode == 't'){
				bTest = !bTest;
				decision = 4;
			}
			if (mode == 'g'){
				bAnalysis = !bAnalysis;
				solid = 2 + auxi1;
			}
			break;
		case '6':
			if (mode == 'g'){
				bAnalysis = !bAnalysis;
				solid = 3 + auxi1;
			}
			break;
		case '7':
			if (mode == 'g'){
				bAnalysis = !bAnalysis;
				solid = 4 + auxi1;
			}
			break;
		case '8':
			if (mode == 'g'){
				bAnalysis = !bAnalysis;
				solid = 5 + auxi1;
			}
			break;
		case '9':
			if (mode == 'g'){
				bAnalysis = !bAnalysis;
				solid = 6 + auxi1;
			}
			break;
		case '0':
			if (mode == '0'){
				mode = 0;
			}
			if (mode == 'g'){
				bFixed = !bFixed;
			}
			if (mode == 'd'){
				bFixed = !bFixed;
			}
			break;
		case CVWAITKEY_CURSOR_TOP:
			switch (mode){
			case 'a':
				angle_x += ANGLE_ADJUSTMENT_DEGREE;
				if (angle_x >= 360){
					angle_x -= 360;
				}
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			case 's':
				dy -= 100 * zoom;
				break;
			case 'z':
				zoom += 0.01f;
				break;
			case 'w':
				aa_height -= 2;
				break;
			case 'h':
				height += 100;
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			}
			break;
		case CVWAITKEY_CURSOR_BOTTOM:
			switch (mode){
			case 'a':
				angle_x -= ANGLE_ADJUSTMENT_DEGREE;
				if (angle_x < 0){
					angle_x += 360;
				}
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			case 's':
				dy += 100 * zoom;
				break;
			case 'z':
				zoom -= 0.01f;
				break;
			case 'w':
				aa_height += 2;
				break;
			case 'h':
				height -= 100;
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			}
			break;
		case CVWAITKEY_CURSOR_RIGHT:
			switch (mode){
			case 'a':
				angle_z += ANGLE_ADJUSTMENT_DEGREE;
				if (angle_z >= 360){
					angle_z -= 360;
				}
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			case 's':
				dx += 100 * zoom;
				break;
			case 'w':
				aa_width += 2;
				break;
			}
			break;
		case CVWAITKEY_CURSOR_LEFT:
			switch (mode){
			case 'a':
				angle_z -= ANGLE_ADJUSTMENT_DEGREE;
				if (angle_z < 0){
					angle_z += 360;
				}
				if (ChangeAttribute(tof, 0, 0, height * -1, angle_x, angle_y, angle_z) == false){
					std::cout << "TOF ID " << tof.tofinfo.tofid << " Set Camera Attributee Error" << endl;
				}
				break;
			case 's':
				dx -= 100 * zoom;
				break;
			case 'w':
				aa_width -= 2;
				break;
			}
			break;
		case  'q':
			if (mode == key){
				mode = 0;
			}
			else {
				mode = key;
			}
			break;
		case  'y':
			if (mode == 'q'){
				brun = false;
			}
			mode = 0;
			break;
		default:
			break;
		}
	}

	//Stop and closr TOF sensor
	bool berror = false;
	if (tof.Stop() != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Stop Error" << endl;
		berror = true;
	}

	os_sleep(2000);

	if (tof.Close() != Result::OK){
		std::cout << "TOF ID " << tof.tofinfo.tofid << " Close Error" << endl;
		berror = true;
	}

	cv::destroyAllWindows();

	if (berror){
		cerr << "err:" << __FILE__ << endl;
	}

	//Save ini file
	if (SaveIniFile() == false){
		std::cout << "Ini File Write Error" << endl;
	}

	_DOUTFUNC();

	return 0;
}
