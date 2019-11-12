/**
* @file			tof.h
* @brief		Header File for Hitachi-LG Data Storage (HLDS)'s TOF Motion Sensor SDK
* @author		Hitachi-LG Data Storage, Inc. (HLDS)
* @date			2019.02.14
* @version		v2.3.0
* @copyright	Hitachi-LG Data Storage,Inc.
*
* @par Change History:
* - 2016.03.29 New
* - 2016.04.22 v1.1.0
*					- Auto setting for empty items in initialization file
*					- Cancel Standby in Tof::Open()
*					- Change argument of Tof::GetVersion()
*					- Add Tof::ReadFrame(Frame3d*)
*					- Add change endian in FrameDepth, FrameIr data
*					- Enable Save() of FrameDepth, FrameIr
*					- Add FrameDepth::CalculateLength()
*					- Delete FrameDepth::ConvertDepthTo3D()
*					- Add Frame3d class
*					- Add error code in Result type
* - 2016.06.03 v1.2.0
*					- Bug fix of SetIrGain() (Could not set over 10)
*					- Add FrameDepth::Save(), FrameIr::Save() for JPEG and PNG
*					- Set Standby Off Mode at Tof::Run()
*					- Set Standby On Mode at Tof::Stop()
*					- Improve communication speed with TOF sensor
*					- Add Tof::SetDistanceMode(), GetDistanceMode()
*					- Add Tof::SetFrameRate(), GetFrameRate()
* - 2016.07.01 v1.3.0
*					- Support Capture function
*					- Support human detection function
*					- Add Tof::SetLowSignalCutoff(), Tof::GetLowSignalCutoff()
*					- Add Tof::SetAttribute(), Tof::GetAttribute()
* - 2016.09.01 v2.0.0
*					- Support Depth/IR dual output
*					- Add modes in CameraMode, DistanceMode
*					- Add Tof::ReadFrame() for dual output
*					- Add Tof::ResetBackground()
*					- Add Tof::SetBackgroundInterval(), GetBackgroundInterval()
*					- Add Tof::SetBackgroundQuantity(), GetBackgroundQuantity()
*					- Add Tof::SetDistanceMin(), Tof::GetDistanceMin()
*					- Add Tof::PauseEmulationTof()
* - 2017.08.08 v2.1.0
*					- Add Tof::SetFarSignalCutoff(), Tof::GetFarSignalCutoff()
*					- Add Tof::SetEdgeSignalCutoff(), Tof::GetEdgeSignalCutoff()
*					- Add Frame3d::RotateZYX()
*					- Delete Tof::SetDistanceMin(), Tof::GetDistanceMin()
*					- Support hand detection function
* - 2018.06.20 v2.2.0
*					- Add FrameRate::fr10fps, and change enum order
*					- Add Tof::bConsoleLog, TofManager::bConsoleLog
* - 2019.02.14 v2.3.0
*					- Add Tof::SetImpulseSignalCutoff(), GetImpulseSignalCutoff()
*					- Add long exposure mode in FrameRate
*					- Add humandetect member in Tof class
*					- Add Tof::GetOperationTime()
*					- Add Copy Constructor in Frame3d class
*					- Add Tof::GetStatus(TofStatusInfo* status)
*					- Add Camera class
*/

#ifndef _hlds_H
#define _hlds_H

#include <string>
#include <vector>

using namespace std;

/**
* @brief
* 	Namespace for HLDS TOF Sensor SDK
*/
namespace hlds{

#define SDK_VER				"v2.3.0"	///< SDK Version

#define IMAGE_MAX_WIDTH		(640)		///< Max pixels of an image (horizontal)
#define IMAGE_MAX_HEIGHT	(480)		///< Max pixels of an image (vertical)
#define COLOR_CH_NUM		(3)			///< Number of Channels at converting to a color image
#define MAX_BYTE			(0xFF)		///< Max value of byte
#define MAX_INT16			(0xFFFF)	///< Max value of int16
#define BYTE_PER_PIXEL		(2)			///< Byte size of a pixel
#define NUM_OF_DISTORTION	(8)			///< Number of Distortion of lens of TOF sensor which the data was gotten
#define NUM_OF_SHADING		(5)			///< Number of Shading correction data

#define PRIVATE_ATTRIBUTES		class Impl; Impl* pImpl;

	typedef string	TofId;			///< Type of TOF ID (It is allocated at production of TOF sensor and never be changed)
	typedef string	TofMac;			///< Type of TOF MAC address
	typedef string	TofSn;			///< Type of TOF S/N(Serial Number)
	typedef string	TofIp;			///< Type of TOF IP address

	/**
	* @brief
	* 	TOF version
	*/
	enum class TofVersion
	{
		TOFv1 = 1,			///< TOFv1 sensor
		TOFv2 = 2,			///< TOFv2 sensor
		Unknown = -1,		///< Other TOFs
	};
	
	/**
	* @brief
	* Structure of TOF Information of a TOF sensor
	*/
	struct TofInfo {
		TofId	tofid;				///< TOF ID (It is allocated at production of TOF sensor and never be changed)
		TofMac	tofmac;				///< MAC address
		TofSn	tofsn;				///< TOF S/N (Serial Number)
		TofIp	tofip;				///< IP address
		int		rtp_port;			///< RTP port number
		float	distance_min;		///< Minimum measurable distance [mm] of the data (Actual distance for 0x0000 of data)
		float	distance_max;		///< Maximum measurable distance [mm] of the data (Actual distance for 0xfffe of data)
		TofVersion tofver;			///< TOF Hardware Type
	};

	/**
	* @brief
	* 	Structure of timestamp
	* @remarks
	*	- Timestamp (UTC)
	*/
	struct TimeStamp {
		unsigned short year;		///< Year
		unsigned short month;		///< Month
		unsigned short dayofweek;	///< Weekday(Sun=0, Mon=1, Tue=2, Wed=3, Thu=4, Fri=5, Sat=6)
		unsigned short day;			///< Day
		unsigned short hour;		///< Hour
		unsigned short minute;		///< Minute
		unsigned short second;		///< Second
		unsigned short msecond;		///< Millisecond
	};

	/**
	* @brief
	* 	Lens information of the TOF sensor which got the frame data
	*/
	struct LensParam {
		float	focallength;					///< Focal length of TOF sensor which the data was gotten
		float	fov_x;							///< Horizontal FOV (Degree) of TOF sensor which the data was gotten
		float	fov_y;							///< Vertical FOV (Degree) of TOF sensor which the data was gotten
		float	ellipticity;					///< Ellipticity of lens of TOF sensor which the data was gotten
		double	distortion[NUM_OF_DISTORTION];	///< Distortion of lens of TOF sensor which the data was gotten
		double	shading[NUM_OF_SHADING];		///< Shading correction data
	};

	/**
	* @brief
	* 	Structure to set capture file
	*/
	struct CaptureInfo{
		string path;			///<Path of capture file (Finish with file separator '/')
		string filename;		///<Name of capture file
	};

	/**
	* @brief
	* 	3D point coordinate
	*/
	struct TofPoint {
		float	x;				///< x-coordinate
		float	y;				///< y-coordinate
		float	z;				///< z-coordinate
	};

	/**
	* @brief
	* Return code of function (Error Code)
	*/
	enum class Result {
		// General (Success)
		OK = 0,	///< Success

		// General (Fail)
		SequenceError			= -1,		///< Sequence Error
		ArgumentInvalid			= -2,		///< Argument Error
		TimeOut					= -3,		///< Time-out Error
		MemoryAllocError		= -4,		///< Memory Allocation Error
		Unsupported				= -5,		///< Unsupported Error

		// Sensor
		SensorArgumentInvalid	= -100,		///< Sensor Argument Error
		SensorConnectionFail	= -101,		///< Sensor Connection Error
		SensorAlreadyConnected	= -102,		///< Sensor Already connected
		SensorCommandFail		= -103,		///< Sensor Command Error
		SensorTimeout			= -104,		///< Sensor Time-out Error
		SensorOtherError		= -105,		///< Sensor Other Error

		// File operation
		IniOperationFail		= -200,		///< ini file Related Error
		SDKEnvPathFail			= -201,		///< SDK Environment Path Error
		DriverOpenFail			= -202,		///< TOF Driver Open Error
		TofConflictError		= -203,		///< TOF Conflict Error
		CaptureOpenFail			= -204,		///< Capture Open Error

		// Other Error
		OtherError				= -1000,	///< Other Error
	};

	/**
	* @brief
	* 	Run Mode
	* @remarks
	*	- Set for Run() method of Tof class.
	*/
	enum class RunMode
	{
		Normal		= 0,			///< Normal Mode
		HumanDetect	= 1,			///< Human Detect Mode
		FrameEmulation = 2,			///< Frame by frame play in emulation mode
		Unknown		= -1,			///< Not set or Unknown Mode
	};

	/**
	* @brief
	* 	Camera Mode
	* @remarks
	*	- Set through SetCamera() method of Tof class.
	* @note
	*	- Motion, Background, and dual output are available from v2.0.0 or later.
	*/
	enum class CameraMode
	{
		CameraModeDepth			= 0,	///< Depth Mode
		CameraModeIr			= 1,	///< IR Mode
		CameraModeMotion		= 2,	///< Motion
		CameraModeBackground	= 3,	///< Background
		Depth_Motion			= 4,	///< Depth and Motion
		Depth_Background		= 5,	///< Depth and Background
		Depth_Ir				= 6,	///< Depth and IR
		Motion_Background		= 7,	///< Motion and Background
		Motion_Ir				= 8,	///< Motion and IR
		Background_Ir			= 9,	///< Background and IR
		CameraModeUnknown		= -1,	///< Not set or Unknown Mode
	};

	/**
	* @brief
	* 	To set pixels of a Camera Image
	* @remarks
	*	- Set through SetCameraPixel() method of Tof class.
	*/
	enum class CameraPixel {
		w640h480 = 0,	///< 640 x 480(VGA)
		w320h240 = 1,	///< 320 x 240(QVGA)
		w160h120 = 2,	///< 160 x 120(QQVGA)
		w80h60 = 3,		///< 80 x 60
		w64h48 = 4,		///< 64 x 48
		w40h30 = 5,		///< 40 x 30
		w32h24 = 6,		///< 32 x 24
	};

	/**
	* @brief
	* 	To set distance mode of TOF sensor
	* @remarks
	*	- Set through SetDistanceMode(), GetDistanceMode() method of Tof class.
	*/
	enum class DistanceMode {
		dm_0_5x = 0,	///< 0.5x: Half Distance of Default Distance Mode
		dm_1_0x = 1,	///< 1.0x: Default Distance Mode
		dm_1_5x = 2,	///< 1.5x: One and a half Distance of Default Distance Mode
		dm_2_0x = 3,	///< 2.0x: Twice Distance of Default Distance Mode
		Unknown = -1,	///< Not set or Unknown Mode
	};

	/**
	* @brief
	* 	Status of a human
	* @remarks
	*	- status member of Human struct
	*/
	enum class HumanStatus {
		Untracked		= -1,	///< Not detected
		Walk			= 0,	///< Walking
		Stand			= 1,	///< Standing(&Stopping)
		Crouch			= 2,	///< Crouching
		StandHand		= 3,	///< Standing and reaching hand
		CrouchHand		= 4,	///< Crouching and reaching hand
	};

	/**
	* @brief
	* Structure of human detect information
	*/
	struct Human {
		long	id;				///< Human ID
		float	x;				///< X-coordinate of human [mm]
		float	y;				///< Y-coordinate of human [mm]
		float	direction;		///< Direction of human [0 to 360 degree]
		float	headheight;		///< Tall of human head [mm]
		float	handheight;		///< Tall of human hand [mm]
		HumanStatus	status;		///< Status of human
	};

	/**
	* @brief
	* 	To set frame rate of TOF sensor
	* @remarks
	*	- Set through SetFrameRate(), GetFrameRate() method of Tof class.
	*	- 
	* @note
	*	- TOF sensor which supports the function is required.
	*   - fr16fps_long and fr10fps_long are only available with sensors
	*     which support the long exposure mode.
	*     (An error is returned if the sensor does not support it.)
	*/
	enum class FrameRate {
		fr30fps = 0,		///< 30 fps
		fr16fps = 1,		///< 16 fps
		fr10fps = 2,		///< 10 fps
		fr8fps = 3,			///< 8 fps
		fr4fps = 4,			///< 4 fps
		fr2fps = 5,			///< 2 fps
		fr1fps = 6,			///< 1 fps
		fr16fps_long = 7,   ///< 16 fps (long exposure mode)
		fr10fps_long = 8,   ///< 10 fps (long exposure mode)
	};

	/**
	* @brief
	* 	To set status of capture function
	* @remarks
	*	- Set for GetCaptureStatus() method of Tof class.
	*/
	enum class CaptureStatus{
		Disable = 0,	///< Disabled
		Stop = 1,		///< Stopping
		Run = 2,		///< Capturing
	};

	/**
	* @brief
	* 	To set save file type
	* @remarks
	*	- Set through Save() method of FrameData class and FrameIr class.
	*/
	enum class OutputDataType {
		BinFile = 0,	///< Binary File
		PngFile = 1,	///< PNG File(Not available yet)
		JpgFile = 2,	///< JPG File(Not available yet)
	};

	/**
	* @brief
	* 	To set interval to update background
	* @remarks
	*	- Set through SetBackgroundInterval(), GetBackgroundInterval() method of Tof class.
	*/
	enum class BgInterval {
		bg1min = 1,		///< 1 minute
		bg3min = 2,		///< 3 minutes
		bg5min = 3,		///< 5 minutes
		bg10min = 4,	///< 10 minutes
		bg30min = 5,	///< 30 minutes
		bg60min = 6,	///< 1 hour
		bg120min = 7,	///< 2 hours
		bg300min = 8,	///< 5 hours
	};

	/**
	* @brief
	* 	To set quantity to update background
	* @remarks
	*	- Set through SetBackgroundQuantity(), GetBackgroundQuantity() method of Tof class.
	*/
	enum class BgQuantity {
		None = 1,		///< No update
		bgLv1 = 2,		///< Update Level1
		bgLv2 = 3,		///< Update Level2
		bgLv3 = 4,		///< Update Level3
		bgLv4 = 5,		///< Update Level4
		bgLv5 = 6,		///< Update Level5
		bgLv6 = 7,		///< Update Level6
		bgLv7 = 8,		///< Update Level7
		bgLv8 = 9,		///< Update Level8
		bgLv9 = 10,		///< Update Level9
		Full = 11,		///< Full update
	};


	/**
	* @brief
	* 	To set edge noise reduction mode
	* @remarks
	*	- Set through SetEdgeSignalCutoff() method of Tof class.
	*/
	enum class EdgeSignalCutoff {
		Disable = 0,	///< Disabled
		Enable = 1,		///< Enabled
		Unknown = -1,	///< Not set or Unknown Mode
	};

	/**
	* @brief
	* 	To set impulse noise reduction mode
	* @remarks
	*	- Set through SetImpulseSignalCutoff() method of Tof class.
	*/
	enum class ImpulseSignalCutoff {
		Disable = 0,	///< Disabled
		Normal = 1,		///< Normal
		Unknown = -1,	///< Not set or Unknown Mode
	};

	/**
	* @brief
	* 	Status of a Tof Sensor
	* @remarks
	*	- status member of Tof Sensor
	*/
	enum class TofStatus {
		Standby = 0,	///< Standby
		Run = 1,		///< Run(Normal)
		Builtin = 2,	///< Run(Built-in Application)
		Alert = -1,		///< Error(For details on this error, refer to "alert")
	};

	/**
	* @brief
	* 	Abstract class to store a Tof status information
	* @remarks
	* 	Abstract class
	*/
	struct TofStatusInfo {
		TofStatus status;	///< TOF sensor's status
		struct {
			bool bTemp;		///< Temperature error is occured
			bool bEmit;		///< Emit error is occured
			bool bCover;	///< TOF sensor's cover is open (only HLS-LFOM1,HLS-LFOM3)
		} alert;			///< TOF sensor's error description flags
		int operationtime;	///< Accumulated operation time [h]
		int temperature;	///< TOF sensor's internal temperature[degC]
	};

	/**
	* @brief
	* 	Abstract class to store a Frame Data
	* @remarks
	* 	Abstract class
	*/
	class FrameData{
	public:
		TofInfo	tofinfo;			///< TOF sensor information which the data was gotten
		TimeStamp timestamp;		///< Timestamp(UTC)
		long	framenumber;		///< Frame Number (0 to 0x7fffffff(Round every about 820 days))
		string	modelname;			///< Model name of TOF sensor which the data was gotten
		float	distance_min;		///< Minimum measurable distance [mm] of the data (Actual distance for 0x0000 of data)
		float	distance_max;		///< Maximum measurable distance [mm] of the data (Actual distance for 0xfffe of data)
		LensParam lens;				///< Lens correction information
	};

	/**
	* @brief
	* 	Abstract class to store a Frame of Depth or IR Matrix data
	* @remarks
	* 	- Abstract class
	* 	- A pixel data (16bit) is stored as little endian from v1.1.0.
	*/
	class FrameMatrix : public FrameData{
	public:
		int	width;								///< Width of Matrix
		int	height;								///< Height of Matrix
		int	picbyte;							///< Byte size of a pixel
		int pixel;								///< Total pixels
		std::vector<unsigned short> databuf;	///< Matrix data (16 bit (little endian) x pixels)

		/**
		* @brief
		* 	Constructor
		*
		* 	- Created as max pixels(640 x 480 x 16bit)
		*/
		FrameMatrix(){
			width = IMAGE_MAX_WIDTH;
			height = IMAGE_MAX_HEIGHT;
			picbyte = BYTE_PER_PIXEL;
			pixel = width * height;
			databuf.resize(pixel);
		};
	};

	/**
	* @brief
	* 	Class to store a Frame of Depth data
	*
	*	- Use instance of this class to read Frame data in Depth mode.
	*	- Data (16 bit) of each pixel is 0x0000 to 0xfffe. (0xffff is invalid data)
	*	- To convert the data to actual distance between the object,
	*	  it should be calculated as 0x0000 = distance_min member, 0xfffe = distance_max member.
	*	  (both members are defined in FrameData class.) The unit is [mm].
	*	  (from v1.1.0, it is possible by CalculateLength() method also.)
	* 	- A pixel data (16bit) is stored as little endian from v1.1.0.
	*/
	class FrameDepth : public FrameMatrix{
	public:
		unsigned char ColorTable[COLOR_CH_NUM][(MAX_INT16 + 1)];	///< Color table

		/**
		* @brief
		* 	Create color table to display Dept data as a color image
		*
		* 	- Set color table to output to JPEG/PNG file.
		* @param	scaleMin	Min value of Scale
		* @param	scaleRange	Range of Scale
		*/
		void	CreateColorTable(unsigned short scaleMin, unsigned short scaleRange);

		/**
		* @brief
		* 	Save to file
		*
		* 	- Convert Matrix data to selected data type and save to file.
		* @param	filename	Output file name
		* @param	type		Output data type
		* @return	#Result
		* @pre
		* 	Set color table by FrameDepth::CreateColorTable() method in advance.
		* @note
		*	- Save as Binary is Available from v1.1.0. Save as JPEG/PNG is Available from v1.2.0.
		*	- If filename is empty, file name is automatically allocated based on Timestamp.
		*/
		Result	Save(char* filename, OutputDataType type);

		/**
		* @brief
		* 	Calculate actual length
		*
		* 	- Convert depth data to actual length (mm).
		* @param	depth	Depth data read by Tof::ReadFrame() method
		* @return	Length (mm)
		*			-1 is returned for invalid data (0xFFFF)
		* @note
		* 	Available from v1.1.0.
		*/
		float CalculateLength(unsigned short depth);
	};

	/**
	* @brief
	* 	Class to store a Frame of IR data
	*
	*	- Use instance of this class to read Frame data in IR mode.
	* @note
	* 	- At v1.1.0, IR data is set as 12bit data (0~4095). From v1.2.0, IR data is set as 16bit data (0~65535).
	*/
	class FrameIr : public FrameMatrix{
	public:
		/**
		* @brief
		* 	Save to file
		*
		* 	- Convert Matrix data to selected data type and save to file.
		* @param	filename	Output file name
		* @param	type		Output data type
		* @return	#Result
		* @pre
		* 	- No need to set color table because IR image is black & white.
		* @note
		*	- Save as Binary is Available from v1.1.0. Save as JPEG/PNG is Available from v1.2.0.
		*	- If filename is empty, file name is automatically allocated based on Timestamp.
		*/
		Result	Save(char* filename, OutputDataType type);
	};

	/**
	* @brief
	* 	Class to store a Frame of 3D data
	*
	* 	- Use instance of this class to read Frame data in 3D mode.
	* 	- 3D coordinate of each point corresponding to pixel is represented as x,y,z- coordinate.
	* 	- 3D frame data can be directly read by Tof::ReadFrame(Frame3d*).
	* 	- Depth frame data can be converted to 3D by Frame3d::Convert(FrameDepth*) method.
	* @note
	* 	Available from v1.1.0.
	*/
	class Frame3d : public FrameData{
	private:
		PRIVATE_ATTRIBUTES
#ifdef	_TOFBUILT_IN
		bool doneInitialize;
#endif

	public:
		int	width;								///< Width of Matrix
		int	height;								///< Height of Matrix
		int pixel;								///< Total pixels
		std::vector<TofPoint> frame3d;			///< Coordinate data

		/**
		* @brief
		* 	Constructor
		*/
		Frame3d();

		/**
		* @brief
		* 	Copy Constructor
		*/
		Frame3d(const Frame3d &frame3d);

		/**
		* @brief
		* 	Destructor
		*/
		~Frame3d();

		/**
		* @brief
		* 	Convert depth data to 3D coordinate
		*
		* 	- Convert depth data to 3D point data after lens correction based on lens correction information.
		* @param	frame		FrameDepth instance which has frame data read by Tof::ReadFrame(FrameDepth*).
		* @return	#Result
		* @note
		* 	Available from v1.1.0.
		*/
		Result Convert(FrameDepth* frame);
		
		/**
		* @brief
		* 	Rotate 3D point data
		*
		*	- Rotate all 3D point data by specified angle.
		*	- Rotate direction is clockwise toward the positive direction of each axis.
		*	- Rotation order is, X-axis rotation, Y-axis rotation, and Z-axis rotation
		* @param	rx		X-axis rotate angle (0 to 360 degree)
		* @param	ry		Y-axis rotate angle (0 to 360 degree)
		* @param	rz		Z-axis rotate angle (0 to 360 degree)
		* @return	#Result
		* @note
		* 	Available from v1.3.0.
		*/
		Result Rotate(float rx, float ry, float rz);
		
		/**
		* @brief
		* 	Rotate 3D point data
		*
		*	- Rotate all 3D point data by specified angle.
		*	- Rotate direction is clockwise toward the positive direction of each axis.
		*	- Rotation order is, Z-axis rotation, Y-axis rotation, and X-axis rotation
		* @param	rx		X-axis rotate angle (0 to 360 degree)
		* @param	ry		Y-axis rotate angle (0 to 360 degree)
		* @param	rz		Z-axis rotate angle (0 to 360 degree)
		* @return	#Result
		* @note
		* 	Available from v2.1.0.
		*/
		Result RotateZYX(float rx, float ry, float rz);
	};

	/**
	* @brief
	* 	Class to store a Frame of human detection data
	*
	* 	- Use instance of this class to read Frame data of human detection.
	* @note
	* 	Available from v1.3.0.
	*/
	class FrameHumans : public FrameData{
	public:
		std::vector<Human>	humans;		///< Array of human information for detected humans.
		int		numofhuman;				///< Number of detected humans.
		float	z_max;					///< Z-coordinate of upper limit of detection range[mm] (Negative value as floor level is 0mm)
		float	z_min;					///< Z-coordinate of lower limit of detection range[mm] (Negative value as floor level is 0mm)

		/**
		* @brief
		* 	Constructor
		*/
		FrameHumans(){
			humans.clear();
			numofhuman = 0;
			z_max = 0.0f;
			z_min = 0.0f;
		};
	};

	/**
	* @brief
	* 	Class to control a TOF sensor
	*
	* 	- Set TOF information by Tof::Open() method after creating instance.
	* 	- Get TOF information of all TOF sensor managed by SDK through TofManager::GetTofList() method.
	*/
	class Tof{
	private:
		PRIVATE_ATTRIBUTES
		void HumanThread(void);			
		Result		GetStandbyMode(bool* enabled);
		Result		GetTemperature(int* temperature);

	public:
		TofInfo		tofinfo;	///< TOF information of the TOF sensor
#ifdef	_TOFBUILT_IN
		unsigned int capturetime;	///< Max capture duration (seconds). Default is 600 seconds.
		bool bConsoleLog;			///< Enable/Disable output log by SDK
#else
		unsigned int capturetime = 600;	///< Max capture duration (seconds). Default is 600 seconds.
		bool bConsoleLog = true;		///< Enable/Disable output log by SDK
#endif

		struct {
			float detect_high;		///< High limit from floor to detect humans [mm]
			float detect_low;		///< Low limit from floor to detect humans [mm]
		} humandetect;				///< Conditions to detect humans in RunMode::HumanDetect

		/**
		* @brief
		* 	Constructor
		*/
		Tof();

		/**
		* @brief
		* 	Destructor
		*/
		~Tof();

		/**
		* @brief
		*	Start to use a TOF sensor
		*
		*	- Invalid to access to TOF sensor until this method is called even after Tof instance is created.
		*	- Input TOF information of a TOF sensor gotten through TofManager::GetTofList() as an argument.
		*	- If TOF sensor is in Standby mode, Standby mode is canceled. (from v1.1.0)
		* @param	tofinfo		TOF information of a TOF to be started
		* @return	#Result
		*
		*/
		Result		Open(TofInfo tofinfo);

		/**
		* @brief
		*	Start to use an Emulated TOF sensor
		*
		*	- Start Tof instance in emulation mode to replay a capture file.
		*	- Possible to start without connecting real TOF sensor.
		*	- Settings for TOF sensor is according to the capture file.
		*	- Functions other than replaying capture file are basically invalid.
		* @param	info		Full path of capture file + file name (Folder separator is \\)
		* @return	#Result
		* @note
		* 	Available from v1.3.0 or later.
		*/
		Result		Open(CaptureInfo info);

		/**
		* @brief
		*	End to use the TOF sensor
		*
		*	- Disconnected with a TOF sensor in this method.
		* @return	#Result
		*/
		Result		Close(void);

		/**
		* @brief
		*	Start transferring from the TOF sensor
		*
		*	- Start RTP transferring from TOF sensor and start to receive Frame data.
		* @return	#Result
		*/
		Result		Run(void);

		/**
		* @brief
		*	Start transferring from the TOF sensor (for Human Detect)
		*
		*	- In case of RunMode::Normal, Start RTP transferring from TOF sensor and start to receive Frame data.
		*	- In case of RunMode::HumanDetect, Start RTP transferring from TOF sensor and
		*	  human detection is started in background.
		*	- In case of RunMode::HumanDetect, installed position of TOF sensor must be set by SetAttribute() method in advance.
		* @param	mode		Operation with receiving data.
		* @return	#Result
		* @note
		* 	Available from v1.3.0 or later.
		*/
		Result		Run(RunMode mode);

		/**
		* @brief
		*	Stop transferring from the TOF sensor
		*
		*	- Stop RTP transferring from TOF sensor and stop to receive Frame data.
		* @return	#Result
		*/
		Result		Stop(void);

		/**
		* @brief
		*	Restart the TOF sensor
		*
		*	- Restart and initialize the TOF sensor.
		*	- Execute this method to restart the TOF sensor when TOF sensor does not reply properly or
		*	  there is no response from TOF sensor.
		*	  As in case of network trouble, could not restart TOF sensor by this method,
		*	  check network status or restart TOF sensor by disconnecting and connecting the cable.
		*	- It takes 30 seconds to several minutes for Restart the TOF sensor.
		* @return	#Result
		*/
		Result		Restart(void);

		/**
		* @brief
		*	Set Standby Mode for the TOF sensor
		*
		*	- If the TOF sensor will not be used for long time,
		*	  set Standby mode in order to reduce power consumption.
		*	- Since it is automatically set in Standby mode during stop status, 
		*	  this method is not necessary to be used.
		* @param	enabled		Mode (true: Standby Mode false: Cancel)
		* @pre
		*	- It shall be set in stopping status before running by Tof::Run() or after stopping by Tof::Stop().
		*	- Error is returned if it is set during transferring.
		* @return	#Result
		*/
		Result		SetStandbyMode(bool enabled);

		/**
		* @brief
		*	Get versions of each module in the TOF sensor
		*
		*	- Get versions of each module in the TOF sensor.
		* @param	fpga	Pointer to output FPGA version
		* @param	osvers	Pointer to output LinuxOS version
		* @param	root	Pointer to output TOF sensor firmware version
		* @return	#Result
		*/
		Result		GetVersion(string* fpga, string* osvers, string* root);

		/**
		* @brief
		*	Get status of the TOF sensor
		*
		*	- Get the latest status of the TOF sensor.
		* @param	status	Pointer to output status
		* @return	#Result
		*/
		Result		GetStatus(unsigned char* status);

		/**
		* @brief
		*	Get status of the TOF sensor
		*
		*	- Get the latest status of the TOF sensor.
		* @param	TofStatusInfo 	Pointer to output status
		* @return	#Result
		*/
		Result		GetStatus(TofStatusInfo* status);

		/**
		* @brief
		*	Get operation time
		*
		*	- Get the accumulated time of laser emitting.
		* @param	operation_time		Operation Time (0 to 0xffffffff) [hour]
		* @return	#Result
		* @note
		* 	- TOF sensor which supports the function is required.
		* 	- If TOF sensor which not supports the function, Result returns an error and operation_time is set to 0.
		* 	- Available from v2.3.0.
		*/
		Result		GetOperationTime(unsigned int* operation_time);

		/**
		* @brief
		*	Set physical installation position of TOF sensor
		*
		*	- Set physical installation position of TOF sensor.
		*	- 3D rotation order is, Z-axis, Y-axis, and X-axis.
		* @param	x		x-coordinate of TOF sensor [mm]
		* @param	y		y-coordinate of TOF sensor [mm]
		* @param	z		z-coordinate of TOF sensor(-1 * height from floor) [mm]
		* @param	rx		X-axis rotation angle(0 to 359 degree)
		* @param	ry		Y-axis rotation angle(0 to 359 degree)
		* @param	rz		Z-axis rotation angle(0 to 359 degree)
		* @return	#Result
		* @note
		* 	- Available from v1.3.0.
		*/
		Result		SetAttribute(float x,
								 float y,
								 float z,
								 float rx,
								 float ry,
								 float rz
								 );

		/**
		* @brief
		*	Get physical installation position of TOF sensor
		*
		*	- Get physical installation position of TOF sensor.
		* @param	x		x-coordinate of TOF sensor [mm]
		* @param	y		y-coordinate of TOF sensor [mm]
		* @param	z		z-coordinate of TOF sensor(-1 * height from floor) [mm]
		* @param	rx		X-axis rotation angle(0 to 359 degree)
		* @param	ry		Y-axis rotation angle(0 to 359 degree)
		* @param	rz		Z-axis rotation angle(0 to 359 degree)
		* @return	#Result
		* @note
		* 	- Available from v1.3.0.
		*/
		Result		GetAttribute(float* x,
								 float* y,
								 float* z,
								 float* rx,
								 float* ry,
								 float* rz
								 );

		/**
		* @brief
		*	Get status of the latest received Frame
		*
		*	- Get the latest Frame information which SDK received.
		*	- If Frame number or Timestamp is different from previous recieved data, it means a new Frame was recieved.
		*	- In case of emulation mode, frameno is reported as -2 at the end of capture file.
		* @param	frameno		Frame Number (0 to 0x7fffffff(Round every about 820 days)) of the latest Frame, -1 is set if not received yet)
		* @param	timestamp	Timestamp(UTC)
		* @return	#Result
		* @post
		*	- The latest Frame can be read calling ReadFrame() after Frame number or Timestamp is changed.
		*	- The newer Frame than this method reported may be read by ReadFrame() method depending on timing.
		*/
		Result		GetFrameStatus(long* frameno, TimeStamp* timestamp);

		/**
		* @brief
		*	Read the latest received Depth Frame data
		*
		*	- Read the latest Depth Frame data into indicated FrameDepth instance.
		* @param	frame		FrameDepth instance to be read into
		* @return	#Result
		* @remarks
		*	- This method waits until the first Frame is received.
		*	- If a newer Frame than the Frame in inputted FrameDepth instance is not received yet, wait until a new Frame arrival.
		* @pre
		*	- Create FrameDepth instance in user application.
		*	- Set Camera Mode in Depth Mode in advance.
		*	- It is recommended to call this method after confirming received Frame is changed by GetFrameStatus().
		*/
		Result		ReadFrame(FrameDepth* frame);

		/**
		* @brief
		*	Read two latest received Depth Frame data simultaneously
		*
		*	- Read two latest Depth Frame data simultaneously into indicated FrameDepth instance.
		* @param	frame1		FrameDepth instance1 to be read into
		* @param	frame2		FrameDepth instance2 to be read into
		* @return	#Result
		* @remarks
		*	- This method waits until the first Frame is received.
		*	- If a newer Frame than the Frame in inputted FrameDepth instance is not received yet, wait until a new Frame arrival.
		* @pre
		*	- Create FrameDepth instance in user application.
		*	- Set Camera Mode in Depth Mode/Motion/Background dual output in advance.
		*	- It is recommended to call this method after confirming received Frame is changed by GetFrameStatus().
		* @note
		* 	Available from v2.0.0 or later.
		*/
		Result		ReadFrame(FrameDepth* frame1, FrameDepth* frame2);

		/**
		* @brief
		*	Read the latest received Depth and IR data Frames simultaneously
		*
		*	- Read the latest Depth Frame data into indicated FrameDepth instance.
		*	- Read the latest IR Frame data into indicated FrameIr instance.
		* @param	frame1		FrameDepth instance1 to be read into
		* @param	frame2		FrameIr instance to be read into
		* @return	#Result
		* @remarks
		*	- This method waits until the first Frame is received.
		*	- If a newer Frame than the Frame in inputted FrameDepth instance is not received yet, wait until a new Frame arrival.
		* @pre
		*	- Create FrameDepth, FrameIr instance in user application.
		*	- Set Camera Mode in Depth Mode/Motion/Background and IR dual output in advance.
		*	- It is recommended to call this method after confirming received Frame is changed by GetFrameStatus().
		* @note
		* 	Available from v2.0.0 or later.
		*/
		Result		ReadFrame(FrameDepth* frame1, FrameIr* frame2);

		/**
		* @brief
		*	Read the latest received IR data Frame
		*
		*	- Read the latest IR Frame data into indicated FrameIr instance.
		* @param	frame		FrameIr instance to be read into
		* @return	#Result
		* @remarks
		*	- This method waits until the first Frame is received.
		*	- If a newer Frame than the Frame in inputted FrameDepth instance is not received yet, wait until a new Frame arrival.
		* @pre
		*	- Create FrameIr instance in user application.
		*	- Set Camera Mode in IR Mode in advance.
		*	- It is recommended to call this method after confirming received Frame is changed by GetFrameStatus().
		*/
		Result		ReadFrame(FrameIr* frame);

		/**
		* @brief
		*	Read the latest received Depth Frame data and convert to 3D
		*
		*	- Read the latest 3D Frame data into indicated Frame3d instance after lens correction and conversion from depth data.
		* @param	frame		Frame3d instance to be read into
		* @return	#Result
		* @remarks
		*	- This method waits until the first Frame is received.
		*	- If a newer Frame than the Frame in inputted Frame3d instance is not received yet, wait until a new Frame arrival.
		* @pre
		*	- Create Frame3d instance in user application.
		*	- Set Camera Mode in Depth Mode in advance.
		*	- It is recommended to call this method after confirming received Frame is changed by GetFrameStatus().
		* @note
		* 	Available from v1.1.0.
		*/
		Result		ReadFrame(Frame3d* frame);

		/**
		* @brief
		*	Result of human detection in the latest frame
		*
		*	- Read the latest result of human detection in the latest frame to FrameHumans instance.
		* @param	frame		FrameHumans instance to be read into
		* @return	#Result
		* @pre
		*	- Set RunMode::HumanDetect mode in Run method.
		*	- Create FrameHumans instance in user application.
		*	- It is recommended to call this method after confirming received Frame is changed by GetFrameStatus().
		* @note
		* 	Available from v1.3.0.
		*/
		Result		ReadFrame(FrameHumans* frame);

		/**
		* @brief
		*	Set Camera Mode
		*
		*	- Set Camera Mode to the TOF sensor.
		*	- Refer to #CameraMode for setting values.
		* @param	mode	Camera Mode
		* @return	#Result
		* @pre
		*	- It shall be set in stopping status before running by Tof::Run() or after stopping by Tof::Stop().
		*	- Error is returned if it is set during transferring.
		*/
		Result		SetCameraMode(CameraMode mode);

		/**
		* @brief
		*	Get the current setting of Camera Mode
		*
		*	- Get the current setting of Camera Mode of the TOF sensor.
		*	- Refer to #CameraMode for setting values.
		* @param	mode	Pointer to output Camera Mode setting
		* @return	#Result
		*/
		Result		GetCameraMode(CameraMode* mode);

		/**
		* @brief
		*	Set Pixel size of an image
		*
		*	- Change Pixel size of an image of the TOF sensor.
		*	- Refer to #CameraPixel for setting values.
		* @param	pixel	Pixel size mode
		* @return	#Result
		* @pre
		*	- It shall be set in stopping status before running by Tof::Run() or after stopping by Tof::Stop().
		*	- Error is returned if it is set during transferring.
		*/
		Result		SetCameraPixel(CameraPixel pixel);

		/**
		* @brief
		*	Get the current setting of Pixel size of an image
		*
		*	- Get the current setting of Pixel size of the TOF sensor.
		*	- Refer to #CameraPixel for setting values.
		* @param	pixel	Pointer to output Pixel size setting
		* @return	#Result
		*/
		Result		GetCameraPixel(CameraPixel* pixel);

		/**
		* @brief
		*	Set Exposure Sensitivity for IR Image
		*
		*	- Exposure Sensitivity for IR can be adjusted if IR image is too dark or bright due to surrounding light.
		* @param	irgain	Gain setting value for IR image (set from 1 to 15. Default is 8.)
		* @return	#Result
		* @pre		This setting can be changed during transferring.
		*/
		Result		SetIrGain(int irgain);

		/**
		* @brief
		*	Get the current setting of Exposure Sensitivity for IR Image
		*
		*	- Get the current setting of Exposure Sensitivity for IR Image.
		* @param	irgain	Pointer to output Gain setting (1 to 15) for IR image
		* @return	#Result
		*/
		Result		GetIrGain(int* irgain);

		/**
		* @brief
		*	Set Distance Mode of the TOF sensor
		*
		*	- Set Distance Mode of the TOF sensor.
		* @param	Refer to #DistanceMode for setting values.
		* @return	#Result
		*/
		Result		SetDistanceMode(DistanceMode mode);

		/**
		* @brief
		*	Get the current setting of Distance Mode of the TOF sensor
		*
		*	- Get the current setting of Distance Mode of the TOF sensor.
		* @param	Refer to #DistanceMode for setting values.
		* @return	#Result
		*/
		Result		GetDistanceMode(DistanceMode* mode);

		/**
		* @brief
		*	Set Frame Rate of the TOF sensor
		*
		*	- Set Frame Rate of the TOF sensor.
		* @param	fps	Refer to #FrameRate for setting values.
		* @return	#Result
		* @note
		*	- Not available yet in v1.2.0.
		*	- TOF sensor which supports the function is required.
		*/
		Result		SetFrameRate(FrameRate fps);

		/**
		* @brief
		*	Get the current setting of Frame Rate of the TOF sensor
		*
		*	- Get the current setting of Frame Rate of the TOF sensor.
		* @param	fps	Refer to #FrameRate for setting values.
		* @return	#Result
		* @note
		*	- Not available yet in v1.2.0.
		*	- TOF sensor which supports the function is required.
		*/
		Result		GetFrameRate(FrameRate* fps);

		/**
		* @brief
		*	Set low signal cutoff
		*
		*	- Set signal level to be cutoff.
		*	- Noise near by or far from sensor can be cutoff.
		*	- If cutoff level is too strong, normal signal may be also cutoff.
		*	- 0(No cutoff) to 30 is recommended value.
		* @param	value		Signal level to cutoff (0 to 4095). Larger value is larger cutoff level.
		* @return	#Result
		* @note
		*	- It is possible to change during Run() of TOF.
		* 	- Available from v1.3.0.
		*/
		Result		SetLowSignalCutoff(unsigned int value);

		/**
		* @brief
		*	Get low signal cutoff
		*
		*	- Get signal level to be cutoff.
		* @param	value		Signal level to cutoff (0 to 4095). Larger value is larger cutoff level.
		* @return	#Result
		* @note
		*	- It is possible to get during Run() of TOF.
		* 	- Available from v1.3.0.
		*/
		Result		GetLowSignalCutoff(unsigned int* value);

		/**
		* @brief
		*	Set far signal cutoff rate
		*
		*	- Set distance rate to be cutoff.
		*	- Signal far from sensor to be cutoff according to rate.
		*	- 0.0 means no cutoff. 0.1 means farther than 90% of max distance is cutoff.
		* @param	rate		Distance rate far from TOF (rate: 0.0 ~ 1.0. Default is 0.0.)
		* @return	#Result
		* @note
		*	- It is possible to change during Run() of TOF.
		* 	- Available from v2.1.0.
		*/
		Result		SetFarSignalCutoff(float rate);

		/**
		* @brief
		*	Get far signal cutoff rate
		*
		*	- Get distance rate to be cutoff.
		*	- Signal far from sensor to be cutoff according to rate.
		*	- 0.0 means no cutoff. 0.1 means farther than 90% of max distance is cutoff.
		* @param	rate		Distance rate far from TOF (rate: 0.0 ~ 1.0. Default is 0.0.)
		* @return	#Result
		* @note
		*	- It is possible to get during Run() of TOF.
		* 	- Available from v2.1.0.
		*/
		Result		GetFarSignalCutoff(float* rate);

		/**
		* @brief
		*	Set edge noise reduction mode
		*
		*	- Reduce edge noise.
		* @param	mode		edge noise reduction mode
		* @return	#Result
		* @pre
		*	- This setting can be changed during transferring.
		* @note
		* 	- Available from v2.1.0.
		*/
		Result		SetEdgeSignalCutoff(EdgeSignalCutoff mode);

		/**
		* @brief
		*	Get edge noise reduction mode
		*
		*	- Get edge noise reduction mode.
		* @param	mode		edge noise reduction mode
		* @return	#Result
		* @pre
		*	- This setting can be changed during transferring.
		* @note
		* 	- Available from v2.1.0.
		*/
		Result		GetEdgeSignalCutoff(EdgeSignalCutoff* mode);

		/**
		* @brief
		*	Set impulse noise reduction mode
		*
		*	- Reduce edge noise.
		* @param	mode		impulse noise reduction mode
		* @return	#Result
		* @pre
		*	- This setting can be changed during transferring.
		* @note
		* 	- Available from v2.3.0.
		*/
		Result		SetImpulseSignalCutoff(ImpulseSignalCutoff mode);

		/**
		* @brief
		*	Get impulse noise reduction mode1
		*
		*	- Get impulse noise reduction mode.
		* @param	mode		impulse noise reduction mode
		* @return	#Result
		* @pre
		*	- This setting can be changed during transferring.
		* @note
		* 	- Available from v2.3.0.
		*/
		Result		GetImpulseSignalCutoff(ImpulseSignalCutoff* mode);
	
		/**
		* @brief
		*	Initialize background(Reset)
		*
		*	- Initialize background(Reset).
		* @return	#Result
		* @note
		* 	Available from v2.0.0.
		*/
		Result		ResetBackground(void);

		/**
		* @brief
		*	Set interval to update background
		*
		*	- Set interval to update background.
		* @param	interval	Refer to #BgInterval  for setting values.
		* @return	#Result
		* @note
		*	- It is possible to set during Run() of TOF
		* 	- Available from v2.0.0.
		*/
		Result		SetBackgroundInterval(BgInterval interval);

		/**
		* @brief
		*	Get the current setting of interval to update background
		*
		*	- Get the current setting of interval to update background.
		* @param	interval	Refer to #BgInterval  for setting values.
		* @return	#Result
		* @note
		* 	- Available from v2.0.0.
		*/
		Result		GetBackgroundInterval(BgInterval* interval);

		/**
		* @brief
		*	Set quantity to update background
		*
		*	- Set quantity to update background.
		* @param	quantity	Refer to #BgQuantity  for setting values.
		* @return	#Result
		* @note
		*	- It is possible to set during Run() of TOF
		* 	- Available from v2.0.0.
		*/
		Result		SetBackgroundQuantity(BgQuantity quantity);

		/**
		* @brief
		*	Get the current setting of quantity to update background
		*
		*	- Get the current setting of quantity to update background.
		* @param	quantity	Refer to #BgQuantity  for setting values.
		* @return	#Result
		* @note
		* 	- Available from v2.0.0.
		*/
		Result		GetBackgroundQuantity(BgQuantity* quantity);

		/**
		* @brief
		*	Create capture file
		*
		*	- Create capture file.
		* @param	path		Path to save. (Finish with folder separator (\))
		*                       If it is blank, same path with .exe is set.
		* @param	filename	File name to save.
		* @return	#Result
		* @note
		*	- If already existing file is inputted, it will be overwritten.
		* 	- Available from v1.3.0.
		*/
		Result		CreateCaptureFile(CaptureInfo info);

		/**
		* @brief
		*	Start or stop capturing
		*
		*	- Start or stop capturing.
		*	- Capturing only if capture is stared and it is in Run status.
		* @param	bEnable		true: Start capturing.
		*                       false: Stop capturing.
		* @return	#Result
		* @note
		*	- Don't change settings for TOF sensor during capture is stopped.
		*	- Start again from CreateCaptureFile() if any setting for TOF sensor is changed.
		* 	- Available from v1.3.0.
		*/
		Result		Capture(bool bEnable);

		/**
		* @brief
		*	Get status of capturing
		*
		*	- Get status of capturing.
		* @param	status	Refer to #CaptureStatus for capture status.
		* @return	#Result
		* @note
		* 	Available from v1.3.0.
		*/
		Result		GetCaptureStatus(CaptureStatus* status);

		/**
		* @brief
		*	Pause/Resume of play
		*
		*	- Pause/Resume of play frame data in emulation mode.
		* @param	bPause		true: Pause
		*						false: Resume
		* @return	#Result
		* @note
		* 	Available from v2.0.0.
		*/
		Result		PauseEmulationTof(bool bPause);

	};

	/**
	* @brief
	* 	Class to manage all TOF sensors
	*
	* 	- Create at calling Tof::Open() for Tof instances.
	* 	- TofManager instance can be destructed after all Tof instances start to use TOF sensors by Tof::Open().
	*/
	class TofManager{
	protected:
		int numoftof;				///< Number of all TOF sensors
		vector<TofInfo>	tofinfo;	///< List of TOF information of all TOF sensors

	public:
		string inifilepath;			///< Folder of Initialization File(Default is "", Finish with '\' if it is indicated)
		string inifilename;			///< File name of Initialization File (Default is "tof.ini")
#ifdef	_TOFBUILT_IN
		bool bConsoleLog;			///< Enable/Disable console log by SDK
#else
		bool bConsoleLog = true;	///< Enable/Disable console log by SDK
#endif

		/**
		* @brief
		*	Constructor
		*/
		TofManager();

		/**
		* @brief
		*	Initialize Initialization File (Create as new)
		*
		*	- A new Initialization File is created which no TOF sensor is registered.
		*	- If old Initialization File remains, it is overwritten.
		*	- In default, Initialization File is saved to the folder where user application is, and name is "tof.ini".
		*	- #inifilepath member variable is changed if the save folder is changed. (Finish with file separator '\')
		*	- #inifilename member variable is changed if the file name of Initialization File is changed.
		* @return	#Result
		* @pre
		*	- The method shall be called while TofManager instance is in the close state.
		*	  (Before TofManager::Open() or after TofManager::Close())
		*	- Error is reported if it is called while TofManager instance is in open state.
		* @post
		*	- Initialization File is modified.
		*	- TofManager instance is not yet available with the initialized information even after this method is called
		*	  until TofManager::Open() method is called.
		*/
		Result Initialize(void);

		/**
		* @brief
		*	Start to use TofManager instance (Load Initialization File)
		*
		*	- Load Initialization File, and change TofManager instance to open state.
		*	- In default, Initialization File is save to the folder where user application is, and name is "tof.ini".
		*	- #inifilepath member variable is changed if the save folder is changed. (Finish with file separator '\')
		*	- #inifilename member variable is changed if the file name of Initialization File is changed.
		* @return	#Result
		*/
		Result Open(void);

		/**
		* @brief
		*	End to use TofManager instance
		*
		*	- Clear information loaded by TofManager::Open(), change TofManager instance to close state.
		* @return	#Result
		*/
		Result Close(void);

		/**
		* @brief
		*	Get TOF information list of all TOF sensors
		*
		*	- Get TOF information list of all TOF sensors which SDK manages.
		*	- Input an element of TOF information list to Tof::Open method of Tof class.
		* @param	toflist	Pointer to point a Pointer to point TofInfo type array.
		* @return	Number of TOF sensor (-1 is returned if TofManager instance is close state.)
		* @pre
		*	- TofManager instance shall be valid by TofManager::Open() method in advance.
		* @post
		*	- The output is a pointer to the list created in TofManager instance.
		*	  If TofManager instance is destructed or end to be used by TofManager::Close() instance,
		*	  contents of the list becomes invalid.
		*/
		int GetTofList(const TofInfo** toflist);

		/**
		* @brief
		*	Add a TOF sensor
		*
		*	- Add a new TOF sensor information to Initialization File, and SDK will manage it.
		* @param	tofmac		MAC address of the TOF sensor which is added
		* @param	tofip		IP address of the TOF sensor which is added
		* @param	rtp_port	RTP port number of the TOF sensor which is added
		* @return	#Result
		* @pre
		*	- The method shall be called while TofManager instance is in the open state (after TofManager::Open()).
		*	- Error is reported if it is called while TofManager instance is in close state.
		*	- Unique TOF ID is assigned to the newly added TOF sensor.
		* @post
		*	- Initialization File is modified.
		*/
		Result AddTof(TofMac tofmac, TofIp tofip, int rtp_port);

		/**
		* @brief
		*	Delete a TOF sensor
		*
		*	- Delete a TOF sensor information in Initialization File, and SDK will not manage it.
		* @param	tofid		TOF ID of the TOF sensor to be deleted
		* @return	#Result
		* @pre
		*	- Call this method after destruct Tof instance correspond to the TOF sensor.
		* @post
		*	- Initialization File is modified.
		*/
		Result DeleteTof(TofId tofid);
	};

	/**
	* @brief
	* 	Camera view coordinate
	*/
	struct CameraPos {
		float	nx;				///< Normarized x-coordinate (0.0 - 1.0)
		float	ny;				///< Normarized y-coordinate (0.0 - 1.0)
	};

	/**
	* @brief
	* 	Class to convert coordinate to camera view
	*
	* 	- Set TOF information by Tof::Open() method after creating instance.
	* 	- Get TOF information of all TOF sensor managed by SDK through TofManager::GetTofList() method.
	*/
	class Camera{
	private:
		PRIVATE_ATTRIBUTES

	public:

		/**
		* @brief
		* 	Constructor
		*/
		Camera();

		/**
		* @brief
		* 	Destructor
		*/
		~Camera();

		/**
		* @brief
		*	Set physical installation position of the camera
		*
		*	- Set physical installation position of the camera.
		*	- 3D rotation order is, Z-axis, Y-axis, and X-axis.
		* @param	x			x-world coordinate of the camera [mm]
		* @param	y			y-world coordinate of the camera [mm]
		* @param	z			z coordinate of the camera (-1 * height differences from TOF sensor) [mm]
		* @param	rx			X-axis rotation angle(0 to 359 degree)
		* @param	ry			Y-axis rotation angle(0 to 359 degree)
		* @param	rz			Z-axis rotation angle(0 to 359 degree)
		* @param	direction	Direction in world coordinate(the positive direction of Y-axis is 0degree and clockwise)
		* @param	fov_h		Horizontal FOV(degree)
		* @param	fov_v		Vertical FOV(degree)
		* @return	#Result
		*/
		Result		SetAttribute(float x,
			float y,
			float h,
			float rx,
			float ry,
			float rz,
			float direction,
			float fov_h,
			float fov_v
			);

		/**
		* @brief
		*	Get physical installation position of the camera
		*
		*	- Get physical installation position of the camera.
		* @param	x			x-world coordinate of the camera [mm]
		* @param	y			y-world coordinate of the camera [mm]
		* @param	z			z-world coordinate of the camera(-1 * height from floor) [mm]
		* @param	rx			X-axis rotation angle(0 to 359 degree)
		* @param	ry			Y-axis rotation angle(0 to 359 degree)
		* @param	rz			Z-axis rotation angle(0 to 359 degree)
		* @param	direction	Direction in world coordinate(the positive direction of Y-axis is 0degree and clockwise)
		* @param	fov_h		Horizontal FOV(degree)
		* @param	fov_v		Vertical FOV(degree)
		* @return	#Result
		*/
		Result		GetAttribute(float* x,
			float* y,
			float* z,
			float* rx,
			float* ry,
			float* rz,
			float* direction,
			float* fov_h,
			float* fov_v
			);

		/**
		* @brief
		* 	Convert a 3D world coordinate to a position on the camera view
		*
		* 	- Convert a 3D world coordinate to a normalized position on the camera view.
		* @param	point		A 3D point in world coordinate
		* @param	position	Pointer to output normalized position on the camera view.
		* @param	distance	Pointer to output distance between the point and the camera.
		* @return	#Result
		* @pre
		*	- Camera position in world coordinate must be set by SetAttribute() in advance.
		* @note
		*	- If distance is negative, the point is behind the camera.
		*/
		Result Convert(TofPoint point, CameraPos* position, float* distance);
	};

}
#endif //_hlds_H
