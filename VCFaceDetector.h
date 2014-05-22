#ifndef _FACE_DETECTOR_H
#define _FACE_DETECTOR_H

#include <vector>
#include <opencv2\opencv.hpp>

class CFaceDetector
{
private:
	//CImageDlg m_imageFaceDlg;
	//CImageDlg m_imageTestDlg;

	// Debug info 
	int HaarDBG;
	int EllipseDBG;
	int NoseDBG;
	int EyeDBG;
	int MouthDBG;
	int ptcMDBG;
	int ECHIWINDOWDBG;

	// Images
	IplImage* image;
	IplImage* gray;
	// IplImage* LKgray;
	IplImage* prev_gray;
	IplImage* m_CannyTpl;
	IplImage* m_CannyRes;
	IplImage* m_CannyImg;


	// For PackFaceParameters
	float dMouthToEyesLineDist_Div_EyesDist; 
	float dNoseToEyesLineDist_Div_EyesMouthDist; 
	float dProjectionMouthOnEyesLine;
	//float dEyesP3;
	float dLEyeMAngle;
	float dREyeMAngle;
	float dEyesDistance;
	//float dMouthNoseP;
	float dMouthLeye;
	float dMouthReye;

	float EPS;

	// Rotation Correction
	CvPoint3D64f Angles;
	float m_frotation_matrix[9];
	float m_ftranslation_vector[3];
	float m_fOtranslation_vector[3];

	// Reference for POSIT
	float m_fRefX;
	float m_fRefY;
	float m_fRefZ;
	
	// HaarCascade for Face features
	CvRect rectFaceCascade;
	CvRect rectLeyeCascade;
	CvRect rectReyeCascade;
	CvRect rectNoseCascade;
	CvRect rectMouthCascade;

	// Final points
	CvPoint ptcFace;
	CvPoint ptcLeye;
	CvPoint ptcReye;
	CvPoint ptcNose;
	CvPoint ptcMouth;

	// Return points
	CvPoint ptcRLeye;
	CvPoint ptcRReye;
	CvPoint ptcRNose;
	CvPoint ptcRMouth;

	// Detected points
	CvPoint ptcTFace;
	CvPoint ptcTLeye;
	CvPoint ptcTReye;
	CvPoint ptcTNose;
	CvPoint ptcTMouth;
	CvPoint ptcTNose1;

	// Predicted points (without Face point)
	CvPoint ptcMLeye;
	CvPoint ptcMReye;
	CvPoint ptcMNose;
	CvPoint ptcMMouth;

	// Service points
	CvPoint ptmEye;
	CvPoint ptmMEye;
	CvPoint ptcCenter;
	CvPoint ptcMCenter;
	CvPoint ptcTCenter;

	// For HaarCascades
	CvMemStorage* storage;
	CvHaarClassifierCascade* HeadCascade;
	CvHaarClassifierCascade* EyeCascadeL;
	CvHaarClassifierCascade* EyeCascadeR;
	CvHaarClassifierCascade* NoseCascade;
	CvHaarClassifierCascade* MouthCascade;
	bool bUseHaar;
	int  Stability;

	int iTormoz[20];
	float ModelNosePos;
	float ModelMouthPos;
	float Multipier;
	int StartX;

	// For face region detect
	int m_BAVG,m_GAVG,m_RAVG,m_GrAVG;
	double m_Gmean,m_Gsdv;
	double m_Rmean,m_Rsdv;
	double m_Bmean,m_Bsdv;
	double m_BaseMean,m_BaseSdv;
	double m_Smean,m_Ssdv;
	double m_Hmean,m_Hsdv;

	float CenterXparam;

	// For Eye Detect
	int CannyThresholdLeye;
	int CannyThresholdReye;
	IplImage* Bigtpl;
	int LDarkCircleCnt;
	int RDarkCircleCnt;
	int CornerCorCnt;
	long m_iEyeTplAV;
	float m_fLEyeHaarEPSMult;
	float m_fREyeHaarEPSMult;
	
	// LK Prediction
	static const int MAX_POINTS = 500;
	CvPoint2D32f points[2][MAX_POINTS];
	char status[MAX_POINTS];
	float feature_errors[MAX_POINTS];
	CvPoint LkPle,LkPre,LkPm;
	float m_fLKdelta;

	// Analize points
	bool bLeye;
	bool bReye;
	bool bMouth;
	bool bNose;
	bool bMeye;
	bool bPackFace;
	int iLKLEye;
	int iLKREye;
	int iLKMouth;
	bool m_bNeedVerticalFlip;

	// Main answer
	bool m_bDetected;

	//mutable win32::CCriticalSectionGuard m_DataLock;
	CCriticalSection  m_DataLock;
	HANDLE m_hReadyWork;

	// Some survey data
	CvSize m_FrameSize;

	std::vector<CvPoint3D32f> modelPoints;
	std::vector<CvPoint2D32f> imagePoints;
	std::vector<CvPoint3D32f> modelPointsLK;
	std::vector<CvPoint2D32f> imagePointsLK;

	float f_leftX;
	float f_rightX;
	float f_upY;
	float f_downY;

private:
	void	InitDbg(); 
	bool	GetSkinParameters (float FaceSize, int EyeAngle);
	bool	GetAdaptivSkinRGBMetod (CvRect& rect, float FaceSize, int EyeAngle, int& tx, int& ty, int& tw, int& th, IplImage** rImg);
	bool	GetAdaptivSkinHSVMetod (CvRect& rect, float FaceSize, int EyeAngle, int& tx, int& ty, int& tw, int& th, IplImage** rImg);
	bool	FindFace(float FaceSize, int EyeAngle);
	bool	GetPtmMEye (bool bWithMouth = true);

			// Geometry
	CvPoint GetCenterPoint (CvPoint p1, CvPoint p2);
	CvPoint GetBetweenPoint (CvPoint p1, CvPoint p2, float Part);
	int		GetDistance (CvPoint p1,CvPoint p2);
	int		GetDistance (int x1, int y1, int x2, int y2);
	int		GetEyesCorner(CvPoint pLE,CvPoint pRE);
	int		GetMouthSize (CvPoint pLE,CvPoint pRE,CvPoint pM);
	int		GetFaceSize ();

	bool	Cross2Lines (int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4,int* x,int* y);
	float	Point2Line (int x1,int y1,int x2,int y2,int x,int y);
	float	Point2Line (CvPoint P1,CvPoint P2,CvPoint P3);
	void	ProjectionPoint2Line (int x1,int y1,int x2,int y2,int px,int py,int* x,int* y);
	CvPoint ProjectionPoint2Line (CvPoint P1,CvPoint P2,CvPoint P3);
	void	RotatePoint( int* x, int* y, int Cx, int Cy, int Angle );

			// Some function on 8-bit (gray scale) images
	long	getAverImValue (IplImage* img);
	void	NormalizeImage (IplImage* img, int FromColor = 255, int ToColor = 0);
	uchar	GetMaxImValue  (IplImage* img);
	void	InvertImage (IplImage* img);
	int		GetMinBox (IplImage* img, int w, int h, int& rx, int& ry);
	int		FindDarkestCircle (IplImage* img, int minR, int maxR, int* retx, int* rety, int* retr);

	void	SaveImg (IplImage* img);

			// This function makes random point movement much softer
	bool	CorrectPoint (CvPoint& MainR, CvPoint& NewR, int MaxDelta, int ComDelta, int MinStep = 4, int TormozMax = 3, int Pnumber = 0);

			// Control rect on image edges
	bool	ControlRect (CvRect& rect, int w, int h);

			// Cut part of the image with a certain angle
	void	MakeItemsImg (IplImage** dstImg, IplImage* srcImg, CvRect rect, int Angle = 0, CvScalar FillColor = cvScalarAll(0), bool bSerious = false, int tx = 0, int ty = 0, int tw = 0, int th = 0);

			// Haar Cascade part
	int		Detect_Haar (IplImage* small_img, double scale, CvHaarClassifierCascade* cascade, CvRect& rect, CvSize size, bool FindMinimal = false);
	int		GetHaarFace(void);
	bool	GetHaar(bool bUseAllField = false);

			// Canny functions
	void	CannyPrepare (IplImage* img, int minR, int& CannyThreshold, unsigned int sx = 0, unsigned int sy = 0);
	bool	CheckCanny (int minR, int x, int y, unsigned int sx = 0, unsigned int sy = 0);
	void	CannyRelease ();

			// Detect stage
	bool	GetLEyePosition (float FaceSize, int EyeAngle);
	bool	GetREyePosition (float FaceSize, int EyeAngle);
	bool	GetREyeHistogram (float FaceSize, int EyeAngle);
	bool	GetLEyeHistogram (float FaceSize, int EyeAngle);
	bool	GetMouthPosition (float FaceSize, int EyeAngle);
	bool	GetNosePosition (float FaceSize, int EyeAngle);

			// Pack and extract part
	bool	PackFaceParameters (bool FirstTime = false);
	bool	ExtractNose (void);
	bool	ExtractMouth(void);
	bool	ExtractLeye (void);
	bool	ExtractReye (void);
	bool	AnalizeAndRecover3P (void);
			
			// Analize geometry
	bool	AnalizeGeometry (int FaceSize);

			// Optical flow prediction 
	void	PreparePoints (CvPoint2D32f* pts, CvPoint P, float PredSh);
	bool	AnalizeLKPoints (int Bnum, CvPoint& P, float delta, float PredSh);
	void	PredictPoints (void);

	bool	PredictPoints2Part1 (void);
	void	PredictPoints2Part2 (void);

			// 3D Recognition
	float	GetModelZ (float x, float y);
	void	getSinModel(float px, float py, float* fx, float* fy, float* fz);
	void	ModelToView (float in_x, float in_y, IplImage* frame, int* out_x, int* out_y, CvMatr32f rotation_matrix,CvVect32f translation_vector, bool bUseModel = true, float in_z = 0, bool bVariantA = false);
	bool	ViewToModel (int in_x, int in_y, IplImage* frame, float* out_x, float* out_y, CvMatr32f rotation_matrix,CvVect32f translation_vector);
	void	CalculateRange (CvMatr32f rotation_matrix,CvVect32f translation_vector);
	bool	PutPoints();
	void	AnalizePoints (CvMatr32f rotation_matrix, CvVect32f translation_vector);
	bool	UpdatePOSITdata (bool Analize = true);

			// Some service functions
	void	InitInternal (IplImage* frame, int w, int h);
	void	CreateGray ();
	void	PrepareImages (IplImage* frame);

public:
	CFaceDetector(bool bNeedVerticalFlip);
	~CFaceDetector(void);

	HANDLE			m_hFreeFrame;

	void			LoadData (std::wstring strDataPath);	
	void			LoadData (char* strDataPath);

	void			Stop (void);
	bool			Detect (IplImage* frame);
	void			SendFrame (IplImage* frame);

	bool			IsDetected (void);
	CvPoint			GetCenter (void);
	CvSize			GetSize (void);
	int				GetAngle (void);
	void			GetGlPositMatrix (double* GlPositMatrix);
	CvPoint3D64f	GetAngles (void); 
	CvPoint3D64f	GetTranslationVector (void);
	CvSize			GetFrameSize (void);
};

#endif // _FACE_DETECTOR_H
