#include "stdafx.h"
#include "math.h"
#include "VCFaceDetector.h"
#include "stdint.h"

#define EHCIFOCUS			602
#define MinFaceSize			40
#define DetectScale			2
#define M_PI				3.14159265358979323846
#define MaxDarkCircle		20
#define MaxLKUse			10
#define MaxCornerCor		10
#define CANNY_THRESH_AV		30
#define MIN_STABLE_ANGLE	2

const float SQRT_2 = float(sqrt(2.0));

CFaceDetector::CFaceDetector(bool bNeedVerticalFlip)
	: image(0)
	, gray(0)
	, prev_gray(0)
	, EPS(0.25)
	, storage(0)
	, HeadCascade(0)
	, EyeCascadeL(0)
	, EyeCascadeR(0)
	, NoseCascade(0)
	, MouthCascade(0)
	, m_bNeedVerticalFlip(bNeedVerticalFlip)
	, HaarDBG(0)
	, EllipseDBG(0)
	, NoseDBG(0)
	, EyeDBG(0)
	, MouthDBG(0)
	, ptcMDBG(0)
	, ECHIWINDOWDBG(0)
	, m_FrameSize(cvSize(0,0))
	, m_fLEyeHaarEPSMult(1)
	, m_fREyeHaarEPSMult(1)
{
	Bigtpl = cvCreateImage (cvSize(80,40),8,1);
	cvRectangle (Bigtpl,cvPoint(0,0),cvPoint(80,40),cvScalar(0,0,0),CV_FILLED);
	cvEllipse (Bigtpl,cvPoint(40,0),cvSize (28,28),0,180,360,cvScalar(255,255,255),15);

	m_hReadyWork = CreateEvent (NULL,true,true,NULL);
	m_hFreeFrame = CreateEvent (NULL,true,true,NULL);
}

CFaceDetector::~CFaceDetector(void)
{
	CloseHandle(m_hFreeFrame);
	CloseHandle(m_hReadyWork);

	if (image) cvReleaseImage (&image);
	if (Bigtpl) cvReleaseImage (&Bigtpl);
	if (gray) cvReleaseImage (&gray);
	if (prev_gray) cvReleaseImage (&prev_gray);
	if (storage) cvReleaseMemStorage (&storage);
	if (HeadCascade) cvReleaseHaarClassifierCascade (&HeadCascade);
	if (EyeCascadeL) cvReleaseHaarClassifierCascade (&EyeCascadeL);
	if (EyeCascadeR) cvReleaseHaarClassifierCascade (&EyeCascadeR);
	if (NoseCascade) cvReleaseHaarClassifierCascade (&NoseCascade);
	if (MouthCascade) cvReleaseHaarClassifierCascade (&MouthCascade);
	//if (ECHIWINDOWDBG) cvDestroyWindow("Face Window");
}

void CFaceDetector::InitDbg()
{
	if (ECHIWINDOWDBG) 
	{
		//m_imageFaceDlg.Create(IDD_IMGDLG,AfxGetApp()->m_pMainWnd);
		//m_imageFaceDlg.ShowWindow(SW_SHOW);

		//m_imageTestDlg.Create(IDD_IMGDLG,AfxGetApp()->m_pMainWnd);
		//m_imageTestDlg.ShowWindow(SW_SHOW);
	}

}

void CFaceDetector::LoadData (char* strDataPath)
{
	InitDbg();

	if (!storage) storage = cvCreateMemStorage(0);

	char strFile1[300] = "\0";
	char strFile2[300] = "\0";
	char strFile3[300] = "\0";
	char strFile4[300] = "\0";
	strcat(strFile1,strDataPath);
	strcat(strFile1,"\\haarcascade_frontalface_alt2.xml");
	strcat(strFile2,strDataPath);
	strcat(strFile2,"\\LEye.xml");
	strcat(strFile3,strDataPath);
	strcat(strFile3,"\\REye.xml");
	strcat(strFile4,strDataPath);
	strcat(strFile4,"\\Mouth.xml");
	HeadCascade  = (CvHaarClassifierCascade*)cvLoad(strFile1, 0,0,0 );
	EyeCascadeL  = (CvHaarClassifierCascade*)cvLoad(strFile2, 0,0,0 );
	EyeCascadeR  = (CvHaarClassifierCascade*)cvLoad(strFile3, 0,0,0 );
	MouthCascade = (CvHaarClassifierCascade*)cvLoad(strFile4, 0,0,0 );
}

void CFaceDetector::LoadData (std::wstring strDataPath)
{
	InitDbg();

	_wchdir(strDataPath.c_str());

	if (!storage) storage = cvCreateMemStorage(0);

	HeadCascade  = (CvHaarClassifierCascade*)cvLoad("haarcascades\\haarcascade_frontalface_alt2.xml", 0,0,0 );
	EyeCascadeL  = (CvHaarClassifierCascade*)cvLoad("haarcascades\\LEye.xml", 0,0,0 );
	EyeCascadeR  = (CvHaarClassifierCascade*)cvLoad("haarcascades\\REye.xml", 0,0,0 );
	//NoseCascade  = (CvHaarClassifierCascade*)cvLoad("haarcascades\\Nose.xml", 0,0,0 );
	MouthCascade = (CvHaarClassifierCascade*)cvLoad("haarcascades\\Mouth.xml", 0,0,0 );
}

void CFaceDetector::Stop (void)
{
	if (WaitForSingleObject(m_hReadyWork,1000) == WAIT_TIMEOUT)
		return;

	if (image) cvReleaseImage (&image);
	if (gray) cvReleaseImage (&gray);
	if (prev_gray) cvReleaseImage (&prev_gray);
	//if (ECHIWINDOWDBG) cvDestroyWindow("Face Window");
}

void CFaceDetector::CreateGray ()
{
	cvCvtColor (image,gray,CV_BGR2GRAY);

	//IplImage* yCrCb = cvCloneImage (image);
	//cvCvtColor (image,yCrCb,CV_BGR2YCrCb);
	//cvSplit (yCrCb,gray,0,0,0);
	//cvReleaseImage (&yCrCb);
}

void CFaceDetector::InitInternal (IplImage* frame, int w, int h)
{
	if (image) cvReleaseImage(&image);

	while (w>640) 
	{
		w-=160;
		h-=120;
	}
	while (w<400) 
	{
		w+=160;
		h+=120;
	}

	{
		m_DataLock.Lock();
		
		Multipier = (float)frame->height/h;
		m_FrameSize = cvSize (w,h);
		image		= cvCreateImage (m_FrameSize,8,3);
		gray		= cvCreateImage (m_FrameSize,8,1);
		prev_gray	= cvCreateImage (m_FrameSize,8,1);

		m_DataLock.Unlock();
	}

	cvResize (frame,image,CV_INTER_NN);
	if (m_bNeedVerticalFlip)
		cvFlip (image,image,0);
	CreateGray();
	cvCopy (gray,prev_gray);

	Angles.x = 180;
	Angles.y = 0;
	Angles.z = 0;
	m_fOtranslation_vector[0] = 0;
	m_fOtranslation_vector[1] = 0;
	m_fOtranslation_vector[2] = 0;

	bUseHaar = true;
	Stability = 1000;
	ModelNosePos = 0;
	ModelMouthPos = 0;

	ptcMNose = cvPoint(0,0);
	ptcMMouth = cvPoint(0,0);
	ptcMLeye = cvPoint(0,0);
	ptcMReye = cvPoint(0,0);

	dMouthToEyesLineDist_Div_EyesDist = 0; 
	dNoseToEyesLineDist_Div_EyesMouthDist = 0; 
	dProjectionMouthOnEyesLine = 0.5; 
	dEyesDistance = 0;
	dMouthLeye = 0;
	dMouthReye = 0;
	dLEyeMAngle = 70;
	dREyeMAngle = -70;

	CannyThresholdLeye = 0;
	CannyThresholdReye = 0;
	LDarkCircleCnt = 0;
	RDarkCircleCnt = 0;
	CornerCorCnt = 0;

	for (int i = 0; i < 20; i++) iTormoz[i] = 0;

	iLKLEye  = 0;
	iLKREye  = 0;
	iLKMouth = 0;

	//win32::CAutoLockCriticalSection lockingGuard(m_DataLock);
	m_DataLock.Lock();
	ptcRLeye  = cvPoint(0,0);
	ptcRReye  = cvPoint(0,0);
	ptcRNose  = cvPoint(0,0);
	ptcRMouth = cvPoint(0,0);
	m_bDetected = false;
	m_DataLock.Unlock();
}

void CFaceDetector::PrepareImages (IplImage* frame)
{
	win32::CAutoSetEvent eventGuard( m_hFreeFrame );

	float w = float(frame->width);
	float h = float(frame->height);
	float Ratio = (float)h/w;

	if (Ratio<0.65)
	{
		w = w * 3/4;
		win32::CAutoLockCriticalSection lockingGuard(m_DataLock);
			
		StartX = int(floor(0.5+(float(frame->width) - w)/2.0));
		cvSetImageROI (frame,cvRect (StartX,0,int(floor(0.5+w)) , int(floor(0.5+h)) ));
	}
	else 
	{
		win32::CAutoLockCriticalSection lockingGuard(m_DataLock);

		StartX = 0;
	}

	if (image) 
	{
		{
			win32::CAutoLockCriticalSection lockingGuard(m_DataLock);
			Multipier = h/image->height;
		}
		cvResize (frame,image,CV_INTER_NN);
		if (m_bNeedVerticalFlip)
			cvFlip (image,image,0);
		CreateGray();
	}
	else
		InitInternal (frame,int(floor(0.5+w)),int(floor(0.5+h)) );

	if (Ratio<0.65)
		cvResetImageROI (frame);
}

/////////////////////////////////////////////////////////////
//
// Geometry
//
/////////////////////////////////////////////////////////////

CvPoint CFaceDetector::GetCenterPoint (CvPoint p1, CvPoint p2)
{
	return cvPoint ((p1.x+p2.x)/2,(p1.y+p2.y)/2);
}

CvPoint CFaceDetector::GetBetweenPoint (CvPoint p1, CvPoint p2, float Part)
{
	//warning fix 9.12.12
	float nx = float(p2.x - p1.x);
	float ny = float(p2.y - p1.y);
	return cvPoint (int(floor(0.5+float(p1.x) + nx*Part)), int(floor(0.5+float(p1.y) + ny*Part)));
}

int CFaceDetector::GetDistance (int x1, int y1, int x2, int y2)
{
	int dx = x1 - x2;
	int dy = y1 - y2;
	return int(floor(0.5+sqrt(float(dx*dx + dy*dy))));
}

int CFaceDetector::GetDistance (CvPoint p1,CvPoint p2)
{
	int dx = p1.x - p2.x;
	int dy = p1.y - p2.y;
	return int(floor(0.5+sqrt(float(dx*dx + dy*dy))));
}

int CFaceDetector::GetMouthSize (CvPoint pLE,CvPoint pRE,CvPoint pM)
{
	int ptmx = (pLE.x + pRE.x) / 2;
	int ptmy = (pLE.y + pRE.y) / 2;
	int dx = ptmx - pM.x;
	int dy = ptmy - pM.y;
	return int(floor(0.5+sqrt(float(dx*dx + dy*dy))));
}

int CFaceDetector::GetEyesCorner(CvPoint pLE,CvPoint pRE)
{
	int dx = pLE.x - pRE.x;
	int dy = pLE.y - pRE.y;

	if (dx == 0) return (dy < 0) ? -90 : 90;
	return int(floor(0.5+atan (float(dy)/float(dx)) * float(180.0) / M_PI));
}

int CFaceDetector::GetFaceSize ()
{
	int EyeWidth = int(floor(0.5+double(GetDistance (ptcMLeye,ptcMReye)) * 1.15));
	int EyeMouth = GetMouthSize(ptcMLeye,ptcMReye,ptcMMouth);
	return (EyeWidth > EyeMouth) ? EyeWidth : EyeMouth;
}

bool CFaceDetector::Cross2Lines (int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4,int* x,int* y)
{
    float Z  = float((y2-y1)*(x3-x4)-(y3-y4)*(x2-x1));
    float Ca = float((y2-y1)*(x3-x1)-(y3-y1)*(x2-x1));
    float Cb = float((y3-y1)*(x3-x4)-(y3-y4)*(x3-x1));

    if( (Z == 0)&&(Ca == 0)&&(Cb == 0) )
        return false;

    if( Z == 0 )
        return false;

    float Ub = Cb/Z;

    *x = int(floor(0.5+x1 + (x2 - x1) * Ub));
    *y = int(floor(0.5+y1 + (y2 - y1) * Ub));

	return true;
}

float CFaceDetector::Point2Line (int x1,int y1,int x2,int y2,int x,int y)
{
	float dx = float(x2 - x1);
	float dy = float(y2 - y1);

	if (dx == 0)
		return abs(float(x-x1));

	return fabs(((x - x2)*dy - (y - y2)*dx) / sqrt(dx*dx + dy*dy));
}

float CFaceDetector::Point2Line (CvPoint P1,CvPoint P2,CvPoint P3)
{
	return Point2Line (P1.x,P1.y,P2.x,P2.y,P3.x,P3.y);
}

void CFaceDetector::ProjectionPoint2Line (int x1,int y1,int x2,int y2,int px,int py,int* x,int* y)
{
	int dx = x2 - x1;
	int dy = y2 - y1;

	if ((dx == 0) && (dy == 0))
	{
		*x = px;
		*y = py;
	}
	else
	{
		int px1 = px - dy;
		int py1 = py + dx;

		if (!Cross2Lines (x1,y1,x2,y2,px,py,px1,py1,x,y))
		{
			*x = px;
			*y = py;
		}
	}
}

CvPoint CFaceDetector::ProjectionPoint2Line (CvPoint P1,CvPoint P2,CvPoint P3)
{
	CvPoint RetP;
	ProjectionPoint2Line (P1.x,P1.y,P2.x,P2.y,P3.x,P3.y,&RetP.x,&RetP.y);
	return RetP;
}

void CFaceDetector::RotatePoint( int* x, int* y, int Cx, int Cy, int Angle )
{
	//warning fix 9.12.12
	
	float cos_angle = float(cos ((float)Angle*M_PI/180));
	float sin_angle = float( ((float)Angle*M_PI/180));

	int nx =int(floor(0.5+ float(Cx) + float((*x-Cx))*cos_angle - float((*y-Cy))*sin_angle));
	int ny = int(floor(0.5+float(Cy) + float(*y-Cy)*cos_angle + float(*x-Cx)*sin_angle));

	*x = nx;
	*y = ny;

	return;
}

bool CFaceDetector::ControlRect (CvRect& rect, int w, int h)
{
	bool res = false;
	if (rect.x < 0) { rect.x = 0; res = true; }
	if (rect.y < 0) { rect.y = 0; res = true; }
	if (rect.width  < 1) { rect.width  = 1; res = true; }
	if (rect.height < 1) { rect.height = 1; res = true; }
	if (rect.width  > w) { rect.width  = w; res = true; }
	if (rect.height > h) { rect.height = h; res = true; }
	if (rect.x + rect.width  > w) { rect.x = w - rect.width; res = true; }
	if (rect.y + rect.height > h) { rect.y = h - rect.height; res = true; }
	return res;
}

/////////////////////////////////////////////////////////////
//
// This function makes random point movement much softer
//
/////////////////////////////////////////////////////////////
bool CFaceDetector::CorrectPoint (CvPoint& MainR, CvPoint& NewR, int MaxDelta, int ComDelta, int MinStep, int TormozMax, int Pnumber)
{
	if (MainR.x == 0)
	{
		MainR = NewR;
		return false;
	}

	int Delta = abs(NewR.x -  MainR.x) + abs (NewR.y -  MainR.y);
	if (Delta/3 > ComDelta)
		return false;

	int dx = NewR.x - MainR.x;
	int dy = NewR.y - MainR.y;
	if (Pnumber > 19) Pnumber = 0;
	iTormoz[Pnumber]++;
	if (iTormoz[Pnumber] > TormozMax) iTormoz[Pnumber]=0;
	int iM = iTormoz[Pnumber] / TormozMax;

	if (abs(dx) > MaxDelta)
		MainR.x = (dx>0) ? NewR.x - MaxDelta : NewR.x + MaxDelta; 
	else if (dx > 0)
		MainR.x += (dx>MinStep)?dx/MinStep:1*iM;
	else if (dx < 0)
		MainR.x += (dx<-MinStep)?dx/MinStep:-1*iM;

	if (abs(dy) > MaxDelta)
		MainR.y = (dy>0) ? NewR.y - MaxDelta : NewR.y + MaxDelta;
	else if (dy > 0)
		MainR.y += (dy>MinStep)?dy/MinStep:1*iM;
	else if (dy < 0)
		MainR.y += (dy<-MinStep)?dy/MinStep:-1*iM;

	return true;
}

/////////////////////////////////////////////////////////////
//
// Cut part of the image with a certain angle
//
/////////////////////////////////////////////////////////////
void CFaceDetector::MakeItemsImg (IplImage** dstImg, IplImage* srcImg, CvRect rect, int Angle, CvScalar FillColor, bool bSerious, int tx, int ty, int tw, int th)
{
	// Hide mouth
	if (*dstImg) cvReleaseImage( dstImg );
	if (tw == 0) tw = rect.width - tx;
	if (th == 0) th = rect.height - ty;
	*dstImg = NULL;
	if (bSerious) // it means that we will cut part of the image, which has rotated angles
	{
		bool bNeedRelease = false;
		IplImage* sImg = 0;
		if (abs(Angle) > MIN_STABLE_ANGLE)
		{
			// Make image more than we need
			int cx = rect.x + rect.width / 2;
			int cy = rect.y + rect.height / 2;
			int ow = rect.width;
			int oh = rect.height;
			int w = ow * int(floor(0.5+double(SQRT_2)));
			int h = oh * int(floor(0.5+double(SQRT_2)));
			rect = cvRect (cx - w/2, cy - h/2, w, h);

			CvRect prevRect(rect);
			if (ControlRect(rect,gray->width,gray->height))
			{
				bNeedRelease = true;
				rect=prevRect;
				rect.x += srcImg->width/2;
				rect.y += srcImg->height/2;
				if (ControlRect(rect,gray->width*2,gray->height*2))
					return;
				sImg = cvCreateImage( cvSize(srcImg->width*2, srcImg->height*2), srcImg->depth, srcImg->nChannels );
				cvSet (sImg,FillColor);
				cvSetImageROI (sImg,cvRect(srcImg->width/2,srcImg->height/2,srcImg->width, srcImg->height));
				cvCopy (srcImg,sImg);
				cvResetImageROI(sImg);
			}
			else
				sImg = srcImg;

			IplImage* tmpImg = cvCreateImage( cvSize(w, h), sImg->depth, sImg->nChannels );
			cvSetImageROI (sImg,rect);
			// Rotate
			CvMat* Mat = cvCreateMat(2,3,CV_32FC1);
			cv2DRotationMatrix( cvPoint2D32f (w/2,h/2), Angle, 1, Mat );
			cvWarpAffine( sImg, tmpImg, Mat);
			cvReleaseMat (&Mat);
			// Cut
			int fx = (w - ow)/2;
			int fy = (h - oh)/2;
			if (((tx > 0) || (ty > 0) || (tw > 0) || (th > 0)) && ((tx + tw)<=ow) && ((ty + th)<=oh))
			{
				rect = cvRect (fx + tx, fy + ty, tw, th);
				ow = tw;
				oh = th;
			}
			else
				rect = cvRect (fx, fy, ow, oh);
			cvSetImageROI (tmpImg,rect);
			*dstImg = cvCreateImage( cvSize(ow, oh), sImg->depth, sImg->nChannels );
			cvCopy (tmpImg,*dstImg);
			cvReleaseImage (&tmpImg);
		}
		else 
		{
			int ow = rect.width;
			int oh = rect.height;
			int fx = rect.x;
			int fy = rect.y;
			if (((tx > 0) || (ty > 0) || (tw > 0) || (th > 0)) && ((tx + tw)<=ow) && ((ty + th)<=oh))
			{
				rect = cvRect (fx + tx, fy + ty, tw, th);
				ow = tw;
				oh = th;
			}
			else
				rect = cvRect (fx, fy, ow, oh);
			
			CvRect prevRect(rect);
			if (ControlRect(rect,gray->width,gray->height))
			{
				bNeedRelease = true;
				rect=prevRect;
				rect.x += srcImg->width/2;
				rect.y += srcImg->height/2;
				if (ControlRect(rect,gray->width*2,gray->height*2))
					return;
				sImg = cvCreateImage( cvSize(srcImg->width*2, srcImg->height*2), srcImg->depth, srcImg->nChannels );
				cvSet (sImg,FillColor);
				cvSetImageROI (sImg,cvRect(srcImg->width/2,srcImg->height/2,srcImg->width, srcImg->height));
				cvCopy (srcImg,sImg);
				cvResetImageROI(sImg);
			}
			else
			{
				sImg = srcImg;
			}

			cvSetImageROI (sImg,rect);
			*dstImg = cvCreateImage( cvSize(ow, oh), sImg->depth, sImg->nChannels );
			cvCopy (sImg,*dstImg);
		}

		if (bNeedRelease)
		{
			cvReleaseImage (&sImg);
		}
		else
		{
			cvResetImageROI(sImg);
		}
	}
	else
	{
		bool bNeedRelease = false;
		IplImage* sImg = 0;
		CvRect prevRect(rect);
		if (ControlRect(rect,gray->width,gray->height))
		{
			bNeedRelease = true;
			rect=prevRect;
			rect.x += srcImg->width/2;
			rect.y += srcImg->height/2;
			if (ControlRect(rect,gray->width*2,gray->height*2))
				return;
			sImg = cvCreateImage( cvSize(srcImg->width*2, srcImg->height*2), srcImg->depth, srcImg->nChannels );
			cvSet (sImg,FillColor);
			cvSetImageROI (sImg,cvRect(srcImg->width/2,srcImg->height/2,srcImg->width, srcImg->height));
			cvCopy (srcImg,sImg);
			cvResetImageROI(sImg);
		}
		else
		{
			sImg = srcImg;
		}

		*dstImg = cvCreateImage( cvSize(rect.width, rect.height), sImg->depth, sImg->nChannels );
		cvSetImageROI (sImg,rect);
		if (abs(Angle) > MIN_STABLE_ANGLE)
		{
			CvMat* Mat = cvCreateMat(2,3,CV_32FC1);
			cv2DRotationMatrix( cvPoint2D32f (rect.width / 2,rect.height / 2), Angle, 1, Mat );
			cvWarpAffine( sImg, *dstImg, Mat);
			cvReleaseMat (&Mat);
		}
		else
		{
			cvCopy (sImg,*dstImg);
		}
		if (bNeedRelease)
		{
			cvReleaseImage (&sImg);
		}
		else
			cvResetImageROI(sImg);
	}
}

/////////////////////////////////////////////////////////////
//
// Haar Cascade part
//
/////////////////////////////////////////////////////////////
int CFaceDetector::Detect_Haar (IplImage* small_img, double scale, CvHaarClassifierCascade* cascade, CvRect& rect, CvSize size, bool FindMinimal)
{
	cvClearMemStorage (storage);

	if (cascade && small_img)
	{
		CvSeq* Seq = cvHaarDetectObjects( small_img, cascade, storage,
			1.2, 2, (FindMinimal) ? 0 : CV_HAAR_FIND_BIGGEST_OBJECT, size );

		if (Seq->total > 0)
		{
			CvRect* r;
			if (FindMinimal)
			{
				r = (CvRect*)cvGetSeqElem( Seq, 0 );
				int MinArea = r->width * r->height;
				//warning fix 9/12/12
				rect = cvRect (int(floor(0.5+double(r->x)*scale)),
								int(floor(0.5+double(r->y)*scale)),
								int(floor(0.5+double(r->width)*scale)),
								int(floor(0.5+double(r->height)*scale)));

				for(int i = 1; i < Seq->total; i++ )
				{	
					r = (CvRect*)cvGetSeqElem( Seq, i );
					int CurrArea = r->width * r->height;
					if (MinArea>CurrArea)
					{
						MinArea = CurrArea;
						rect = cvRect (int(floor(0.5+double(r->x)*scale)),
										int(floor(0.5+double(r->y)*scale)),
										int(floor(0.5+double(r->width)*scale)), 
										int(floor(0.5+double(r->height)*scale)));
					}
				}
			}
			else
			{
				r = (CvRect*)cvGetSeqElem( Seq, 0 );
				rect = cvRect (int(floor(0.5+double(r->x)*scale)),
								int(floor(0.5+double(r->y)*scale)),
								int(floor(0.5+double(r->width)*scale)),
								int(floor(0.5+double(r->height)*scale)));
			}//(int(floor(0.5+double(r->x)*scale)),int(floor(0.5+double(r->y)*scale)),int(floor(0.5+double(r->width)*scale)), int(floor(0.5+double(r->height)*scale)) //r->width*scale,r->height*scale)
			return 1;
		}
	}
	return 0;
}

bool CFaceDetector::GetHaar(bool bUseAllField)
{
	if ( !storage ) 
		return false;

	int MinX = 0;
	int MinY = 0;
	int MaxX = gray->width;
	int MaxY = gray->height;
	if (!bUseAllField)
	{
		MinX = ptcMLeye.x;
		MinY = ptcMLeye.y;
		MaxX = MinX;
		MaxY = MinY;
		if (MinX > ptcMReye.x) MinX = ptcMReye.x;
		if (MinY > ptcMReye.y) MinY = ptcMReye.y;
		if (MaxX < ptcMReye.x) MaxX = ptcMReye.x;
		if (MaxY < ptcMReye.y) MaxY = ptcMReye.y;
		if (MinX > ptcMMouth.x) MinX = ptcMMouth.x;
		if (MinY > ptcMMouth.y) MinY = ptcMMouth.y;
		if (MaxX < ptcMMouth.x) MaxX = ptcMMouth.x;
		if (MaxY < ptcMMouth.y) MaxY = ptcMMouth.y;

		if (MaxX - MinX < 1) MaxX = MinX+1;
		if (MaxY - MinY < 1) MaxY = MinY+1;
		int dx = MaxX - MinX;
		int dy = MaxY - MinY;
		MinX -= dx/2;
		MinY -= dy/2;
		MaxX += dx/2;
		MaxY += dy;
		if (MinX < 0) MinX = 0; 
		if (MinY < 0) MinY = 0; 
		if (MaxX >= gray->width) MaxX = gray->width-1; 
		if (MaxY >= gray->height) MaxY = gray->height-1; 
	}
	CvRect rect = cvRect (MinX,MinY,MaxX-MinX,MaxY-MinY);
	IplImage* workImg = 0;
	MakeItemsImg (&workImg,gray,rect);
	if (!workImg) 
		return false;

	bool res = false;

	if (Detect_Haar(workImg,1,HeadCascade,rectFaceCascade,cvSize(MinFaceSize,MinFaceSize)))
	{
		rectFaceCascade.x += MinX;
		rectFaceCascade.y += MinY;
		rectLeyeCascade = cvRect (0,0,0,0);
		rectReyeCascade = cvRect (0,0,0,0);
		rectMouthCascade = cvRect (0,0,0,0);
		IplImage* face_img = 0;
		
		CvSize FaceItemsSize = cvSize(rectFaceCascade.width / 6, rectFaceCascade.height / 6);
		if (EyeCascadeL) 
		{
			MakeItemsImg (&face_img,gray,cvRect (rectFaceCascade.x, 
												rectFaceCascade.y + int(floor(0.5+float(rectFaceCascade.height) * 0.2)),
												int(floor(0.5+float(rectFaceCascade.width) * 0.6)),
												int(floor(0.5+float(rectFaceCascade.height) / 3)) ));
			if (Detect_Haar (face_img,1,EyeCascadeL,rectLeyeCascade,FaceItemsSize,true))
			{
				rectLeyeCascade.x += rectFaceCascade.x;
				rectLeyeCascade.y += rectFaceCascade.y + int(floor(0.5+double(rectFaceCascade.height) * 0.2));
				rectLeyeCascade.width = int(floor(0.5+double(rectLeyeCascade.width)*0.85));
			}
		}

		if (EyeCascadeR) 
		{
			MakeItemsImg (&face_img,gray,cvRect (rectFaceCascade.x + (int)(rectFaceCascade.width * 0.4), 
												rectFaceCascade.y + (int)(rectFaceCascade.height * 0.2),
												(int)(rectFaceCascade.width * 0.6),
												rectFaceCascade.height / 3));

			if (Detect_Haar (face_img,1,EyeCascadeR,rectReyeCascade,FaceItemsSize,true))
			{
				rectReyeCascade.x += rectFaceCascade.x + (int)(rectFaceCascade.width * 0.4) + (int)(rectReyeCascade.width * 0.15);
				rectReyeCascade.y += rectFaceCascade.y + (int)(rectFaceCascade.height * 0.2) + (int)(rectReyeCascade.height * 0.02);
				rectReyeCascade.width = (int) (rectReyeCascade.width * 0.85);
			}
		}

		if (MouthCascade) 
		{
			MakeItemsImg (&face_img,gray,cvRect (rectFaceCascade.x,
												rectFaceCascade.y + rectFaceCascade.height / 2,
												rectFaceCascade.width,
												rectFaceCascade.height / 2));
			if (Detect_Haar (face_img,1,MouthCascade,rectMouthCascade,FaceItemsSize))
			{
				rectMouthCascade.x += rectFaceCascade.x;
				rectMouthCascade.y += rectFaceCascade.y + rectFaceCascade.height / 2;
			}
		}

		if (face_img) cvReleaseImage( &face_img );

		if ((rectLeyeCascade.x != 0) && (rectReyeCascade.x != 0) && (rectMouthCascade.x != 0)) 
		{
			ptcMLeye  = ptcTLeye  = cvPoint(rectLeyeCascade.x + rectLeyeCascade.width / 2, rectLeyeCascade.y + rectLeyeCascade.height / 2);
			ptcMReye  = ptcTReye  = cvPoint(rectReyeCascade.x + rectReyeCascade.width / 2, rectReyeCascade.y + rectReyeCascade.height / 2);
			ptcMMouth = ptcTMouth = cvPoint(rectMouthCascade.x + rectMouthCascade.width / 2, rectMouthCascade.y + rectMouthCascade.height / 2);

			res = true;
		}
	}

	cvReleaseImage( &workImg );

	return res;
}

int CFaceDetector::GetHaarFace(void)
{
//	int iDelta;
	int iErrorC = 0;

	ptcTFace  = cvPoint (0,0);
	ptcTLeye  = cvPoint (0,0);
	ptcTReye  = cvPoint (0,0);
	ptcTNose  = cvPoint (0,0);
	ptcTMouth = cvPoint (0,0);

	if ( !storage ) return 0;

	IplImage* small_img;

	small_img = cvCreateImage( cvSize(gray->width/DetectScale, gray->height/DetectScale), 8, 1 );

	cvResize( gray, small_img, CV_INTER_NN );
	//cvEqualizeHist( small_img, small_img );

	rectFaceCascade = cvRect (0,0,0,0);

	if (Detect_Haar(small_img,DetectScale,HeadCascade,rectFaceCascade,cvSize(MinFaceSize,MinFaceSize)))
	{
		rectLeyeCascade = cvRect (0,0,0,0);
		rectReyeCascade = cvRect (0,0,0,0);
		rectNoseCascade = cvRect (0,0,0,0);
		rectMouthCascade = cvRect (0,0,0,0);
		IplImage* face_img = 0;
		
		CvSize FaceItemsSize = cvSize(rectFaceCascade.width / 6, rectFaceCascade.height / 6);

		if (EyeCascadeL) 
		{
			MakeItemsImg (&face_img,gray,cvRect (rectFaceCascade.x, 
												rectFaceCascade.y + (int)(rectFaceCascade.height * 0.2),
												(int)(rectFaceCascade.width * 0.6),
												rectFaceCascade.height / 3));
			if (Detect_Haar (face_img,1,EyeCascadeL,rectLeyeCascade,FaceItemsSize,true))
			{
				rectLeyeCascade.x += rectFaceCascade.x;
				rectLeyeCascade.y += rectFaceCascade.y + (int)(rectFaceCascade.height * 0.2);
				rectLeyeCascade.width = (int)(rectLeyeCascade.width * 0.85);
			}
		}

		if (EyeCascadeR) 
		{
			MakeItemsImg (&face_img,gray,cvRect (rectFaceCascade.x + (int)(rectFaceCascade.width * 0.4), 
												rectFaceCascade.y + (int)(rectFaceCascade.height * 0.2),
												(int)(rectFaceCascade.width * 0.6),
												rectFaceCascade.height / 3));

			if (Detect_Haar (face_img,1,EyeCascadeR,rectReyeCascade,FaceItemsSize,true))
			{
				rectReyeCascade.x += rectFaceCascade.x + (int)(rectFaceCascade.width * 0.4) + (int)(rectReyeCascade.width * 0.15);
				rectReyeCascade.y += rectFaceCascade.y + (int)(rectFaceCascade.height * 0.2) + (int)(rectReyeCascade.height * 0.02);
				rectReyeCascade.width = (int)(rectReyeCascade.width * 0.85);
			}
		}

		//if (NoseCascade) 
		//{
		//	int dx = rectFaceCascade.width / 4;
		//	MakeItemsImg (&face_img,gray,cvRect (rectFaceCascade.x + dx,
		//										rectFaceCascade.y + dx*1.8,
		//										rectFaceCascade.width / 2,
		//										dx*1.2));
		//	if (Detect_Haar (face_img,1,NoseCascade,rectNoseCascade,FaceItemsSize))
		//	{
		//		rectNoseCascade.x += rectFaceCascade.x + dx;
		//		rectNoseCascade.y += rectFaceCascade.y + dx * 1.8;
		//		rectNoseCascade.width *= 1.05;
		//	}
		//}

		if (MouthCascade) 
		{
			MakeItemsImg (&face_img,gray,cvRect (rectFaceCascade.x,
												rectFaceCascade.y + rectFaceCascade.height / 2,
												rectFaceCascade.width,
												rectFaceCascade.height / 2));
			if (Detect_Haar (face_img,1,MouthCascade,rectMouthCascade,FaceItemsSize))
			{
				rectMouthCascade.x += rectFaceCascade.x;
				rectMouthCascade.y += rectFaceCascade.y + rectFaceCascade.height / 2;
			}
		}

		cvReleaseImage( &small_img );
		if (face_img) cvReleaseImage( &face_img );

		if (rectLeyeCascade.x == 0) 
		{
			iErrorC++;
			ptcTLeye = ptcLeye;
		} else 
			ptcTLeye  = cvPoint(rectLeyeCascade.x + rectLeyeCascade.width / 2, rectLeyeCascade.y + rectLeyeCascade.height / 2);

		if (rectReyeCascade.x == 0) 
		{
			iErrorC++;
			ptcTReye = ptcReye;
		} else
			ptcTReye  = cvPoint(rectReyeCascade.x + rectReyeCascade.width / 2, rectReyeCascade.y + rectReyeCascade.height / 2);

		//if (rectNoseCascade.x == 0) 
		//{
		//	iErrorC++;
		//	ptcTNose = ptcNose;
		//} else
		//	ptcTNose  = cvPoint(rectNoseCascade.x + rectNoseCascade.width / 2, rectNoseCascade.y + rectNoseCascade.height / 2);

		ptcTNose  = cvPoint(rectFaceCascade.x + rectFaceCascade.width / 2, rectFaceCascade.y + (int)(rectFaceCascade.width *2.4 / 4));

		if (rectMouthCascade.x == 0) 
		{
			iErrorC++;
			ptcTMouth = ptcMouth;
		} else
			ptcTMouth = cvPoint(rectMouthCascade.x + rectMouthCascade.width / 2, rectMouthCascade.y + rectMouthCascade.height / 2);

		if ((ptcLeye.x == 0) && (iErrorC > 0))
			return 0;

		//iDelta = rectFaceCascade.width * 0.06;

		//ptcTFace = cvPoint(rectFaceCascade.x + rectFaceCascade.width / 2.02, rectFaceCascade.y + rectFaceCascade.height / 2);

		//CvPoint ptcOFace  = ptcFace;
		//CvPoint ptcOLeye  = ptcLeye;
		//CvPoint ptcOReye  = ptcReye;
		//CvPoint ptcONose  = ptcNose;
		//CvPoint ptcOMouth = ptcMouth;

		//int ComDelta  = abs(ptcTFace.x -  ptcFace.x)  + abs (ptcTFace.y -  ptcFace.y) +
		//				abs(ptcTLeye.x -  ptcLeye.x)  + abs (ptcTLeye.y -  ptcLeye.y) +
		//				abs(ptcTReye.x -  ptcReye.x)  + abs (ptcTReye.y -  ptcReye.y) +
		//				abs(ptcTNose.x -  ptcNose.x)  + abs (ptcTNose.y -  ptcNose.y) +
		//				abs(ptcTMouth.x - ptcMouth.x) + abs (ptcTMouth.y - ptcMouth.y);
		//ComDelta /= 5;

		//CorrectPoint (ptcFace,ptcTFace,iDelta,ComDelta);
		//CorrectPoint (ptcLeye,ptcTLeye,iDelta,ComDelta);
		//CorrectPoint (ptcReye,ptcTReye,iDelta,ComDelta);
		//CorrectPoint (ptcNose,ptcTNose,iDelta,ComDelta,2,1);
		//CorrectPoint (ptcMouth,ptcTMouth,iDelta,ComDelta);

		ptcLeye = ptcTLeye;
		ptcReye = ptcTReye;
		ptcNose = ptcTNose;
		ptcMouth = ptcTMouth;

		int StableCriteria = 20;

		if (ModelNosePos != 0)
			Stability = (int)(Stability*0.1) + iErrorC*1000;
		else
			Stability = (int)(Stability*0.6) + iErrorC*1000;

		if (Stability <= StableCriteria)
		{
			bUseHaar = false;
			return 1;
		}
		
		return 0;
	}
	else
	{
		cvReleaseImage( &small_img );
		return 0;
	}

	return 0;
}

/////////////////////////////////////////////////////////////
//
// Some function on 8-bit (gray scale) images
//
/////////////////////////////////////////////////////////////
long CFaceDetector::getAverImValue (IplImage* img)
{
	int imW = img->width;
	int imH = img->height;
	if ((imW == 0) || (imH == 0))
		return 0;

	long allVal = 0;
	int width = img->widthStep;
	uchar* ptr = (uchar*)img->imageData;

	for (int y=0; y<imH; y++ ) 
	{
		uchar* ptrIm = ptr;
		for( int x=0; x<imW; x++ )
		{
			allVal += *ptrIm;
			ptrIm++;
		}
		ptr += width;
	}

	return allVal / imH / imW;
}

uchar CFaceDetector::GetMaxImValue  (IplImage* img)
{
	if ((img->width == 0) || (img->height == 0))
		return 0;

	uchar MaxVal = 0;
	uchar* ptr = (uchar*)img->imageData;

	for (int y=0; y<img->height; y++ ) 
	{
		uchar* ptr1 = ptr;
		for( int x=0; x<img->width; x++ ) 
		{
			uchar Val = *ptr1;

			if (Val>MaxVal)
				  MaxVal = Val;

			ptr1++;
		}
		ptr += img->widthStep;
	}

	return MaxVal;
}

void CFaceDetector::InvertImage (IplImage* img)
{
	int imW = img->width;
	int imH = img->height;
	if ((imW == 0) || (imH == 0))
		return;

	int step = img->widthStep;
	uchar* ptr = (uchar*)img->imageData;

	for (int y=0; y<imH; y++ ) 
	{
		uchar* iptr = ptr;
		for( int x=0; x<imW; x++ ) 
		{
		  *iptr = 255 - *iptr;
		  iptr++;
		}
		ptr += step;
	}
}

void CFaceDetector::NormalizeImage (IplImage* img, int FromColor, int ToColor)
{
	int imW = img->width;
	int imH = img->height;
	if ((imW == 0) || (imH == 0))
		return;

	float MinVal = 255;
	float MaxVal = 0;
	int step = img->widthStep;
	uchar* ptr = (uchar*)img->imageData;

	for (int y=0; y<imH; y++ ) 
	{
		uchar* iptr = ptr;
		for( int x=0; x<imW; x++ ) 
		{
		  int Val = *iptr;
		  if (Val > FromColor)
		  {
			  Val = ToColor;
			  *iptr = ToColor;
		  }
		  if (Val>MaxVal)
			  MaxVal = (float)Val;
		  if (Val<MinVal)
			  MinVal = (float)Val;

		  iptr++;
		}
		ptr += step;
	}

	if (MaxVal>MinVal)
	{
		uchar* ptr = (uchar*)img->imageData;
		float delta = 255 / (MaxVal - MinVal);
		for (int y=0; y<imH; y++ ) 
		{
			uchar* iptr = ptr;
			for( int x=0; x<imW; x++ ) 
			{
				*iptr = (uchar)((*iptr - (uchar)MinVal) * delta);
				iptr++;
			}
			ptr += step;
		}
	}
}

int CFaceDetector::GetMinBox (IplImage* img, int w, int h, int& rx, int& ry)
{
	if (h == 0) h = 1;
	if (w == 0) w = 1;
	int retlv = 255;
	int lv;
	int divlv;
	int cnt = w*h;
	int imW = img->width;
	int imH = img->height;
	if ((h>=imH) || (w>=imW))
		return retlv;
	int width = img->widthStep;
	uchar* ptr = (uchar*)img->imageData;

	bool bFirstStep = true;
	bool bRightDir = true;
	int ex = imW - w;
	int ey = imH - h;
	for (int sy = 0; sy <= ey; sy++)
	{
		for (int sx = 0; sx <= ex; sx++)
		{
			if (bFirstStep)
			{
				bFirstStep = false;
				// Get first
				lv = 0;
				uchar* _ptr = ptr;
				for (int y = 0; y < h; y++)
				{
					uchar* iPtr = _ptr;
					for (int x = 0; x < w; x++)
					{
						lv += *iPtr;
						//*iPtr = 0; // debug
						iPtr++;
					}
					_ptr += width;
				}
			}
			else
			{
				if (sx == 0)
				{
					//Step down

					// Change direction
					bRightDir = !bRightDir;

					// Decrement prev
					uchar* imptr = ptr;
					for (int x = 0; x < w; x++)
					{
						lv -= *imptr;
						//*imptr = 255; // debug
						imptr++;
					}
					// Increment new
					imptr = ptr + h*width;
					for (int x = 0; x < w; x++)
					{
						lv += *imptr;
						//*imptr = 0; // debug
						imptr++;
					}
					// Shift down
					ptr += width;
				}
				else
				{
					// Usual Step

					// Decrement prev
					uchar* imptr = (bRightDir) ? ptr : ptr+w-1;
					for (int y = 0; y < h; y++)
					{
						lv -= *imptr;
						//*imptr = 255; // debug
						imptr += width;
					}
					// Shift to side
					ptr = (bRightDir) ? ptr+1 : ptr-1;
					// Increment new
					imptr = (bRightDir) ? ptr+w-1 : ptr;
					for (int y = 0; y < h; y++)
					{
						lv += *imptr;
						//*imptr = 0; // debug
						imptr += width;
					}
				}
			}

			divlv = lv/cnt;

			// debug
			//IplImage* img1 = cvCloneImage (img);
			//char number[100];
			//CvFont font;
			//cvInitFont (&font,CV_FONT_HERSHEY_PLAIN,1,1); 
			//_itoa (divlv,number,10);
			//cvPutText (img1,number,cvPoint(0,15),&font,cvScalarAll (125));
			//cvRectangle (img1,cvPoint(bRightDir ? sx : imW - sx - w,sy),cvPoint(bRightDir ? sx + w - 1 : imW - sx - 1,sy + h - 1),cvScalarAll(100));
			//SaveImg (img1);
			//cvReleaseImage (&img1);

			if (retlv > divlv )
			{
				retlv = divlv;
				rx = bRightDir ? sx : imW - sx - w;
				ry = sy;
			}
		}
	}
	
	return retlv;
}

//static bool bWantSave = false;
int	 CFaceDetector::FindDarkestCircle (IplImage* img, int minR, int maxR, int* retx, int* rety, int* retr)
{
	int bx,by,br;
	int imW = img->width;
	int imH = img->height;
	long retlv = 255;
	long lv;
	int  cnt;
	int shiftX[1000];
	int shiftY[1000];
	int shiftCount;
	int width = img->widthStep;
	for (br = minR; br<=maxR; br++)
	{
		shiftCount = 2*br;
		// create circle template 
	    int x = 0;
		int y = br;
		int delta = 3 - br - br;
	    while(x<y) 
		{
			shiftX[br-y] = x;
			shiftX[br+y] = x;
			shiftX[br-x] = y;
			shiftX[br+x] = y;
			shiftY[br-y] = x*width;
			shiftY[br+y] = x*width;
			shiftY[br-x] = y*width;
			shiftY[br+x] = y*width;
	        if (delta<0)
		        delta+=4*x+6;
			else 
			{
				delta+=4*(x-y)+10;
				y--;
			}
			x++;
		}
		shiftX[br-y] = x;
		shiftX[br+y] = x;
		shiftX[br-x] = y;
		shiftX[br+x] = y;
		shiftY[br-y] = x*width;
		shiftY[br+y] = x*width;
		shiftY[br-x] = y*width;
		shiftY[br+x] = y*width;

		bool bFirstStep = true;
		bool bRightDir = true;
		uchar* ptr = (uchar*)img->imageData + br;
		
		int rightX = (imW - br);
		int downY = (imH - br);
		for (by = br; by<downY; by++)
		{
			for (bx = br; bx<rightX; bx++)
			{
				if (bFirstStep)
				{
					bFirstStep = false;
					// Calculate first circle
					lv = 0;
					cnt = 0;
					uchar* ptrIm = ptr;
					for (int y = 0; y <= shiftCount; y++)
					{
						int w = shiftX[y];
						uchar* imptr = ptrIm - w;
						w *= 2;
						for (int x = 0; x <= w; x++)
						{
							lv += *imptr; 
							//*imptr = 255; // test
							imptr++;
							cnt++;
						}
						ptrIm += width;
					}
				}
				else
				{
					if (bx == br)
					{
						// Change direction
						bRightDir = !bRightDir;
						// Decrement prev
						uchar* imptr = ptr + br*width - br;
						for (int i = 0; i <= shiftCount; i++)
						{
							uchar* ptrIm = imptr - shiftY[i];
							lv -= *ptrIm;
							//*ptrIm = 0; // test
							imptr++;
						}
						// Shift down
						ptr += width;
						// Increment new
						imptr = ptr + br*width - br;
						for (int i = 0; i <= shiftCount; i++)
						{
							uchar* ptrIm = imptr + shiftY[i];
							lv += *ptrIm;
							//*ptrIm = 255; // test
							imptr++;
						}
					}
					else
					{	
						// Decrement prev
						uchar* imptr = ptr;
						for (int i = 0; i <= shiftCount; i++)
						{
							uchar* ptrIm = bRightDir ? imptr - shiftX[i] : imptr + shiftX[i];
							lv -= *ptrIm;
							//*ptrIm = 0; // test
							imptr += width;
						}
						// Shift left or right depend on direction
						ptr = bRightDir ? ptr+1 : ptr-1;
						// Increment new
						imptr = ptr;
						for (int i = 0; i <= shiftCount; i++)
						{
							uchar* ptrIm = bRightDir ? imptr + shiftX[i] : imptr - shiftX[i];
							lv += *ptrIm;
							//*ptrIm = 255; // test
							imptr += width;
						}
					}
				}
				
				long divLv = 255;
				if (cnt>0) divLv = lv/cnt;
				//if (bWantSave)
				//{
				//	IplImage* img1 = cvCloneImage (img);
				//	char number[100];
				//	CvFont font;
				//	cvInitFont (&font,CV_FONT_HERSHEY_PLAIN,1,1); 
				//	_itoa (divLv,number,10);
				//	cvPutText (img1,number,cvPoint(0,15),&font,cvScalarAll (125));
				//	cvCircle (img1,cvPoint(bRightDir ? bx : imW - bx,by),br+1,cvScalarAll (125));
				//	SaveImg (img1);
				//	cvReleaseImage (&img1);
				//}
				if (divLv<retlv)
				{
					retlv = divLv;
					*retx = bRightDir ? bx : imW - bx;
					*rety = by;
					*retr = br;
				}
			}
		}
	}
	return retlv;
}

/////////////////////////////////////////////////////////////
//
// Canny functions
//
/////////////////////////////////////////////////////////////
void CFaceDetector::CannyPrepare (IplImage* img, int minR, int& CannyThreshold, unsigned int sx, unsigned int sy)
{
	CvSize wSize = cvGetSize(img);
	m_CannyImg = cvCreateImage (wSize,8,1);
	int CANNY_THRESH = 70 - (minR-1) * 10;
	// Create cvCanny
	if (CannyThreshold == 0)
	{
		for (CannyThreshold = 250; CannyThreshold > 20; CannyThreshold--)
		{
			cvCanny (img,m_CannyImg,CannyThreshold,CannyThreshold/2);
			if (getAverImValue(m_CannyImg)>CANNY_THRESH)
				break;
		}
	}
	else
	{
		cvCanny (img,m_CannyImg,CannyThreshold,CannyThreshold/2);
		if (getAverImValue(m_CannyImg) < CANNY_THRESH) CannyThreshold -= 2; else CannyThreshold +=2;
	}

	if (EyeDBG)
	{
		CvRect rect;
		rect.x = sx;
		rect.y = sy;
		rect.width  = wSize.width;
		rect.height = wSize.height;
		ControlRect (rect,image->width,image->height);
		cvSetImageROI (image,rect);
		cvCvtColor (m_CannyImg,image,CV_GRAY2RGB);
		cvResetImageROI (image);
	}

	int csize = minR*5;
	m_CannyTpl = cvCreateImage (cvSize(csize,csize/2),8,1);
	m_CannyRes = cvCreateImage (cvSize(csize,csize/2),8,1);

	cvResize (Bigtpl,m_CannyTpl);

	m_iEyeTplAV = getAverImValue (m_CannyTpl);
}

bool CFaceDetector::CheckCanny (int minR, int x, int y, unsigned int sx, unsigned int sy)
{
	if (EyeDBG)
		cvCircle(image,cvPoint(x+sx,y+sy),3,cvScalar(0,255,0));

	bool res = true;

	int csize = m_CannyTpl->width;
	CvRect rtpl = cvRect(x - csize/2,y,csize,csize/2);
	if (!ControlRect (rtpl,m_CannyImg->width,m_CannyImg->height))
	{
		cvSetImageROI (m_CannyImg,rtpl);
		cvAnd(m_CannyImg,m_CannyTpl,m_CannyRes);
		cvResetImageROI (m_CannyImg);

		long resAV = getAverImValue(m_CannyRes);

		if ((resAV == 0) || ((float)m_iEyeTplAV/(float)resAV > (float)minR*0.9+1))
		{
			res = false;
		}
		else if (EyeDBG)
		{
			rtpl.x += sx + 55;
			rtpl.y += sy;
			ControlRect (rtpl,image->width,image->height);
			cvSetImageROI (image,rtpl);
			cvCvtColor (m_CannyRes,image,CV_GRAY2RGB);
			cvResetImageROI (image);
			cvEllipse (image,cvPoint(x + sx,y + sy),
				cvSize ((int)(minR*1.5), (int)(minR*1.5)),
				0,180,360,cvScalar(0,255,255),2);
		}
	}
	else
		res = false;

	return res;
}

void CFaceDetector::CannyRelease ()
{
	cvReleaseImage (&m_CannyTpl);
	cvReleaseImage (&m_CannyRes);
	cvReleaseImage (&m_CannyImg);
}

/////////////////////////////////////////////////////////////
//
// Detect stage
//
/////////////////////////////////////////////////////////////
bool CFaceDetector::GetLEyePosition (float FaceSize, int EyeAngle)
{
	CvRect Lrect;
	int x,y,r;
	
	int EyesSizeDevider = 20;

	// First Darkest circle mode
	IplImage* LEye_img = 0;
	//warning fix 9/12/12
	int cx = int(floor(0.5+FaceSize*0.3));
	int cy = int(floor(0.5+FaceSize*0.3));
	Lrect = cvRect(ptcMLeye.x - cx,ptcMLeye.y - cy, cx*2, cy*2);
	MakeItemsImg (&LEye_img,gray,Lrect,EyeAngle,cvScalarAll(255),true);
	if (!LEye_img)
		return false;
	
	//warning fix 9/12/12
	int minR = int(floor(0.5+FaceSize/(float)EyesSizeDevider));
	int maxR = int(floor(0.5+FaceSize/(float)EyesSizeDevider));
	bool bDarkRes = FindDarkestCircle (LEye_img,minR,maxR,&x,&y,&r) < 200;
	bool bCannyRes = false;
	if (bDarkRes && (minR > 1))
	{
		CannyPrepare(LEye_img,minR,CannyThresholdLeye,1,1);
		bCannyRes = CheckCanny (minR,x,y,1,1);
		CannyRelease();
	}
	else
		bCannyRes = bDarkRes;

	cvReleaseImage(&LEye_img);

	if (bDarkRes)
	{
		if (abs(EyeAngle) > MIN_STABLE_ANGLE)
			RotatePoint (&x,&y,cx,cy,EyeAngle);
	}
	else
	{
		x = 0;
		y = 0;
	}

	bool bHistRes = false;
	bool bHaarRes = false;

	if (bCannyRes)
	{
		ptcTLeye = cvPoint (x + Lrect.x,y + Lrect.y);
		if (GetDistance (ptcTLeye,ptcMLeye) < FaceSize*EPS) bHaarRes = true;
	}

	bHaarRes = bCannyRes;

	if (!bHaarRes) 
	{
		// Second Haar Cascade mode
		CvSize FaceItemsSize = cvSize((int)(FaceSize / 3), (int)(FaceSize / 3));
		IplImage* face_img = 0;
		CvRect Hrect;
		if (EyeCascadeL) 
		{
			MakeItemsImg (&face_img,gray,cvRect (int(floor(0.5+float(ptcMLeye.x) - FaceSize / 2.0)), 
												 int(floor(0.5+float(ptcMLeye.y) - FaceSize / 2.0)), 
												 int(floor(0.5+FaceSize)),
												 int(floor(0.5+FaceSize)) ),EyeAngle);
			if (Detect_Haar (face_img,1,EyeCascadeL,Hrect,FaceItemsSize,true))
			{
				if (EyeDBG)
				{
					CvRect DBGr = cvRect(80,1,Hrect.width,Hrect.height);
					ControlRect(DBGr,image->width,image->height);
					cvSetImageROI (image,DBGr);
					cvSetImageROI (face_img,Hrect);
					cvCvtColor (face_img,image,CV_GRAY2RGB);
					cvResetImageROI (image);
				}
				int Hx = Hrect.x + Hrect.width / 2;
				int Hy = Hrect.y + Hrect.height / 2;
				if (abs(EyeAngle) > MIN_STABLE_ANGLE)
					RotatePoint (&Hx,&Hy, (int)(FaceSize / 2), (int)(FaceSize / 2),EyeAngle);
				Hx += ptcMLeye.x - (int)(FaceSize / 2);
				Hy += ptcMLeye.y - (int)(FaceSize / 2);
				ptcTLeye = cvPoint(Hx,Hy);
				bHaarRes = GetDistance (ptcTLeye,ptcMLeye) < FaceSize*EPS*m_fLEyeHaarEPSMult;
			}
			cvReleaseImage (&face_img);
			m_fLEyeHaarEPSMult = bHaarRes ? 1 : m_fLEyeHaarEPSMult*float(1.1);
			if (m_fLEyeHaarEPSMult > 3) m_fLEyeHaarEPSMult = 3;
		}
	}
	else
		m_fLEyeHaarEPSMult = 1;
	
	bHistRes = bHaarRes;

	if (!bHistRes)
	{
		// Third Histogramm mode
		bHistRes = GetLEyeHistogram (FaceSize,EyeAngle);
	}

	if (bHistRes)
	{
		LDarkCircleCnt = 0;
		return true;
	}
	else if (x != 0)
	{
		LDarkCircleCnt++;
		if (LDarkCircleCnt < MaxDarkCircle)
		{
			ptcTLeye = cvPoint (x + Lrect.x,y + Lrect.y);
			if (GetDistance (ptcTLeye,ptcMLeye) < FaceSize*EPS/2) return true;
		}
	}	

	return false;
}

bool CFaceDetector::GetREyePosition (float FaceSize, int EyeAngle)
{
	CvRect Rrect;
	int x,y,r;
	
	int EyesSizeDevider = 20;

	// First Darkest circle mode plus Canny template confirmation
	IplImage* REye_img = 0;
	int cx = int(floor(0.5+FaceSize*0.3));
	int cy = int(floor(0.5+FaceSize*0.3));
	Rrect = cvRect(ptcMReye.x - cx,ptcMReye.y - cy, cx*2, cy*2);
	MakeItemsImg (&REye_img,gray,Rrect,EyeAngle,cvScalarAll(255),true);
	if (!REye_img)
		return false;

	int minR = int(floor(0.5+FaceSize/(float)EyesSizeDevider));
	int maxR = int(floor(0.5+FaceSize/(float)EyesSizeDevider));
	bool bDarkRes = FindDarkestCircle (REye_img,minR,maxR,&x,&y,&r) < 200;
	bool bCannyRes = false;
	if (bDarkRes && (minR > 1))
	{
		CannyPrepare(REye_img,minR,CannyThresholdReye,1,150);
		bCannyRes = CheckCanny (minR,x,y,1,150);
		CannyRelease();
	}
	else
		bCannyRes = bDarkRes;

	cvReleaseImage(&REye_img);

	if (bDarkRes)
	{
		if (abs(EyeAngle) > MIN_STABLE_ANGLE)
			RotatePoint (&x,&y,cx,cy,EyeAngle);
	}
	else
	{
		x = 0;
		y = 0;
	}

	bool bHistRes = false;
	bool bHaarRes = false;

	if (bCannyRes)
	{
		ptcTReye = cvPoint (x + Rrect.x,y + Rrect.y);
		if (GetDistance (ptcTReye,ptcMReye) < FaceSize*EPS) bHaarRes = true;
	}

	bHaarRes = bCannyRes;

	if (!bHaarRes) 
	{
		// Second Haar Cascade mode
		CvSize FaceItemsSize = cvSize(int(floor(0.5+FaceSize / 3.0)), int(floor(0.5+FaceSize / 3.0)));
		IplImage* face_img = 0;
		CvRect Hrect;
		if (EyeCascadeR) 
		{
			MakeItemsImg (&face_img,gray,cvRect (ptcMReye.x - int(floor(0.5+FaceSize / 2.0)), ptcMReye.y - int(floor(0.5+FaceSize / 2.0)), int(floor(0.5+FaceSize)), int(floor(0.5+FaceSize)) ),EyeAngle);
			if (Detect_Haar (face_img,1,EyeCascadeR,Hrect,FaceItemsSize,true))
			{
				if (EyeDBG)
				{
					CvRect DBGr = cvRect(80,150,Hrect.width,Hrect.height);
					ControlRect(DBGr,image->width,image->height);
					cvSetImageROI (image,DBGr);
					cvSetImageROI (face_img,Hrect);
					cvCvtColor (face_img,image,CV_GRAY2RGB);
					cvResetImageROI (image);
				}
				int Hx = Hrect.x + Hrect.width / 2;
				int Hy = Hrect.y + Hrect.height / 2;
				if (abs(EyeAngle) > MIN_STABLE_ANGLE)
					RotatePoint (&Hx,&Hy,(int)(FaceSize / 2), (int)(FaceSize / 2), EyeAngle);
				Hx += ptcMReye.x - (int)(FaceSize / 2);
				Hy += ptcMReye.y - (int)(FaceSize / 2);
				ptcTReye = cvPoint(Hx,Hy);
				bHaarRes = GetDistance (ptcTReye,ptcMReye) < FaceSize*EPS*m_fREyeHaarEPSMult;
			}
			cvReleaseImage (&face_img);
			m_fREyeHaarEPSMult = bHaarRes ? 1 : (float)(m_fREyeHaarEPSMult*1.1);
			if (m_fREyeHaarEPSMult > 3) m_fREyeHaarEPSMult = 3;
		}
	}
	else
		m_fREyeHaarEPSMult = 1;

	bHistRes = bHaarRes;

	if (!bHistRes)
	{
		// Third Histogramm mode
		 bHistRes = GetREyeHistogram (FaceSize,EyeAngle);
	}

	if (bHistRes)
	{
		RDarkCircleCnt = 0;
		return true;
	}
	else if (x != 0)
	{
		RDarkCircleCnt++;
		if (RDarkCircleCnt < MaxDarkCircle)
		{
			ptcTReye = cvPoint (x + Rrect.x,y + Rrect.y);
			if (GetDistance (ptcTReye,ptcMReye) < FaceSize*EPS/2) return true;
		}
	} 

	return false;
}

bool CFaceDetector::GetLEyeHistogram (float FaceSize, int EyeAngle)
{
	IplImage* LEye_img = 0;
	int Size = (int)(FaceSize*0.6);
	int pupilSize = ( Size > 20 ) ? (int)(Size*0.05) : 1;
	int lx = ptcMLeye.x - Size/2;
	int ly = ptcMLeye.y - (int)(Size/1.7);
	float delta = GetDistance (ptcMLeye,ptcMReye)/(float)(FaceSize*0.8);
	int tx = 0;
	if (delta < 1)
	{
		int tw = (int)(Size * delta);
		tx = (Size - tw)/2;
		MakeItemsImg (&LEye_img,gray,cvRect (lx, ly, Size, Size), EyeAngle, cvScalarAll(0), true, tx, 0, tw, Size);
	}
	else
		MakeItemsImg (&LEye_img,gray,cvRect (lx, ly, Size, Size), EyeAngle, cvScalarAll(0), true);
	
	if (LEye_img)
	{
		NormalizeImage (LEye_img);

		int imW = LEye_img->width;
		int imH = LEye_img->height;
		int wStep = LEye_img->widthStep;
		uchar* ptr = (uchar*)LEye_img->imageData;

		// Get Vertical histogram
		int HistogramV[1000];
		BYTE HistogramGausV[1000];
		int MaxHY = 0;
		int MinHY = 1000000;

		for (int y = 0; y < imH; y++)
		{
			float Lsum = 0;
			uchar* ptrX = ptr;
			for (int x = 0; x < imW; x++, ptrX++)
			{
				uchar val = *ptrX;
				int val1 = 255-val;
				Lsum += val1*val1;
			}
			//warning fix 9/12/12
			if (MaxHY < int(floor(0.5+Lsum))) MaxHY = int(floor(0.5+Lsum));
			if (MinHY > int(floor(0.5+Lsum))) MinHY = int(floor(0.5+Lsum));
			HistogramV[y] = int(floor(0.5+Lsum));
			ptr += wStep;
		}

		// resizing to byte size (max value = 255)
		for (int y = 0; y < imH; y++)
			HistogramGausV[y] = (MaxHY - HistogramV[y]) * 255 / MaxHY;

		MinHY = 0;
		MaxHY = 255;

		// Smooth vertical histogram
		CvMat VGaus = cvMat(1,imH,CV_8U,HistogramGausV);
		cvSmooth (&VGaus,&VGaus,CV_GAUSSIAN,9);

		// Get two minimums
		int StartVal = MinHY+1;
		int p1 = -1, p2 = -1;
		int dCount = 0;
		
		while (dCount < 4)
		{
			dCount = 0;
			int inVal,outVal;
			for (int y = imH-1; y >= 0; y--)
			{
				if (dCount%2 == 0)
				{
					if (HistogramGausV[y]<StartVal)
					{
						dCount++;
						outVal = y;
					}
				}
				else if (HistogramGausV[y]>StartVal)
				{
					dCount++;
					inVal = y;
					if (p1 == -1) 
					{
						p1 = (inVal+outVal)/2;
					}
					else if ((p1 < inVal) || (p1 > outVal))
					{
						if (p2 == -1) p2 = (inVal+outVal)/2;
					}
					if (dCount == 4) 
						break;
				}
			}	

			StartVal ++;
			if (StartVal > MaxHY)
				break;
		}

		if (p1 > p2)
		{
			int t = p1;
			p1 = p2;
			p2 = t;
		}
		
		int xPos = -1;
		if (p2 >= 0)
		{
			// Get horizontal histogram
			int HistogramH[1000];
			BYTE HistogramGausH[1000];
			int MinHX = 1000000;
			int MaxHX = 1;
			int MidHX = 0;
			int startY = p2 - pupilSize; if (startY < 0) startY = 0;
			int endY   = p2 + pupilSize; if (endY >= imH)  endY = imH;
			ptr = (uchar*)LEye_img->imageData + startY*wStep;
			for (int x = 0; x < imW; x++)
				HistogramH[x] = 0;

			for (int y = startY; y < endY; y++)
			{
				uchar* ptrX = ptr;
				for (int x = 0; x < imW; x++, ptrX++)
					HistogramH[x] += *ptrX;	

				ptr += wStep;
			}

			// Get Max value
			for (int x = 0; x < imW; x++)
				if (MaxHX < HistogramH[x]) MaxHX = HistogramH[x];

			// resizing to byte size (max value = 255)
			for (int x = 0; x < imW; x++)
				HistogramGausH[x] = HistogramH[x] * 255 / MaxHX;

			// Smooth horizontal histogram
			CvMat HGaus = cvMat(1,imW,CV_8U,HistogramGausH);
			cvSmooth (&HGaus,&HGaus,CV_GAUSSIAN,15);

			// Get X pos with minimal histogram value
			for (int x = 0; x < imW; x++)
			{
				if (MinHX > HistogramGausH[x]) 
				{
					MinHX = HistogramGausH[x];
					xPos = x;
				}
				MidHX += HistogramGausH[x];
			}

			MidHX = (int)(MidHX /(imW*0.9));

			if (EyeDBG) 
			{
				for (int x = 0; x < imW; x++)
					cvLine (LEye_img,cvPoint (x,0),cvPoint(x,HistogramGausH[x]*imW/600),cvScalar(0));

				for (int y = 0; y < imH; y++)
					cvLine (LEye_img,cvPoint (0,y),cvPoint(HistogramGausV[y]*imW/600,y),cvScalar(0));

				cvLine (LEye_img,cvPoint (xPos,0),cvPoint(xPos,imH),cvScalar(255));
				cvLine (LEye_img,cvPoint (0,p1),cvPoint(imW,p1),cvScalar(255));
				cvLine (LEye_img,cvPoint (0,p2),cvPoint(imW,p2),cvScalar(255));
				cvLine (LEye_img,cvPoint (0,startY),cvPoint(imW,startY),cvScalar(155));
				cvLine (LEye_img,cvPoint (0,endY),cvPoint(imW,endY),cvScalar(155));

				CvRect DBGr = cvRect(150,1,LEye_img->width,LEye_img->height);
				ControlRect(DBGr,image->width,image->height);
				cvSetImageROI (image,DBGr);
				cvCvtColor (LEye_img,image,CV_GRAY2RGB);
				cvResetImageROI (image);
			}
		}

		cvReleaseImage (&LEye_img);

		if (xPos >= 0)
		{
			xPos += tx;
			int resX = xPos;
			int resY = p2;

			if (abs(EyeAngle) > MIN_STABLE_ANGLE)
				RotatePoint (&resX,&resY,Size/2,Size/2,EyeAngle);

			resX += lx;
			resY += ly;

			ptcTLeye = cvPoint (resX,resY);
			if (GetDistance (ptcTLeye,ptcMLeye) < FaceSize*EPS)
				return true;
		}
	}
	return false;
}

bool CFaceDetector::GetREyeHistogram (float FaceSize, int EyeAngle)
{
	int Size = int(floor(0.5+FaceSize*0.6));
	int pupilSize = ( Size > 20 ) ? int(floor(0.5+double(Size)*0.05)) : 1;
	int lx = ptcMReye.x - int(floor(0.5+float(Size)/2));
	int ly = ptcMReye.y - int(floor(0.5+float(Size)/1.7));
	IplImage* REye_img = 0;
	float delta = GetDistance (ptcMLeye,ptcMReye)/(float)(FaceSize*0.8);
	int tx = 0;
	if (delta < 1)
	{
		int tw = (int)(Size * delta);
		tx = (Size - tw)/2;
		MakeItemsImg (&REye_img,gray,cvRect (lx, ly, Size, Size), EyeAngle, cvScalarAll(0), true, tx, 0, tw, Size);
	}
	else
		MakeItemsImg (&REye_img,gray,cvRect (lx, ly, Size, Size), EyeAngle, cvScalarAll(0), true);
	
	if (REye_img)
	{
		NormalizeImage (REye_img);

		int imW = REye_img->width;
		int imH = REye_img->height;
		int wStep = REye_img->widthStep;
		uchar* ptr = (uchar*)REye_img->imageData;

		// Get Vertical histogram
		int HistogramV[1000];
		BYTE HistogramGausV[1000];
		int MaxHY = 0;
		int MinHY = 1000000;

		for (int y = 0; y < imH; y++)
		{
			float Lsum = 0;
			uchar* ptrX = ptr;
			for (int x = 0; x < imW; x++, ptrX++)
			{
				uchar val = *ptrX;
				int val1 = 255-val;
				Lsum += val1*val1;
			}

			if (MaxHY < Lsum) MaxHY = (int)Lsum;
			if (MinHY > Lsum) MinHY = (int)Lsum;
			HistogramV[y] = (int)Lsum;
			ptr += wStep;
		}

		// resizing to byte size (max value = 255)
		for (int y = 0; y < imH; y++)
			HistogramGausV[y] = (MaxHY - HistogramV[y]) * 255 / MaxHY;

		MinHY = 0;
		MaxHY = 255;

		// Smooth vertical histogram
		CvMat VGaus = cvMat(1,imH,CV_8U,HistogramGausV);
		cvSmooth (&VGaus,&VGaus,CV_GAUSSIAN,9);

		// Get two minimums
		int StartVal = MinHY+1;
		int p1 = -1, p2 = -1;
		int dCount = 0;
		
		while (dCount < 4)
		{
			dCount = 0;
			int inVal,outVal;
			for (int y = imH-1; y >= 0; y--)
			{
				if (dCount%2 == 0)
				{
					if (HistogramGausV[y]<StartVal)
					{
						dCount++;
						outVal = y;
					}
				}
				else if (HistogramGausV[y]>StartVal)
				{
					dCount++;
					inVal = y;
					if (p1 == -1) 
					{
						p1 = (inVal+outVal)/2;
					}
					else if ((p1 < inVal) || (p1 > outVal))
					{
						if (p2 == -1) p2 = (inVal+outVal)/2;
					}
					if (dCount == 4) 
						break;
				}
			}	

			StartVal ++;
			if (StartVal > MaxHY)
				break;
		}

		if (p1 > p2)
		{
			int t = p1;
			p1 = p2;
			p2 = t;
		}
		
		int xPos = -1;
		if (p2 >= 0)
		{
			// Get horizontal histogram
			int HistogramH[1000];
			BYTE HistogramGausH[1000];
			int MinHX = 1000000;
			int MaxHX = 1;
			int MidHX = 0;
			int startY = p2 - pupilSize; if (startY < 0) startY = 0;
			int endY   = p2 + pupilSize; if (endY >= imH)  endY = imH;
			ptr = (uchar*)REye_img->imageData + startY*wStep;
			for (int x = 0; x < imW; x++)
				HistogramH[x] = 0;

			for (int y = startY; y < endY; y++)
			{
				uchar* ptrX = ptr;
				for (int x = 0; x < imW; x++, ptrX++)
					HistogramH[x] += *ptrX;	

				ptr += wStep;
			}

			// Get Max value
			for (int x = 0; x < imW; x++)
				if (MaxHX < HistogramH[x]) MaxHX = HistogramH[x];

			// resizing to byte size (max value = 255)
			for (int x = 0; x < imW; x++)
				HistogramGausH[x] = HistogramH[x] * 255 / MaxHX;

			// Smooth horizontal histogram
			CvMat HGaus = cvMat(1,imW,CV_8U,HistogramGausH);
			cvSmooth (&HGaus,&HGaus,CV_GAUSSIAN,15);

			// Get X pos with minimal histogram value
			for (int x = 0; x < imW; x++)
			{
				if (MinHX > HistogramGausH[x]) 
				{
					MinHX = HistogramGausH[x];
					xPos = x;
				}
				MidHX += HistogramGausH[x];
			}

			MidHX = (int)(MidHX / (imW*0.9)); //MidHX /= imW*0.9;

			if (EyeDBG) 
			{
				for (int x = 0; x < imW; x++)
					cvLine (REye_img,cvPoint (x,0),cvPoint(x,HistogramGausH[x]*imW/600),cvScalar(0));

				for (int y = 0; y < imH; y++)
					cvLine (REye_img,cvPoint (0,y),cvPoint(HistogramGausV[y]*imW/600,y),cvScalar(0));

				cvLine (REye_img,cvPoint (xPos,0),cvPoint(xPos,imH),cvScalar(255));
				cvLine (REye_img,cvPoint (0,p1),cvPoint(imW,p1),cvScalar(255));
				cvLine (REye_img,cvPoint (0,p2),cvPoint(imW,p2),cvScalar(255));
				cvLine (REye_img,cvPoint (0,startY),cvPoint(imW,startY),cvScalar(155));
				cvLine (REye_img,cvPoint (0,endY),cvPoint(imW,endY),cvScalar(155));

				CvRect DBGr = cvRect(150,150,REye_img->width,REye_img->height);
				ControlRect(DBGr,image->width,image->height);
				cvSetImageROI (image,DBGr);
				cvCvtColor (REye_img,image,CV_GRAY2RGB);
				cvResetImageROI (image);
			}
		}

		cvReleaseImage (&REye_img);

		if (xPos >= 0)
		{
			xPos += tx;
			int resX = xPos;
			int resY = p2;

			if (abs(EyeAngle) > MIN_STABLE_ANGLE)
				RotatePoint (&resX,&resY,Size/2,Size/2,EyeAngle);

			resX += lx;
			resY += ly;

			ptcTReye = cvPoint (resX,resY);
			if (GetDistance (ptcTReye,ptcMReye) < FaceSize*EPS)
				return true;
		}
	}
	return false;
}

bool CFaceDetector::GetMouthPosition (float FaceSize, int EyeAngle)
{
	float size = (float)(FaceSize*0.8);
	int tx = 0;
	int tw = (int)size;
	int ty = (int)(size*0.2);
	int th = (int)(size*0.6);
	IplImage* tImg = 0;

	CvRect rect = cvRect(ptcMMouth.x - (int)(size/2), ptcMMouth.y - (int)(size/2), (int)size, (int)size);

	MakeItemsImg (&tImg,image,rect,EyeAngle,cvScalarAll(255),true,tx,ty,tw,th);

	if (!tImg)
		return false;

	CvSize wSize = cvGetSize(tImg);
	IplImage* Gr = cvCreateImage (wSize,8,1);
	IplImage* Sb = cvCreateImage (wSize,8,1);

	cvCvtColor (tImg,Gr,CV_RGB2GRAY);

	cvFlip(Gr,Sb);
	cvSobel (Sb,Sb,0,1);
	cvFlip(Sb,Sb);

	NormalizeImage (Sb);
	//NormalizeImage (Gr);

	IplImage* workImg = cvCreateImage (wSize,8,1);

	int ResW = workImg->widthStep;
	int RgbW = tImg->widthStep;
	//int GrW = Gr->widthStep;
	int SbW = Sb->widthStep;
	uchar* ResPtr = (uchar*)workImg->imageData;
	uchar* RgbPtr = (uchar*)tImg->imageData;
	//uchar* GrPtr = (uchar*)Gr->imageData;
	uchar* SbPtr = (uchar*)Sb->imageData;

	for (int y=0; y<workImg->height; y++)
	{
		uchar* _ResPtr = ResPtr;
		uchar* _RgbPtr = RgbPtr;
		//uchar* _GrPtr  = GrPtr;	
		uchar* _SbPtr  = SbPtr;
		for (int x=0; x<workImg->width; x++)
		{
			int b  = *_RgbPtr; _RgbPtr++;
			int g  = *_RgbPtr; _RgbPtr++;
			int r  = *_RgbPtr; _RgbPtr++;
			int sb = *_SbPtr;  _SbPtr++;
			//int gr = *_GrPtr;  _GrPtr++;

			int divider = r + g + b;
			int res = (divider == 0) ? 0 : (255 - r * 255 / divider);
			res = (3*res + /*gr + */255 - sb) / 4;
			*_ResPtr = res; _ResPtr++;
		}
		ResPtr += ResW;
		RgbPtr += RgbW;
		//GrPtr += GrW;
		SbPtr += SbW;
	}

	NormalizeImage(workImg);

	if (MouthDBG)
	{
		CvRect r1 = cvRect(1,250,wSize.width,wSize.height);
		ControlRect (r1,image->width,image->height);
		cvSetImageROI (image,r1);
		cvCvtColor (workImg,image,CV_GRAY2RGB);
		cvResetImageROI (image);
	}

	cvReleaseImage (&tImg);
	cvReleaseImage (&Gr);
	cvReleaseImage (&Sb);

	int x1,y1;
	int bw = (int)(size*0.8);  if (bw == 0) bw = 1;
	int bh = bw/7;		if (bh == 0) bh = 1;
	int lv = GetMinBox (workImg,bw,bh,x1,y1);

	cvReleaseImage (&workImg);

	if (lv < 250)
	{
		x1 += bw/2;
		y1 += bh/2;

		if (MouthDBG) 
		{
			cvCircle (image,cvPoint (x1 + 1,y1 + 250),5,cvScalar(255,0,255));
			cvRectangle (image,cvPoint(x1-bw/2+1,y1-bh/2+250),cvPoint(x1+bw/2+1,y1+bh/2+250),cvScalarAll(255));
		}

		x1 += tx;
		y1 += ty;

		if (abs(EyeAngle) > MIN_STABLE_ANGLE)
			RotatePoint (&x1,&y1, (int)(size/2), (int)(size/2),EyeAngle);

		x1 += rect.x;
		y1 += rect.y;

		if (MouthDBG) 
			cvCircle (image,cvPoint(x1,y1),2,cvScalar(255,255,255));

		ptcTMouth = cvPoint (x1,y1);
		return GetDistance (ptcTMouth,ptcMMouth) < FaceSize*EPS;
	}
	return false;
}

bool CFaceDetector::GetNosePosition (float FaceSize, int EyeAngle)
{
	// Hide mouth
	ptmEye = GetCenterPoint (ptcMLeye,ptcMReye);
	CvPoint CpE = ptmEye;
	CvPoint Cp = GetCenterPoint (ptcMMouth,CpE);
	float eyemouth = (float)GetDistance(CpE,ptcMMouth);
	if (eyemouth == 0)
		return false;
	float size = (float)(FaceSize*0.8);
	int FillColor = (int)(m_GrAVG * 1.5);
	if (FillColor>250) FillColor=250;
	if (dNoseToEyesLineDist_Div_EyesMouthDist*1.1 < ModelNosePos)
	{
		float EllipseHval = size/4;
		cvEllipse (gray, 
			ptcMMouth,
			cvSize((int)(size/1.0), (int)EllipseHval), -EyeAngle, 0, 360, cvScalarAll(FillColor), CV_FILLED);
	}
	else
	{
		CvPoint pts[8];

		float dHx = (float)((ptcMReye.x - ptmEye.x)*1.6);
		float dHy = (float)((ptcMReye.y - ptmEye.y)*1.6);
		float dVx;
		float dVy;
		if (ModelNosePos != 0)
		{
			CvPoint P1 = GetBetweenPoint (ptmEye,ptcMMouth,dNoseToEyesLineDist_Div_EyesMouthDist);
			float vMult = (float) 0.4;
			dVx = (P1.x - ptcMMouth.x)*vMult;
			dVy = (P1.y - ptcMMouth.y)*vMult;
		}
		else
		{
			dVx = (float)((ptmEye.x - ptcMMouth.x)*0.08);
			dVy = (float)((ptmEye.y - ptcMMouth.y)*0.08);
		}
		pts[0].x = ptcMMouth.x + (int)(dHx*0.3 + dVx);
		pts[0].y = ptcMMouth.y + (int)(dHy*0.3 + dVy);
		pts[1].x = ptcMMouth.x + (int)(dHx + 2*dVx);
		pts[1].y = ptcMMouth.y + (int)(dHy + 2*dVy);
		pts[2].x = ptcMMouth.x + (int)(dHx - dVx);
		pts[2].y = ptcMMouth.y + (int)(dHy - dVy);
		pts[3].x = ptcMMouth.x + (int)(dHx*0.3 - 2*dVx);
		pts[3].y = ptcMMouth.y + (int)(dHy*0.3 - 2*dVy);
		pts[4].x = ptcMMouth.x - (int)(dHx*0.3 + 2*dVx);  //pts[4].x = ptcMMouth.x - dHx*0.3 - 2*dVx;
		pts[4].y = ptcMMouth.y - (int)(dHy*0.3 + 2*dVy);  //pts[4].y = ptcMMouth.y - dHy*0.3 - 2*dVy;
		pts[5].x = ptcMMouth.x - (int)(dHx + dVx);        //pts[5].x = ptcMMouth.x - dHx - dVx;
		pts[5].y = ptcMMouth.y - (int)(dHy + dVy);        //pts[5].y = ptcMMouth.y - dHy - dVy;
		pts[6].x = ptcMMouth.x - (int)(dHx - 2*dVx);      //pts[6].x = ptcMMouth.x - dHx + 2*dVx;
		pts[6].y = ptcMMouth.y - (int)(dHy - 2*dVy);      //pts[6].y = ptcMMouth.y - dHy + 2*dVy;
		pts[7].x = ptcMMouth.x - (int)(dHx*0.3 - dVx);    //pts[7].x = ptcMMouth.x - dHx*0.3 + dVx;
		pts[7].y = ptcMMouth.y - (int)(dHy*0.3 - dVy);    //pts[7].y = ptcMMouth.y - dHy*0.3 + dVy;     

		cvFillConvexPoly (gray,&pts[0],4,cvScalarAll(FillColor));
		cvFillConvexPoly (gray,&pts[4],4,cvScalarAll(FillColor));
		pts[1] = pts[3];
		pts[2] = pts[4];
		pts[3] = pts[7];
		cvFillConvexPoly (gray,&pts[0],4,cvScalarAll(FillColor));
	}

	//CvPoint CpE = ProjectionPoint2Line (ptcMLeye,ptcMReye,ptcMMouth);
	int Size = (int)eyemouth;
	int lx = Cp.x - Size/2;
	int ly = Cp.y - Size/2;
	int th;
	int ty;
	if (ModelNosePos == 0)
	{
		th = (int)(eyemouth*0.8);
		ty = (Size - th)/2;
	}
	else
	{
		th = (int)(eyemouth*0.7);
		ty = (int)((float)Size * (dNoseToEyesLineDist_Div_EyesMouthDist + ModelNosePos) / 2.0 - (float)th / 2.0);
		if (ty+th>Size) ty = Size - th;
		if (ty<0) ty = 0;
	}
	int tw = (int)eyemouth;
	int tx = (Size - tw)/2;

	IplImage* workImg = 0;
	MakeItemsImg (&workImg,gray,cvRect(lx,ly,Size,Size),EyeAngle,cvScalarAll(255),true,tx,ty,tw,th);
	if (!workImg)
		return false;

	int imW = workImg->width;
	int imH = workImg->height;
	int widthStep = workImg->widthStep;
	uchar* ptr = (uchar*)workImg->imageData;

	float Grad[1000];
	int AV = 0;
	float oldLsum;
	for (int y = 0; y < imH; y++)
	{
		uchar* iPtr = ptr;
		float Lsum = 0;
		for (int x = 0; x < imW; x++)
		{
			int val = *iPtr; 
			Lsum += val;
			iPtr++;
		}

		Lsum /= (float)imW;
		AV = (int)(AV + Lsum); //AV += Lsum;
		Grad[y] = (y>0) ? Lsum-oldLsum : 0;
		oldLsum = Lsum;
		ptr += widthStep;
	}
	AV /= imH;

	float MaxGrad = 0;
	int MaxGradY = 0;
	float gMult = (dNoseToEyesLineDist_Div_EyesMouthDist > ModelNosePos) ? float(3.0) : float(1.5);
	for (int y = 0; y < imH; y++)
		Grad[y] += Grad[y] * gMult * (float)y/(float)imH;

	//for (int y = imH; y < 2*imH; y++) Grad[y] = 0;
	CvMat VGaus = cvMat(1,imH/**1.5*/,CV_32F,Grad);
	cvSmooth (&VGaus,&VGaus,CV_GAUSSIAN,7);
	for (int y = 0; y < imH; y++)
	{
		if (Grad[y] < MaxGrad)
		{
			MaxGrad = Grad[y];
			MaxGradY = y;
		}
	}

	if (MaxGradY*100/imH < 10)
	{
		m_fLEyeHaarEPSMult = 3;
		m_fREyeHaarEPSMult = 3;
	}
	if (NoseDBG)
	{
		cvLine (workImg,cvPoint(0,MaxGradY),cvPoint(imW,MaxGradY),cvScalar(255,255,255));
		CvRect r1 = cvRect(image->width - imW - 1,0,imW,imH);
		ControlRect (r1,image->width,image->height);
		cvSetImageROI (image,r1);
		cvCvtColor (workImg,image,CV_GRAY2RGB);
		cvResetImageROI (image);
		for (int y = 0; y < imH; y++)
			cvLine (image,cvPoint (r1.x,y),cvPoint(r1.x+int(floor(0.5+Grad[y]*20.0/MaxGrad)),y),cvScalar(0,255,0));
	}
	cvReleaseImage (&workImg);

	int x1 = Size/2;
	int y1 = MaxGradY + ty;

	int x2 = x1 + (int)(1.5*CenterXparam);
	int y2 = y1;
	if (abs(EyeAngle)>1)
	{
		RotatePoint (&x1,&y1,Size/2,Size/2,EyeAngle);
		RotatePoint (&x2,&y2,Size/2,Size/2,EyeAngle);
	}

	x1 += lx;
	y1 += ly;
	x2 += lx;
	y2 += ly;

	ptcTNose = cvPoint (x1,y1);
	if (ModelNosePos != 0)
	{
		ptmEye = GetCenterPoint (ptcMLeye,ptcMReye);
		float nx = float(ptcMMouth.x - ptmEye.x);
		float ny = float(ptcMMouth.y - ptmEye.y);
		float Mdist = sqrt(float(nx*nx + ny*ny));
		if (Mdist == 0) Mdist = 1;
		float CenterYparam = Point2Line (ptcMLeye,ptcMReye,ptcTNose)/Mdist - ModelNosePos;
		if (CenterYparam < 0) 
			CenterYparam = (float)(CenterYparam * 1.7);  //CenterYparam *= 1.7; 
		else 
			CenterYparam = (float)(CenterYparam *1.3);  //CenterYparam *= 1.3;

		ptcTNose1 = cvPoint (x2 + (int)(nx*CenterYparam), y2 + (int)(ny*CenterYparam));

		if (GetDistance (ptmEye,ptcTNose1) > GetDistance (ptmEye,ptcMMouth))
		{
			m_fLEyeHaarEPSMult = 3;
			m_fREyeHaarEPSMult = 3;
		}
	}
	else
	{
		ptcTNose1 = ptcNose;
	}

	return true;
}

///////////////////////////////////////////////////////////////////
//
// Pack and extract part
//
///////////////////////////////////////////////////////////////////

bool CFaceDetector::PackFaceParameters (bool FirstTime)
{
	float FaceSize = (float)GetFaceSize();
		if (FaceSize == 0) 
			return false;
	float EyesDistance = (float)GetDistance (ptcMLeye,ptcMReye); 
		if (EyesDistance == 0) 
			return false;
	float MouthToEyesLine = Point2Line (ptcMLeye.x,ptcMLeye.y,ptcMReye.x,ptcMReye.y,ptcMMouth.x,ptcMMouth.y);
		if (MouthToEyesLine == 0) 
			return false;
	float NoseToEyesLine = Point2Line (ptcMLeye.x,ptcMLeye.y,ptcMReye.x,ptcMReye.y,ptcMNose.x,ptcMNose.y);
		if (NoseToEyesLine == 0) 
			return false;
	float MouthToNoseDistance = (float)GetDistance (ptcMMouth,ptcMNose); 
		if (MouthToNoseDistance == 0) 
			return false;

	float LEyeToMouthDistance = (float)GetDistance (ptcMLeye,ptcMMouth);
	float REyeToMouthDistance = (float)GetDistance (ptcMReye,ptcMMouth);

	dMouthToEyesLineDist_Div_EyesDist = MouthToEyesLine / EyesDistance;
	dNoseToEyesLineDist_Div_EyesMouthDist = NoseToEyesLine / MouthToEyesLine;

	dProjectionMouthOnEyesLine = sqrt (LEyeToMouthDistance*LEyeToMouthDistance - MouthToEyesLine*MouthToEyesLine) / EyesDistance;

	dLEyeMAngle = (float)(GetEyesCorner (ptcLeye,ptcReye) - GetEyesCorner (ptcLeye,ptcMouth));
	dREyeMAngle = (float)(GetEyesCorner (ptcReye,ptcLeye) - GetEyesCorner (ptcReye,ptcMouth));

	dEyesDistance = EyesDistance;

	dMouthLeye = LEyeToMouthDistance;
	dMouthReye = REyeToMouthDistance;

	if (FirstTime)
	{
		if (ModelMouthPos == 0) ModelMouthPos = dMouthToEyesLineDist_Div_EyesDist;

		if (ModelNosePos == 0) ModelNosePos = dNoseToEyesLineDist_Div_EyesMouthDist;
	}

	return true;
}

bool CFaceDetector::ExtractNose ()
{
	float EyesDistance = (float)GetDistance (ptcMLeye,ptcMReye); 
		if (EyesDistance == 0) return false;

	float NoseToEyesLine = dNoseToEyesLineDist_Div_EyesMouthDist * EyesDistance;

	float dx = (ptcMReye.x - ptcMLeye.x) * dProjectionMouthOnEyesLine;
	float dy = (ptcMReye.y - ptcMLeye.y) * dProjectionMouthOnEyesLine;

	float x = dx + ptcMLeye.x;
	float y = dy + ptcMLeye.y;
	
	float dist = sqrt(dx*dx+dy*dy);
	if (dist == 0) return false;

	dx /= dist;
	dy /= dist;

	CvPoint MNose = cvPoint ((int)(x - dy * NoseToEyesLine), (int)(y + dx * NoseToEyesLine));
	if ((MNose.x >=0) && (MNose.x < gray->width) && (MNose.y >= 0) && (MNose.y < gray->height))
	{
		ptcTNose = MNose;
		return true;
	}
	else
		return false;
}

bool CFaceDetector::ExtractMouth()
{
	float EyesDistance = (float)GetDistance (ptcMLeye,ptcMReye); 
		if (EyesDistance == 0) return false;

	float MouthToEyesLine = EyesDistance * dMouthToEyesLineDist_Div_EyesDist;

	float dx = (ptcMReye.x - ptcMLeye.x) * dProjectionMouthOnEyesLine;
	float dy = (ptcMReye.y - ptcMLeye.y) * dProjectionMouthOnEyesLine;

	float x = dx + ptcMLeye.x;
	float y = dy + ptcMLeye.y;
	
	float dist = sqrt(dx*dx+dy*dy);
	if (dist == 0) return false;

	dx /= dist;
	dy /= dist;

	CvPoint Mouth = cvPoint ((int)(x - dy * MouthToEyesLine), (int)(y + dx * MouthToEyesLine));
	if ((Mouth.x >=0) && (Mouth.x < gray->width) && (Mouth.y >= 0) && (Mouth.y < gray->height))
	{
		ptcTMouth = Mouth;
		return GetDistance (ptcMMouth,ptcTMouth) < EyesDistance*EPS;
	}
	else
		return false;
}

bool CFaceDetector::ExtractLeye ()
{
	float RM = (float)GetDistance(ptcMReye,ptcMMouth);
	if (RM == 0) return false;
	float eyeDist = dEyesDistance*RM/dMouthReye;
	float dx = (float)(ptcMMouth.x - ptcMReye.x);
	float dy = (float)(ptcMMouth.y - ptcMReye.y);
	dx = dx * eyeDist/RM;
	dy = dy * eyeDist/RM;
	ptcTLeye.x = ptcMReye.x + (int)dx;
	ptcTLeye.y = ptcMReye.y + (int)dy;
	RotatePoint (&ptcTLeye.x,&ptcTLeye.y,ptcMReye.x,ptcMReye.y,(int)dREyeMAngle);
	
	return GetDistance (ptcMLeye,ptcTLeye) < RM*EPS*2;
}

bool CFaceDetector::ExtractReye ()
{
	float LM = (float)GetDistance(ptcMLeye,ptcMMouth);
	if (LM == 0) return false;
	float eyeDist = dEyesDistance*LM/dMouthLeye;
	float dx = (float)(ptcMMouth.x - ptcMLeye.x);
	float dy = (float)(ptcMMouth.y - ptcMLeye.y);
	dx = dx * eyeDist/LM;
	dy = dy * eyeDist/LM;
	ptcTReye.x = ptcMLeye.x + (int)dx;
	ptcTReye.y = ptcMLeye.y + (int)dy;
	RotatePoint (&ptcTReye.x,&ptcTReye.y,ptcMLeye.x,ptcMLeye.y,(int)dLEyeMAngle);

	return GetDistance (ptcMReye,ptcTReye) < LM*EPS*2;
}

bool CFaceDetector::AnalizeAndRecover3P (void)
{
	if (bLeye && bReye)
	{
		ptcMLeye = ptcTLeye;
		ptcMReye = ptcTReye;
		bPackFace = false;
		bMouth = ExtractMouth();
		if (bMouth) 
		{
			ptcMMouth = ptcTMouth;
			if (ECHIWINDOWDBG) cvCircle (image,ptcMMouth,5,cvScalar (0,0,255),1);	
		}
	}
	else if (bMouth && bLeye)
	{
		ptcMLeye = ptcTLeye;
		ptcMMouth = ptcTMouth;
		bPackFace = false;
		bReye = ExtractReye ();
		if (bReye)
		{
			ptcMReye = ptcTReye;
			if (ECHIWINDOWDBG) cvCircle (image,ptcMReye,5,cvScalar (0,0,255),1);	
		}
	}
	else if (bMouth && bReye)
	{
		ptcMReye = ptcTReye;
		ptcMMouth = ptcTMouth;
		bPackFace = false;
		bLeye = ExtractLeye ();
		if (bLeye)
		{
			ptcMLeye = ptcTLeye;
			if (ECHIWINDOWDBG) cvCircle (image,ptcMLeye,5,cvScalar (0,0,255),1);	
		}
	}
	else if (!bLeye && !bReye)
	{
		iLKLEye++;
		iLKREye++;
		if ((iLKLEye < MaxLKUse) && (iLKREye < MaxLKUse))
		{
			if (LkPle.x != 0)
			{
				ptcTLeye = LkPle;
				ptcMLeye = ptcTLeye;
				if (ECHIWINDOWDBG) cvCircle (image,ptcMLeye,5,cvScalar (255,0,255),1);	
				bLeye = true;
			}
			if (LkPre.x != 0)
			{
				ptcTReye = LkPre;
				ptcMReye = ptcTReye;
				if (ECHIWINDOWDBG) cvCircle (image,ptcMReye,5,cvScalar (255,0,255),1);	
				bReye = true;
			}
		}
	}
	else if (!bMouth && !bLeye)
	{
		iLKLEye++;
		iLKMouth++;
		if ((iLKLEye < MaxLKUse) && (iLKMouth < MaxLKUse))
		{
			if (LkPm.x != 0)
			{
				ptcTMouth = LkPm;
				ptcMMouth = ptcTMouth;
				if (ECHIWINDOWDBG) cvCircle (image,ptcMMouth,5,cvScalar (255,0,255),1);	
				bMouth = true;
			}
			if (LkPle.x != 0)
			{
				ptcTLeye = LkPle;
				ptcMLeye = ptcTLeye;
				if (ECHIWINDOWDBG) cvCircle (image,ptcMLeye,5,cvScalar (255,0,255),1);	
				bLeye = true;
			}
		}
	}
	else if (!bMouth && !bReye)
	{
		iLKREye++;
		iLKMouth++;
		if ((iLKREye < MaxLKUse) && (iLKMouth < MaxLKUse))
		{
			if (LkPm.x != 0)
			{
				ptcTMouth = LkPm;
				ptcMMouth = ptcTMouth;
				if (ECHIWINDOWDBG) cvCircle (image,ptcMMouth,5,cvScalar (255,0,255),1);	
				bMouth = true;
			}
			if (LkPre.x != 0)
			{
				ptcTReye = LkPre;
				ptcMReye = ptcTReye;
				if (ECHIWINDOWDBG) cvCircle (image,ptcMReye,5,cvScalar (255,0,255),1);	
				bReye = true;
			}
		}
	}
		
	return bLeye && bReye && bMouth;
}

///////////////////////////////////////////////////////////////////
//
// 3D part
//
///////////////////////////////////////////////////////////////////

float CFaceDetector::GetModelZ (float x, float y)
{
	// TODO: Improve it. Make better.

	float resZ;
	float dx = (float)fabs(x - 0.5);

	resZ = 1 - dx*dx*dx*dx;

	return resZ;
}

void CFaceDetector::getSinModel(float px, float py, float* fx, float* fy, float* fz)
{
	*fx =  px;
	*fy = -py;
	*fz = GetModelZ (*fx,py); 
}

void CFaceDetector::ModelToView (float in_x, float in_y, IplImage* frame, int* out_x, int* out_y, CvMatr32f rotation_matrix,CvVect32f translation_vector, bool bUseModel, float in_z, bool bVariantA)
{
	float sx;
	float sy;
	float sz;
	float width = (float)frame->width;
	float height = (float)frame->height;
	float HalfWidth = width / 2;
	float HalfHeight = height / 2;
	float focalLength = EHCIFOCUS;
	float farPlane=10000.0;
	float nearPlane=1.0;

	if (!bUseModel)
	{
		sx = in_x;
		sy = in_y;
		sz = in_z;
	}
	else
		getSinModel (in_x,in_y,&sx,&sy,&sz);

	sx -= m_fRefX;
	sy -= m_fRefY;
	sz -= m_fRefZ;

	if (bVariantA)
	{
		float a[] = {   rotation_matrix[0], rotation_matrix[1], rotation_matrix[2], translation_vector[0],
						rotation_matrix[3], rotation_matrix[4], rotation_matrix[5], translation_vector[1],
						rotation_matrix[6], rotation_matrix[7], rotation_matrix[8], translation_vector[2],
						0,					0,					0,					1 };

		float projectionMatrix[16];

		projectionMatrix[0] = 2*focalLength/width;
		projectionMatrix[1] = 0.0;
		projectionMatrix[2] = 0.0;
		projectionMatrix[3] = 0.0;

		projectionMatrix[4] = 0.0;
		projectionMatrix[5] = 2*focalLength/height;
		projectionMatrix[6] = 0.0;
		projectionMatrix[7] = 0.0;

		projectionMatrix[8] = 0;
		projectionMatrix[9] = 0;
		projectionMatrix[10] = (float)(- ( farPlane+nearPlane ) / ( farPlane - nearPlane ));
		projectionMatrix[11] = (float)(-2.0 * farPlane * nearPlane / ( farPlane - nearPlane ));

		projectionMatrix[12] = 0.0;
		projectionMatrix[13] = 0.0;
		projectionMatrix[14]= -1.0;
		projectionMatrix[15] = 0.0;

		float pos[] = {0,0,0};

		CvMat Ma = cvMat(4, 4, CV_32FC1, a);
		CvMat Mp = cvMat(4, 4, CV_32FC1, projectionMatrix);

		float pontos[4];
		pontos[0] =  sx;
		pontos[1] =  sy;
		pontos[2] =  sz;
		pontos[3] = 1.0;

		CvMat Mpoint = cvMat( 4, 1, CV_32FC1,&pontos);

		CvMat* Mr1 =  cvCreateMat(4,1,CV_32FC1);

		float up[] = { 0.0 ,-1.0 , 0.0 };
		float s[] =  {-1.0 , 0.0 , 0.0 };
		float f[] =  { 0.0 , 0.0 , 1.0 };
		float u[] =  { 0.0 , 1.0 , 0.0 };

		float look[] = {s[0], s[1], s[2], 0,
						u[0], u[1], u[2], 0,
					   -f[0],-f[1],-f[2], 0,
						0   , 0   , 0   , 1};

		CvMat Mlook = cvMat(4, 4, CV_32FC1, look);

		cvMatMul(&Ma,&Mpoint,Mr1);
		cvMatMul(&Mp,Mr1,Mr1);
		cvMatMul(&Mlook,Mr1,Mr1);

		*out_x = (int)(cvmGet(Mr1,0,0) / cvmGet(Mr1,3,0) * HalfWidth + HalfWidth);
		*out_y = (int)(-cvmGet(Mr1,1,0) / cvmGet(Mr1,3,0) * HalfHeight + HalfHeight);
	}
	else
	{
		float rx = sx * rotation_matrix[0] + sy * rotation_matrix[1] + sz * rotation_matrix[2] + translation_vector[0];
		float ry = sx * rotation_matrix[3] + sy * rotation_matrix[4] + sz * rotation_matrix[5] + translation_vector[1];
		float rz = sx * rotation_matrix[6] + sy * rotation_matrix[7] + sz * rotation_matrix[8] + translation_vector[2];

		float x = rx * 2*focalLength/width;
		float y = ry * 2*focalLength/height;
		//float z = rz * (- ( farPlane+nearPlane ) / ( farPlane - nearPlane )) + ( -2.0 * farPlane * nearPlane / ( farPlane - nearPlane ));
		float w = rz;

		*out_x = (int)(x / w * HalfWidth + HalfWidth);
		*out_y = (int)(y / w * HalfHeight + HalfHeight);
	}
}

static float fLeftX  = -0.5;
static float fRightX =  1.5;
static float fUpY    = (float) -0.9;
static float fDownY  = (float) 1.3;

void CFaceDetector::CalculateRange (CvMatr32f rotation_matrix,CvVect32f translation_vector)
{
	float LeftX = -0.5;
	float RightX = 1.5;
	float b1,b2;
	float Distance1, Distance2;
	float minDist = 2;
	int x1,y1,x2,y2;
	int Max_Iter = 100;
	int iter = 0;

	b1 = LeftX + (RightX - LeftX)/2;
	b2 = RightX;
	ModelToView (LeftX, (float)0.3,image,&x1,&y1,rotation_matrix,translation_vector);
	ModelToView (b2, (float)0.3,image,&x2,&y2,rotation_matrix,translation_vector);
	Distance1 = (float)GetDistance (x1,y1,x2,y2);

	do
	{
		iter++;

		ModelToView (b1,(float)0.3,image,&x2,&y2,rotation_matrix,translation_vector);
		Distance2 = (float)GetDistance (x1,y1,x2,y2);
		if (Distance2 < Distance1)
		{
			b1 += (b2-b1)/2;
		}
		else 
		{
			b2 -= (b2-b1)/2;
			ModelToView (b2,(float)0.3,image,&x2,&y2,rotation_matrix,translation_vector);
			Distance1 = (float)GetDistance (x1,y1,x2,y2);
		}
		if (fabs(b2-b1)<0.01)
		{
			RightX = b1;
			break;
		}
	}
	while (iter < Max_Iter);

	fRightX = RightX;

	b1 = RightX - (RightX - LeftX)/2;
	b2 = LeftX;
	ModelToView (RightX, (float)0.3,image,&x1,&y1,rotation_matrix,translation_vector);
	ModelToView (b2,(float)0.3,image,&x2,&y2,rotation_matrix,translation_vector);
	Distance1 = (float)GetDistance (x1,y1,x2,y2);

	iter = 0;
	do
	{
		iter++;

		ModelToView (b1,(float)0.3,image,&x2,&y2,rotation_matrix,translation_vector);
		Distance2 = (float)GetDistance (x1,y1,x2,y2);
		if (Distance2 < Distance1)
		{
			b1 += (b2-b1)/2;
		}
		else 
		{
			b2 -= (b2-b1)/2;
			ModelToView (b2,(float)0.3,image,&x2,&y2,rotation_matrix,translation_vector);
			Distance1 = (float)GetDistance (x1,y1,x2,y2);
		}
		if (fabs(b2-b1)<0.01)
		{
			fLeftX = b1;
			return;
		}
	}
	while (iter < Max_Iter);
}

bool CFaceDetector::ViewToModel (int in_x, int in_y, IplImage* frame, float* out_x, float* out_y, CvMatr32f rotation_matrix,CvVect32f translation_vector)
{
	float LeftX = fLeftX;
	float RightX = fRightX;
	float UpY = fUpY;
	float DownY = fDownY;
	float Distance1, Distance2, Distance3;
	float devider = 2;
	float minDist = 1;
	int x1,y1,x2,y2;

	*out_y = 0.5;
	int Max_Iter = 100;
	int iter = 0;
	int mode = 0;
	do
	{
		iter++;
		
		if (mode == 0)
		{
			mode = 1;

			ModelToView (LeftX, *out_y,frame,&x1,&y1,rotation_matrix,translation_vector);
			ModelToView (RightX,*out_y,frame,&x2,&y2,rotation_matrix,translation_vector);

			Distance3 = (float)GetDistance(x1,y1,x2,y2);
			//if (Distance3 > 2*minDist)
			//{
				int cx,cy;
				ProjectionPoint2Line (x1,y1,x2,y2,in_x,in_y,&cx,&cy);
				Distance1 = (float)GetDistance(cx,cy,x1,y1);
				Distance2 = (float)GetDistance(cx,cy,x2,y2);
				if ((Distance3 < Distance1) || (Distance3 < Distance2))
				{
					if (Distance1 < Distance2)
					{
						*out_x = LeftX;
						LeftX -= (RightX - LeftX);
						if (LeftX<fLeftX) 
						{
							return false;
							//LeftX = *out_x;
							//*out_x = RightX;
						}
						else
							RightX = *out_x;
						//if (Distance1 < minDist) 
						//	return true;
					}
					else
					{
						*out_x = RightX;
						RightX += (RightX - LeftX);
						if (RightX>fRightX) 
						{
							return false;
							//RightX = *out_x;
							//*out_x = LeftX;
						}
						else
							LeftX = *out_x;
						//if (Distance2 < minDist) 
						//	return true;
					}
				}
				else 
				{
					Distance1 = (float)GetDistance(in_x,in_y,x1,y1);
					Distance2 = (float)GetDistance(in_x,in_y,x2,y2);
					if (Distance1 > Distance2)
					{
						LeftX += (RightX - LeftX)/devider;
						*out_x = RightX;
						if (Distance2 < minDist) 
							return true;
					}
					else
					{
						RightX -= (RightX - LeftX)/devider;
						*out_x = LeftX;
						if (Distance1 < minDist) 
							return true;
					}
				}
			//}
		}
		else
		{
			mode = 0;

			ModelToView (*out_x,UpY,frame,&x1,&y1,rotation_matrix,translation_vector);
			ModelToView (*out_x,DownY,frame,&x2,&y2,rotation_matrix,translation_vector);

			Distance3 = (float)GetDistance(x1,y1,x2,y2);
			//if (Distance3 > 2*minDist)
			//{
				int cx,cy;
				ProjectionPoint2Line (x1,y1,x2,y2,in_x,in_y,&cx,&cy);
				Distance1 = (float)GetDistance(cx,cy,x1,y1);
				Distance2 = (float)GetDistance(cx,cy,x2,y2);
				if ((Distance3 < Distance1) || (Distance3 < Distance2))
				{
					if (Distance1 < Distance2)
					{
						*out_y = UpY;
						UpY -= (DownY - UpY);
						if (UpY < fUpY)
							return false;
						else
							DownY = *out_y;
						//if (Distance1 < minDist) 
						//	return true;
					}
					else
					{
						*out_y = DownY;
						DownY += (DownY - UpY);
						if (DownY > fDownY)
							return false;
						else
							UpY = *out_y;
						//if (Distance2 < minDist) 
						//	return true;
					}
				}
				else
				{
					Distance1 = (float)GetDistance(in_x,in_y,x1,y1);
					Distance2 = (float)GetDistance(in_x,in_y,x2,y2);
					if (Distance1 > Distance2)
					{
						UpY += (DownY - UpY)/devider;
						*out_y = DownY;
						if (Distance2 < minDist) 
							return true;
					}
					else
					{
						DownY -= (DownY - UpY)/devider;
						*out_y = UpY;
						if (Distance1 < minDist) 
							return true;
					}
				}
			//}
		}

		//int rx,ry;
		//ModelToView (*out_x,UpY,frame,&rx,&ry,rotation_matrix,translation_vector);
		//cvCircle(frame,cvPoint(rx,ry),1,cvScalar(0,255,255),1);
		//ModelToView (*out_x,DownY,frame,&rx,&ry,rotation_matrix,translation_vector);
		//cvCircle(frame,cvPoint(rx,ry),1,cvScalar(0,255,255),1);
		//ModelToView (LeftX,*out_y,frame,&rx,&ry,rotation_matrix,translation_vector);
		//cvCircle(frame,cvPoint(rx,ry),1,cvScalar(0,255,255),1);
		//ModelToView (RightX,*out_y,frame,&rx,&ry,rotation_matrix,translation_vector);
		//cvCircle(frame,cvPoint(rx,ry),1,cvScalar(0,255,255),1);
		//m_imageTestDlg.SetImage (frame);
		//Sleep(10);
	} while (iter < Max_Iter);

	return false;
}


///////////////////////////////////////////////////////////////////
//
// Optical flow prediction and POSIT part
//
///////////////////////////////////////////////////////////////////
bool CFaceDetector::PutPoints ()
{
	ptmEye = GetCenterPoint (ptcLeye,ptcReye);
	if (ModelMouthPos == 0)
	{
		int eyeSize = GetDistance (ptcLeye,ptcReye);
		int mouthSize = GetDistance (ptmEye,ptcMouth);
		if (eyeSize==0)
			return false;
		ModelMouthPos = (float)(mouthSize/eyeSize);
		if ((ModelMouthPos>1.5) || (ModelMouthPos<0.7))
			return false;
	}
	ptcNose = GetCenterPoint (ptmEye,ptcMouth);
	ModelNosePos = (float)(0.5 * ModelMouthPos);

	UpdatePOSITdata();
	modelPoints.clear();
	imagePoints.clear();

	CvPoint RefPoint;
	int w = gray->width;
	int h = gray->height;
	int w2 = w/2;
	int h2 = h/2;
	float fx,fy,fz;

	getSinModel(0,0,&fx,&fy,&fz); // Left eye
	modelPoints.push_back( cvPoint3D32f(fx, fy , fz) );
	ModelToView (0,0,gray,&RefPoint.x,&RefPoint.y,m_frotation_matrix,m_fOtranslation_vector);
	imagePoints.push_back( cvPoint2D32f(RefPoint.x - w2, RefPoint.y - h2) );

	getSinModel(1,0,&fx,&fy,&fz); // Right eye
	modelPoints.push_back( cvPoint3D32f(fx, fy , fz) );
	ModelToView (1,0,gray,&RefPoint.x,&RefPoint.y,m_frotation_matrix,m_fOtranslation_vector);
	imagePoints.push_back( cvPoint2D32f(RefPoint.x - w2, RefPoint.y - h2) );

	getSinModel(0.5,ModelMouthPos,&fx,&fy,&fz); // Mouth
	modelPoints.push_back( cvPoint3D32f(fx, fy , fz) );
	ModelToView (0.5,ModelMouthPos,gray,&RefPoint.x,&RefPoint.y,m_frotation_matrix,m_fOtranslation_vector);
	imagePoints.push_back( cvPoint2D32f(RefPoint.x - w2, RefPoint.y - h2) );

	// Put grid
	for (double y = -0.4; y < 1.31; y +=0.3)
		for (double x = -0.25; x < 1.3; x +=0.5)
			if (y<0.5+sin((x+0.25)/1.5*M_PI))
			{
				getSinModel(float(x),float(y)*ModelMouthPos,&fx,&fy,&fz); 
				modelPoints.push_back( cvPoint3D32f(fx, fy , fz) );
				ModelToView (float(x),float(y)*ModelMouthPos,gray,&RefPoint.x,&RefPoint.y,m_frotation_matrix,m_fOtranslation_vector);
				imagePoints.push_back (cvPoint2D32f(RefPoint.x - w2,RefPoint.y - h2));
			}

	cvCopy( gray, prev_gray );

	return true;
}

std::vector<int> getRandomSet(int sampleSize, int setSize, int startPos = 0)
{
	std::vector<int> lista;
	for(int i = startPos; i < sampleSize; i++)
		lista.push_back(i);

	std::vector<int> randomSet;
	while(randomSet.size()<size_t(setSize))
	{
		int pos = rand()%lista.size();
		randomSet.push_back(lista[pos]);
		lista.erase(lista.begin()+pos);
	}
	return randomSet;
}

#define MinPointsCnt 30
bool CFaceDetector::PredictPoints2Part1 (void)
{
	float fx, fy;//, fz;
//	CvPoint RefPoint;
	int w2 = gray->width / 2;
	int h2 = gray->height / 2;
	int RansacDist = (int)(GetFaceSize()*0.2);
	double min_distance = RansacDist/2;

	// Calculate Range Box
	float f_xd = fRightX - fLeftX;
	float f_yd = fDownY - fUpY;

	f_leftX  = fLeftX + f_xd/4;
	f_rightX = fRightX - f_xd/4;
	f_upY    = fUpY + f_yd/8;
	f_downY  = fDownY - f_yd/8;

	int x;
	int y;
	int MaxX = 0;
	int MaxY = 0;
	int MinX = image->width;
	int MinY = image->height;
	CalculateRange(m_frotation_matrix,m_fOtranslation_vector);
	float f_midY = (float)((f_upY + f_downY)/2.0);
	ModelToView (f_leftX,f_upY,image,&x,&y,m_frotation_matrix,m_fOtranslation_vector);
	if (MinX > x) MinX = x; if (MinY > y) MinY = y; if (MaxX < x) MaxX = x;	if (MaxY < y) MaxY = y;
	ModelToView (f_rightX,f_upY,image,&x,&y,m_frotation_matrix,m_fOtranslation_vector);
	if (MinX > x) MinX = x;	if (MinY > y) MinY = y;	if (MaxX < x) MaxX = x;	if (MaxY < y) MaxY = y;
	ModelToView (f_leftX,f_midY,image,&x,&y,m_frotation_matrix,m_fOtranslation_vector);
	if (MinX > x) MinX = x;	if (MinY > y) MinY = y;	if (MaxX < x) MaxX = x;	if (MaxY < y) MaxY = y;
	ModelToView (f_rightX,f_midY,image,&x,&y,m_frotation_matrix,m_fOtranslation_vector);
	if (MinX > x) MinX = x;	if (MinY > y) MinY = y;	if (MaxX < x) MaxX = x;	if (MaxY < y) MaxY = y;
	ModelToView (f_leftX,f_downY,image,&x,&y,m_frotation_matrix,m_fOtranslation_vector);
	if (MinX > x) MinX = x;	if (MinY > y) MinY = y;	if (MaxX < x) MaxX = x;	if (MaxY < y) MaxY = y;
	ModelToView (f_rightX,f_downY,image,&x,&y,m_frotation_matrix,m_fOtranslation_vector);
	if (MinX > x) MinX = x;	if (MinY > y) MinY = y;	if (MaxX < x) MaxX = x;	if (MaxY < y) MaxY = y;
	if ((f_leftX < 0.5) && (f_rightX>0.5))
	{
		float f_midX = (float)((f_leftX + f_rightX)/2.0);
		ModelToView (f_midX,f_upY,image,&x,&y,m_frotation_matrix,m_fOtranslation_vector);
		if (MinX > x) MinX = x;	if (MinY > y) MinY = y;	if (MaxX < x) MaxX = x;	if (MaxY < y) MaxY = y;
		ModelToView (f_midX,f_downY,image,&x,&y,m_frotation_matrix,m_fOtranslation_vector);
		if (MinX > x) MinX = x;	if (MinY > y) MinY = y;	if (MaxX < x) MaxX = x;	if (MaxY < y) MaxY = y;
	}

	if ((MaxX > MinX) && (MaxY > MinY))
	{
		// Add new points if need
		if (modelPoints.size() < MinPointsCnt)
		{
			CvRect rect = cvRect (MinX,MinY,MaxX-MinX,MaxY-MinY);
			ControlRect (rect,gray->width,gray->height);
			//cvRectangle (image,cvPoint (rect.x,rect.y),cvPoint (rect.x + rect.width,rect.y + rect.height),cvScalarAll(255));

			IplImage* result = cvCreateImage( cvSize(rect.width, rect.height), prev_gray->depth, prev_gray->nChannels );
			cvSetImageROI (prev_gray,rect);
			cvCopy(prev_gray,result);
			cvResetImageROI(prev_gray);

			IplImage* eig = cvCreateImage( cvGetSize(result), 32, 1 );
			IplImage* temp = cvCreateImage( cvGetSize(result), 32, 1 );

			CvPoint2D32f Sptcs[MinPointsCnt];
			double quality = 0.01;
			int numPoints = MinPointsCnt/* - modelPoints.size();
			numPoints = (numPoints < 10) ? 10 : numPoints*/;

			cvGoodFeaturesToTrack( result, eig, temp, Sptcs, &numPoints, quality, min_distance, 0, 3, 0, 0.04 );

			cvReleaseImage (&result);
			cvReleaseImage (&eig);
			cvReleaseImage (&temp);

			float eyed = (float)GetDistance (ptcMLeye,ptcMReye);
			CvPoint PME = ProjectionPoint2Line (ptcMLeye,ptcMReye,ptcMMouth);
			float emd = (float)GetDistance (PME,ptcMMouth);
			for(int i=0;i<numPoints;i++)
			{
				int px = (int)Sptcs[i].x + rect.x;
				int py = (int)Sptcs[i].y + rect.y;

				if (ViewToModel (px,py,gray,&fx,&fy,m_frotation_matrix,m_fOtranslation_vector))
				{
					px -= w2;
					py -= h2;
					bool bGood = true;
					for (int j = 0; j < (int)imagePoints.size(); j++)
					{
						if (GetDistance (px,py,(int)imagePoints[j].x,(int)imagePoints[j].y)<=min_distance)
						{
							bGood = false;
							break;
						}
					}

					if (bGood && (fy<0.4+1.3*sin((fx+0.5)/2*M_PI)))
					{
						modelPoints.push_back( cvPoint3D32f(fx, -fy, GetModelZ (fx,fy)) );
						imagePoints.push_back( cvPoint2D32f(px, py) );

						if (imagePoints.size() >= MinPointsCnt)
							break;
					}
				}
			}
		}

		//for (int i=0;i<imagePoints.size();i++)
		//	cvCircle (image,cvPoint(imagePoints[i].x + w2,imagePoints[i].y+h2),3,cvScalarAll (200));

		// Optical flow stage
		for (int i=0; i < (int)imagePoints.size(); i++)
		{
			points[0][i].x = imagePoints[i].x + w2;
			points[0][i].y = imagePoints[i].y + h2;
			points[1][i].x = imagePoints[i].x + w2;
			points[1][i].y = imagePoints[i].y + h2;
		}
		int win_size = 10;
		cvCalcOpticalFlowPyrLK( prev_gray, gray, NULL, NULL,
								points[0], points[1], imagePoints.size(), cvSize(win_size,win_size), 3, status, feature_errors,
								cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.3), 0 );

		// Get rid of optical flow outliers. Part 1
		m_fLKdelta = 0;
		int cnt1 = 0;
		for( int i = 0; i < (int)imagePoints.size(); i++ )
		{
			if( status[i] && feature_errors[i]<250)
			{
				float dx = points[0][i].x - points[1][i].x;
				float dy = points[0][i].y - points[1][i].y;
				m_fLKdelta += sqrt(dx*dx + dy*dy);
				cnt1++;
			}
		}
		if (cnt1) 
			m_fLKdelta /= (float)cnt1;

		modelPointsLK.clear();
		imagePointsLK.clear();

		return true;
	}

	return false;
}

void CFaceDetector::PredictPoints2Part2 (void)
{
	float fx, fy;//, fz;
	CvPoint RefPoint;
	int w2 = gray->width / 2;
	int h2 = gray->height / 2;
	int RansacDist = (int)(GetFaceSize()*0.2);

	// Add manual points

	if (bMeye)
	{
		ptmEye = GetCenterPoint (ptcLeye,ptcReye);
		modelPointsLK.push_back( cvPoint3D32f(0.5, 0, 0.6) );
		imagePointsLK.push_back( cvPoint2D32f(ptmEye.x - w2, ptmEye.y - h2) );
	}
	else
	{
		if (bLeye)
		{
			modelPointsLK.push_back( cvPoint3D32f(0, 0, 0.6) );
			imagePointsLK.push_back( cvPoint2D32f(ptcLeye.x - w2, ptcLeye.y - h2) );
		}

		if (bReye)
		{
			modelPointsLK.push_back( cvPoint3D32f(1, 0, 0.6) );
			imagePointsLK.push_back( cvPoint2D32f(ptcReye.x - w2, ptcReye.y - h2) );
		}

		if (bMouth)
		{
			int k = (imagePoints.size() < 10) ? 1 : 6;
			for (int i = 0; i < k; i++)
			{
				modelPointsLK.push_back( cvPoint3D32f(0.5, -ModelMouthPos, 0.8) );
				imagePointsLK.push_back( cvPoint2D32f(ptcMouth.x - w2, ptcMouth.y - h2) );
			}
		}
	}

	//int k = (imagePoints.size() < 10) ? 1 : 6;
	//for (int i = 0; i < k; i++)
	//{
	//	//getSinModel(0,0,&fx,&fy,&fz); // Left eye
	//	modelPointsLK.push_back( cvPoint3D32f(0, 0, 0.6) );
	//	imagePointsLK.push_back( cvPoint2D32f(ptcLeye.x - w2, ptcLeye.y - h2) );

	//	//getSinModel(1,0,&fx,&fy,&fz); // Right eye
	//	modelPointsLK.push_back( cvPoint3D32f(1, 0, 0.6) );
	//	imagePointsLK.push_back( cvPoint2D32f(ptcReye.x - w2, ptcReye.y - h2) );

	//	//getSinModel(0.5,ModelMouthPos,&fx,&fy,&fz); // Mouth
	//	modelPointsLK.push_back( cvPoint3D32f(0.5, -ModelMouthPos, 0.8) );
	//	imagePointsLK.push_back( cvPoint2D32f(ptcMouth.x - w2, ptcMouth.y - h2) );
	//}

	////getSinModel(0.5,ModelNosePos,&fx,&fy,&fz); // Nose
	//modelPointsLK.push_back( cvPoint3D32f(0.5, -ModelNosePos, 1.3) );
	//imagePointsLK.push_back( cvPoint2D32f(ptcNose.x - w2, ptcNose.y - h2) );

	int ManualPointsCnt = modelPointsLK.size();

	// Get rid of optical flow outliers. Part 2
	for( int i = 0; i < (int)imagePoints.size(); i++ )
	{
		if( status[i] && feature_errors[i]<250)
		{
			float dx = points[0][i].x - points[1][i].x;
			float dy = points[0][i].y - points[1][i].y;
			float Tdelta = sqrt(dx*dx + dy*dy);

			if (Tdelta/4 < m_fLKdelta)
			{
				imagePointsLK.push_back( cvPoint2D32f(points[1][i].x - w2, points[1][i].y - h2) );
				modelPointsLK.push_back( modelPoints[i] );
				cvCircle (image,cvPoint((int)points[0][i].x,(int)points[0][i].y),2,cvScalarAll (100));
				cvLine (image,
					cvPoint((int)points[1][i].x,(int)points[1][i].y),
					cvPoint((int)points[0][i].x,(int)points[0][i].y),
					cvScalarAll(150));
				cvCircle (image,cvPoint((int)points[1][i].x, 
					(int)points[1][i].y),2,
					cvScalarAll (200));
			}
		}
	}

	std::vector<int> BestSet;
	if (modelPointsLK.size()>120)
	{
		// RANSAC
		float rotationTestMatrix[9];
		float translationTestVector[3];
		float RefX,RefY,RefZ;
		int bestSize = 0;
		long Distance = -1;
		for (int i = 0; i < 5; i++)
		{
			std::vector<int> selectedPoints = getRandomSet (modelPointsLK.size(),modelPointsLK.size()/2,ManualPointsCnt);

			std::vector<CvPoint3D32f> chosenModelPoints;
			std::vector<CvPoint2D32f> chosenImagePoints;
			// Add manual points
			for (int j = 0; j < ManualPointsCnt; j++)
			{
				chosenModelPoints.push_back(modelPointsLK[j]);
				chosenImagePoints.push_back(imagePointsLK[j]);
			}
			// Add random selected points
			for(int j = 0; j < (int)selectedPoints.size(); j++)
			{
				int index = selectedPoints[j];
				chosenModelPoints.push_back(modelPointsLK[index]);
				chosenImagePoints.push_back(imagePointsLK[index]);
			}
			// Set reference
			m_fRefX = chosenModelPoints[0].x;
			m_fRefY = chosenModelPoints[0].y;
			m_fRefZ = chosenModelPoints[0].z;
			// POSIT
			CvPOSITObject *positObject = cvCreatePOSITObject( &chosenModelPoints[0], static_cast<int>(chosenModelPoints.size()) );
			CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 1.0e-5 );
			cvPOSIT( positObject, &chosenImagePoints[0], EHCIFOCUS, criteria, m_frotation_matrix, m_ftranslation_vector );
			cvReleasePOSITObject (&positObject);

			std::vector<int> inliers;
			long Tdist = 0;
			for (int j = ManualPointsCnt; j < (int)modelPointsLK.size(); j++)
			{
				ModelToView (modelPointsLK[j].x,-modelPointsLK[j].y,image,&RefPoint.x,&RefPoint.y,m_frotation_matrix,m_ftranslation_vector);
				float dist = (float)GetDistance (RefPoint.x,RefPoint.y,
					(int)imagePointsLK[j].x+w2,
					(int)imagePointsLK[j].y+h2);

				Tdist += (long)dist;
				if(dist<RansacDist)
					inliers.push_back(j);
			}
			
			//if ((Distance==-1) || (Distance > Tdist))
			if (bestSize < (int)inliers.size())
			{
				Distance = Tdist;
				bestSize = inliers.size();
				RefX = m_fRefX;
				RefY = m_fRefY;
				RefZ = m_fRefZ;
				BestSet = inliers;
				RtlCopyMemory (rotationTestMatrix,m_frotation_matrix,sizeof(float)*9);
				RtlCopyMemory (translationTestVector,m_ftranslation_vector,sizeof(float)*3);
			}
		}
		RtlCopyMemory (m_frotation_matrix,rotationTestMatrix,sizeof(float)*9);
		RtlCopyMemory (m_ftranslation_vector,translationTestVector,sizeof(float)*3);
		UpdatePOSITdata(false);
		m_fRefX = RefX;
		m_fRefY = RefY;
		m_fRefZ = RefZ;

	}
	else if (modelPointsLK.size()>3)
	{
		float rotationTestMatrix[9];
		float translationTestVector[3];
		RtlCopyMemory (rotationTestMatrix,m_frotation_matrix,sizeof(float)*9);
		RtlCopyMemory (translationTestVector,m_fOtranslation_vector,sizeof(float)*3);
		CvPoint3D64f TAngles = Angles;

		m_fRefX = modelPointsLK[0].x;
		m_fRefY = modelPointsLK[0].y;
		m_fRefZ = modelPointsLK[0].z;
		CvPOSITObject* positObject = cvCreatePOSITObject( &modelPointsLK[0], static_cast<int>(modelPointsLK.size()) );
		CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 1.0e-5 );
		cvPOSIT( positObject, &imagePointsLK[0], EHCIFOCUS, criteria, m_frotation_matrix, m_ftranslation_vector );
		cvReleasePOSITObject (&positObject);
		
		UpdatePOSITdata (false);
		if (bNose)
		{
			if (Angles.x < 0) Angles.x += 360;
			if (TAngles.x < 0) TAngles.x += 360;

			m_fOtranslation_vector[0] = m_fOtranslation_vector[0] + (translationTestVector[0] - m_fOtranslation_vector[0]) / 2;
			m_fOtranslation_vector[1] = m_fOtranslation_vector[1] + (translationTestVector[1] - m_fOtranslation_vector[1]) / 2;
			m_fOtranslation_vector[2] = m_fOtranslation_vector[2] + (translationTestVector[2] - m_fOtranslation_vector[2]) / 2;

			Angles.x = Angles.x + (TAngles.x - Angles.x) / 2;
			Angles.y = Angles.y + (TAngles.y - Angles.y) / 2;
			Angles.z = Angles.z + (TAngles.z - Angles.z) / 2;

			if (Angles.x > 180) Angles.x -= 360;

			double cosX = cos(Angles.x * M_PI/180);
			double cosY = cos(Angles.y * M_PI/180);
			double cosZ = cos(Angles.z * M_PI/180);
			double sinX = sin(Angles.x * M_PI/180);
			double sinY = sin(Angles.y * M_PI/180);
			double sinZ = sin(Angles.z * M_PI/180);

			CvMat Mm = cvMat (3,3,CV_32FC1,m_frotation_matrix);
			CvMat* Rm = cvCreateMat (3,3,CV_32FC1);
			CvMat* Xm = cvCreateMat (3,3,CV_32FC1);
			CvMat* Ym = cvCreateMat (3,3,CV_32FC1);
			CvMat* Zm = cvCreateMat (3,3,CV_32FC1);

			*((float*)CV_MAT_ELEM_PTR(*Xm,0,0)) = 1;
			*((float*)CV_MAT_ELEM_PTR(*Xm,1,0)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Xm,2,0)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Xm,0,1)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Xm,1,1)) = (float)cosX;
			*((float*)CV_MAT_ELEM_PTR(*Xm,2,1)) = (float)sinX;
			*((float*)CV_MAT_ELEM_PTR(*Xm,0,2)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Xm,1,2)) = (float)(-sinX);
			*((float*)CV_MAT_ELEM_PTR(*Xm,2,2)) = (float)cosX;

			*((float*)CV_MAT_ELEM_PTR(*Ym,0,0)) = (float)cosY;
			*((float*)CV_MAT_ELEM_PTR(*Ym,1,0)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Ym,2,0)) = (float)(-sinY);
			*((float*)CV_MAT_ELEM_PTR(*Ym,0,1)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Ym,1,1)) = 1;
			*((float*)CV_MAT_ELEM_PTR(*Ym,2,1)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Ym,0,2)) = (float)(sinY);
			*((float*)CV_MAT_ELEM_PTR(*Ym,1,2)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Ym,2,2)) = (float)cosY;

			*((float*)CV_MAT_ELEM_PTR(*Zm,0,0)) = (float)cosZ;
			*((float*)CV_MAT_ELEM_PTR(*Zm,1,0)) = (float)(-sinZ);
			*((float*)CV_MAT_ELEM_PTR(*Zm,2,0)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Zm,0,1)) = (float)sinZ;
			*((float*)CV_MAT_ELEM_PTR(*Zm,1,1)) = (float)cosZ;
			*((float*)CV_MAT_ELEM_PTR(*Zm,2,1)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Zm,0,2)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Zm,1,2)) = 0;
			*((float*)CV_MAT_ELEM_PTR(*Zm,2,2)) = 1;

			cvMatMul (Xm,Ym,Rm);
			cvMatMul (Rm,Zm,&Mm);

			cvReleaseMat (&Rm);
			cvReleaseMat (&Xm);
			cvReleaseMat (&Ym);
			cvReleaseMat (&Zm);
		}

		for (int j = ManualPointsCnt; j < (int)modelPointsLK.size(); j++)
		{
			ModelToView (modelPointsLK[j].x,-modelPointsLK[j].y,image,&RefPoint.x,&RefPoint.y,m_frotation_matrix,m_fOtranslation_vector);
			float dist = (float)GetDistance (RefPoint.x,
				RefPoint.y,
				(int)imagePointsLK[j].x+w2,
				(int)imagePointsLK[j].y+h2);

			if(dist<RansacDist)
				BestSet.push_back(j);
		}
	}

	// Delete Model OutLiers
	imagePoints.clear();
	modelPoints.clear();
	if (BestSet.size() > 0/*(modelPointsLK.size()-ManualPointsCnt)/2*/)
	{
		for (int j = 0; j < (int)BestSet.size(); j++)
		{
			fx = modelPointsLK[BestSet[j]].x;
			fy = -modelPointsLK[BestSet[j]].y;
			if ((f_leftX < fx) && (fx < f_rightX) && (f_upY < fy) && (fy < f_downY))
			{
				modelPoints.push_back (modelPointsLK[BestSet[j]]);
				imagePoints.push_back (imagePointsLK[BestSet[j]]);
			}
		}
	}
	else
	{
		for (int i = ManualPointsCnt; i < (int)imagePointsLK.size(); i++)
		{
			fx = modelPointsLK[i].x;
			fy = -modelPointsLK[i].y;
			if ((f_leftX < fx) && (fx < f_rightX) && (f_upY < fy) && (fy < f_downY))
			{
				modelPoints.push_back (modelPointsLK[i]);
				imagePoints.push_back (imagePointsLK[i]);
			}
		}
	}

	//m_imageFaceDlg.SetImage(image);
	//Sleep(200);

	for (float y = -0.9f; y < 1.31f; y +=0.1f)
		for (float x = -0.5f; x < 1.51f; x +=0.1f)
			if (y<0.4+1.3*sin((x+0.5)/2*M_PI))
			{
				ModelToView (x,y/**ModelMouthPos*/,image,&RefPoint.x,&RefPoint.y,m_frotation_matrix,m_fOtranslation_vector,true);
				cvCircle (image,RefPoint,1,cvScalar (255,0,0),1);	
				ModelToView (x,y/**ModelMouthPos*/,image,&RefPoint.x,&RefPoint.y,m_frotation_matrix,m_fOtranslation_vector);
				cvCircle (image,RefPoint,1,cvScalar (0,0,255),1);	
			}
	for (int i=0; i < (int)imagePointsLK.size();i++)
	{
		ModelToView (modelPointsLK[i].x,modelPointsLK[i].y,image,&RefPoint.x,&RefPoint.y,m_frotation_matrix,m_fOtranslation_vector,false,modelPointsLK[i].z);
		cvLine (image,
			cvPoint((int)imagePointsLK[i].x + w2, (int)imagePointsLK[i].y + h2),
			RefPoint,cvScalarAll(255));
		cvCircle (image,RefPoint,2,cvScalar (0,0,255));
	}
}

///////////////////////////////////////////////////////////////////
//
// Optical flow prediction by 4 points part
//
///////////////////////////////////////////////////////////////////
void CFaceDetector::PreparePoints (CvPoint2D32f* pts, CvPoint P, float PredSh)
{
	pts[0] = cvPoint2D32f(P.x,P.y);
	pts[1] = cvPoint2D32f(P.x-PredSh,P.y-PredSh);
	pts[2] = cvPoint2D32f(P.x-PredSh,P.y+PredSh);
	pts[3] = cvPoint2D32f(P.x+PredSh,P.y-PredSh);
	pts[4] = cvPoint2D32f(P.x+PredSh,P.y+PredSh);
}

bool CFaceDetector::AnalizeLKPoints (int Bnum, CvPoint& P, float delta, float PredSh)
{
	int PC = 0;
	P.x = 0;
	P.y = 0;
	for( int i = Bnum; i < Bnum + 5; i++ )
	{
		if( status[i] && feature_errors[i]<250)
		{
			float dx = points[0][i].x - points[1][i].x;
			float dy = points[0][i].y - points[1][i].y;
			float Tdelta = sqrt(dx*dx + dy*dy);
			if (Tdelta/1.5 <= delta)
			{
				//cvLine(image,cvPoint(points[0][i].x,points[0][i].y),cvPoint(points[1][i].x,points[1][i].y),cvScalar(255,255,255));
				switch (i - Bnum) 
				{
				case 1:
					P.x += (int)(points[1][i].x+PredSh);
					P.y += (int)(points[1][i].y+PredSh);
					break;
				case 2:
					P.x += (int)(points[1][i].x+PredSh);
					P.y += (int)(points[1][i].y-PredSh);
					break;
				case 3:
					P.x += (int)(points[1][i].x-PredSh);
					P.y += (int)(points[1][i].y+PredSh);
					break;
				case 4:
					P.x += (int)(points[1][i].x-PredSh);
					P.y += (int)(points[1][i].y-PredSh);
					break;
				default:
					P.x += (int)points[1][i].x;
					P.y += (int)points[1][i].y;
				}
				PC++;
			}
		}
	}		
	if (PC > 2)
	{
		P.x /= PC;
		P.y /= PC;
		return true;
	}
	else
		return false;
}

void CFaceDetector::PredictPoints (void)
{
	float MouthSize = (float)GetMouthSize (ptcMLeye,ptcMReye,ptcMMouth);
	int FaceSize = GetFaceSize();
	float PredSh = (float)(FaceSize/10);
	if (PredSh < 2) PredSh = 2;
	PreparePoints (&points[0][0],ptcMLeye,PredSh);
	PreparePoints (&points[0][5],ptcMReye,PredSh);
	PreparePoints (&points[0][10],ptcMMouth,PredSh);
	//PreparePoints (&points[0][15],ptcMNose,PredSh);

	int numberOfTrackingPoints = 15;
	int win_size = 10;

	cvCalcOpticalFlowPyrLK( prev_gray, gray, NULL, NULL,
							points[0], points[1], numberOfTrackingPoints, cvSize(win_size,win_size), 3, status, feature_errors,
							cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.3), 0 );

	float delta = 0;
	for( int i = 0; i < numberOfTrackingPoints; i++ )
	{
		if( status[i] && feature_errors[i]<250)
		{
			float dx = points[0][i].x - points[1][i].x;
			float dy = points[0][i].y - points[1][i].y;
			delta += sqrt(dx*dx + dy*dy);
		}
	}
	delta /= numberOfTrackingPoints;

//	CvPoint P;
	if (AnalizeLKPoints (0,LkPle,delta,PredSh))
		if (GetDistance (LkPle,ptcMLeye) < FaceSize*EPS)
			ptcMLeye = LkPle;		
	if (AnalizeLKPoints (5,LkPre,delta,PredSh))
		if (GetDistance (LkPre,ptcMReye) < FaceSize*EPS)
			ptcMReye = LkPre;		
	if (AnalizeLKPoints (10,LkPm,delta,PredSh))
		if (GetDistance (LkPm,ptcMMouth) < FaceSize*EPS)
			ptcMMouth = LkPm;		
	//if (AnalizeLKPoints (15,P,delta,PredSh))
	//	if (GetDistance (P,ptcMNose) < FaceSize*EPS)
	//		ptcMNose = P;		
}

///////////////////////////////////////////////////////////////////
//
// Analize geometry
//
///////////////////////////////////////////////////////////////////
bool CFaceDetector::AnalizeGeometry (int FaceSize)
{
	bool res = true;

	float EyesDistance = (float)GetDistance (ptcTLeye,ptcTReye); 
	float NoseToEyesLine = Point2Line (ptcTLeye.x,ptcTLeye.y,ptcTReye.x,ptcTReye.y,ptcTNose.x,ptcTNose.y);
	float MouthToEyesLine = Point2Line (ptcTLeye.x,ptcTLeye.y,ptcTReye.x,ptcTReye.y,ptcTMouth.x,ptcTMouth.y);
	//float MouthNose = GetDistance(ptcTMouth,ptcTNose);
	if ((EyesDistance != 0) && (MouthToEyesLine != 0))
	{
		if (MouthToEyesLine/5 > EyesDistance)
			res = false;
		else if (EyesDistance/5 > MouthToEyesLine)
			res = false;
		else 
		{
			CvPoint ProjMouth = ProjectionPoint2Line (ptcTLeye,ptcTReye,ptcTMouth);

			float dLeye = (float)GetDistance(ProjMouth,ptcTLeye);
			float dReye = (float)GetDistance(ProjMouth,ptcTReye);
			if ((dLeye > EyesDistance*1.5) || (dReye > EyesDistance*1.5))
				res = false;
		}

		if (NoseToEyesLine > MouthToEyesLine)
		{
			if (bPackFace)
			{
				bNose = ExtractNose();
				ptcMNose = ptcTNose;
				if (ECHIWINDOWDBG) 
					cvCircle (image,ptcMNose,5,cvScalar (0,0,255),1);	
				bMouth = ExtractMouth();
				ptcMMouth = ptcTMouth;
				if (ECHIWINDOWDBG) 
					cvCircle (image,ptcMMouth,5,cvScalar (0,0,255),1);	
			}
			else
				res = false;
		}
	} 
	else
		res = false;

	return res;
}

///////////////////////////////////////////////////////////////////
//
// POSIT part
//
///////////////////////////////////////////////////////////////////
void CFaceDetector::AnalizePoints (CvMatr32f rotation_matrix, CvVect32f translation_vector)
{
	//int EyesSize = GetDistance(ptcLeye,ptcReye);
	//ptmEye = GetCenterPoint (ptcReye,ptcLeye);
	//int EyeMouthSize = GetDistance (ptmEye,ptcMouth);
	int frWd2 = gray->width/2;
	int frHd2 = gray->height/2;

	CvPOSITObject *positObject;
	std::vector<CvPoint3D32f> modelPoints1;
	std::vector<CvPoint2D32f> imagePoints1;

	//float fx,fy,fz;
	// Center eye
	ptmEye = GetCenterPoint (ptcLeye,ptcReye);
	modelPoints1.push_back( cvPoint3D32f(0.5, 0, 0.6) );
	imagePoints1.push_back( cvPoint2D32f(ptmEye.x - frWd2, ptmEye.y - frHd2) );

	// Left eye
	//getSinModel(0,0,&fx,&fy,&fz); 
	modelPoints1.push_back( cvPoint3D32f(0, 0, 0.6) );
	imagePoints1.push_back( cvPoint2D32f(ptcLeye.x - frWd2, ptcLeye.y - frHd2) );

	// Right eye
	//getSinModel(1,0,&fx,&fy,&fz); 
	modelPoints1.push_back( cvPoint3D32f(1, 0, 0.6) );
	imagePoints1.push_back( cvPoint2D32f(ptcReye.x - frWd2, ptcReye.y - frHd2) );

	//getSinModel(0.5,ModelMouthPos,&fx,&fy,&fz); // Mouth
	modelPoints1.push_back( cvPoint3D32f(0.5, -ModelMouthPos, 0.8) );
	imagePoints1.push_back( cvPoint2D32f(ptcMouth.x - frWd2, ptcMouth.y - frHd2) );

	//getSinModel(0.5,ModelNosePos,&fx,&fy,&fz); // Nose
	modelPoints1.push_back( cvPoint3D32f(0.5, -ModelNosePos*fabs(ModelMouthPos), 1.35) );
	imagePoints1.push_back( cvPoint2D32f(ptcNose.x - frWd2, ptcNose.y - frHd2) );

	m_fRefX = modelPoints1[0].x;
	m_fRefY = modelPoints1[0].y;
	m_fRefZ = modelPoints1[0].z;
	positObject = cvCreatePOSITObject( &modelPoints1[0], static_cast<int>(modelPoints1.size()) );
	CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 1.0e-5 );
	cvPOSIT( positObject, &imagePoints1[0], EHCIFOCUS, criteria, rotation_matrix, translation_vector );
	cvReleasePOSITObject (&positObject);
}

bool CFaceDetector::UpdatePOSITdata (bool Analize)
{
	win32::CAutoLockCriticalSection lockingGuard(m_DataLock);

	ptcRLeye  = ptcLeye;
	ptcRReye  = ptcReye;
	ptcRNose  = ptcNose;
	ptcRMouth = ptcMouth;

	if (Analize)
		AnalizePoints(m_frotation_matrix,m_ftranslation_vector);

	m_fOtranslation_vector[0] += (m_ftranslation_vector[0] - m_fOtranslation_vector[0]) / 2;
	m_fOtranslation_vector[1] += (m_ftranslation_vector[1] - m_fOtranslation_vector[1]) / 2;
	m_fOtranslation_vector[2] += (m_ftranslation_vector[2] - m_fOtranslation_vector[2]) / 2;
	//Otranslation_vector[0] = translation_vector[0];
	//Otranslation_vector[1] = translation_vector[1];
	//Otranslation_vector[2] = translation_vector[2];

	CvMat Mm = cvMat (3,3,CV_32FC1,m_frotation_matrix);
	CvMat* Rm = cvCreateMat (3,3,CV_32FC1);
	CvMat* Om = cvCreateMat (3,3,CV_32FC1);
	CvMat* Xm = cvCreateMat (3,3,CV_32FC1);
	CvMat* Ym = cvCreateMat (3,3,CV_32FC1);
	CvMat* Zm = cvCreateMat (3,3,CV_32FC1);
	CvPoint3D64f TAngles;

	//cvRQDecomp3x3 (&Mm,Rm,Om,Xm,Ym,Zm,&TAngles);
	try //victor - fix me!!!
	{
		cvRQDecomp3x3 (&Mm,Rm,Om,Xm,Ym,Zm,&TAngles);
	}
	catch(cv::Exception & e)
	{
#ifdef _DEBUG
		OutputDebugStringA(e.msg.c_str());
#endif
		e.code = e.code;
		for (int i = 0; i < 9; i++)
			m_frotation_matrix[i] = 0.0;
		for (int i = 0; i < 3; i ++)
		{
			m_ftranslation_vector[i] = 0.0;
			m_fOtranslation_vector[i] = 0.0;
		}
		cvReleaseMat (&Rm);
		cvReleaseMat (&Om);
		cvReleaseMat (&Xm);
		cvReleaseMat (&Ym);
		cvReleaseMat (&Zm);
		return false;
	}

	//Angles = TAngles;

	double dv;
	double MaxDAngle = 8;
	TAngles.x = (TAngles.x < 0) ? TAngles.x+360 : TAngles.x;
	Angles.x  = (Angles.x  < 0) ? Angles.x +360 : Angles.x;

	dv = (TAngles.x - Angles.x) / 2;
	if (fabs(dv) > MaxDAngle) dv = (dv < 0) ? -MaxDAngle : MaxDAngle;
	//Angles.x += dv;
	if (fabs(dv)>1) Angles.x += (abs(dv)>5) ? dv : (dv>0) ? 1 : -1;

	dv = (TAngles.y - Angles.y) / 2;
	if (fabs(dv) > MaxDAngle) dv = (dv < 0) ? -MaxDAngle : MaxDAngle;
	//Angles.y += dv;
	if (fabs(dv)>1) Angles.y += (abs(dv)>5) ? dv : (dv>0) ? 1 : -1;

	dv = (TAngles.z - Angles.z) / 2;
	if (fabs(dv) > MaxDAngle) dv = (dv < 0) ? -MaxDAngle : MaxDAngle;
	//Angles.z += dv;
	if (fabs(dv)>1) Angles.z += (abs(dv)>5) ? dv : (dv>0) ? 1 : -1;

	Angles.x  = (Angles.x  > 180) ? Angles.x-360 : Angles.x;

	double cosX = cos(Angles.x * M_PI/180);
	double cosY = cos(Angles.y * M_PI/180);
	double cosZ = cos(Angles.z * M_PI/180);
	double sinX = sin(Angles.x * M_PI/180);
	double sinY = sin(Angles.y * M_PI/180);
	double sinZ = sin(Angles.z * M_PI/180);

	char number[100];
	CvFont font;
	cvInitFont (&font,CV_FONT_HERSHEY_PLAIN,1,1); 
	_itoa ((int)Angles.x,number,10);
	cvPutText (image,number,cvPoint(0,15),&font,cvScalar (0,0,255));
	_itoa ((int)Angles.y,number,10);
	cvPutText (image,number,cvPoint(70,15),&font,cvScalar (0,0,255));
	_itoa ((int)Angles.z,number,10);
	cvPutText (image,number,cvPoint(140,15),&font,cvScalar (0,0,255));
	//sprintf(number, "%.3g", ModelNosePos );
	//cvPutText (image,number,cvPoint(210,15),&font,cvScalar (0,0,255));

	*((float*)CV_MAT_ELEM_PTR(*Xm,0,0)) = 1;
	*((float*)CV_MAT_ELEM_PTR(*Xm,1,0)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Xm,2,0)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Xm,0,1)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Xm,1,1)) = (float)cosX;
	*((float*)CV_MAT_ELEM_PTR(*Xm,2,1)) = (float)sinX;
	*((float*)CV_MAT_ELEM_PTR(*Xm,0,2)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Xm,1,2)) = (float)(-sinX);
	*((float*)CV_MAT_ELEM_PTR(*Xm,2,2)) = (float)cosX;

	*((float*)CV_MAT_ELEM_PTR(*Ym,0,0)) = (float)cosY;
	*((float*)CV_MAT_ELEM_PTR(*Ym,1,0)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Ym,2,0)) = (float)(-sinY);
	*((float*)CV_MAT_ELEM_PTR(*Ym,0,1)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Ym,1,1)) = 1;
	*((float*)CV_MAT_ELEM_PTR(*Ym,2,1)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Ym,0,2)) = (float)sinY;
	*((float*)CV_MAT_ELEM_PTR(*Ym,1,2)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Ym,2,2)) = (float)cosY;

	*((float*)CV_MAT_ELEM_PTR(*Zm,0,0)) = (float)cosZ;
	*((float*)CV_MAT_ELEM_PTR(*Zm,1,0)) = (float)(-sinZ);
	*((float*)CV_MAT_ELEM_PTR(*Zm,2,0)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Zm,0,1)) = (float)sinZ;
	*((float*)CV_MAT_ELEM_PTR(*Zm,1,1)) = (float)cosZ;
	*((float*)CV_MAT_ELEM_PTR(*Zm,2,1)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Zm,0,2)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Zm,1,2)) = 0;
	*((float*)CV_MAT_ELEM_PTR(*Zm,2,2)) = 1;

	cvMatMul (Xm,Ym,Rm);
	cvMatMul (Rm,Zm,&Mm);

	cvReleaseMat (&Rm);
	cvReleaseMat (&Om);
	cvReleaseMat (&Xm);
	cvReleaseMat (&Ym);
	cvReleaseMat (&Zm);
	return true;
}

///////////////////////////////////////////////////////////////////
//
// Find face skin and x shift
//
///////////////////////////////////////////////////////////////////
bool CFaceDetector::GetSkinParameters (float FaceSize, int EyeAngle)
{
	ptmEye = GetCenterPoint(ptcLeye,ptcReye);
	int eyeSize = GetDistance (ptcLeye,ptcReye);
	int eyeMouthSize = GetDistance (ptmEye,ptcMouth);
	
	if ((eyeSize > 10) && (eyeMouthSize>10))
	{
		int imW = image->width;
		int imH = image->height;
		int rootX = ptmEye.x;
		int rootY = ptmEye.y;
		float nEx = (float)(ptcReye.x - ptcLeye.x); nEx /= (float)eyeSize; 
		float nEy = (float)(ptcReye.y - ptcLeye.y); nEy /= (float)eyeSize;
		float nMx = (float)(ptcMouth.x - rootX); nMx /= (float)eyeMouthSize; 
		float nMy = (float)(ptcMouth.y - rootY); nMy /= (float)eyeMouthSize;
		int edges[12] = {2,3,4,3,0,0,0,1,2,4,5,5};
		float deltaEyeSize = (float)(eyeSize*0.1);
		float deltaEyeMouthSize = (float)(eyeMouthSize*0.1);
		float eyeMouthPos = (float)(-0.6*eyeMouthSize);
		m_BAVG = 0;
		m_GAVG = 0;
		m_RAVG = 0;
		IplImage* _imG = cvCreateImage(cvSize(70,1),8,1);
		IplImage* _imR = cvCreateImage(cvSize(70,1),8,1);
		IplImage* _imB = cvCreateImage(cvSize(70,1),8,1);
		IplImage* _imBase = cvCreateImage(cvSize(70,1),IPL_DEPTH_16U,1);
		IplImage* _imS = cvCreateImage(cvSize(70,1),8,1);
		IplImage* _imH = cvCreateImage(cvSize(70,1),8,1);
		uchar* ptrG = (uchar*)_imG->imageData;
		uchar* ptrR = (uchar*)_imR->imageData;
		uchar* ptrB = (uchar*)_imB->imageData;
		WORD* ptrBase = (WORD*)_imBase->imageData;
		uchar* ptrS = (uchar*)_imS->imageData;
		uchar* ptrH = (uchar*)_imH->imageData;
		int idx = 0;
		for (int y = 0; y < 12; y++)
		{
			int edge = edges[y];
			float eyePos = -edge*deltaEyeSize;
			for (int x = -edge; x <=edge; x++)
			{
				int xPos = rootX + (int)(nMx*eyeMouthPos + nEx*eyePos);
				int yPos = rootY + (int)(nMy*eyeMouthPos + nEy*eyePos);
				if ((xPos >= 0) && (xPos < imW) && (yPos > 0) && (yPos < imH))
				{
					CvScalar d = cvGet2D (image,yPos,xPos);
					int B = (int)d.val[0];
					int G = (int)d.val[1];
					int R = (int)d.val[2];
					m_BAVG += B;
					m_GAVG += G;
					m_RAVG += R;
					int Base = R+G+B;
					if (Base>0)
					{
						R = R*255/Base;
						G = G*255/Base;
						B = B*255/Base;
					}
					*ptrR = R; ptrR++;
					*ptrG = G; ptrG++;
					*ptrB = B; ptrB++;
					*ptrBase = Base; ptrBase++;

					int maxRGB = MAX(MAX(R,G),B);
					int minRGB = MIN(MIN(R,G),B);
					int MaxMinDif = maxRGB - minRGB;
					
					int Saturation, Hue, Cr, Cg, Cb; 

					if (maxRGB == 0)
						Saturation = 0;
					else
						Saturation = MaxMinDif*255/maxRGB;

					if (Saturation == 0)
						Hue = 255;
					else
					{
						Cr = (maxRGB-R)*40/MaxMinDif;
						Cg = (maxRGB-G)*40/MaxMinDif;
						Cb = (maxRGB-B)*40/MaxMinDif;
						if (R == maxRGB)
							Hue = 80 + Cb - Cg;
						else if (G == maxRGB)
							Hue = 160 + Cr - Cb;
						else if (B == maxRGB)
							Hue = Cg - Cr;
						
						if (Hue < 0)
							Hue +=240;
					}

					*ptrS = Saturation; ptrS++;
					*ptrH = Hue; ptrH++;

					//cvCircle (image,cvPoint(rootX + nMx*eyeMouthPos + nEx*eyePos,rootY + nMy*eyeMouthPos + nEy*eyePos),2,cvScalar(0,255,0));
					eyePos += deltaEyeSize;
					idx++;
				}
			}
			eyeMouthPos += deltaEyeMouthSize;
		}
		if (idx>0)
		{
			m_BAVG /= idx;
			m_GAVG /= idx;
			m_RAVG /= idx;
		}
		else
		{
			cvReleaseImage (&_imG);
			cvReleaseImage (&_imR);
			cvReleaseImage (&_imB);
			cvReleaseImage (&_imBase);
			cvReleaseImage (&_imS);
			cvReleaseImage (&_imH);
			return false;
		}
		IplImage* imG;
		IplImage* imR;
		IplImage* imB;
		IplImage* imBase;
		IplImage* imS;
		IplImage* imHu;
		if (idx < 70)
		{
			imG = cvCreateImage(cvSize(idx,1),8,1);
			imR = cvCreateImage(cvSize(idx,1),8,1);
			imB = cvCreateImage(cvSize(idx,1),8,1);
			imBase = cvCreateImage(cvSize(idx,1),IPL_DEPTH_16U,1);
			imS = cvCreateImage(cvSize(idx,1),8,1);
			imHu = cvCreateImage(cvSize(idx,1),8,1);
			CvRect ROIRect = cvRect (0,0,idx,1);
			cvSetImageROI (_imG,ROIRect);
			cvSetImageROI (_imR,ROIRect);
			cvSetImageROI (_imB,ROIRect);
			cvSetImageROI (_imBase,ROIRect);
			cvSetImageROI (_imS,ROIRect);
			cvSetImageROI (_imH,ROIRect);
			cvCopy (_imG,imG);
			cvCopy (_imR,imR);
			cvCopy (_imB,imB);
			cvCopy (_imBase,imBase);
			cvCopy (_imS,imS);
			cvCopy (_imH,imHu);
			cvReleaseImage (&_imG);
			cvReleaseImage (&_imR);
			cvReleaseImage (&_imB);
			cvReleaseImage (&_imBase);
			cvReleaseImage (&_imS);
			cvReleaseImage (&_imH);
		}
		else
		{
			imG = _imG;
			imR = _imR;
			imB = _imB;
			imBase = _imBase;
			imS = _imS;
			imHu = _imH;
		}

		cvMean_StdDev (imG,&m_Gmean,&m_Gsdv);
		cvMean_StdDev (imR,&m_Rmean,&m_Rsdv);
		cvMean_StdDev (imB,&m_Bmean,&m_Bsdv);
		cvMean_StdDev (imBase,&m_BaseMean,&m_BaseSdv);
		cvMean_StdDev (imS,&m_Smean,&m_Ssdv);
		cvMean_StdDev (imHu,&m_Hmean,&m_Hsdv);

		cvReleaseImage (&imG);
		cvReleaseImage (&imR);
		cvReleaseImage (&imB);
		cvReleaseImage (&imBase);
		cvReleaseImage (&imS);
		cvReleaseImage (&imHu);

		return true;
	}

	return false;
}

bool CFaceDetector::GetAdaptivSkinRGBMetod (CvRect& rect,float FaceSize, int EyeAngle, int& tx, int& ty, int& tw, int& th, IplImage** rImg)
{
	ptmEye = GetCenterPoint(ptcMLeye,ptcMReye);
	int imW = image->width;
	int imH = image->height;
	int Size = (int)(FaceSize*4);
	int lx = ptmEye.x - (int)(Size*0.5);
	int ly = ptmEye.y - (int)(Size*0.5);
	tx = 0;
	ty = (int)(Size*0.3);
	tw = Size;
	th = (int)(Size*0.6);
	rect = cvRect (lx, ly, Size, Size);

	IplImage* tImg = 0;
	MakeItemsImg (&tImg,image,rect,EyeAngle,cvScalarAll(0),true,tx,ty,tw,th);
	if (!tImg)
		return false;

	int lbr = (int)(14     + 0.75  * (m_Rmean - m_Rsdv)); if (lbr < 1) lbr = 1;
	int ubr = (int)(-25    + 1.385 * (m_Rmean + m_Rsdv)); if (ubr > 254) ubr = 254;
	int lbg = (int)(-152   + 2.857 * (m_Gmean - m_Gsdv)); if (lbg < 1) lbg = 1;
	int ubg = (int)(-43.35 + 1.6   * (m_Gmean + m_Gsdv)); if (ubg > 254) ubg = 254;
	int lbb = (int)(-45.26 + 0.79  * (m_BaseMean - m_BaseSdv)); if (lbb < 1) lbb = 1;
	int ubb = 500;

	CvSize SmallImgSize = cvSize(100,60);
	IplImage* SmallImg = cvCreateImage (SmallImgSize,8,3);
	cvResize (tImg,SmallImg,CV_INTER_NN);
	cvReleaseImage (&tImg);

	// Detect skin
	*rImg = cvCreateImage (SmallImgSize,8,1);
	int ResW = (*rImg)->widthStep;
	int ImW  = SmallImg->widthStep;
	uchar* ResPtr = (uchar*)(*rImg)->imageData;
	uchar* ImPtr  = (uchar*)SmallImg->imageData;

	for (int y = 0; y < SmallImg->height; y++)
	{
		uchar* ptrRs = ResPtr;
		uchar* ptrIm = ImPtr;
		for (int x = 0; x < SmallImg->width; x++)
		{
			int b = *ptrIm;	ptrIm++;
			int g = *ptrIm;	ptrIm++;
			int r = *ptrIm;	ptrIm++;

			int base = r+g+b;
			if (base > 0)
			{
				r = r * 255 / base;
				g = g * 255 / base;
				b = b * 255 / base;
			}

			int val = ((r > lbr) && (r < ubr) && (g > lbg) && (g < ubg) && (base > lbb) && (base < ubb)) ? 0 : 255;
			//int val = 0;
			//val += ((r > lbr) && (r < ubr)) ? 0 : 80;
			//val += ((g > lbg) && (g < ubg)) ? 0 : 80;
			//val += ((base > lbb) && (base < ubb)) ? 0 : 80;

			*ptrRs = val;

			ptrRs++;
		}
		ResPtr += ResW;
		ImPtr  += ImW;
	}
	cvReleaseImage (&SmallImg);

	return true;
}

bool CFaceDetector::GetAdaptivSkinHSVMetod (CvRect& rect, float FaceSize, int EyeAngle, int& tx, int& ty, int& tw, int& th, IplImage** rImg)
{
	ptmEye = GetCenterPoint(ptcMLeye,ptcMReye);
	int imW = image->width;
	int imH = image->height;
	int Size = (int)(FaceSize*4);
	int lx = ptmEye.x - (int)(Size*0.5);
	int ly = ptmEye.y - (int)(Size*0.5);
	tx = 0;
	ty = (int)(Size*0.3);
	tw = Size;
	th = (int)(Size*0.6);
	rect = cvRect (lx, ly, Size, Size);
	if (Size < 10)
		return false;

	IplImage* tImg = 0;
	MakeItemsImg (&tImg,image,rect,EyeAngle,cvScalarAll(0),true,tx,ty,tw,th);
	if (!tImg)
		return false;

	float HSVMult = (dNoseToEyesLineDist_Div_EyesMouthDist*1.3<ModelNosePos) ? (float)1.5 : (float)2.0;

	int lbs = (int)(m_Smean - m_Ssdv*HSVMult); //if (lbs < 0) lbs += 240;
	int ubs = (int)(m_Smean + m_Ssdv*HSVMult); //if (ubs > 239) ubs -= 240;
	int lbh = (int)(m_Hmean - m_Hsdv*HSVMult); //if (lbh < 0) lbh += 240;
	int ubh = (int)(m_Hmean + m_Hsdv*HSVMult); //if (ubh > 239) ubh -= 240;

	CvSize SmallImgSize = cvSize(100,60);
	IplImage* SmallImg = cvCreateImage (SmallImgSize,8,3);
	cvResize (tImg,SmallImg,CV_INTER_NN);
	//cvResize (tImg,SmallImg);
	cvReleaseImage (&tImg);

	// Detect skin
	*rImg = cvCreateImage (SmallImgSize,8,1);
	int ResW = (*rImg)->widthStep;
	int ImW  = SmallImg->widthStep;
	uchar* ResPtr = (uchar*)(*rImg)->imageData;
	uchar* ImPtr  = (uchar*)SmallImg->imageData;

	for (int y = 0; y < SmallImg->height; y++)
	{
		uchar* ptrRs = ResPtr;
		uchar* ptrIm = ImPtr;
		for (int x = 0; x < SmallImg->width; x++)
		{
			int B = *ptrIm;	ptrIm++;
			int G = *ptrIm;	ptrIm++;
			int R = *ptrIm;	ptrIm++;

			int base = R+G+B;
			if (base > 0)
			{
				R = R * 255 / base;
				G = G * 255 / base;
				B = B * 255 / base;
			}

			int maxRGB = MAX(MAX(R,G),B);
			int minRGB = MIN(MIN(R,G),B);
			int MaxMinDif = maxRGB - minRGB;
			
			int Saturation, Hue, Cr, Cg, Cb; 

			if (maxRGB == 0)
				Saturation = 0;
			else
				Saturation = MaxMinDif*255/maxRGB;

			if (Saturation == 0)
				Hue = 255;
			else
			{
				Cr = (maxRGB-R)*40/MaxMinDif;
				Cg = (maxRGB-G)*40/MaxMinDif;
				Cb = (maxRGB-B)*40/MaxMinDif;
				if (R == maxRGB)
					Hue = 80 + Cb - Cg;
				else if (G == maxRGB)
					Hue = 160 + Cr - Cb;
				else if (B == maxRGB)
					Hue = Cg - Cr;
				
				if (Hue < 0)
					Hue +=240;
			}

			int val = ((Saturation > lbs) && (Saturation < ubs) && (Hue > lbh) && (Hue < ubh)) ? 0 : 255;

			*ptrRs = val;

			ptrRs++;
		}
		ResPtr += ResW;
		ImPtr  += ImW;
	}
	cvReleaseImage (&SmallImg);

	return true;
}

bool CFaceDetector::FindFace(float FaceSize, int EyeAngle)
{

	CvMat MatL;
	IplImage* rImg = 0;
	int rHistCnt ;
	BYTE LBHist[200],RBHist[200];
	long LDHist[200],RDHist[200];
	int lHistCnt;
	int idx = 0;

	int tx,ty,tw,th;
	CvRect rect;

	if (!GetSkinParameters (FaceSize,EyeAngle))
		return false;

	if (!GetAdaptivSkinHSVMetod (rect,FaceSize,EyeAngle,tx,ty,tw,th,&rImg))
		return false;

	int siW = rImg->width;
	int siH = rImg->height;
	int Size = rect.width;

	int x,y,r;
	FindDarkestCircle (rImg,26,26,&x,&y,&r);

	// Find previous Eyes
	int lEx = ptcLeye.x - rect.x;
	int lEy = ptcLeye.y - rect.y;
	if (abs(EyeAngle) > MIN_STABLE_ANGLE)
			RotatePoint (&lEx,&lEy,(int)(Size*0.5),(int)(Size*0.5),-EyeAngle);
	lEx -= tx;
	lEy -= ty;
	lEx = (int)(lEx * (float)siW/(float)tw);
	lEy = (int)(lEy * (float)siH/(float)th);
	//cvCircle (rImg,cvPoint(lEx,lEy),2,cvScalarAll(150),CV_FILLED);

	int rEx = ptcMReye.x - rect.x;
	int rEy = ptcMReye.y - rect.y;
	if (abs(EyeAngle) > MIN_STABLE_ANGLE)
			RotatePoint (&rEx,&rEy,(int)(Size*0.5),(int)(Size*0.5),-EyeAngle);
	rEx -= tx;
	rEy -= ty;
	rEx = (int)(rEx * (float)siW/(float)tw);
	rEy = (int)(rEy * (float)siH/(float)th);
	//cvCircle (rImg,cvPoint(rEx,rEy),2,cvScalarAll(150),CV_FILLED);

	if (rEx < 0) rEx = 
		20;
	if (rEx >= rImg->width) rEx = rImg->width - 10;
	if (lEx > rEx) lEx = rEx - 1;
	if (lEx < 0) lEx = 10;
	long LHist[200],RHist[200]; 
	long MaxLH = 0, MaxRH = 0;
	float step = (float)(10.0/60.0);

	for (int i = 0; i < 100; i++) 
	{
		LHist[i] = 0;
		RHist[i] = 0;
	}

	for (int x = lEx-3; x >= 0; x--)
	{
		for (int y = 10; y < rImg->height-10; y++)
		{
			LHist[idx] += (long) cvGet2D (rImg, y, x+(int)(y*step)).val[0];
			//cvCircle (rImg,cvPoint(x+y*step,y),0,cvScalarAll(100));
		}
		long val = (idx > 0) ? LHist[idx] - LHist[idx-1] : 0;
		LDHist[idx] = (val > 0) ? val : 0;
		if (MaxLH < LDHist[idx]) MaxLH = LDHist[idx];
		idx++;
	}
	lHistCnt = idx;
	idx = 0;

	for (int x = rEx+3; x < rImg->width; x++)
	{
		for (int y = 10; y < rImg->height-10; y++)
		{
			RHist[idx] += (long) cvGet2D (rImg,y,x-(int)(y*step)).val[0];
			//cvCircle (rImg,cvPoint(x-y*step,y),0,cvScalarAll(150));
		}
		long val = (idx > 0) ? RHist[idx] - RHist[idx-1] : 0;
		RDHist[idx] = (val > 0) ? val : 0;
		if (MaxRH < RDHist[idx]) MaxRH = RDHist[idx];
		idx++;
	}
	rHistCnt = idx;

	if (MaxLH == 0) MaxLH = 1;
	for (int i = 0; i < lHistCnt; i++)
		LBHist[i] = (BYTE)(LDHist[i]*255/MaxLH);
	MatL = cvMat(1,lHistCnt,CV_8U,LBHist); 

	//victor
    if(!CV_IS_MAT(&MatL))
	{
		if( !CV_IS_MATND(&MatL) )
		{
			if( !CV_IS_MATND(&MatL) )
			{
				if( !CV_IS_IMAGE(&MatL))
				{
					if( !CV_IS_SEQ(&MatL) )
					{
						return false;
					}
				}
			}
		}
	}
	//victor

	cvSmooth (&MatL,&MatL,CV_GAUSSIAN,7);
	if (MaxRH == 0) MaxRH = 1;
	for (int i = 0; i < rHistCnt; i++)
		RBHist[i] = (BYTE)(RDHist[i]*255/MaxRH);
	CvMat MatR = cvMat (1,rHistCnt,CV_8U,RBHist);
	cvSmooth (&MatR,&MatR,CV_GAUSSIAN,7);

	MaxLH = 0;
	for (int i = 1; i < lHistCnt; i++)
		if (LBHist[MaxLH] < LBHist[i]) MaxLH = i;
	MaxRH = 0;
	for (int i = 1; i < rHistCnt; i++)
		if (RBHist[MaxRH] < RBHist[i]) MaxRH = i;

	int Ly1 = 17;
	int Lx1 = lEx - MaxLH - 2 ;// + Ly1*step;
	int Ly2 = siH-7;
	int Lx2 = lEx - MaxLH + (int)(Ly2*step) + 3;
	cvLine (rImg,cvPoint (Lx1,Ly1),cvPoint (Lx2,Ly2),cvScalarAll(255));
	int Ry1 = 17;
	int Rx1 = rEx + MaxRH + 2;// - Ry1*step;
	int Ry2 = siH-7;
	int Rx2 = rEx + MaxRH - (int)(Ry2*step) - 3;
	int iter = 0;
	while (Rx2 - Lx2 < 10)
	{
		if (Rx1 < siW-1) 
		{
			Rx1++;
			Rx2++;
		}

		if (Lx1 > 0)
		{
			Lx1--;
			Lx2--;
		}
		iter++;
		if (iter > 30)
			break;
	}
	cvLine (rImg,cvPoint (Rx1,Ry1),cvPoint (Rx2,Ry2),cvScalarAll(255));

	CvPoint pts[4];
	pts[0] = cvPoint (Lx1,Ly1);
	pts[1] = cvPoint (Lx2,Ly2);
	pts[2] = cvPoint (Rx2,Ry2);
	pts[3] = cvPoint (Rx1,Ry1);
	int MinX = gray->width;
	int MinY = gray->height;
	int MaxX = 0;
	int MaxY = 0;
	for (int i = 0; i < 4; i++)
	{
		pts[i].x = (int)(pts[i].x * (float)tw/(float)siW);
		pts[i].y = (int)(pts[i].y * (float)th/(float)siH);
		pts[i].x += tx;
		pts[i].y += ty;
		if (abs(EyeAngle) > MIN_STABLE_ANGLE)
			RotatePoint (&pts[i].x,&pts[i].y,(int)(Size*0.5),(int)(Size*0.5),EyeAngle);
		int x = pts[i].x + rect.x;
		int y = pts[i].y + rect.y;
		pts[i].x = x;
		pts[i].y = y;
		if (MinX > x) MinX = x;
		if (MinY > y) MinY = y;
		if (MaxX < x) MaxX = x;
		if (MaxY < y) MaxY = y;
	}
	if (MaxX - MinX < 1) MaxX = MinX+1;
	if (MaxY - MinY < 1) MaxY = MinY+1;
	int dx = MaxX - MinX;
	int dy = MaxY - MinY;
	MinX -= dx/5;
	MinY -= dy/5;
	MaxX += dx/5;
	MaxY += dy/5;
	if (MinX < 0) MinX = 0; 
	if (MinY < 0) MinY = 0; 
	if (MaxX >= gray->width) MaxX = gray->width-1; 
	if (MaxY >= gray->height) MaxY = gray->height-1; 
	CvRect r1 = cvRect (MinX,MinY,MaxX-MinX,MaxY-MinY);
	for (int i = 0; i < 4; i++)
	{
		pts[i].x -= MinX;
		pts[i].y -= MinY;
	}

	IplImage* mask = cvCreateImage (cvSize(r1.width,r1.height),8,1);
	cvSet(mask,cvScalarAll(255));
	cvFillConvexPoly (mask,pts,4,cvScalarAll(0));
	IplImage* image2 = cvCreateImage (cvGetSize(mask),8,3);
	cvSet (image2,cvScalar(m_BAVG,m_GAVG,m_RAVG));
	cvSetImageROI (image,r1);
	cvCopy (image2,image,mask);
	cvResetImageROI (image);

	IplImage* gray2 = cvCreateImage (cvGetSize(mask),8,1);
	double GrayMean, GraySdv;
	cvSetImageROI (gray,r1);
	cvMean_StdDev (gray,&GrayMean,&GraySdv,mask);
	m_GrAVG = (int)GrayMean;
	cvSet (gray2,cvScalarAll(m_GrAVG));
	cvCopy (gray2,gray,mask);
	cvResetImageROI (gray);

	cvReleaseImage (&gray2);
	cvReleaseImage (&image2);
	cvReleaseImage (&mask);

	if (ECHIWINDOWDBG)
	{
		cvCircle (rImg,cvPoint(x,y),26,cvScalarAll(255),1);
		cvLine (rImg,cvPoint(x-3,y),cvPoint(x+3,y),cvScalarAll(255),1);
		cvLine (rImg,cvPoint(x,y-3),cvPoint(x,y+3),cvScalarAll(255),1);
		CvRect rect;
		rect.x = 1;
		rect.y = 75;
		rect.width  = siW;
		rect.height = siH;
		ControlRect (rect,image->width,image->height);
		cvSetImageROI (image,rect);
		cvCvtColor (rImg,image,CV_GRAY2RGB);
		cvResetImageROI (image);
	}

	cvReleaseImage (&rImg); // rImg no more need

	x = (int)(x * (float)tw/(float)siW);
	y = (int)(y * (float)th/(float)siH);
	x += tx;
	y += ty;

	if (abs(EyeAngle) > MIN_STABLE_ANGLE)
		RotatePoint (&x,&y,(int)(Size*0.5),(int)(Size*0.5),EyeAngle);

	x += rect.x;
	y += rect.y;

	ptcTCenter = cvPoint (x,y);

	return true;
}

bool CFaceDetector::GetPtmMEye (bool bWithMouth)
{
	float XScaleFactor = (int)0.4;
	float nx = (float)(ptcMReye.x - ptcMLeye.x);
	float ny = (float)(ptcMReye.y - ptcMLeye.y);
	float EyeDist = sqrt(nx*nx + ny*ny);
	float MouthToEyesLine = (bWithMouth) ? Point2Line (ptcMLeye,ptcMReye,ptcMMouth) : 1;
	if ((EyeDist != 0) && (MouthToEyesLine != 0))
	{
		ptmEye = GetCenterPoint (ptcMLeye,ptcMReye);
		CvPoint px = (bWithMouth) ? ptcMMouth : cvPoint(ptmEye.x - (int)ny, ptmEye.y + (int)nx);
		CenterXparam = Point2Line (ptmEye,px,ptcMCenter)*XScaleFactor;
		if (GetDistance(ptcMCenter,ptcMLeye) > GetDistance(ptcMCenter,ptcMReye))
			CenterXparam *= -1;

		ptmMEye.x = ptmEye.x + (int)(nx*CenterXparam/EyeDist);
		ptmMEye.y = ptmEye.y + (int)(ny*CenterXparam/EyeDist);

		return true;
	}

	return false;
}

///////////////////////////////////////////////////////////////////
//
// Main function
//
///////////////////////////////////////////////////////////////////
bool CFaceDetector::Detect (IplImage* frame)
{
	if ( !frame ) return false;

	bool res = true;

	try
	{
		PrepareImages (frame);

		if (bUseHaar)
		{
			GetHaarFace();
			if (!bUseHaar)
			{
				// Prepare points
				ptcMLeye  = ptcLeye;
				ptcMReye  = ptcReye;
				ptcMNose  = ptcNose;
				ptcMMouth = ptcMouth;

				int EyeAngle = GetEyesCorner(ptcMLeye,ptcMReye); 
				int FaceSize = GetFaceSize();
				bUseHaar = true;

				if (FindFace(float(FaceSize), EyeAngle) &&
					GetLEyePosition (float(FaceSize),EyeAngle) &&
					GetREyePosition (float(FaceSize),EyeAngle) &&
					GetMouthPosition(float(FaceSize),EyeAngle))
				{
					ptcMLeye  = ptcTLeye; 
					ptcMReye  = ptcTReye;
					ptcMMouth = ptcTMouth;
					ptcMCenter = ptcTCenter;

					if (GetPtmMEye())
					{
						GetNosePosition ((float)FaceSize,EyeAngle);
						ptcMNose = ptcTNose;

						if (PackFaceParameters(true))
						{
							ptcLeye   = ptcTLeye;
							ptcReye   = ptcTReye;
							ptcMouth  = ptcTMouth;
							ptcNose   = ptcTNose1;
							ptcCenter = ptcTCenter;

						//	UpdatePOSITdata(); //victor - fix me
							if (!UpdatePOSITdata()) 
								return false;

							imagePoints.clear();
							modelPoints.clear();
							cvCopy( gray, prev_gray );
						
							bUseHaar = false;
						}
					}
				}
				res = false;
			}
			else
			{
				res = false;
			}
		}
		else 
		{
			PredictPoints();

			//PredictPoints2Part1();
			cvCopy( gray, prev_gray );

			bPackFace = true;

			if ((m_fLEyeHaarEPSMult > 1.5) || (m_fREyeHaarEPSMult > 1.5))
			{
				res = GetHaar(true);
				if (res)
				{
					m_fLEyeHaarEPSMult = 1;
					m_fREyeHaarEPSMult = 1;
				}
			}
			else 
			{
				res = false;
			}
			int FaceSize = GetFaceSize();
			int EyeAngle = GetEyesCorner(ptcMLeye,ptcMReye); 
			if (!FindFace((float)FaceSize, EyeAngle))
				ptcTCenter = ptcMCenter;
			ptcMCenter = ptcTCenter;

			if (!res)
			{
				if ((FaceSize < gray->width/2) && (GetDistance(ptcMLeye,ptcMReye)>10))
				{
					bLeye = GetLEyePosition (float(FaceSize),EyeAngle);
					bReye = GetREyePosition (float(FaceSize),EyeAngle);
					bMouth = GetMouthPosition (float(FaceSize),EyeAngle);
					bNose = false;
					bMeye = false;
					if (bLeye) 
					{
						iLKLEye = 0;
						ptcMLeye = ptcTLeye;
					}
					if (bReye)  
					{
						iLKREye = 0;
						ptcMReye = ptcTReye;
					}
					if (bMouth)
					{
						iLKMouth = 0;
						ptcMMouth = ptcTMouth;
					}

					res = bLeye && bReye && bMouth;

					if (!res)
						res = AnalizeAndRecover3P ();
					if (bLeye && bReye)
						bMeye = GetPtmMEye(bMouth);
					if (bMeye && bMouth)
					{
						int FaceSize = GetFaceSize();
						int EyeAngle = GetEyesCorner(ptcMLeye,ptcMReye); 
						bNose = GetNosePosition (float(FaceSize),EyeAngle);
						ptcMNose = ptcTNose;
						res = bNose;
					}
					if (res)
						res = AnalizeGeometry (FaceSize);
				}
			}
			else
			{
				bMeye = GetPtmMEye();
				if (bMeye)
				{
					int FaceSize = GetFaceSize();
					int EyeAngle = GetEyesCorner(ptcMLeye,ptcMReye); 
					bNose = GetNosePosition ((float)FaceSize,EyeAngle);
					ptcMNose = ptcTNose;
					res = bNose;
				}
				else
				{
					res = false;
				}
			}
			if (res && bPackFace) 
				res = PackFaceParameters();
			static int TrubleCnt = 30;
			if (!res && (TrubleCnt>0))
			{
				TrubleCnt--;

				m_fLEyeHaarEPSMult = 3;
				m_fREyeHaarEPSMult = 3;

				ptcTLeye  = ptcLeye;
				ptcTReye  = ptcReye;
				ptcTMouth = ptcMouth;
				ptcMLeye  = ptcLeye;
				ptcMReye  = ptcReye;
				ptcMMouth = ptcMouth;
				GetNosePosition ((float)FaceSize,EyeAngle);
				ptcMNose = ptcTNose;
				res = true;
			}
			else
				TrubleCnt = 30;
			if (res)
			{
				int ComDelta  =	abs(ptcTLeye.x -  ptcLeye.x)  + abs (ptcTLeye.y -  ptcLeye.y) +
								abs(ptcTReye.x -  ptcReye.x)  + abs (ptcTReye.y -  ptcReye.y) +
								abs(ptcTNose1.x - ptcNose.x)  + abs (ptcTNose1.y -  ptcNose.y) +
								abs(ptcTMouth.x - ptcMouth.x) + abs (ptcTMouth.y - ptcMouth.y);
				ComDelta /= 4;

				int iDelta = (int)(GetFaceSize() * 0.1);

				CorrectPoint (ptcLeye,ptcTLeye,iDelta,ComDelta,6,5,0);
				CorrectPoint (ptcReye,ptcTReye,iDelta,ComDelta,6,5,1);
				CorrectPoint (ptcNose,ptcTNose1,iDelta,ComDelta,6,5,2);
				CorrectPoint (ptcMouth,ptcTMouth,iDelta,ComDelta,6,5,3);
				if (!CorrectPoint (ptcCenter,ptcTCenter,iDelta,ComDelta,6,5,2))
					ptcCenter = ptcTCenter;
			}

			if (res)
			{
				//UpdatePOSITdata ();//victor - fix me
				if (!UpdatePOSITdata()) 
					return false;
		 		if (ECHIWINDOWDBG)
				{
					CvPoint RefPoint;
					for (float y = -0.9f; y < 1.31f; y +=0.1f)
						for (float x = -0.5f; x < 1.51f; x +=0.1f)
							if (y<0.4+1.3*sin((x+0.5)/2*M_PI))
							{
								ModelToView (x,y*ModelMouthPos,image,&RefPoint.x,&RefPoint.y,m_frotation_matrix,m_fOtranslation_vector);
								cvCircle (image,RefPoint,1,cvScalar (0,255,255),1);	
							}
				}
			}
			if (res)
			{
				//PredictPoints2Part2 ();

				if (ptcMDBG)
				{
					cvCircle (image,ptcMLeye,2,cvScalar (255,255,0),1);	
					cvCircle (image,ptcMReye,2,cvScalar (255,255,0),1);	
					cvCircle (image,ptcMNose,2,cvScalar (255,0,0),1);	
					cvCircle (image,ptcMMouth,2,cvScalar (255,0,0),1);	
				}
				if (ECHIWINDOWDBG)
				{
					cvCircle (image,ptcLeye,3,cvScalar (0,255,0),1);	
					cvCircle (image,ptcReye,3,cvScalar (0,255,0),1);	
					cvCircle (image,ptcNose,3,cvScalar (0,255,0),1);	
					cvCircle (image,ptcMouth,3,cvScalar (0,255,0),1);	
				}
				win32::CAutoLockCriticalSection lockingGuard(m_DataLock);
				m_bDetected = true;
			}
			if (!res)
			{
				cvCircle (image,ptcMLeye,3, (bLeye)  ? cvScalar (0,255,255) : cvScalar (0,0,255),1);	
				cvLine   (image,ptcMLeye,ptcTLeye,(bLeye)  ? cvScalar (0,255,255) : cvScalar (0,0,255));	
				cvCircle (image,ptcTLeye,3, (bLeye)  ? cvScalar (0,255,255) : cvScalar (0,0,255),1);	
				cvCircle (image,ptcMReye,3, (bReye)  ? cvScalar (0,255,255) : cvScalar (0,0,255),1);	
				cvLine   (image,ptcMReye,ptcTReye,(bReye)  ? cvScalar (0,255,255) : cvScalar (0,0,255));	
				cvCircle (image,ptcTReye,3, (bReye)  ? cvScalar (0,255,255) : cvScalar (0,0,255),1);	
				cvCircle (image,ptcMNose,3, (bNose)  ? cvScalar (0,255,255) : cvScalar (0,0,255),1);	
				cvLine   (image,ptcMNose,ptcTNose,(bNose)  ? cvScalar (0,255,255) : cvScalar (0,0,255));	
				cvCircle (image,ptcTNose,3, (bNose)  ? cvScalar (0,255,255) : cvScalar (0,0,255),1);	
				cvCircle (image,ptcMMouth,3,(bMouth) ? cvScalar (0,255,255) : cvScalar (255,0,0),1);	
				cvLine   (image,ptcMMouth,ptcTMouth,(bMouth)  ? cvScalar (0,255,255) : cvScalar (0,0,255));	
				cvCircle (image,ptcTMouth,3,(bMouth) ? cvScalar (0,255,255) : cvScalar (255,0,0),1);	
				bUseHaar = true;					
				CannyThresholdLeye = 0;
				CannyThresholdReye = 0;
				Stability = 1000;
				m_fLEyeHaarEPSMult = 1;
				m_fREyeHaarEPSMult = 1;

				win32::CAutoLockCriticalSection lockingGuard(m_DataLock);
					m_bDetected = false;
			}
			if (ECHIWINDOWDBG)
			{
				//m_imageFaceDlg.SetImage(image);
				//Sleep(2000);
				//cvWaitKey(2);
				//SaveImg(image);
			}
		}

	}//try
	catch(cv::Exception & e)
	{
#ifdef _DEBUG
		OutputDebugStringA(e.msg.c_str());
#endif
		e.code = e.code;
		return false;
	}

	return res && !bUseHaar;
}

void CFaceDetector::SaveImg (IplImage* img)
{
	static int Winnumber = 0;
	char strFile[100] = "d:\\img\\\0";
	char numb[10];
	_itoa (Winnumber,numb,10);
	strcat(strFile,numb);
	strcat(strFile,".jpg\0");
	cvSaveImage (strFile,img);
	Winnumber++;
}

///////////////////////////////////////////////////////////////////
//
// Get detection data and work with another thread
//
///////////////////////////////////////////////////////////////////
bool CFaceDetector::IsDetected (void)
{
	win32::CAutoLockCriticalSection lockingGuard(m_DataLock);
	return m_bDetected;
}

CvPoint CFaceDetector::GetCenter (void)
{
	win32::CAutoLockCriticalSection lockingGuard(m_DataLock);
	CvPoint ptmREye = GetCenterPoint (ptcRLeye,ptcRReye);
	CvPoint retP = GetCenterPoint (ptmREye,ptcRMouth);
	return cvPoint(StartX + int(floor(0.5+float(retP.x)*Multipier)),
					int(floor(0.5+float(retP.y)*Multipier)));
}

CvSize CFaceDetector::GetSize (void)
{
	win32::CAutoLockCriticalSection lockingGuard(m_DataLock);
	CvPoint ptmREye = GetCenterPoint (ptcRLeye,ptcRReye);
	return cvSize ((int)(GetDistance(ptcRLeye,ptcRReye)*Multipier),
		(int)(GetDistance(ptmREye,ptcRMouth)*Multipier));
}

int CFaceDetector::GetAngle (void)
{
	win32::CAutoLockCriticalSection lockingGuard(m_DataLock);
	return GetEyesCorner(ptcRLeye,ptcRReye);
}

void CFaceDetector::GetGlPositMatrix (double* GlPositMatrix)
{
	win32::CAutoLockCriticalSection lockingGuard(m_DataLock);

	GlPositMatrix[0] = m_frotation_matrix[0];
	GlPositMatrix[1] = m_frotation_matrix[3];
	GlPositMatrix[2] = m_frotation_matrix[6];
	GlPositMatrix[3] = 0.0;

	GlPositMatrix[4] = m_frotation_matrix[1];
	GlPositMatrix[5] = m_frotation_matrix[4];
	GlPositMatrix[6] = m_frotation_matrix[7];
	GlPositMatrix[7] = 0.0;

	GlPositMatrix[8] =  m_frotation_matrix[2];
	GlPositMatrix[9] =  m_frotation_matrix[5];
	GlPositMatrix[10] = m_frotation_matrix[8];
	GlPositMatrix[11] = 0.0;

	GlPositMatrix[12] =  m_fOtranslation_vector[0];
	GlPositMatrix[13] =  m_fOtranslation_vector[1];
	GlPositMatrix[14] =  m_fOtranslation_vector[2];
	GlPositMatrix[15] = 1.0; //homogeneous
}

CvPoint3D64f CFaceDetector::GetAngles (void) 
{
	return Angles; 
};

CvPoint3D64f CFaceDetector::GetTranslationVector (void) 
{ 
	CvPoint3D64f retdata;

	win32::CAutoLockCriticalSection lockingGuard(m_DataLock);

	retdata.x = m_fOtranslation_vector[0];
	retdata.y = m_fOtranslation_vector[1];
	retdata.z = m_fOtranslation_vector[2];

	return retdata;
};

CvSize CFaceDetector::GetFrameSize (void)
{
//	win32::CAutoLockCriticalSection lockingGuard(m_DataLock);
	return m_FrameSize;
}

void CFaceDetector::SendFrame (IplImage* frame)
{
	if (WaitForSingleObject(m_hReadyWork,0) == WAIT_TIMEOUT)
	{
		SetEvent( m_hFreeFrame );
		return;
	}

	ResetEvent (m_hReadyWork);

	win32::CAutoSetEvent eventGuard( m_hReadyWork );

	Detect (frame);
}
