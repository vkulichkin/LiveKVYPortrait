#include "StdAfx.h"
#include "MyImage.h"


CMyImage::CMyImage(cv::Mat* pmatImg, cv::Size sizeShow)
	: m_pImg(NULL)
	//, m_pmatCVImgShow(NULL)
{
	if (!pmatImg)
		return;
	m_matCVImg = pmatImg->clone();
	m_sizeShow = sizeShow;
}


CMyImage::~CMyImage(void)
{
	//if (m_pmatCVImgShow)
	//	delete m_pmatCVImgShow;
	if (m_pImg)
	{
		m_pImg->ReleaseDC();
		delete m_pImg;
	}
}


CImage* CMyImage::GetImage(void)
{
	if (!m_matCVImg.data)
		return m_pImg;
	if (m_pImg)
	{
		m_pImg->ReleaseDC();
		delete m_pImg;
		m_pImg = NULL;
	}

	//HDC hDC = CreateCompatibleDC(NULL);
	BITMAPINFO bitInfo;
	bitInfo.bmiHeader.biBitCount = 24;
	bitInfo.bmiHeader.biWidth = m_sizeShow.width;
	bitInfo.bmiHeader.biHeight = m_sizeShow.height;
	bitInfo.bmiHeader.biPlanes = 1;
	bitInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bitInfo.bmiHeader.biCompression = BI_RGB;
    bitInfo.bmiHeader.biClrImportant = 
					bitInfo.bmiHeader.biClrUsed = 
					bitInfo.bmiHeader.biSizeImage = 
                    bitInfo.bmiHeader.biXPelsPerMeter =
                    bitInfo.bmiHeader.biYPelsPerMeter = 0;

	m_pImg = new CImage;
	if (!m_pImg)
		return NULL;

	cv::Mat matImgTmp;
	if (m_matCVImg.size() !=  m_sizeShow)
	{
		matImgTmp = cv::Mat(m_sizeShow, CV_8UC3);
		cv::resize(m_matCVImg, matImgTmp, 	
			m_sizeShow, 0, 0, cv::INTER_AREA);
	}
	else
		matImgTmp = m_matCVImg.clone();
	cv::flip(matImgTmp, matImgTmp, 0);

	m_pImg->Create(m_sizeShow.width,  m_sizeShow.height, 24);
	StretchDIBits(m_pImg->GetDC(), 0, 0, 
		m_sizeShow.width, m_sizeShow.height, 0, 0, 
		m_sizeShow.width, m_sizeShow.height, 
		matImgTmp.data, &bitInfo, DIB_RGB_COLORS, SRCCOPY);  
//	DeleteDC(hDC);

	return m_pImg;
}


int CMyImage::IsFace(void* pmatImgStart)
{
	cv::Mat* pmatImg;
	if (!pmatImgStart)
		pmatImg = &m_matCVImg;
	else
		pmatImg = (cv::Mat*) pmatImgStart;

	cv::CascadeClassifier cascadeFace;
//	cv::CascadeClassifier cascadeEyes;
	std::string strDatabasePath;
	std::vector<cv::Rect> vecFaces;
	std::vector<cv::Rect> vecBody;
	cv::Mat matGrayFrame;
	int iRes = 0;
	cv::Rect rectBody = cv::Rect();
	cv::Rect rectFace = cv::Rect();
	cv::Rect rectFullBody = cv::Rect();
	int numberRect;
	int nYardage;
	
	cv::cvtColor(*pmatImg, matGrayFrame, CV_BGR2GRAY );

#ifdef _DEBUG
	strDatabasePath = "D:\\vc.rac\\LiveKVYPortrait\\LiveKVYPortrait\\";
#else
	char strPath[MAX_PATH] = {0};
	GetCurrentDirectoryA(MAX_PATH, strPath);
	strDatabasePath += strPath;
	strDatabasePath += L'\\';
#endif
	strDatabasePath += "haarcascades\\";
	if (!cascadeFace.load(strDatabasePath + "haarcascade_mcs_upperbody.xml")) 
		return -2;
	cascadeFace.detectMultiScale(matGrayFrame, vecBody, 1.15, 2, 
		0 |  CV_HAAR_DO_CANNY_PRUNING, //| CV_HAAR_SCALE_IMAGE, // 
		cv::Size(10, 40));
	if (!vecBody.empty())
	{
		numberRect = 0;
		nYardage = 0;
		for( size_t i = 0; i <  vecBody.size(); i++ )
		{
			cv::Rect rc =  vecBody[i];
			int nYardageCurrent = rc.width * rc.height;
			if (nYardage < nYardageCurrent)
			{
				nYardage  = nYardageCurrent;
				numberRect = i;
			}
		}
		rectBody = vecBody[numberRect];
		//delete me
		cv::rectangle(*pmatImg, rectBody, cv::Scalar( 0, 255, 0), 4, 8, 0);
		//delete me
		iRes = 1;
	}

	if (!cascadeFace.load(strDatabasePath + "haarcascade_frontalface_alt.xml"))
		return -1;

	cascadeFace.detectMultiScale(matGrayFrame, vecFaces, 1.1, 2, 
		0,//CV_HAAR_SCALE_IMAGE | CV_HAAR_DO_CANNY_PRUNING, 
		cv::Size(20, 20));
	if (vecFaces.empty())
		return iRes; 
	iRes ++;

	if (iRes == 2)
	{//we have upperbody
		//if (!cascadeEyes.load(strDatabasePath + "haarcascade_eye_tree_eyeglasses.xml"))
		//	return 0;
		for( size_t i = 0; i <  vecFaces.size(); i++ )
		{
			//cv::Mat  matFaceROI = matGrayFrame(vecFaces[i]);
			//std::vector<cv::Rect> vecEyes;
			//cascadeEyes.detectMultiScale(matFaceROI, vecEyes, 1.1, 2,  
			//	CV_HAAR_SCALE_IMAGE,
			//	cvSize(20, 20));
			//if (vecEyes.empty())
			//	continue;
		//	iRes = 1;

			if (rectBody.x <= vecFaces[i].x &&
				rectBody.y <= vecFaces[i].y &&
				rectBody.x + rectBody.width >= vecFaces[i].x + vecFaces[i].width &&
				rectBody.y + rectBody.height >= vecFaces[i].y + vecFaces[i].height)
			{
				rectFace = vecFaces[i];
			//delete me
				cv::rectangle(*pmatImg, vecFaces[i], cv::Scalar(255, 0, 255), 4, 8, 0);
			//delete me
			}

			//delete me
			//for( size_t j = 0; j < vecEyes.size(); j++ )
			//{
			//	cv::Point pointCenter2(vecFaces[i].x + vecEyes[j].x + (int)(vecEyes[j].width * 0.5), 
			//		vecFaces[i].y + vecEyes[j].y + (int)(vecEyes[j].height * 0.5));
			//	int radius = cvRound( (vecEyes[j].width + vecEyes[j].height) * 0.25 );
			//	cv::circle(*pmatImg, pointCenter2, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
			//}
			//delete me
		}
	}
	else
	{//only face
		for( size_t i = 0; i <  vecFaces.size(); i++ )
		{
			numberRect = 0;
			nYardage = 0;
			cv::Rect rc =  vecFaces[i];
			int nYardageCurrent = rc.width * rc.height;
			if (nYardage < nYardageCurrent)
			{
				nYardage  = nYardageCurrent;
				numberRect = i;
			}
		}
		rectFace = vecFaces[numberRect];
		//delete me
		cv::rectangle(*pmatImg, rectFace, cv::Scalar(255, 0, 255), 4, 8, 0);
		//delete me
		rectBody = rectFace;
	}

	return iRes;
}

void CMyImage::RotateImage(const void* src, void** dst, int angleDegrees)
{//???? replaced by cv::flip()
	if (!src)
		return;
	cv::Mat* pmatSrc = (cv::Mat*)src; 
	
	//take the dimention of original image
	cv::Size size = pmatSrc->size(); 
	int w = size.width;
    int h = size.height; 

    // Make a new image for the result
	cv::Mat* pmatRotated = new cv::Mat(h, w, CV_8UC3);
	if (!pmatRotated)
		return;
	
	cv::Point2f src_center( pmatSrc->cols/2.0F, pmatSrc->rows/2.0F);
    cv::Mat M = cv::getRotationMatrix2D(src_center,  angleDegrees, 1.0);

    // Transform the image
	cv::warpAffine(*pmatSrc, *pmatRotated, M, cv::Size(w, h));
	*dst = pmatRotated;
    return;
}
