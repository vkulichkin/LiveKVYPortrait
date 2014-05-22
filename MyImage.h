#pragma once
#include <opencv2\opencv.hpp>

class CMyImage
{
public:
	CMyImage(cv::Mat* pmatImg, cv::Size sizeShow);
	~CMyImage(void);
	cv::Size m_sizeShow;
private:
	cv::Mat m_matCVImg;
//	cv::Mat* m_pmatCVImgShow;
	CImage* m_pImg;
	void RotateImage(const void* src, void** dst, int angleDegrees);
public:
	CImage* GetImage(void);
	int IsFace(void* pmatImgStart = NULL);
};

