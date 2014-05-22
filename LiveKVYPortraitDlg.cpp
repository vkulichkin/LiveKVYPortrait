
// LiveKVYPortraitDlg.cpp : implementation file
//

#include "stdafx.h"
#include "LiveKVYPortrait.h"
#include "LiveKVYPortraitDlg.h"
#include "afxdialogex.h"
#include "MyImage.h"

#include <opencv2\opencv.hpp>


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define MAX_FRAMES 50
#define FRAME_TIME 100 //ms
#define MAX_FRAME_WIDTH 1280
#define MAX_FRAME_HEIGHT 960

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CLiveKVYPortraitDlg dialog



CLiveKVYPortraitDlg::CLiveKVYPortraitDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CLiveKVYPortraitDlg::IDD, pParent)
	, m_nFrameNumber(0)
	, m_strDisplaySec(_T(""))
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CLiveKVYPortraitDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_BUTTON_LOADIMAGE, m_buttonLoadImage);
	DDX_Control(pDX, IDC_BUTTON_START, m_buttonStart);
	DDX_Control(pDX, IDC_BUTTON_SAVETOVIDEO, m_buttonSaveToVideo);
	DDX_Control(pDX, IDC_STATIC_IMAGE, m_staticImage);
	DDX_Control(pDX, IDC_SLIDER1, m_slider);
	DDX_Text(pDX, IDC_EDIT_SEC, m_strDisplaySec);
}

BEGIN_MESSAGE_MAP(CLiveKVYPortraitDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_LOADIMAGE, &CLiveKVYPortraitDlg::OnBnClickedButtonLoadimage)
	ON_WM_CLOSE()
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON_START, &CLiveKVYPortraitDlg::OnBnClickedButtonStart)
	ON_NOTIFY(NM_RELEASEDCAPTURE, IDC_SLIDER1, &CLiveKVYPortraitDlg::OnNMReleasedcaptureSlider1)
END_MESSAGE_MAP()


// CLiveKVYPortraitDlg message handlers

BOOL CLiveKVYPortraitDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	m_buttonSaveToVideo.EnableWindow(FALSE);
	m_buttonStart.EnableWindow(FALSE);
	
	m_slider.SetRange(0, MAX_FRAMES -1);
	m_slider.EnableWindow(FALSE);
	m_slider.SetLineSize(1);
	m_slider.SetTicFreq(1);

	m_strDisplaySec.Format(L"0.0 Sec");

	UpdateData(FALSE);
	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CLiveKVYPortraitDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CLiveKVYPortraitDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		m_lockToArray.Lock();
		int size = m_arrImages.GetSize();
		if (size)
		{
			int nFrameNumber = m_nFrameNumber >= MAX_FRAMES ? 0 : m_nFrameNumber;
			CMyImage* pImage = (CMyImage*)m_arrImages.GetAt(nFrameNumber);
			CImage* pCImg = pImage->GetImage();
			if (pCImg)
			{
				pCImg->BitBlt(::GetDC(m_staticImage.m_hWnd), 0, 0);
			}
		}
		m_lockToArray.Unlock();
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CLiveKVYPortraitDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CLiveKVYPortraitDlg::OnBnClickedButtonLoadimage()
{
	// TODO: Add your control notification handler code here
	CFileDialog* pDlg = new CFileDialog(TRUE, L".jpg", NULL, 
		OFN_HIDEREADONLY | OFN_FILEMUSTEXIST, 
		L"Image Files (*.jpg)|*.jpg|All Files (*.*)|*.*||");
	if (pDlg->DoModal() == IDOK)
	{
		CString fileImagePath = pDlg->GetPathName();
		delete pDlg;
		LoadMyImage(fileImagePath);
	}
	else
		delete pDlg;
}

void CLiveKVYPortraitDlg::MessageImage(CStringA strMsg, unsigned char red, unsigned char green, unsigned char blue)
{
	int iOriginalWidth, iOriginalHeight;
	RECT r;
	CMyImage* pImage = NULL;
	CImage* pCImg = NULL;
	cv::Size sizeShow;

	m_staticImage.GetClientRect(&r);
	iOriginalWidth = r.right;
	iOriginalHeight = r.bottom;
	sizeShow = cv::Size(iOriginalWidth, iOriginalHeight);

	cv::Mat matEmpty(sizeShow, CV_8UC3);
	matEmpty.setTo(cv::Scalar(0));
	if (strMsg.GetLength() != 0)
	{
		cv::putText(matEmpty, strMsg.GetBuffer(), cv::Point(10, 100), CV_FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar( blue, green, red)); 
	}
	pImage = new CMyImage(&matEmpty, sizeShow);
	pCImg = pImage->GetImage(); 
	if (pCImg)
	{
		pCImg->BitBlt(::GetDC(m_staticImage.m_hWnd), 0, 0);
	}
	delete pImage;
}

bool CLiveKVYPortraitDlg::LoadMyImage(CString strPathW)
{
	m_buttonStart.EnableWindow(FALSE);
	m_slider.EnableWindow(FALSE);
	KillTimer(FrameTimerID);

	cv::Mat matSrc;
	CString strErr;
	int iOriginalWidth, iOriginalHeight;
	CStringA strPath(strPathW);
	RECT r;
	CMyImage* pImage = NULL;
	CImage* pCImg = NULL;
	cv::Size sizeShow;
	
	m_staticImage.GetClientRect(&r);
	iOriginalWidth = r.right;
	iOriginalHeight = r.bottom;
	
	matSrc = cv::imread(strPath.GetBuffer());
	if (!matSrc.data)
	{
		strErr.Format(L"Cannot load this image:\n%s", strPath);
		AfxMessageBox(strErr);
		return false;
	}

	if (matSrc.size().width * matSrc.size().height > MAX_FRAME_WIDTH * MAX_FRAME_HEIGHT)
	{
		int iWidth, iHeight;
		if (matSrc.size().width >=  matSrc.size().height)
		{
			iWidth = MAX_FRAME_WIDTH;
			iHeight = MAX_FRAME_HEIGHT; 
		}
		else
		{
			iWidth = MAX_FRAME_HEIGHT;
			iHeight = MAX_FRAME_WIDTH; 
		}
		cv::Mat matTmp(iHeight, iWidth, CV_8UC3);
		cv::resize(matSrc, matTmp, cv::Size(iHeight, iWidth), 0, 0, cv::INTER_LINEAR);
		matSrc = matTmp;
	}
	sizeShow = cv::Size(iOriginalWidth, iOriginalHeight);

	//clear a show window
	MessageImage("Wait...", 255, 255, 255);

	int iRes;
	bool bFacedetect = true;
	m_lockToArray.Lock();
	ClearImageArray();
	for (int i = 0; i < MAX_FRAMES; i++)
	{
		pImage = new CMyImage(&matSrc, sizeShow);
		if (!i)
		{
			if ((iRes = pImage->IsFace()) < 1)
			{
				delete pImage;
				bFacedetect = false;
				break;
			}
		}
		m_arrImages.Add(pImage);
	}
	m_lockToArray.Unlock();
	if (!bFacedetect)
	{
		switch(iRes)
		{
		//case -2:
		//	AfxMessageBox(L"Cannot detect a body on this photo.");
		//	break;
		case -1:
			AfxMessageBox(L"Problem with loading haarcascade.xml");
			MessageImage("Failed.", 255, 0, 0);
			break;
		case 0:
			AfxMessageBox(L"Cannot detect a person on this photo.");
			MessageImage("Failed.", 255, 0, 0);
			break;
		default:
			break;
		}
		return false;
	}

	if (m_arrImages.GetSize() != 0)
	{
		pImage = (CMyImage*)m_arrImages.GetAt(m_nFrameNumber);
		pCImg = pImage->GetImage(); 
		if (pCImg)
			pCImg->BitBlt(::GetDC(m_staticImage.m_hWnd), 0, 0);
	}

	m_buttonStart.EnableWindow(TRUE);
	return true;
}


void CLiveKVYPortraitDlg::ClearImageArray(void)
{
	CMyImage* pImg = NULL;
	int size = m_arrImages.GetSize();
	if (size)
	{
		for (int i = 0; i < size; i++)
		{
			pImg = (CMyImage*)m_arrImages.GetAt(i);
			delete pImg;
			pImg = NULL;
		}
		m_arrImages.RemoveAll();
	}
}


void CLiveKVYPortraitDlg::OnClose()
{
	// TODO: Add your message handler code here and/or call default
	KillTimer(FrameTimerID);
	ClearImageArray();
	CDialogEx::OnClose();
}


void CLiveKVYPortraitDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	if (nIDEvent == FrameTimerID)
	{
		m_lockToArray.Lock();
		if (m_arrImages.GetSize())
		{
			CImage* pCImg;
			CMyImage* pImage; 
			if (m_nFrameNumber == MAX_FRAMES -1 )
			{//finish
				KillTimer(FrameTimerID);
				m_nFrameNumber = 0;
				m_slider.SetPos(m_nFrameNumber);
				m_buttonStart.EnableWindow(TRUE);
				m_buttonLoadImage.EnableWindow(TRUE);
				CalculateDisplaySec(m_nFrameNumber);
				pImage = (CMyImage*)m_arrImages.GetAt(m_nFrameNumber);
			}
			else
			{
				m_slider.SetPos(m_nFrameNumber);
				CalculateDisplaySec(m_nFrameNumber);
				pImage = (CMyImage*)m_arrImages.GetAt(m_nFrameNumber++);
			}
			pCImg = pImage->GetImage();
			if (pCImg)
			{
				pCImg->BitBlt(::GetDC(m_staticImage.m_hWnd), 0, 0);
			}
			UpdateData(FALSE);
		}
		m_lockToArray.Unlock();
	}
	CDialogEx::OnTimer(nIDEvent);
}


void CLiveKVYPortraitDlg::OnBnClickedButtonStart()
{
	// TODO: Add your control notification handler code here
	m_buttonStart.EnableWindow(FALSE);
	m_buttonLoadImage.EnableWindow(FALSE);
	SetTimer(FrameTimerID, FRAME_TIME, NULL);
	m_slider.EnableWindow(TRUE);
	m_nFrameNumber = 0;
}


void CLiveKVYPortraitDlg::OnNMReleasedcaptureSlider1(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO: Add your control notification handler code here
	m_lockToArray.Lock();
	m_nFrameNumber = m_slider.GetPos();
	int size = m_arrImages.GetSize(); 
	if (size )
	{
		CMyImage* pImage = (CMyImage*)m_arrImages.GetAt(m_nFrameNumber);
		CalculateDisplaySec(m_nFrameNumber);
		CImage* pCImg = pImage->GetImage();
		if (pCImg)
		{
			pCImg->BitBlt(::GetDC(m_staticImage.m_hWnd), 0, 0);
		}
		UpdateData(FALSE);
	}
	m_lockToArray.Unlock();

	*pResult = 0;
}


void CLiveKVYPortraitDlg::CalculateDisplaySec(int nFameNumber)
{
	float fFullTime = (float)(nFameNumber * FRAME_TIME) / 1000;
	m_strDisplaySec.Format(L"%.1f Sec", fFullTime);
}
