
// LiveKVYPortraitDlg.h : header file
//

#pragma once
#include "afxwin.h"
#include "afxcmn.h"
#include "afxmt.h"

enum
{
	FrameTimerID = 1957
};

// CLiveKVYPortraitDlg dialog
class CLiveKVYPortraitDlg : public CDialogEx
{
// Construction
public:
	CLiveKVYPortraitDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_LIVEKVYPORTRAIT_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
private:
	CButton m_buttonLoadImage;
	CButton m_buttonStart;
	CButton m_buttonSaveToVideo;
public:
	afx_msg void OnBnClickedButtonLoadimage();
private:
	CStatic m_staticImage;
	CSliderCtrl m_slider;
	bool LoadMyImage(CString strPathW);
	CPtrArray m_arrImages;
	void ClearImageArray(void);
public:
	afx_msg void OnClose();
private:
	int m_nFrameNumber;
public:
	afx_msg void OnTimer(UINT_PTR nIDEvent);
private:
	CCriticalSection  m_lockToArray;
public:
	afx_msg void OnBnClickedButtonStart();
	afx_msg void OnNMReleasedcaptureSlider1(NMHDR *pNMHDR, LRESULT *pResult);
	void MessageImage(CStringA strMsg, unsigned char red, unsigned char green, unsigned char blue);
private:
	CString m_strDisplaySec;
	void CalculateDisplaySec(int nFameNumber);
};
