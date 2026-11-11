/*
 * motor_gui.c
 * STM32 모터 파워(%) / 인코더 증가 이동 / 리셋 / 목표 인코더 위치 이동 제어 GUI + 실시간 2차원 그래프 (Win32 C)
 *
 * 기능:
 *  1. 모터 파워(%) 입력 (0 ~ 100%)
 *  2. 밑에 입력한 만큼 인코더 값이 증가하게 이동 (상대 위치 이동)
 *  3. 인코더 리셋 버튼 (현재 인코더 값 0으로 초기화)
 *  4. 원하는 인코더 값을 입력하면 그 값으로 이동 (절대 위치 이동)
 *  5. 입력창 오른쪽 옆에 실시간 2차원 그래프 표시:
 *     - 파란색 실선: 현재 인코더 값 (시간에 따른 2차원 곡선)
 *     - 빨간색 직선: 목표 인코더 값 (수평 직선)
 *  6. USART (115200 bps) 시리얼 통신 연동 + 미연결 시 자체 시뮬레이션 동작 지원
 */

#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 컨트롤 ID 정의 */
#define ID_EDIT_COM         101
#define ID_BTN_CONNECT      102
#define ID_EDIT_POWER       103
#define ID_BTN_SETPOWER     104
#define ID_EDIT_INC         105
#define ID_BTN_INCMOVE      106
#define ID_BTN_RESET        107
#define ID_EDIT_TARGET      108
#define ID_BTN_TARGETMOVE   109
#define ID_BTN_STOP         110
#define ID_BTN_CLEARGRAPH   111
#define ID_TIMER_UPDATE     201

/* 2차원 그래프 히스토리 버퍼 크기 (200 샘플 = 10초) */
#define GRAPH_MAX_POINTS    200

/* UI 핸들 */
static HWND hEditCom, hBtnConnect, hStaticConnStatus;
static HWND hEditPower, hBtnSetPower;
static HWND hEditInc, hBtnIncMove;
static HWND hStaticCurEnc, hStaticTgtEnc, hStaticState;
static HWND hBtnReset;
static HWND hEditTarget, hBtnTargetMove, hBtnStop;
static HWND hBtnClearGraph;
static HFONT hFontTitle, hFontNormal, hFontBigEnc, hFontSmall;

/* 상태 변수 */
static HANDLE hSerial = INVALID_HANDLE_VALUE;
static long current_encoder = 0;
static long target_encoder  = 0;
static int  motor_power     = 50;
static int  motor_moving    = 0; /* 0: 정지, 1: 정방향(CW), 2: 역방향(CCW) */

/* 2차원 그래프 샘플 배열 */
static long graph_cur_hist[GRAPH_MAX_POINTS];
static long graph_tgt_hist[GRAPH_MAX_POINTS];
static int  graph_count = 0;

/* 오른쪽 그래프 영역 좌표 */
static const RECT rcGraphPanel = { 520, 42, 1085, 500 };

static char rx_line_buf[256];
static int  rx_line_idx = 0;

/* 그래프 샘플 추가 */
static void Push_GraphSample(void)
{
    if (graph_count < GRAPH_MAX_POINTS)
    {
        graph_cur_hist[graph_count] = current_encoder;
        graph_tgt_hist[graph_count] = target_encoder;
        graph_count++;
    }
    else
    {
        memmove(&graph_cur_hist[0], &graph_cur_hist[1], sizeof(long) * (GRAPH_MAX_POINTS - 1));
        memmove(&graph_tgt_hist[0], &graph_tgt_hist[1], sizeof(long) * (GRAPH_MAX_POINTS - 1));
        graph_cur_hist[GRAPH_MAX_POINTS - 1] = current_encoder;
        graph_tgt_hist[GRAPH_MAX_POINTS - 1] = target_encoder;
    }
}

/* 시리얼 명령 전송 */
static void Serial_SendCmd(const char *cmd)
{
    if (hSerial != INVALID_HANDLE_VALUE)
    {
        DWORD written = 0;
        WriteFile(hSerial, cmd, (DWORD)strlen(cmd), &written, NULL);
    }
}

/* 화면의 인코더 표시 라벨 갱신 */
static void Update_DisplayLabels(void)
{
    char buf[128];

    snprintf(buf, sizeof(buf), "현재 인코더 값 :  %ld", current_encoder);
    SetWindowTextA(hStaticCurEnc, buf);

    snprintf(buf, sizeof(buf), "목표 인코더 값 :  %ld   (설정 파워: %d%%)", target_encoder, motor_power);
    SetWindowTextA(hStaticTgtEnc, buf);

    if (motor_moving == 1)
        snprintf(buf, sizeof(buf), "모터 상태 :  정방향 회전 중 ▲");
    else if (motor_moving == 2)
        snprintf(buf, sizeof(buf), "모터 상태 :  역방향 회전 중 ▼");
    else
        snprintf(buf, sizeof(buf), "모터 상태 :  정지 ■");

    SetWindowTextA(hStaticState, buf);
}

/* STM32로부터 수신한 한 줄(STAT:ENC=...,TGT=...,PWR=...,MOV=...) 파싱 */
static void Parse_TelemetryLine(const char *line)
{
    if (strncmp(line, "STAT:", 5) == 0)
    {
        long enc = current_encoder, tgt = target_encoder, pwr = motor_power;
        unsigned int mov = 0;
        if (sscanf(line + 5, "ENC=%ld,TGT=%ld,PWR=%ld,MOV=%u", &enc, &tgt, &pwr, &mov) >= 3)
        {
            current_encoder = enc;
            target_encoder  = tgt;
            motor_power     = (int)pwr;
            motor_moving    = (int)mov;
            Update_DisplayLabels();
        }
    }
}

/* 실시간 2차원 그래프 그리기 (현재 인코더 곡선 + 목표값 직선) */
static void Draw_EncoderGraph(HDC hdc)
{
    int width  = rcGraphPanel.right - rcGraphPanel.left;
    int height = rcGraphPanel.bottom - rcGraphPanel.top;

    /* 더블 버퍼링으로 깜빡임 방지 */
    HDC memDC = CreateCompatibleDC(hdc);
    HBITMAP memBmp = CreateCompatibleBitmap(hdc, width, height);
    HBITMAP oldBmp = (HBITMAP)SelectObject(memDC, memBmp);

    RECT rcLocal = { 0, 0, width, height };
    HBRUSH hBrWhite = CreateSolidBrush(RGB(255, 255, 255));
    FillRect(memDC, &rcLocal, hBrWhite);
    DeleteObject(hBrWhite);

    int padL = 62, padR = 20, padT = 38, padB = 34;
    int plotW = width - padL - padR;
    int plotH = height - padT - padB;

    /* Y축 최소/최대 자동 스케일 계산 */
    long minVal = (current_encoder < target_encoder) ? current_encoder : target_encoder;
    long maxVal = (current_encoder > target_encoder) ? current_encoder : target_encoder;
    if (0 < minVal) minVal = 0;
    if (100 > maxVal) maxVal = 100;

    for (int i = 0; i < graph_count; i++)
    {
        if (graph_cur_hist[i] < minVal) minVal = graph_cur_hist[i];
        if (graph_cur_hist[i] > maxVal) maxVal = graph_cur_hist[i];
        if (graph_tgt_hist[i] < minVal) minVal = graph_tgt_hist[i];
        if (graph_tgt_hist[i] > maxVal) maxVal = graph_tgt_hist[i];
    }

    double span = (double)(maxVal - minVal);
    if (span < 40.0)
    {
        double mid = (maxVal + minVal) * 0.5;
        minVal = (long)(mid - 20.0);
        maxVal = (long)(mid + 20.0);
        span = 40.0;
    }
    else
    {
        long margin = (long)(span * 0.15);
        minVal -= margin;
        maxVal += margin;
        span = (double)(maxVal - minVal);
    }

    SetBkMode(memDC, TRANSPARENT);
    HFONT oldFont = (HFONT)SelectObject(memDC, hFontSmall);

    /* 상단 범례 출력 */
    char legendBuf[160];
    long err = target_encoder - current_encoder;
    SetTextColor(memDC, RGB(37, 99, 235));
    snprintf(legendBuf, sizeof(legendBuf), "■ 현재 인코더: %ld", current_encoder);
    TextOutA(memDC, padL, 10, legendBuf, (int)strlen(legendBuf));

    SetTextColor(memDC, RGB(220, 38, 38));
    snprintf(legendBuf, sizeof(legendBuf), "― 목표값(직선): %ld", target_encoder);
    TextOutA(memDC, padL + 165, 10, legendBuf, (int)strlen(legendBuf));

    SetTextColor(memDC, RGB(51, 65, 85));
    snprintf(legendBuf, sizeof(legendBuf), "오차: %+ld", err);
    TextOutA(memDC, padL + 345, 10, legendBuf, (int)strlen(legendBuf));

    /* 1. 가로 격자선 및 Y축 수치 */
    HPEN hPenGrid = CreatePen(PS_SOLID, 1, RGB(226, 232, 240));
    HPEN oldPen = (HPEN)SelectObject(memDC, hPenGrid);
    SetTextColor(memDC, RGB(71, 85, 105));

    for (int i = 0; i <= 5; i++)
    {
        double v = minVal + (span * i) / 5.0;
        int y = padT + plotH - (int)(((v - minVal) / span) * plotH);
        MoveToEx(memDC, padL, y, NULL);
        LineTo(memDC, padL + plotW, y);

        char yLbl[32];
        snprintf(yLbl, sizeof(yLbl), "%ld", (long)(v + (v >= 0 ? 0.5 : -0.5)));
        RECT rcLbl = { 2, y - 9, padL - 6, y + 9 };
        DrawTextA(memDC, yLbl, -1, &rcLbl, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
    }

    /* 2. 세로 격자선 */
    for (int i = 0; i <= 5; i++)
    {
        int x = padL + (plotW * i) / 5;
        MoveToEx(memDC, x, padT, NULL);
        LineTo(memDC, x, padT + plotH);
    }

    /* 3. 목표 인코더 값 수평 직선 (빨간색 직선) */
    int yTarget = padT + plotH - (int)(((target_encoder - minVal) / span) * plotH);
    HPEN hPenTarget = CreatePen(PS_SOLID, 2, RGB(220, 38, 38));
    SelectObject(memDC, hPenTarget);
    MoveToEx(memDC, padL, yTarget, NULL);
    LineTo(memDC, padL + plotW, yTarget);

    char tgtTag[48];
    snprintf(tgtTag, sizeof(tgtTag), "목표: %ld", target_encoder);
    SetTextColor(memDC, RGB(220, 38, 38));
    TextOutA(memDC, padL + 8, (yTarget > padT + 18) ? (yTarget - 18) : (yTarget + 4), tgtTag, (int)strlen(tgtTag));

    /* 4. 현재 인코더 값 2차원 그래프 곡선 (파란색 실선) */
    HPEN hPenCur = CreatePen(PS_SOLID, 3, RGB(37, 99, 235));
    SelectObject(memDC, hPenCur);
    if (graph_count > 0)
    {
        for (int i = 0; i < graph_count; i++)
        {
            int x = padL + (i * plotW) / (GRAPH_MAX_POINTS - 1);
            int y = padT + plotH - (int)(((graph_cur_hist[i] - minVal) / span) * plotH);
            if (i == 0)
                MoveToEx(memDC, x, y, NULL);
            else
                LineTo(memDC, x, y);
        }
    }

    /* 5. 테두리 */
    HPEN hPenBorder = CreatePen(PS_SOLID, 1, RGB(148, 163, 184));
    SelectObject(memDC, hPenBorder);
    SelectObject(memDC, GetStockObject(NULL_BRUSH));
    Rectangle(memDC, padL, padT, padL + plotW, padT + plotH);

    /* 화면으로 복사 */
    BitBlt(hdc, rcGraphPanel.left, rcGraphPanel.top, width, height, memDC, 0, 0, SRCCOPY);

    SelectObject(memDC, oldPen);
    SelectObject(memDC, oldFont);
    DeleteObject(hPenGrid);
    DeleteObject(hPenTarget);
    DeleteObject(hPenCur);
    DeleteObject(hPenBorder);
    SelectObject(memDC, oldBmp);
    DeleteObject(memBmp);
    DeleteDC(memDC);
}

/* 시리얼 포트 열기/닫기 */
static void Toggle_SerialConnection(HWND hwnd)
{
    if (hSerial != INVALID_HANDLE_VALUE)
    {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        SetWindowTextA(hBtnConnect, "포트 연결");
        SetWindowTextA(hStaticConnStatus, "상태: 미연결 (시뮬레이션 모드)");
        return;
    }

    char comName[32] = {0};
    GetWindowTextA(hEditCom, comName, sizeof(comName));

    char fullPortName[64];
    snprintf(fullPortName, sizeof(fullPortName), "\\\\.\\%s", comName);

    hSerial = CreateFileA(fullPortName,
                          GENERIC_READ | GENERIC_WRITE,
                          0,
                          NULL,
                          OPEN_EXISTING,
                          0,
                          NULL);

    if (hSerial == INVALID_HANDLE_VALUE)
    {
        MessageBoxA(hwnd,
                    "시리얼 포트를 열 수 없습니다.\r\n"
                    "포트 번호를 확인하거나, 미연결 상태에서 시뮬레이션 모드로 테스트하세요.",
                    "포트 연결 실패",
                    MB_ICONWARNING | MB_OK);
        return;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (GetCommState(hSerial, &dcb))
    {
        dcb.BaudRate = CBR_115200;
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity   = NOPARITY;
        SetCommState(hSerial, &dcb);
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadTotalTimeoutConstant    = 0;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant   = 10;
    SetCommTimeouts(hSerial, &timeouts);

    SetWindowTextA(hBtnConnect, "연결 해제");
    char statusMsg[96];
    snprintf(statusMsg, sizeof(statusMsg), "상태: %s 연결됨 (115200 bps)", comName);
    SetWindowTextA(hStaticConnStatus, statusMsg);

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "P:%d\r\n", motor_power);
    Serial_SendCmd(cmd);
}

/* 파워(%) 입력값 읽기 및 적용 */
static void Apply_PowerFromEdit(void)
{
    char buf[32] = {0};
    GetWindowTextA(hEditPower, buf, sizeof(buf));
    int pwr = atoi(buf);
    if (pwr < 0)   pwr = 0;
    if (pwr > 100) pwr = 100;
    motor_power = pwr;

    snprintf(buf, sizeof(buf), "%d", motor_power);
    SetWindowTextA(hEditPower, buf);

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "P:%d\r\n", motor_power);
    Serial_SendCmd(cmd);

    Update_DisplayLabels();
}

/* 주기적 타이머 (50ms): 시리얼 수신 처리, 시뮬레이션 이동 및 2차원 그래프 갱신 */
static void On_TimerTick(HWND hwnd)
{
    if (hSerial != INVALID_HANDLE_VALUE)
    {
        uint8_t temp[64];
        DWORD bytesRead = 0;
        while (ReadFile(hSerial, temp, sizeof(temp), &bytesRead, NULL) && bytesRead > 0)
        {
            for (DWORD i = 0; i < bytesRead; i++)
            {
                char ch = (char)temp[i];
                if (ch == '\r' || ch == '\n')
                {
                    if (rx_line_idx > 0)
                    {
                        rx_line_buf[rx_line_idx] = '\0';
                        Parse_TelemetryLine(rx_line_buf);
                        rx_line_idx = 0;
                    }
                }
                else if (rx_line_idx < (int)sizeof(rx_line_buf) - 1)
                {
                    rx_line_buf[rx_line_idx++] = ch;
                }
            }
        }
    }
    else
    {
        if (motor_moving != 0 && motor_power > 0)
        {
            long err = target_encoder - current_encoder;
            if (labs(err) <= 2)
            {
                current_encoder = target_encoder;
                motor_moving = 0;
            }
            else
            {
                long step = (long)(labs(err) * 0.15);
                long maxStep = (motor_power * 25) / 100;
                if (maxStep < 2) maxStep = 2;
                if (step < 2) step = 2;
                if (step > maxStep) step = maxStep;

                if (err > 0) { current_encoder += step; motor_moving = 1; }
                else         { current_encoder -= step; motor_moving = 2; }
            }
            Update_DisplayLabels();
        }
    }

    Push_GraphSample();
    InvalidateRect(hwnd, &rcGraphPanel, FALSE);
}

/* 윈도우 프로시저 */
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg)
    {
    case WM_CREATE:
    {
        hFontTitle  = CreateFontA(18, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
                                  DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                  CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, "맑은 고딕");
        hFontNormal = CreateFontA(16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                                  DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                  CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, "맑은 고딕");
        hFontBigEnc = CreateFontA(23, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
                                  DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                  CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, "맑은 고딕");
        hFontSmall  = CreateFontA(14, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
                                  DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                  CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, "맑은 고딕");

        /* 0. 시리얼 포트 연결 그룹 */
        HWND hGrpCom = CreateWindowA("BUTTON", "STM32 시리얼 연결 (USART - 115200 bps)",
                                     WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                                     15, 10, 490, 75, hwnd, NULL, NULL, NULL);
        SendMessage(hGrpCom, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        HWND hLblCom = CreateWindowA("STATIC", "포트:", WS_CHILD | WS_VISIBLE,
                                     30, 38, 45, 24, hwnd, NULL, NULL, NULL);
        SendMessage(hLblCom, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        hEditCom = CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", "COM5",
                                   WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL,
                                   75, 34, 75, 28, hwnd, (HMENU)ID_EDIT_COM, NULL, NULL);
        SendMessage(hEditCom, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        hBtnConnect = CreateWindowA("BUTTON", "포트 연결",
                                    WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                                    160, 33, 95, 30, hwnd, (HMENU)ID_BTN_CONNECT, NULL, NULL);
        SendMessage(hBtnConnect, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        hStaticConnStatus = CreateWindowA("STATIC", "상태: 미연결 (시뮬레이션 모드)",
                                          WS_CHILD | WS_VISIBLE,
                                          265, 38, 230, 24, hwnd, NULL, NULL, NULL);
        SendMessage(hStaticConnStatus, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        /* 1. 모터 파워 (%) 입력 영역 */
        HWND hGrpPwr = CreateWindowA("BUTTON", "1. 모터 파워 (%) 설정",
                                     WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                                     15, 95, 490, 75, hwnd, NULL, NULL, NULL);
        SendMessage(hGrpPwr, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        HWND hLblPwr = CreateWindowA("STATIC", "모터 파워 (0 ~ 100%) :",
                                     WS_CHILD | WS_VISIBLE,
                                     30, 127, 165, 24, hwnd, NULL, NULL, NULL);
        SendMessage(hLblPwr, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        hEditPower = CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", "50",
                                     WS_CHILD | WS_VISIBLE | ES_NUMBER | ES_CENTER,
                                     200, 123, 100, 28, hwnd, (HMENU)ID_EDIT_POWER, NULL, NULL);
        SendMessage(hEditPower, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        HWND hLblPct = CreateWindowA("STATIC", "%",
                                     WS_CHILD | WS_VISIBLE,
                                     308, 127, 25, 24, hwnd, NULL, NULL, NULL);
        SendMessage(hLblPct, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        hBtnSetPower = CreateWindowA("BUTTON", "파워 적용",
                                     WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                                     345, 121, 140, 32, hwnd, (HMENU)ID_BTN_SETPOWER, NULL, NULL);
        SendMessage(hBtnSetPower, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        /* 2. 입력한 만큼 인코더 값 증가 & 현재 인코더 표시 & 리셋 버튼 */
        HWND hGrpInc = CreateWindowA("BUTTON", "2. 입력한 만큼 인코더 값 증가 및 리셋",
                                     WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                                     15, 180, 490, 195, hwnd, NULL, NULL, NULL);
        SendMessage(hGrpInc, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        HWND hLblInc = CreateWindowA("STATIC", "증가할 인코더 값 입력 :",
                                     WS_CHILD | WS_VISIBLE,
                                     30, 214, 165, 24, hwnd, NULL, NULL, NULL);
        SendMessage(hLblInc, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        hEditInc = CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", "356",
                                   WS_CHILD | WS_VISIBLE | ES_CENTER,
                                   200, 210, 130, 28, hwnd, (HMENU)ID_EDIT_INC, NULL, NULL);
        SendMessage(hEditInc, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        hBtnIncMove = CreateWindowA("BUTTON", "입력한 만큼 증가 ▲",
                                    WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                                    345, 208, 140, 32, hwnd, (HMENU)ID_BTN_INCMOVE, NULL, NULL);
        SendMessage(hBtnIncMove, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        hStaticCurEnc = CreateWindowA("STATIC", "현재 인코더 값 :  0",
                                      WS_CHILD | WS_VISIBLE,
                                      30, 255, 300, 32, hwnd, NULL, NULL, NULL);
        SendMessage(hStaticCurEnc, WM_SETFONT, (WPARAM)hFontBigEnc, TRUE);

        hBtnReset = CreateWindowA("BUTTON", "인코더 리셋 (0)",
                                  WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                                  345, 252, 140, 38, hwnd, (HMENU)ID_BTN_RESET, NULL, NULL);
        SendMessage(hBtnReset, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        hStaticTgtEnc = CreateWindowA("STATIC", "목표 인코더 값 :  0   (설정 파워: 50%)",
                                      WS_CHILD | WS_VISIBLE,
                                      30, 300, 450, 24, hwnd, NULL, NULL, NULL);
        SendMessage(hStaticTgtEnc, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        hStaticState = CreateWindowA("STATIC", "모터 상태 :  정지 ■",
                                     WS_CHILD | WS_VISIBLE,
                                     30, 332, 450, 24, hwnd, NULL, NULL, NULL);
        SendMessage(hStaticState, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        /* 3. 원하는 목표 인코더 값으로 이동 */
        HWND hGrpTgt = CreateWindowA("BUTTON", "3. 원하는 인코더 값으로 이동 (절대 위치 이동)",
                                     WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                                     15, 385, 490, 120, hwnd, NULL, NULL, NULL);
        SendMessage(hGrpTgt, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        HWND hLblTgt = CreateWindowA("STATIC", "원하는 인코더 값 입력 :",
                                     WS_CHILD | WS_VISIBLE,
                                     30, 420, 165, 24, hwnd, NULL, NULL, NULL);
        SendMessage(hLblTgt, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        hEditTarget = CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", "1425",
                                      WS_CHILD | WS_VISIBLE | ES_CENTER,
                                      200, 416, 130, 28, hwnd, (HMENU)ID_EDIT_TARGET, NULL, NULL);
        SendMessage(hEditTarget, WM_SETFONT, (WPARAM)hFontNormal, TRUE);

        hBtnTargetMove = CreateWindowA("BUTTON", "원하는 값으로 이동 ▶",
                                       WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                                       345, 414, 140, 32, hwnd, (HMENU)ID_BTN_TARGETMOVE, NULL, NULL);
        SendMessage(hBtnTargetMove, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        hBtnStop = CreateWindowA("BUTTON", "모터 즉시 정지 (STOP)",
                                 WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                                 30, 458, 455, 34, hwnd, (HMENU)ID_BTN_STOP, NULL, NULL);
        SendMessage(hBtnStop, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        /* 4. 오른쪽: 실시간 2차원 그래프 그룹박스 */
        HWND hGrpGraph = CreateWindowA("BUTTON", "4. 실시간 인코더 2차원 그래프 (현재값 & 목표값 직선)",
                                       WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                                       515, 10, 578, 495, hwnd, NULL, NULL, NULL);
        SendMessage(hGrpGraph, WM_SETFONT, (WPARAM)hFontTitle, TRUE);

        hBtnClearGraph = CreateWindowA("BUTTON", "그래프 지우기",
                                       WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                                       980, 12, 105, 26, hwnd, (HMENU)ID_BTN_CLEARGRAPH, NULL, NULL);
        SendMessage(hBtnClearGraph, WM_SETFONT, (WPARAM)hFontSmall, TRUE);

        Push_GraphSample();
        SetTimer(hwnd, ID_TIMER_UPDATE, 50, NULL);
        break;
    }

    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);
        Draw_EncoderGraph(hdc);
        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_COMMAND:
    {
        int id = LOWORD(wParam);
        switch (id)
        {
        case ID_BTN_CONNECT:
            Toggle_SerialConnection(hwnd);
            break;

        case ID_BTN_SETPOWER:
            Apply_PowerFromEdit();
            break;

        case ID_BTN_INCMOVE:
        {
            Apply_PowerFromEdit();
            char buf[64] = {0};
            GetWindowTextA(hEditInc, buf, sizeof(buf));
            long delta = strtol(buf, NULL, 10);

            target_encoder = current_encoder + delta;
            if (target_encoder > current_encoder)      motor_moving = 1;
            else if (target_encoder < current_encoder) motor_moving = 2;
            else                                       motor_moving = 0;

            char cmd[64];
            snprintf(cmd, sizeof(cmd), "I:%ld\r\n", delta);
            Serial_SendCmd(cmd);

            Update_DisplayLabels();
            break;
        }

        case ID_BTN_RESET:
        {
            current_encoder = 0;
            target_encoder  = 0;
            motor_moving    = 0;
            Serial_SendCmd("R\r\n");
            Update_DisplayLabels();
            break;
        }

        case ID_BTN_TARGETMOVE:
        {
            Apply_PowerFromEdit();
            char buf[64] = {0};
            GetWindowTextA(hEditTarget, buf, sizeof(buf));
            long tgt = strtol(buf, NULL, 10);

            target_encoder = tgt;
            if (target_encoder > current_encoder)      motor_moving = 1;
            else if (target_encoder < current_encoder) motor_moving = 2;
            else                                       motor_moving = 0;

            char cmd[64];
            snprintf(cmd, sizeof(cmd), "T:%ld\r\n", tgt);
            Serial_SendCmd(cmd);

            Update_DisplayLabels();
            break;
        }

        case ID_BTN_STOP:
        {
            motor_moving = 0;
            target_encoder = current_encoder;
            Serial_SendCmd("S\r\n");
            Update_DisplayLabels();
            break;
        }

        case ID_BTN_CLEARGRAPH:
        {
            graph_count = 0;
            Push_GraphSample();
            InvalidateRect(hwnd, &rcGraphPanel, FALSE);
            break;
        }
        }
        break;
    }

    case WM_TIMER:
        if (wParam == ID_TIMER_UPDATE)
        {
            On_TimerTick(hwnd);
        }
        break;

    case WM_DESTROY:
        KillTimer(hwnd, ID_TIMER_UPDATE);
        if (hSerial != INVALID_HANDLE_VALUE)
        {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
        }
        DeleteObject(hFontTitle);
        DeleteObject(hFontNormal);
        DeleteObject(hFontBigEnc);
        DeleteObject(hFontSmall);
        PostQuitMessage(0);
        break;

    default:
        return DefWindowProcA(hwnd, msg, wParam, lParam);
    }
    return 0;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    (void)hPrevInstance;
    (void)lpCmdLine;

    WNDCLASSEXA wc = {0};
    wc.cbSize        = sizeof(WNDCLASSEXA);
    wc.lpfnWndProc   = WndProc;
    wc.hInstance     = hInstance;
    wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    wc.lpszClassName = "MotorControlGUI";

    if (!RegisterClassExA(&wc))
    {
        return 1;
    }

    HWND hwnd = CreateWindowExA(
        0,
        "MotorControlGUI",
        "STM32 모터 파워(%) 및 인코더 위치 제어기 (2차원 실시간 그래프)",
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
        CW_USEDEFAULT, CW_USEDEFAULT, 1120, 555,
        NULL, NULL, hInstance, NULL);

    if (!hwnd)
    {
        return 1;
    }

    ShowWindow(hwnd, nCmdShow);
    UpdateWindow(hwnd);

    MSG msg;
    while (GetMessageA(&msg, NULL, 0, 0) > 0)
    {
        if (!IsDialogMessageA(hwnd, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }
    }

    return (int)msg.wParam;
}
