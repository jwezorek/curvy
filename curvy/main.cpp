#include <windows.h>
#include <chrono>
#include "curvy.h"
#include "colors.h"

namespace chrono = std::chrono;

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT HandleWmSize(curvy::state& state, WPARAM wParam, LPARAM lParam);
LRESULT HandleWmPaint(HWND hwnd, curvy::state& state, WPARAM wParam, LPARAM lParam);

static curvy::state g_curvy(0, 40);

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{

    MSG msg = { 0 };
    WNDCLASS wc = { 0 };
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hbrBackground = (HBRUSH)(COLOR_BACKGROUND);
    wc.lpszClassName = L"curvy";
    if (!RegisterClass(&wc))
        return 1;

    int sz = 800;
    RECT r = { 20, 20, sz, sz };
    AdjustWindowRect( &r, WS_BORDER | WS_SYSMENU | WS_VISIBLE, TRUE);

    HWND hwnd = CreateWindow(wc.lpszClassName,
        L"curvy",
        WS_BORDER | WS_SYSMENU | WS_VISIBLE,
        r.left, r.top, r.right - r.left, r.bottom - r.top + 3, 0, 0, hInstance, NULL);

    auto last_time = chrono::high_resolution_clock::now();
    while (true) {
        if (PeekMessage(&msg, 0, 0, 0, PM_REMOVE)) {
            if (WM_QUIT == msg.message) {
                break;
            } else {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
        } else {
            auto current_time = chrono::high_resolution_clock::now();
            auto elapsed = chrono::duration_cast<chrono::duration<double>>(current_time - last_time);
            g_curvy.update( elapsed.count() );
            InvalidateRect(hwnd, NULL, FALSE);
            last_time = current_time;
        }
    }

    return 0;
}

LRESULT HandleWmSize(curvy::state& state, WPARAM wParam, LPARAM lParam) {
    auto wd = LOWORD(lParam);
    state.set_dimensions(wd);
    return 0;
}

LRESULT HandleWmPaint(HWND hwnd, curvy::state& state, WPARAM wParam, LPARAM lParam) {
    int sz = state.get_size();
    auto buffer = state.get_bitmap();
    HDC hdc_scr = GetDC(NULL);
    HDC hdc_buff = CreateCompatibleDC(hdc_scr);

    auto hbm_old = SelectObject(hdc_buff, buffer);

    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(hwnd, &ps);
    BitBlt(hdc, 0, 0, sz, sz, hdc_buff, 0, 0, SRCCOPY); 
    EndPaint(hwnd, &ps);
    
    SelectObject(hdc_buff, hbm_old);
    DeleteDC(hdc_buff);
    ReleaseDC(NULL, hdc_scr);
    return 0;
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{

    switch (message) {
        case WM_CREATE:
            g_curvy.insert( {
                curvy::circle_rotation_state{  0, 0,  0, 12.0, 4 },
                colors::Red
            });

            g_curvy.insert({
                curvy::circle_rotation_state{ curvy::pi(),  0, 0, 12.0, 0 },
                colors::Yellow
            });

            break;
        case WM_CLOSE:
            PostQuitMessage(0);
            break;
        case WM_SIZE:
            return HandleWmSize(g_curvy, wParam, lParam);
        case WM_PAINT:
            return HandleWmPaint(hwnd, g_curvy, wParam, lParam);
        default:
            return DefWindowProc(hwnd, message, wParam, lParam);
    }

    return 0;

}