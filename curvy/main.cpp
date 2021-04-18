#include <windows.h>
#include <windowsx.h>
#include <chrono>
#include <memory>
#include "state.h"
#include "curvy_world_simulation.h"
#include "curvy_vector_viewer.h"
#include "util.h"
#include "circle.h"
#include "colors.h"
#include <gdiplus.h>
#pragma comment (lib,"Gdiplus.lib")

namespace chrono = std::chrono;

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT HandleWmSize(curvy::state& state, WPARAM wParam, LPARAM lParam);
LRESULT HandleWmPaint(HWND hwnd, curvy::state& state, WPARAM wParam, LPARAM lParam);
LRESULT HandleWmLButtonMsg(HWND hwnd, curvy::state& state, WPARAM wParam, LPARAM lParam, bool button_down);
LRESULT HandleWmMouseMove(HWND hwnd, curvy::state& state, WPARAM wParam, LPARAM lParam);

std::unique_ptr<curvy::state> g_simulation = std::make_unique<curvy::curvy_world_simulation>(0, 40);
std::unique_ptr<curvy::state> g_viewer = std::make_unique<curvy::curvy_vector_viewer>(0, 20);

curvy::state* g_state = g_viewer.get();

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    gdi::GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR           gdiplusToken;

    // Initialize GDI+.
    gdi::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

    MSG msg = { 0 };
    WNDCLASS wc = { 0 };
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hbrBackground = (HBRUSH)(COLOR_BACKGROUND);
    wc.lpszClassName = L"curvy";
    if (!RegisterClass(&wc))
        return 1;

    //int sz = 1400;
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
            g_state->update( elapsed.count() );
            InvalidateRect(hwnd, NULL, FALSE);
            last_time = current_time;
        }
    }

    return 0;
}

LRESULT HandleWmSize(curvy::state& state, WPARAM wParam, LPARAM lParam) {
    auto wd = LOWORD(lParam);
    state.set_pixel_dimensions(wd);
    return 0;
}

LRESULT HandleWmPaint(HWND hwnd, curvy::state& state, WPARAM wParam, LPARAM lParam) {

    PAINTSTRUCT ps;
    auto hdc = BeginPaint(hwnd, &ps);
    auto g = gdi::Graphics(hdc);
    g.DrawImage(state.get_bitmap(), 0, 0);
    EndPaint(hwnd, &ps);
    return 0;
}

LRESULT HandleWmLButtonMsg(HWND hwnd, curvy::state& state, WPARAM wParam, LPARAM lParam, bool button_down) {
    auto x = GET_X_LPARAM(lParam);
    auto y = GET_Y_LPARAM(lParam);
    if (state.handle_mouse_click({ x, y }, button_down)) {
        InvalidateRect(hwnd, NULL, FALSE);
    }
    return 0;
}

int get_client_width(HWND hwnd) {
    RECT r;
    GetClientRect(hwnd, &r);
    return r.right - r.left;
}

LRESULT HandleKeyboardMsg(HWND hwnd, curvy::state& state, WPARAM wParam, LPARAM lParam, bool keydown) {
    if (wParam == VK_RETURN && keydown) {
        g_state = (g_state == g_simulation.get()) ? g_viewer.get() : g_simulation.get();
        g_state->set_pixel_dimensions( get_client_width(hwnd) );
        g_state->initialize();
        state.update();
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;
    }
    if (state.handle_key_press(static_cast<unsigned int>(wParam), keydown)) {
        state.update();
        InvalidateRect(hwnd, NULL, FALSE);
    }
    return 0;
}


LRESULT HandleWmMouseMove(HWND hwnd, curvy::state& state, WPARAM wParam, LPARAM lParam) {
    auto x = GET_X_LPARAM(lParam);
    auto y = GET_Y_LPARAM(lParam);
    if (state.handle_mouse_move({ x, y })) {
        InvalidateRect(hwnd, NULL, FALSE);
    }
    return 0;
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message) {
        case WM_CREATE:
            g_state->initialize();
            break;
        case WM_CLOSE:
            PostQuitMessage(0);
            break;
        case WM_KEYDOWN:
            return HandleKeyboardMsg(hwnd, *g_state, wParam, lParam, true);
        case WM_KEYUP:
            return HandleKeyboardMsg(hwnd, *g_state, wParam, lParam, false);
        case WM_LBUTTONDOWN:
            return HandleWmLButtonMsg(hwnd, *g_state, wParam, lParam, true);
        case WM_LBUTTONUP:
            return HandleWmLButtonMsg(hwnd, *g_state, wParam, lParam, false);
        case WM_MOUSEMOVE:
            return HandleWmMouseMove(hwnd, *g_state, wParam, lParam);
        case WM_SIZE:
            return HandleWmSize(*g_state, wParam, lParam);
        case WM_PAINT:
            return HandleWmPaint(hwnd, *g_state, wParam, lParam);
        default:
            return DefWindowProc(hwnd, message, wParam, lParam);
    }

    return 0;

}