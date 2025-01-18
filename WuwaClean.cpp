#include <Windows.h>
#include <chrono>
#include <filesystem>
#include <future>
#include <limits>
#include <string>
#include <vector>

#include <BS_thread_pool.hpp>
#include <opencv2/opencv.hpp>

#include "pch.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

const double LOWEST_TRUST_RATE = 0.7;
const int WAIT_TIME_BETWEEN_ITEMS = 170;

HWND hwnd = NULL;

const string SuitName[] = {
    "celestial_light",      "empyrean_anthem",   "eternal_radiance",
    "freezing_forst",       "frosty_resolve",    "havoc_eclipse",
    "lingering_tunes",      "midnight_veil",     "molten_rift",
    "moonlit_clouds",       "rejuvenating_glow", "sierra_gale",
    "tidebreaking_courage", "void_thunder"};

enum class Suit : int {
  celestial_light,
  empyrean_anthem,
  eternal_radiance,
  freezing_forst,
  frosty_resolve,
  havoc_eclipse,
  lingering_tunes,
  midnight_veil,
  molten_rift,
  moonlit_clouds,
  rejuvenating_glow,
  sierra_gale,
  tidebreaking_courage,
  void_thunder
};

const unordered_map<string, Suit> SuitMap = {
    {"celestial_light", Suit::celestial_light},
    {"empyrean_anthem", Suit::empyrean_anthem},
    {"eternal_radiance", Suit::eternal_radiance},
    {"freezing_forst", Suit::freezing_forst},
    {"frosty_resolve", Suit::frosty_resolve},
    {"havoc_eclipse", Suit::havoc_eclipse},
    {"lingering_tunes", Suit::lingering_tunes},
    {"midnight_veil", Suit::midnight_veil},
    {"molten_rift", Suit::molten_rift},
    {"moonlit_clouds", Suit::moonlit_clouds},
    {"rejuvenating_glow", Suit::rejuvenating_glow},
    {"sierra_gale", Suit::sierra_gale},
    {"tidebreaking_courage", Suit::tidebreaking_courage},
    {"void_thunder", Suit::void_thunder}};

const string AttrName[] = {
    "aero",   "atk",    "crit_dmg", "crit_rate", "def", "electro", "energy",
    "fusion", "glacio", "havoc",    "healing",   "hp",  "spectro"};

enum class Attr : int {
  aero,
  atk,
  crit_dmg,
  crit_rate,
  def,
  electro,
  energy,
  fusion,
  glacio,
  havoc,
  healing,
  hp,
  spectro
};

const unordered_map<string, Attr> AttrMap = {
    {"aero", Attr::aero},         {"atk", Attr::atk},
    {"crit_dmg", Attr::crit_dmg}, {"crit_rate", Attr::crit_rate},
    {"def", Attr::def},           {"electro", Attr::electro},
    {"energy", Attr::energy},     {"fusion", Attr::fusion},
    {"glacio", Attr::glacio},     {"havoc", Attr::havoc},
    {"healing", Attr::healing},   {"hp", Attr::hp},
    {"spectro", Attr::spectro}};

RECT getWindowRect() {
  RECT rect;
  if (!GetWindowRect(hwnd, &rect)) {
    throw runtime_error("无法获取窗口矩形");
  }
  return rect;
}

RECT getClientRect() {
  RECT rect;
  if (!GetClientRect(hwnd, &rect)) {
    throw runtime_error("无法获取窗口客户区");
  }
  return rect;
}

int getTitleBarHeight(RECT windowRect, RECT clientRect) {
  POINT topLeft = {clientRect.left, clientRect.top};
  ClientToScreen(hwnd, &topLeft);
  return topLeft.y - windowRect.top;
}

unique_ptr<cv::Mat> cropWindowFromScreen() {
  RECT windowRect = getWindowRect();
  RECT clientRect = getClientRect();
  int titleBarHeight = getTitleBarHeight(windowRect, clientRect);
  int width = clientRect.right - clientRect.left;
  int height = clientRect.bottom - clientRect.top;
  return grab(0, titleBarHeight, width, height);
}

void sendMouseInput(LONG x, LONG y, DWORD flags) {
  SetCursorPos(x, y);

  INPUT input = {0};
  input.type = INPUT_MOUSE;
  input.mi.dwFlags = flags;
  SendInput(1, &input, sizeof(INPUT));
}

void mouseDrag(LONG startX, LONG startY, LONG endX, LONG endY) {
  // 将屏幕坐标转换为绝对坐标
  LONG screenWidth = GetSystemMetrics(SM_CXSCREEN);
  LONG screenHeight = GetSystemMetrics(SM_CYSCREEN);
  LONG absStartX = startX * 65535 / screenWidth;
  LONG absStartY = startY * 65535 / screenHeight;
  LONG absEndX = endX * 65535 / screenWidth;
  LONG absEndY = endY * 65535 / screenHeight;

  // 移动到起始位置
  INPUT input = {0};
  input.type = INPUT_MOUSE;
  input.mi.dx = absStartX;
  input.mi.dy = absStartY;
  input.mi.dwFlags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE;
  SendInput(1, &input, sizeof(INPUT));

  // 按下鼠标左键
  input.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
  SendInput(1, &input, sizeof(INPUT));

  // 模拟拖动过程
  LONG steps = 50; // 调整步数以控制拖动的平滑度
  for (LONG i = 0; i <= steps; ++i) {
    LONG currentX = absStartX + (absEndX - absStartX) * i / steps;
    LONG currentY = absStartY + (absEndY - absStartY) * i / steps;

    input.mi.dx = currentX;
    input.mi.dy = currentY;
    input.mi.dwFlags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE;
    SendInput(1, &input, sizeof(INPUT));
    Sleep(10); // 调整间隔以模拟自然拖动
  }

  Sleep(500); // 等待一段时间以模拟拖动后的停顿
  // 松开鼠标左键
  input.mi.dwFlags = MOUSEEVENTF_LEFTUP;
  SendInput(1, &input, sizeof(INPUT));
}

void ClickWindow(int x, int y) {
  SetCursorPos(x, y);

  sendMouseInput(x, y, MOUSEEVENTF_LEFTDOWN);
  sendMouseInput(x, y, MOUSEEVENTF_LEFTUP);
}

void pressDiscard() {
  INPUT input = {0};
  input.type = INPUT_KEYBOARD;
  input.ki.wVk = 0x5A;

  SendInput(1, &input, sizeof(INPUT));

  input.ki.dwFlags = KEYEVENTF_KEYUP;
  SendInput(1, &input, sizeof(INPUT));
}

struct templateMatchResult {
  double val;
  string name;
  Point loc;
};

struct itemMatchResult {
  const string attr;
  const string suit;
};

vector<pair<string, pair<Mat, Mat>>> loadTemplates(const string &path) {
  vector<pair<string, pair<Mat, Mat>>> templates;
  namespace fs = filesystem;

  for (const auto &entry : fs::directory_iterator(path)) {
    if (entry.path().extension() == ".png") {
      Mat templ = imread(entry.path().string(), IMREAD_UNCHANGED);
      if (!templ.empty()) {
        // Split the channels
        vector<Mat> channels;
        split(templ, channels);

        // Create a new image without the alpha channel
        Mat bgr;
        merge(vector<Mat>{channels[0], channels[1], channels[2]}, bgr);

        // Create a mask from the alpha channel
        Mat alpha = channels[3];
        Mat mask;
        threshold(alpha, mask, 0, 255, THRESH_BINARY);

        // Apply the mask to the BGR image
        Mat result;
        bgr.copyTo(result, mask);

        // Add the result to the templates
        // imshow(entry.path().filename().string(), result);
        templates.push_back(
            {entry.path().filename().stem().string(), {result, mask}});
      }
    }
  }
  return templates;
}

bool containsColor(const cv::Mat &image, const cv::Scalar &lower,
                   const cv::Scalar &upper) {
  cv::Mat hsvImage;
  cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsvImage, lower, upper, mask);

  return cv::countNonZero(mask) > 0;
}

BS::thread_pool pool(16);
optional<itemMatchResult>
processTemplates(const Mat &bgrInfoBox,
                 const vector<pair<string, pair<Mat, Mat>>> &attr_templates,
                 const vector<pair<string, pair<Mat, Mat>>> &suit_templates) {
  BS::multi_future<templateMatchResult> attrMatchFuture = pool.submit_blocks(
      0ull, attr_templates.size(),
      [bgrInfoBox, attr_templates](const size_t start, const size_t end) {
        Mat result;
        matchTemplate(bgrInfoBox, attr_templates[start].second.first, result,
                      TM_CCOEFF_NORMED);

        double minVal, maxVal;
        Point minLoc, maxLoc;
        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

        Point matchLoc = Point(maxLoc.x, maxLoc.y);
        return templateMatchResult{maxVal, attr_templates[start].first,
                                   matchLoc};
      },
      attr_templates.size());
  BS::multi_future<templateMatchResult> suitMatchFuture = pool.submit_blocks(
      0ull, suit_templates.size(),
      [bgrInfoBox, suit_templates](const size_t start, const size_t end) {
        Mat result;
        matchTemplate(bgrInfoBox, suit_templates[start].second.first, result,
                      TM_CCOEFF_NORMED, suit_templates[start].second.second);

        double minVal, maxVal;
        Point minLoc, maxLoc;
        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

        Point matchLoc = Point(maxLoc.x, maxLoc.y);
        return templateMatchResult{maxVal, suit_templates[start].first,
                                   matchLoc};
      },
      suit_templates.size());
  templateMatchResult mostTrustedAttr(
      0, "", Point(numeric_limits<int>::max(), numeric_limits<int>::max()));
  for (std::future<templateMatchResult> &fut : attrMatchFuture) {
    templateMatchResult result = fut.get();
    // cout << "looked for " << setw(15) << result.name << setw(10) <<
    // result.val << endl;
    if (result.val > LOWEST_TRUST_RATE &&
        result.loc.y < mostTrustedAttr.loc.y) {
      mostTrustedAttr = result;
    }
  }
  templateMatchResult mostTrustedSuit(
      0, "", Point(numeric_limits<int>::max(), numeric_limits<int>::max()));
  for (std::future<templateMatchResult> &fut : suitMatchFuture) {
    templateMatchResult result = fut.get();
    // cout << "looked for " << setw(15) << result.name << setw(10) <<
    // result.val << endl;
    if (result.val > LOWEST_TRUST_RATE &&
        result.loc.y < mostTrustedSuit.loc.y) {
      mostTrustedSuit = result;
    }
  }
  if (mostTrustedAttr.val > LOWEST_TRUST_RATE &&
      mostTrustedSuit.val > LOWEST_TRUST_RATE) {
    return itemMatchResult{mostTrustedAttr.name, mostTrustedSuit.name};
  }
  return nullopt;
}

int main() {
  SetProcessDPIAware();
  // 标题就是有两个空格
  vector<wstring> titles = {L"鸣潮  ", L"Wuthering Waves  "};
  for (const auto &title : titles) {
    hwnd = FindWindowW(L"UnrealWindow", title.c_str());
    if (hwnd != NULL) {
      break;
    }
  }
  if (hwnd == NULL) {
    MessageBoxW(NULL, L"未找到窗口", L"错误", MB_OK | MB_ICONERROR);
    return 1;
  }
  auto attr_templates = loadTemplates("assets/attr_icons");
  auto suit_templates = loadTemplates("assets/suit_icons");

  init_dxgi(hwnd);
  RECT windowRect = getWindowRect();
  RECT clientRect = getClientRect();
  int clientWidth = clientRect.right - clientRect.left;
  int clientHeight = clientRect.bottom - clientRect.top;
  double scale = min(clientWidth / 2560.0, clientHeight / 1440.0);

  int loop = 80;
  while (loop--) {
    auto start = high_resolution_clock::now();
    unique_ptr<cv::Mat> image = cropWindowFromScreen();

    Mat processedImage;
    Mat gray;
    cvtColor(*image, gray, COLOR_BGRA2GRAY);
    Canny(gray, processedImage, 100, 200);
    dilate(processedImage, processedImage, Mat(), Point(-1, -1), 2);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(processedImage, contours, hierarchy, RETR_EXTERNAL,
                 CHAIN_APPROX_SIMPLE);

    vector<Rect> rectangles;
    Rect infoBox;
    for (const auto &contour : contours) {
      Rect rect = boundingRect(contour);
      if (rect.area() > 200.0 * 200.0 * scale * scale &&
          rect.area() < 300.0 * 300.0 * scale * scale &&
          abs(rect.width - rect.height) < 100) {
        rectangles.push_back(rect);
        // rectangle(*image, rect, Scalar(0, 255, 0), 2);
      } else if (rect.area() > 500.0 * 500.0 * scale * scale) {
        infoBox = rect;
        // rectangle(*image, infoBox, Scalar(255, 0, 0), 2);
      }
    }
    // imshow("processedImage", processedImage);
    // imshow("image", *image);
    sort(rectangles.begin(), rectangles.end(),
         [](const Rect &a, const Rect &b) {
           if (abs(a.y - b.y) < 20) {
             return a.x < b.x;
           }
           return a.y < b.y;
         });
    Rect topLeft = rectangles.front(), bottomRight = rectangles.back();
    Rect infoBoxAction = infoBox;
    infoBoxAction.x += infoBoxAction.width * 2 / 3;
    infoBoxAction.width /= 3;
    infoBoxAction.y += infoBoxAction.height / 5;
    infoBoxAction.height = infoBoxAction.height * 3 / 5;
    infoBox.width /= 3;
    infoBox.y += infoBox.height / 5;
    infoBox.height = infoBox.height * 3 / 5;
    RegisterHotKey(NULL, 1, MOD_CONTROL, VK_F5);
    MSG msg;
    for (auto itemRec : rectangles) {
      if (GetMessage(&msg, NULL, 0, 0)) {
        if (msg.message == WM_HOTKEY) {
          if (msg.wParam == 1) {
            return 0;
          }
        }
        TranslateMessage(&msg);
        DispatchMessage(&msg);
      }
      SetForegroundWindow(hwnd);
      ClickWindow(windowRect.left + itemRec.x + itemRec.width / 2,
                  windowRect.top + itemRec.y + itemRec.height / 2);
      Sleep(WAIT_TIME_BETWEEN_ITEMS);
      unique_ptr<cv::Mat> infoBoxActionMat =
          grab(infoBoxAction.x, infoBoxAction.y, infoBoxAction.width,
               infoBoxAction.height);

      const cv::Scalar lower(0, 55, 100);
      const cv::Scalar upper(10, 255, 255);
      if (containsColor(*infoBoxActionMat, lower, upper)) {
        cout << "already discard" << endl;
        continue;
      }
      unique_ptr<cv::Mat> infoBoxMat =
          grab(infoBox.x, infoBox.y, infoBox.width, infoBox.height);
      Mat bgrInfoBox;
      cvtColor(*infoBoxMat, bgrInfoBox, COLOR_BGRA2BGR);
      resize(bgrInfoBox, bgrInfoBox, Size(), 1.0 / scale, 1.0 / scale,
             INTER_LINEAR);
      // imshow("infoBox", bgrInfoBox);
      const auto &result =
          processTemplates(bgrInfoBox, attr_templates, suit_templates);
      if (!result.has_value()) {
        imshow("infobox", bgrInfoBox);
        cout << "no trusted attr or suit found" << endl;
        return 1;
      } else {
        cout << "most trusted suit: " << setw(20) << result->suit
             << " attr:" << setw(20) << result->attr << endl;
        bool shouldDiscard = false;
        const Suit suit = SuitMap.at(result->suit);
        const Attr attr = AttrMap.at(result->attr);
        auto isCommonAttr = [](Attr attr) {
          return attr == Attr::atk || attr == Attr::crit_rate ||
                 attr == Attr::crit_dmg || attr == Attr::energy;
        };
        switch (suit) {
        case ::Suit::celestial_light:
        case ::Suit::eternal_radiance:
          shouldDiscard = !isCommonAttr(attr) && attr != Attr::spectro;
          break;
        case ::Suit::freezing_forst:
        case ::Suit::frosty_resolve:
          shouldDiscard = !isCommonAttr(attr) && attr != Attr::glacio;
          break;
        case ::Suit::havoc_eclipse:
        case ::Suit::midnight_veil:
          shouldDiscard = !isCommonAttr(attr) && attr != Attr::havoc;
          break;
        case ::Suit::molten_rift:
          shouldDiscard = !isCommonAttr(attr) && attr != Attr::fusion;
          break;
        case ::Suit::rejuvenating_glow:
          shouldDiscard = (!isCommonAttr(attr) && attr != Attr::healing) ||
                          attr == Attr::hp;
          break;
        case ::Suit::sierra_gale:
          shouldDiscard = !isCommonAttr(attr) && attr != Attr::aero;
          break;
        case ::Suit::void_thunder:
          shouldDiscard = !isCommonAttr(attr) && attr != Attr::electro;
          break;
        case ::Suit::lingering_tunes:
        case ::Suit::empyrean_anthem:
        case ::Suit::moonlit_clouds:
        case ::Suit::tidebreaking_courage:
          shouldDiscard = !isCommonAttr(attr);
          break;
        default:
          break;
        }
        if (shouldDiscard) {
          cout << "discard current item" << endl;
          pressDiscard();
          Sleep(WAIT_TIME_BETWEEN_ITEMS);
        }
      }
    }
    auto duration =
        duration_cast<milliseconds>(high_resolution_clock::now() - start);
    cout << "duration: " << duration.count() << "ms" << endl;
    cout << "time / item: " << duration.count() / rectangles.size() << "ms"
         << endl;
    mouseDrag((topLeft.x + bottomRight.x) / 2,
              bottomRight.y + bottomRight.height / 2,
              (topLeft.x + bottomRight.x) / 2, topLeft.y + topLeft.height / 3);
    Sleep(WAIT_TIME_BETWEEN_ITEMS * 15);
  }
  destroy();
  waitKey(0);
  UnregisterHotKey(NULL, 1);
  return 0;
}
