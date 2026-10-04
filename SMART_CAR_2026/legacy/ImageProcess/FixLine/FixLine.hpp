#pragma once
#include "headfile.hpp"

using namespace cv;
using namespace std;

namespace ImageProcess
{
    /*
     * 赛道线修补类。
     *
     * 当左右边线出现断裂、出界、元素干扰时，
     * 该类负责识别异常并对线段进行补线、清线和中线重建。
     */
    enum FixLineState
    {
        FIX_LINE_STATE_NONE = 0,
        FIX_LINE_STATE_CONTINUE,
        FIX_LINE_STATE_BREAK,
        FIX_LINE_STATE_DROP,
        FIX_LINE_OK
    };

    struct FixInfo
    {
        Point start;
        Point end;
    };

    enum LineType
    {
        LEFT,
        CENTER,
        RIGHT
    };

    class FixLine
    {
    private:
        vector<pair<LineType, FixInfo *>> fix_infos;

        Point left_start_pos;
        Point right_start_pos;
        Point left_end_pos;
        Point right_end_pos;

        vector<Point> left_line_contours;
        vector<Point> right_line_contours;

        int *left_line = nullptr;
        int *center_line = nullptr;
        int *right_line = nullptr;

        /**
         * @brief 计算 calculateAngle 结果
         *
         * @details
         * 按照 ImageProcess / FixLine 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
         *
         * @param x1 x1 参数，参与 calculateAngle 的业务处理或状态更新。
         * @param y1 y1 参数，参与 calculateAngle 的业务处理或状态更新。
         * @param x2 x2 参数，参与 calculateAngle 的业务处理或状态更新。
         * @param y2 y2 参数，参与 calculateAngle 的业务处理或状态更新。
         *
         * @return 返回浮点数结果，通常表示速度、角度、误差或比例计算值。
         */
        double calculateAngle(int x1, int y1, int x2, int y2);

        /**
         * @brief 查找 findAllLineBreakFromBottomSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FixLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param line 线段或线数组数据。
         * @param threshold 阈值参数，用于判断误差区间或算法分段。
         * @param sampleNumber sampleNumber 参数，参与 findAllLineBreakFromBottomSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        vector<pair<Point, Point>> findAllLineBreakFromBottomSimple(int *line, int threshold, int sampleNumber);
        /**
         * @brief 查找 findAllContoursCornerSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FixLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param icontours icontours 参数，参与 findAllContoursCornerSimple 的业务处理或状态更新。
         * @param threshold 阈值参数，用于判断误差区间或算法分段。
         * @param sampleNumber sampleNumber 参数，参与 findAllContoursCornerSimple 的业务处理或状态更新。
         * @param startHeight startHeight 参数，参与 findAllContoursCornerSimple 的业务处理或状态更新。
         * @param endHeight endHeight 参数，参与 findAllContoursCornerSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        vector<pair<Point, int>> findAllContoursCornerSimple(const vector<Point> &icontours, int threshold, int sampleNumber, int startHeight = -1, int endHeight = -1);
        /**
         * @brief 查找 findALLLineCornerSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FixLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param line 线段或线数组数据。
         * @param threshold 阈值参数，用于判断误差区间或算法分段。
         * @param sampleNumber sampleNumber 参数，参与 findALLLineCornerSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        vector<Point> findALLLineCornerSimple(int *line, int threshold, int sampleNumber);
        // No Test
        /**
         * @brief 查找 findLineBreakFromBottomSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FixLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param line 线段或线数组数据。
         * @param threshold 阈值参数，用于判断误差区间或算法分段。
         * @param startPos startPos 参数，参与 findLineBreakFromBottomSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        pair<Point, Point> findLineBreakFromBottomSimple(int *line, int threshold, int startPos);
        /**
         * @brief 查找 findLineBreakFromTopSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FixLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param line 线段或线数组数据。
         * @param threshold 阈值参数，用于判断误差区间或算法分段。
         * @param startPos startPos 参数，参与 findLineBreakFromTopSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        pair<Point, Point> findLineBreakFromTopSimple(int *line, int threshold, int startPos);
        /**
         * @brief 查找 findContoursCornerFromStartSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FixLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param icontours icontours 参数，参与 findContoursCornerFromStartSimple 的业务处理或状态更新。
         * @param threshold 阈值参数，用于判断误差区间或算法分段。
         * @param sampleNumber sampleNumber 参数，参与 findContoursCornerFromStartSimple 的业务处理或状态更新。
         * @param startHeight startHeight 参数，参与 findContoursCornerFromStartSimple 的业务处理或状态更新。
         * @param endHeight endHeight 参数，参与 findContoursCornerFromStartSimple 的业务处理或状态更新。
         * @param startPos startPos 参数，参与 findContoursCornerFromStartSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        pair<Point, int> findContoursCornerFromStartSimple(const vector<Point> &icontours, int threshold, int sampleNumber, int startHeight = -1, int endHeight = -1, int startPos = 0);
        /**
         * @brief 查找 findContoursCornerFromEndSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FixLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param icontours icontours 参数，参与 findContoursCornerFromEndSimple 的业务处理或状态更新。
         * @param threshold 阈值参数，用于判断误差区间或算法分段。
         * @param sampleNumber sampleNumber 参数，参与 findContoursCornerFromEndSimple 的业务处理或状态更新。
         * @param startHeight startHeight 参数，参与 findContoursCornerFromEndSimple 的业务处理或状态更新。
         * @param endHeight endHeight 参数，参与 findContoursCornerFromEndSimple 的业务处理或状态更新。
         * @param startPos startPos 参数，参与 findContoursCornerFromEndSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        pair<Point, int> findContoursCornerFromEndSimple(const vector<Point> &icontours, int threshold, int sampleNumber, int startHeight = -1, int endHeight = -1, int startPos = 0);
        /**
         * @brief 查找 findOutAreaFromButtomSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FixLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param line 线段或线数组数据。
         * @param startPos startPos 参数，参与 findOutAreaFromButtomSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        Point findOutAreaFromButtomSimple(int *line, int startPos);
        /**
         * @brief 查找 findBreakPointInOutAreaFromBottomSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FixLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param line 线段或线数组数据。
         * @param threshold 阈值参数，用于判断误差区间或算法分段。
         * @param startPos startPos 参数，参与 findBreakPointInOutAreaFromBottomSimple 的业务处理或状态更新。
         * @param endPos endPos 参数，参与 findBreakPointInOutAreaFromBottomSimple 的业务处理或状态更新。
         * @param searchLevel searchLevel 参数，参与 findBreakPointInOutAreaFromBottomSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        Point findBreakPointInOutAreaFromBottomSimple(int *line, int threshold, int startPos, int endPos, int searchLevel = 2);
        /**
         * @brief 查找 findInOutAreaPointFromBottomSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FixLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param line 线段或线数组数据。
         * @param startPos startPos 参数，参与 findInOutAreaPointFromBottomSimple 的业务处理或状态更新。
         * @param endPos endPos 参数，参与 findInOutAreaPointFromBottomSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        Point findInOutAreaPointFromBottomSimple(int *line, int startPos, int endPos);

        /**
         * @brief countJumpBlockInRowFromBinFrameSimple 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FixLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param binImage binImage 参数，参与 countJumpBlockInRowFromBinFrameSimple 的业务处理或状态更新。
         * @param rowIndex rowIndex 参数，参与 countJumpBlockInRowFromBinFrameSimple 的业务处理或状态更新。
         * @param startPos startPos 参数，参与 countJumpBlockInRowFromBinFrameSimple 的业务处理或状态更新。
         * @param endPos endPos 参数，参与 countJumpBlockInRowFromBinFrameSimple 的业务处理或状态更新。
         * @param whiteThreshold 阈值参数，用于判断误差区间或算法分段。
         * @param blackThreshold 阈值参数，用于判断误差区间或算法分段。
         * @param whiteBlockValue 数值参数，用于提交当前值、目标值、限幅值或配置值。
         * @param blackBlockValue 数值参数，用于提交当前值、目标值、限幅值或配置值。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int countJumpBlockInRowFromBinFrameSimple(Mat binImage, int rowIndex, int startPos, int endPos, int whiteThreshold, int blackThreshold, unsigned char whiteBlockValue, unsigned char blackBlockValue);

        /**
         * @brief countOutAreaPointSimple 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FixLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param line 线段或线数组数据。
         * @param startPos startPos 参数，参与 countOutAreaPointSimple 的业务处理或状态更新。
         * @param endPos endPos 参数，参与 countOutAreaPointSimple 的业务处理或状态更新。
         * @param cut cut 参数，参与 countOutAreaPointSimple 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int countOutAreaPointSimple(int *line, int startPos, int endPos, int cut = -20);

        /**
         * @brief 检查 checkPointIsOutAreaSimple 状态
         *
         * @details
         * 检测 ImageProcess / FixLine 模块当前资源或设备状态是否可用，并将检查结果返回给调用方。
         *
         * @param pointX pointX 参数，参与 checkPointIsOutAreaSimple 的业务处理或状态更新。
         * @param cut cut 参数，参与 checkPointIsOutAreaSimple 的业务处理或状态更新。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool checkPointIsOutAreaSimple(int pointX, int cut = -20);

        /**
         * @brief 释放 clearLineFormTopSimple 相关资源
         *
         * @details
         * 关闭、清理或释放 ImageProcess / FixLine 模块占用的动态内存、文件描述符、缓存节点或硬件资源。
         *
         * @param line 线段或线数组数据。
         * @param height 目标图像高度或裁剪高度。
         */
        void clearLineFormTopSimple(int *line, int height);

        /**
         * @brief drawLineSimple 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FixLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param line 线段或线数组数据。
         * @param fixInfos fixInfos 参数，参与 drawLineSimple 的业务处理或状态更新。
         */
        void drawLineSimple(int *line, FixInfo fixInfos);

        int element_type = -1;

#ifdef FIXLINE_INCLUDE_ELEMENTS_ZEBRA

        Mat zebra_image;
        /**
         * @brief void 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FixLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @return 返回 std::function< 类型结果，表示 void 的处理结果。
         */
        std::function<void(bool)> zebra_callback = nullptr;
        int zebra_state = 0;

#endif

    public:
        int image_width = 0;
        int image_height = 0;

        /**
         * @brief 构造 FixLine 对象
         *
         * @details
         * 创建 ImageProcess / FixLine 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         */
        FixLine();

        /**
         * @brief 提交 submitLeftStartPos 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param leftStartPos leftStartPos 参数，参与 submitLeftStartPos 的业务处理或状态更新。
         */
        void submitLeftStartPos(Point leftStartPos);
        /**
         * @brief 提交 submitRightStartPos 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param rightStartPos rightStartPos 参数，参与 submitRightStartPos 的业务处理或状态更新。
         */
        void submitRightStartPos(Point rightStartPos);
        /**
         * @brief 提交 submitLeftEndPos 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param leftEndPos leftEndPos 参数，参与 submitLeftEndPos 的业务处理或状态更新。
         */
        void submitLeftEndPos(Point leftEndPos);
        /**
         * @brief 提交 submitRightEndPos 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param rightEndPos rightEndPos 参数，参与 submitRightEndPos 的业务处理或状态更新。
         */
        void submitRightEndPos(Point rightEndPos);

        /**
         * @brief 提交 submitLeftLineContours 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param leftLineContours leftLineContours 参数，参与 submitLeftLineContours 的业务处理或状态更新。
         */
        void submitLeftLineContours(vector<Point> leftLineContours);
        /**
         * @brief 提交 submitRightLineContours 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param rightLineContours rightLineContours 参数，参与 submitRightLineContours 的业务处理或状态更新。
         */
        void submitRightLineContours(vector<Point> rightLineContours);

        /**
         * @brief 提交 submitLeftLine 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param leftLine 指针参数，指向调用方提供的数据或内部管理的资源。
         */
        void submitLeftLine(int *leftLine);
        /**
         * @brief 提交 submitCenterLine 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param centerLine 中心线数组指针，保存每一行对应的赛道中心位置。
         */
        void submitCenterLine(int *centerLine);
        /**
         * @brief 提交 submitRightLine 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param rightLine 指针参数，指向调用方提供的数据或内部管理的资源。
         */
        void submitRightLine(int *rightLine);

#ifdef FIXLINE_INCLUDE_ELEMENTS_ZEBRA

        /**
         * @brief 提交 submitZebraCallback 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param callback 回调函数指针，在 PID 计算前后或特定处理阶段被调用。
         */
        void submitZebraCallback(std::function<void(bool)> callback);

        /**
         * @brief 提交 submitZebraImage 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param zebraImage zebraImage 参数，参与 submitZebraImage 的业务处理或状态更新。
         */
        void submitZebraImage(Mat zebraImage);

        /**
         * @brief 检查 checkZebraSimple 状态
         *
         * @details
         * 检测 ImageProcess / FixLine 模块当前资源或设备状态是否可用，并将检查结果返回给调用方。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool checkZebraSimple();

        /**
         * @brief handleZebraSimple 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FixLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @return 返回 FixLineState 类型结果，表示 handleZebraSimple 的处理结果。
         */
        FixLineState handleZebraSimple();
#endif

        /**
         * @brief analyzeLineSimple 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FixLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         */
        void analyzeLineSimple();

        /**
         * @brief fixLineSimple 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FixLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         */
        void fixLineSimple();

        /**
         * @brief 析构 FixLine 对象
         *
         * @details
         * 释放 ImageProcess / FixLine 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~FixLine();
    };
}