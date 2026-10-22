#pragma once
#include "headfile.hpp"

using namespace cv;
using namespace std;

namespace ImageProcess
{
    /*
     * 赛道边线提取类。
     *
     * 核心职责：
     * - 从轮廓图中识别左右起点与终点；
     * - 拆分左右边线轮廓；
     * - 生成按行索引的左线、右线和中线数组。
     */
    class FindLine
    {
    private:
        Mat contour_frame;

        vector<Point> all_contours;

        Point left_start_pos;
        Point right_start_pos;
        Point left_end_pos;
        Point right_end_pos;

        vector<Point> left_line_contours;
        vector<Point> right_line_contours;

        int *left_line = nullptr;
        int *center_line = nullptr;
        int *right_line = nullptr;
        bool missing_edge_proxy_enabled = true;

    public:
        /**
         * @brief 构造 FindLine 对象
         *
         * @details
         * 创建 ImageProcess / FindLine 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         */
        FindLine();
        /**
         * @brief 提交 submitCountorFrame 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FindLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param countorFrame countorFrame 参数，参与 submitCountorFrame 的业务处理或状态更新。
         */
        void submitCountorFrame(Mat countorFrame);
        /**
         * @brief 提交 submitContours 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FindLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param contours contours 参数，参与 submitContours 的业务处理或状态更新。
         */
        void submitContours(vector<Point> contours);
        /**
         * @brief handleContoursSimple 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FindLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         */
        void handleContoursSimple();
        /**
         * @brief handleContoursFromCountorFrameSimple 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FindLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool handleContoursFromCountorFrameSimple();
        /**
         * @brief handleContoursInTopSimple 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FindLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param isKeepTop 布尔开关参数，用于启用或关闭对应功能。
         */
        void handleContoursInTopSimple(bool isKeepTop = true);
        /**
         * @brief handleContoursInTopFromCountorFrameSimple 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / FindLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param isKeepTop 布尔开关参数，用于启用或关闭对应功能。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool handleContoursInTopFromCountorFrameSimple(bool isKeepTop = true);

        /**
         * @brief 查找 findLineContoursSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FindLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         */
        void findLineContoursSimple();

        /**
         * @brief 获取 getLeftLineContours 对应数据
         *
         * @details
         * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        vector<Point> getLeftLineContours();
        /**
         * @brief 获取 getRightLineContours 对应数据
         *
         * @details
         * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        vector<Point> getRightLineContours();

        /**
         * @brief 获取 getLeftStartPos 对应数据
         *
         * @details
         * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        Point getLeftStartPos();
        /**
         * @brief 获取 getRightStartPos 对应数据
         *
         * @details
         * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        Point getRightStartPos();
        /**
         * @brief 获取 getLeftEndPos 对应数据
         *
         * @details
         * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        Point getLeftEndPos();
        /**
         * @brief 获取 getRightEndPos 对应数据
         *
         * @details
         * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        Point getRightEndPos();

        /**
         * @brief 获取 getLeftLine 对应数据
         *
         * @details
         * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int *getLeftLine();
        void setMissingEdgeProxyEnabled(bool enabled);
        /**
         * @brief 获取 getCenterLine 对应数据
         *
         * @details
         * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int *getCenterLine();
        /**
         * @brief 获取 getRightLine 对应数据
         *
         * @details
         * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int *getRightLine();
        /**
         * @brief 析构 FindLine 对象
         *
         * @details
         * 释放 ImageProcess / FindLine 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~FindLine();
    };

}
