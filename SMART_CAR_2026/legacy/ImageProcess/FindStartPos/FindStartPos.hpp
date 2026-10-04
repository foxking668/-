#pragma once
#include "headfile.hpp"

using namespace cv;
using namespace std;

namespace ImageProcess
{
    /*
     * 起始点搜索类。
     *
     * 用于从轮廓或二值图中寻找赛道左右边界在图像底部的起始位置，
     * 这是后续左右边线分离与中线计算的基础。
     */
    class FindStartPos
    {
    private:
        Mat countor_frame;

        Point left_start_pos;
        Point right_start_pos;

    public:
        /**
         * @brief 构造 FindStartPos 对象
         *
         * @details
         * 创建 ImageProcess / FindStartPos 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         */
        FindStartPos();

        /**
         * @brief 获取 getLeftStartPos 对应数据
         *
         * @details
         * 从 ImageProcess / FindStartPos 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        Point getLeftStartPos();
        /**
         * @brief 获取 getRightStartPos 对应数据
         *
         * @details
         * 从 ImageProcess / FindStartPos 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        Point getRightStartPos();

        /**
         * @brief 提交 submitCountorFrame 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / FindStartPos 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param countorFrame countorFrame 参数，参与 submitCountorFrame 的业务处理或状态更新。
         */
        void submitCountorFrame(Mat countorFrame);

        /**
         * @brief 查找 findStartPosByContours 对应节点
         *
         * @details
         * 在 ImageProcess / FindStartPos 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param contours contours 参数，参与 findStartPosByContours 的业务处理或状态更新。
         * @param imageHeight imageHeight 参数，参与 findStartPosByContours 的业务处理或状态更新。
         */
        void findStartPosByContours(vector<Point> contours, int imageHeight);

        /**
         * @brief 查找 findStartPosFromFrameByContoursSimple 对应节点
         *
         * @details
         * 在 ImageProcess / FindStartPos 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param imageHeight imageHeight 参数，参与 findStartPosFromFrameByContoursSimple 的业务处理或状态更新。
         * @param pContours 指针参数，指向调用方提供的数据或内部管理的资源。
         */
        void findStartPosFromFrameByContoursSimple(int imageHeight, vector<Point> *pContours = nullptr);

        /**
         * @brief 析构 FindStartPos 对象
         *
         * @details
         * 释放 ImageProcess / FindStartPos 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~FindStartPos();
    };

}
