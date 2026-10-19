#include "FindStartPos.hpp"

namespace ImageProcess
{
    /*
     * 起点搜索模块实现。
     *
     * 该模块专门负责从轮廓中确定赛道左右边界在图像底部的起始点。
     */

    /**
     * @brief 构造 FindStartPos 对象
     *
     * @details
     * 创建 ImageProcess / FindStartPos 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     */
    FindStartPos::FindStartPos() = default;

    /**
     * @brief 获取 getLeftStartPos 对应数据
     *
     * @details
     * 从 ImageProcess / FindStartPos 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    Point FindStartPos::getLeftStartPos()
    {
        // gImageProcessAttribute.left_start_pos = this->left_start_pos;
        return this->left_start_pos;
    }

    /**
     * @brief 获取 getRightStartPos 对应数据
     *
     * @details
     * 从 ImageProcess / FindStartPos 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    Point FindStartPos::getRightStartPos()
    {
        // gImageProcessAttribute.right_start_pos = this->right_start_pos;
        return this->right_start_pos;
    }

    /**
     * @brief 提交 submitCountorFrame 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FindStartPos 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param countorFrame countorFrame 参数，参与 submitCountorFrame 的业务处理或状态更新。
     */
    void FindStartPos::submitCountorFrame(Mat countorFrame)
    {
        this->countor_frame = countorFrame;
    }

    /**
     * @brief 查找 findStartPosByContours 对应节点
     *
     * @details
     * 在 ImageProcess / FindStartPos 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
     *
     * @param contours contours 参数，参与 findStartPosByContours 的业务处理或状态更新。
     * @param imageHeight imageHeight 参数，参与 findStartPosByContours 的业务处理或状态更新。
     */
    void FindStartPos::findStartPosByContours(vector<Point> contours, int imageHeight)
    {
        vector<Point> countZeroYPoint;
        for (int i = 0; i < contours.size(); i++)
        {
            const Point &point = contours[i];
            if (point.y == imageHeight)
            {
                countZeroYPoint.push_back(point);
            }
        }

        this->left_start_pos = {-1, -1};
        this->right_start_pos = {-1, -1};

        if (countZeroYPoint.size() == 0)
        {
            return;
        }

        this->left_start_pos.x = countZeroYPoint[0].x;
        this->left_start_pos.y = countZeroYPoint[0].y;
        this->right_start_pos.x = countZeroYPoint[0].x;
        this->right_start_pos.y = countZeroYPoint[0].y;

        for (int i = 0; i < countZeroYPoint.size(); i++)
        {
            const Point &point = countZeroYPoint[i];
            if (point.x < this->left_start_pos.x)
            {
                this->left_start_pos.x = point.x;
                this->left_start_pos.y = point.y;
            }
            else if (point.x > this->right_start_pos.x)
            {
                this->right_start_pos.x = point.x;
                this->right_start_pos.y = point.y;
            }
        }
    }

    /**
     * @brief 查找 findStartPosFromFrameByContoursSimple 对应节点
     *
     * @details
     * 在 ImageProcess / FindStartPos 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
     *
     * @param imageHeight imageHeight 参数，参与 findStartPosFromFrameByContoursSimple 的业务处理或状态更新。
     * @param pContours 指针参数，指向调用方提供的数据或内部管理的资源。
     */
    void FindStartPos::findStartPosFromFrameByContoursSimple(int imageHeight, vector<Point> *pContours)
    {
        HandleImage handleImage;
        handleImage.submitFindContourFrame(this->countor_frame);

        int chooseNum = 0;

        while (true)
        {
            vector<Point> contours = handleImage.findWhiteContourBigToSmall(chooseNum);

            if (contours.empty())
            {
                if (pContours != nullptr)
                {
                    pContours->clear();
                    *pContours = vector<Point>();
                }
                break;
            }

            this->findStartPosByContours(contours, imageHeight);

            if (this->left_start_pos.x == -1 || this->right_start_pos.x == -1)
            {
                chooseNum++;
                continue;
            }
            else
            {
                if (pContours != nullptr)
                {
                    pContours->clear();
                    *pContours = contours;
                }
                break;
            }
        }
    }

    /**
     * @brief 析构 FindStartPos 对象
     *
     * @details
     * 释放 ImageProcess / FindStartPos 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    FindStartPos::~FindStartPos() = default;

}