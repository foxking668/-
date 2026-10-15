#include "FindLine.hpp"
#include "MissingBoundaryPolicy.hpp"

namespace ImageProcess
{
    /**
     * @brief 构造 FindLine 对象
     *
     * @details
     * 创建 ImageProcess / FindLine 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     */
    FindLine::FindLine() = default;
    /**
     * @brief 提交 submitCountorFrame 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FindLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param countorFrame countorFrame 参数，参与 submitCountorFrame 的业务处理或状态更新。
     */
    void FindLine::submitCountorFrame(Mat countorFrame)
    {
        this->contour_frame = countorFrame;
    }
    /**
     * @brief 提交 submitContours 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FindLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param contours contours 参数，参与 submitContours 的业务处理或状态更新。
     */
    void FindLine::submitContours(vector<Point> contours)
    {
        this->all_contours = contours;
    }
    /**
     * @brief handleContoursSimple 函数说明
     *
     * @details
     * 该函数属于 ImageProcess / FindLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     */
    void FindLine::handleContoursSimple()
    {
        /*
         * 根据已提交的完整轮廓，寻找底部左右起点和边线终点。
         * 该函数主要用于已经有轮廓数据的场景。
         */
        FindStartPos findStartPos;
        findStartPos.findStartPosByContours(this->all_contours, this->contour_frame.rows - 1);

        Point leftStartPos = findStartPos.getLeftStartPos();
        Point rightStartPos = findStartPos.getRightStartPos();
        Point left_end_pos = {-1, -1};
        Point right_end_pos = {-1, -1};

        vector<Point> overLeftContours;
        vector<Point> overRightContours;
        vector<Point> overTopContours;

        int cutNumber = right_end_pos.x - left_end_pos.x;

        if (leftStartPos.x == -1 || rightStartPos.x == -1 || cutNumber > this->contour_frame.rows / 5)
        {
            goto __End;
        }

        for (int i = 0; i < this->all_contours.size(); i++)
        {
            const Point &point = this->all_contours[i];
            if (point.x == 0 && point.y != this->contour_frame.rows - 1)
            {
                overLeftContours.push_back(point);
            }
            else if (point.x == this->contour_frame.cols - 1 && point.y != this->contour_frame.rows - 1)
            {
                overRightContours.push_back(point);
            }
            else if (point.y == 0)
            {
                overTopContours.push_back(point);
            }
        }

        // check the top
        if (overTopContours.size() > 0)
        {
            if (overTopContours.size() == 1)
            {
                left_end_pos = overTopContours[0];
                right_end_pos = overTopContours[0];
                goto __End;
            }
            else if (overTopContours.size() >= 2)
            {
                sort(overTopContours.begin(), overTopContours.end(), [](Point a, Point b)
                     { return a.x < b.x; });

                left_end_pos = overTopContours[0];
                right_end_pos = overTopContours[overTopContours.size() - 1];

                goto __End;
            }
        }

        if (overLeftContours.size() > 0)
        {
            if (overLeftContours.size() == 1)
            {
                left_end_pos = overLeftContours[0];
                right_end_pos = overLeftContours[0];
            }
            else if (overLeftContours.size() >= 2)
            {
                sort(overLeftContours.begin(), overLeftContours.end(), [](Point a, Point b)
                     { return a.y < b.y; });

                right_end_pos = overLeftContours[0];
                left_end_pos = overLeftContours[1];
            }
        }

        if (overRightContours.size() > 0)
        {
            if (overRightContours.size() == 1)
            {
                if (overRightContours[0].y < right_end_pos.y || right_end_pos.y == -1)
                {
                    left_end_pos = overRightContours[0];
                    right_end_pos = overRightContours[0];
                }
            }
            else if (overRightContours.size() >= 2)
            {
                sort(overRightContours.begin(), overRightContours.end(), [](Point a, Point b)
                     { return a.y < b.y; });

                if (overRightContours[0].y < right_end_pos.y || right_end_pos.y == -1)
                {
                    left_end_pos = overRightContours[0];
                    right_end_pos = overRightContours[1];
                }
            }
        }

    __End:
        this->left_start_pos = leftStartPos;
        this->right_start_pos = rightStartPos;
        this->left_end_pos = left_end_pos;
        this->right_end_pos = right_end_pos;
        return;
    }
    /**
     * @brief handleContoursFromCountorFrameSimple 函数说明
     *
     * @details
     * 该函数属于 ImageProcess / FindLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool FindLine::handleContoursFromCountorFrameSimple()
    {
        /*
         * 从二值轮廓图中自动寻找最大有效轮廓，
         * 再基于轮廓边界判断左右起点和终点。
         */
        FindStartPos findStartPos;
        findStartPos.submitCountorFrame(this->contour_frame);
        findStartPos.findStartPosFromFrameByContoursSimple(this->contour_frame.rows - 1, &this->all_contours);

        Point leftStartPos = findStartPos.getLeftStartPos();
        Point rightStartPos = findStartPos.getRightStartPos();
        Point left_end_pos = {-1, -1};
        Point right_end_pos = {-1, -1};

        vector<Point> overLeftContours;
        vector<Point> overRightContours;
        vector<Point> overTopContours;

        int cutNumber = 0;

        if (leftStartPos.x != -1 && rightStartPos.x != -1)
        {
            cutNumber = rightStartPos.x - leftStartPos.x;
        }

        if (cutNumber < this->contour_frame.rows / 5)
        {
            goto __End;
        }

        for (int i = 0; i < this->all_contours.size(); i++)
        {
            const Point &point = this->all_contours[i];
            if (point.x == 0 && point.y != this->contour_frame.rows - 1)
            {
                overLeftContours.push_back(point);
            }
            else if (point.x == this->contour_frame.cols - 1 && point.y != this->contour_frame.rows - 1)
            {
                overRightContours.push_back(point);
            }
            else if (point.y == 0)
            {
                overTopContours.push_back(point);
            }
        }

        // printf("1");

        // check the top
        if (overTopContours.size() > 0)
        {
            if (overTopContours.size() == 1)
            {
                left_end_pos = overTopContours[0];
                right_end_pos = overTopContours[0];
                goto __End;
            }
            else if (overTopContours.size() >= 2)
            {
                sort(overTopContours.begin(), overTopContours.end(), [](Point a, Point b)
                     { return a.x < b.x; });

                left_end_pos = overTopContours[0];
                right_end_pos = overTopContours[overTopContours.size() - 1];

                goto __End;
            }
        }

        if (overLeftContours.size() > 0)
        {
            if (overLeftContours.size() == 1)
            {
                left_end_pos = overLeftContours[0];
                right_end_pos = overLeftContours[0];
            }
            else if (overLeftContours.size() >= 2)
            {
                sort(overLeftContours.begin(), overLeftContours.end(), [](Point a, Point b)
                     { return a.y < b.y; });

                right_end_pos = overLeftContours[0];
                left_end_pos = overLeftContours[1];
            }
        }

        if (overRightContours.size() > 0)
        {
            if (overRightContours.size() == 1)
            {
                if (overRightContours[0].y < right_end_pos.y || right_end_pos.y == -1)
                {
                    left_end_pos = overRightContours[0];
                    right_end_pos = overRightContours[0];
                }
            }
            else if (overRightContours.size() >= 2)
            {
                sort(overRightContours.begin(), overRightContours.end(), [](Point a, Point b)
                     { return a.y < b.y; });

                if (overRightContours[0].y < right_end_pos.y || right_end_pos.y == -1)
                {
                    left_end_pos = overRightContours[0];
                    right_end_pos = overRightContours[1];
                }
            }
        }

    __End:
        this->left_start_pos = leftStartPos;
        this->right_start_pos = rightStartPos;
        this->left_end_pos = left_end_pos;
        this->right_end_pos = right_end_pos;

        if (this->left_end_pos.x == -1 || this->right_end_pos.x == -1)
        {
            return false;
        }

        return true;
    }
    /**
     * @brief handleContoursInTopSimple 函数说明
     *
     * @details
     * 该函数属于 ImageProcess / FindLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     *
     * @param isKeepTop 布尔开关参数，用于启用或关闭对应功能。
     */
    void FindLine::handleContoursInTopSimple(bool isKeepTop)
    {
        FindStartPos findStartPos;
        findStartPos.findStartPosByContours(this->all_contours, this->contour_frame.rows - 1);

        Point leftStartPos = findStartPos.getLeftStartPos();
        Point rightStartPos = findStartPos.getRightStartPos();
        Point left_end_pos = {-1, -1};
        Point right_end_pos = {-1, -1};

        vector<Point> overTopContours;

        int cutNumber = right_end_pos.x - left_end_pos.x;

        int maxTop = 0;

        if (leftStartPos.x == -1 || rightStartPos.x == -1 || cutNumber > this->contour_frame.rows / 5)
        {
            goto __End;
        }

        maxTop = leftStartPos.y;

        for (int i = 0; i < this->all_contours.size(); i++)
        {
            if (maxTop > this->all_contours[i].y)
            {
                maxTop = this->all_contours[i].y;
            }
        }

        for (int i = 0; i < this->all_contours.size(); i++)
        {
            const Point &point = this->all_contours[i];
            if (point.y == maxTop)
            {
                overTopContours.push_back(point);
            }
        }

        // check the top
        if (overTopContours.size() > 0)
        {
            if (overTopContours.size() == 1)
            {
                left_end_pos = overTopContours[0];
                right_end_pos = overTopContours[0];
                goto __End;
            }
            else if (overTopContours.size() >= 2)
            {

                if (isKeepTop)
                {
                    left_end_pos = overTopContours[overTopContours.size() / 2];
                    right_end_pos = overTopContours[overTopContours.size() / 2 + 1];
                    goto __End;
                }
                sort(overTopContours.begin(), overTopContours.end(), [](Point a, Point b)
                     { return a.x < b.x; });

                left_end_pos = overTopContours[0];
                right_end_pos = overTopContours[overTopContours.size() - 1];

                goto __End;
            }
        }

    __End:
        this->left_start_pos = leftStartPos;
        this->right_start_pos = rightStartPos;
        this->left_end_pos = left_end_pos;
        this->right_end_pos = right_end_pos;
        return;
    }
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
    bool FindLine::handleContoursInTopFromCountorFrameSimple(bool isKeepTop)
    {
        /*
         * 当前主流程使用的轮廓处理方式。
         * 它会优先寻找轮廓最高处作为赛道远端边界，
         * 适合低分辨率前视赛道图像的中线估计。
         */
        FindStartPos findStartPos;
        findStartPos.submitCountorFrame(this->contour_frame);
        findStartPos.findStartPosFromFrameByContoursSimple(this->contour_frame.rows - 1, &this->all_contours);

        Point leftStartPos = findStartPos.getLeftStartPos();
        Point rightStartPos = findStartPos.getRightStartPos();
        Point left_end_pos = {-1, -1};
        Point right_end_pos = {-1, -1};

        vector<Point> overTopContours;

        int maxTop = 0;

        int cutNumber = 0;

        if (leftStartPos.x != -1 && rightStartPos.x != -1)
        {
            cutNumber = rightStartPos.x - leftStartPos.x;
        }
        if (cutNumber < this->contour_frame.rows / 5)
        {
            goto __End;
        }

        maxTop = leftStartPos.y;

        for (int i = 0; i < this->all_contours.size(); i++)
        {
            if (maxTop > this->all_contours[i].y)
            {
                maxTop = this->all_contours[i].y;
            }
        }

        for (int i = 0; i < this->all_contours.size(); i++)
        {
            const Point &point = this->all_contours[i];
            if (point.y == maxTop)
            {
                overTopContours.push_back(point);
            }
        }

        if (overTopContours.size() > 0)
        {
            if (overTopContours.size() == 1)
            {
                left_end_pos = overTopContours[0];
                right_end_pos = overTopContours[0];
                goto __End;
            }
            else if (overTopContours.size() >= 2)
            {

                if (isKeepTop)
                {
                    left_end_pos = overTopContours[overTopContours.size() / 2];
                    right_end_pos = overTopContours[overTopContours.size() / 2 + 1];
                    goto __End;
                }

                sort(overTopContours.begin(), overTopContours.end(), [](Point a, Point b)
                     { return a.x < b.x; });

                left_end_pos = overTopContours[0];
                right_end_pos = overTopContours[overTopContours.size() - 1];

                goto __End;
            }
        }

    __End:
        this->left_start_pos = leftStartPos;
        this->right_start_pos = rightStartPos;
        this->left_end_pos = left_end_pos;
        this->right_end_pos = right_end_pos;
        if (this->left_end_pos.x == -1 || this->right_end_pos.x == -1)
        {
            return false;
        }
        return true;
    }
    /**
     * @brief 查找 findLineContoursSimple 对应节点
     *
     * @details
     * 在 ImageProcess / FindLine 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
     */
    void FindLine::findLineContoursSimple()
    {
        /*
         * 根据起点和终点在总轮廓序列中的位置，
         * 将完整轮廓拆分为左边线轮廓与右边线轮廓。
         */
        if (this->left_start_pos.x == -1 || this->right_start_pos.x == -1)
        {
            return;
        }

        int left_start_pos_i = -1;
        int right_start_pos_i = -1;
        int left_end_pos_i = -1;
        int right_end_pos_i = -1;

        for (int i = 0; i < this->all_contours.size(); i++)
        {
            if (this->all_contours[i].x == this->left_start_pos.x && this->all_contours[i].y == this->left_start_pos.y)
            {
                left_start_pos_i = i;
            }
            if (this->all_contours[i].x == this->right_start_pos.x && this->all_contours[i].y == this->right_start_pos.y)
            {
                right_start_pos_i = i;
            }
            if (this->all_contours[i].x == this->left_end_pos.x && this->all_contours[i].y == this->left_end_pos.y)
            {
                left_end_pos_i = i;
            }
            if (this->all_contours[i].x == this->right_end_pos.x && this->all_contours[i].y == this->right_end_pos.y)
            {
                right_end_pos_i = i;
            }
        }

        for (int i = 0; i < this->all_contours.size(); i++)
        {
            if (left_end_pos_i < left_start_pos_i)
            {
                if (i <= left_start_pos_i && i >= left_end_pos_i)
                {
                    this->left_line_contours.push_back(this->all_contours[i]);
                }
            }
            else
            {
                if (i <= left_start_pos_i || i >= left_end_pos_i)
                {
                    this->left_line_contours.push_back(this->all_contours[i]);
                }
            }

            if (right_end_pos_i < right_start_pos_i)
            {
                if (i >= right_start_pos_i || i <= right_end_pos_i)
                {
                    this->right_line_contours.push_back(this->all_contours[i]);
                }
            }
            else
            {
                if (i >= right_start_pos_i && i <= right_end_pos_i)
                {
                    this->right_line_contours.push_back(this->all_contours[i]);
                }
            }
        }
    }
    /**
     * @brief 获取 getLeftLineContours 对应数据
     *
     * @details
     * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    vector<Point> FindLine::getLeftLineContours()
    {
        return this->left_line_contours;
    }
    /**
     * @brief 获取 getRightLineContours 对应数据
     *
     * @details
     * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    vector<Point> FindLine::getRightLineContours()
    {
        return this->right_line_contours;
    }
    /**
     * @brief 获取 getLeftStartPos 对应数据
     *
     * @details
     * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    Point FindLine::getLeftStartPos()
    {
        return this->left_start_pos;
    }
    /**
     * @brief 获取 getRightStartPos 对应数据
     *
     * @details
     * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    Point FindLine::getRightStartPos()
    {
        return this->right_start_pos;
    }
    /**
     * @brief 获取 getLeftEndPos 对应数据
     *
     * @details
     * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    Point FindLine::getLeftEndPos()
    {
        return this->left_end_pos;
    }
    /**
     * @brief 获取 getRightEndPos 对应数据
     *
     * @details
     * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    Point FindLine::getRightEndPos()
    {
        return this->right_end_pos;
    }
    /**
     * @brief 获取 getLeftLine 对应数据
     *
     * @details
     * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    int *FindLine::getLeftLine()
    {
        if (this->left_line != nullptr)
        {
            return this->left_line;
        }

        const int screenHeight = this->contour_frame.rows;

        this->left_line = new int[screenHeight];
        std::fill(this->left_line, this->left_line + screenHeight, -1); // 初始化为默认值

        if (this->left_line_contours.empty())
        {
            return this->left_line;
        }

        for (const auto &point : this->left_line_contours)
        {
            if (this->left_line[point.y] < point.x || this->left_line[point.y] == -1)
            {
                this->left_line[point.y] = point.x;
            }
        }

        return this->left_line;
    }

    void FindLine::setMissingEdgeProxyEnabled(bool enabled)
    {
        this->missing_edge_proxy_enabled = enabled;
    }
    /**
     * @brief 获取 getCenterLine 对应数据
     *
     * @details
     * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    int *FindLine::getCenterLine()
    {
        if (this->center_line != nullptr)
        {
            return this->center_line;
        }

        if (this->left_line == nullptr)
        {
            this->getLeftLine();
        }

        if (this->right_line == nullptr)
        {
            this->getRightLine();
        }

        const int screenHeight = this->contour_frame.rows;
        const int screenWidth = this->contour_frame.cols;
        const int endHeight = this->left_end_pos.y > this->right_end_pos.y ? this->left_end_pos.y : this->right_end_pos.y;

        this->center_line = new int[screenHeight];

        for (int i = 0; i < screenHeight; i++)
        {
            if (i <= endHeight || endHeight == -1)
            {
                this->center_line[i] = -1;
                continue;
            }
            this->center_line[i] = applyMissingBoundaryPolicy(
                this->left_line[i],
                this->right_line[i],
                screenWidth,
                this->missing_edge_proxy_enabled);
        }

        return this->center_line;
    }
    /**
     * @brief 获取 getRightLine 对应数据
     *
     * @details
     * 从 ImageProcess / FindLine 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    int *FindLine::getRightLine()
    {
        if (this->right_line != nullptr)
        {
            return this->right_line;
        }

        const int screenHeight = this->contour_frame.rows;

        this->right_line = new int[screenHeight];
        std::fill(this->right_line, this->right_line + screenHeight, -1);

        if (this->right_line_contours.empty())
        {
            return this->right_line;
        }

        for (const auto &point : this->right_line_contours)
        {
            if (this->right_line[point.y] > point.x || this->right_line[point.y] == -1)
            {
                this->right_line[point.y] = point.x;
            }
        }

        return this->right_line;
    }
    /**
     * @brief 析构 FindLine 对象
     *
     * @details
     * 释放 ImageProcess / FindLine 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    FindLine::~FindLine()
    {
        if (this->left_line != nullptr)
            delete[] this->left_line;

        if (this->center_line != nullptr)
            delete[] this->center_line;

        if (this->right_line != nullptr)
            delete[] this->right_line;
    };
}
