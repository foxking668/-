#include "FixLine.hpp"
#include "ImageProcess/FindLine/MissingBoundaryPolicy.hpp"

// using namespace Other;

namespace ImageProcess
{
    /*
     * 计算两个向量之间的夹角。
     *
     * 该函数用于曲率/拐点识别，通过比较前后方向变化，
     * 判断轮廓或边线在某一点是否形成明显转角。
     */
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
    double FixLine::calculateAngle(int x1, int y1, int x2, int y2)
    {
        // 计算两个向量的模长
        double magnitudeA = std::sqrt(x1 * x1 + y1 * y1);
        double magnitudeB = std::sqrt(x2 * x2 + y2 * y2);

        // 如果任一向量为零向量，则返回180度作为特殊处理
        if (magnitudeA == 0.0 || magnitudeB == 0.0)
        {
            return 180.0; // 注意: 这里选择返回180度而非抛出异常
        }

        // 计算两向量的点积
        double dotProduct = x1 * x2 + y1 * y2;

        // 计算夹角余弦值，并确保其在[-1, 1]范围内以避免浮点运算误差引起的acos函数参数溢出
        double cosTheta = dotProduct / (magnitudeA * magnitudeB);
        cosTheta = std::max(std::min(cosTheta, 1.0), -1.0);

        // 使用acos计算夹角（单位：弧度），然后转换为角度
        double angleDegrees = std::acos(cosTheta) * (180.0 / M_PI);

        return angleDegrees;
    }

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
    vector<pair<Point, Point>> FixLine::findAllLineBreakFromBottomSimple(int *line, int threshold, int sampleNumber)
    {
        vector<pair<Point, Point>> breakPoints; // 用于存储断裂点

        for (int i = this->image_height - 1; i > 1; i--)
        {
            const int nowPos = line[i];
            const int nextPos = line[i - 1];

            if (nowPos < 0 || nextPos < 0)
            {
                continue;
            }

            if (abs(nextPos - nowPos) > threshold)
            {
                int checkSamplePoint = 0;
                for (size_t s_i = 0; s_i < sampleNumber; s_i++)
                {
                    if (i + s_i > this->image_height - 1)
                    {
                        break;
                    }

                    const int nowSamplePos = line[i];
                    const int nextSamplePos = line[i - 1];

                    checkSamplePoint += nextSamplePos - nowSamplePos;
                }

                if (checkSamplePoint / sampleNumber >= threshold)
                {
                    i -= sampleNumber;
                }

                // 记录断裂点
                breakPoints.push_back(make_pair(Point(nextPos, i - 1), Point(nowPos, i)));
            }
        }

        return breakPoints; // 返回所有断裂点
    }
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
    std::vector<pair<Point, int>> FixLine::findAllContoursCornerSimple(const vector<Point> &icontours, int threshold, int sampleNumber, int startHeight, int endHeight)
    {
        if (icontours.empty())
        {
            return {};
        }

        vector<Point> contours = icontours;

        if (startHeight != -1 || endHeight != -1)
        {
            contours.erase(std::remove_if(contours.begin(), contours.end(),
                                          [startHeight, endHeight](const Point &point)
                                          {
                                              return (startHeight != -1 && point.y < startHeight) || (endHeight != -1 && point.y > endHeight);
                                          }),
                           contours.end());
        }

        if (sampleNumber % 2 == 0)
        {
            throw std::invalid_argument("sampleNumber must be odd");
        }

        const int halfSampleNumber = sampleNumber / 2;
        const int maxCutY = 3;
        const int maxCutX = 3;

        std::vector<pair<Point, int>> breakPoints;

        for (size_t i = halfSampleNumber; i < contours.size() - halfSampleNumber; ++i)
        {
            const Point &nowPoint = contours[i];
            int curvityFrontCounterX = 0;
            int curvityFrontCounterY = 0;
            int curvityBackCounterX = 0;
            int curvityBackCounterY = 0;

            // 前向部分
            for (int frontI = halfSampleNumber; frontI >= 1; --frontI)
            {
                const Point &lastFrontPoint = contours[i - frontI];
                const Point &nowFrontPoint = contours[i - frontI + 1];

                int cutX = nowPoint.x - nowFrontPoint.x;
                int cutY = nowPoint.y - nowFrontPoint.y;

                if (std::abs(cutY) > maxCutY || std::abs(cutX) > maxCutX)
                {
                    continue;
                }

                curvityFrontCounterX += cutX;
                curvityFrontCounterY += cutY;
            }

            // 后向部分
            for (int backI = 0; backI < halfSampleNumber; ++backI)
            {
                const Point &lastBackPoint = contours[i + backI + 1];
                const Point &nowBackPoint = contours[i + backI];

                int cutX = nowPoint.x - nowBackPoint.x;
                int cutY = nowPoint.y - nowBackPoint.y;

                if (std::abs(cutY) > maxCutY || std::abs(cutX) > maxCutX)
                {
                    continue;
                }

                curvityBackCounterX += cutX;
                curvityBackCounterY += cutY;
            }

            // 计算角度
            double angle = this->calculateAngle(curvityFrontCounterX, curvityFrontCounterY, curvityBackCounterX, curvityBackCounterY);

            if (angle <= threshold)
            {
                breakPoints.push_back(make_pair(nowPoint, i));
                i += halfSampleNumber;
            }
        }

        contours.clear();

        return breakPoints;
    }

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
    vector<Point> FixLine::findALLLineCornerSimple(int *line, int threshold, int sampleNumber)
    {
        if (line == nullptr)
        {
            return vector<Point>();
        }

        const int halfSampleNumber = sampleNumber / 2;
        const int height = this->image_height;
        const int width = this->image_width;

        std::vector<Point> breakPoints;

        for (size_t i = 0; i < height - halfSampleNumber; i++)
        {
            const int x = line[i];
            const int y = i;

            int curvityFrontCounterX = 0;
            int curvityFrontCounterY = 0;
            int curvityBackCounterX = 0;
            int curvityBackCounterY = 0;

            if (x == 0 || x == width - 1)
            {
                continue;
            }

            for (size_t frontI = halfSampleNumber; frontI >= 1; frontI++)
            {

                const int frontX1 = line[y - frontI];
                const int frontX2 = line[y - frontI + 1];

                if (frontX1 != 0 && frontX2 != 0 && frontX1 != width - 1 && frontX2 != width - 1)
                {
                    curvityFrontCounterX += frontX2 - frontX1;
                    curvityFrontCounterY += 1;
                }
            }

            for (int backI = 0; backI < halfSampleNumber; ++backI)
            {

                const int backX1 = line[y + backI + 1];
                const int backX2 = line[y + backI];

                if (backX1 != 0 && backX2 != 0 && backX1 != width - 1 && backX2 != width - 1)
                {
                    curvityBackCounterX += backX1 - backX2;
                    curvityBackCounterY += 1;
                }
            }

            double angle = this->calculateAngle(curvityFrontCounterX, curvityFrontCounterY, curvityBackCounterX, curvityBackCounterY);

            if (angle <= threshold)
            {
                breakPoints.push_back(Point(x, y));
                i += halfSampleNumber;
            }
        }

        return breakPoints;
    }

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
    pair<Point, Point> FixLine::findLineBreakFromBottomSimple(int *line, int threshold, int startPos)
    {
        pair<Point, Point> breakPoint = {{-1, -1}, {-1, -1}}; // 用于存储断裂点

        for (int i = this->image_height - 1 - startPos; i > 1; i--)
        {
            const int nowPos = line[i];
            const int nextPos = line[i - 1];

            if (nowPos < 0 || nextPos < 0)
            {
                continue;
            }

            if (abs(nextPos - nowPos) > threshold)
            {
                // 记录断裂点
                breakPoint = make_pair(Point(nextPos, i - 1), Point(nowPos, i));
                break;
            }
        }

        return breakPoint; // 返回所有断裂点
    }

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
    pair<Point, Point> FixLine::findLineBreakFromTopSimple(int *line, int threshold, int startPos)
    {
        pair<Point, Point> breakPoint = {{-1, -1}, {-1, -1}}; // 用于存储断裂点

        for (int i = startPos; i < this->image_height - 1; i++)
        {
            const int nowPos = line[i];
            const int nextPos = line[i + 1];

            if (nowPos < 0 || nextPos < 0)
            {
                continue;
            }

            if (abs(nextPos - nowPos) > threshold)
            {
                // 记录断裂点
                breakPoint = make_pair(Point(nextPos, i - 1), Point(nowPos, i));
                break;
            }
        }

        return breakPoint; // 返回所有断裂点
    }

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
    pair<Point, int> FixLine::findContoursCornerFromStartSimple(const vector<Point> &icontours, int threshold, int sampleNumber, int startHeight, int endHeight, int startPos)
    {

        if (icontours.empty())
        {
            return {};
        }

        vector<Point> contours = icontours;

        if (startHeight != -1 || endHeight != -1)
        {
            contours.erase(std::remove_if(contours.begin(), contours.end(),
                                          [startHeight, endHeight](const Point &point)
                                          {
                                              return (startHeight != -1 && point.y < startHeight) || (endHeight != -1 && point.y > endHeight);
                                          }),
                           contours.end());
        }

        if (sampleNumber % 2 == 0)
        {
            throw std::invalid_argument("sampleNumber must be odd");
        }

        const int halfSampleNumber = sampleNumber / 2;
        const int maxCutY = 3;
        const int maxCutX = 3;

        Point breakPoints = {-1, -1};
        int index = -1;

        for (size_t i = startPos + halfSampleNumber; i < contours.size() - halfSampleNumber; ++i)
        {
            const Point &nowPoint = contours[i];
            int curvityFrontCounterX = 0;
            int curvityFrontCounterY = 0;
            int curvityBackCounterX = 0;
            int curvityBackCounterY = 0;

            // 前向部分
            for (int frontI = halfSampleNumber; frontI >= 1; --frontI)
            {
                const Point &lastFrontPoint = contours[i - frontI];
                const Point &nowFrontPoint = contours[i - frontI + 1];

                int cutX = nowPoint.x - nowFrontPoint.x;
                int cutY = nowPoint.y - nowFrontPoint.y;

                if (std::abs(cutY) > maxCutY || std::abs(cutX) > maxCutX)
                {
                    continue;
                }

                curvityFrontCounterX += cutX;
                curvityFrontCounterY += cutY;
            }

            // 后向部分
            for (int backI = 0; backI < halfSampleNumber; ++backI)
            {
                const Point &lastBackPoint = contours[i + backI + 1];
                const Point &nowBackPoint = contours[i + backI];

                int cutX = nowPoint.x - nowBackPoint.x;
                int cutY = nowPoint.y - nowBackPoint.y;

                if (std::abs(cutY) > maxCutY || std::abs(cutX) > maxCutX)
                {
                    continue;
                }

                curvityBackCounterX += cutX;
                curvityBackCounterY += cutY;
            }

            // 计算角度
            double angle = this->calculateAngle(curvityFrontCounterX, curvityFrontCounterY, curvityBackCounterX, curvityBackCounterY);

            if (angle <= threshold)
            {
                index = i;
                i += halfSampleNumber;
                breakPoints = nowPoint;
                break;
            }
        }

        contours.clear();

        return make_pair(breakPoints, index);
    }

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
    pair<Point, int> FixLine::findContoursCornerFromEndSimple(const vector<Point> &icontours, int threshold, int sampleNumber, int startHeight, int endHeight, int startPos)
    {
        if (icontours.empty())
        {
            return {};
        }

        vector<Point> contours = icontours;

        if (startHeight != -1 || endHeight != -1)
        {
            contours.erase(std::remove_if(contours.begin(), contours.end(),
                                          [startHeight, endHeight](const Point &point)
                                          {
                                              return (startHeight != -1 && point.y < startHeight) || (endHeight != -1 && point.y > endHeight);
                                          }),
                           contours.end());
        }

        std::reverse(contours.begin(), contours.end());

        if (sampleNumber % 2 == 0)
        {
            throw std::invalid_argument("sampleNumber must be odd");
        }

        const int halfSampleNumber = sampleNumber / 2;
        const int maxCutY = 3;
        const int maxCutX = 3;

        Point breakPoints = {-1, -1};
        int index = -1;

        for (size_t i = startPos + halfSampleNumber; i < contours.size() - halfSampleNumber; ++i)
        {
            const Point &nowPoint = contours[i];
            int curvityFrontCounterX = 0;
            int curvityFrontCounterY = 0;
            int curvityBackCounterX = 0;
            int curvityBackCounterY = 0;

            // 前向部分
            for (int frontI = halfSampleNumber; frontI >= 1; --frontI)
            {
                const Point &lastFrontPoint = contours[i - frontI];
                const Point &nowFrontPoint = contours[i - frontI + 1];

                int cutX = nowPoint.x - nowFrontPoint.x;
                int cutY = nowPoint.y - nowFrontPoint.y;

                if (std::abs(cutY) > maxCutY || std::abs(cutX) > maxCutX)
                {
                    continue;
                }

                curvityFrontCounterX += cutX;
                curvityFrontCounterY += cutY;
            }

            // 后向部分
            for (int backI = 0; backI < halfSampleNumber; ++backI)
            {
                const Point &lastBackPoint = contours[i + backI];
                const Point &nowBackPoint = contours[i + backI + 1];

                int cutX = nowPoint.x - nowBackPoint.x;
                int cutY = nowPoint.y - nowBackPoint.y;

                if (std::abs(cutY) > maxCutY || std::abs(cutX) > maxCutX)
                {
                    continue;
                }

                curvityBackCounterX += cutX;
                curvityBackCounterY += cutY;
            }

            // 计算角度
            double angle = this->calculateAngle(curvityFrontCounterX, curvityFrontCounterY, curvityBackCounterX, curvityBackCounterY);

            if (angle <= threshold)
            {
                index = i;
                i += halfSampleNumber;
                breakPoints = nowPoint;
                break;
            }
        }

        contours.clear();

        return make_pair(breakPoints, index);
    }

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
    Point FixLine::findOutAreaFromButtomSimple(int *line, int startPos)
    {
        Point outAreaBreak = {-1, -1};
        for (int i = this->image_height - 1 - startPos; i > 1; i--)
        {
            const int nowPos = line[i];
            const int nextPos = line[i - 1];

            if (nowPos == this->image_width - 1 && nextPos != this->image_width - 1)
            {
                outAreaBreak = Point(nowPos, i);
                break;
            }

            if (nowPos != this->image_width - 1 && nextPos == this->image_width - 1)
            {
                outAreaBreak = Point(nowPos, i);
                break;
            }

            if (nowPos == 0 && nowPos != 0)
            {
                outAreaBreak = Point(nowPos, i);
                break;
            }

            if (nowPos != 0 && nextPos == 0)
            {
                outAreaBreak = Point(nowPos, i);
                break;
            }
        }
        return outAreaBreak;
    }

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
    Point FixLine::findBreakPointInOutAreaFromBottomSimple(int *line, int threshold, int startPos, int endPos, int searchLevel)
    {
        Point breakPoint = {-1, -1}; // 用于存储断裂点

        startPos = this->image_height - startPos - 1;
        endPos = this->image_height - endPos - 1 + searchLevel;

        for (int i = startPos; i > endPos; i--)
        {
            const int nowPos = line[i];

            if (nowPos < 0)
            {
                continue;
            }

            if (this->checkPointIsOutAreaSimple(nowPos))
            {
                int countDistance = 0;
                for (int n = 1; n < searchLevel; n++)
                {
                    const int searchPos = line[i - n];

                    if (this->checkPointIsOutAreaSimple(searchPos))
                    {
                        break;
                    }
                    else
                    {
                        countDistance += std::abs(searchPos - nowPos);
                    }
                }

                if ((countDistance / searchLevel) > threshold)
                {
                    breakPoint = Point(line[i - searchLevel], i - searchLevel);
                    break;
                }
            }
        }

        return breakPoint;
    }

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
    Point FixLine::findInOutAreaPointFromBottomSimple(int *line, int startPos, int endPos)
    {
        Point breakPoint = {-1, -1}; // 用于存储断裂点

        startPos = this->image_height - startPos - 1;
        endPos = this->image_height - endPos - 1;

        for (int i = startPos; i > endPos + 1; i--)
        {
            const int nowPos = line[i];

            if (nowPos < 0)
            {
                continue;
            }

            if (this->checkPointIsOutAreaSimple(nowPos))
            {
                if (this->checkPointIsOutAreaSimple(line[i + 1]))
                {
                    breakPoint = Point(line[i + 1], i + 1);
                }
            }
        }

        return breakPoint;
    }

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
    int FixLine::countJumpBlockInRowFromBinFrameSimple(Mat binImage, int rowIndex, int startPos, int endPos, int whiteThreshold, int blackThreshold, unsigned char whiteBlockValue, unsigned char blackBlockValue)
    {
        // 参数检查
        if (binImage.empty() || rowIndex < 0 || rowIndex >= binImage.rows)
        {
            return 0;
        }

        // 确保起始和结束位置在有效范围内
        startPos = max(0, startPos);
        endPos = min(binImage.cols - 1, endPos);
        if (startPos >= endPos)
        {
            return 0;
        }

        int jumpCount = 0;
        int currentWhiteRun = 0; // 当前白色连续计数
        int currentBlackRun = 0; // 当前黑色连续计数
        bool lastValidBlockWasWhite = false;
        bool hasValidBlock = false;

        // 获取指定行的指针
        const uchar *rowPtr = binImage.ptr<uchar>(rowIndex);

        for (int col = startPos; col <= endPos; col++)
        {
            uchar pixel = rowPtr[col];

            if (pixel == whiteBlockValue)
            {
                currentWhiteRun++;
                currentBlackRun = 0; // 重置黑色计数

                // 检查是否达到白色阈值
                if (currentWhiteRun == whiteThreshold)
                {
                    // 如果是第一个有效块或者与前一个有效块类型不同
                    if (!hasValidBlock || !lastValidBlockWasWhite)
                    {
                        jumpCount++;
                        hasValidBlock = true;
                        lastValidBlockWasWhite = true;
                    }
                }
            }
            else if (pixel == blackBlockValue)
            {
                currentBlackRun++;
                currentWhiteRun = 0; // 重置白色计数

                // 检查是否达到黑色阈值
                if (currentBlackRun == blackThreshold)
                {
                    // 如果是第一个有效块或者与前一个有效块类型不同
                    if (!hasValidBlock || lastValidBlockWasWhite)
                    {
                        jumpCount++;
                        hasValidBlock = true;
                        lastValidBlockWasWhite = false;
                    }
                }
            }
            else
            {
                // 非白非黑的像素，重置两个计数器
                currentWhiteRun = 0;
                currentBlackRun = 0;
            }
        }

        return jumpCount;
    }

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
    int FixLine::countOutAreaPointSimple(int *line, int startPos, int endPos, int cut)
    {
        if (line == nullptr)
        {
            return 0;
        }

        if (line != nullptr && startPos >= 0 && endPos < this->image_height)
        {
            int count = 0;
            for (int i = startPos; i < endPos; i++)
            {
                if (this->checkPointIsOutAreaSimple(line[i], cut))
                {
                    count++;
                }
            }
            return count;
        }
        else
        {
            return 0;
        }
    }

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
    bool FixLine::checkPointIsOutAreaSimple(int pointX, int cut)
    {

        // I dont know this code can be use or not
        if (pointX < 0)
        {
            return false;
        }

        if (cut < 0)
        {
            cut = this->image_width / (-cut);
        }

        const int outlineLeft = cut;
        const int outlineRight = this->image_width - cut - 1;

        return pointX <= outlineLeft || pointX >= outlineRight;
    }

    /**
     * @brief 释放 clearLineFormTopSimple 相关资源
     *
     * @details
     * 关闭、清理或释放 ImageProcess / FixLine 模块占用的动态内存、文件描述符、缓存节点或硬件资源。
     *
     * @param line 线段或线数组数据。
     * @param height 目标图像高度或裁剪高度。
     */
    void FixLine::clearLineFormTopSimple(int *line, int height)
    {
        if (line == nullptr)
        {
            return;
        }

        if (height < 0)
        {
            return;
        }

        const int screenHeight = this->image_height;

        if (height > screenHeight)
        {
            height = screenHeight;
        }

        for (int i = 0; i < height; i++)
        {
            line[i] = -1;
        }
    }

    /**
     * @brief drawLineSimple 函数说明
     *
     * @details
     * 该函数属于 ImageProcess / FixLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     *
     * @param line 线段或线数组数据。
     * @param fixInfo fixInfo 参数，参与 drawLineSimple 的业务处理或状态更新。
     */
    void FixLine::drawLineSimple(int *line, FixInfo fixInfo)
    {
        const Point &startPos = fixInfo.start;
        const Point &endPos = fixInfo.end;
        const int screenHeight = this->image_height;

        int x0 = startPos.x;
        int y0 = startPos.y;
        int x1 = endPos.x;
        int y1 = endPos.y;

        // 检查起点和终点是否相同
        if (x0 == x1 && y0 == y1)
        {
            if (y0 >= 0 && y0 < screenHeight)
            {
                line[y0] = x0;
            }
            return;
        }

        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            // 添加屏幕高度边界检查
            if (y0 >= 0 && y0 < screenHeight)
            {
                line[y0] = x0; // 仅在合法范围内写入
            }

            if ((x0 == x1 && y0 == y1) || (y0 > screenHeight) || (y0 < 0))
                break;

            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    /**
     * @brief 构造 FixLine 对象
     *
     * @details
     * 创建 ImageProcess / FixLine 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     */
    FixLine::FixLine() = default;
    /**
     * @brief 提交 submitLeftStartPos 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param leftStartPos leftStartPos 参数，参与 submitLeftStartPos 的业务处理或状态更新。
     */
    void FixLine::submitLeftStartPos(Point leftStartPos)
    {
        this->left_start_pos = leftStartPos;
    }

    /**
     * @brief 提交 submitRightStartPos 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param rightStartPos rightStartPos 参数，参与 submitRightStartPos 的业务处理或状态更新。
     */
    void FixLine::submitRightStartPos(Point rightStartPos)
    {
        this->right_start_pos = rightStartPos;
    }

    /**
     * @brief 提交 submitLeftEndPos 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param leftEndPos leftEndPos 参数，参与 submitLeftEndPos 的业务处理或状态更新。
     */
    void FixLine::submitLeftEndPos(Point leftEndPos)
    {
        this->left_end_pos = leftEndPos;
    }

    /**
     * @brief 提交 submitRightEndPos 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param rightEndPos rightEndPos 参数，参与 submitRightEndPos 的业务处理或状态更新。
     */
    void FixLine::submitRightEndPos(Point rightEndPos)
    {
        this->right_end_pos = rightEndPos;
    }

    /**
     * @brief 提交 submitLeftLineContours 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param leftLineContours leftLineContours 参数，参与 submitLeftLineContours 的业务处理或状态更新。
     */
    void FixLine::submitLeftLineContours(vector<Point> leftLineContours)
    {
        this->left_line_contours.clear();
        this->left_line_contours = leftLineContours;
    }
    /**
     * @brief 提交 submitRightLineContours 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param rightLineContours rightLineContours 参数，参与 submitRightLineContours 的业务处理或状态更新。
     */
    void FixLine::submitRightLineContours(vector<Point> rightLineContours)
    {
        this->right_line_contours.clear();
        this->right_line_contours = rightLineContours;
    }
    /**
     * @brief 提交 submitLeftLine 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param leftLine 指针参数，指向调用方提供的数据或内部管理的资源。
     */
    void FixLine::submitLeftLine(int *leftLine)
    {
        this->left_line = leftLine;
    }
    /**
     * @brief 提交 submitCenterLine 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param centerLine 中心线数组指针，保存每一行对应的赛道中心位置。
     */
    void FixLine::submitCenterLine(int *centerLine)
    {
        this->center_line = centerLine;
    }
    /**
     * @brief 提交 submitRightLine 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / FixLine 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param rightLine 指针参数，指向调用方提供的数据或内部管理的资源。
     */
    void FixLine::submitRightLine(int *rightLine)
    {
        this->right_line = rightLine;
    }
    /**
     * @brief analyzeLineSimple 函数说明
     *
     * @details
     * 该函数属于 ImageProcess / FixLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     */
    void FixLine::analyzeLineSimple()
    {

        if (this->left_line == nullptr)
        {
            throw "left_line is nullptr";
        }

        if (this->right_line == nullptr)
        {
            throw "right_line is nullptr";
        }

#ifdef FIXLINE_INCLUDE_ELEMENTS_ZEBRA

        if (this->element_type == -1)
        {
            if (this->checkZebraSimple())
            {
                this->element_type = FIXLINE_INCLUDE_ZEBRA_SYMBOL;
            }
        }

        if (this->element_type == FIXLINE_INCLUDE_ZEBRA_SYMBOL)
        {
            FixLineState state = this->handleZebraSimple();
            if (state == FIX_LINE_OK)
            {
                this->element_type = -1;
            }
            else if (state == FIX_LINE_STATE_BREAK)
            {
                this->element_type = -1;
            }
            else if (state == FIX_LINE_STATE_DROP)
            {
                this->element_type = -1;
            }
            else if (state == FIX_LINE_STATE_NONE)
            {
                this->element_type = -1;
            }
        }

#endif
    }

    /**
     * @brief fixLineSimple 函数说明
     *
     * @details
     * 该函数属于 ImageProcess / FixLine 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     */
    void FixLine::fixLineSimple()
    {
        /*
         * 根据前期分析阶段生成的修补信息对左右边线进行补线，
         * 然后重新计算整条中心线。
         */
        std::vector<FixInfo> center_fix_infos;
        for (const auto fixInfo : this->fix_infos)
        {
            switch (fixInfo.first)
            {
            case LEFT:
                this->drawLineSimple(this->left_line, *fixInfo.second);
                delete fixInfo.second;
                break;
            case RIGHT:
                this->drawLineSimple(this->right_line, *fixInfo.second);
                delete fixInfo.second;
                break;
            default:
                break;
            }
        }

        const Point leftStartPoint = this->left_start_pos;
        const Point leftEndPoint = this->left_end_pos;
        const Point rightStartPoint = this->right_start_pos;
        const Point rightEndPoint = this->right_end_pos;
        const int screenHeight = this->image_height;
        const int screenWidth = this->image_width;
        const int endHeight = leftEndPoint.y > rightEndPoint.y ? leftEndPoint.y : rightEndPoint.y;

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

        for (const auto fixInfo : center_fix_infos)
        {
            this->drawLineSimple(this->center_line, fixInfo);
        }

        this->fix_infos.clear();
    }

    void FixLine::setMissingEdgeProxyEnabled(bool enabled)
    {
        this->missing_edge_proxy_enabled = enabled;
    }

    /**
     * @brief 析构 FixLine 对象
     *
     * @details
     * 释放 ImageProcess / FixLine 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    FixLine::~FixLine() = default;
}
