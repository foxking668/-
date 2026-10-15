#pragma once

#include "headfile.hpp"

namespace Other
{
    /*
     * 轻量级配置文件读写类。
     *
     * 该类采用 `key = value` 的文本格式保存配置项，支持：
     * - 字符串、整数、浮点数、布尔值读取；
     * - 配置项存在性判断；
     * - 将配置写回原文件或新文件。
     *
     * 适用于本项目中运行参数、调试参数和行为开关的管理。
     */
    class Config
    {
    private:
        std::unordered_map<std::string, std::string> config;
        std::string path;

        /**
         * @brief trim 函数说明
         *
         * @details
         * 该函数属于 Other / Config 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param str 待处理的字符串。
         *
         * @return 返回 static std::string 类型结果，表示 trim 的处理结果。
         */
        static std::string trim(const std::string& str);
        /**
         * @brief toLower 函数说明
         *
         * @details
         * 该函数属于 Other / Config 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param str 待处理的字符串。
         *
         * @return 返回 static std::string 类型结果，表示 toLower 的处理结果。
         */
        static std::string toLower(const std::string& str);
        /**
         * @brief parseBool 函数说明
         *
         * @details
         * 该函数属于 Other / Config 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param str 待处理的字符串。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        static bool parseBool(const std::string& str);

    public:
        /**
         * @brief 构造 Config 对象
         *
         * @details
         * 创建 Other / Config 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         */
        Config();

        /**
         * @brief load 函数说明
         *
         * @details
         * 该函数属于 Other / Config 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param filename 配置文件路径，用于读取或保存 key=value 配置项。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool load(const std::string &filename);
        /**
         * @brief save 函数说明
         *
         * @details
         * 该函数属于 Other / Config 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool save();
        /**
         * @brief save 函数说明
         *
         * @details
         * 该函数属于 Other / Config 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param filename 配置文件路径，用于读取或保存 key=value 配置项。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool save(const std::string &filename);
        /**
         * @brief has 函数说明
         *
         * @details
         * 该函数属于 Other / Config 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param key 配置项键名。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool has(const std::string &key);
        /**
         * @brief 获取 getString 对应数据
         *
         * @details
         * 从 Other / Config 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @param key 配置项键名。
         * @param defaultValue 当配置项不存在或解析失败时返回的默认值。
         *
         * @return 返回 std::string 类型结果，表示 getString 的处理结果。
         */
        std::string getString(const std::string &key, const std::string &defaultValue = "");
        /**
         * @brief 获取 getInt 对应数据
         *
         * @details
         * 从 Other / Config 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @param key 配置项键名。
         * @param defaultValue 当配置项不存在或解析失败时返回的默认值。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int getInt(const std::string &key, int defaultValue = 0);
        /**
         * @brief 获取 getFloat 对应数据
         *
         * @details
         * 从 Other / Config 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @param key 配置项键名。
         * @param defaultValue 当配置项不存在或解析失败时返回的默认值。
         *
         * @return 返回浮点数结果，通常表示速度、角度、误差或比例计算值。
         */
        double getFloat(const std::string &key, double defaultValue = 0.0);
        /**
         * @brief 获取 getDouble 对应数据
         *
         * @details
         * 从 Other / Config 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @param key 配置项键名。
         * @param defaultValue 当配置项不存在或解析失败时返回的默认值。
         *
         * @return 返回浮点数结果，通常表示速度、角度、误差或比例计算值。
         */
        double getDouble(const std::string &key, double defaultValue = 0.0);
        /**
         * @brief 获取 getBool 对应数据
         *
         * @details
         * 从 Other / Config 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @param key 配置项键名。
         * @param defaultValue 当配置项不存在或解析失败时返回的默认值。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool getBool(const std::string &key, bool defaultValue = false);

        /**
         * @brief 设置 set 对应参数
         *
         * @details
         * 根据传入参数更新 Other / Config 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param key 配置项键名。
         * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
         */
        void set(const std::string &key, const std::string &value);
        /**
         * @brief 设置 set 对应参数
         *
         * @details
         * 根据传入参数更新 Other / Config 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param key 配置项键名。
         * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
         */
        void set(const std::string &key, int value);
        /**
         * @brief 设置 set 对应参数
         *
         * @details
         * 根据传入参数更新 Other / Config 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param key 配置项键名。
         * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
         */
        void set(const std::string &key, float value);
        /**
         * @brief 设置 set 对应参数
         *
         * @details
         * 根据传入参数更新 Other / Config 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param key 配置项键名。
         * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
         */
        void set(const std::string &key, double value);
        /**
         * @brief 设置 set 对应参数
         *
         * @details
         * 根据传入参数更新 Other / Config 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param key 配置项键名。
         * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
         */
        void set(const std::string &key, bool value);

        /**
         * @brief 析构 Config 对象
         *
         * @details
         * 释放 Other / Config 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~Config();
    };

}