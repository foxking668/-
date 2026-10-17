#include "Config.hpp"

namespace Other
{
    /* 去除字符串首尾空白字符，便于解析配置文件中的键和值。 */
    /**
     * @brief trim 函数说明
     *
     * @details
     * 该函数属于 Other / Config 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     *
     * @param str 待处理的字符串。
     *
     * @return 返回 std::string 类型结果，表示 trim 的处理结果。
     */
    std::string Config::trim(const std::string &str)
    {
        auto start = str.begin();
        while (start != str.end() && std::isspace(*start))
        {
            start++;
        }

        auto end = str.end();
        do
        {
            end--;
        } while (std::distance(start, end) > 0 && std::isspace(*end));

        return std::string(start, end + 1);
    }

    /* 将字符串统一转换为小写，主要用于布尔值和键名的无大小写解析。 */
    /**
     * @brief toLower 函数说明
     *
     * @details
     * 该函数属于 Other / Config 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     *
     * @param str 待处理的字符串。
     *
     * @return 返回 std::string 类型结果，表示 toLower 的处理结果。
     */
    std::string Config::toLower(const std::string &str)
    {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(),
                       [](unsigned char c)
                       { return std::tolower(c); });
        return result;
    }

    /*
     * 将常见布尔文本表示转换为布尔值。
     * 支持 true/false、yes/no、1/0、on/off。
     */
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
    bool Config::parseBool(const std::string &str)
    {
        std::string lower = toLower(trim(str));
        if (lower == "true" || lower == "yes" || lower == "1" || lower == "on")
        {
            return true;
        }
        if (lower == "false" || lower == "no" || lower == "0" || lower == "off")
        {
            return false;
        }
        return false;
    }

    /**
     * @brief 构造 Config 对象
     *
     * @details
     * 创建 Other / Config 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     */
    Config::Config() = default;

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
    bool Config::load(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            return false;
        }

        this->path = filename;

        std::string line;
        while (std::getline(file, line))
        {
            // 跳过空行和注释行
            line = trim(line);
            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            size_t delimiter_pos = line.find('=');
            if (delimiter_pos != std::string::npos)
            {
                std::string key = trim(line.substr(0, delimiter_pos));
                std::string value = trim(line.substr(delimiter_pos + 1));

                // 处理值中的引号
                if (value.size() >= 2 &&
                    ((value.front() == '"' && value.back() == '"') ||
                     (value.front() == '\'' && value.back() == '\'')))
                {
                    value = value.substr(1, value.size() - 2);
                }

                this->config[key] = value;
            }
        }
        return true;
    }

    /**
     * @brief save 函数说明
     *
     * @details
     * 该函数属于 Other / Config 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool Config::save()
    {
        std::ofstream file(this->path);
        if (!file.is_open())
        {
            return false;
        }

        for (const auto &pair : this->config)
        {
            file << pair.first << " = " << pair.second << "\n";
        }
        return true;
    }

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
    bool Config::save(const std::string &filename)
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            return false;
        }

        for (const auto &pair : this->config)
        {
            file << pair.first << " = " << pair.second << "\n";
        }
        return true;
    }

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
    bool Config::has(const std::string &key)
    {
        return this->config.find(key) != this->config.end();
    }

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
    std::string Config::getString(const std::string &key, const std::string &defaultValue)
    {
        auto it = this->config.find(key);
        return it != this->config.end() ? it->second : defaultValue;
    }

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
    int Config::getInt(const std::string &key, int defaultValue)
    {
        auto it = this->config.find(key);
        if (it == this->config.end())
        {
            return defaultValue;
        }

        try
        {
            return std::stoi(it->second);
        }
        catch (...)
        {
            return defaultValue;
        }
    }

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
    double Config::getFloat(const std::string &key, double defaultValue)
    {
        auto it = this->config.find(key);
        if (it == this->config.end())
        {
            return defaultValue;
        }

        try
        {
            return std::stof(it->second);
        }
        catch (...)
        {
            return defaultValue;
        }
    }

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
    double Config::getDouble(const std::string &key, double defaultValue)
    {
        auto it = this->config.find(key);
        if (it == this->config.end())
        {
            return defaultValue;
        }

        try
        {
            return std::stod(it->second);
        }
        catch (...)
        {
            return defaultValue;
        }
    }

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
    bool Config::getBool(const std::string &key, bool defaultValue)
    {
        auto it = this->config.find(key);
        if (it == this->config.end())
        {
            return defaultValue;
        }

        return parseBool(it->second);
    }

    /**
     * @brief 设置 set 对应参数
     *
     * @details
     * 根据传入参数更新 Other / Config 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param key 配置项键名。
     * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
     */
    void Config::set(const std::string &key, const std::string &value)
    {
        this->config[key] = value;
    }

    /**
     * @brief 设置 set 对应参数
     *
     * @details
     * 根据传入参数更新 Other / Config 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param key 配置项键名。
     * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
     */
    void Config::set(const std::string &key, int value)
    {
        this->config[key] = std::to_string(value);
    }

    /**
     * @brief 设置 set 对应参数
     *
     * @details
     * 根据传入参数更新 Other / Config 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param key 配置项键名。
     * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
     */
    void Config::set(const std::string &key, float value)
    {
        this->config[key] = std::to_string(value);
    }

    /**
     * @brief 设置 set 对应参数
     *
     * @details
     * 根据传入参数更新 Other / Config 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param key 配置项键名。
     * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
     */
    void Config::set(const std::string &key, double value)
    {
        this->config[key] = std::to_string(value);
    }

    /**
     * @brief 设置 set 对应参数
     *
     * @details
     * 根据传入参数更新 Other / Config 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param key 配置项键名。
     * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
     */
    void Config::set(const std::string &key, bool value)
    {
        this->config[key] = value ? "true" : "false";
    }

    /**
     * @brief 析构 Config 对象
     *
     * @details
     * 释放 Other / Config 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    Config::~Config() = default;
}