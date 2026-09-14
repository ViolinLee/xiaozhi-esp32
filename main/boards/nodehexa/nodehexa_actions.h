#pragma once

#include <cctype>
#include <string>

// Canonical names match NodeHexa's JSON motion API on both UART envelopes.
namespace nodehexa_actions {
inline std::string Normalize(const std::string& input) {
    std::string token;
    for (unsigned char c : input) {
        if (c == '_' || c == '-' || c == ' ')
            continue;
        token += c < 128 ? static_cast<char>(std::tolower(c)) : static_cast<char>(c);
    }
    return token;
}

inline std::string Motion(const std::string& input) {
    const auto token = Normalize(input);
    struct Alias {
        const char* name;
        const char* chinese;
        const char* extra;
    };
    static const Alias aliases[] = {
        {"forward", "前进", "向前走"},         {"backward", "后退", "向后走"},
        {"turnleft", "左转", "向左转"},        {"turnright", "右转", "向右转"},
        {"shiftleft", "左移", "向左平移"},     {"shiftright", "右移", "向右平移"},
        {"forwardfast", "快速前进", "迈大步"}, {"climb", "攀爬", "抬高腿"},
        {"rotatex", "摇头", "摇一下头"},       {"rotatey", "耸肩", "耸一下肩"},
        {"rotatez", "扭身体", "扭一下身体"},   {"twist", "扭屁股", "扭一下屁股"},
        {"standby", "停止", "停下"},
    };
    if (token == "stop" || token == "待机")
        return "standby";
    for (const auto& alias : aliases)
        if (token == alias.name || token == alias.chinese || token == alias.extra)
            return alias.name;
    return "";
}

inline std::string Performance(const std::string& input) {
    const auto token = Normalize(input);
    if (token == "showtime" || token == "登场秀" || token == "登场")
        return "showtime";
    if (token == "freestyle" || token == "自由舞" || token == "跳舞")
        return "freestyle";
    if (token == "beatsway" || token == "律动" || token == "节奏摇摆")
        return "beatsway";
    return "";
}
}  // namespace nodehexa_actions
