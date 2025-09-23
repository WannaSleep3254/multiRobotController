#pragma once
#include <QString>

namespace Common {

// 로그 레벨 정의
enum class LogLevel {
    Debug,
    Info,
    Warn,
    Error
};

// 문자열 변환 (선택적)
inline QString toString(LogLevel level) {
    switch (level) {
    case LogLevel::Debug: return "Debug";
    case LogLevel::Info:  return "Info";
    case LogLevel::Warn:  return "Warn";
    case LogLevel::Error: return "Error";
    }
    return "Unknown";
}

} // namespace Common
