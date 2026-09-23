#ifndef TOOLS__COMMAND_LINE_HPP
#define TOOLS__COMMAND_LINE_HPP

#include <string>

namespace tools
{
// OpenCV 的 CommandLineParser 只接受 --key=value；写成 --key value 时该开关的值会变成字符串
// "true"（按数值读回是 0），而且后面那个值会被当成位置参数吃掉（例如改写 config-path）。
// 传入 cli.get<std::string>(key) 得到的原始值即可判断这种写法；必须在任何 get<int>/get<double>
// 之前调用，类型化读取会破坏这个信息。
inline bool cli_value_flag_misused(const std::string & raw_value) { return raw_value == "true"; }
}  // namespace tools

#endif  // TOOLS__COMMAND_LINE_HPP
