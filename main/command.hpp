#pragma once

#include <cstdint>

#include "format.hpp"

enum class CommandCode : uint8_t {
  NONE = 0,  // expects 0 data
  STOP,      // expects 0 data
  SET_ANGLE, // expects 1 float in radians
  SET_SPEED, // expects 1 float in radians per second
};

struct Command {
  CommandCode code{CommandCode::NONE};
  union {
    float angle_radians;
    float speed_radians_per_second;
    uint8_t data[4] = {0};
  } __attribute__((packed));

  Command() = default;

  static constexpr float MIN_ANGLE_DIFF = 0.1f;
  static constexpr float MIN_SPEED_DIFF = 0.1f;

  bool operator==(const Command &other) const {
    if (code != other.code) {
      return false;
    }
    switch (code) {
    case CommandCode::NONE:
    case CommandCode::STOP:
      return true;
    case CommandCode::SET_ANGLE:
      return fabs(angle_radians - other.angle_radians) < MIN_ANGLE_DIFF;
    case CommandCode::SET_SPEED:
      return fabs(speed_radians_per_second - other.speed_radians_per_second) < MIN_SPEED_DIFF;
    }
    return false;
  }

  size_t size() const {
    switch (code) {
    case CommandCode::NONE:
    case CommandCode::STOP:
      return 1;
    case CommandCode::SET_ANGLE:
      return 5;
    case CommandCode::SET_SPEED:
      return 5;
    }
    return 0;
  }

} __attribute__((packed));

// for libfmt printing of commandcode
template <> struct fmt::formatter<CommandCode> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(CommandCode c, FormatContext &ctx) const {
    std::string name;
    switch (c) {
    case CommandCode::NONE:
      name = "NONE";
      break;
    case CommandCode::STOP:
      name = "STOP";
      break;
    case CommandCode::SET_ANGLE:
      name = "SET_ANGLE";
      break;
    case CommandCode::SET_SPEED:
      name = "SET_SPEED";
      break;
    }
    return fmt::formatter<std::string>::format(name, ctx);
  }
};

// for libfmt printing of command
template <> struct fmt::formatter<Command> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(const Command &c, FormatContext &ctx) const {
    std::string str;
    switch (c.code) {
    case CommandCode::NONE:
      str = "Command{code=NONE}";
      break;
    case CommandCode::STOP:
      str = "Command{code=STOP}";
      break;
    case CommandCode::SET_ANGLE:
      str = fmt::format("Command{{code=SET_ANGLE, angle_radians={}}}", c.angle_radians);
      break;
    case CommandCode::SET_SPEED:
      str = fmt::format("Command{{code=SET_SPEED, speed_radians_per_second={}}}",
                        c.speed_radians_per_second);
      break;
    }
    return fmt::formatter<std::string>::format(str, ctx);
  }
};
