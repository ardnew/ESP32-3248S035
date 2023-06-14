#ifndef debug_h
#define debug_h

#include <string>

#include <stdio.h> // snprintf

template<typename... Args>
size_t swritef(Stream &out, const char * fmt, Args... args) {
  static constexpr char eol[] = "\r\n";
  // Cannot rely on the target implementation of Stream to provide printf
  size_t size = snprintf(nullptr, 0, fmt, args...);
  std::string str;
  str.reserve(size + 1);
  str.resize(size);
  snprintf(&str[0], size + 1, fmt, args...);
  return out.write(str.c_str(), size) + out.write(eol);
}

#endif // debug_h
