find src/ include/ \
  -type f \( -iname '*.cpp' -o -iname '*.hpp' -o -iname '*.h' -o -iname '*.c' \) \
  | xargs clang-format -i
