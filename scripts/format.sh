find src/ \
  -type f \( -iname '*.cpp' -o -iname '*.ixx' \) \
  | xargs clang-format -i
