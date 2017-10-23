#ifndef NAMEDTYPE_H
#define NAMEDTYPE_H

#include <algorithm>
#include <type_traits>

// Named Type idiom taken from
// https://www.fluentcpp.com/2017/03/06/passing-strong-types-reference-revisited/
template <typename T, typename Parameter>
class NamedType {
  public:
  NamedType()
      : value_(T{})
  {
  }

  explicit NamedType(T const& value)
      : value_(value)
  {
  }

  template <typename T_ = T>
  explicit NamedType(T&& value,
      typename std::enable_if<!std::is_reference<T_>{},
          std::nullptr_t>::type
      = nullptr)
      : value_(std::move(value))
  {
  }

  T& get() { return value_; }
  T const& get() const { return value_; }

  private:
  T value_;
};

#endif /* NAMEDTYPE_H */
