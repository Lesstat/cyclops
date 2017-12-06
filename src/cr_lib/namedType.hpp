#ifndef NAMEDTYPE_H
#define NAMEDTYPE_H

#include <algorithm>
#include <functional>
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
  operator T() const { return value_; }

  private:
  T value_;
};

#endif /* NAMEDTYPE_H */

namespace std {

template <typename T, typename Parameter>
struct hash<NamedType<T, Parameter>> {

  //using checkIfHashable = typename std::enable_if<StrongType::is_hashable, void>::type;

  size_t operator()(const NamedType<T, Parameter>& x) const
  {
    return std::hash<T>()(x.get());
  }
};
}
