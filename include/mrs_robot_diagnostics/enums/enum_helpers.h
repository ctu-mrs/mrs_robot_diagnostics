#pragma once
#include <boost/preprocessor.hpp>

// modified from https://stackoverflow.com/questions/5093460/how-to-convert-an-enum-type-variable-to-a-string
#define X_DEFINE_ENUM_STRING_CONVERSIONS_TOSTRING_CASE(r, name, elem)         \
          case name::elem : return BOOST_PP_STRINGIZE(elem);


#define DEFINE_ENUM_WITH_CONVERSIONS(name, base_type, enumerators)            \
    enum class name : base_type                                               \
    {                                                                         \
        BOOST_PP_SEQ_ENUM(enumerators),                                       \
        UNKNOWN                                                               \
    };                                                                        \
                                                                              \
    inline constexpr const char* to_string(name v)                            \
    {                                                                         \
        switch (v)                                                            \
        {                                                                     \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_ENUM_STRING_CONVERSIONS_TOSTRING_CASE,               \
                name,                                                         \
                enumerators                                                   \
            )                                                                 \
            default: return "UNKNOWN";                                        \
        }                                                                     \
    }                                                                         

namespace enum_helpers
{

  template <typename Enum_T>
  struct enum_updater
  {
    public:
      enum_updater(const std::string_view name) : m_name(name), m_enum(Enum_T::UNKNOWN) {}

      enum_updater(const std::string_view name, const Enum_T init_value) : m_name(name), m_enum(init_value) {}

      Enum_T value() {return m_enum;}

      void set(const Enum_T new_value)
      {
        if (m_enum == new_value)
          return;

        ROS_INFO_STREAM("Changing " << m_name << ": \"" << to_string(m_enum) << "\" => \"" << to_string(new_value) << "\"");
        m_enum = new_value;
      }

      bool operator==(const Enum_T other)
      {
        return other == m_enum;
      }

      bool operator!=(const Enum_T other)
      {
        return other != m_enum;
      }

    private:
      std::string m_name;
      Enum_T m_enum;
  };

}
