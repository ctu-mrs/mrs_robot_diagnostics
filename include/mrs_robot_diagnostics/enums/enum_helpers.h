#pragma once
#include <boost/preprocessor.hpp>
#include <string>

// modified from https://stackoverflow.com/questions/5093460/how-to-convert-an-enum-type-variable-to-a-string
// a helper macro to generate a switch case in the format
// case enum_t::element_name: return "ELEMENT_NAME";
#define X_DEFINE_ENUM_STRING_CONVERSIONS_TOSTRING_CASE(r, enum_t, elem)       \
          case enum_t::elem: return BOOST_PP_STRINGIZE(elem);

// this macro defines an enumerator using the given name, base type and enumeration values
// together with a to_string() conversion function
#define DEFINE_ENUM_WITH_CONVERSIONS(enum_t, base_type, enumerators)          \
    enum class enum_t : base_type                                             \
    {                                                                         \
        BOOST_PP_SEQ_ENUM(enumerators),                                       \
        UNKNOWN                                                               \
    };                                                                        \
                                                                              \
    inline constexpr const char* to_string(enum_t v)                          \
    {                                                                         \
        switch (v)                                                            \
        {                                                                     \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_ENUM_STRING_CONVERSIONS_TOSTRING_CASE,               \
                enum_t,                                                       \
                enumerators                                                   \
            )                                                                 \
            default: return "UNKNOWN";                                        \
        }                                                                     \
    }                                                                         

// some helper macros for expanding and concatenating macro variables using black magic
#define X_PPCAT_NX(A, B) A ## B
#define X_PPCAT(A, B) X_PPCAT_NX(A, B)
// a helper macro to generate a switch case in the format
// case enum_t::element_name: return msg_t::PREFIX_ELEMENT_NAME;
#define X_DEFINE_ENUM_MSG_CONVERSIONS_TOMSG_CASE(r, data, elem)               \
          case BOOST_PP_TUPLE_ELEM(3, 0, data)::elem: return BOOST_PP_TUPLE_ELEM(3, 1, data)::X_PPCAT(BOOST_PP_TUPLE_ELEM(3, 2, data), elem);

// this macro defines a to_ros() enum to ROS message conversion function given the enumeration type,
// ROS message type, name of the message element, prefix of the message elements corresponding to
// the enumeration values, and a list of the enumeration values
#define DEFINE_ENUM_MSG_CONVERSIONS(enum_t, msg_t, elem_name, elem_pre, enumerators)    \
                                                                              \
    inline constexpr decltype(msg_t::elem_name) to_ros(enum_t v)              \
    {                                                                         \
        switch (v)                                                            \
        {                                                                     \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_ENUM_MSG_CONVERSIONS_TOMSG_CASE,                     \
                (                                                             \
                 enum_t,                                                      \
                 msg_t,                                                       \
                 elem_pre\
                 ),                                                           \
                enumerators                                                   \
            )                                                                 \
          default: return msg_t::X_PPCAT(elem_pre, UNKNOWN);                  \
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
