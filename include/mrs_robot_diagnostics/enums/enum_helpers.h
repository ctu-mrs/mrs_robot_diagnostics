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
