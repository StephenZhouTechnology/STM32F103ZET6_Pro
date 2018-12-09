#ifndef __SK_COMMON_H__
#define __SK_COMMON_H__

typedef enum {
    SK_SUCCESS,
    SK_FAIL;
} SK_TYPE_e;

#define RETURN_CHECK(ret)           \
{                                   \
    if (ret != SK_SUCCESS)          \
        break;                      \
}

#endif __SK_COMMON_H__ //__SK_COMMON_H__