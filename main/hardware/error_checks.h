#pragma once

#define ESP_ERROR_CHECK_RETURN_FALSE(x) ({                                     \
        esp_err_t err_rc_ = (x);                                               \
        if (unlikely(err_rc_ != ESP_OK)) {                                     \
            _esp_error_check_failed_without_abort(err_rc_, __FILE__, __LINE__, \
                                                  __ASSERT_FUNC, #x);          \
            return false;                                                      \
        }                                                                      \
    })

#define ERROR_CHECK_RETURN_NULL(x)  do {                             \
        esp_err_t err_rc_ = (x);                                      \
        if (unlikely(err_rc_ != ESP_OK)) {                          \
            ESP_LOGW(TAG,"error %d in line %d" ,(int)err_rc_, __LINE__);  \
            return NULL;                                              \
        }                                                             \
    } while(0)

#define IF_NULL_RETURN_FALSE(x)  ({                             \
        if (unlikely((x) == NULL)) {                            \
            ESP_LOGW(TAG, #x "==NULL in line %d" , __LINE__);   \
            return false;                                       \
        }                                                       \
    })
