#ifdef __cplusplus
extern "C" {
#endif

uint8_t cfg_nvs_get_u8_def(const char *key, uint8_t def);

void cfg_nvs_set_u8(const char *key, uint8_t v);

uint16_t cfg_nvs_get_u16_def(const char *key, uint16_t def);

void cfg_nvs_set_u16(const char *key, uint16_t v);

#ifdef __cplusplus
}
#endif