#ifndef JSON_BUILDER_H_
#define JSON_BUILDER_H_

#define COMMA 		(true)
#define NO_COMMA	(false)

char *cs_json_new(void);
void cs_json_free(char *buf);

////////////////////////////
// ITEMS API
////////////////////////////

int cs_json_add_chars(char *buf, char *chars);
int cs_json_add_item_start(char *buf, char *key, char *val);
int cs_json_add_item_close(char *buf, char *val);
int cs_json_add_item_string(char *buf, char *key, char *val, bool comma);
int cs_json_add_item_double(char *buf, char *key, double val, bool comma);
int cs_json_add_item_int(char *buf, char *key, int32_t val, bool comma);
int cs_json_add_item_bool(char *buf, char *key, bool val, bool comma);
int cs_json_add_item_int64(char *buf, char *key, int64_t val, bool comma);
int cs_json_add_item_float_array(char *buf, char *key, float *val_arr, int arr_len, bool comma);
int cs_json_add_item_int_array(char *buf, char *key, int *val_arr, int arr_len, bool comma);

#endif /*JSON_BUILDER_H_*/