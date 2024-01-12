#include <zephyr.h>

#include <string.h>
#include <stdio.h>

/*Logging*/
#include <logging/log.h>
LOG_MODULE_REGISTER(cs_json);
// LOG_LEVEL_SET(LOG_LEVEL_INF);

#include "cs_json.h"


/**
 * @brief @private main writer to json_payload buffer
 * @note no comma handling is performed, commas must be included in *val if required
 * 
 * @param key key name 
 * @param val value in formatted string
 * @return int 0 if successful, -ENOMEM if val can't fit in buffer
 */
static int cs_json_write_key_val(char *json_buffer, char *key, char *val)
{
    int current_json_len = strlen(json_buffer);
    int concat_len = strlen(val);
    
    // count open/close chars for key: "<key>":
    if (key)
    {
        concat_len += strlen("\"") + strlen(key) + strlen("\":"); 
    }
    
    // -2 for closing brace plus null
    if (current_json_len + concat_len > CONFIG_TELEMETRY_PAYLOAD_BUFFER_SIZE - 2)
    {
        LOG_ERR("new item exceeds json string buffer size - discarding");
        return -ENOMEM;
    } 

    if (key)
    {
        strcat(json_buffer, "\"");
        strcat(json_buffer, key);
        strcat(json_buffer, "\":");   
    }

    if (val)
    {
        strcat(json_buffer, val);
    }

    return 0;
}

/**
 * @brief Starts formatting string to JSON. Caller must also call cs_json_free(buf) when done
 * 
 * @return Returns 0 if it is a success
 */
char *cs_json_new(void)
{
    // malloc here to ensure json_str size checks are using the same size
    char *buf = k_malloc(CONFIG_TELEMETRY_PAYLOAD_BUFFER_SIZE);
    
    // clear json buffer
    memset(buf, 0, CONFIG_TELEMETRY_PAYLOAD_BUFFER_SIZE);
    
    return buf;
}

void cs_json_free(char *buf)
{
    k_free(buf);
}

////////////////////////////
// ITEMS API
////////////////////////////

/**
 * @brief helper to add char sequence (aka: string), use this to add one of more chars to buf.
 * e.g. root open brace, commas, closing braces, brackets, etc. 
 * 
 * @param buf k_malloc'd char buffer
 * @param seq sequence of characters to add buf
 * @return int 0 if success, -ENOMEM if buf would overflow
 */
int cs_json_add_chars(char *buf, char *chars)
{
    return cs_json_write_key_val(buf, NULL, chars);
}

/**
 * @brief add start of an item container as defined by value of *val
 * 
 * @param key 
 * @param val 
 * @return int 
 */
int cs_json_add_item_start(char *buf, char *key, char *val)
{
    return cs_json_write_key_val(buf, key, val);
}

/**
 * @brief Updates the buffer and formats the key and value to JSON
 * 
 * @param key key of the object
 * @param val value of the object
 * 
 * @return Returns 0 if it is a success
 */
int cs_json_add_item_string(char *buf, char *key, char *val, bool comma)
{
    int err = 0;
    
    err = cs_json_add_item_start(buf, key, "\"");
    err = cs_json_write_key_val(buf, NULL, val);
    err = cs_json_add_chars(buf, comma ? "\"," : "\"");

    return err;
}

/**
 * @brief Updates the buffer and formats the key and value to JSON
 * 
 * @param key key of the object
 * @param val value of the object
 * 
 * @return Returns 0 if it is a success
 */
int cs_json_add_item_double(char *buf, char *key, double val, bool comma)
{
    char temp[64] = { 0 };

    snprintk(temp, sizeof(temp), comma ? "%.3f," : "%.3f", val);
    return cs_json_write_key_val(buf, key, temp);
}

/**
 * @brief Updates the buffer and formats the key and value to JSON
 * 
 * @param key key of the object
 * @param val value of the object
 * 
 * @return Returns 0 if it is a success
 */
int cs_json_add_item_int(char *buf, char *key, int32_t val, bool comma)
{
    char temp[64] = { 0 };

    snprintk(temp, sizeof(temp), comma ? "%i," : "%i", val);
    return cs_json_write_key_val(buf, key, temp);
}

/**
 * @brief Updates the buffer and formats the key and value to JSON
 * 
 * @param key key of the object
 * @param val value of the object
 * 
 * @return Returns 0 if it is a success
 */
int cs_json_add_item_bool(char *buf, char *key, bool val, bool comma)
{
    if (comma)
    {
        return cs_json_write_key_val(buf, key, val ? "true," : "false,");
    }

    return cs_json_write_key_val(buf, key, val ? "true" : "false");
}

/**
 * @brief Updates the buffer and formats the key and value to JSON
 * 
 * @param key key of the object
 * @param val value of the object
 * 
 * @return Returns 0 if it is a success
 */
int cs_json_add_item_int64(char *buf, char *key, int64_t val, bool comma)
{
    char temp[64] = { 0 };

    snprintk(temp, sizeof(temp), comma ? "%lld," : "%lld", val);
    return cs_json_write_key_val(buf, key, temp);
}

int cs_json_add_item_float_array(char *buf, char *key, float *val_arr, int arr_len, bool comma)
{
    int err = 0;
    char temp[64];

    cs_json_add_item_start(buf, key, "[");

    for (int i = 0; i < arr_len; i++)
    {
        // no trailing comma on last element
        snprintk(temp, sizeof(temp), (i + 1 < arr_len) ? "%.3f," : "%.3f", val_arr[i]);
        err = cs_json_write_key_val(buf, NULL, temp);

        if (err)
        {
            return err;
        }
    }

    return cs_json_add_chars(buf, comma ? "]," : "]");
}

int cs_json_add_item_int_array(char *buf, char *key, int *val_arr, int arr_len, bool comma)
{
    int err = 0;
    char temp[64];

    cs_json_add_item_start(buf, key, "[");

    for (int i = 0; i < arr_len; i++)
    {
        // no trailing comma on last element
        snprintk(temp, sizeof(temp), (i + 1 < arr_len) ? "%i," : "%i", val_arr[i]);
        err = cs_json_write_key_val(buf, NULL, temp);

        if (err)
        {
            return err;
        }
    }

    return cs_json_add_chars(buf, comma ? "]," : "]");
}
