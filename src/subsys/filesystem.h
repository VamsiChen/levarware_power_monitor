#ifndef FILESYSTEM_H_
#define FILESYSTEM_H_

/* Matches LFS_NAME_MAX */
#define MAX_PATH_LEN 255
#define MAX_FILE_LEN 255


void init_lfs(bool enable_store_forward_dir);
void deinit_lfs(void);
int save_store_forward_data(int64_t unix_timestamp, char *json_string);
bool get_next_store_forward_data(char *file_data, int max_data_len, char *file_name_out);
int delete_store_forward_data(char *path);
int move_store_forward_data(char *path);
void test_send_store_forward_data(uint16_t data_len);
int test_lfs(void);
void erase_flash(void);
void fs_basic_test(void);


/* 
*   General function of file handling
*/
bool is_file_exists(char *file_name);
int save_to_file(char *content, size_t content_len, char *file_name);
int read_file(char *output_buf, int output_buf_len, char *file_name);
int delete_file(char *file_name);
int filesystem_fat_fs_init(void);

#endif /*FILESYSTEM_H_*/