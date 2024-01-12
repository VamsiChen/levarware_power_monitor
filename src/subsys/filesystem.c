/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Sample which uses the filesystem API with littlefs */

#include <stdio.h>

#include <zephyr.h>
#include <device.h>
#include <fs/fs.h>
#include <fs/littlefs.h>
#include <storage/flash_map.h>
#include <sys/time.h>
#include <disk/disk_access.h>
// #include <ff.h>

#include "filesystem.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(filesystem);

#define WIPE_STORAGE        0
#define STORE_FORWARD_DIR   "sfdata"
#define STORE_FORWARD_DIR_AWS "sfdata_aws"

// A change of the name of the littlefs storage symbol
// This is to avoid compiler error on flash_map_pm.h, line#29
// A similar issue is post here: 
// https://devzone.nordicsemi.com/f/nordic-q-a/74859/not-able-to-combine-openthread-mcuboot-and-littlefs/308671
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_storage_mnt = {
    .type = FS_LITTLEFS,
    .fs_data = &storage,
    .storage_dev = (void *)FLASH_AREA_ID(littlefs_storage),
    .mnt_point = "/lfs",
};

// mp pointer is used as a mount flag
static struct fs_mount_t *mp = NULL;

// static FATFS fat_fs;

/* mounting info */
// static struct fs_mount_t mp_fat = {
// 	.type = FS_FATFS,
// 	.fs_data = &fat_fs,
// };

// static const char *disk_mount_pt = "/SD:"; 

/************************/
/*Function Prototype*/
int create_store_forward_dir(void);

int unmount_lfs(void)
{
    int rc = fs_unmount(mp);
    if (rc < 0)
    {
        LOG_ERR("FAIL: unmount id %u at %s: %d",
                (unsigned int)mp->storage_dev, log_strdup(mp->mnt_point),
                rc);
        return rc;
    }

    mp = NULL;

    return 0;
}

int mount_lfs(void)
{
    if (mp == NULL)
    {
        mp = &lfs_storage_mnt;
    }  

    int rc = fs_mount(mp);
    if (rc < 0)
    {
        LOG_ERR("FAIL: mount id %u at %s: %d", (unsigned int)mp->storage_dev, log_strdup(mp->mnt_point), rc);
        mp = NULL;
        return rc;
    }

    return 0;
}

void init_lfs(bool enable_store_forward_dir)
{
    int rc = 0;

    rc = mount_lfs();
    
    if (rc != 0)
    {
        LOG_ERR("Unable to mount the id %u at %s: %d", (unsigned int)mp->storage_dev, mp->mnt_point, rc);
    }

 /* The following Store and  Forward code hasn't been  completed and generates errors. In addition, developers need to be careful about 
 *  implementing Store and Forward that  continouslys writes to Flash as flash wear can occur (even with wear leveling). Calculations need to be 
 * done  to ensure the write frequency is low enough to give enough hardware lifetime.
 *
 *  so we will disable this code for now
 */
/* 
if (enable_store_forward_dir)
    {
        create_store_forward_dir();
    }
 */   
}



void deinit_lfs(void)
{
    unmount_lfs();
}

int create_store_forward_dir(void)
{
    int rc = 0;
    char dname[MAX_PATH_LEN];

    // create data directory if it doesn't exist
    LOG_INF("Creating data directory...");

    snprintk(dname, sizeof(dname), "%s/%s", mp->mnt_point, STORE_FORWARD_DIR);

    rc = fs_mkdir(dname);
    if (rc && rc != -EEXIST)
    {
        LOG_ERR("mkdir(%s) error -> rc=%d", log_strdup(dname), rc);
        return rc;
    }
    LOG_INF("mkdir(%s) SUCCESS -> rc=%d", log_strdup(dname), rc);

    return rc;
}

/* 
 * store-forward data will be located at /<mp->mnt_point>/<STORE_FORWARD_DIR>/<timestamp>.json where each json file will be one data record
 * save_store_forward_data creates a new file with a name of the unix timestamp of the data
 * then write the json_string parameter to the file as-is 
 */
int save_store_forward_data(int64_t unix_timestamp_s, char *json_string)
{
    char fname[MAX_PATH_LEN];
    int rc = 0;

    /* Convert time to make string for file name */
    time_t now = (time_t)unix_timestamp_s;
    char time_str[sizeof("19700101_000000")];
    struct tm now_tm;

    gmtime_r(&now, &now_tm);
    strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &now_tm);

    if (strlen(STORE_FORWARD_DIR) == 0)
    {
        snprintk(fname, sizeof(fname), "%s/%s.json", mp->mnt_point, time_str);
    }
    else
    {
        snprintk(fname, sizeof(fname), "%s/%s/%s.json", mp->mnt_point, STORE_FORWARD_DIR, time_str);
    }

    struct fs_file_t file;

    fs_file_t_init(&file);

    rc = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
    if (rc < 0)
    {
        LOG_ERR("FAIL: open %s: %d", log_strdup(fname), rc);
        goto end;
    }

    LOG_INF("opened %s: rc=%d", log_strdup(fname), rc);

    rc = fs_write(&file, json_string, strlen(json_string));
    if (rc >= 0)
    {
        LOG_INF("%s written: %s, bytes written=%d", log_strdup(fname), log_strdup(json_string), rc);
    }
    else
    {
        LOG_ERR("FAIL: write %s: %d", log_strdup(fname), rc);
    }

    rc = fs_close(&file);
    LOG_INF("%s closed: rc=%d", log_strdup(fname), rc);

end:
    return rc;
}

int delete_store_forward_data(char *path)
{
    int rc = 0;

    rc = fs_unlink(path);
    if (rc < 0)
    {
        LOG_ERR("FAIL: unlink %s: %d", log_strdup(path), rc);
    }
    else
    {
        LOG_INF("deleted %s", log_strdup(path));
    }

    return rc;
}

// int move_store_forward_data(char *path)
// {
//     char fname[MAX_PATH_LEN];
//     int rc = 0;

//     char *file_name;
//     file_name = strtok("/lfs/sfdata/20210621_155241.json", "/");
//     while (file_name != NULL)
//     {
//         file_name = strtok (NULL, "/");
//     }

//     snprintk(fname, sizeof(fname), "%s/%s/%s.json", mp->mnt_point, STORE_FORWARD_DIR_AWS, file_name);
//     LOG_INF("new path: %s", log_strdup(fname));

//     return rc;
// }

/* 
 * store-forward data will be located at /lfs/sf/<YYYYmmDD_HHMMSS>.json where each jsone file will be one data record
 * get_store_forward_data will read the newest file (sorted by name desc) into a buffer and 
 * returning buffer pointer
 */
bool get_next_store_forward_data(char *file_data, int max_data_len, char *file_name_out)
{
    int rc = 0;
    char path[MAX_PATH_LEN];
    char fname[MAX_PATH_LEN];
    struct fs_dirent ent = {0};
    struct fs_dir_t dir = {0};
    bool end_of_files = false;

    snprintk(path, sizeof(path), "%s/%s", mp->mnt_point, STORE_FORWARD_DIR);


    rc = fs_opendir(&dir, path);
    if (rc < 0)
    {
        LOG_ERR("FAIL: opendir %s : %d", log_strdup(path), rc);
    }

    rc = fs_readdir(&dir, &ent);
    if (rc < 0)
    {
        LOG_ERR("FAIL: readdir %s: %d", log_strdup(path), rc);

    }

    if (ent.name[0] == 0)
    {
        end_of_files = true;
        LOG_INF("End of files");
    }
    else
    { 
        struct fs_file_t file;

        fs_file_t_init(&file);

        snprintk(fname, sizeof(fname), "%s/%s", path, ent.name);
        strcpy(file_name_out, fname);
        rc = fs_open(&file, fname, FS_O_READ);
        if (rc < 0)
        {
            LOG_ERR("FAIL: open %s: %d", log_strdup(ent.name), rc);
        }
        else
        {
            fs_read(&file, file_data, MIN(ent.size, max_data_len - 1));

            file_data[MIN(ent.size, max_data_len - 1)] = '\0';
            
            fs_close(&file);
        }
    }

    fs_closedir(&dir);

    return end_of_files;
}

void test_send_store_forward_data(uint16_t data_len)
{
    LOG_INF("Testing send of store forward data...");

    // get sensor data and publish to server
    char publish_data[data_len];
    int data_to_send = true;
    char store_forward_fname[MAX_PATH_LEN] = "";

    while (data_to_send == true)
    {
        // see if there is any store-forward data and assign it to `publish_data` for next loop
        bool end_of_files = get_next_store_forward_data(publish_data, data_len, store_forward_fname);
        // if end of files, no more data to send
        data_to_send = !end_of_files;

        if (data_to_send)
        {
            // assume a real network publish works
            LOG_INF("Data to publish: %s", log_strdup(publish_data));

            // message publish success
            if (strlen(store_forward_fname) != 0)
            {
                LOG_INF("Deleting SF data file...");
                // there's a file to delete
                int err = delete_store_forward_data(store_forward_fname);
                if (err != 0)
                {
                    LOG_ERR("TEST_DATA_SEND: delete_store_forward_data() failed: err = %d", err);

                }
            }
        }
    }

}


int dir_ls(struct fs_dir_t *dir, char *dir_name, bool cat_files, bool delete_files)
{
    int rc = 0;
    char file_data[255];
    char fname[MAX_PATH_LEN];

    while (rc >= 0)
    {
        struct fs_dirent ent = {0};

        rc = fs_readdir(dir, &ent);
        if (rc < 0)
        {
            break;
        }
        if (ent.name[0] == 0)
        {
            LOG_INF("End of files");
            break;
        }
        LOG_INF("  %c %u %s",
                (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
                ent.size,
                log_strdup(ent.name));

        if (cat_files && ent.type == FS_DIR_ENTRY_FILE)
        {
            // print file contents
            struct fs_file_t file;

            fs_file_t_init(&file);

            snprintk(fname, sizeof(fname), "%s/%s", dir_name, ent.name);
            rc = fs_open(&file, fname, FS_O_READ);
            if (rc < 0)
            {
                LOG_ERR("FAIL: open %s: %d", log_strdup(ent.name), rc);
            }
            else
            {
                fs_read(&file, file_data, sizeof(file_data));
                LOG_INF("    %s", log_strdup(file_data));
                fs_close(&file);
            }
        }

        if (delete_files && ent.type == FS_DIR_ENTRY_FILE)
        {
            snprintk(fname, sizeof(fname), "%s/%s", dir_name, ent.name);
            rc = fs_unlink(fname);
            if (rc < 0)
            {
                LOG_ERR("FAIL: unlink %s: %d", log_strdup(fname), rc);
            }
            else
            {
                LOG_INF("deleted %s", log_strdup(fname));
            }
        }
    }

    return rc;
}

int test_lfs(void)
{
    struct fs_statvfs sbuf;
    int rc = 0;

    rc = fs_statvfs(mp->mnt_point, &sbuf);
    if (rc < 0)
    {
        LOG_ERR("FAIL: statvfs: %d", rc);
        goto end;
    }

    LOG_INF("%s: bsize = %lu ; frsize = %lu ;"
            " blocks = %lu ; bfree = %lu",
            log_strdup(mp->mnt_point),
            sbuf.f_bsize, sbuf.f_frsize,
            sbuf.f_blocks, sbuf.f_bfree);

    struct fs_dir_t dir = {0};
    rc = fs_opendir(&dir, mp->mnt_point);
    LOG_INF("%s opendir: %d", log_strdup(mp->mnt_point), rc);
    dir_ls(&dir, (char *)mp->mnt_point, true, false);
    (void)fs_closedir(&dir);

    if (strlen(STORE_FORWARD_DIR) > 0)
    {
        char dname[MAX_PATH_LEN];
        snprintk(dname, sizeof(dname), "%s/%s", mp->mnt_point, STORE_FORWARD_DIR);
        rc = fs_opendir(&dir, dname);
        LOG_INF("%s opendir: %d", log_strdup(dname), rc);
        dir_ls(&dir, dname, true, false);
        (void)fs_closedir(&dir);
    }

end:
    return rc;
}

void erase_flash(void)
{
    unsigned int id = (uintptr_t)mp->storage_dev;
    const struct flash_area *pfa;

    int rc = flash_area_open(id, &pfa);
    if (rc < 0)
    {
        LOG_ERR("FAIL: unable to find flash area %u: %d\n",
                id, rc);
        return;
    }

    LOG_INF("Area %u at 0x%x on %s for %u bytes\n",
            id, (unsigned int)pfa->fa_off, log_strdup(pfa->fa_dev_name),
            (unsigned int)pfa->fa_size);

    /* Optional wipe flash contents */
    if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE))
    {
        LOG_INF("Erasing flash area ... ");
        rc = flash_area_erase(pfa, 0, pfa->fa_size);
        LOG_INF("%d\n", rc);
    }

    flash_area_close(pfa);
}

bool is_file_exists(char *file_name)
{
    char fname[MAX_PATH_LEN];
    int rc;
    bool retval = false;

    snprintk(fname, sizeof(fname), "%s/%s", mp->mnt_point, file_name);

    struct fs_dirent config_ent;
    rc = fs_stat(fname, &config_ent);
    if (rc == 0)
    {
        // LOG_INF("fs_stat %s: rc=%d", log_strdup(fname), rc);
        // file exists if size > 0
        retval = config_ent.size;
    }
    else if (rc == -2)
    {
        //return 0 if no config file exist
        LOG_INF("FILE NOT EXIST: %s: %d", log_strdup(fname), rc);
    }
    else
    {
        LOG_ERR("CONFIG FILE FAIL: fs_stat %s, rc: %d", log_strdup(fname), rc);
    }

    return retval;
}

int save_to_file(char *content, size_t content_len, char *file_name)
{
    char fname[MAX_PATH_LEN];
    int rc = 0;

    snprintk(fname, sizeof(fname), "%s/%s", mp->mnt_point, file_name);

    struct fs_file_t file;

    fs_file_t_init(&file);

    rc = fs_open(&file, fname, FS_O_CREATE | FS_O_WRITE);
    if (rc < 0)
    {
        LOG_ERR("FAIL: open %s: %d", log_strdup(fname), rc);
        goto end;
    }

    // LOG_INF("opened %s: rc=%d", log_strdup(fname), rc);

    // this is to check, if the written buffer is larger or shorter than the previous file size
    // if not, then truncate or expend to the new size
    off_t file_len = -1;
    struct fs_dirent file_ent;
    rc = fs_stat(fname, &file_ent);
    if (rc == 0)
    {
        // LOG_INF("fs_stat %s: rc=%d", log_strdup(fname), rc);
        // file exists if size > 0
        file_len = file_ent.size;
    }

    if (file_len != content_len)
    {
        rc = fs_truncate(&file, content_len);
        if (rc)
        {
            LOG_ERR("FAIL: truncate %s: %d", log_strdup(fname), rc);
        }
    }

    rc = fs_write(&file, content, content_len);
    if (rc >= 0)
    {
        // LOG_INF("%s written: bytes written=%d", log_strdup(fname), rc);
    }
    else
    {
        LOG_ERR("FAIL: write %s: %d", log_strdup(fname), rc);
    }

    fs_close(&file);
    // LOG_INF("%s closed: rc=%d", log_strdup(fname), rc);

end:
    //unmount_lfs();
    return rc;
}

int read_file(char *output_buf, int output_buf_len, char *file_name)
{
    char fname[MAX_PATH_LEN];
    int rc = 0;

    snprintk(fname, sizeof(fname), "%s/%s", mp->mnt_point, file_name);

    struct fs_file_t file;

    fs_file_t_init(&file);

    rc = fs_open(&file, fname, FS_O_READ);
    if (rc < 0)
    {
        LOG_ERR("FAIL: open %s: %d", log_strdup(fname), rc);
        goto end;
    }

    // check size first
    struct fs_dirent file_ent;
    rc = fs_stat(fname, &file_ent);
    if (rc == 0)
    {
        // LOG_INF("FILESYSTEM: %s size: %d", log_strdup(fname), file_ent.size);
    }

    rc = fs_read(&file, output_buf, output_buf_len);
    if (rc >= 0)
    {
        // LOG_INF("%s read: bytes read=%d", log_strdup(fname), rc);
    }
    else
    {
        LOG_ERR("FAIL: read %s: %d", log_strdup(fname), rc);
    }

    fs_close(&file);

end:
    //unmount_lfs();
    return rc;
}

int delete_file(char *file_name)
{
    char fname[MAX_PATH_LEN];
    int rc = 0;

    snprintk(fname, sizeof(fname), "%s/%s", mp->mnt_point, file_name);

    // First check if file existed
    rc = is_file_exists(file_name);
    if (rc == 0)
    {
        // if not existed, exit function
        goto end;
    }

    rc = fs_unlink(fname);
    if (rc != 0)
    {
        LOG_ERR("FAIL: delete %s: %d", log_strdup(fname), rc);
    }
    else
    {
        LOG_INF("delete %s: rc=%d", log_strdup(fname), rc);
    }

end:
    //unmount_lfs();
    return rc;
}

void fs_basic_test()
{
	mp = &lfs_storage_mnt;
	unsigned int id = (uintptr_t)mp->storage_dev;
	char fname[MAX_PATH_LEN];
	struct fs_statvfs sbuf;
	const struct flash_area *pfa;
	int rc;

	snprintf(fname, sizeof(fname), "%s/boot_count", mp->mnt_point);

	rc = flash_area_open(id, &pfa);
	if (rc < 0)
	{
		LOG_ERR("FAIL: unable to find flash area %u: %d",
			   id, rc);
		return;
	}

	LOG_INF("Area %u at 0x%x on %s for %u bytes\n",
		   id, (unsigned int)pfa->fa_off, log_strdup(pfa->fa_dev_name),
		   (unsigned int)pfa->fa_size);

	/* Optional wipe flash contents */
	if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE))
	{
		LOG_DBG("Erasing flash area ... ");
		rc = flash_area_erase(pfa, 0, pfa->fa_size);
		LOG_DBG("%d", rc);
	}

	flash_area_close(pfa);

    init_lfs(false);
    
	LOG_INF("%s mount: %d\n", log_strdup(mp->mnt_point), rc);

	rc = fs_statvfs(mp->mnt_point, &sbuf);
	if (rc < 0)
	{
		LOG_ERR("FAIL: statvfs: %d", rc);
		goto out;
	}

	LOG_INF("%s: bsize = %lu ; frsize = %lu ;"
		   " blocks = %lu ; bfree = %lu",
		   log_strdup(mp->mnt_point),
		   sbuf.f_bsize, sbuf.f_frsize,
		   sbuf.f_blocks, sbuf.f_bfree);

	struct fs_dirent dirent;

	rc = fs_stat(fname, &dirent);
	LOG_INF("%s stat: %d", log_strdup(fname), rc);
	if (rc >= 0)
	{
		LOG_ERR("fn '%s' siz %u", log_strdup(dirent.name), dirent.size);
	}

	struct fs_file_t file;

    fs_file_t_init(&file);

	rc = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
	if (rc < 0)
	{
		LOG_ERR("FAIL: open %s: %d", log_strdup(fname), rc);
		goto out;
	}

	uint32_t boot_count = 0;

	if (rc >= 0)
	{
		rc = fs_read(&file, &boot_count, sizeof(boot_count));
		LOG_DBG("%s read count %u: %d", log_strdup(fname), boot_count, rc);
		rc = fs_seek(&file, 0, FS_SEEK_SET);
		LOG_DBG("%s seek start: %d", log_strdup(fname), rc);
	}

	boot_count += 1;
	rc = fs_write(&file, &boot_count, sizeof(boot_count));
	LOG_INF("%s write new boot count %u: %d", log_strdup(fname),
		   boot_count, rc);

	rc = fs_close(&file);
	LOG_INF("%s close: %d", log_strdup(fname), rc);

out:
	rc = fs_unmount(mp);
	LOG_INF("%s unmount: %d", log_strdup(mp->mnt_point), rc);

}


// int filesystem_fat_fs_init(void)
// {
//     int res;
// 	static const char *disk_pdrv = "SD";
// 	uint64_t memory_size_mb;
// 	uint32_t block_count;
// 	uint32_t block_size;

// 	LOG_INF("Filesystem init...");
// 	if (disk_access_init(disk_pdrv) != 0) {
// 		LOG_ERR("FAT init ERROR!");
// 		return -1;
// 	}

// 	if (disk_access_ioctl(disk_pdrv,
// 			DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
// 		LOG_ERR("Unable to get SD sector count");
// 		return -1;
// 	}

// 	LOG_INF("Block count %u", block_count);

// 	if (disk_access_ioctl(disk_pdrv,
// 			DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
// 		LOG_ERR("Unable to get SD sector size");
// 		return -1;
// 	}

// 	LOG_INF("Sector size %u", block_size);

// 	memory_size_mb = (uint64_t)block_count * block_size;
// 	printk("Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));
	
// 	mp_fat.mnt_point = disk_mount_pt;
// 	res = fs_mount(&mp_fat);
// 	if (res == FR_OK) {
//         LOG_INF("SD mounted.");
// 	} else {
// 		LOG_ERR("Error mounting SD");
// 		return -1;
// 	}

// 	LOG_DBG("FAT FS Init Done");
//     return 1;
// }