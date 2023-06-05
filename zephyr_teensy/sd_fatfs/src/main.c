/*
 * Copyright (c) 2019 Tavish Naruka <tavishnaruka@gmail.com>
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

/* Sample which uses the filesystem API and SDHC driver */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <stdio.h>

LOG_MODULE_REGISTER(main);

#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/"DISK_DRIVE_NAME":"
#define MAX_PATH 128
#define SOME_FILE_NAME "test3.txt"
#define SOME_DIR_NAME "test3"
#define SOME_REQUIRED_LEN MAX(sizeof(SOME_FILE_NAME), sizeof(SOME_DIR_NAME))

static FATFS fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

static int lsdir(const char *path);
// #ifdef CONFIG_SAMPLE_FATFS_CREATE_SOME_ENTRIES
static bool create_some_entries(const char *base_path)
{
	char path[MAX_PATH];
	struct fs_file_t file;
	int base = strlen(base_path);
	char buffer[5] = "Hello";


	fs_file_t_init(&file);

	if (base >= (sizeof(path) - SOME_REQUIRED_LEN)) {
		LOG_ERR("Not enough concatenation buffer to create file paths");
		return false;
	}

	// LOG_INF("Creating some dir entries in %s", base_path);
	strncpy(path, base_path, sizeof(path));

	path[base++] = '/';
	path[base] = 0;
	strcat(&path[base], SOME_FILE_NAME);

	if (fs_open(&file, path, FS_O_WRITE) == 0) {
		// LOG_ERR("Failed to open file %s", path);
		LOG_ERR("Opened  file %s", path);
		// LOG_ERR("FOUND  file %s", path);
		// return false;
	}
	else{
		LOG_ERR("Failed to open file %s", path);
		return false;
	}
	
	if (fs_write(&file, &buffer,5 ) < 0) {
		LOG_ERR("Failed to WRITE file %s", path);
		return false;
	}

	printk("File CREATED#########\n");
	fs_close(&file);

	// path[base] = 0;
	// strcat(&path[base], SOME_DIR_NAME);

	// if (fs_mkdir(path) != 0) {
	// 	LOG_ERR("Failed to create dir %s", path);
	// 	/* If code gets here, it has at least successes to create the
	// 	 * file so allow function to return true.
	// 	 */
	// }
	return true;
}
// #endif

/*
*  Note the fatfs library is able to mount only strings inside _VOLUME_STRS
*  in ffconf.h
*/
static const char *disk_mount_pt = DISK_MOUNT_PT;

int main(void)
{

	usb_enable(NULL);
	

	mp.mnt_point = disk_mount_pt;

	int res = fs_mount(&mp);

	while (1)
	{

	printk("\n TEENSY Working\n");
	if (create_some_entries(disk_mount_pt)) {
			lsdir(disk_mount_pt);
		}
	else{
		printk("Could not create entries\n");
	}
	
	}
}

/* List dir entry by path
 *
 * @param path Absolute path to list
 *
 * @return Negative errno code on error, number of listed entries on
 *         success.
 */
static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		printk("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	printk("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("[DIR ] %s\n", entry.name);
		} else {
			printk("[FILE] %s (size = %zu)\n",
				entry.name, entry.size);
		}
		count++;
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);
	if (res == 0) {
		res = count;
	}

	return res;
}
