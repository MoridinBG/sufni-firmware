#include "data_sync.h"

#include "display.h"
#include "helpers.h"

#include "../net/tcpclient.h"
#include "../util/list.h"
#include "../util/log.h"

#include "ff.h"
#include "pico/time.h"

#include <stdio.h>

static void display_sync_progress(ssd1306_t *disp, uint current, uint total, uint failed_count) {
    char status[10];
    char failed[12];

    snprintf(status, sizeof(status), "%u / %u", current, total);
    snprintf(failed, sizeof(failed), "failed: %u", failed_count);

    ssd1306_clear(disp);
    ssd1306_draw_string(disp, 0, 0, 2, status);
    ssd1306_draw_string(disp, 0, 24, 1, failed);
    ssd1306_show(disp);
}

void sync_recorded_data(ssd1306_t *disp) {
    LOG("SYNC", "Starting data sync\n");
    display_message(disp, "CONNECT");

    if (!wifi_connect(true)) {
        LOG("SYNC", "Could not connect wifi\n");
        display_message(disp, "CONN ERR");
        sleep_ms(1000);
        wifi_disconnect();
        return;
    }

    display_message(disp, "DAT SYNC");

    FRESULT fr;
    DIR directory;
    FILINFO file_info;
    uint total_files = 0;

    struct list *to_import = list_create();
    fr = f_findfirst(&directory, &file_info, "", "?????.SST");
    while (fr == FR_OK && file_info.fname[0]) {
        ++total_files;
        list_push(to_import, file_info.fname);
        fr = f_findnext(&directory, &file_info);
    }
    f_closedir(&directory);
    LOG("SYNC", "Found %u files to sync\n", total_files);

    uint failed_files = 0;
    uint current_file = 0;
    struct node *node = to_import->head;
    TCHAR uploaded_path[19];

    while (node != NULL) {
        ++current_file;
        LOG("SYNC", "Sending file: %s (%u/%u)\n", (char *)node->data, current_file, total_files);
        if (send_file(node->data)) {
            LOG("SYNC", "File sent successfully\n");
            snprintf(uploaded_path, sizeof(uploaded_path), "uploaded/%s", (char *)node->data);
            f_rename(node->data, uploaded_path);
        } else {
            LOG("SYNC", "File send failed\n");
            ++failed_files;
        }

        display_sync_progress(disp, current_file, total_files, failed_files);
        sleep_ms(100);
        node = node->next;
    }

    list_delete(to_import);
    LOG("SYNC", "Sync complete: %u succeeded, %u failed\n", total_files - failed_files, failed_files);

    sleep_ms(3000);
    wifi_disconnect();
}