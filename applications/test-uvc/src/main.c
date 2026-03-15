/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uvc_app, LOG_LEVEL_INF);

void main(void)
{
    LOG_INF("esp32c3_supermini_uvc started");

    while (1) {
        k_msleep(1000);
    }
}
