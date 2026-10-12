#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "DGLAB_EMU";

#define DEVICE_NAME "47L121000"

/* V3 UUIDs */
#define UUID_SVC_COMMAND   0x180C
#define UUID_CHR_WRITE     0x150A
#define UUID_CHR_NOTIFY    0x150B
#define UUID_SVC_BATTERY   0x180A
#define UUID_CHR_BATTERY   0x1500

static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_notify_val_handle = 0;
static uint16_t g_battery_val_handle = 0;
static uint8_t g_addr_type = BLE_ADDR_PUBLIC;

static uint8_t g_battery = 100;

/* Protocol-only simulated state. No physical output is connected. */
static uint8_t g_strength_a = 0;
static uint8_t g_strength_b = 0;
static uint8_t g_soft_limit_a = 200;
static uint8_t g_soft_limit_b = 200;
static uint8_t g_freq_balance_a = 0;
static uint8_t g_freq_balance_b = 0;
static uint8_t g_power_balance_a = 0;
static uint8_t g_power_balance_b = 0;

static void dump_hex(const char *label, const uint8_t *data, uint16_t len)
{
    printf("%s (%u): ", label, (unsigned)len);
    for (uint16_t i = 0; i < len; ++i) {
        printf("%02X", data[i]);
    }
    printf("\n");
}

static uint8_t clamp_strength(int value, uint8_t limit)
{
    if (value < 0) {
        return 0;
    }
    if (value > 200) {
        value = 200;
    }
    if (value > limit) {
        value = limit;
    }
    return (uint8_t)value;
}

static void send_b1(uint8_t seq)
{
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE || g_notify_val_handle == 0) {
        return;
    }

    uint8_t b1[4] = {
        0xB1,
        seq,
        g_strength_a,
        g_strength_b
    };

    struct os_mbuf *om = ble_hs_mbuf_from_flat(b1, sizeof(b1));
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate B1 mbuf");
        return;
    }

    int rc = ble_gatts_notify_custom(g_conn_handle, g_notify_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "B1 notify failed rc=%d", rc);
    } else {
        dump_hex("TX B1", b1, sizeof(b1));
    }
}

static void handle_b0(const uint8_t *data, uint16_t len)
{
    if (len != 20) {
        ESP_LOGW(TAG, "B0 invalid length: %u", (unsigned)len);
        return;
    }

    /*
     * V3:
     * byte 0: B0
     * byte 1: sequence(upper nibble) + strength parsing method(lower nibble)
     * byte 2: A strength setting
     * byte 3: B strength setting
     * byte 4..7: A frequencies
     * byte 8..11: A waveform strengths
     * byte 12..15: B frequencies
     * byte 16..19: B waveform strengths
     */
    uint8_t seq = (data[1] >> 4) & 0x0F;
    uint8_t method = data[1] & 0x0F;
    uint8_t set_a = data[2];
    uint8_t set_b = data[3];

    uint8_t method_a = (method >> 2) & 0x03;
    uint8_t method_b = method & 0x03;

    uint8_t old_a = g_strength_a;
    uint8_t old_b = g_strength_b;

    if (method_a == 1) {
        g_strength_a = clamp_strength((int)g_strength_a + set_a, g_soft_limit_a);
    } else if (method_a == 2) {
        g_strength_a = clamp_strength((int)g_strength_a - set_a, g_soft_limit_a);
    } else if (method_a == 3) {
        g_strength_a = clamp_strength(set_a, g_soft_limit_a);
    }

    if (method_b == 1) {
        g_strength_b = clamp_strength((int)g_strength_b + set_b, g_soft_limit_b);
    } else if (method_b == 2) {
        g_strength_b = clamp_strength((int)g_strength_b - set_b, g_soft_limit_b);
    } else if (method_b == 3) {
        g_strength_b = clamp_strength(set_b, g_soft_limit_b);
    }

    ESP_LOGI(TAG,
             "B0 seq=%u method=0x%X A=%u B=%u -> simulated A=%u B=%u",
             seq, method, set_a, set_b, g_strength_a, g_strength_b);

    /*
     * Keep the protocol parser intentionally separate from hardware control.
     * Waveform bytes are logged only.
     */
    ESP_LOGI(TAG, "A freq: %u %u %u %u",
             data[4], data[5], data[6], data[7]);
    ESP_LOGI(TAG, "A power: %u %u %u %u",
             data[8], data[9], data[10], data[11]);
    ESP_LOGI(TAG, "B freq: %u %u %u %u",
             data[12], data[13], data[14], data[15]);
    ESP_LOGI(TAG, "B power: %u %u %u %u",
             data[16], data[17], data[18], data[19]);

    if (seq != 0 && (old_a != g_strength_a || old_b != g_strength_b)) {
        send_b1(seq);
    }
}

static void handle_bf(const uint8_t *data, uint16_t len)
{
    if (len != 7) {
        ESP_LOGW(TAG, "BF invalid length: %u", (unsigned)len);
        return;
    }

    if (data[1] <= 200) g_soft_limit_a = data[1];
    if (data[2] <= 200) g_soft_limit_b = data[2];

    g_freq_balance_a = data[3];
    g_freq_balance_b = data[4];
    g_power_balance_a = data[5];
    g_power_balance_b = data[6];

    if (g_strength_a > g_soft_limit_a) g_strength_a = g_soft_limit_a;
    if (g_strength_b > g_soft_limit_b) g_strength_b = g_soft_limit_b;

    ESP_LOGI(TAG,
             "BF limits A=%u B=%u freqBalance A=%u B=%u powerBalance A=%u B=%u",
             g_soft_limit_a, g_soft_limit_b,
             g_freq_balance_a, g_freq_balance_b,
             g_power_balance_a, g_power_balance_b);
}

static int write_chr_access(uint16_t conn_handle,
                            uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt,
                            void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0 || len > 20) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t data[20];
    if (os_mbuf_copydata(ctxt->om, 0, len, data) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    dump_hex("RX 150A", data, len);

    if (data[0] == 0xB0) {
        handle_b0(data, len);
    } else if (data[0] == 0xBF) {
        handle_bf(data, len);
    } else {
        ESP_LOGW(TAG, "Unknown command HEAD=0x%02X", data[0]);
    }

    return 0;
}

static int notify_chr_access(uint16_t conn_handle,
                             uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    /* Notify-only characteristic; respond to reads with empty value. */
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int battery_access(uint16_t conn_handle,
                          uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, &g_battery, 1) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(UUID_SVC_COMMAND),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_WRITE),
                .access_cb = write_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_NOTIFY),
                .access_cb = notify_chr_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &g_notify_val_handle,
            },
            { 0 }
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(UUID_SVC_BATTERY),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(UUID_CHR_BATTERY),
                .access_cb = battery_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &g_battery_val_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

static void advertise(void);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE connected, conn_handle=%u", g_conn_handle);
        } else {
            ESP_LOGI(TAG, "BLE connection failed status=%d; restarting advertising",
                     event->connect.status);
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected reason=%d", event->disconnect.reason);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete; restarting");
        advertise();
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: %u", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static void advertise(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    const char *name = DEVICE_NAME;
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    ble_uuid16_t svc_uuid = BLE_UUID16_INIT(UUID_SVC_COMMAND);
    fields.uuids16 = &svc_uuid;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(g_addr_type,
                           NULL,
                           BLE_HS_FOREVER,
                           &adv_params,
                           gap_event,
                           NULL);

    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as %s", DEVICE_NAME);
    }
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &g_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed rc=%d", rc);
        return;
    }

    uint8_t addr_val[6];
    rc = ble_hs_id_copy_addr(g_addr_type, addr_val, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE address type=%u %02X:%02X:%02X:%02X:%02X:%02X",
                 g_addr_type,
                 addr_val[5], addr_val[4], addr_val[3],
                 addr_val[2], addr_val[1], addr_val[0]);
    } else {
        ESP_LOGW(TAG, "ble_hs_id_copy_addr failed rc=%d (type=%u)", rc, g_addr_type);
    }

    advertise();
}

static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    ESP_LOGI(TAG, "DG-LAB V3 BLE protocol emulator");
    ESP_LOGI(TAG, "Protocol-only emulator; no physical actuator output.");

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_svc_gap_device_name_set(DEVICE_NAME);

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed rc=%d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed rc=%d", rc);
        return;
    }

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(nimble_host_task);
}
