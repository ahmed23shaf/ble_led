#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define RUN_LED_BLINK_INTERVAL 1000

struct bt_conn *my_conn = NULL;
static struct k_work adv_work;

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
	BT_GAP_ADV_FAST_INT_MIN_1, /* 0x30 units, 48 units, 30ms */
	BT_GAP_ADV_FAST_INT_MAX_1, /* 0x60 units, 96 units, 60ms */
	NULL); /* Set to NULL for undirected advertising */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0x12345678, 0x9abc, 0xdef0, 0x1234, 0x56789abcdef0)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME)-1)
};

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection error %d\n", err);
        return;
    }
    printk("Connected\n");
    my_conn = bt_conn_ref(conn);

	struct bt_conn_info info;
	err = bt_conn_get_info(conn, &info);
	if (err) {
		printk("bt_conn_get_info() returned %d", err);
		return;
	}

	double connection_interval = info.le.interval*1.25; // in ms
	uint16_t supervision_timeout = info.le.timeout*10; // in ms
	printk("Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms\n", connection_interval, info.le.latency, supervision_timeout);

    gpio_pin_set_dt(&led, 1);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected. Reason %d\n", reason);
    bt_conn_unref(my_conn);

	gpio_pin_set_dt(&led, 0);
}

void on_recycled(void)
{
    advertising_start();
}

void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double connection_interval = interval*1.25;         // in ms
    uint16_t supervision_timeout = timeout*10;          // in ms
    printf("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms\n", connection_interval, latency, supervision_timeout);
}

struct bt_conn_cb connection_callbacks = {
    .connected              = on_connected,
    .disconnected           = on_disconnected,
    .recycled               = on_recycled,
	.le_param_updated       = on_le_param_updated
};

static int init_led(void)
{
	int err;

	if (!gpio_is_ready_dt(&led)) {
		return -1;
	}

	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		return err;
	}

	return 0;
}

static int init_bt(void)
{
	int err = 0;

	err = bt_conn_cb_register(&connection_callbacks);
	if (err) {
	printk("Connection callback register failed (err %d)\n", err);
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

	printk("Bluetooth initialized\n");
	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	return err;
}

int main(void)
{
	int err;

	printk("Starting Lesson 3 - Exercise 1\n");

	err = init_led();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return -1;
	}

	err = init_bt();
	if (err) {
		printk("BT init failed (err %d)\n", err);
		return -1;
	}

	while (1)
	{
		struct bt_conn_info info;
		if (my_conn && bt_conn_get_info(my_conn, &info) == 0) {
			if (info.state != BT_CONN_STATE_CONNECTED)
				gpio_pin_toggle_dt(&led);
		}

		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}
