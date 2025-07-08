#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>

//		Basic zephyr BLE example
//

// There is a service, 'led_svc' which has 
// a characteristic contained inside of it called 'led_state'
// within this characteristic, there will be an attribute that holds a value (0 or 1)
// There's also a 'counter' characteristic which exists to demonstrate the notify
// mechanism (server-side) operation of the GATT protocol. This counter increments on
// every software-timer based periodic interrupt. 
//
// led_svc (Service)
// └── led_state (Characteristic)
//     └── Value Attribute (0 or 1)
// └── counter
//     └── count (0->1->2,...)
//

LOG_MODULE_REGISTER(main_,LOG_LEVEL_DBG);

uint16_t counter = 0;
bool	 notify_en = 0;
uint8_t  led_state;

#define TMR_PERIOD 			   K_MSEC(1000)
#define RUN_LED_BLINK_INTERVAL K_MSEC(1000)

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define LED_SVC_UUID \
	BT_UUID_128_ENCODE(0x12345678, 0x0000, 0xdef0, 0x1234, 0x56789abcdef0)

#define LED_STATE_UUID \
	BT_UUID_128_ENCODE(0x12345678, 0x0001, 0xdef0, 0x1234, 0x56789abcdef0)

#define COUNTER_UUID \
	BT_UUID_128_ENCODE(0x12345678, 0x0002, 0xdef0, 0x1234, 0x56789abcdef0)

static struct bt_uuid_128 led_svc_uuid 	 	  = BT_UUID_INIT_128(LED_SVC_UUID);
static struct bt_uuid_128 led_state_char_uuid = BT_UUID_INIT_128(LED_STATE_UUID);
static struct bt_uuid_128 counter_char_uuid   = BT_UUID_INIT_128(COUNTER_UUID);

struct bt_conn *my_conn = NULL;
static struct k_work adv_work;

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
	BT_GAP_ADV_FAST_INT_MIN_1, /* 0x30 units, 48 units, 30ms */
	BT_GAP_ADV_FAST_INT_MAX_1, /* 0x60 units, 96 units, 60ms */
	NULL); /* Set to NULL for undirected advertising */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, LED_SVC_UUID),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME)-1)
};

static ssize_t read_count(struct bt_conn *conn,
						  const struct bt_gatt_attr *attr, void *buf,
						  uint16_t len, uint16_t offset)
{
	const uint8_t *val = attr->user_data;
	LOG_INF("%s: Value 0x%x read.\n", __func__, *val);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, val, sizeof(*val));/*  */
}

// conn 	Represents the BLE connection.
// attr 	Represents the attribute (the low level representation of a characteristic).
// buf 		A buffer into which you can write the actual value the client will receives.
// len 		Length of the data in the buffer.
// offset 	Offset to start reading from.
static ssize_t read_led(struct bt_conn *conn,
						const struct bt_gatt_attr *attr, void *buf,
						uint16_t len, uint16_t offset)
{
	const uint8_t *val = attr->user_data;
	LOG_INF("%s: Value 0x%x read.\n", __func__, *val);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, val, sizeof(*val));
}

// conn 	Represents the BLE connection.
// attr 	Represents the attribute (the low-level representation of a characteristic).
// buf 		A buffer from which the client wants to write.
// len 		Length of the data in the buffer.
// offset 	Offset to start reading from.
// flags 	Attribute write flags (to indicate long write etc.)
static ssize_t write_led(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr, const void *buf,
                         uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *val = attr->user_data;
	*val = *((uint8_t *)buf);

	LOG_INF("Value 0x%x written.\n", *val);
	LOG_INF("Current LED state %s - turning LED %s\n", led_state ? "off" : "on",
			led_state ? "on" : "off");

	gpio_pin_set_dt(&led, led_state);
	return len;
}

static void ledsvc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_en = (value == BT_GATT_CCC_NOTIFY);
}

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection error %d", err);
        return;
    }
    LOG_INF("Connected");
    my_conn = bt_conn_ref(conn);

	struct bt_conn_info info;
	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("bt_conn_get_info() returned %d", err);
		return;
	}

	double connection_interval = info.le.interval*1.25; // in ms
	uint16_t supervision_timeout = info.le.timeout*10; // in ms
	uint16_t mtu = bt_gatt_get_mtu(conn);
	LOG_INF("Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms, MTU: %d", connection_interval, info.le.latency, supervision_timeout, mtu);

    gpio_pin_set_dt(&led, 1);
	led_state = 1;
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_ERR("Disconnected. Reason %d", reason);
    bt_conn_unref(my_conn);

	gpio_pin_set_dt(&led, 0);
	led_state = 0;
}

void on_recycled(void)
{
    advertising_start();
}

void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double connection_interval = interval*1.25;         // in ms
    uint16_t supervision_timeout = timeout*10;          // in ms

	uint16_t mtu = bt_gatt_get_mtu(conn);
	LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms, MTU: %d", connection_interval, latency, supervision_timeout, mtu);
}

struct bt_conn_cb connection_callbacks = {
    .connected              = on_connected,
    .disconnected           = on_disconnected,
    .recycled               = on_recycled,
	.le_param_updated       = on_le_param_updated
};

static void tmr_isr(struct k_timer *timer_id)
{
	counter++;
	// LOG_INF("count=%d", counter);
}

K_TIMER_DEFINE(my_timer, tmr_isr, NULL);

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

	led_state = 0;
	return 0;
}

static int init_bt(void)
{
	int err = 0;

	err = bt_conn_cb_register(&connection_callbacks);
	if (err) {
	LOG_INF("Connection callback register failed (err %d)", err);
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return -1;
	}

	LOG_INF("Bluetooth initialized");
	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	return err;
}

BT_GATT_SERVICE_DEFINE(
	led_svc, 
	BT_GATT_PRIMARY_SERVICE(&led_svc_uuid),
	BT_GATT_CHARACTERISTIC(	// led_state {attributes: 1,2}
		&led_state_char_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_led,
		write_led,
		&led_state
	),
	BT_GATT_CHARACTERISTIC(	// counter{3,4}
		&counter_char_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		read_count,
		NULL,
		&counter
	),
	BT_GATT_CCC(	// Client Characteristic Configuration {5}
		ledsvc_ccc_cfg_changed,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
	),
);

int main(void)
{
	int err;

	LOG_INF("Starting BLE_LED...");

	err = init_led();
	if (err) {
		LOG_ERR("LEDs init failed (err %d)", err);
		return -1;
	}

	err = init_bt();
	if (err) {
		LOG_ERR("BT init failed (err %d)", err);
		return -1;
	}

	k_timer_start(&my_timer, TMR_PERIOD, TMR_PERIOD);
	while (1)
	{
		struct bt_conn_info info;
		bt_conn_get_info(my_conn, &info);
		if (info.state != BT_CONN_STATE_CONNECTED)
		{
			gpio_pin_toggle_dt(&led);
			led_state ^= 1;	// invert the state (toggle)
		}

		// send notifications
		if (notify_en)
			bt_gatt_notify(NULL, &led_svc.attrs[4], &counter, sizeof(counter));

		k_sleep(RUN_LED_BLINK_INTERVAL);
	}
}
