#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

bool led_state = false;

// BT STUFF ----------------------------------------------------------------------------------------

// Service and Characteristics UUIDs
#define LED_SERVICE_UUID_VAL \
  BT_UUID_128_ENCODE(0xf7547938, 0x68ba, 0x11ec, 0x90d6, 0x0242ac120003)

static struct bt_uuid_128 led_svc_uuid =
	BT_UUID_INIT_128(LED_SERVICE_UUID_VAL);

static struct bt_uuid_128 led_state_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x9c85a726, 0xb7f1, 0x11ec, 0xb909, 0x0242ac120002));

// Advertisement Data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, LED_SERVICE_UUID_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1)
};

// GATT Access Callbacks

static ssize_t read_led_state(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset) {
  const uint8_t *value = attr->user_data;
  printk("Value 0x%x read.\n", *value);
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t write_led_state(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr, const void *buf,
                               uint16_t len, uint16_t offset, uint8_t flags) {
  uint8_t *value = attr->user_data;
  *value = *((uint8_t *)buf);
  printk("Value 0x%x written.\n", *value);

  printk("Current LED state %s - turning LED %s\n", led_state ? "off" : "on",
         led_state ? "on" : "off");
  gpio_pin_set_dt(&led, led_state);
  return len;
}

// Attribute Table
BT_GATT_SERVICE_DEFINE(
    led_svc, BT_GATT_PRIMARY_SERVICE(&led_svc_uuid),
    BT_GATT_CHARACTERISTIC(&led_state_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_led_state, write_led_state, &led_state), );

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
		printk("Connection failed (err 0x%02x)\n", err);
	else
		printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static int init_bt(void)
{
	int err; 

	// initialize BLE
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}
	printk("Bluetooth initialized\n");

	// start advertising
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return err;
	}

	printk("Advertising successfully started\n");
	return 0;
}

// BT STUFF END ------------------------------------------------------------------------------------

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

int main(void)
{
	if (init_led())	return -1;
	if (init_bt())	return -1;

	while (1) { k_sleep(K_SECONDS(1)); }
	return 0;
}
