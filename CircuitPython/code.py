from os import getenv
import board
import digitalio
import rtc
import time
import busio
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
from adafruit_esp32spi import adafruit_esp32spi
import adafruit_requests as requests
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from seeed_hm3301 import HM3301_I2C


# Get wifi details and more from a settings.toml file
# tokens used by this Demo: CIRCUITPY_WIFI_SSID, CIRCUITPY_WIFI_PASSWORD
secrets = {
    "ssid": getenv("CIRCUITPY_WIFI_SSID"),
    "password": getenv("CIRCUITPY_WIFI_PASSWORD"),
}
if secrets == {"ssid": None, "password": None}:
    try:
        # Fallback on secrets.py until depreciation is over and option is removed
        from secrets import secrets
    except ImportError:
        print("WiFi secrets are kept in settings.toml, please add them there!")
        raise

print("WSAQI Python Version.")

# If you are using an RPI Feather with an Airlift Feather wing, use this pin out
esp32_cs = digitalio.DigitalInOut(board.D16)
esp32_ready = digitalio.DigitalInOut(board.D18)
esp32_reset = digitalio.DigitalInOut(board.D17)

#esp32_gpio0 = digitalio.DigitalInOut(board.D6) # For airlift wifi shield

## Setting up the Wifi...
spi = board.SPI()
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)
requests.set_socket(socket, esp)

# Set up the dust sensor
i2c0_bus = busio.I2C(board.SCL, board.SDA, frequency=100000)
hm3301 = HM3301_I2C(i2c0_bus, address=0x40)


# Info for the MQTT Server
mqtt_broker = "69.109.130.206"
weather_feed = "weather/test"
mqtt_username = "power"
mqtt_password = "nD3M$3AhDob2K+xhAE"
mqtt_port = 1883;


if esp.status == adafruit_esp32spi.WL_IDLE_STATUS:
    print("ESP32 found and in idle mode")
print("Firmware vers.", esp.firmware_version)
print("MAC addr:", [hex(i) for i in esp.MAC_address])

print("Connecting to AP...")
while not esp.is_connected:
    try:
        esp.connect_AP(getenv('CIRCUITPY_WIFI_SSID'), getenv('CIRCUITPY_WIFI_PASSWORD'))
    except OSError as e:
        print("could not connect to AP, retrying: ", e)
        continue
print("Connected to", str(esp.ssid, "utf-8"), "\tRSSI:", esp.rssi)
print("My IP address is", esp.pretty_ip(esp.ip_address))

# Set up the time using Adafruit server.
TEXT_URL = "https://io.adafruit.com/api/v2/time/seconds?x-aio-key=5e2a8804feafe7214cd3711c5138a2f2a02b4414&tz=UTC"

# Initializing the base time on the board.
print("Fetching text from", TEXT_URL)
r = requests.get(TEXT_URL)
print(r.text)
retrievedTime = int(r.text)
r.close()

value = time.localtime(retrievedTime)

r = rtc.RTC()
r.datetime = value

print(value)
print("Setting time")
print(time.time())


# Set up MQTT

# Define callback methods which are called when events occur
# pylint: disable=unused-argument, redefined-outer-name
def connected(client, userdata, flags, rc):
    # This function will be called when the client is connected
    # successfully to the broker.
    print(f"Connected to Baugh.org. Listening for msgs")
    # Subscribe to all changes on the FEED_NAME.
    #client.subscribe(FEED_NAME)


def disconnected(client, userdata, rc):
    # This method is called when the client is disconnected
    print("Disconnected from Baugh.org.")


def message(client, topic, message):
    # This method is called when a topic the client is subscribed to
    # has a new message.
    print(f"New message on topic {topic}: {message}")


def get_hm3301_data(hm3301):
    # acquire the data
    try:
        PM_std = hm3301.get_std_readings()
        print(PM_std)
        PM_atm= hm3301.get_atm_readings()
        print(PM_atm)
        retval = '"pm1_0":'+str(PM_std[0])+', "pm2_5":'+str(PM_std[1])+', "pm10_0":'+str(PM_std[2])

    except:
        retval = "ERROR"

    return str(retval)

def createAndSendMqttMsg( eventTime ):

    message = '{ "dateTime":'+str(eventTime)+', '+get_hm3301_data(hm3301)+' }'
    print(message)

    mqtt_client.publish(weather_feed, message)

    return message


MQTT.set_socket(socket, esp)

# Set up a MiniMQTT Client
mqtt_client = MQTT.MQTT(
    broker=mqtt_broker,
    port=mqtt_port,
    username=mqtt_username,
    password=mqtt_password
)

# Setup the callback methods above
mqtt_client.on_connect = connected
mqtt_client.on_disconnect = disconnected
mqtt_client.on_message = message

print("Connecting to MQTT Server...")
mqtt_client.connect()

check_time = time.monotonic()

while True:

    # Poll the message queue
    mqtt_client.loop()


    # this code here could be better but works for our half-assed implementation
    if time.monotonic() - check_time >= 20:
        createAndSendMqttMsg(time.time() )
        check_time = time.monotonic()

