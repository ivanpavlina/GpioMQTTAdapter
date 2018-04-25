import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from time import sleep
from etc import config
import logging
from lib.logrotate.cloghandler import ConcurrentRotatingFileHandler
from datetime import datetime
import json


class Worker:

    def __init__(self, logger):
        self.LOGGER = logger

        # Init MQTT client and callbacks
        self._client = mqtt.Client()
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

        # Init GPIO and set board pin numbering
        self._configured_pins = []
        GPIO.setmode(GPIO.BOARD)

    # Callback
    def _on_connect(self, mqttc, obj, flags, rc):
        self.LOGGER.info("Connected MQTT broker")
        # When connected subscribe to control topics
        try:
            subscribe_topics = []
            for device in config.devices:
                device_name = device['name']
                device_control_topic = config.mqtt['device_control_topic_template'].format(device_name)
                device_state_topic = config.mqtt['device_state_topic_template'].format(device_name)
                subscribe_topics.append((device_control_topic, 1))
                subscribe_topics.append((device_state_topic, 1))
            self.LOGGER.info("Generated {} topics for subscribing".format(len(subscribe_topics)))
        except:
            self.LOGGER.error("Error building topic list for subscribing")
            raise

        try:
            self._client.subscribe(subscribe_topics)
        except Exception, e:
            self.LOGGER.error("Error subscribing to topics [{}]".format(e))
            raise

        # Publish LWT message on connect
        self._client.publish(config.mqtt['availability_topic'], config.mqtt['payload_available'], qos=2, retain=True)

    # Callback
    def _on_disconnect(self, client, userdata, rc):
        self.LOGGER.warning("Disconnected MQTT broker")

    # Callback
    # Called when message is received on subscribed topics
    def _on_message(self, mqttc, obj, msg):
        self.LOGGER.info("Received message {} -> {}".format(msg.topic, msg.payload))
        try:
            # Find which device is on received topic
            for device in config.devices:
                device_name = device['name']
                device_gpio_pin = device['gpio_board_pin']

                device_control_topic = config.mqtt['device_control_topic_template'].format(device_name)
                device_status_topic = config.mqtt['device_state_topic_template'].format(device_name)

                if msg.topic == device_control_topic:
                    # Check if payload is correct and do appropriate action on GPIO
                    if msg.payload == config.mqtt['payload_on']:
                        self._gpio_switch_pin_state(device_gpio_pin, 'ON')
                    elif msg.payload == config.mqtt['payload_off']:
                        self._gpio_switch_pin_state(device_gpio_pin, 'OFF')
                    elif msg.payload == config.mqtt['payload_toggle']:
                        pin_state = self._gpio_read_pin_state(device_gpio_pin)
                        if pin_state == 'ON':
                            self._gpio_switch_pin_state(device_gpio_pin, 'OFF')
                        elif pin_state == 'OFF':
                            self._gpio_switch_pin_state(device_gpio_pin, 'ON')
                    else:
                        raise Exception("Invalid payload received [{}] for [{}]".format(msg.payload, device['name']))

                    pin_state = self._gpio_read_pin_state(device_gpio_pin)
                    self._client.publish(device_status_topic, pin_state, qos=0)
                    self.LOGGER.info("Published pin [{}] state [{}] on MQTT topic [{}]".format(pin_state,
                                                                                               pin_state,
                                                                                               device_status_topic))
        except Exception, e:
            self.LOGGER.warning("Error processing message [{}] >> \n{}".format(msg, e))

    def _gpio_switch_pin_state(self, pin, real_state='OFF'):
        try:
            # Il je banana il je relej al GPIO.HIGH je na releju low, a GPIO.LOW je high
            if real_state == 'ON':
                gpio_state = GPIO.LOW
            else:
                gpio_state = GPIO.HIGH

            GPIO.output(pin, gpio_state)
            self.LOGGER.info("Pin {} switched to {}".format(pin, real_state))
        except Exception, e:
            self.LOGGER.error("Error switching GPIO pin {} state to {} >> {}".format(pin, real_state, e))
            return False

        self.LOGGER.debug("GPIO pin {} state switched to {}".format(pin, real_state))

    def _gpio_read_pin_state(self, pin):
        try:
            res = GPIO.input(pin)
            if res == GPIO.LOW:
                return 'ON'
            else:
                return 'OFF'
        except Exception, e:
            self.LOGGER.error("Error reading pin {} state >> {}".format(pin, e))

    def setup(self):
        # Connect MQTT
        self.LOGGER.debug("Connecting MQTT broker")
        try:
            # Before connecting to broker setup LWT message so the broker publishes it for us
            self._client.will_set(config.mqtt['availability_topic'], config.mqtt['payload_not_available'], retain=True)

            self._client.connect(config.mqtt_broker['host'],
                                 config.mqtt_broker['port'],
                                 config.mqtt_broker['keepalive'])
        except Exception, e:
            self.LOGGER.error("Error connecting MQTT broker [{}]".format(e))

        # Initialize GPIO pins
        for device in config.devices:
            try:
                GPIO.setup(device['gpio_board_pin'], GPIO.OUT, initial=GPIO.HIGH)
                self._configured_pins.append(device['gpio_board_pin'])
            except Exception, e:
                self.LOGGER.error("Error initializing GPIO pin for device {} >> {}".format(device, e))
        self.LOGGER.info("GPIO pins initialized")

    def _cleanup(self):
        self.LOGGER.info("Cleanup started")

        # Publish LWT message on disconnect
        self._client.publish(config.mqtt['availability_topic'],
                             config.mqtt['payload_not_available'],
                             qos=2,
                             retain=True)
        self._client.loop()
        self.LOGGER.info("Published 'unavailable' on availablitiy topic")

        self.LOGGER.info("Disconnecting MQTT client")
        # Disconnect broker
        self._client.disconnect()
        self._client.loop()

        # Cleanup GPIO
        # Set every pin to off
        for gpio_pin in self._configured_pins:
            self._gpio_switch_pin_state(gpio_pin, 'OFF')

        # Release GPIO
        GPIO.cleanup()
        self.LOGGER.info("GPIO pins cleaned up")

        self.LOGGER.info("Cleanup finished")

    def run(self):
        self.LOGGER.info("Starting thread loop")

        # Looping for initialization
        self._client.loop_start()
        # Give mqtt a moment to connect and settle
        sleep(1)

        while True:
            self.LOGGER.info("Running...")
            self._client.loop_stop()

            try:
                # On each run publish this apps status on MQTT
                topic = config.mqtt['state_topic']
                payload = {'Time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                           'Uptime': 0}
                self._client.publish(topic, json.dumps(payload), qos=0)

                self._client.loop_start()
                try:
                    sleep(config.mqtt['status_message_interval'])
                except KeyboardInterrupt:
                    self.LOGGER.error("Got keyboard shutdown")
                    self._cleanup()
                    break

            except Exception, e:
                self.LOGGER.error("Exception in loop\n***{}".format(e))
                self._cleanup()
                break


if __name__ == '__main__':
    logger = logging.getLogger()
    formatter = logging.Formatter(config.log['formatter_main'])
    handler = ConcurrentRotatingFileHandler(config.log['location'], "a", config.log['size'], config.log['backups'])
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.getLevelName(config.log['level']))

    worker = Worker(logger)
    worker.setup()
    try:
        worker.run()
    except Exception, e:
        logger.warning("Main exiting >> {}".format(e))
    logger.warning("Bye Bye")
