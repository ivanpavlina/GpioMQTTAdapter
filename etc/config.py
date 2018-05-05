# Remove .EXAMPLE from filename to use it

log = {
    #'location': '/some/log/location',            # Location for application logs
    'location': '',                               # If empty using stdout
    'formatter_main': '[%(asctime)s]:[%(levelname)s]: %(message)s',
    'size': 1024 * 1024 * 50,  # 50MB limit
    'backups': 10,             # for max 10 log files
    'level': 'INFO'
}

mqtt_broker = {
    'host': 'hassio',
    'port': 1883,
    'keepalive': 60
}

mqtt = {
    'status_message_interval': 5,

    # LWT MQTT topic
    'availability_topic': 'tele/mainrelay/LWT',  # LWT message topic
    # MQTT state topic
    'state_topic': 'tele/mainrelay/STATE',  # State topic, publishing current state on every status_message_interval

    # MQTT topic templates
    'device_control_topic_template': 'cmnd/mainrelay/{}/POWER',  # Relay control topic {}.format(devices.name)
    'device_state_topic_template': 'stat/mainrelay/{}/POWER',  # Relay status topic {}.format(devices.name)

    # MQTT payloads
    #  Control
    'payload_on': b'ON',
    'payload_off': b'OFF',
    'payload_toggle': b'TOGGLE',
    #  Availability
    'payload_available': b'Online',
    'payload_not_available': b'Offline'
}

# Defined GPIO pins.
# GPIO.BOARD schema is used
devices = [
    {'name': 'ch1',
     'gpio_board_pin': 3},
    {'name': 'ch2',
     'gpio_board_pin': 5},
    {'name': 'ch3',
     'gpio_board_pin': 7},
    {'name': 'ch4',
     'gpio_board_pin': 8},
    {'name': 'ch5',
     'gpio_board_pin': 10},
    {'name': 'ch6',
     'gpio_board_pin': 11},
    {'name': 'ch7',
     'gpio_board_pin': 12},
    {'name': 'ch8',
     'gpio_board_pin': 13}
]
