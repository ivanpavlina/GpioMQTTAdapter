import logging
from etc import config
from lib.logrotate.cloghandler import ConcurrentRotatingFileHandler


def get_logger(class_name):
    logger = logging.getLogger(class_name)
    formatter = logging.Formatter(config.log['formatter_main'])
    handler = ConcurrentRotatingFileHandler(config.log['location']+"/"+class_name, "a", config.log['size'], config.log['backups'])
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.getLevelName(config.log['level']))
    return logger