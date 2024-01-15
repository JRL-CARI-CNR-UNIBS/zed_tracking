import rospy
import logging
import yaml
import os.path

def read_param(param_name, default_value=None):
    param_value = rospy.get_param(param_name, default_value)
    if param_value is None:
        rospy.logerr(f"Failed to read {param_name} from the parameter server.")
        return None
    rospy.loginfo(f"Successfully read {param_name} from the parameter server: {param_value}")
    return param_value


def parse_config_file(config_file):
    logger = create_logger('YAML Parser', level=logging.DEBUG)
    with open(config_file, "r") as stream:
        try:
            params = yaml.safe_load(stream)
            logger.debug("Parsing " + config_file + "...\n" + yaml.dump(params))
        except yaml.YAMLError as exc:
            logger.error(exc)
    return params


def create_logger(name, level=logging.INFO):
    handler = logging.StreamHandler()
    handler.setLevel(level)
    handler.setFormatter(CustomFormatter())

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger


class CustomFormatter(logging.Formatter):
    green = "\x1b[32m"
    grey = "\x1b[38m"
    yellow = "\x1b[33m"
    red = "\x1b[31m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = "[ %(asctime)s - %(name)s - %(levelname)s ]:  %(message)s"

    FORMATS = {
        logging.DEBUG: green + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)