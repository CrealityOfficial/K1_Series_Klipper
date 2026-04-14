from .serial_485_wrapper import Serial_485_Wrapper
def load_config_prefix(config):
    return(Serial_485_Wrapper(config))
