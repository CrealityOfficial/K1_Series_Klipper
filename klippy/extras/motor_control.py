from . import motor_control_wrapper

def load_config(config):
    return(motor_control_wrapper.Motor_Control(config))
