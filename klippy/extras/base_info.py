import os, re, logging

class BaseDirSystemInfo:
    _instance = None

    def __new__(cls):
        if not cls._instance:
            cls._instance = super(BaseDirSystemInfo, cls).__new__(cls)
            cls._instance._base_dir = "/usr/data"
            cls._instance._h264_encoder_flag = "H264_ENCODER"
            cls._instance._initialize_base_dir()
        return cls._instance

    def _initialize_base_dir(self):
        path = "/etc/openwrt_release"
        if os.path.exists(path):
            self._base_dir = "/mnt/UDISK"
            try:
                text = ""
                with open(path, "r") as f:
                    text = f.read()
                match = re.search(r"DISTRIB_TARGET='(.*?)'", text)
                if match:
                    if "t113" in match.group(0):
                        self._h264_encoder_flag = "NO_H264_ENCODER"
            except Exception as err:
                logging.error(err)

    def get_base_dir(self):
        return self._base_dir


system_info_instance = BaseDirSystemInfo()
base_dir = system_info_instance.get_base_dir()
