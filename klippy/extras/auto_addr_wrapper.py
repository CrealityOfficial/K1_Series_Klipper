import logging
import copy
from dataclasses import dataclass
from typing import List


########################### 使用说明 ###############################
# cfg文件添加 [auto_addr]配置项
# dev_table_map_table 包含三种类型的设备，MB，CLM，BTM 分别表示料盒 闭环电机 皮带张紧电机
# 如果不需要 注释掉响应的行即可，比如只需要料盒自动分配，可以只保留MB行 
# 外部模块通过获取 addr_manager_table_xx[i].online 来获取在线状态，如果为1表示在线，否则离线
# 在线状态是通过online_check来更新，连续3次没回复会认为掉线，有几秒的滞后
# 本文件所有命令未开启底层重传，有重试需要的在应用层实现
# 通过 PRINT_ADDR_TABLE 命令可以查看当前地址表的状态
# 地址列表发生变化后，uniid会保存在 cfg文件末尾，形如
# #*# [auto_addr]
# #*# mb_addr_table_uniids = xxx

# 最大包长度
MAX_DATA_LEN = 100  
# 最大uid长度
MAX_UNIID_LEN = 12
MAX_LOST_CNT = 3
# 包头
PACK_HEAD = 0xF7
# 广播地址
BROADCAST_ADDR = 0xFF
BROADCAST_ADDR_MB = 0xFE
BROADCAST_ADDR_CLM = 0xFD
BROADCAST_ADDR_BTM = 0xFC
# 自动地址获取相关命令
CMD_GET_SLAVE_INFO = 0xA1
CMD_SET_SLAVE_ADDR = 0xA0
CMD_ONLINE_CHECK = 0xA2
CMD_GET_ADDR_TABLE = 0xA3
CMD_LOADER_TO_APP = 0x0B

# 当前的状态 loader还是app
MODE_APP = 0
MODE_LOADER = 1

# 设备类型
# MB-料盒 CLM-闭环电机 BTM-皮带张紧电机
DEV_TYPE_MB = 1
DEV_TYPE_CLM = 2
DEV_TYPE_BTM = 3
# 下标偏移量
DEV_TYPE_INDEX_OFFSET = DEV_TYPE_MB

# CRC8校验计算参数
POLY = 0x07 

# package status
STATUS_OK = 0x00 
# STATUS_ERROR = 1

# online state
ONLINE_STATE_OFFLINE = 0
ONLINE_STATE_ONLINE = 1
ONLINE_STATE_INIT = 2
ONLINE_STATE_WAIT_FOR_ACK = 3

# 最大设置和获取次数 
MAX_GET_TIMES = 2
MAX_SET_TIMES = 2

# 命令超时时间
TIMEOUT_SHORT_TIME = 0.05
TIMEOUT_MEDIUM_TIME = 0.1
TIMEOUT_LONG_TIME = 1.0 

cmd_timeout = { CMD_GET_SLAVE_INFO: TIMEOUT_LONG_TIME,
                CMD_SET_SLAVE_ADDR: TIMEOUT_SHORT_TIME, 
                CMD_GET_ADDR_TABLE: TIMEOUT_SHORT_TIME,
                CMD_ONLINE_CHECK:   TIMEOUT_MEDIUM_TIME,
                CMD_LOADER_TO_APP:  TIMEOUT_SHORT_TIME }
name_map = {
    DEV_TYPE_MB: "mb_addr_table_uniids",
    # DEV_TYPE_CLM: "clm_addr_table_uniids",
    # DEV_TYPE_BTM: "btm_addr_table_uniids",
}

# 包格式
@dataclass
class DataPackage:
    head: int
    slave_addr: int
    length: int
    status: int
    function_code: int
    data: List[int]
    crc: int

# 返回数据的格式
@dataclass
class FcAckData:
    dev_type: int
    mode: int  # 0: app, 1: loader
    uniid: List[int]

@dataclass
class AddrManager:
    addr: int
    uniid: List[int]
    mapped: int
    online: int
    acked: int
    lost_cnt: int
    mode: int

# 料盒地址
addr_manager_table_mb = [
    AddrManager(0x01, [0x00], 0, 0, 0, 0, 0),
    AddrManager(0x02, [0x00], 0, 0, 0, 0, 0),
    AddrManager(0x03, [0x00], 0, 0, 0, 0, 0),
    AddrManager(0x04, [0x00], 0, 0, 0, 0, 0),
]

# 闭环电机地址
addr_manager_table_cl_motor = [
    AddrManager(0x81, [0x00], 0, 0, 0, 0, 0),
    AddrManager(0x82, [0x00], 0, 0, 0, 0, 0),
    AddrManager(0x83, [0x00], 0, 0, 0, 0, 0),
    AddrManager(0x84, [0x00], 0, 0, 0, 0, 0),
]

# 皮带张紧电机地址
addr_manager_table_bt_motor = [
    AddrManager(0x91, [0x00], 0, 0, 0, 0, 0),
    AddrManager(0x92, [0x00], 0, 0, 0, 0, 0),
]

class DevTableMap:
    def __init__(self, dev_type, broadcast_addr, addr_manager_table):
        self.dev_type = dev_type
        self.broadcast_addr = broadcast_addr
        self.addr_manager_table = addr_manager_table
        self.size = len(addr_manager_table)

dev_table_map_table = [
    DevTableMap(DEV_TYPE_MB, BROADCAST_ADDR_MB, addr_manager_table_mb),
    # DevTableMap(DEV_TYPE_CLM, BROADCAST_ADDR_CLM, addr_manager_table_cl_motor),
    # DevTableMap(DEV_TYPE_BTM, BROADCAST_ADDR_BTM, addr_manager_table_bt_motor),
]

class AutoAddrWrapper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.configfile = self.printer.lookup_object('configfile')
        self.config = config
        self.gcode = self.printer.lookup_object('gcode')
        self._serial = self.printer.lookup_object("serial_485 " + "serial485")
        # self.parse = ParseData()
        # self.get_finished = False
        self.uniid_changed = False
        self.print_stats = None
        self.gcode.register_command('PRINT_ADDR_TABLE', self.cmd_PRINT_ADDR_TABLE)
        self.gcode.register_command('BOX_GET_ADDR_TABLE', self.cmd_BOX_GET_ADDR_TABLE)

        # 获取保存的地址列表
        for i in range(len(dev_table_map_table)):
            dev_table_map = dev_table_map_table[i]
            self.get_addr_table_uniids(dev_table_map)

        if config.has_section("motor_control") and config.getsection('motor_control').getint('switch')==1:
            self.printer.register_event_handler('auto_addr:start', self.reg_auto_addr_set)
        else:
            self.printer.register_event_handler('klippy:ready', self.reg_auto_addr_set)

        self.printer.register_event_handler('auto_addr:set_finished', self.reg_auto_addr_process)

        logging.info("auto addr wrapper init")

    ###################### 获取上次的地址对应的uniid ########################
    def get_addr_table_uniids(self, dev_table_map):
        name = name_map[dev_table_map.dev_type]
        if self.config.get(name, None) is not None:
            def custom_int_parser(value):
                try:
                    if value.startswith('0x') or value.startswith('0X'):
                        return int(value, 16)
                    else:
                        return int(value)
                except ValueError as e:
                    raise ValueError(f"Invalid literal for int with base 10 or 16: '{value}'") from e
            uniids = self.config.getlists(name, seps=(',', '\n'), parser=custom_int_parser)
            if len(uniids) != dev_table_map.size:
                logging.info("Error: %s, uniids len: %d, dev_table_map.size: %d" % (name, len(uniids), dev_table_map.size))
            else:
                logging.info("addr table uniid get %s", name)
                for i in range(dev_table_map.size):
                    dev_table_map.addr_manager_table[i].uniid = list(uniids[i])
                    ## 如果uniid不为空，说明该地址上次关机前已分配，需要置位相关标志
                    if len(uniids[i]) >= 1:
                        if uniids[i][0] != 0:
                            dev_table_map.addr_manager_table[i].mapped = 1
                            dev_table_map.addr_manager_table[i].online = ONLINE_STATE_INIT 
                    logging.info("addr %d: %s", dev_table_map.addr_manager_table[i].addr, dev_table_map.addr_manager_table[i].uniid)
        else:
            logging.info("Error: %s not exist" % name)

    ###################### 保存地址对应的uniid ########################
    def save_addr_table_uniids(self, dev_table_map):
        dev_type = dev_table_map.dev_type
        name = name_map[dev_type] 
        logging.info("addr table uniid save %s", name)
        uniids = ""
        for i in range(dev_table_map.size):
            uniids += "\n  "
            for j in range(len(dev_table_map.addr_manager_table[i].uniid)):
                uniids += "0x%02X, " % dev_table_map.addr_manager_table[i].uniid[j]
            uniids = uniids[:-2]
        logging.info("uniids: %s", uniids)
        section = "auto_addr"
        self.configfile.set(section, name, uniids)
        self.gcode.run_script_from_command("CXSAVE_CONFIG")

    ###################### gcode cmd ########################
    def cmd_PRINT_ADDR_TABLE(self, gcmd):
        for i in range(len(dev_table_map_table)):
            dev_table_map = dev_table_map_table[i]
            addr_manager_table = dev_table_map.addr_manager_table
            size = len(addr_manager_table)
            for i in range(size):
                response = "0x%02X, " % addr_manager_table[i].addr
                for j in range(len(addr_manager_table[i].uniid)):
                    response += "0x%02X, " % addr_manager_table[i].uniid[j]
                response += "%d, %d, %d, %d, mode:%d" % (addr_manager_table[i].mapped, addr_manager_table[i].online,
                        addr_manager_table[i].acked, addr_manager_table[i].lost_cnt, addr_manager_table[i].mode)
                gcmd.respond_info(response)

    def cmd_BOX_GET_ADDR_TABLE(self, gcmd):
        self.process_get_addr_table(None)

    ##################### addr allocate #####################
    def addr_allocate(self, uniid, addr_manager_table):
        # 优先分配已经分配过，并且uniid对的上的地址
        size = len(addr_manager_table)
        for i in range(size):
            if addr_manager_table[i].mapped == 1:
                # 如果掉线(或者刚初始化的，在前面的设置和获取环节都没有返回)，则需要重新分配
                if addr_manager_table[i].online == ONLINE_STATE_OFFLINE or addr_manager_table[i].online == ONLINE_STATE_INIT:
                    if uniid == addr_manager_table[i].uniid:
                        addr_manager_table[i].mapped = 1
                        addr_manager_table[i].online = ONLINE_STATE_WAIT_FOR_ACK
                        logging.info("alloc method 1 addr %d", addr_manager_table[i].addr)
                        return addr_manager_table[i].addr
                # 如果上位机显示在线，但是下位机回复了广播指令，说明下位机重启了，地址变成了广播地址，也需要重新分配地址
                elif addr_manager_table[i].online == ONLINE_STATE_ONLINE and uniid == addr_manager_table[i].uniid:
                    logging.info("Error: addr already allocated, but broadcast ack happened, maybe slave restarted, "
                            "clear the arcked flag and try to allocate the addr again")
                    # 能收到返回说明在线，需要置位
                    addr_manager_table[i].online = ONLINE_STATE_WAIT_FOR_ACK
                    addr_manager_table[i].acked = 0
                    return -1
        # 然后是未分配，的地址
        for i in range(size):
            if addr_manager_table[i].mapped == 0:
                addr_manager_table[i].mapped = 1
                addr_manager_table[i].online = ONLINE_STATE_WAIT_FOR_ACK 
                addr_manager_table[i].uniid = uniid
                self.uniid_changed = True
                logging.info("alloc method 2 addr %d", addr_manager_table[i].addr)
                return addr_manager_table[i].addr

        # 然后是已分配，但是uniid对不上，掉线的地址
        for i in range(size):
            if addr_manager_table[i].mapped == 1:
                if addr_manager_table[i].online == ONLINE_STATE_OFFLINE or addr_manager_table[i].online == ONLINE_STATE_INIT:
                    if addr_manager_table[i].uniid != uniid:
                        # 覆盖原来的uniid
                        addr_manager_table[i].uniid = uniid
                        addr_manager_table[i].mapped = 1
                        addr_manager_table[i].online = ONLINE_STATE_WAIT_FOR_ACK
                        self.uniid_changed = True
                        logging.info("alloc method 3 addr %d", addr_manager_table[i].addr)
                        return addr_manager_table[i].addr
        # 如果没有满足条件的地址，返回-1
        return -1

    def print_buff(self, buff):
        _len = len(buff)
        message = ""
        for i in range(_len):
            message += "0x%02X " % buff[i]
        logging.info(message)

    def crc8_cal(self, data, len):
        crc = 0
        for i in range(len):
            crc ^= data[i]
            for j in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ POLY
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def cal_pack_crc(self, package):
        crc_buff = [package.length, package.status, package.function_code]
        crc_buff += package.data
        return self.crc8_cal(crc_buff, len(crc_buff))

    def is_dev_type_valid(self, dev_type):
        if dev_type == DEV_TYPE_BTM or dev_type == DEV_TYPE_CLM or dev_type == DEV_TYPE_MB:
            return 1
        else:
            return 0

    def function_code_cb(self, package):
        function_code = package.function_code
        size = 0
        ack_data = None
        if function_code == CMD_SET_SLAVE_ADDR or function_code == CMD_GET_SLAVE_INFO \
            or function_code == CMD_ONLINE_CHECK or function_code == CMD_GET_ADDR_TABLE:
            ack_data = FcAckData(package.data[0], package.data[1], package.data[2:])
        
        ## 记录是否在loader的标记
        if ack_data is not None:
            addr_manager_table = dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET].addr_manager_table
            uniid = ack_data.uniid
            size = dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET].size
            for i in range(size):
                if addr_manager_table[i].uniid == uniid:
                    addr_manager_table[i].mode = ack_data.mode 
                    if ack_data.mode == MODE_LOADER:
                        logging.info("addr 0x%02X in loader mode", addr_manager_table[i].addr)
                    break

        if function_code == CMD_SET_SLAVE_ADDR:
            # ack_data = FcAckData(package.data[0], package.data[1], package.data[2:])
            if self.is_dev_type_valid(ack_data.dev_type):
                # logging.info("uniid:")
                # self.print_buff(ack_data.uniid)
                addr_manager_table = dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET].addr_manager_table
                size = dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET].size
                addr = package.slave_addr
                uniid = ack_data.uniid
                logging.info("received addr %d uniid %s", addr, uniid)
                for i in range(size):
                    logging.info("table[%d]: online %d addr %d uniid %s acked %d", \
                                 i, addr_manager_table[i].online, addr_manager_table[i].addr, addr_manager_table[i].uniid, addr_manager_table[i].acked)
                    if (addr_manager_table[i].online == ONLINE_STATE_INIT or addr_manager_table[i].online == ONLINE_STATE_WAIT_FOR_ACK) and \
                        addr_manager_table[i].addr == addr and \
                        addr_manager_table[i].uniid == uniid and \
                        addr_manager_table[i].acked == 0:
                        addr_manager_table[i].acked = 1
                        addr_manager_table[i].online = ONLINE_STATE_ONLINE
                        addr_manager_table[i].lost_cnt = 0
                        logging.info("addr %d acked", addr_manager_table[i].addr)
                        break
        elif function_code == CMD_GET_SLAVE_INFO:
            # ack_data = FcAckData(package.data[0], package.data[1], package.data[2:])
            if self.is_dev_type_valid(ack_data.dev_type):
                logging.info("dev_type: %d", ack_data.dev_type)
                logging.info("mode: %d", ack_data.mode)
                logging.info("uniid:")
                self.print_buff(ack_data.uniid)
                addr_manager_table = dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET].addr_manager_table
                size = dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET].size
                addr = self.addr_allocate(ack_data.uniid, addr_manager_table)
                logging.info("addr: %d", addr)

        elif function_code == CMD_ONLINE_CHECK:
            # ack_data = FcAckData(package.data[0], package.data[1], package.data[2:])
            if self.is_dev_type_valid(ack_data.dev_type):
                logging.info("uniid:")
                self.print_buff(ack_data.uniid)
                addr_manager_table = dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET].addr_manager_table
                size = dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET].size
                addr = package.slave_addr
                uniid = ack_data.uniid
                for i in range(size):
                    if addr_manager_table[i].addr == addr and \
                        addr_manager_table[i].uniid == uniid:
                        addr_manager_table[i].acked = 1
                        addr_manager_table[i].lost_cnt = 0
                        if addr_manager_table[i].online != ONLINE_STATE_ONLINE:
                            addr_manager_table[i].online = ONLINE_STATE_ONLINE
                            self.gcode.respond_info("BOX_MODIFY_TN_DATA ADDR=%d PART=state DATA=connect" % (addr_manager_table[i].addr))
                            self.gcode.run_script_from_command("BOX_MODIFY_TN_DATA ADDR=%d PART=state DATA=connect" % (addr_manager_table[i].addr))
                        logging.info("addr %d acked", addr_manager_table[i].addr)
                        break

        elif function_code == CMD_GET_ADDR_TABLE:
            # ack_data = FcAckData(package.data[0], package.data[1], package.data[2:])
            if self.is_dev_type_valid(ack_data.dev_type):
                logging.info("uniid:")
                self.print_buff(ack_data.uniid)
                addr_manager_table = dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET].addr_manager_table
                size = dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET].size
                addr = package.slave_addr
                uniid = ack_data.uniid
                for i in range(size):
                    if addr_manager_table[i].addr == addr:
                        addr_manager_table[i].uniid = uniid
                        addr_manager_table[i].mapped = 1
                        addr_manager_table[i].acked = 1
                        addr_manager_table[i].online = ONLINE_STATE_ONLINE 
                        addr_manager_table[i].lost_cnt = 0
                        self.uniid_changed = True 
                        logging.info("addr %d acked", addr_manager_table[i].addr)
                        break
        else:
            logging.info("unknown function code: %d", function_code)

        if self.uniid_changed:
            self.uniid_changed = False
            self.save_addr_table_uniids(dev_table_map_table[ack_data.dev_type - DEV_TYPE_INDEX_OFFSET])

    def data_handler(self, ret):
        # bytes经过迭代自动转换为int
        package = DataPackage(ret[0], ret[1], ret[2], ret[3], ret[4], [b for b in ret[5:-1]], ret[-1])
        if package.status == STATUS_OK:
            self.function_code_cb(package)
        else:
            logging.info("Error: status: %d" % package.status)

    def data_pack(self, slave_addr, cmd, data):
        _len = len(data)
        package = DataPackage(PACK_HEAD, slave_addr, _len + 3, STATUS_OK, cmd, data, 0x00)
        package.crc = self.cal_pack_crc(package)
        return package

    def send_package(self, package):
        data_send = bytes([package.slave_addr]) +  \
                    bytes([package.length]) + \
                    bytes([package.status]) + \
                    bytes([package.function_code]) + \
                    bytes(int(c) for c in package.data)
        timeout = cmd_timeout[package.function_code]
        # logging.info("data_send: %s", data_send)
        # logging.info("timeout: %f", timeout)
        ret = self._serial.cmd_send_data_with_response(data_send, timeout, False)
        if ret is None:
            logging.info("Error: no response")
            return
        logging.info("response is not null")
        # logging.info("type of ret is %s", type(ret))
        self.print_buff(ret)
        # ret 类型是bytes
        self.data_handler(ret)

    ###################### communication interface ########################
    def communication_get_addr_table(self, addr):
        package = self.data_pack(addr, CMD_GET_ADDR_TABLE, [])
        self.send_package(package)

    def communication_get_slave_info(self, broadcast_addr, send_data):
        package = self.data_pack(broadcast_addr, CMD_GET_SLAVE_INFO, send_data)
        self.send_package(package)

    def communication_set_slave_addr(self, broadcast_addr, addr, uniid):
        send_data = []
        send_data.append(addr)
        send_data += uniid
        # send_data.append(uniid)
        package = self.data_pack(broadcast_addr, CMD_SET_SLAVE_ADDR, send_data)
        self.send_package(package)

    def communication_online_check(self, addr):
        package = self.data_pack(addr, CMD_ONLINE_CHECK, [])
        self.send_package(package)
    
    def communication_loader_check(self, addr):
        package = self.data_pack(addr, CMD_LOADER_TO_APP, [0x01])
        self.send_package(package)

    def print_addr_manager_table(self, addr_manager_table):
        size = len(addr_manager_table)
        for i in range(size):
            log_message = ""
            log_message += "0x%02X, " % addr_manager_table[i].addr
            for j in range(len(addr_manager_table[i].uniid)):
                log_message += "0x%02X, " % addr_manager_table[i].uniid[j]
            log_message += "%d, %d, %d, %d, mode:%d" % (addr_manager_table[i].mapped, addr_manager_table[i].online,
                    addr_manager_table[i].acked, addr_manager_table[i].lost_cnt, addr_manager_table[i].mode)
            logging.info(log_message)

    ###################### logic interface ########################
    def get_addr_table(self, dev_table_map):
        logging.info("**************************** get addr table ****************************")
        size = dev_table_map.size
        # 如果底层有重试，上层就不需要重试了
        # 此文件中已经把底层重试关闭，所以此处在应用层添加重试
        for i in range(MAX_GET_TIMES):
            logging.info("**** get times %d" % (i + 1))
            for j in range(size):
                # if dev_table_map.addr_manager_table[j].acked == 0:
                if dev_table_map.addr_manager_table[j].online != ONLINE_STATE_ONLINE:
                    self.communication_get_addr_table(dev_table_map.addr_manager_table[j].addr)

            online_slave_num = 0
            for k in range(size):
                if dev_table_map.addr_manager_table[k].online == ONLINE_STATE_ONLINE:
                # if dev_table_map.addr_manager_table[k].acked == 1:
                    online_slave_num += 1
            logging.info("online slave num: %d", online_slave_num)
            # 如果已经获取所有从设备此信息则直接返回 不需要再尝试
            if online_slave_num == size:
                self.print_addr_manager_table(dev_table_map.addr_manager_table)
                logging.info("online slave num is max: %d", online_slave_num)
                return 
        self.print_addr_manager_table(dev_table_map.addr_manager_table)
    
    def set_addr_table(self, dev_table_map):
        logging.info("**************************** set addr table ****************************")
        addr_manager_table = dev_table_map.addr_manager_table
        size = dev_table_map.size
        broadcast_addr = dev_table_map.broadcast_addr
        logging.info("before set addr table")
        self.print_addr_manager_table(addr_manager_table) 
        for i in range(MAX_SET_TIMES):
            logging.info("**** set times %d" % (i + 1))
            mapped_cnt = 0
            for i in range(size):
                if addr_manager_table[i].mapped == 1:
                    if addr_manager_table[i].online == ONLINE_STATE_INIT:
                        mapped_cnt += 1
                        self.communication_set_slave_addr(broadcast_addr, addr_manager_table[i].addr, addr_manager_table[i].uniid)

            valid_slave_num = 0
            for k in range(size):
                if addr_manager_table[k].online == ONLINE_STATE_ONLINE:
                    valid_slave_num += 1
            logging.info("valid slave num: %d", valid_slave_num)
            # 如果所有设备都成功设置地址 不需要再尝试
            if valid_slave_num == mapped_cnt:
                self.print_addr_manager_table(addr_manager_table)
                logging.info("valid slave num is max: %d", valid_slave_num)
                return
        self.print_addr_manager_table(addr_manager_table) 

    def get_slave_info(self, dev_table_map):
        broadcast_addr = dev_table_map.broadcast_addr
        send_data = [broadcast_addr, broadcast_addr]
        online_slave_num = 0
        addr_manager_table = dev_table_map.addr_manager_table 
        size = dev_table_map.size
        logging.info("**************************** get slave info ****************************")
        for i in range(size):
            if addr_manager_table[i].online == ONLINE_STATE_ONLINE or addr_manager_table[i].online == ONLINE_STATE_WAIT_FOR_ACK:
                online_slave_num += 1

        ## 如果全都在线，或者已经处于等待地址回复的状态，则后面不用扫描
        if online_slave_num == size:
            logging.info("online slave num is max %d", online_slave_num)
            return 
        self.communication_get_slave_info(broadcast_addr, send_data)

    def set_slave_addr(self, dev_table_map):
        logging.info("**************************** set slave addr ****************************")
        addr_manager_table = dev_table_map.addr_manager_table
        size = dev_table_map.size
        broadcast_addr = dev_table_map.broadcast_addr
        for i in range(size):
            if addr_manager_table[i].mapped == 1:
                if addr_manager_table[i].online == ONLINE_STATE_WAIT_FOR_ACK:
                    self.communication_set_slave_addr(broadcast_addr, addr_manager_table[i].addr, addr_manager_table[i].uniid)

    def online_check(self, dev_table_map):
        logging.info("**************************** online check ****************************")
        lost_flag = 0
        mapped_exist = 0
        addr_manager_table = dev_table_map.addr_manager_table 
        size = dev_table_map.size
        for i in range(size):
            if addr_manager_table[i].mapped == 1:
                mapped_exist = 1
                addr_manager_table[i].lost_cnt += 1
                self.communication_online_check(addr_manager_table[i].addr)
                if addr_manager_table[i].lost_cnt > MAX_LOST_CNT:
                    addr_manager_table[i].acked = 0
                    if addr_manager_table[i].online != ONLINE_STATE_OFFLINE:
                        addr_manager_table[i].online = ONLINE_STATE_OFFLINE
                        self.gcode.respond_info("BOX_MODIFY_TN_DATA ADDR=%d PART=state DATA=disconnect" % (addr_manager_table[i].addr))
                        self.gcode.run_script_from_command("BOX_MODIFY_TN_DATA ADDR=%d PART=state DATA=disconnect" % (addr_manager_table[i].addr))
                    logging.info("Error: addr %.2X offline", addr_manager_table[i].addr)
                    lost_flag = 1
        if mapped_exist == 1:
            self.print_addr_manager_table(addr_manager_table)
            if lost_flag == 0:
                logging.info("***************** all online ********************")
    def loader_check(self, dev_table_map):
        logging.info("**************************** loader check ****************************")
        addr_manager_table = dev_table_map.addr_manager_table 
        size = dev_table_map.size
        for i in range(size):
            # for test
            # addr_manager_table[i].mode = MODE_LOADER
            if addr_manager_table[i].mode == MODE_LOADER:
                ## 如果在loader模式需要使其进入app模式
                self.communication_loader_check(BROADCAST_ADDR)
                return True 
        return False 
    ###################### reg process ##########################
    def reg_auto_addr_get(self):
        self.reactor.register_callback(self.process_get_addr_table)
    def reg_auto_addr_set(self):
        self.print_stats = self.printer.lookup_object('print_stats')
        self.reactor.register_callback(self.process_set_addr_table)
    def reg_auto_addr_process(self):
        # self.reactor.register_callback(self.process_set_slave_addr)
        # self.reactor.register_callback(self.process_online_check)
        # self.reactor.register_callback(self.process_loader_check)
        self.reactor.register_callback(self.process_all)
        pass


    ###################### process ##########################
    def process_get_addr_table(self, eventtime):
        for i in range(len(dev_table_map_table)):
            dev_table_map = dev_table_map_table[i]
            self.get_addr_table(dev_table_map)
        # self.get_finished = True
        logging.info("get addr table finished")
        self.printer.send_event("auto_addr:get_finished")

    def process_set_addr_table(self, eventtime):
        for i in range(len(dev_table_map_table)):
            dev_table_map = dev_table_map_table[i]
            ## 根据上次保存的地址列表，设置设备地址
            self.set_addr_table(dev_table_map)
            ## 对于设置不成功的设备，重新获取
            self.get_addr_table(dev_table_map)
        # self.get_finished = True
        logging.info("set && get addr table finished")
        self.printer.send_event("auto_addr:set_finished")

    def process_set_slave_addr(self, eventtime):
        while True:
            if self.printer.is_shutdown():
                return
            logging.info("set slave addr")
            for i in range(len(dev_table_map_table)):
                dev_table_map = dev_table_map_table[i]
                self.get_slave_info(dev_table_map)
                self.set_slave_addr(dev_table_map)
            self.reactor.pause(self.reactor.monotonic() + 3.)

    def process_online_check(self, eventtime):
        while True:
            time_interval = 1.5
            if self.print_stats.state == "printing" or self.print_stats.state == "pause":
                time_interval = 10
            if self.printer.is_shutdown():
                return
            logging.info("online check")
            for i in range(len(dev_table_map_table)):
                dev_table_map = dev_table_map_table[i]
                self.online_check(dev_table_map)
            self.reactor.pause(self.reactor.monotonic() + time_interval)
    def process_loader_check(self, eventtime):
        while True:
            time_interval = 2.0 
            if self.print_stats.state == "printing" or self.print_stats.state == "pause":
                time_interval = 10
            if self.printer.is_shutdown():
                return
            logging.info("loader check")
            for i in range(len(dev_table_map_table)):
                dev_table_map = dev_table_map_table[i]
                ## 一个周期只需要发送一次即可
                if True == self.loader_check(dev_table_map):
                    break
            self.reactor.pause(self.reactor.monotonic() + time_interval)

    def process_all(self, eventtime):
        box_obj = self.printer.lookup_object('box',None)
        while True:
            time_interval = 1.0 
            if self.print_stats.state == "printing" or self.print_stats.state == "pause":
                time_interval = 10
            if self.printer.is_shutdown():
                return

            if box_obj and not all(box_obj.box_action.heart_process_enable):
                # 手动进料过程暂时关闭地址分配
                logging.info("box heart process not enable")
                self.reactor.pause(self.reactor.monotonic() + 10)
                continue
            logging.info("set slave addr")
            for i in range(len(dev_table_map_table)):
                dev_table_map = dev_table_map_table[i]
                self.get_slave_info(dev_table_map)
                self.set_slave_addr(dev_table_map)

            logging.info("online check")
            for i in range(len(dev_table_map_table)):
                dev_table_map = dev_table_map_table[i]
                self.online_check(dev_table_map)

            logging.info("loader check")
            for i in range(len(dev_table_map_table)):
                dev_table_map = dev_table_map_table[i]
                ## 一个周期只需要发送一次即可
                if True == self.loader_check(dev_table_map):
                    break
            self.reactor.pause(self.reactor.monotonic() + time_interval)
