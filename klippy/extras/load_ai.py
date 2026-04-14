import logging
import http.client
from email.mime.base import MIMEBase
from email.encoders import encode_base64
import os
import subprocess
import json
import re
import copy
import threading
from subprocess import check_output
from extras.base_info import base_dir
    
class LoadAI:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.flowdetect_img_dir = os.path.join(base_dir, "ai_image/flowdetect_img")
        self.user_print_refer_path = os.path.join(base_dir, "creality/userdata/config/user_print_refer.json")
        self.pic_dir = config.get('path', self.flowdetect_img_dir)
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = None
        self.box_action = None
        self.printer.register_event_handler('klippy:ready', self.find_objs)
        self.gcode.register_command(
            "LOAD_AI_T_CMD_TEST", self.cmd_LOAD_AI_T_CMD_TEST)
        self.gcode.register_command(
            "LOAD_AI_NOZZLE_CAM_POWER_ON", self.cmd_LOAD_AI_NOZZLE_CAM_POWER_ON)
        self.gcode.register_command(
            "LOAD_AI_NOZZLE_CAM_POWER_OFF", self.cmd_LOAD_AI_NOZZLE_CAM_POWER_OFF)
        self.gcode.register_command(
            "LOAD_AI_SET_AI_CONTROL_PREFER", self.cmd_LOAD_AI_SET_AI_CONTROL_PREFER)
        self.gcode.register_command(
            "LOAD_AI_DEAL", self.cmd_LOAD_AI_DEAL)
        self.gcode.register_command(
            "LOAD_AI_DETECT_WASTE", self.cmd_LOAD_AI_DETECT_WASTE)
        self.gcode.register_command(
            "LOAD_AI_GET_STATUS", self.cmd_LOAD_AI_GET_STATUS)
        # ai_control_values = self.extract_ai_control_prefer_values(self.user_print_refer_path, ["switch", "wasteSwitch"])
        # self.ai_switch = ai_control_values.get("switch") if ai_control_values else None
        # self.ai_waste_switch = ai_control_values.get("wasteSwitch") if ai_control_values else None
        # self.cx_ai_engine_status = {
        #     "ai_switch": self.ai_switch,
        #     "ai_waste_switch": self.ai_waste_switch,
        #     "command_type": "",
        #     "command": "",
        #     "command_description": "",
        #     "stderr": "",
        #     "ai_results": "",
        #     "max_re_prob": 0.0,
        #     "normalized_total_area": 0.0,
        #     "output_width": 0,
        #     "output_height": 0
        # }
        self.cx_ai_engine_status = {}
        self.ai_switch = 0
        self.ai_waste_switch = 0
        self.result = ""
        self.stderr = ""
        self.t_command_count = 2

    def find_objs(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.box_action = self.printer.lookup_object('box').box_action

    def extract_ai_control_prefer_values(self, json_file, keys):
        # 读取 JSON 文件内容
        try:
            with open(json_file, 'r') as file:
                data = json.load(file)
        except Exception as e:
            logging.error(f"Error opening or reading the JSON file: {e}")
            return None

        # 查找 ai_control 中的指定键值
        values = {}
        for key in keys:
            if 'ai_control' in data and key in data['ai_control']:
                values[key] = data['ai_control'][key]
            else:
                logging.warning(f"Key '{key}' not found in 'ai_control'")
                values[key] = None

        return values

    def nozzle_cam_power_on(self):
        try:
            logging.info("nozzle_cam_power.sh on")
            result_capture = subprocess.run(['nozzle_cam_power.sh', 'on'], capture_output=True, text=True)
            # 打印 ai_capture 的输出
            logging.info(result_capture.stdout)
            logging.info(result_capture.stderr)

        except Exception as e:
            logging.info(f"Error: {e}")

    def nozzle_cam_power_off(self):
        try:
            logging.info("nozzle_cam_power.sh off")
            result_capture = subprocess.run(['nozzle_cam_power.sh', 'off'], capture_output=True, text=True)
            # 打印 ai_capture 的输出
            logging.info(result_capture.stdout)
            logging.info(result_capture.stderr)

        except Exception as e:
            logging.info(f"Error: {e}")

    def ai_capture(self):
        try:
            logging.info("ai_capture 1")
            # 运行 ai_capture 命令并捕获输出
            result_capture = subprocess.run(['ai_capture', '1'], capture_output=True, text=True)

            # 打印 ai_capture 的输出（可选）
            logging.info(result_capture.stdout)
            logging.info(result_capture.stderr)

            return result_capture.stdout  # 返回标准输出
        except Exception as e:
            logging.info(f"Error: {e}")
            return None
    
    def remove_files(self, file_path):
        command = 'rm -rf ' + file_path
        try:
            # Execute the command
            subprocess.run(command, shell=True, check=True)
            print("Files removed successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Error occurred: {e}")

    def calculate_overlap_area(self, rectangles):
        """使用扫描线算法计算矩形的重叠区域总面积"""
        try:
            # 事件定义为 (x, opening/closing, y1, y2)
            OPEN, CLOSE = 1, -1
            events = []

            # 为所有矩形生成事件
            for (x1, y1, x2, y2) in rectangles:
                events.append((x1, OPEN, y1, y2))
                events.append((x2, CLOSE, y1, y2))

            # 按 x 坐标对事件进行排序
            events.sort()

            # 使用扫描线计算区域面积
            def calc_area(active_y_intervals):
                """计算当前活动的 y 区间覆盖的 y 长度"""
                total = 0
                current_y = -1
                for (y1, y2) in active_y_intervals:
                    current_y = max(current_y, y1)
                    total += max(0, y2 - current_y)
                    current_y = max(current_y, y2)
                return total

            active_intervals = []
            last_x = 0
            total_area = 0

            for x, typ, y1, y2 in events:
                logging.info(f"Processing event: x={x}, typ={typ}, y1={y1}, y2={y2}")

                # 计算 last_x 和当前 x 之间区域的面积
                area = calc_area(active_intervals) * (x - last_x)
                logging.info(f"Area between x={last_x} and x={x}: {area}")
                total_area += area
                last_x = x

                if typ == OPEN:
                    active_intervals.append((y1, y2))
                    active_intervals.sort()
                    logging.info(f"Added interval: {(y1, y2)}, active_intervals={active_intervals}")
                elif typ == CLOSE:
                    try:
                        active_intervals.remove((y1, y2))
                        logging.info(f"Removed interval: {(y1, y2)}, active_intervals={active_intervals}")
                    except ValueError:
                        logging.warning(f"Warning: Interval {(y1, y2)} not found in {active_intervals}")

            return total_area

        except Exception as e:
            logging.error(f"An error occurred in calculate_overlap_area: {e}")
            logging.exception("Exception details:")
            return 0

    def process_waste_ai_detect_result(self, result_stdout_str):
        cnt_pattern = r"ai detection completed, cnt = (\d+)"
        result_pattern = r"(\d+)\s+(\d+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)"
        output_size_pattern = r"output width:\s+(\d+),\s+height:\s+(\d+)"
        
        # 获取AI识别个数
        cnt_match = re.search(cnt_pattern, result_stdout_str)
        if cnt_match:
            ai_size_int = int(cnt_match.group(1))
            ai_results = []

            logging.info("ai_size_int:%d", ai_size_int)

            # 提取输出图片的宽度和高度
            output_size_match = re.search(output_size_pattern, result_stdout_str)
            if output_size_match:
                output_width = int(output_size_match.group(1))
                output_height = int(output_size_match.group(2))
            else:
                output_width = 0
                output_height = 0

            # 提取检测结果
            data_start = result_stdout_str.split("num / re_label / re_prob / re_obj_rect_x / re_obj_rect_y / re_obj_rect_width / re_obj_rect_height")[-1]
            data_start = data_start.strip().splitlines()

            rectangles = []
            max_re_prob = 0

            for line in data_start:
                match = re.match(result_pattern, line)
                if match:
                    ai_result = {
                        "num": int(match.group(1)),
                        "re_label": int(match.group(2)),
                        "re_prob": float(match.group(3)),
                        "re_obj_rect_x": float(match.group(4)),
                        "re_obj_rect_y": float(match.group(5)),
                        "re_obj_rect_width": float(match.group(6)),
                        "re_obj_rect_height": float(match.group(7))
                    }

                    # 更新最大 re_prob 值
                    if ai_result["re_prob"] > max_re_prob:
                        max_re_prob = ai_result["re_prob"]

                    # 将矩形的坐标转换为 (x1, y1, x2, y2) 格式，并添加到列表中
                    rectangles.append((
                        ai_result["re_obj_rect_x"],
                        ai_result["re_obj_rect_y"],
                        ai_result["re_obj_rect_x"] + ai_result["re_obj_rect_width"],
                        ai_result["re_obj_rect_y"] + ai_result["re_obj_rect_height"]
                    ))
                    
                     # 移除不需要的键
                    del ai_result["re_obj_rect_x"]
                    del ai_result["re_obj_rect_y"]
                    del ai_result["re_obj_rect_width"]
                    del ai_result["re_obj_rect_height"]

                    re_prob = ai_result["re_prob"]
                    # ai_results.append(re_prob)
                    ai_results.append(ai_result)
            
            ai_results = json.dumps(ai_results)
            # 计算所有矩形的重叠面积
            total_area = self.calculate_overlap_area(rectangles)
            # 面积归一化
            if output_width and output_height:
                normalized_total_area = total_area / (output_width * output_height)
                logging.info("Total Area: %f, Overlap Normalized Total Area: %f", total_area, normalized_total_area)
            else:
                logging.warning("Output dimensions are not available. Cannot normalize total area.")
                normalized_total_area = 0
            
            # 面积归一化，最大面积为1
            result_dict = {
                "ai_results": ai_results,
                "max_re_prob": max_re_prob if normalized_total_area > 0.35 else 0.0,
                "normalized_total_area": normalized_total_area,
                "output_width": output_width,
                "output_height": output_height
            }

            return result_dict

        return None

    def execute_toolhead_ai_waste_management(self):
        logging.info(f"execute_toolhead_ai_waste_management start: ai_switch={self.ai_switch}, ai_waste_switch={self.ai_waste_switch}, t_command_count={self.t_command_count}")
        self.gcode.respond_info("ai_switch = %d, ai_waste_switch = %d \n" % (self.ai_switch, self.ai_waste_switch))
        if self.t_command_count < 2:
            self.t_command_count += 1
            return
        # T指令达到2次后检测
        self.t_command_count = 0  # 重置计数器
        if int(self.ai_waste_switch) == 1:  # AI检测开启
            self.nozzle_cam_power_on()  # 进料前给喷头上电 LOAD_AI_NOZZLE_CAM_POWER_ON
            self.box_action.go_to_extrude_pos() # BOX_GO_TO_EXTRUDE_POS
            self.toolhead.wait_moves() # M400
            self.gcode.run_script_from_command("G91")
            self.gcode.run_script_from_command("G1 X-2 F12000")
            self.toolhead.wait_moves() # M400
            self.gcode.run_script_from_command("G1 X9 F12000")
            self.toolhead.wait_moves() # M400
            self.reactor.pause(self.reactor.monotonic() + 2)
            self.gcode.respond_info("WILL LOAD_AI_DEAL")
            # LOAD_AI_DEAL
            self.gcode.run_script_from_command("LOAD_AI_DETECT_WASTE")  # 废料槽检测
            self.nozzle_cam_power_off()  # 关灯 LOAD_AI_NOZZLE_CAM_POWER_OFF
            self.gcode.run_script_from_command("G1 X-7")
            self.toolhead.wait_moves() # M400
            self.gcode.run_script_from_command("G90")
            # self.gcode.run_script_from_command("BOX_NOZZLE_CLEAN")  # 擦嘴
            # self.box_action.move_to_safe_pos()  # 去安全位置 BOX_MOVE_TO_SAFE_POS

        logging.info(f"execute_toolhead_ai_waste_management end!!!")

    def cmd_LOAD_AI_T_CMD_TEST(self, gcmd):
        """
        根据指定的温度和T编号测试T指令换料擦嘴流程
        示例：LOAD_AI_T_CMD_TEST TEMP=220 TCMD_NUM=0
        """
        self.t_command_count = 2 # 立即触发废料槽检测
        logging.info("LOAD_AI_T_CMD_TEST gcmd: %s"% gcmd.get_command_parameters())
        temp = gcmd.get_int("TEMP", minval=180, maxval=300, default=220)
        tcmd_num = gcmd.get_int("TCMD_NUM", minval=0, maxval=16, default=0)
        self.gcode.run_script_from_command("BOX_GO_TO_EXTRUDE_POS")
        self.gcode.run_script_from_command(f"M109 S{temp}")
        self.gcode.run_script_from_command(f"T{tcmd_num}")
        self.gcode.run_script_from_command("BOX_GO_TO_EXTRUDE_POS")
        self.gcode.run_script_from_command("M106 P0 S255")
        self.gcode.run_script_from_command("M106 P2 S255")
        self.gcode.run_script_from_command("M109 S140")
        self.gcode.run_script_from_command("M106 P0 S0")
        self.gcode.run_script_from_command("M106 P2 S0")
        self.gcode.run_script_from_command("BOX_NOZZLE_CLEAN")
        self.gcode.run_script_from_command("M109 S0")
        self.gcode.run_script_from_command("G90")
        self.gcode.run_script_from_command("G1 X150 Y150 F7800")

    def cmd_LOAD_AI_NOZZLE_CAM_POWER_ON(self, gcmd):
        self.nozzle_cam_power_on()
        
    def cmd_LOAD_AI_NOZZLE_CAM_POWER_OFF(self, gcmd):
        self.nozzle_cam_power_off()

    def cmd_LOAD_AI_SET_AI_CONTROL_PREFER(self, gcmd):
        logging.info("gcmd: %s"% gcmd.get_command_parameters())
        self.ai_switch = gcmd.get_int("SWITCH", minval=0, maxval=1, default=self.ai_switch)
        self.ai_waste_switch = gcmd.get_int("WASTE_SWITCH", minval=0, maxval=1, default=self.ai_waste_switch)
        logging.info("ai_switch: %d, ai_waste_switch: %d" % (self.ai_switch, self.ai_waste_switch))
        # ai_control_values = self.extract_ai_control_prefer_values(self.user_print_refer_path, ["switch", "wasteSwitch"])
        # self.ai_switch = ai_control_values.get("switch") if ai_control_values else None
        # self.ai_waste_switch = ai_control_values.get("wasteSwitch") if ai_control_values else None
        self.cx_ai_engine_status = {
            "ai_switch": self.ai_switch,
            "ai_waste_switch": self.ai_waste_switch,
            "command_type": "",
            "command": "",
            "command_description": "",
            "stderr": "",
            "ai_results": "",
            "max_re_prob": 0.0,
            "normalized_total_area": 0.0,
            "output_width": 0,
            "output_height": 0
        }
        logging.info("LOAD_AI_SET_AI_CONTROL_PREFER:%s" % self.cx_ai_engine_status)

    def cmd_LOAD_AI_DEAL(self, gcmd):
        # 加载AI上传图片
        try:
            # ip = self.get_ip()
            # if not ip:
            #     self.gcode.respond_info("LOAD_AI_DEAL net error")
            #     return
            # self.nozzle_cam_power_on()
            # self.reactor.pause(self.reactor.monotonic() + 1)
            self.ai_capture()
            self.reactor.pause(self.reactor.monotonic() + 2)
            filename = self.find_latest_photo()
            if not filename or not os.path.exists(filename):
                # 关灯
                # self.nozzle_cam_power_off()
                self.gcode.respond_info("LOAD_AI_DEAL photo error, filename is %s" % filename)
                return
            files = {'file': filename}
            response = self.send_post_request(files)
            logging.info("LOAD_AI_DEAL:%s" % response)
            self.gcode.respond_info("LOAD_AI_DEAL:%s" % response)
            logging.info("files:%s",files)
            self.remove_files(filename)
        except Exception as e:
            logging.exception(e)
        # 关灯
        # self.nozzle_cam_power_off()
    def ai_engine_capture(self, cmd):
        logging.info(f"Executing command: {cmd}")
        try:
            # 运行命令，捕获标准输出和标准错误
            process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
            self.result, self.stderr = process.communicate()

            # 检查返回码
            if process.returncode != 0:
                logging.error(f"Command '{cmd}' failed with return code {process.returncode}")
                if self.stderr:
                    logging.error(f"Error output: {self.stderr.strip()}")
                else:
                    logging.error("No error output captured.")

            logging.info(f"Command '{cmd}' returned output: {self.result.strip()}")
        except subprocess.CalledProcessError as e:
            logging.error(f"Command '{e.cmd}' returned non-zero exit status {e.returncode}")
            logging.error(f"Error output: {e.stderr.strip() if e.stderr else 'No error output captured.'}")
        except Exception as e:
            logging.error(f"An unexpected error occurred: {str(e)}")

    def execute_ai_waste_detection(self):
        # ai_control_values = self.extract_ai_control_prefer_values(self.user_print_refer_path, ["switch", "wasteSwitch"])
        # self.ai_switch = ai_control_values.get("switch") if ai_control_values else None
        # self.ai_waste_switch = ai_control_values.get("wasteSwitch") if ai_control_values else None
        # # self.gcode.respond_info(f"switch: {ai_switch}")
        # if ai_switch != 1:
        #     # self.gcode.respond_info(f"switch: {ai_switch}")
        #     return
        cmd = f"ai_engine 1 5 --user_data_dir={base_dir}"
        json_output = {
            "ai_switch": self.ai_switch,
            "ai_waste_switch": self.ai_waste_switch,
            "command_type": "ai_engine",
            "command": cmd,
            "command_description": "waste",
            "stderr": "",
            "ai_results": [],
            "max_re_prob": 0.0,
            "normalized_total_area": 0.0,
            "output_width": 0,
            "output_height": 0
        }

        try:
            self.result = {}
            self.stderr = ""
            # 启动后台线程执行命令
            background_thread = threading.Thread(target=self.ai_engine_capture, args=(cmd,))
            background_thread.start()

            # 等待结果
            for _ in range(100):
                if self.result:
                    break
                self.reactor.pause(self.reactor.monotonic() + 0.1)
            else:
                logging.info("run cmd_LOAD_AI_DETECT_WASTE failed: timeout")
                return
            
            # 处理结果
            if self.stderr:
                json_output["stderr"] = self.stderr
            else:
                ai_results = self.process_waste_ai_detect_result(self.result)
                if ai_results is not None:
                    json_output["ai_results"] = ai_results["ai_results"]
                    json_output["max_re_prob"] = ai_results["max_re_prob"]
                    json_output["normalized_total_area"] = ai_results["normalized_total_area"]
                    json_output["output_width"] = ai_results["output_width"]
                    json_output["output_height"] = ai_results["output_height"]

             # 更新状态并记录信息
            self.cx_ai_engine_status = copy.deepcopy(json_output)
            json_output["stdout"] = self.result
            json_output_str = json.dumps(json_output, indent=4)
            logging.info(json_output_str)

            # 打印 ai_capture 的输出（可选）
            # self.gcode.respond_info(json_output_str)
            return self.result  # 返回标准输出
        except Exception as e:
            json_output["stderr"] = str(e)
            self.cx_ai_engine_status = json_output
            json_output_str = json.dumps(json_output, indent=4)
            logging.info(json_output_str)
            # self.gcode.respond_info(json_output_str)
            return None

    # AI 废料槽检测
    def cmd_LOAD_AI_DETECT_WASTE(self,gcmd):  
        return self.execute_ai_waste_detection()

    def create_multipart_form_data(self, files):
        """
        创建并返回multipart/form-data的字节串，用于HTTP POST请求体。
        """
        boundary = '---------------------------' + os.urandom(16).hex()
        body = []

        for name, filepath in files.items():
            with open(filepath, 'rb') as f:
                part = MIMEBase('application', 'octet-stream')
                part.set_payload(f.read())
                encode_base64(part)

                # 添加头部信息（作为字节串列表）
                # waste_ip_时间戳
                # upload_new_filename = "waste_" + ip + "_" + os.path.basename(filepath)
                part_headers = [
                    f'Content-Disposition: form-data; name="{name}"; filename="{os.path.basename(filepath)}"',
                    # f'Content-Disposition: form-data; name="{name}"; filename="{upload_new_filename}"',
                    f'Content-Type: {part.get_content_type()}',
                    f'Content-Transfer-Encoding: base64'
                ]
                headers_bytes = [header.encode('utf-8') + b'\r\n' for header in part_headers]

                # 添加到body中
                body.append(f'--{boundary}\r\n'.encode('utf-8'))
                body.extend(headers_bytes)
                body.append(b'\r\n')
                # 添加base64编码的内容（已经是字节串）
                body.append(part.get_payload(decode=True))
                body.append(b'\r\n')

                # 添加最后的边界（带有两个破折号）
        body.append(f'--{boundary}--\r\n'.encode('utf-8'))

        # 将所有部分连接成一个字节串
        return b''.join(body), boundary

    def send_post_request(self, files):
        # 假设URL格式是 http://hostname/path
        # hostname, path = url.split('/', 2)[2], '/' + '/'.join(url.split('/')[3:])
        hostname, path = "http://172.23.88.101:38765", "upload/"
        # 创建multipart/form-data体和边界
        body, boundary = self.create_multipart_form_data(files)

        # 创建HTTP连接并发送请求
        conn = http.client.HTTPConnection("172.23.88.101", 38765)
        headers = {
            'Content-Type': f'multipart/form-data; boundary={boundary}',
            'Content-Length': str(len(body))  # 设置内容长度
        }
        conn.request('POST', path, body, headers)

        # 获取响应
        response = conn.getresponse()
        print(f'Status: {response.status}, Reason: {response.reason}')
        resp_text = response.read().decode('utf-8')
        print(resp_text)  # 假设响应是文本

        # 关闭连接
        conn.close()
        return resp_text

    def find_latest_photo(self):
        """
        查找指定目录中最新的照片文件。

        :param directory: 包含照片的目录路径
        :return: 最新照片文件的完整路径，如果没有找到照片则返回None
        """
        latest_photo_path = None
        latest_photo_mtime = None

        # 遍历目录中的所有文件和文件夹
        for root, dirs, files in os.walk(self.pic_dir):
            for file in files:
                # 检查文件扩展名，以确定它是否是图片
                if file.lower().endswith(('.jpg', '.jpeg', '.png', '.gif', '.bmp')):
                    file_path = os.path.join(root, file)
                    mtime = os.path.getmtime(file_path)

                    # 如果这是第一个找到的图片，或者比当前已知的最新图片更新
                    if latest_photo_mtime is None or mtime > latest_photo_mtime:
                        latest_photo_path = file_path
                        latest_photo_mtime = mtime

        return latest_photo_path
    
    def cmd_LOAD_AI_GET_STATUS(self,gcmd):  
        # 测试数据初始化
        detection_results = [
            [0, 0, 0.931467, 1.0, 0.0, 4.0, 4.0],
            [1, 0, 0.831467, 3.0, 1.0, 3.0, 4.0],
            [2, 0, 0.731467, 0.0, 3.0, 7.0, 3.0]
        ]
        cnt = len(detection_results)
        # 将数据转换回字符串格式
        detection_results_str = "\n".join(
            "\t".join(map(str, result)) for result in detection_results
        )
        
        # 接口功能测试
        self.cx_ai_engine_status = {
            "ai_switch": 1,
            "ai_waste_switch": 1,
            "command_type": "ai_engine",
            "command": f"ai_engine 1 5 --user_data_dir={base_dir}",
            "command_description": "waste",
            "stderr": "",
            "ai_results": (
                "cam_type=1\n"
                "mode=5\n"
                "debug=0\n"
                f"user_data_dir={base_dir}\n"
                "gcode_path=\n"
                "z_height=0.000000\n"
                "ParseParamFile model_str_=F008\n"
                "ParseParamFile sys_version_=1.1.0.15\n"
                "the pid is alive...!\n"
                "flag = 0\n"
                f"input = {base_dir}/ai_image/sub_capture.bmp\n"
                "AI_upload_mode = 1\n"
                "{\"reqId\":\"1722419562737\",\"dn\":\"00000000000000\",\"code\":\"key609\",\"data\":\"0.000000|1722419562.736825|/usr/data/ai_image/ai_property/F008-waste-2024_7_31_17_52_42.jpg\\n\"}\n"
                "output width: 1600, height: 1200\n"
                f"output = {base_dir}/ai_image/sub_processed_ai_waste_mode.jpg\n"
                f"ai detection completed, cnt = {cnt}\n"
                "num / re_label / re_prob / re_obj_rect_x / re_obj_rect_y / re_obj_rect_width / re_obj_rect_height\n"
                f"{detection_results_str}"
            ),
            "max_re_prob": 0.0,
            "normalized_total_area": 0.0,
            "output_width": 0,
            "output_height": 0
        }
        result_stdout = self.cx_ai_engine_status["ai_results"]
        ai_results = self.process_waste_ai_detect_result(result_stdout)
        if ai_results is not None:
            self.cx_ai_engine_status["ai_results"] = ai_results["ai_results"]
            self.cx_ai_engine_status["max_re_prob"] = ai_results["max_re_prob"]
            self.cx_ai_engine_status["normalized_total_area"] = ai_results["normalized_total_area"]
            self.cx_ai_engine_status["output_width"] = ai_results["output_width"]
            self.cx_ai_engine_status["output_height"] = ai_results["output_height"]
        json_output_str = json.dumps(self.cx_ai_engine_status, indent=4)  
        logging.info(json_output_str)
        
    def get_status(self, eventtime):
        # ai_control_values = self.extract_ai_control_prefer_values(self.user_print_refer_path, ["switch", "wasteSwitch"])
        # self.ai_switch = ai_control_values.get("switch") if ai_control_values else None
        # self.ai_waste_switch = ai_control_values.get("wasteSwitch") if ai_control_values else None
        # self.cx_ai_engine_status["ai_switch"] = self.ai_switch
        return self.cx_ai_engine_status

def load_config(config):
    return LoadAI(config)