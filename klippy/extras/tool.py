import re, os, logging, threading
from subprocess import call
import json, random, time

def send(msg, data={}):
    pipeFilePath = "/usr/data/creality/userdata/config/pipe.json"
    try:
        if not os.path.exists(pipeFilePath):
            call("touch %s" % pipeFilePath, shell=True)
            os.chmod(pipeFilePath, 0o700)
        # net_state = call("ping -c 2 -w 2 api.crealitycloud.com > /dev/null 2>&1", shell=True)
        # if net_state:
        #     return
        ret = re.findall('key(\d+)', msg)
        if ret:
            msg = "key%s" % ret[0]
            if os.path.getsize(pipeFilePath) > 0:
                random_float = random.uniform(0.1, 1)
                time.sleep(random_float)

            result = compress_key701(msg, data)
            if result:
                data = result
            if os.path.getsize(pipeFilePath) == 0:
                send_data = {"reqId": str(int(time.time()*1000)), "dn": "00000000000000", "code": msg, "data": data}
                with open(pipeFilePath, "w") as f:
                    f.write(json.dumps(send_data))
                    f.flush()
    except Exception as err:
        logging.error("reportInformation err:%s" % err)

def reportInformation(msg, data={}):
    t = threading.Thread(target=send, args=(msg, data))
    t.start()

def compress_key701(code, data):
    if code == "key701":
        try:
            data = data.get("jobs", [])[0] if data.get("jobs", []) else {}
            metadata = data.get("metadata", {})
            model_info = metadata.get("model_info", {})
            result = "%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s" % (
                data.get("end_time", 0), data.get("filament_used", 0), data.get("filename", ""), data.get("print_duration", 0),
                data.get("start_time", 0), data.get("status", ""), data.get("error_msg", ""), data.get("total_duration", 0), 
                metadata.get("estimated_time", 0), metadata.get("filament_total", 0), metadata.get("filament_weight_total", 0), metadata.get("first_layer_bed_temp", 0), metadata.get("first_layer_extr_temp", 0),
                metadata.get("first_layer_height", 0), metadata.get("gcode_end_byte", 0), metadata.get("gcode_start_byte", 0), metadata.get("layer_count", 0),
                metadata.get("layer_height", 0), metadata.get("modified", 0), metadata.get("object_height", 0), metadata.get("size", 0),
                metadata.get("slicer", ""), metadata.get("slicer_version", ""), model_info.get("MaterialType", ""), model_info.get("MaterialName", ""),
                model_info.get("MAXX", 0), model_info.get("MAXY", 0), model_info.get("MAXZ", 0), model_info.get("MINX", 0), model_info.get("MINY", 0), model_info.get("MINZ", 0),
            )
            # "end_time|filament_used|filename|print_duration|start_time|status|error_msg|total_duration|estimated_time|filament_total|filament_weight_total|first_layer_bed_temp|first_layer_extr_temp|first_layer_height|gcode_end_byte|gcode_start_byte|layer_count|layer_height|modified|object_height|size|slicer|slicer_version|MaterialType|MaterialName|MAXX|MAXY|MAXZ|MINX|MINY|MINZ"
            return result
        except Exception as err:
            logging.exception(err)
    return None