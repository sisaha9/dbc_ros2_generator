import pathlib
import yaml
from pathlib import Path
from dataclasses import dataclass
from typing import List
import cantools
import math
import bisect
from distutils.dir_util import copy_tree
from distutils.file_util import copy_file

supported_int_bit_lengths = [8, 16, 32, 64]
supported_float_bit_lengths = [32, 64]
# dbc_fp = 'New_Eagle_DBW_3.4.dbc'
# yaml_fp = 'New_Eagle_DBW_3.4.yaml'
dbc_fp = 'IAC-CAN1-INDY-V9.dbc'
yaml_fp = 'IAC-CAN1-INDY-V9.yaml'
template_pkg_dir = Path('template_package/')
out_pkg_dir = Path('generated/')
out_pkg_msg_dir = out_pkg_dir / 'raptor_dbw_msgs'
out_pkg_can_dir = out_pkg_dir / 'raptor_dbw_can'
out_msg_dir = out_pkg_msg_dir / 'msgs'
out_msg_cmake = out_pkg_msg_dir / 'CMakeLists.txt'
out_dbc_dir = out_pkg_can_dir / 'config'
out_launch_dir = out_pkg_can_dir / 'launch'
out_dispatch_file = out_pkg_can_dir / 'include' / 'raptor_dbw_can' / 'dispatch.hpp'
out_can_header = out_pkg_can_dir / 'include' / 'raptor_dbw_can' / 'raptor_dbw_can.hpp'
out_can_source = out_pkg_can_dir / 'src' / 'raptor_dbw_can.cpp'

@dataclass
class DbcSignal:
    name: str
    data_type: str
    
@dataclass
class DbcMessage:
    name: str
    id: str
    comm_type: List[str]
    signals: List[DbcSignal]

def process_interval(bit_lengths, nbits):
    i =  bisect.bisect_right(bit_lengths, nbits)
    interval = bit_lengths[i-1:i+1]
    if len(interval) == 0:
        return bit_lengths[0]
    elif len(interval) == 1:
        return bit_lengths[-1]
    else:
        if nbits == interval[0]:
            return nbits
        else:
            return interval[1]

def return_bits(signal_length, is_int):
    nbits = (math.floor((signal_length - 1) / 8) + 1) * 8
    if is_int:
        return process_interval(supported_int_bit_lengths, nbits)
    else:
        return process_interval(supported_float_bit_lengths, nbits)

def return_datatype(signal):
    ans = ""
    if isinstance(signal.scale, float):
        signal_bits = str(return_bits(signal.length, False))
        ans = "float"
    else:
        signal_bits = str(return_bits(signal.length, True))
        if signal.is_signed:
            ans = "int"
        else:
            ans = "uint"
    return ans + signal_bits

def build_dataclasses(can_dbc, dbc_dict):
    dbc_msgs = []
    for msg in can_dbc.messages:
        dbc_msg = DbcMessage(msg.name, f'0x{msg.frame_id:004x}', \
            dbc_dict[msg.name] , [])
        for signal in msg.signals:
            msg_signal = DbcSignal(signal.name, return_datatype(signal))
            dbc_msg.signals.append(msg_signal)
        dbc_msgs.append(dbc_msg)
    return dbc_msgs

def parse_dbc_dict(dbc_dict):
    reversed_dict = {}
    for comm_type, msgs in dbc_dict.items():
        for msg in msgs:
            if msg in reversed_dict.keys():
                reversed_dict[msg].append(comm_type)
            else:
                reversed_dict[msg] = [comm_type]
    return reversed_dict

def create_new_package():
    if Path('generated').is_dir():
        return False
    copy_tree(str(template_pkg_dir), str(out_pkg_dir))

def make_ros_messages(dbc_msgs, msg_template, out_msg_dir, out_msg_cmake):
    ros_msgs = []
    ros_msg_names = []
    out_msg_dir.mkdir(exist_ok = True)
    for dbc_msg in dbc_msgs:
        ros_msg = msg_template
        for signal in dbc_msg.signals:
            ros_msg += signal.data_type + " " + signal.name + "\n"
        ros_msgs.append(ros_msg.strip())
    for idx, ros_msg in enumerate(ros_msgs):
        ros_msg_name = "".join([dbc_msg_part.capitalize() for dbc_msg_part in dbc_msgs[idx].name.split("_")]) + '.msg'
        ros_msg_names.append(ros_msg_name)
        ros_msg_path = out_msg_dir / ros_msg_name
        ros_msg_path.write_text(ros_msg)
    cmake_out_msg = out_msg_cmake.read_text()
    cmake_out_msg = cmake_out_msg.replace("MSG_FILES", "\n".join(["\t" + '"' + \
        ros_msg_name + '"' for ros_msg_name in ros_msg_names]).strip())
    out_msg_cmake.write_text(cmake_out_msg)

def copy_dbc(dbc_fp, out_dbc_dir):
    out_dbc_dir.mkdir(exist_ok = True)
    copy_file(dbc_fp, str(out_dbc_dir / dbc_fp))

def add_dbc_to_launch(dbc_fp, out_launch_dir):
    for launch_file in out_launch_dir.iterdir():
        launch_text = launch_file.read_text()
        launch_text = launch_text.replace("DBC_FILE_NAME", Path(dbc_fp).name)
        launch_file.write_text(launch_text)

def modify_dispatch(dbc_msgs, out_dispatch_file):
    dispatch_ids = ""
    for dbc_msg in dbc_msgs:
        dispatch_msg_name = "\tID_" + "_".join([dbc_msg_part.upper() for dbc_msg_part in dbc_msg.name.split("_")])
        dispatch_msg_id = dbc_msg.id
        dispatch_ids += (dispatch_msg_name + " = " + dispatch_msg_id + ",\n")
    dispatch_ids = dispatch_ids.strip()
    dispatch_out_text = out_dispatch_file.read_text()
    dispatch_out_text = dispatch_out_text.replace("MESSAGE_IDS", dispatch_ids)
    out_dispatch_file.write_text(dispatch_out_text)

def generate_new_package(dbc_fp, yaml_fp, out_msg_dir, out_msg_cmake, out_dbc_dir, out_launch_dir):
    can_dbc = cantools.database.load_file(dbc_fp, database_format='dbc')
    dbc_dict = parse_dbc_dict(yaml.safe_load(Path(yaml_fp).read_text()))
    dbc_msgs = build_dataclasses(can_dbc, dbc_dict)

    result = create_new_package()
    if result == False:
        print("Generated already exists. Can't generate driver")
    else:
        msg_template = "builtin_interfaces/Time stamp\n\n"
        make_ros_messages(dbc_msgs, msg_template, out_msg_dir, out_msg_cmake)
        copy_dbc(dbc_fp, out_dbc_dir)
        add_dbc_to_launch(dbc_fp, out_launch_dir)
        modify_dispatch(dbc_msgs, out_dispatch_file)

generate_new_package(dbc_fp, yaml_fp, out_msg_dir, out_msg_cmake, out_dbc_dir, out_launch_dir)