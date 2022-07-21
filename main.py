import yaml
from pathlib import Path
from dataclasses import dataclass
from typing import List
import cantools
import math
import bisect
from distutils.dir_util import copy_tree
from distutils.file_util import copy_file
import re

supported_int_bit_lengths = [8, 16, 32, 64]
supported_float_bit_lengths = [32, 64]
dbc_fp = 'New_Eagle_DBW_3.4.dbc'
yaml_fp = 'New_Eagle_DBW_3.4.yaml'
# dbc_fp = 'IAC-CAN1-INDY-V9.dbc'
# yaml_fp = 'IAC-CAN1-INDY-V9.yaml'
template_pkg_dir = Path('template_package/')
out_pkg_dir = Path('generated') / 'src'
out_pkg_msg_dir = out_pkg_dir / 'raptor_dbw_msgs'
out_pkg_can_dir = out_pkg_dir / 'raptor_dbw_can'
ros_msg_dir = 'msg/'
out_msg_dir = out_pkg_msg_dir / ros_msg_dir
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
    ros_name: str
    dispatch_id_name: str
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
        dbc_msg = DbcMessage(msg.name, '', '', f'0x{msg.frame_id:004x}', \
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
    if Path(out_pkg_dir).is_dir():
        return False
    copy_tree(str(template_pkg_dir), str(out_pkg_dir))

def make_ros_messages(dbc_msgs, msg_template, out_msg_dir, out_msg_cmake, ros_msg_dir):
    ros_msgs = []
    ros_msg_names = []
    out_msg_dir.mkdir(exist_ok = True)
    for dbc_msg in dbc_msgs:
        ros_msg = msg_template
        for signal in dbc_msg.signals:
            ros_msg += signal.data_type + " " + signal.name.lower() + "\n"
        ros_msgs.append(ros_msg.strip())
    for idx, ros_msg in enumerate(ros_msgs):
        ros_msg_name = "".join([dbc_msg_part.capitalize() for dbc_msg_part in dbc_msgs[idx].name.split("_")]) + '.msg'
        dbc_msgs[idx].ros_name = ros_msg_name.replace(".msg", "")
        ros_msg_names.append(ros_msg_name)
        ros_msg_path = out_msg_dir / ros_msg_name
        ros_msg_path.write_text(ros_msg)
    cmake_out_msg = out_msg_cmake.read_text()
    cmake_out_msg = cmake_out_msg.replace("MSG_FILES", "\n".join(["\t" + '"' + \
        ros_msg_dir + ros_msg_name + '"' for ros_msg_name in ros_msg_names]).strip())
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
        dbc_msg.dispatch_id_name = dispatch_msg_name.strip()
        dispatch_msg_id = dbc_msg.id
        dispatch_ids += (dispatch_msg_name + " = " + dispatch_msg_id + ",\n")
    dispatch_ids = dispatch_ids.strip()
    dispatch_out_text = out_dispatch_file.read_text()
    dispatch_out_text = dispatch_out_text.replace("MESSAGE_IDS", dispatch_ids)
    out_dispatch_file.write_text(dispatch_out_text)

def modify_header(dbc_msgs, out_can_header):
    raptor_msg_imports = ""
    msg_import_template = '#include "raptor_dbw_msgs/msg/IMPORT.hpp"\n'
    raptor_usings = ""
    raptor_usings_template = "using raptor_dbw_msgs::msg::USING;\n"
    recv_can_msgs = ""
    recv_can_msgs_template = "\tvoid recvCANFUNC(const Frame::SharedPtr msg, DbcMessage * message);\n"
    recv_ros_msgs = ""
    recv_ros_msgs_template = "\tvoid recvROSFUNC(const ROSFUNC::SharedPtr msg);\n"
    ros_subscribers = ""
    ros_subscribers_template = "\trclcpp::Subscription<ROSMSG>::SharedPtr subROSMSG_;\n"
    ros_publishers = ""
    ros_publishers_template = "\trclcpp::Publisher<ROSMSG>::SharedPtr pubROSMSG_;\n"
    for dbc_msg in dbc_msgs:
        raptor_msg_import_name = dbc_msg.name.lower()
        for digit in range(10):
            raptor_msg_import_name = re.sub(f'_{digit}' , f'{digit}' ,raptor_msg_import_name)
        raptor_msg_imports += (msg_import_template.replace("IMPORT", raptor_msg_import_name))
        raptor_usings += (raptor_usings_template.replace("USING", dbc_msg.ros_name))
        if 'publish' in dbc_msg.comm_type:
            recv_can_msgs += (recv_can_msgs_template.replace("CANFUNC", dbc_msg.ros_name))
        if 'receive' in dbc_msg.comm_type:
            recv_ros_msgs += (recv_ros_msgs_template.replace("ROSFUNC", dbc_msg.ros_name))
        ros_subscribers += (ros_subscribers_template.replace("ROSMSG", dbc_msg.ros_name))
        ros_publishers += (ros_publishers_template.replace("ROSMSG", dbc_msg.ros_name))
    raptor_msg_imports = raptor_msg_imports.strip()
    raptor_usings = raptor_usings.strip()
    recv_can_msgs = recv_can_msgs.strip()
    recv_ros_msgs = recv_ros_msgs.strip()
    ros_subscribers = ros_subscribers.strip()
    ros_publishers = ros_publishers.strip()
    out_can_header_text = out_can_header.read_text()
    out_can_header_text = out_can_header_text.replace("RAPTOR_MSG_IMPORTS", raptor_msg_imports)
    out_can_header_text = out_can_header_text.replace("RAPTOR_USING", raptor_usings)
    out_can_header_text = out_can_header_text.replace("RECV_CAN_MESSAGES", recv_can_msgs)
    out_can_header_text = out_can_header_text.replace("RECV_ROS_MESSAGES", recv_ros_msgs)
    out_can_header_text = out_can_header_text.replace("ROS_SUBSCRIBERS", ros_subscribers)
    out_can_header_text = out_can_header_text.replace("ROS_PUBLISHERS", ros_publishers)
    out_can_header.write_text(out_can_header_text)

def modify_source(dbc_msgs, out_can_source):
    ros_publishers_initialize = ""
    ros_publishers_initialize_template = '\tpubROSMSG_ = this->create_publisher<ROSMSG>("MSGNAME", rclcpp::SensorDataQoS());\n'
    ros_subscribers_initialize = ""
    ros_subscribers_initialize_template = '\tsubROSMSG_ = this->create_subscription<ROSMSG>("MSGNAME", rclcpp::SensorDataQoS(), std::bind(&RaptorDbwCAN::recvROSMSG, this, std::placeholders::_1));\n'
    switch_case_id = ""
    switch_case_recv_id_template = "\t\t\tcase SWITCHID:\n\t\t\t\tRECV_DBC(recvROSMSG);\n\t\t\t\tbreak;\n"
    switch_case_pub_id_template = "\t\t\tcase SWITCHID:\n\t\t\t\tbreak;\n"
    recv_can_body = ""
    recv_can_body_template = 'void RaptorDbwCAN::recvROSMSG(const Frame::SharedPtr msg, DbcMessage * message)\n{\n\tROSMSG out;\n\tout.stamp = msg->header.stamp;\n\n\tFILL_SIGNALS\n\n\tpubROSMSG_->publish(out);\n}\n\n'
    recv_can_body_signal_template = '\tout.FIELDNAME = message->GetSignal("SIGNAME")->GetResult();\n'
    recv_ros_body = ""
    recv_ros_body_template = 'void RaptorDbwCAN::recvROSMSG(const ROSMSG::SharedPtr msg)\n{\n\tNewEagle::DbcMessage * message = dbw_dbc_.GetMessageById(DISPATCH_ID);\n\n\tFILLSIGNALS\n\n\tFrame frame = message->GetFrame();\n\tpub_can_->publish(frame);\n}\n\n'
    recv_ros_body_field_template = '\tmessage->GetSignal("SIGNAME")->SetResult(msg->FIELDNAME);\n'
    for dbc_msg in dbc_msgs:
        if 'publish' in dbc_msg.comm_type:
            ros_publishers_initialize += (ros_publishers_initialize_template.replace("ROSMSG", dbc_msg.ros_name).replace("MSGNAME", dbc_msg.name.lower()))
            switch_case_id += (switch_case_recv_id_template.replace("SWITCHID", dbc_msg.dispatch_id_name).replace("ROSMSG", dbc_msg.ros_name))
            recv_can_body_signal = ""
            for signal in dbc_msg.signals:
                recv_can_body_signal += (recv_can_body_signal_template.replace("FIELDNAME", signal.name.lower()).replace("SIGNAME", signal.name))
            recv_can_body_signal = recv_can_body_signal.strip()
            recv_can_body += (recv_can_body_template.replace("ROSMSG", dbc_msg.ros_name).replace("FILL_SIGNALS", recv_can_body_signal))
        if 'receive' in dbc_msg.comm_type:
            ros_subscribers_initialize += (ros_subscribers_initialize_template.replace("ROSMSG", dbc_msg.ros_name).replace("MSGNAME", dbc_msg.name.lower()))
            # switch_case_id += (switch_case_pub_id_template.replace("SWITCHID", dbc_msg.dispatch_id_name).replace("ROSMSG", dbc_msg.ros_name))
            recv_ros_body_field = ""
            for signal in dbc_msg.signals:
                recv_ros_body_field += (recv_ros_body_field_template.replace("FIELDNAME", signal.name.lower()).replace("SIGNAME", signal.name))
            recv_ros_body_field = recv_ros_body_field.strip()
            recv_ros_body += (recv_ros_body_template.replace("ROSMSG", dbc_msg.ros_name).replace("DISPATCH_ID", dbc_msg.dispatch_id_name).replace("FILLSIGNALS", recv_ros_body_field))
    ros_publishers_initialize = ros_publishers_initialize.strip()
    ros_subscribers_initialize = ros_subscribers_initialize.strip()
    switch_case_id = switch_case_id.strip()
    recv_can_body = recv_can_body.strip()
    recv_ros_body = recv_ros_body.strip()
    out_can_source_text = out_can_source.read_text()
    out_can_source_text = out_can_source_text.replace("ROS_PUBLISHERS_INITIALIZE", ros_publishers_initialize)
    out_can_source_text = out_can_source_text.replace("ROS_SUBSCRIBERS_INITIALIZE", ros_subscribers_initialize)
    out_can_source_text = out_can_source_text.replace("SWITCH_CASE_ID", switch_case_id)
    out_can_source_text = out_can_source_text.replace("RECV_CAN_BODY", recv_can_body)
    out_can_source_text = out_can_source_text.replace("RECV_ROS_BODY", recv_ros_body)
    out_can_source.write_text(out_can_source_text)

def generate_new_package(dbc_fp, yaml_fp, out_msg_dir, out_msg_cmake, out_dbc_dir, out_launch_dir, ros_msg_dir, out_can_header, out_can_source):
    can_dbc = cantools.database.load_file(dbc_fp, database_format='dbc')
    dbc_dict = parse_dbc_dict(yaml.safe_load(Path(yaml_fp).read_text()))
    dbc_msgs = build_dataclasses(can_dbc, dbc_dict)

    result = create_new_package()
    if result == False:
        print("Generated already exists. Can't generate driver")
    else:
        msg_template = "builtin_interfaces/Time stamp\n\n"
        make_ros_messages(dbc_msgs, msg_template, out_msg_dir, out_msg_cmake, ros_msg_dir)
        copy_dbc(dbc_fp, out_dbc_dir)
        add_dbc_to_launch(dbc_fp, out_launch_dir)
        modify_dispatch(dbc_msgs, out_dispatch_file)
        modify_header(dbc_msgs, out_can_header)
        modify_source(dbc_msgs, out_can_source)

generate_new_package(dbc_fp, yaml_fp, out_msg_dir, out_msg_cmake, out_dbc_dir, out_launch_dir, ros_msg_dir, out_can_header, out_can_source)