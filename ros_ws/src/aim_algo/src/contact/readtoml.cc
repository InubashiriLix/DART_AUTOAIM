#include <contact/pid.h>

#include <string>
#include <toml.hpp>

pid_config read_toml(std::string src_path, std::string session_name) {
    // please make sure that the path exists
    auto tbl = toml::parse_file(src_path);
    auto sub = tbl[session_name].as_table();
    if (!sub) throw std::runtime_error("No section: " + session_name);

    pid_config rtn;
    rtn.PID_KP = sub->at("kp").value_or(0.8);
    rtn.PID_KI = sub->at("ki").value_or(0.0);
    rtn.PID_KD = sub->at("kd").value_or(0.1);
    rtn.I_max = sub->at("i_max").value_or(100.0);
    rtn.I_min = sub->at("i_min").value_or(-100.0);
    return rtn;
}
