// Exception-safe wrappers around the JAKA SDK C API.
//
// The SDK throws C++ exceptions instead of returning error codes for some
// failures, e.g. kine_inverse on an unreachable pose. A C++ exception crossing
// the FFI boundary into Rust aborts the process, so every SDK call is wrapped
// in try/catch and translated to a sentinel error code. The SDK header's
// extern "C" guard is misspelled (__cpluscplus), so the prototypes are
// declared manually here.

extern "C" {
    int create_handler(const char *ip, int *handle);
    int destory_handler(const int *handle);
    int power_on(const int *handle);
    int power_off(const int *handle);
    int enable_robot(const int *handle);
    int disable_robot(const int *handle);
    int get_robot_state(const int *handle, void *state);
    int get_robot_status_simple(const int *handle, void *status);
    int set_digital_output(const int *handle, int type, int index, int value);
    int get_digital_output(const int *handle, int type, int index, int *value);
    int get_digital_input(const int *handle, int type, int index, int *value);
    int set_tio_vout_param(const int *handle, int vout_enable, int vout_vol);
    int get_tio_vout_param(const int *handle, int *vout_enable, int *vout_vol);
    int get_tio_pin_mode(const int *handle, int pin_type, int *pin_mode);
    int set_tio_pin_mode(const int *handle, int pin_type, int pin_mode);
    int get_rs485_chn_mode(const int *handle, int chn_id, int *chn_mode);
    int get_joint_position(const int *handle, void *pos);
    int get_tcp_position(const int *handle, void *tcp_position);
    int get_dh_param(const int *handle, void *dh_param);
    int kine_inverse(const int *handle, const void *ref_pos, const void *cartesian_pose, void *joint_pos);
    int kine_forward(const int *handle, const void *joint_pos, void *cartesian_pose);
    int joint_move_extend(const int *handle, const void *joint_pos, int move_mode, int is_block, double speed, double acc, double tol, const void *option_cond);
    int linear_move_extend(const int *handle, const void *end_pos, int move_mode, int is_block, double speed, double acc, double tol, const void *option_cond);
    int motion_abort(const int *handle);
    int jog(const int *handle, int aj_num, int move_mode, int coord_type, double vel_cmd, double pos_cmd);
    int jog_stop(const int *handle, int num);
    int servo_move_enable(const int *handle, int enable);
    int servo_p(const int *handle, const void *cartesian_pose, int move_mode, unsigned int step_num);
    int servo_move_use_joint_NLF(const int *handle, double max_vr, double max_ar, double max_jr);
    int servo_move_use_none_filter(const int *handle);
    int is_in_estop(const int *handle, int *in_estop);
    int clear_error(const int *handle);
}

#define JAKA_SENTINEL_EXCEPTION (-999)

#define WRAP(name, sig, call)                  \
    extern "C" int jk_safe_##name sig          \
    {                                          \
        try {                                  \
            return name call;                  \
        } catch (...) {                        \
            return JAKA_SENTINEL_EXCEPTION;    \
        }                                      \
    }

WRAP(create_handler, (const char *ip, int *handle), (ip, handle))
WRAP(destory_handler, (const int *handle), (handle))
WRAP(power_on, (const int *handle), (handle))
WRAP(power_off, (const int *handle), (handle))
WRAP(enable_robot, (const int *handle), (handle))
WRAP(disable_robot, (const int *handle), (handle))
WRAP(get_robot_state, (const int *handle, void *state), (handle, state))
WRAP(get_robot_status_simple, (const int *handle, void *status), (handle, status))
WRAP(set_digital_output, (const int *handle, int type, int index, int value), (handle, type, index, value))
WRAP(get_digital_output, (const int *handle, int type, int index, int *value), (handle, type, index, value))
WRAP(get_digital_input, (const int *handle, int type, int index, int *value), (handle, type, index, value))
WRAP(set_tio_vout_param, (const int *handle, int vout_enable, int vout_vol), (handle, vout_enable, vout_vol))
WRAP(get_tio_vout_param, (const int *handle, int *vout_enable, int *vout_vol), (handle, vout_enable, vout_vol))
WRAP(get_tio_pin_mode, (const int *handle, int pin_type, int *pin_mode), (handle, pin_type, pin_mode))
WRAP(set_tio_pin_mode, (const int *handle, int pin_type, int pin_mode), (handle, pin_type, pin_mode))
WRAP(get_rs485_chn_mode, (const int *handle, int chn_id, int *chn_mode), (handle, chn_id, chn_mode))
WRAP(get_joint_position, (const int *handle, void *pos), (handle, pos))
WRAP(get_tcp_position, (const int *handle, void *tcp_position), (handle, tcp_position))
WRAP(get_dh_param, (const int *handle, void *dh_param), (handle, dh_param))
WRAP(kine_inverse, (const int *handle, const void *ref_pos, const void *cartesian_pose, void *joint_pos), (handle, ref_pos, cartesian_pose, joint_pos))
WRAP(kine_forward, (const int *handle, const void *joint_pos, void *cartesian_pose), (handle, joint_pos, cartesian_pose))
WRAP(joint_move_extend, (const int *handle, const void *joint_pos, int move_mode, int is_block, double speed, double acc, double tol, const void *option_cond), (handle, joint_pos, move_mode, is_block, speed, acc, tol, option_cond))
WRAP(linear_move_extend, (const int *handle, const void *end_pos, int move_mode, int is_block, double speed, double acc, double tol, const void *option_cond), (handle, end_pos, move_mode, is_block, speed, acc, tol, option_cond))
WRAP(motion_abort, (const int *handle), (handle))
WRAP(jog, (const int *handle, int aj_num, int move_mode, int coord_type, double vel_cmd, double pos_cmd), (handle, aj_num, move_mode, coord_type, vel_cmd, pos_cmd))
WRAP(jog_stop, (const int *handle, int num), (handle, num))
WRAP(servo_move_enable, (const int *handle, int enable), (handle, enable))
WRAP(servo_p, (const int *handle, const void *cartesian_pose, int move_mode, unsigned int step_num), (handle, cartesian_pose, move_mode, step_num))
WRAP(servo_move_use_joint_NLF, (const int *handle, double max_vr, double max_ar, double max_jr), (handle, max_vr, max_ar, max_jr))
WRAP(servo_move_use_none_filter, (const int *handle), (handle))
WRAP(is_in_estop, (const int *handle, int *in_estop), (handle, in_estop))
WRAP(clear_error, (const int *handle), (handle))
