std_msgs/Header header          # 引用其他包的消息类型（时间戳+坐标系ID）
float64[] position              # [] 表示动态数组（长度不固定）
float64[] velocity
float64[] orientation_xyzw
float64[] orientation_wxyz
bool[] buttons
bool is_connected               # 没有 [] = 单值字段
float64 data_rate_hz