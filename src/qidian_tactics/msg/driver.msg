std_msgs/Header header  #标准消息头，包含时间戳等信息

# uint16 red_7_robot_HP
# uint16 red_outpost_HP
# uint16 red_base_HP
# uint16 blue_7_robot_HP
# uint16 blue_outpost_HP
# uint16 blue_base_HP
# uint16 current_HP
# float64 x
# float64 y
# float64 angle
uint16 own_robot_HP
uint16 own_outpost_HP
uint16 own_base_HP
uint16 enemy_robot_HP
uint16 enemy_outpost_HP
uint16 enemy_base_HP
uint32 event_data
uint16 maximum_HP
uint8 defence_buff
uint8 vulnerability_buff
uint16 attack_buff
uint8 armor_id
uint16 shoot_num
uint32 rfid_status
bool detect_enemy
time start_time
float64 Virtual_Shield_Status#虚拟护盾血量百分比
bool own_supply_area_Status #己方补给区状态 被占领1 未被占领0
bool Central_area_status #中心区是否可占领 可1，否0