string TOPIC = /slam/slam_order_req   

#1 建图 2 重定位 3 切换机器人位置状态 4 停止
uint8 order

uint8 MAPPING = 1
uint8 RELOCATING = 2 
uint8 SWITCH_WORK_TYPE = 3
uint8 STOP = 4

string map_folder
#工作状态 1 池底，2 池壁，3 水线，4 入水
uint8 work_type

uint8 BOTTOM = 1
uint8 WALL = 2
uint8 PIPELINE = 3
uint8 SINKING = 4

