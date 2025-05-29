#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import os

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        
        # 发布器，消息类型为OccupancyGrid，Topic名称为/map
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # 读取PGM文件
        pgm_path = os.path.join(os.getcwd(), 'map.pgm')  # 修改为实际路径
        self.get_logger().info(f"Loading PGM map from: {pgm_path}")
        
        # 解析PGM文件并转换为OccupancyGrid
        occupancy_grid = self.pgm_to_occupancy_grid(pgm_path)
        
        # 持续发布地图（可改为单次发布）
        self.timer = self.create_timer(1.0, lambda: self.publisher_.publish(occupancy_grid))
        self.get_logger().info("Map publisher started")

    def pgm_to_occupancy_grid(self, pgm_path: str) -> OccupancyGrid:
        """将PGM文件转换为OccupancyGrid消息"""
        msg = OccupancyGrid()
        
				# 判断文件类型并读取
        with open(pgm_path, 'rb') as pgm_file:
            header_line = pgm_file.readline().decode().strip()
            if header_line not in ('P2', 'P5'):
            		raise ValueError(f"Unsupported PGM format: {header_line}")
        
        		# 跳过注释
            comment_line = pgm_file.readline()
            while comment_line.startswith(b'#'):
                comment_line = pgm_file.readline()
        
        		# 读取宽高
            width, height = map(int, comment_line.decode().split())
            max_val = int(pgm_file.readline().decode())
        
        		# 读取像素数据
            if header_line == 'P2':
            		# ASCII格式：逐行读取所有数字
                data = []
                for line in pgm_file:
                    data += list(map(int, line.decode().split()))
                data = np.array(data, dtype=np.uint8)
            else:
            		# 二进制格式：直接读取字节
                data = np.frombuffer(pgm_file.read(), dtype=np.uint8)

        # 填充消息头
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # 坐标系名称
        
        # 地图元数据
        msg.info.resolution = 0.06  # 修改为实际分辨率（单位：米/像素）
        msg.info.width = width
        msg.info.height = height
        
        # 设置地图原点（假设原点在左下角）
        msg.info.origin.position.x = -15.0  # 修改为实际X偏移
        msg.info.origin.position.y = -13.0  # 修改为实际Y偏移
        msg.info.origin.orientation.w = 1.0
        
        # 转换像素值为占用值（ROS规范：0=空闲, 100=占用, -1=未知）
        # PGM中通常 0=占用（黑色）, 255=空闲（白色）
        scaled_data = (255 - data) * (100.0 / 255.0)
        scaled_data = scaled_data.astype(np.int8)
        
        # 反转Y轴（PGM原点在左上角，ROS地图原点在左下角）
        scaled_data = scaled_data.reshape((height, width))
        scaled_data = np.flipud(scaled_data)
        msg.data = scaled_data.flatten().tolist()
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
