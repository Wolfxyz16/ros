#!/usr/bin/env python

import glob
import csv
from ament_index_python import get_package_share_directory
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
import numpy as np
import matplotlib.pyplot as plt
import math


# Se define la clase Robot que hereda de Node en ROS 2
class Robot(Node):
    def __init__(self):
        super().__init__('rwander_node')
        self.declare_parameter('vel_topic', 'cmd_vel')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('pose_topic', 'rosbot_pose')
        vel_topic_ = self.get_parameter('vel_topic').value
        scan_topic_ = self.get_parameter('scan_topic').value
        pose_topic_ = self.get_parameter('pose_topic').value

        # ROS Subscribers
        self._laser_sub = self.create_subscription(LaserScan, scan_topic_, self.obstacle_detect, 10)
        self._pose_sub = self.create_subscription(Pose2D, pose_topic_, self.pose_callback, 10)
        # ROS Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, vel_topic_, 10)       
        self._scan_count = 0
        self._max_range = 100.0
        self._uninitialized = 1
        self._bearings = []
        self._scan = []

        self.x_pos = []
        self.y_pos = []
        self.theta_pos= []

        
    # Método para detectar obstáculos con los datos de escaneo  
    def obstacle_detect(self, scan_msg):
        
        self._scan = scan_msg.ranges

        if self._uninitialized:
            self._uninitialized = 0
            self._scan_count = len(scan_msg.ranges)
            self._max_range = scan_msg.range_max
            # Bearings stores the angle of each range measurement (radians)
            for i in range(0, self._scan_count):
                self._bearings.append(scan_msg.angle_min + scan_msg.angle_increment * i)
            self.get_logger().info("# Scan count %d"%(self._scan_count))  
            self.get_logger().info("# Laser angle min: %.2f"%(np.rad2deg(scan_msg.angle_min)))
            self.get_logger().info("# Laser angle max: %.2f"%(np.rad2deg(scan_msg.angle_max)))
            self.get_logger().info("# Laser angle increment:  %.4f rad (%.2f deg)"%(scan_msg.angle_increment, np.rad2deg(scan_msg.angle_increment)))
            self.get_logger().info("# Time between mesurements [seconds]:  %.2f"%(scan_msg.time_increment))
            self.get_logger().info("# Time between scans [seconds]:  %.2f"%(scan_msg.scan_time))
            self.get_logger().info("# Minimum range value:  %.2f"%(scan_msg.range_min))
            self.get_logger().info("# Maximum range value:  %.2f "%(scan_msg.range_max))
            resolution = (scan_msg.angle_max - scan_msg.angle_min)/len(scan_msg.ranges)
            self.get_logger().info("# Resolution:  %.2f"%(np.rad2deg(resolution))) 

        # Replace infinity values with max_range 
        self._scan = [x if x < self._max_range else self._max_range for x in self._scan] 

        # Reorganize scan indexes to make it easier to work with. 
        # 0 index corresponds to the back side of the robot for both, scan and bearings.
        self._scan = [self._scan[i - 800] for i in range(self._scan_count)]    
    
        # TODO: add your code here

        # Seleccionamos las velocidades iniciales de movimiento y giro
        speed = 0.5
        turn = 0.0

        # Seleccionamos una sección del escaneo frontal para el análisis
        vector = np.array(self._scan[400:1200])

        # Dividimos el vector en 5 segmentos: dos a la izquierda, uno al frente y dos a la derecha
        posiciones = np.array_split(vector, 5)

        vectorL1 = posiciones[0]
        vectorL2 = posiciones[1]
        vectorF = posiciones[2]
        vectorR1 = posiciones[3]
        vectorR2 = posiciones[4]

        # Calculamos la media de cada segmento
        medias = np.array([np.mean(vectorL1), np.mean(vectorL2), np.min(vectorF) ,np.mean(vectorR1), np.mean(vectorR2)])

        # Se encuentra el índice del segmento con el mayor valor medio
        maximo = np.argsort(medias)[-1]
        
        # Se determina el giro según el segmento "libre" más amplio
        # Si el espacio está a la izquierda, gira a la izquierda
        if maximo <= 1:
            turn = -0.5
        # Si el espacio está a la derecha, gira a la derecha
        elif maximo > 2:
            turn = 0.5
        # Si el espacio está al frente, no gira
        else:
            turn = 0.0

        # Si hay un obstáculo muy cerca en el frente, se para
        if np.min(vectorF) < 0.5:
            speed = 0.0

        # Imprimimos valores para verificar las medias y el giro
        print(f"MEDIAS | L1,L2: {np.mean(vectorL1):03}, {np.mean(vectorL2):03} \tF: {np.mean(vectorF):03} \tR1,R2: {np.mean(vectorR1):03}, {np.mean(vectorR2):03}")
        print("")
        print(f"MIN F: {np.min(vectorF)}")
        print(f"MAXIMO | {maximo}") 

        ## end TODO

        # Se publica el mensaje de la velocidad ajustada
        cmd_vel_msg_ = Twist()
        cmd_vel_msg_.linear.x  = speed
        cmd_vel_msg_.linear.y  = 0.0
        cmd_vel_msg_.angular.z = turn
        self._cmd_vel_pub.publish( cmd_vel_msg_ ) 
    
    # Método callback para actualizar la posición del robot
    def pose_callback(self, msg):
        #self.get_logger().info("Robot pose: x: %.2f, y: %.2f, theta: %.2f"%(msg.x, msg.y, msg.theta))
        self.x_pos.append(msg.x)
        self.y_pos.append(msg.y)
        self.theta_pos.append(msg.theta)
        
    # Guardamos las posiciones en un archivo CSV
    def save_to_csv(self, filename="robot_positions.csv"):
        # Verificamos que haya datos para guardar
        if len(self.x_pos) == 0 or len(self.y_pos) == 0 or len(self.theta_pos) == 0:
            self.get_logger().info("No hay datos para guardar en el archivo CSV.")
            return

        # Abrir el archivo en modo escritura
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Escribir el encabezado del CSV
            writer.writerow(['x', 'y', 'theta'])
            # Escribir las posiciones del robot
            for x, y, theta in zip(self.x_pos, self.y_pos, self.theta_pos):
                writer.writerow([x, y, theta])

        self.get_logger().info(f"Datos guardados en {filename}")



def main(args=None):
    rclpy.init(args=args)
    rwander_node = Robot()
    try:
        # Se mantiene el nodo activo hasta que se interrumpa manualmente
        rclpy.spin(rwander_node)
    except KeyboardInterrupt:
        rwander_node.get_logger().info("Node interrupted by keyboard (CTRL+C)")
    finally:
        # Al cerrar, se guardan las posiciones en un archivo y se cierra el nodo
        print('Voy a guardar')
        rwander_node.save_to_csv()
        rwander_node.destroy_node()  
        rclpy.shutdown()
    
if __name__=="__main__":
    main()
