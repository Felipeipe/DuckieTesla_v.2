#!/usr/bin/env python
import numpy as np
import cv2
import os
import tensorflow as tf
import rospy #importar ros para python
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped 
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image

path = '/home/fablab/RLTESLA/RLDuckietown/RL'
#Se tiene modelo de prueba por defecto en carpeta models
#Se ejecuta el modelo de red neuronal

class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args
        #sucribir a joy 
        self.Sub_Cam = rospy.Subscriber("/duckiebot/camera_node/image/rect", Image, self.callback)
        #publicar la intrucciones del control en possible_cmd
        self.publi = rospy.Publisher("duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = 0)
        self.pub_pos = rospy.Publisher("/duckiebot/smart", Point, queue_size = 1) 
        self.twist = Twist2DStamped()
        self.model = tf.keras.models.load_model(os.path.join(path,"models", "modeloco.h5"))

    def callback(self,msg):
        #se obtiene una imagen
        smart = Point()
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = image[90:240,0:320]
        scale_percent = 25 # porcentaje de la imagen original
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        obs_ = np.expand_dims(resized,axis=0)
        _key = self.model.predict(obs_)
        print(_key)
        _key = np.argmax(_key[0])
        print(_key)
        if _key == 0:
            #pa delente
            self.twist.v = 0.4
            self.twist.omega = 0
            print("adelante")
            smart.x=_key
            self.publi.publish(self.twist)
        elif _key == 1:
            #pa la derecha
            self.twist.omega = -15
            self.twist.v = 0
            print("derecha")
            smart.x=_key
            self.publi.publish(self.twist)
        elif _key == 2:
            #pa la izquierda
            self.twist.omega = 1*10

        
        self.pub_pos.publish(smart)
        print("publicado")

# Se cierra el environment y termina el programa
def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()


