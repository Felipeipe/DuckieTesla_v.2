#!/usr/bin/env python3
import rospy #importar ros para python
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
import cv2 # importar libreria opencv
import os
# from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
from cv_bridge import CvBridge # importar convertidor de formato de imagenes

import message_filters
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image, Joy
from duckietown_msgs.msg import WheelsCmdStamped


class TemplateNode(DTROS):
	def __init__(self, node_name):
		super(TemplateNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
		#configuración de topicos
		vehicle_name = os.environ['VEHICLE_NAME']
		self._camera_topic = f"/{vehicle_name}/camera_node/image/compressed"
		self._joy_topic = f"/{vehicle_name}/joy"
		#configuracion suscriber y message filters
		self.camera_sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.tomar_img)
		self.joy_sub = rospy.Subscriber(self._joy_topic, Joy, self.writter)
		# dts = message_filters.ApproximateTimeSynchronizer([camera_sub, joy_sub], queue_size=5, slop=0.1)
		# dts.registerCallback(tomar_img(self, camera_sub, joy_sub))
		#importar CvBridge
		self._bridge = CvBridge()
		# aqui configuramos el suscriber
		# self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.tomar_img)
		# variable auxiliar
		self.i = 0

	def tomar_img(self, msg_camera):
		rate = rospy.Rate(10)
		image = self._bridge.compressed_imgmsg_to_cv2(msg_camera)
		img2=image[180:480,: ,:]
		self._imagencilla = img2

	def writter(self, msg_joy):
		imagen = self._imagencilla
		directorio =r'/code/'
		os.chdir(directorio)
		nombre = "imagen"+str(self.i)+".jpg"
		# nombre_path = r"/code/"+"imagen"+str(self.i)+".jpg"
		print(os.listdir(directorio))
		cv2.imwrite(nombre, imagen) #luego se puede procesar la imagen
		print("la imagen "+str(nombre)+" se guardó bien uwu")

		#CAPTURA DE VELOCIDADES
		"""
		sugerencia de código
		eliminar las lineas 58-66

		agregar:
		velocidad = str(axes)
		"""
		avanzar = msg_joy.axes[1] #va a la izquierda
		if msg_joy.axes[3]>=0: 
			left=msg_joy.axes[3]
		elif msg_joy.axes[3]<0:
			right=-msg_joy.axes[3]
		
		velocidad = str(str(avanzar)+","+str(left)+","+str(right))
		print("esta es la velocidad: "+velocidad)
		# direction = "vel" #aqui esta el absolute
		if self.i == 1:
			#velocidades = open("vel.txt", "x")
			#velocidades.close()
			velocidades = open("vel.txt", "w")
			velocidades.write(velocidad+"\n")
			velocidades.close()
		else:
			velocidades = open("vel.txt", "a")
			velocidades.write(velocidad+"\n")
			velocidades.close()
		print("la velocidad es :"+velocidad)
		self.i += 1


	def tomar_img1(self, camera_sub, joy_sub): #cambiar las cosas por camera y joy
		rate = rospy.Rate(10)
		# CAPTURA DE IMAGENES, aqui y siempre se usará el directorio /code
		image = self._bridge.compressed_imgmsg_to_cv2(camera_sub, "bgr8") #listo
		directorio =r'/code/' 
		#Se escribe la imagen en la carpeta del path
		os.chdir(directorio)
		nombre = "imagen"+str(self.i)+".jpg"
		nombre_path = r"/code/"+"imagen"+str(self.i)+".jpg"
		print("Before saving image:")
		print(os.listdir(directorio)) #listo
		#aqui se graba la imagen
		cv2.imwrite(nombre, imagen) #luego se puede procesar la imagen
		print("la imagen "+str(nombre)+" se guardó bien uwu")

		#CAPTURA DE VELOCIDADES
		avanzar = joy_sub.axes[0]
		if joy_sub.axes[1]>=0:
			left=joy_sub.axes[1]
			right=-joy_sub.axes[1]
		elif msg.axes[1]<0:
			left=-joy_sub.axes[1]
			right=joy_sub.axes[1]
		velocidad = str(str(avanzar)+","+str(left)+","+str(right))
		direction = "/code/vel.txt" #aqui esta el absolute
		if self.i == 1:
			velocidades = open(direction, "x")
			velocidades.close()
			velocidades = open(direction, "w")
			velocidades.write(velocidad+"\n")
			velocidades.close()
		else:
			velocidades = open(direction, "a")
			velocidades.write(velocidad+"\n")
			velocidades.close()
		print("la velocidad es :"+velocidad)
		self.i+=1

if __name__ =='__main__':
	# create the node
	node = TemplateNode(node_name='template_node') # creacion del nodo
	# keep spinning
	rospy.spin()
