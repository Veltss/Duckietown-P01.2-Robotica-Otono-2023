#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from duckietown_msgs.msg import Twist2DStamped
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math
import logging
import os

def vercositas(frame,texto):
  hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  height, width, _ = frame.shape
  a=0
  if texto=="patitos":
    lower = np.array([50, 100, 100])
    upper = np.array([70, 255, 220])
    mask = cv2.inRange(hsv_frame, lower, upper)
    areamin = 300
    radio = 250 #calibrar
  elif texto=="azul":
    #c lower_1 = np.array([0, 100, 20])
    #c upper_1 = np.array([5, 255, 200])
    lower = np.array([0, 100, 100])
    upper = np.array([20, 255, 210])
    #c mask_1 = cv2.inRange(hsv_frame, lower_1, upper_1)
    mask = cv2.inRange(hsv_frame, lower, upper)
    #c mask = cv2.add(mask_1,mask_2)
    areamin = 500
    radio = 300 #calibrar
  elif texto=="redline":
    lower_1 = np.array([0, 100, 20])
    upper_1 = np.array([5, 255, 200])
    lower_2 = np.array([160, 100, 100])
    upper_2 = np.array([180, 255, 210])
    mask_1 = cv2.inRange(hsv_frame, lower_1, upper_1)
    mask_2 = cv2.inRange(hsv_frame, lower_2, upper_2)
    mask = cv2.add(mask_1,mask_2)
    areamin = 500
    radio = 15 #calibrar
    
    
  
  contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
  # Si se detectan objetos amarillos
  if len(contours) > 0:
    coordenadas_puntos = []
    for cnt in contours:
      area = cv2.contourArea(cnt)
      if area > areamin:
        M = cv2.moments(cnt)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        if texto=="redline":
            midy = int(height)
        else: 
            midy = int(height/2)
        midx = int(width/2)
        distancia = ((cx-midx)**2 + (cy-midy)**2)**0.5
        coordenadas_puntos.append((cx,cy))
            # Pinta los puntos en la imagen
        if distancia < radio:
            a=1
            return a
  return a
  

def detect_edges(frame,texto):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float)

    # Definir el rango de color amarillo (patitos de goma)
    lower_yellow = np.array([14, 100, 140])
    upper_yellow = np.array([30, 255, 190])

    # Definir el rango de color blanco (l  neas de la carretera)
    lower_white = np.array([0, 20, 140])
    upper_white = np.array([180, 100, 180])  # Rango ajustado para el color blanco

    # Crear m  scaras para los colores amarillo y blanco
    mascara_amarilla = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    mascara_blanca = cv2.inRange(hsv_frame, lower_white, upper_white)

    # Encontrar los contornos en la m  scara combinada
    if texto=="amarillo":
      edges = cv2.Canny(mascara_amarilla, 200, 400)
    elif texto=="blanco":
      edges = cv2.Canny(mascara_blanca, 200, 400)
    else:
      print("Color incorrecto")
    return edges


def region_of_interest(edges,texto):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    if texto=="amarillo":
      polygon = np.array([[
          (width*0, height * 1/ 6),
          (width*0.45, height * 1 / 6),
          (width*0.45, height * 9 / 10),
          (width*0, height * 9 / 10),
      ]], np.int32)

    elif texto=="blanco":
      polygon = np.array([[
          (width*0.4, height * 1/4),
          (width*0.8, height * 1/4),
          (width, height * 1.0),
          (width*0.4, height * 0.9),
      ]], np.int32)

    else:
      print("Color incorrecto")

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges



def detect_line_segments(cropped_edges,texto):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    if texto=="amarillo":
      rho = 5  # distance precision in pixel, i.e. 1 pixel
      angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
      min_threshold = 20  # minimal of votes
      line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
                                      np.array([]), minLineLength=8, maxLineGap=4)
    elif texto=="blanco":
      rho = 5  # distance precision in pixel, i.e. 1 pixel
      angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
      min_threshold = 20  # minimal of votes
      line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
                                      np.array([]), minLineLength=8, maxLineGap=4)
    else:
      print("Color incorrecto")

    return line_segments



def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]



def average_slope_intercept(frame, line_segments,texto):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines,0

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    if texto=="amarillo":
      boundary = 1
    elif texto=="blanco":
      boundary = 0
    else:
      print("Color incorrecto")
    left_region_boundary = width * (boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else: #slope > 0:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    cantidad = len(lane_lines)
    return lane_lines,cantidad


def limpiar_lineas(lane_lines,c,frame):
  height, width, _ = frame.shape
  for line in lane_lines:
    x1=line[0][0]
    x2=line[0][2]
    if x1<0 and x2<0:
      lane_lines.remove(line)
      c-=1
    elif x1>width and x2>width:
      lane_lines.remove(line)
      c-=1
  return lane_lines,c

def juntar_lineas(lines_amarillo,c_amarillo, lines_blanco,c_blanco):
  lane_lines=[]
  if c_amarillo==1 and c_blanco==1:
    lane_lines.append(lines_amarillo[0])
    lane_lines.append(lines_blanco[0])
    jaja=0
  elif c_amarillo==0 and c_blanco==1:
    lane_lines.append(lines_blanco[0])
    jaja=1
  elif c_amarillo==1 and c_blanco==0:
    lane_lines.append(lines_amarillo[0])
    jaja=-1
  else:
    jaja=100
  return lane_lines,jaja



def Guia(lane_lines,jaja,frame):
  height, width, _ = frame.shape
  if jaja==-1:
    x1, y1, x2, y2 = lane_lines[0][0]
    m_1=(x2-x1)/(y2-y1)
    alpha1=math.degrees(np.arctan(m_1))
    fc1=42#factor correctivo 1 de pendiente
    fc2=1#factor correctivo 2 de pendiente
    alpha2 =fc2*(alpha1+fc1)
    alpha = math.radians(alpha2)
    x_offset=int(width/2 -np.abs(y2-y1)*np.tan(alpha))
    lane_lines.append([[int(width / 2),y1,x_offset, y2]])
    return lane_lines,alpha
  if jaja==0:
    _, _, left_x2, _ = lane_lines[0][0]
    _, _, right_x2, _ = lane_lines[1][0]
    mid = int(width*1.1 / 2)
    x_offset = int((left_x2 + right_x2) / 2)
    y_offset = int(height / 2)
    lane_lines.append([[mid,height,x_offset, y_offset]])
    x1, y1, x2, y2 = mid,height,x_offset, y_offset
    m_1=(x2-x1)/(y2-y1)
    alpha1=np.arctan(m_1)
    return lane_lines,alpha1
  elif jaja==1:
    x1, y1, x2, y2 = lane_lines[0][0]
    m_1=(x2-x1)/(y2-y1)
    alpha1=math.degrees(np.arctan(m_1))
    fc1=42#factor correctivo 1 de pendiente
    fc2=1#factor correctivo 2 de pendiente
    alpha2 =fc2*(alpha1-fc1)
    alpha = math.radians(alpha2)
    x_offset=int(width/2 -np.abs(y2-y1)*np.tan(alpha))
    lane_lines.append([[int(width / 2),y1,x_offset, y2]])
    return lane_lines,alpha
  else:
    return lane_lines,0


def detect_lane(frame):

    edges_blanco = detect_edges(frame,"blanco")
    edges_amarillo = detect_edges(frame,"amarillo")

    cropped_edges_blanco = region_of_interest(edges_blanco,"blanco")
    cropped_edges_amarillo = region_of_interest(edges_amarillo,"amarillo")

    line_segments_blanco = detect_line_segments(cropped_edges_blanco,"blanco")
    line_segments_amarillo = detect_line_segments(cropped_edges_amarillo,"amarillo")

    lane_lines1_blanco,c_blanco = average_slope_intercept(frame, line_segments_blanco,"blanco")
    lane_lines1_amarillo,c_amarillo = average_slope_intercept(frame, line_segments_amarillo,"amarillo")

    lane_lines2_blanco,c2_blanco = limpiar_lineas(lane_lines1_blanco,c_blanco,frame)
    lane_lines2_amarillo,c2_amarillo = limpiar_lineas(lane_lines1_amarillo,c_amarillo,frame)

    lane_lines3,jaja = juntar_lineas(lane_lines2_amarillo,c2_amarillo, lane_lines2_blanco,c2_blanco)

    lane_lines,alpha1 = Guia(lane_lines3,jaja,frame)

    return lane_lines,alpha1


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=20):
    line_image = np.zeros_like(frame)
    if lines is not None and len(lines)>0:
      guia = lines.pop()
      for x1, y1, x2, y2 in guia:
        cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 30)
      for line in lines:
        for x1, y1, x2, y2 in line:
          cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/duckiebot/camera_node/image/raw",Image,self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=10)
        self.im_pub = rospy.Publisher('/duckiebot/detections', Image, queue_size=10)
        self.im = Image()


    def camera_callback(self,msg):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_image.shape
        #print(height,width,channels)

        rows_to_watch = 100
        top_trunc = 1*height / 2 #get 3/4 of the height from the top section of the image
        bot_trunc = top_trunc + rows_to_watch #next set of rows to be used
        crop_img = cv_image[top_trunc:bot_trunc, 0:width]
        
        # Convert from RGB to HSV
        #hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV).astype(np.float)
        lines,angulo=detect_lane(crop_img)
        frame_procesado = display_lines(crop_img,lines) 
        self.im = self.bridge_object.cv2_to_imgmsg(frame_procesado,encoding='bgr8')

        try:
            self.im_pub.publish(self.im)
        except CvBridgeError as e:
            print(e)

        rospy.loginfo("ANGULAR VALUE SENT===>"+str(angulo))
        twist_object = Twist2DStamped();
        patito_detected = vercositas(crop_img,"patitos")
        azul_detected = 0 # vercositas(crop_img,"azul")
        redline_detected = vercositas(crop_img,"redline")
        if patito_detected==1:
          giros=0.45
          avanzo=-0.47
          #girar 1
          twist_object.v =avanzo;
          cte=giros
          twist_object.omega = cte;
          self.cmd_vel_pub.publish(twist_object)
          #giro 2
          #twist_object.v = avanzo;
          #cte=-giro
          #twist_object.omega = cte;
          #self.cmd_vel_pub.publish(twist_object)
        elif azul_detected==1:
          twist_object.v =-0.47;
          cte=-1
          twist_object.omega = cte;
          self.cmd_vel_pub.publish(twist_object)
        elif redline_detected==1:
          twist_object.v =0;
          cte=0
          twist_object.omega = cte;
          self.cmd_vel_pub.publish(twist_object)
        else:
          twist_object.v =-0.5; #calibrar
          cte=6 #calibrar
          twist_object.omega = cte*angulo;
          # Make it start turning
          self.cmd_vel_pub.publish(twist_object)      

# nmn = 1000
def main():
    # global nmn
    # nmn+=1
    while True:
        for i in range(1000):
            if i%1000==0:
                rospy.init_node('line_following_node', anonymous=True)
    
                line_follower_object = LineFollower()
        
                rospy.spin()
    
    
if __name__ == '__main__':
    main() 


  
            


