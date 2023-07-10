# Duckietown-P01.2-Robotica-Otono-2023
Repositorio para el proyecto duckietown P01.2 correspondiente al curso de Robótica (ME5150-1) durante Otoño del del 2023. Fue llevado a cabo por Benjamín Fuentes, Maximiliano Flores, Antonio Díaz y Vicente Tello, alumnos de ingeniería civil mecánica.

Este proyecto se desarrolló en Ubuntu, utilizando ROS, el cual tiene como base las librerías proporcionadas por DuckietownChile. Nuestro código toma inspiración desde [DeePiCar](https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96), por lo que puede servir de ayuda para otros usuarios

El archivo 'Código Duckietown P01.2' contiene todo el código utilizado para la detección de imágenes y procesamiento en la Raspy de Duckiebot. Este archivo es el mismo al que se encuentra en robot ubicado en el fablab, correspondiente al Duckiebot Número 4. Este archivo se puede encontrar usando el comando 'cd /duckietown/cat_kin/src/ros_cap/src/'en la terminal del duckiebot y tiene el nombre de 'Pato.py'. A grandes rasgos el archivo utiliza la detección de colores para distinguir las máscaras amarillas y blancas del circuito, permitiendo guíar el pato. Adicionalmente usando una mascara de color verde detecta patos y los esquiva (necesita algo de color verde). 

Por otro lado, el archivo 'Patostick' contiene un código ya existente que permite controlar el Duckiebot utilizando un Joystick de xbox360 
