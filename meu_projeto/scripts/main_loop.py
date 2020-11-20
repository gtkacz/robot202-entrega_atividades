#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Gabriel Mitelman Tkacz", "Rafael Seicalli Malcervelli", "Enrico Venturini Costa"]

import cv2, rospy, time, tf2_ros, math
from sensor_msgs.msg   import Image, CompressedImage
from cv_bridge         import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from tf import transformations
from tf import TransformerROS
from auxiliar import encontra_cm, controla_direcao, muda_velocidade, rotacionar_procurar_creeper, distacia_ate_creeper, garra

def ajustar_posicao_inicial(cor_creeper_objetivo, maquina_estados):
    if cor_creeper_objetivo == "blue":
        velocidade.angular.z = v_angular
    elif cor_creeper_objetivo == "orange":
        velocidade.angular.z = -v_angular
        print("LARANJA SELECIONADO")

    else:
        maquina_estados = "buscar_creeper"
        print("buscar creeper")

    return velocidade, maquina_estados

def ajustar_posicao_depois_de_pegar_creeper():
    velocidade.linear.x = 0.3
    velocidade.angular.z = -v_angular

    return velocidade


def roda_todo_frame(imagem):
    global velocidade
    global maquina_estados

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp

    try:
        antes = time.clock()
        bgr_img = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        if maquina_estados == "ajustar_posicao_inicial":
            velocidade, maquina_estados = ajustar_posicao_inicial(cor_creeper_objetivo, maquina_estados)

        elif maquina_estados == "buscar_creeper":
            cor_mascara = 'yellow'
            direcao_seguir = controla_direcao(bgr_img, cor_mascara) 

            velocidade = muda_velocidade(direcao_seguir, velocidade, vel_lin, v_angular)
            is_proximo_fim_percurso = rotacionar_procurar_creeper(bgr_img)

            if is_proximo_fim_percurso:
                maquina_estados = "buscar_creeper_pela_mascara"

        elif maquina_estados == "buscar_creeper_pela_mascara":
            velocidade.angular.z = -v_angular
            velocidade.linear.x = 0
            try:
                encontra_cm(bgr_img, cor_creeper_objetivo)
                maquina_estados = "aproximar_creeper"
                velocidade.angular.z = 0
                velocidade.linear.x = 0
            except:
                print("ainda nao enxergou creeper")

        elif maquina_estados == "aproximar_creeper":
            direcao_seguir = controla_direcao(bgr_img, cor_creeper_objetivo)
            velocidade = muda_velocidade(direcao_seguir, velocidade, vel_lin, v_angular)

            if distacia_ate_creeper(bgr_img, id_creeper_objetivo):
                maquina_estados = "para_proximo_ao_creeper"
                print("Deve parar em frente ao creeper")


        elif maquina_estados == "para_proximo_ao_creeper":
            v_lin = velocidade.linear.x
            velocidade.angular.z = 0

            if v_lin > 0.043:
                velocidade.linear.x -= 0.039
            else:
                maquina_estados = "usar_garra_no_creeper"


        elif maquina_estados == "usar_garra_no_creeper":
            print("PEGANDO COM GARRA HEHE")

            print("open") #dando publish duas vezes, pq o primeiro geralmente dá problema
            garra.open()

            garra.open()
            rospy.sleep(0.1)

            print("up")
            garra.up()
            rospy.sleep(0.2)

            print("close")
            garra.close()
            rospy.sleep(0.4)

            print("up")
            garra.up()
            rospy.sleep(0.5)

            maquina_estados = "ajustar_posicao_depois_de_pegar_creeper"
            print(maquina_estados)
        
        elif maquina_estados == "ajustar_posicao_depois_de_pegar_creeper":
            print("ajustar_posicao_depois_de_pegar_creeper")
            velocidade = ajustar_posicao_depois_de_pegar_creeper()

        elif maquina_estados == "carregar_creeper_pra_pista":
            print("carregando creeper de volta pra pista")

            cor_mascara = 'yellow'
            direcao_seguir = controla_direcao(bgr_img, cor_mascara) 

            velocidade = muda_velocidade(direcao_seguir, velocidade, vel_lin, v_angular)

        
        cv2.imshow("camera robo", bgr_img)       
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

bridge = CvBridge()
tfl = 0
tf_buffer = tf2_ros.Buffer()

vel_lin = 0.25
v_angular = math.pi/8
velocidade = Twist(Vector3(0,0,0), Vector3(0,0, 0))
maquina_estados = "ajustar_posicao_inicial"

cor_creeper_objetivo = "blue"
id_creeper_objetivo = 22

garra = garra()

if __name__=="__main__":
    rospy.init_node("cor")  
    topico_imagem = "/camera/image/compressed"
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    out_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 

    garra.close()
    garra.inicializar_garra()

    try:
        while not rospy.is_shutdown():
            out_vel.publish(velocidade)
            rospy.sleep(0.1)
            print(maquina_estados)

            if maquina_estados == "ajustar_posicao_inicial":
                out_vel.publish(velocidade)
                tempo_rotacao_inicial = (math.pi/2)/abs(velocidade.angular.z)
                rospy.sleep(tempo_rotacao_inicial)
                maquina_estados = "buscar_creeper"

            elif maquina_estados == "ajustar_posicao_depois_de_pegar_creeper":
                out_vel.publish(velocidade)
                tempo_rotacao_volta_pista = (math.pi/4.5)/velocidade.angular.z
                rospy.sleep(tempo_rotacao_volta_pista)
                maquina_estados = "carregar_creeper_pra_pista"

            out_vel.publish(velocidade)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
