#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Esta classe deve conter todas as suas implementações relevantes para seu filtro de partículas
"""

from pf import Particle, create_particles
import numpy as np
import inspercles # necessário para o a função nb_lidar que simula o laser
import math


largura = 775 # largura do mapa
altura = 748  # altura do mapa

# Robo
robot = Particle(largura/2, altura/2, math.pi/4, 1.0)

# Nuvem de particulas
particulas = []

num_particulas = 10


# Os angulos em que o robo simulado vai ter sensores
angles = np.linspace(0.0, 2*math.pi, num=8, endpoint=False)

# Lista mais longa
movimentos_longos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0],
              [0,0,math.pi/12.0], [0, 0, math.pi/12.0], [0, 0, math.pi/12],[0,0,-math.pi/4],
              [-5, 0, 0],[-5,0,0], [-5,0,0], [-10,0,0],[-10,0,0], [-10,0,0],[-10,0,0],[-10,0,0],[-15,0,0],
              [0,0,-math.pi/4],[0, 10, 0], [0,10,0], [0, 10, 0], [0,10,0], [0,0,math.pi/8], [0,10,0], [0,10,0], 
              [0,10,0], [0,10,0], [0,10,0],[0,10,0],
              [0,0,-math.radians(90)],
              [math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],
              [math.cos(math.pi/3)*10, math.sin(math.pi/3),0]]

# Lista curta
movimentos_curtos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0]]

movimentos = movimentos_longos



def cria_particulas(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=num_particulas):
    """
        Cria uma lista de partículas distribuídas de forma uniforme entre minx, miny, maxx e maxy
    """
    create_particles = []
    for i in range(n_particulas):
        x = np.random.uniform(minx, maxx)
        y = np.random.uniform(miny,maxy)
        theta = np.random.uniform(0, math.pi*2)
        p = Particle(x, y, theta, w=1.0) # A prob. w vai ser normalizada depois
        create_particles.append(p)

    return create_particles
    
def move_particulas(particulas, movimento):
    """
        Recebe um movimento na forma [x,y, theta]  e o aplica a todas as partículas
        Assumindo um desvio padrão para cada um dos valores
        Esta função não precisa devolver nada, e sim alterar as partículas recebidas.
        
        Você não precisa mover o robô. O código fornecido pelos professores fará isso
        
    """

    for i in particulas:
    	i.move_relative(movimento)


    for i in particulas:
    	i.x += np.random.normal(0, 1.5)
    	i.y += np.random.normal(0, 1.5)
    	i.theta += np.random.normal(0, 1)

    return particulas
    
def leituras_laser_evidencias(robot, particulas):
    """
        Realiza leituras simuladas do laser para o robo e as particulas
        Depois incorpora a evidência calculando
        P(H|D) para todas as particulas
        Lembre-se de que a formula $P(z_t | x_t) = \alpha \prod_{j}^M{e^{\frac{-(z_j - \hat{z_j})}{2\sigma^2}}}$ 
        responde somente P(Hi|D), em que H é a hi
        
        Esta função não precisa retornar nada, mas as partículas precisa ter o seu w recalculado. 
        
        Você vai precisar calcular para o robo
        
    """
    leitura_particulas = []
    leitura_robo = inspercles.nb_lidar(robot, angles)
    for i in particulas:
    	leitura_particulas.append(inspercles.nb_lidar(particulas[i], angles))

    
    # Voce vai precisar calcular a leitura para cada particula usando inspercles.nb_lidar e depois atualizar as probabilidades


    
    
def reamostrar(particulas, n_particulas = num_particulas):
    """
        Reamostra as partículas devolvendo novas particulas sorteadas
        de acordo com a probabilidade e deslocadas de acordo com uma variação normal    
        
        O notebook como_sortear tem dicas que podem ser úteis
        
        Depois de reamostradas todas as partículas precisam novamente ser deixadas com probabilidade igual
        
        Use 1/n ou 1, não importa desde que seja a mesma
    """
    particulas_pesos = [np.round(p.w, decimals=3) for p in particulas]

    novas_particulas = draw_random_sample(particulas, particulas_pesos, 10)
    for i in novas_particulas:
    	i.x += np.random.normal(0, 4)
    	i.y += np.random.normal(0, 4)


    for p in novas_particulas:
    	p.w = 1




    return particulas


    






