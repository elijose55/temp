#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Esta classe deve conter todas as suas implementações relevantes para seu filtro de partículas
"""

from pf import Particle, create_particles, draw_random_sample
import numpy as np
import inspercles # necessário para o a função nb_lidar que simula o laser
import math
from scipy.stats import norm


largura = 775 # largura do mapa
altura = 748  # altura do mapa


# Robo
robot = Particle(largura/2, altura/2, math.pi/4, 1.0)

# Nuvem de particulas
particulas = []

num_particulas = 500

p_h = 1/num_particulas

controle = 0

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

movimentos_relativos = [[0, -math.pi/3],[10, 0],[10, 0], [10, 0], [10, 0],[15, 0],[15, 0],[15, 0],[0, -math.pi/2],[10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [0, -math.pi/2], 
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [0, -math.pi/2], 
					   [10,0], [0, -math.pi/4], [10,0], [10,0], [10,0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0]]



movimentos = movimentos_relativos



def cria_particulas(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=num_particulas):
	"""
		Cria uma lista de partículas distribuídas de forma uniforme entre minx, miny, maxx e maxy
	"""
	part = []
	for i in range(n_particulas):
		x = np.random.uniform(minx, maxx)
		y = np.random.uniform(miny,maxy)
		theta = np.random.uniform(0, math.pi*2)
		p = Particle(x, y, theta, w=1.0) # A prob. w vai ser normalizada depois
		part.append(p)

	return part
	
def move_particulas(particulas, movimento):
	"""
		Recebe um movimento na forma [x,y, theta]  e o aplica a todas as partículas
		Assumindo um desvio padrão para cada um dos valores
		Esta função não precisa devolver nada, e sim alterar as partículas recebidas.
		
		Você não precisa mover o robô. O código fornecido pelos professores fará isso
		
	"""
	for i in particulas:
		i.move_relative(movimento)
		dmov = np.random.normal(0, 3)
		dtheta = np.random.normal(0, math.radians(2))
		i.move_relative([dmov, dtheta])

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



	sig = 8
	soma_pdh = 0 
	leitura_robo = inspercles.nb_lidar(robot, angles)

	for i in particulas:
		leitura_particula = inspercles.nb_lidar(i, angles)
		pdh = 0

		for d in leitura_particula:
		  pdh += norm.pdf(leitura_particula[d], loc = leitura_robo[d], scale = sig)

		particles = i.w * pdh 
		i.w = particles
		soma_pdh += particles

		
	alpha = 1/soma_pdh
		  
	for i in particulas:  
		i.w = i.w * alpha
		


'''
	leitura_robo = inspercles.nb_lidar(robot, angles)
	sigma = 8
	somap_d_h = 0
	lc = []

	for i in particulas:
		leitura_particula = inspercles.nb_lidar(i,angles)

		pdh = 0
		for d in leitura_particula:
			pdh += norm.pdf(leitura_particula[d], loc = leitura_robo[d], scale = sigma)

		i.w *= pdh
		somap_d_h += i.w

	alpha = 1/somap_d_h

	for i in range(len(particulas)):
		particulas[i].w *= alpha



'''

'''

	lc = []
	p_d_h =0
	soma_pdh = 0
	sigma = 6
	leitura_particulas = []
	

	for i in particulas:
		uma_leitura = inspercles.nb_lidar(i, angles)

		p_d_h =0


		#leitura_particulas.append(uma_leitura)


		for d in uma_leitura:
			p_d_h += norm.pdf((leitura_robo[d]-uma_leitura[d]),0,sigma)

		soma_pdh += p_d_h

		lc.append(p_d_h)

	alfa = 1/soma_pdh
	for i in range(len(particulas)):	
		particulas[i].w = lc[i]*alfa


		
-----------------------------
	if controle = 1:
		for i in leitura_particulas.itervalues():
		norm.pdf()
	alfa = 
	w = p_d_h * p_h * alfa

	p_d_h = 

	# Voce vai precisar calcular a leitura para cada particula usando inspercles.nb_lidar e depois atualizar as probabilidades

'''
	
	
def reamostrar(particulas, n_particulas = num_particulas):
	"""
		Reamostra as partículas devolvendo novas particulas sorteadas
		de acordo com a probabilidade e deslocadas de acordo com uma variação normal    
		
		O notebook como_sortear tem dicas que podem ser úteis
		
		Depois de reamostradas todas as partículas precisam novamente ser deixadas com probabilidade igual
		
		Use 1/n ou 1, não importa desde que seja a mesma
	"""
	particulas_pesos = [p.w for p in particulas]

	novas_particulas = draw_random_sample(particulas, particulas_pesos, num_particulas)
	for i in novas_particulas:
		i.x += np.random.normal(0, 12)
		i.y += np.random.normal(0, 12)
		i.theta += np.random.normal(0, math.radians(11))


	for p in novas_particulas:
		p.w = 1





	return novas_particulas


	







