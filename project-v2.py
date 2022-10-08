from os import get_inheritable
import numpy as np
from queue import Queue
from collections import defaultdict

import matplotlib.pyplot as plt

import re

from math import radians, cos, sin, asin, sqrt

class Vertex:
    def __init__(self, key, lat, lon):
        self.id = key
        self.lat = lat
        self.lon = lon
        self.connectedTo = {}

    def addNeighbor(self, nbr, weight=0):
        self.connectedTo[nbr] = weight

    def getConnections(self):
        return list(self.connectedTo.keys())

    def getId(self):
        return self.id

    def getWeight(self, nbr):
        return self.connectedTo[nbr]


class Graph:
    def __init__(self):
        self.vertList = {}
    # cria o objeto vertice e adiciona na lista do grafo

    def addVertex(self, key, lat, lon):
        v = Vertex(key, lat, lon)
        self.vertList[key] = v

    # retorna o objeto vértice pelo id
    def getVertex(self, key):
        return self.vertList[key]

    # cria um relacionamento entre dois vértices
    def addEdge(self, f, t, cost):
        # verfifica se os vértices estão na lista, caso contrário retorna falso
        v_f = self.vertList.get(f, False)
        v_t = self.vertList.get(t, False)
        if v_f and v_t:
            v_f.addNeighbor(v_t, cost)
        # v_t.addNeighbor(v_f, cost)

    def getVertices(self):
        # retorna os ids dos vértices do grafo
        return list(self.vertList.keys())

    def getEdges(self):
        # retorna o conjunto de objetos do tipo vertice na forma de tupla (v1, v2, peso)
        # utiliza set pq a conexao entre v1 e v2 pode repetir. Então ele remove duplicidade.
        edges = set()
        for v1 in self.vertList.values():
            for v2, cost in v1.connectedTo.items():
                edges.add((v1, v2, cost))
        return list(edges)

    def dijkstra(self, start, maxD=1e309):
        tdist = defaultdict(lambda: 1e309)
        tdist[start] = 0
        preceding_node = {}
        unvisited = set(self.getVertices())
        while unvisited:
            current = unvisited.intersection(tdist.keys())
            if not current:
                break
            min_node = min(current, key=tdist.get)
            unvisited.remove(min_node)
            vertex = self.getVertex(min_node)
            for neighbour in vertex.getConnections():
                d = tdist[min_node] + vertex.getWeight(neighbour)
                if tdist[neighbour.id] > d and maxD >= d:
                    tdist[neighbour.id] = d
                    preceding_node[neighbour.id] = min_node
        return tdist, preceding_node
    
    def min_path(self, start, end, maxD=1e309):
        tdist, preceding_node = self.dijkstra(start, maxD)
        dist = tdist[end]
        backpath = [end]
        try:
            while end != start:
                end = preceding_node[end]
                backpath.append(end)
            path = list(reversed(backpath))
        except KeyError:
            path = None
        return dist, path

#adiciona vertices a partir do arquivo map.osm.txt
def carrega_pontos(file_name, grafo):
    with open(file_name) as arquivo:
        for linha in arquivo:
            points = re.findall(r'[-+]?\d+.\d+', linha)
            grafo.addVertex(points[0], float(points[1]), float(points[2]))
    return grafo

#adiciona as arestas e o peso destas a partir do arquivo uesb.adjlist
def adiciona_arestas(file_name, grafo):
    with open(file_name) as arquivo:
        for linha in arquivo:
            points = linha.split() #quebra a linha ao encontrar um espaço
            #a = ponto de partida
            a = points[0]
            lat1, lon1 = busca_cord_a('map.osm.txt', a)
            #Necessário converter para float ao "desempacotar" a tupla
            lat1 = float(lat1)
            lon1 = float(lon1)
            for b in points[1: ]: #b = ponto de chegada
                lat2, lon2 = busca_cord_b('map.osm.txt', b)
                lat2 = float(lat2)
                lon2 = float(lon2)
                cost = haversine(lat1, lon1, lat2, lon2)
                grafo.addEdge(a, b, cost)
    return grafo

#busca coordenadas do ponto A no arquivo map.osm.txt
def busca_cord_a(file_name, a):
    file_name = "map.osm.txt"
    with open(file_name) as arquivo:
        for linha in arquivo:
            points = re.findall(r'[-+]?\d+.\d+', linha)
            if a == points[0]:
                return (points[1],points[2])

#busca coordenadas do ponto B no arquivo map.osm.txt
def busca_cord_b(file_name, b):
    file_name = "map.osm.txt"
    with open(file_name) as arquivo:
        for linha in arquivo:
            points = re.findall(r'[-+]?\d+.\d+', linha)
            if b == points[0]:
                return (points[1],points[2])

#calcula distância entre dois pontos
def haversine(lat1, lon1, lat2, lon2):

    #converter de graus para radianos 
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    lat2 = radians(lat2)
    lon2 = radians(lon2)

    #Fórmula de Haversine
    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    

    #6371 raio da terra em km
    #3956 raio da terra em milhas
    #6371000 raio da terra em metros
    distance = c * 6371000

    return distance

#plot do grafo
def plot_pontos(file_name):
    file_name = "map.osm.txt"
    x = list()
    y = list()
    with open(file_name) as arquivo:
        for line in arquivo:
            points = re.findall(r'[-+]?\d+.\d+', line)
            x.append(float(points[1]))
            y.append(float(points[2]))

    fig,ax=plt.subplots()

    ax.scatter(x,y)

    fig.canvas.mpl_connect('button_press_event',on_click)

    #print('\nPonto 1 :{}\nPonto 2: {}'.format(points[0], points[1]))

    plt.show()


#Variáveis globais
xcoords = [1]
ycoords = [1]
points = []


#captura dois pontos a partir de clicks na tela (evento de mouse)
#Resolver problemas da função
def on_click(event):

    #1 = left   /   2 = center  /   3 = right   /   4 = scroll up   /   5 = scroll down
    
    #Click com botão direito -> pega ponto
    if event.button == 3:

        xcoords[0] = (event.xdata)
        ycoords[0] = (event.ydata)
           

        ponto = aproxima_ponto('map.osm.txt', xcoords, ycoords)
        points.append(ponto)

        #print('x atual: ', xcoords[0])
        #print('y atual: ', ycoords[0])
        #print('pontos: ', points)~
        if(len(points) == 2):
            result = g.min_path(points[0], points[1])
            print(result[1])
            xlat = []
            ylong = [] 
            for key in result[1]:
                aux = g.getVertex(key)
                xlat.append(aux.lat)
                ylong.append(aux.lon)

            points.clear()
            plt.plot(xlat, ylong, color = 'r')
            #ax.lines.pop(0)
            

#Aproxima o ponto do click de algum ponto existente na lista
def aproxima_ponto(file_name, xcoords, ycoords):
    menor = 1000000000000
    id_menor = 0
    #Busca no arquivo por um ponto próximo do passado pelo click
    file_name = "map.osm.txt"
    with open(file_name) as arquivo:
        for linha in arquivo:
            points = re.findall(r'[-+]?\d+.\d+', linha)
            xe = float(points[1])
            ye = float(points[2])
            distance = sqrt((xcoords[0] - xe)**2 + (ycoords[0] - ye)**2) #Utilizando fórmula da Distância Euclidiana
            if distance < menor:
                menor = distance
                id_menor = points[0]

    return id_menor


if __name__ == "__main__":
    g = Graph()
    g = carrega_pontos('map.osm.txt', g)
    g = adiciona_arestas('uesb.adjlist', g)
    
    plot_pontos('map.osm.txt')
    
    #print(g.min_path('2323091988', '2323091996'))