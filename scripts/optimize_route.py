#!/usr/bin/env python
# https://the-examples-book.com/data-science/gis/route-optimization

import sys
import numpy as np

import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

import random
import numpy as np
import matplotlib.pyplot as plt

from lib_opt_traj import *

# Define the parameters for the genetic algorithm
POPULATION_SIZE = 300
NUM_GENERATIONS = 150
MUTATION_RATE = 0.01


# Define functions for the genetic algorithm
def generate_random_route(clusters):
    # Generate a random route by shuffling the list of city names
    city_names = list(clusters.keys())
    random.shuffle(city_names)
    return city_names

def calculate_distance(route, clusters):
    # Calculate the total distance of a route by summing the distances between consecutive clusters
    distance = 0
    for i in range(len(route)-1):
        city1 = clusters[route[i]]
        city2 = clusters[route[i+1]]
        distance += np.linalg.norm(np.array(city1)-np.array(city2))
    return distance

def create_initial_population(clusters, population_size):
    # Create an initial population of random routes
    population = []
    for i in range(population_size):
        route = generate_random_route(clusters)
        population.append(route)
    return population

def select_parents(population):
    # Select two parents from the population using tournament selection
    tournament_size = 3
    tournament = random.sample(population, tournament_size)
    tournament.sort(key=lambda x: calculate_distance(x, clusters))
    return tournament[0], tournament[1]

def crossover(parent1, parent2):
    # Perform crossover between two parents to generate a new child
    child = [None]*len(parent1)
    start = random.randint(0, len(parent1)-2)
    end = random.randint(start+1, len(parent1)-1)
    for i in range(start, end+1):
        child[i] = parent1[i]
    j = 0
    for i in range(len(parent2)):
        if parent2[i] not in child:
            while child[j] is not None:
                j += 1
            child[j] = parent2[i]
    return child

def mutate(route, mutation_rate):
    # Perform mutation on a route by swapping two clusters with a certain probability
    for i in range(len(route)):
        if random.random() < mutation_rate:
            j = random.randint(0, len(route)-1)
            route[i], route[j] = route[j], route[i]
    return route

def evolve_population(population, mutation_rate):
    # Evolve the population by selecting parents, performing crossover and mutation, and creating a new generation
    new_population = []
    for i in range(len(population)):
        parent1, parent2 = select_parents(population)
        child = crossover(parent1, parent2)
        child = mutate(child, mutation_rate)
        new_population.append(child)
    return new_population


def run_genetic_algorithm(clusters, population_size, num_generations, mutation_rate):
    # Run the genetic algorithm to find the shortest route that visits all clusters
    population = create_initial_population(clusters, population_size)
    shortest_distance = float('inf')
    evol = []
    evol_x = []
    for i in range(num_generations):
        population.sort(key=lambda x: calculate_distance(x, clusters))
        shortest_route = population[0]
        shortest_distance = calculate_distance(shortest_route, clusters)
        evol.append(shortest_distance)
        evol_x.append(i)
        print(f"Generation {i+1}: Shortest distance = {shortest_distance:.2f}")
        population = evolve_population(population, mutation_rate)
    
    # Add the first city to the end of the shortest route to make it a closed loop
    shortest_route.append(shortest_route[0])
    
    # Plot the shortest route
    x = [clusters[city][0] for city in shortest_route]
    y = [clusters[city][1] for city in shortest_route]

    # delete last element of array
    x.pop()
    y.pop()
    print("xxx",x[len(x)-1])
    print("x",x,"y",y)

    fig, axs = plt.subplots(2)
    #plt.title("Optimized trajectory")
    axs[0].plot(x, y, '-o')
    # Start Point
    axs[0].plot(0, 0, marker="o", markersize=10, markeredgecolor="red", markerfacecolor="black")
    # First Point 
    axs[0].plot(x[0], y[0], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")
    # End Point
    axs[0].plot(x[len(x)-1], y[len(x)-1], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")

    # Arrows to define star and direction 
    axs[0].arrow(0, 0, x[0] - 0, y[0] - 0, color='r', length_includes_head=True, head_width=0.1)
    #axs[0].arrow(x[0], y[0], x[1] - x[0], y[1] - y[0], color='r', length_includes_head=True, head_width=0.2)
    axs[0].grid(color='gray', linestyle='--', linewidth=0.5)
    
    plt.xlabel('x')
    plt.ylabel('y')

    # Print evol

    X_=np.linspace(0, i, NUM_GENERATIONS)
    #axs[1].plot(X_, Y_,'gray')
    axs[1].plot(evol_x, evol, '.k')

    model3 = np.poly1d(np.polyfit(evol_x, evol, 4))
    axs[1].plot(X_, model3(X_), color='red')
    


    #plt.title("evol")
    plt.xlabel('epoch')
    plt.ylabel('dist')
    plt.grid(color='gray', linestyle='--', linewidth=0.5)
    plt.show()

def test_funct(centroids):
    global clusters
    print("all")
    
    # Run the genetic algorithm
    # centroids = send_centorids_for_optim(centr)

    # Create other function that recieves the centroids, generate the city clusters
    # and call the run genetic algorithm
    clusters = generate_cities_from_clusters(len(centroids),centroids)

    run_genetic_algorithm(clusters, POPULATION_SIZE, NUM_GENERATIONS, MUTATION_RATE)




