#ifndef __GENETIC_ALGORITHM_HPP__
#define __GENETIC_ALGORITHM_HPP__

#include "cTaylorModel3.h"

# define POPSIZE 50
//# define MAXGENS 1000
# define MAXGENS 500
//# define NVARS 3  // ����������3���Ա���
# define NVARS 1    // if NVARS<2, crossover modification will not be available.
# define PXOVER 0.8
# define PMUTATION 0.15
//
//  Each GENOTYPE is a member of the population, with
//  gene: a string of variables,
//  fitness: the fitness
//  upper: the variable upper bounds,
//  lower: the variable lower bounds,
//  rfitness: the relative fitness,
//  cfitness: the cumulative fitness.
//
struct genotype
{
    double gene[NVARS];
    double fitness;
    double upper[NVARS];
    double lower[NVARS];
    double rfitness;
    double cfitness;
};
struct ga_res {
    double x;
    double minimum;
};

extern struct genotype population[POPSIZE + 1];
extern struct genotype newpopulation[POPSIZE + 1];

//int main();
ga_res GA(double lb, double ub, cTaylorModel3& dis_square);
void crossover(int& seed);
void elitist();
void evaluate(cTaylorModel3& p_square);
int i4_uniform_ab(int a, int b, int& seed);
void initialize(double lb[], double ub[], int& seed);
void keep_the_best();
void mutate(int& seed);
double r8_uniform_ab(double a, double b, int& seed);
void selector(int& seed);
void Xover(int one, int two, int& seed);
void report(int generation);
void resetpopulation();
double P_evaluate(cTaylorModel3 &dis, double t);
ga_res findminimum(cTaylorModel3& dis, cInterval &T);

#endif // !__GENETIC_ALGORITHM_HPP__
