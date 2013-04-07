using System;
using System.Collections.Generic;

namespace DrRobot.JaguarControl
{
    public class Genetic
    {

        private class Organism
        {
            enum Chrom { K_pho, K_a, K_b, K_p, K_i, K_d };

            #region Organism Data Members
            private static Random rnd_ = new Random();

            private double[] genes_ = null;
            private double geneMax = 127;

            public double fitness;

            private int mutationFactor_;

            #endregion

            public Organism(double K_pho, double K_a, double K_b, double K_p, double K_i, double K_d, int mutationFactor)
            {
                genes_ = new double[6] { K_pho, K_a, K_b, K_p, K_i, K_d };
                fitness = 0;
                mutationFactor_ = (int) Math.Max(0.0, Math.Min(geneMax, mutationFactor));
            }

            public Organism(int mutationFactor)
            {
                genes_ = new double[6] { 0, 0, 0, 0, 0, 0 };
                fitness = 0;
                mutationFactor_ = (int) Math.Max(0, Math.Min(geneMax, mutationFactor));
                Mutate();
            }

            #region Organism Methods

            // Perturb the Chromosomes of a gene.
            public void Mutate()
            {
                for (Chrom it = Chrom.K_pho; it <= Chrom.K_d; it++)
                {
                    genes_[(int)it] += Organism.rnd_.Next(-mutationFactor_, mutationFactor_);
                }
                Stabilize();
            }

            // Stabilize the Chromosomes of a gene.
            public void Stabilize()
            {
                //while (getKpho() <= 0 || Math.Abs(getKpho()) > geneMax)
                //{
                //    genes_[(int)Chrom.K_pho] += Organism.rnd_.Next(-mutationFactor_, mutationFactor_);
                //}

                //while (getKb() >= 0 || Math.Abs(getKb()) > geneMax)
                //{
                //    genes_[(int)Chrom.K_b] += Organism.rnd_.Next(-mutationFactor_, mutationFactor_);
                //}

                //while (getKa() <= getKpho() || Math.Abs(getKa()) > geneMax)
                //{
                //    genes_[(int)Chrom.K_a] += Organism.rnd_.Next(-mutationFactor_, mutationFactor_);
                //}

                for (Chrom it = Chrom.K_p; it <= Chrom.K_d; it++)
                {
                    while (Math.Abs(genes_[(int)it]) > geneMax)
                    {
                        genes_[(int)it] += Organism.rnd_.Next(-mutationFactor_, mutationFactor_);
                    }
                }
            }

            // Simulate reproduction of two different organism
            public Organism Crossover(Organism parent2)
            {
                Organism ret = new Organism(
                    (getKpho() + parent2.getKpho()) / 2,
                    (getKa() + parent2.getKa()) / 2,
                    (getKb() + parent2.getKb()) / 2,
                    (getKp() + parent2.getKp()) / 2,
                    (getKi() + parent2.getKi()) / 2,
                    (getKd() + parent2.getKd()) / 2,
                    mutationFactor_);

                ret.Stabilize();
                return ret;
            }
            #endregion

            #region Accessor Methods

            public double[] getGenes()
            {
                return genes_;
            }

            public double getKpho()
            {
                return genes_[(int)Chrom.K_pho];
            }

            public double getKa()
            {
                return genes_[(int)Chrom.K_a];
            }

            public double getKb()
            {
                return genes_[(int)Chrom.K_b];
            }

            public double getKp()
            {
                return genes_[(int)Chrom.K_p];
            }

            public double getKi()
            {
                return genes_[(int)Chrom.K_i];
            }

            public double getKd()
            {
                return genes_[(int)Chrom.K_d];
            }

            #endregion


        }

        #region Genetic Algorithm Data
        private static Random rnd_ = new Random();
        private int numGenerations_;

        private int curGen_;
        private bool changedOrg_;
        private Organism bestOrg_;

        private int curOrg_;
        private double curOrgClosestPho_;
        private double curOrgClosestTh_;
        private double curOrgFarthestPho_;
        private double curOrgFarthestTh_;

        private int maxSteps_;
        private int curStep_;


        private int popSize_;
        private int numParents_;
        private int mutationRate_;
        private int mutationFactor_;

        private List<Organism> pop_;
        private Navigation nav_;
        #endregion

        public Genetic(int numGenerations, int popSize, int mutationRate, int mutationFactor,
            Navigation nav, int numParents, int maxSteps)
        {
            numGenerations_ = numGenerations;
            popSize_ = popSize;
            numParents_ = Math.Min(numParents, popSize_);
            maxSteps_ = maxSteps;
            mutationRate_ = mutationRate;
            mutationFactor_ = mutationFactor;
            nav_ = nav;

            // Create the initial population
            pop_ = new List<Organism>(popSize_);
            for (int i = 0; i < popSize_; i++)
            {
                pop_.Add(new Organism(mutationFactor_));
            }

            curGen_ = 0;
            curOrg_ = 0;
            curStep_ = 0;
            changedOrg_ = true;
        }

        public double[] Step(double pho, double deltaTh)
        {

            if (curGen_ == numGenerations_)
            {
                return bestOrg_.getGenes();
            }

            if (curStep_ != maxSteps_)
            {
                deltaTh = Math.Abs(deltaTh);
                pho = Math.Abs(pho);
                // Closest and farthest values are stored as positive values.
                if (curOrgClosestTh_ > deltaTh || curOrgClosestPho_ > pho)
                {
                    curOrgClosestTh_ = deltaTh;
                    curOrgClosestPho_ = pho;
                }

                if (curOrgFarthestTh_ < deltaTh || curOrgFarthestPho_ < pho)
                {
                    curOrgFarthestTh_ = deltaTh;
                    curOrgFarthestPho_ = pho;
                }

                ++curStep_;
                return pop_[curOrg_].getGenes();
            }

            // Cursteps == maxSteps, so calculate and set fitness
            // TODO: Work on units and balance of the fitness function
            pop_[curOrg_].fitness = (100.0 / curOrgClosestPho_) +
                (Math.PI / curOrgClosestTh_) -
                (25.0 * curOrgFarthestPho_) -
                (Math.PI / 2.0 * curOrgFarthestTh_);

            // If this is the last org of the pop, sort by fitness
            // and update bestOrg, then set curOrg to 0 and create 
            // new pop using old pop (crossover and mutate)

            if (curOrg_ == popSize_ - 1)
            {
                pop_.Sort(
                      delegate(Organism p1, Organism p2)
                      {
                          return p1.fitness.CompareTo(p2.fitness);
                      }
                );

                if (pop_[popSize_ - 1].fitness > bestOrg_.fitness)
                {
                    bestOrg_ = pop_[popSize_ - 1];
                }

                makeTNG();
                curOrg_ = 0;
            }
            else
            {
                ++curOrg_;
            }

            // Reset nav for next run
            bool loggingWasOn = nav_.loggingOn;
            double desiredX = nav_.desiredX;
            double desiredY = nav_.desiredY;
            double desiredT = nav_.desiredT;
            nav_.Reset();
            nav_.loggingOn = loggingWasOn;
            nav_.desiredX = desiredX;
            nav_.desiredY = desiredY;
            nav_.desiredT = desiredT;

            // Reset Fitness evaluators.
            curOrgClosestPho_ = double.PositiveInfinity;
            curOrgClosestTh_ = double.PositiveInfinity;
            curOrgFarthestPho_ = 0;
            curOrgFarthestTh_ = 0;

            changedOrg_ = true;
            curStep_ = 0;
            return pop_[curOrg_].getGenes();

        }

        private void makeTNG()
        {
            Organism[] parents = new Organism[numParents_];
            pop_.CopyTo((popSize_ - numParents_), parents, 0, numParents_);
            pop_.Clear();

            Organism baby;
            int p1;
            int p2;
            for (int i = 0; i < popSize_; i++)
            {
                p1 = Genetic.rnd_.Next(0, numParents_ -1);
                do
                {
                    p2 = Genetic.rnd_.Next(0, numParents_ -1);
                } while (p1 == p2);
                
                baby = parents[p1].Crossover(parents[p2]);
                baby.Mutate();
                pop_.Add(baby);
            }
        }
    }
}