using System;
using Random;

public class Genetic
{

    private class Organism
    {
        enum Chrom { K_pho, K_a, K_b, K_p, K_i, K_d };

        #region Organism Data Members
        private static Random rnd_ = new Random();

        private int[] genes_ = null;
        private int geneMax = 127;

        public int fitness;

        private int mutationFactor_;
         
        #endregion

        public Organism(int K_pho, int K_a, int K_b, int K_p, int K_i, int K_d, int mutationFactor)
        {
            genes_ = new int[6] {K_pho, K_a, K_b, K_p, K_i, K_d};
            fitness = 0;
            mutationFactor_ = Math.Max(0, Math.Min(geneMax, mutationFactor));
        }

        public Organism(int mutationFactor)
        {
            genes_ = new int[6] {0, 0, 0, 0, 0, 0};
            fitness = 0;
            mutationFactor_ = Math.Max(0, Math.Min(geneMax, mutationFactor));
            Mutate();
        }

        #region Organism Methods

        // Perturb the Chromosomes of a gene.
        public Organism Mutate()
        {
            for (Chrom it = K_pho; it <= K_d; it++)
            {
                genes_[(int)it] += rnd_.next(-mutationFactor, mutationFactor);
            }
            Stabilize();
        }

        // Stabilize the Chromosomes of a gene.
        public void Stabilize()
        {
            while (getKpho() <= 0 || Math.Abs(getKpho()) > geneMax)
            {
                genes_[(int)Chrom.K_pho] += rnd_.next(-mutationFactor, mutationFactor);
            }

            while (getKb() >= 0 || Math.Abs(getKb()) > geneMax)
            {
                genes_[(int)Chrom.K_b] -= rnd_.next(-mutationFactor, mutationFactor);
            }

            while (getKa() <= getKpho() || Math.Abs(getKa()) > geneMax)
            {
                genes_[(int)Chrom.K_a] += rnd_.next(-mutationFactor, mutationFactor);
            }

            for (Chrom it = K_p; it <= K_d; it++)
            {
                while (Math.Abs(genes_[(int)it]) > geneMax)
                {
                    genes_[(int)it] += rnd_.next(-mutationFactor, mutationFactor);
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
                mutationFactor);

            ret.Stabilize();
            return ret;
        }
        #endregion

        #region Accessor Methods

        public int[] getGenes()
        {
            return genes_;
        }

        public int getKpho()
        {
            return genes_[(int)Chrom.K_pho];
        }

        public int getKa()
        {
            return genes_[(int)Chrom.K_a];
        }

        public int getKb()
        {
            return genes_[(int)Chrom.K_b];
        }

        public int getKp()
        {
            return genes_[(int)Chrom.K_p];
        }

        public int getKi()
        {
            return genes_[(int)Chrom.K_i];
        }

        public int getKd()
        {
            return genes_[(int)Chrom.K_d];
        }

        #endregion


    }
    
    #region Genetic Algorithm Data
    private int numGenerations_;

    private int curGen_;
    private bool changedOrg_;
    private Organism bestOrg_;

    private int curOrg_;
    private int curOrgClosestPho_;
    private int curOrgClosestTh_;
    private int curOrgFarthestPho_;
    private int curOrgFarthestTh_;

    private int maxSteps_;
    private int curStep_;


    private int popSize_;
    private int mutationRate_;
    private int mutationFactor_;
    
    private Organism[] pop_;
    #endregion

    public Genetic(int numGenerations, int popSize, int mutationRate, int mutationFactor)
	{
        numGenerations_ = numGenerations;
        popSize_ = popSize;
        mutationRate_ = mutationRate;
        mutationFactor_ = mutationFactor;

        // Create the initial population
        pop_ = new Organism[popSize];
        for (int i = 0; i < popSize; i++)
        {
            pop_[i] = Organism(mutationFactor_);
        }

        curGen_ = 0;
        curOrg_ = 0;
        changedOrg_ = True;
	}
    
    // TODO: make pop a List<Organism>
    public int[] Step(double deltaPho, double deltaTh)
    {
        if (changedOrg_)
        {
            // TODO: Reset robot
            curStep_ = 0;
            return pop_[curOrg_].getGenes();
        }

        if (curGen_ == numGenerations_)
        {
            return bestOrg_.getGenes();
        }

        if (curStep_ == maxSteps_)
        {
            // update curOrg closest and Farthest values
        }

        // Cursteps = maxSteps, so calculate and set fitness

        // If this is the last org of the pop, sort by fitness
        // and update bestOrg, then set curOrg to 0 and create 
        // new pop using old pop (crossover and mutate)

        // Example Sorting:
        //List<Order> objListOrder = GetOrderList();
        //objListOrder.Sort(
        //    delegate(Order p1, Order p2)
        //    {
        //        return p1.OrderDate.CompareTo(p2.OrderDate);
        //    }
        //);

        // Otherwise, curOrg_++ 

        changedOrg_ = true;

    }
}
