#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include "Simulator.hpp"
#include <stdlib.h>
struct Vertex
{
    enum
	{
	    TYPE_NONE = 0,
	    TYPE_INIT = 1,
	    TYPE_GOAL = 2
	};

    int    m_parent;
    double m_state[Simulator::STATE_NR_DIMS];
    int    m_type;
    int    m_nchildren;

};



class MotionPlanner
{
public:
    MotionPlanner(Simulator * const simulator);

    ~MotionPlanner(void);

    void ExtendRandom(void);

    void ExtendRRT(void);

    void ExtendEST(void);

    void ExtendMyApproach(void);

	double distFromGoal(int, const double[]);

protected:

    //here
    int decideVid();
    double calc(int vert){
        return 1.0 / (1.0 * pow( (double)m_vertices[vert]->m_nchildren, 2));
    }

    bool IsProblemSolved(void)
    {
	return m_vidAtGoal >= 0;
    }

    void GetPathFromInitToGoal(std::vector<int> *path) const;

    void AddVertex(Vertex * const v);

    void ExtendTree(const int    vid,
		    const double sto[]);
    bool reachedDest(const double currentPos[],const double dest[]);

    Simulator            *m_simulator;
    std::vector<Vertex *> m_vertices;
    int                   m_vidAtGoal;
    double                m_totalSolveTime;


    friend class Graphics;
};

#endif
