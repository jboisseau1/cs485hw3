#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
    m_simulator = simulator;   

    Vertex *vinit = new Vertex();

    vinit->m_parent   = -1;		
    vinit->m_nchildren= 0;    
    vinit->m_state[0] = m_simulator->GetRobotCenterX();
    vinit->m_state[1] = m_simulator->GetRobotCenterY();

    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  

    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	delete m_vertices[i];
}


void MotionPlanner::ExtendTree(const int    vid, 
			       const double sto[])
{
//your code
	//make a small part (size of step) of the line from the vertex from m_vertices[vid] to the point sto
	//effectively you're just getting the vector made by the x and y coordinates of the vertex and sto
	//then you're normalizing it and scaling it by the step size.
	//check if its a valid point by runnning through the path to the new vertices if it is valid add it to the tree 
	//if it isnt then dont add it and move on.
	//could potentially just run from the vertex to the new point made after you normalized and scaled but unsure
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);

//your code
	//sto is chosen based on some probbability from the goal region using samplestate  
	//vid is chosen randomly from the size of the m_vertices list (its the index)
	//run the ExtendTree function with the new STO and Vertices
	//check if youre within the goal region ti end it and print the solved time
    
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code
	//sto is chosen the same way as above 
	//vid is chosen based on the closest vertex  so we'll have to run through the vertices and find the distance 
	//and keep track of the largest one as we go. potential to optimize by using a good sort but meh
	//check done as above
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);

//your code    
	//sto is chosen as above
	//vid chosen based on with probability proportional to how many children are coming out of it. 
	//we know how much each vertex has (m_nchildren) but im unsure how to make it chose based on that since its a probability distribution 
	//so someone else is gonna have think of something
	//check done as above
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code
    //sto chosen as above
	//vid chosen from the sorting the vertex list by how close it is (we should build a helper function) and taking the
	//5 (or some other number) closest ones and choseing one of those randomly (or we can do it based on how many children it has)
    //check done as above
	m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::AddVertex(Vertex * const v)
{
    if(v->m_type == Vertex::TYPE_GOAL)
	m_vidAtGoal = m_vertices.size();
    m_vertices.push_back(v); 
    if(v->m_parent >= 0)
	(++m_vertices[v->m_parent]->m_nchildren);
}

void MotionPlanner::GetPathFromInitToGoal(std::vector<int> *path) const
{
    std::vector<int> rpath;
    
    rpath.clear();
    
    int i = m_vidAtGoal;
    do
    {
	rpath.push_back(i);
	i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
    
    path->clear();
    for(int i = rpath.size() - 1; i >= 0; --i)
	path->push_back(rpath[i]);
}
