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
  double smallMovement = m_simulator->GetDistOneStep()/10;

  // TODO: x and y should be vertex points
  double X = m_simulator->GetRobotCenterX();
  double Y = m_simulator->GetRobotCenterY();
  bool hitObstacle = false;

  double testState[2] = {X,Y};
  while (!hitObstacle) {
    if(!reachedDest(testState,sto)){
      //moves along the line in small steps
      testState[0]+=smallMovement;
      testState[1]+=smallMovement;
      //sets rotbot state to test
      m_simulator->SetRobotState(testState);
      if(m_simulator->IsValidState()){
        //add up to this point of the line
      }
      else hitObstacle = true;
    }
    else break; //reached the point without hitting an Obstacle


  }


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
  double sto[2];
  m_simulator->SampleState(sto); 
   srand (time(NULL));
   int vid = rand() % m_vertices.size() + 1;
   ExtendTree(vid,sto);

   if(m_simulator->HasRobotReachedGoal()){
     printf("FINISHED IN: %lf\n", m_totalSolveTime);
   }
   else m_totalSolveTime += ElapsedTime(&clk);
	//sto is chosen based on some probbability from the goal region using samplestate
	//vid is chosen randomly from the size of the m_vertices list (its the index)
	//run the ExtendTree function with the new STO and Vertices
	//check if youre within the goal region ti end it and print the solved time


}

void MotionPlanner::ExtendRRT(void)
{
	Clock clk;
	StartTime(&clk);

	//your code
		//sto is chosen the same way as above
	double sto[2];
	m_simulator->SampleState(sto);
	//vid is chosen based on the closest vertex  so we'll have to run through the vertices and find the distance
	int vid = 0;
	int i;
	for (i = 1; i < m_vertices.size(); i++) {
		if (distFromGoal(i - 1, sto) > distFromGoal(i, sto)) {
			vid = i;
		}
	}
	ExtendTree(vid, sto);

	if (m_simulator->HasRobotReachedGoal()) {
		printf("FINISHED IN: %lf\n", m_totalSolveTime);
	}

	//and keep track of the largest one as we go. potential to optimize by using a good sort but meh
	//check done as above
	else {
		m_totalSolveTime += ElapsedTime(&clk);

	}

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
	double sto[2];
	m_simulator->SampleState(sto);
	int sorted[m_vertices.size()];
	int i,j;
	for (i = 0; i < m_vertices.size(); i++) {
		sorted[i] = i;
	}
	//i need a better sort................. currently does bubble didnt want to have to implement mergesort
	for (i = 0; i < m_vertices.size(); i++) {
		for (j = 0; j < m_vertices.size(); j++) {
			//sort in some way based on distance from goal.
			if (distFromGoal(sorted[j], sto) > distFromGoal(sorted[j+1], sto)) {
				int temp = sorted[j];
				sorted[j] = sorted[j + 1];
				sorted[j + 1] = temp;
			}
		}
	}
	if (m_vertices.size() < 4) {
		//choose random number between 0 and size od m_vertices
		int vid = rand() % m_vertices.size();
	}
	else {
		//choose random number between 0 and 3 ;;;;;;;;;;;;;;;;maybe put sort in here.
		int vid = sorted[rand() % 4];
	}
	ExtendTree(vid, sto);
	if (m_simulator->HasRobotReachedGoal()) {
		printf("FINISHED IN: %lf\n", m_totalSolveTime);
	}

	//check done as above
	else {
		m_totalSolveTime += ElapsedTime(&clk);

	}

}

bool MotionPlanner::reachedDest(const double currentPos[],const double dest[]){
  double threshold = 0.03;
  double currentX = currentPos[0];
  double currentY = currentPos[1];
  double destX = dest[0];
  double destY = dest[1];

  return ((currentX>destX-threshold&&currentX<destX+threshold) &&
          (currentY>destY-threshold&&currentY<destY+threshold));
}

void MotionPlanner::AddVertex(Vertex * const v)
{
    if(v->m_type == Vertex::TYPE_GOAL)
	m_vidAtGoal = m_vertices.size();
    m_vertices.push_back(v);
    if(v->m_parent >= 0)
	(++m_vertices[v->m_parent]->m_nchildren);
}

double MotionPlanner::distFromGoal(int vid, double sto[]) 
{
	double x = sto[0] - m_vertices[vid].m_state[0];
	double y = sto[1] - m_vertices[vid].m_state[1];
	return sqrt(x*x + y*y);
	
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
