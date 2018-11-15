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


void MotionPlanner::ExtendTree(const int vid,
                               const double sto[])
{
        //uses given step distance
        double smallMovement = m_simulator->GetDistOneStep();
        //gets x and y of the vertex from vid
        double X = m_vertices[vid]->m_state[0];
        double Y = m_vertices[vid]->m_state[1];

        bool hitObstacle = false;

        //gets vector is direction to goal
        double X_to_sto = sto[0] - X;
        double Y_to_sto = sto[1] - Y;
        double mag = distFromGoal(vid, sto);
        double testVector[2] = { X_to_sto / mag, Y_to_sto / mag };
        double testState[2] = {X,Y};

        //sets first state of robot
        m_simulator->SetRobotState(testState);
        int counter = 0;

        //continues until an Obstacle is hit
        while (!hitObstacle) {

                //continues to add vector segments if it hasnt reached the goal or its destination
                if(!reachedDest(testState,sto) && !m_simulator->HasRobotReachedGoal()) {
                        //moves along the line in small steps
                        testState[0]+=(testVector[0]*smallMovement);
                        testState[1]+=(testVector[1]*smallMovement);
                        counter++;

                        //sets rotbot state to test
                        m_simulator->SetRobotState(testState);

                        //checks if testState is valid
                        if(!m_simulator->IsValidState()) {
                                //adds vector up to the Obstacle
                                if (counter != 1) {
                                        Vertex *newVertex = new Vertex();
                                        newVertex->m_parent = vid;
                                        newVertex->m_nchildren = 0;
                                        newVertex->m_state[0] = testState[0] - (testVector[0] * smallMovement);
                                        newVertex->m_state[1] = testState[1] - (testVector[1] * smallMovement);

                                        AddVertex(newVertex);
                                }
                                break;

                        }//is not valid state
                }
                else {
                        //the vector has reached its destination or goal at this point
                        Vertex *newVertex = new Vertex();
                        //sets type if the goal was found
                        if(m_simulator->HasRobotReachedGoal()) {
                                newVertex->m_type = Vertex::TYPE_GOAL;
                        }
                        newVertex->m_parent = vid;
                        newVertex->m_nchildren = 0;
                        newVertex->m_state[0] = testState[0];
                        newVertex->m_state[1] = testState[1];
                        AddVertex(newVertex);
                        break;
                }//reached the point without hitting an Obstacle
        }
}

void MotionPlanner::ExtendRandom(void)
{
        Clock clk;
        StartTime(&clk);

        //gets sample state
        double sto[2];
        m_simulator->SampleState(sto);
        srand (time(NULL)); //seeds rand
        int vid = rand() % m_vertices.size(); //gets random vertex
        ExtendTree(vid,sto); // extends tree from the random vid to sto

        m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
        Clock clk;
        StartTime(&clk);

        //sto is chosen the same way as above
        double sto[2];
        m_simulator->SampleState(sto);
        //vid is chosen based on the closest vertex
        int vid = 0;
        int i;
        for (i = 1; i < m_vertices.size(); i++) {
                if (distFromGoal(i - 1, sto) > distFromGoal(i, sto)) {
                        vid = i;
                }
        }
        //extends tree from the vid to sto
        ExtendTree(vid, sto);

        m_totalSolveTime += ElapsedTime(&clk);

}

void MotionPlanner::ExtendEST(void)
{
        Clock clk;
        StartTime(&clk);

        // get sample state for sto
        double sto[2];
        m_simulator->SampleState(sto);

        //gets vid based on weight function below
        int vid = decideVid();
        ExtendTree(vid, sto); //extends tree from weighted vid to sto

        m_totalSolveTime += ElapsedTime(&clk);
}

int MotionPlanner::decideVid()
{
        //Calculate the weight with the vertices
        double weight = 0;
        for (int cntr = 0; cntr < m_vertices.size(); cntr++) {
                //add up the weight
                weight += calc(cntr);
        }

        // decide vertex from weight
        double finalWeight = PseudoRandomUniformReal(0,weight);

        //find vid with vertices
        int finalVid = 0;
        weight = 0;
        for (int cntr2 = 0; cntr2 < m_vertices.size(); cntr2++) {
                weight += calc(cntr2);
                if (weight >= finalWeight) {
                        finalVid = cntr2;
                        break;
                }
        }

        return finalVid;
}

void MotionPlanner::ExtendMyApproach(void)
{
        Clock clk;
        StartTime(&clk);
        double sto[2];
        m_simulator->SampleState(sto);
        int vid = 0;
        //choose random number between 0 and size od m_vertices
        if (m_vertices.size() < 5) vid = rand() % m_vertices.size();
        //choose random number between 0 and 3
        else vid = rand() % 5 + (m_vertices.size() - 5);
        ExtendTree(vid, sto);

        m_totalSolveTime += ElapsedTime(&clk);

}

//helper to check id the vectors are within their target destination
bool MotionPlanner::reachedDest(const double currentPos[],const double dest[]){
        double threshold = 0.03;
        double currentX = currentPos[0];
        double currentY = currentPos[1];
        double destX = dest[0];
        double destY = dest[1];

        return ((currentX>destX-threshold&&currentX<destX+threshold) &&
                (currentY>destY-threshold&&currentY<destY+threshold));
}
//helper for finding distance to goal
double MotionPlanner::distFromGoal(int vid, const double sto[])
{
        double x = sto[0] - m_vertices[vid]->m_state[0];
        double y = sto[1] - m_vertices[vid]->m_state[1];
        return sqrt(x*x + y*y);

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
