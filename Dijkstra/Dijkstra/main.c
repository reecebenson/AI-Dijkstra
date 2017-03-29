/*
 * File:   main.c
 * Author: j4-smith
 *  Dijksatra path finding example in C
 * First Created on 5 November 2014, 18:42
 * Modified in light of student progress
 */
#include <stdio.h>
#include <stdlib.h>
#include "Library/PathFindingSpecificDefinitions.h"
#include "Library/StructureDefinitions.h"
#include "Library/SolutionListOperations.h"
#include "Library/PathFindingSpecificSolutionOperations.h"
#include "Library/worlds.h"

candidateSolution workingCandidate;		// Hold the solution we are considering
candidateList  currentListOfCandidates; // Store solutions we have created but not examined
candidateList listOfExaminedCandidates; // Store solutions we have examined
char currentMap[MAXX][MAXY];			// This copies the map we are currently examining

// MAXX, MINX, MAXY, MINY		=> Dimensions of the world
// STARTX, STARTY, ENDX, ENDY	=> The starting and ending positions

/**
 * This is the only one which is different to before, and I have written it to save you time
 * It updates the distance estimates of unvisited neighbours of the current cell
 * void UpdateDistancesOfUnvisitedNeighboursOfWorkingCandidateNeighbours(void);
 *
 * These final few are functions for manipulating maps that I am giving you to let you focus on the logic
 * void ReadComandLineAndSetMap(int argc, const char * argv[]);
 * void CopyMap(const char from[MAXX][MAXY], char to[MAXX][MAXY]);
 * void PrintMap(char map[MAXX][MAXY]);
 * int IsSolutionAtCoordinates(candidateSolution thisSol, int x, int y); // Checks if a candidate soltuion is at a given co-ordinate
 *
 *  Finally this macro tells you the distance between two points with x,y co-ordinates (a,b) and (c,d)
 *  #define SQEUCLIDEAN_DISTANCE(a,b,c,d)   ( (a-c)*(a-c) + (b-d)*(b-d))   <======================== NEW
**/

int main(int argc, const char * argv[])
{
	// > Variables
	int indexOfSolutionWeAreLookingAt;		// Index in list of current solution being examined
    int closestSoFar;						// Used to find which to move next
    int closestEstimatedDistance;			// Used to find which to move next
    int x, y;								// X,Y Coordinates
	int hasReachedGoal = 0;					// Has Finished

    // > Start off by emptying the lists of candidate solutions
    CleanListsOfSolutionsToStart();
    CleanWorkingCandidate();
    
	// > Read command line and set the map
    ReadComandLineAndSetMap(argc,argv);
    
    /**
	 * This time we put all the unvisited cells that aren't obstacles into the open list to start.
     * We could manage the list on-the-fly which might be faster if we have big map
     * To start off with everywhere except the start point has a huge estimated distance
	**/
	for (x = MINX; x < MAXX; x++)
	{
		for (y = MINY; y < MAXY; y++)
		{
			if (currentMap[x][y] != (char)OBSTACLE)
			{
				// > Build new node with these values
				workingCandidate.variableValues[XCOORD] = workingCandidate.variableValues[PARENTXCOORD] = x;
				workingCandidate.variableValues[YCOORD] = workingCandidate.variableValues[PARENTYCOORD] = y;
				workingCandidate.numberOfDefinedValues = 4;
				if (x == STARTX && y == STARTY)
					workingCandidate.score = 0;
				else
					workingCandidate.score = BIGDIST;

				// > Add it to the open list
				AddWorkingCandidateToCurrentList();

				// > Mark this cell on the map as open for graphical display
				currentMap[x][y] = OPEN;
			}
		}
	}
    
    // > Set the working candidate to the start position
    workingCandidate.variableValues[XCOORD] = workingCandidate.variableValues[PARENTXCOORD] = STARTX;
    workingCandidate.variableValues[YCOORD] = workingCandidate.variableValues[PARENTYCOORD] =  STARTY;
    workingCandidate.score = 0;
    PrintWorkingCandidate();
    
    // > Now we will go into a while loop examining solutions until we get to the goal
	while (hasReachedGoal == 0)
	{
		// > Get the WC [Closest to the start]
		indexOfSolutionWeAreLookingAt = 0;
		closestSoFar = -1;

		// > Add neighbours on grid to the OPEN list
		int neighbourIndex;
		for (neighbourIndex = 1; neighbourIndex < currentListOfCandidates.indexOfLastEntryAdded; neighbourIndex++)
		{
			CopySolutionFromCurrentListIntoWorkingCandidate(neighbourIndex);
			x = workingCandidate.variableValues[XCOORD];
			y = workingCandidate.variableValues[YCOORD];
			closestEstimatedDistance = workingCandidate.score;

			// > PSEUDO <
			// FOREACH neighour IN open_list
			//     IF (WC.DISTANCE + DISTANCE(WC_NEIGHBOUR) < NEIGHBOUR_DISTANCE) THEN
			if (closestEstimatedDistance < closestSoFar || closestSoFar == -1)
			{
				indexOfSolutionWeAreLookingAt = neighbourIndex;
				closestSoFar = closestEstimatedDistance;
			}
		}

		// > Copy Solutions
		CopySolutionFromCurrentListIntoWorkingCandidate(indexOfSolutionWeAreLookingAt);
		AddWorkingCandidateToExaminedList();
		RemoveSolutionFromCurrentList(indexOfSolutionWeAreLookingAt);

		// > Update neighbour.dist and neighbour.parent
		UpdateDistancesOfUnvisitedNeighboursOfWorkingCandidate();

		// > Check that the WC is at the goal
		hasReachedGoal = IsSolutionAtCoordinates(workingCandidate, ENDX, ENDY);
	}
	RemoveFromListParam1_CandidateSolutionAtIndexParam2(&listOfExaminedCandidates, (listOfExaminedCandidates.indexOfLastEntryAdded - 1));
    // > End of while loop dealing with search
    
    PrintFinalSolutionAndExit();
    return 0;
}



