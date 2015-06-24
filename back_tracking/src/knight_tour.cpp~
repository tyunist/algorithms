/* This program implements Floyd Warshall algorithm
	Last edited at 11:21 PM 2 September, 2014 by tynguyen
	** Print shortest paths **
 */
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#ifndef V
#define V 4
#endif
#define INF 99999

// Function to print the solution matrix
void printSolution(int dist[][V]);
/* Print path*/
void printPath(int path[][V], int u, int v);

// Solves the all-pairs shortest path problem using Floyd Warshall algorithm
void floydWarshell(int graph[][V])
{
	int dist[V][V], i, j, k;
	int path[V][V]; /* Store parents of veties */
	for(i = 0; i < V; i++)
		for(j = 0; j < V; j ++)
		{
			if(graph[i][j] == 0) path[i][i] = i;
			else if(graph[i][j] == INF) path[i][j] = INF;
			else path[i][j] = j;
		}
	
	cout<<"Original Parent matrix:"<<endl;
	for(int i = 0; i < V; i++)
	{
		for(int j = 0; j < V; j++)
		{
			if(path[i][j] == INF)
				printf("%7s", "INF");
			else
				printf("%7d", path[i][j]);
		}
	    printf("\n");
	}
	for(i = 0; i < V; i++)
		for(j = 0; j <V; j++)
			dist[i][j] = graph[i][j];
    // Find shortest path for all vertices
	for(k = 0; k < V; k++)
		for(i = 0; i < V; i++)
			for(j = 0; j < V; j++)
				if(dist[i][k] + dist[k][j] < dist[i][j])
					{
						dist[i][j] = dist[i][k] + dist[k][j];
						path[i][j] = k;
					}	
	printSolution(dist);
	cout<<"Parent matrix:"<<endl;
	for(int i = 0; i < V; i++)
	{
		for(int j = 0; j < V; j++)
		{
			if(path[i][j] == INF)
				printf("%7s", "INF");
			else
				printf("%7d", path[i][j]);
		}
	    printf("\n");
	}
	cout<<"----------------"<<endl;
	cout<<"Paths are given as follow:"<<endl;
	for(i = 0; i < V; i ++)
		{
			cout<<"PATH FROM "<<i<<":	"<<endl;
			for(j = 0; j < V; j++ )
				{
					cout<<i<<" - "<<j<< ":		"<<i <<"	";
					if(dist[i][j] == INF)
					{
						cout<<"NO PATH"<<endl;
						continue;
					}
					printPath(path, i, j);
					cout<<endl;
				}
			cout<<"\n"<<endl;
		}			
}
					
void printSolution(int dist[][V])
{
	printf("Following is the matrix that shows the shortest  distances \n");
	for(int i = 0; i < V; i++)
	{
		for(int j = 0; j < V; j++)
		{
			if(dist[i][j] == INF)
				printf("%15s", "INF");
			else
				printf("%15d", dist[i][j]);
		}
	    printf("\n");
	}
}

/* Print path*/
void printPath(int path[][V], int u, int v)
{
	if(path[u][v] == v) 
	{
		printf("%d\t", v);
		return;
	}
	
	printPath(path, u, path[u][v]);
	printPath(path, path[u][v], v);
}

int main()
{
	/* Let us create the following weighted graph
	 *            10
	 *      (0)------->(3)
	 *       |         /|\
	 *     5 |          |
	 *       |          | 1
	 *      \|/         |
	 *      (1)------->(2)
	 *            3
	 */
	
	int graph[V][V] = { {0,   4,  INF, 3},
			             {INF, 0,   1, 6},
	                     {5, INF, 0,   1}, 
						 {INF, INF, INF, 0}
	                   };
    // Print the solution
	floydWarshell(graph);
	return 0;
}
