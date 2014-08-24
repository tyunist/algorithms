// A C / C++ program for Dijkstra's single source shortest path algorithm.
// The program is for linkedlist  representation of the graph
 
#include <stdio.h>
#include <limits.h>
#include <iostream>
#include <vector>
#include "graph_class.h"
#include <fstream>

using namespace std;
// Number of vertices in the graph
#define V 9
 
// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[], bool sptSet[])
{
   // Initialize min value
   int min = INT_MAX, min_index;
 
   for (int v = 0; v < V; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;
 
   return min_index;
}
 
// A utility function to print the constructed distance array
void printSolution(int dist[], int n)
{
   printf("Vertex   Distance from Source\n");
   for (int i = 0; i < V; i++)
      printf("%d \t\t %d\n", i, dist[i]);
}
 
void printPath(int (&path)[V], int i)
{
	printf("\t %d", i);
	if(i == 0) return;
	else 
		printPath(path, path[i]);
}

int findWeight(GRAPH graph, int x, int y)  // find weight of edge x-y
{
	struct edgenode *p = graph.g.edges[x];
	while(p!=NULL)
	{
		if(p->y == y) return p->weight;
		p = p->next;
	}
	printf("Error with finding weight of edge %d - %d", x, y);
	return 0;
}

// Function that checks whether an edge exists
bool hasEdge(GRAPH graph, int x, int y)
{
	cout<<"Checking edge "<<x<<"-"<<y<<endl;
	struct edgenode *p = graph.g.edges[x];
	while(p!=NULL)
	{
		if(p->y == y) return true;
		p = p->next;
	}
	cout<<"No edge"<<endl;
	return false;

}

// Funtion that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void dijkstra(GRAPH graph, int src)
{
     int dist[V];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i
 
     bool sptSet[V]; // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized
	  int path[V];
	  path[0] = 0;
	  // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = false;
 
     // Distance of source vertex from itself is always 0
     dist[src] = 0;
 
     // Find shortest path for all vertices
     for (int count = 0; count < V-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = minDistance(dist, sptSet);
 
       // Mark the picked vertex as processed
       sptSet[u] = true;
       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < V; v++)
 
         // Update dist[v] only if is not in sptSet, there is an edge from
         // u to v, and total weight of path from src to  v through u is
         // smaller than current value of dist[v]
		   if (!sptSet[v] && hasEdge(graph, u, v) && dist[u] != INT_MAX
                   && dist[u]+ findWeight(graph, u, v) < dist[v])
			{
				dist[v] = dist[u] + findWeight(graph, u, v);
				cout<<"line 93"<<endl;
     			path[v] = u;
			}
    	 // print the constructed distance array
     	printSolution(dist, V); 
	}
	  printf("Paths are: ");
	  for(int i = 0; i < V; i++)
	  {
		  printf("\nPath from 0 to %d : \t", i);
		  printPath(path, i);
	  }
	  printf("\n");

}
 
// driver program to test above function
int main()
{
   /* Let us create the example graph discussed above */
	GRAPH gh(9, 0);
	gh.readGraph("graph.txt");
	cout <<"Input graph is: "<<endl;
	gh.printGraph();
	cout <<"Result: "<<endl;

    dijkstra(gh, 0);
 
   return 0;
}
