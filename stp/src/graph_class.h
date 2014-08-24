/*
 * graph_class.h
 *
 *  Created on: 11pm_August, 23, 2014
 *      Author: tynguyen
 */

#include <iostream>
#include "string.h"
#include "fstream"
#include <cmath> // pow()
#include <vector>

using namespace std;
#ifndef MAXV
#define MAXV    1000
#endif


#ifndef GRAPH_CLASS_H
#define GRAPH_CLASS_H

typedef struct edgenode {
	int y; //Adjacency infor
	int weight;
	struct edgenode *next; //Point to the next edge in list
};

typedef struct graph{
	struct edgenode *edges[MAXV + 1]; //Adjacency infor
	int degree[MAXV + 1]; //Outdegree of each vertex, increase each time of exploring new adjacent vertex
	int verticeNumber; //No of vertices in graph
	int edgeNumber; //no of edges
	bool isDirected;
};

class GRAPH{
	private: bool isDirected;
	private: int n;
	public: graph g;
	private: GRAPH() {}; //default constructor
	public: ~GRAPH() {}; //default destructor
	public:

		GRAPH(int , bool );
		void initializeGraph(int, bool);

	///Create graph from a text file that describes a matrix
	public: void readGraph(string);

	///Insert edges
	public: void insertEdge(int, int, int, bool );

	public: void printGraph();

	///Get graph g for searching functions
	public: void getGraph(graph &g);

};

#endif



