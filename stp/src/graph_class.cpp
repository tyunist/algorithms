/*
 * main.cpp
 *
 *  Created on: May 19, 2014
 *      Author: tynguyen
 */


#include <iostream>
#include "string.h"
#include "fstream"
#include <cmath> // pow()
#include <vector>
#include "graph_class.h"

using namespace std;
#define MAXV    1000

GRAPH::GRAPH(int n, bool isDirected)
{
	initializeGraph(n, isDirected);
}

void GRAPH::initializeGraph(int n, bool isDirected)
{
	cout<<"Gan gia tri n, is"<<endl;
	this->n = n;
	this->isDirected = isDirected;
	int i;
	cout<<"Gan gia tri default"<<endl;
	g.edgeNumber = 0;
	g.verticeNumber = 0;
	g.isDirected = this->isDirected;

	for(i=1; i<=MAXV; i++) g.degree[i] = 0;
	for(i=1; i<=MAXV; i++) g.edges[i] = NULL;
}


///Create graph from a text file that describes a matrix
void GRAPH:: readGraph(string fileName)
{
	ifstream in;
	int  temp = 0, x = 0, y = 0, count = 0, weight = 0;
	in.open(fileName.c_str());
	if(in.fail()) cout<< "Can not open the file. \n";
	in>>g.verticeNumber;
	for(int i = 0; i < n; i++)
		for(int j = 0; j < n; j++)
		{
			if(in.fail()) break;
			in >> weight;
			if(weight == 0 && i != j)  continue; // No connection
			x = j;
			y = i;

		cout<<"x, y inserted: "<< x<<"-"<<y<<"- weight: "<< weight <<endl;
		insertEdge(x, y, weight, this->isDirected);
		}
	in.clear();
}

///Insert edges
void GRAPH:: insertEdge(int x, int y, int weight, bool isDirected)
{	
	edgenode *p = new edgenode;
	p->weight = weight;
	p->y = y;
	p->next = g.edges[x];
	g.edges[x] = p;//Insert at the head of list
	g.degree[x] ++;

	g.edgeNumber ++;
}

void GRAPH::printGraph()
{
 int i;
 edgenode *p;
 cout<<"Edge    Weight"<<endl;
 for(i= 0; i< g.verticeNumber; i++)
 {
	 p = g.edges[i];
	 while(p!=NULL)
	 {
		 cout<<i<<" - "<<p->y <<": "<< p->weight<<endl;;
		 p = p->next;
	 }
	 cout<<"\n";
 }
}

void GRAPH::getGraph(graph &g)
{
	g = this->g;
}


