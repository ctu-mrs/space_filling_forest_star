#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <vector>

#include "../ann/ann/include/ANN/ANN.h"

using namespace std;

int main(int argc, char **argv) {

    const int dimension = 2;

    const int numPoints = 1000;
	const int k = 1;
	ANNidxArray idx = new ANNidx[k];
	ANNdistArray dist = new ANNdist[k];
	ANNpoint query = annAllocPt(dimension);	
	ANNpointArray ap = annAllocPts(numPoints,dimension);

	for(int i=0;i<numPoints;i++) {
        for(int j=0;j<dimension;j++) {
            ap[i][j] = rand() % 1000 - rand()%1000;
        }
	}

	ANNkd_tree *kdTree = new ANNkd_tree(ap,numPoints,dimension);


    ofstream ofs("nearest.dat");

    for(int i=0;i<numPoints;i++) {
        for(int j=0;j<dimension;j++) {
            query[j] = rand()%1000 - rand()%1000;
        }
		kdTree->annkSearch(query,k,idx,dist,0);

        for(int j=0;j<dimension;j++) {
            ofs << query[j] << " ";
        }
        ofs <<"\n";
        for(int j=0;j<dimension;j++) {
            ofs << ap[idx[0]][j] << " ";
        }
        ofs << "\n\n";
    }
    ofs.close();
	delete [] idx;
	delete [] dist;
	delete kdTree;
	annDeallocPt(query);
	annDeallocPts(ap);
	annClose();



}


