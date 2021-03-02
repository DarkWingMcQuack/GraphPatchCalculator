#ifndef sgraphH
#define sgraphH

// READING IN FMI-CH-FORMAT

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <cmath>
#include <queue>
#include <cstdlib>
#include <tuple>
#include <utility>
#include <vector>

#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>

using namespace std;


// single edge list with source, target, weight only
// in and out lists are IDs only
// try to keep additional information about Edge in extra array



typedef int NodeID;
typedef int EdgeID;

typedef float CoordType;
typedef std::int64_t EdgeCost;

const EdgeCost MAX_EDGE_COST = numeric_limits<EdgeCost>::max();

const NodeID NO_NODE_ID = -1;
const EdgeID NO_EDGE_ID = -1;



class NodeType // not really interesting information
{
public:
    CoordType lat, lon;
    long fmiID;
    long osmID;
    int elev;
    int level;
    //              string carry;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &lat;
        ar &lon;
        ar &fmiID;
        ar &osmID;
        ar &elev;
        ar &level;
    }
};


class EdgeType
{
public:
    EdgeType()
    {
    }
    EdgeType(NodeID _src, NodeID _trg, EdgeCost _wght)
    {
        source = _src;
        target = _trg;
        weight = _wght;
    }
    NodeID source, target;
    EdgeCost weight;

    bool operator<(const EdgeType &itm) const
    {
        if(source < itm.source)
            return true;
        if((source == itm.source) && (target < itm.target))
            return true;
        if((source == itm.source) && (target == itm.target) && (weight < itm.weight))
            return true;
        return false;
    }
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &source;
        ar &target;
        ar &weight;
    }
};

class EdgeExtType
{
public:
    int type;
    int speed;
    EdgeID shortA;
    EdgeID shortB;
    float error;
    // probably also bounding box etc
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &type;
        ar &speed;
        ar &shortA;
        ar &shortB;
        ar &error;
    }
};




class FMIGraph
{
public:
    vector<NodeType> nodeList;

    vector<EdgeType> edgeList;
    vector<EdgeExtType> edgeExtList;

    vector<int> edgeListOut;
    vector<int> edgeOffsetOut;

    vector<int> edgeListIn;
    vector<int> edgeOffsetIn;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &nodeList;
        ar &edgeList;
        ar &edgeExtList;
        ar &edgeListOut;
        ar &edgeOffsetOut;
        ar &edgeListIn;
        ar &edgeOffsetIn;
    }

public:
    int nofNodes() const
    {
        return nodeList.size();
    }
    int nofEdges() const
    {
        return edgeListOut.size();
    }

    void readFromFMIFile(string fname)
    {
        nodeList.clear();

        edgeList.clear();
        edgeExtList.clear();

        edgeListOut.clear();
        edgeOffsetOut.clear();

        edgeListIn.clear();
        edgeOffsetIn.clear();

        ifstream inputFile(fname.c_str());
        char junkC[1024];
        int junkI;
        for(int i = 0; i < 9; i++)
            inputFile.getline(junkC, 1024);

        int nofNodes, nofEdges;
        inputFile >> nofNodes;
        inputFile >> nofEdges;

        cout << "We will read " << nofNodes << " nodes and " << nofEdges << " edges" << endl;

        cout << "Resident memory req should be: " << (nofNodes * (sizeof(NodeType)) + // node stuff
                                                      nofEdges * (sizeof(EdgeType) + sizeof(EdgeExtType)) + // 2 EdgeLists
                                                      nofNodes * 2 * sizeof(int) + // 2 offset arrays
                                                      nofEdges * 2 * sizeof(int) // edge lists
                                                      )
                / (1024 * 1024)
             << "MB" << endl;
        cout << "NodeType: " << sizeof(NodeType) << endl;
        cout << "EdgeType: " << sizeof(EdgeType) << endl;
        cout << "EdgeExtType: " << sizeof(EdgeExtType) << endl;
        cout << "long     : " << sizeof(long) << endl;
        cout << "int      : " << sizeof(int) << endl;
        NodeType curNode;

        nodeList.resize(nofNodes);

        cout << endl
             << "NODES: ";
        for(int i = 0; i < nofNodes; i++) {
            inputFile >> curNode.fmiID;
            inputFile >> curNode.osmID;
            inputFile >> curNode.lat;
            inputFile >> curNode.lon;
            inputFile >> curNode.elev;
            inputFile >> curNode.level;

            // cout<<"Read: "<<curNodeExt.fmiID<<"\n";
            if((i) % (nofNodes / 10) == 1)
                cout << int((i * 100.0) / nofNodes) << "% " << flush;
            inputFile.getline(junkC, 256);
            //                              curNode.carry=string(junkC);
            nodeList[i] = curNode;
        }
        edgeOffsetOut.resize(nofNodes + 1);
        edgeOffsetOut[0] = 0;

        vector<int> nofIncoming(nofNodes, 0);

        int maxOutDegree = 0;
        int maxInDegree = 0;
        int lastSrcID = -1;
        int countZeroWeight = 0;
        int countShortcuts = 0;
        EdgeType curEdge;
        EdgeExtType curExtEdge;
        cout << endl
             << "EDGES: ";
        for(int j = 0; j < nofEdges; j++) {
            NodeID srcID;
            inputFile >> srcID;
            curEdge.source = srcID;
            inputFile >> curEdge.target;
            inputFile >> curEdge.weight;
            if(curEdge.weight == 0) {
                countZeroWeight++;
            }
            inputFile >> curExtEdge.type;
            inputFile >> curExtEdge.speed;
            inputFile >> curExtEdge.shortA;
            inputFile >> curExtEdge.shortB;
            if(curExtEdge.shortA != -1)
                countShortcuts++;
            if((curExtEdge.shortA == -1) || (curExtEdge.shortB == -1))
                assert(curExtEdge.shortA == curExtEdge.shortB);
            inputFile.getline(junkC, 256);
            if((j) % (nofEdges / 10) == 1)
                cout << int((j * 100.0) / nofEdges) << "% " << flush;

            assert(srcID >= lastSrcID);
            lastSrcID = srcID;
            edgeList.push_back(curEdge);
            edgeExtList.push_back(curExtEdge);
            edgeListOut.push_back(j);
            edgeOffsetOut[srcID + 1] = j + 1;
            nofIncoming[curEdge.target]++;
        }
        edgeOffsetOut[nodeList.size()] = edgeListOut.size();

        // SMALL CHECK WHETHER SHORTCUTS ARE CONSISTENT
        for(int j = 0; j < nofEdges; j++) {
            NodeID src = edgeList[j].source;
            NodeID trg = edgeList[j].target;

            if(nodeList[src].level == nodeList[trg].level)
                cout << endl
                     << src << " " << trg << " " << nodeList[src].level << " " << nodeList[trg].level << endl;
            assert((nodeList[src].level != nodeList[trg].level));
            EdgeID shortA = edgeExtList[j].shortA;
            EdgeID shortB = edgeExtList[j].shortB;
            if((shortA != -1) || (shortB != -1)) {
                assert(src == edgeList[shortA].source);
                assert(edgeList[shortA].target == edgeList[shortB].source);
                assert(edgeList[shortB].target == trg);
            }
        }



        // if node v has no outgoing edges, edgeOffsetOut[v+1] is
        // never properly set
        // we need to set edgeOffsetOut[v+1] to edgeOffsetOut[v'] where
        // v'<v+1 and v'-1 had outgoing edges
        for(int i = 0; i < nofNodes; i++)
            if(edgeOffsetOut[i + 1] == 0)
                edgeOffsetOut[i + 1] = edgeOffsetOut[i];

        // we finished construction of outEdge lists and offsets


        // now offsets and stuff for incoming edge list
        edgeOffsetIn.resize(nofNodes + 1, 0);
        edgeListIn.resize(nofEdges);
        edgeOffsetIn[0] = 0;
        for(int i = 0; i < nofNodes; i++)
            edgeOffsetIn[i + 1] = edgeOffsetIn[i] + nofIncoming[i];
        assert(edgeOffsetIn[nofNodes] == nofEdges);


        // now store incoming edges; need to iterate over nodes to get source
        for(int i = 0; i < nofNodes; i++) {
            for(int j = edgeOffsetOut[i]; j < edgeOffsetOut[i + 1]; j++) {
                curEdge = edgeList[edgeListOut[j]];
                assert(curEdge.source == i);
                NodeID src = curEdge.source;
                NodeID trg = curEdge.target;

                assert(nofIncoming[trg] > 0);

                edgeListIn[edgeOffsetIn[trg + 1] - nofIncoming[trg]] = edgeListOut[j];
                nofIncoming[trg] = nofIncoming[trg] - 1;
            }
        }

        inputFile.close();

        for(int j = edgeOffsetOut[154511]; j < edgeOffsetOut[154512]; j++) {
            NodeID trgNode = edgeList[edgeListOut[j]].target;
            NodeID srcNode = edgeList[edgeListOut[j]].source;
            EdgeCost cost = edgeList[edgeListOut[j]].weight;
            cout << "EDGE: " << srcNode << " " << trgNode << " " << cost << endl;
        }

        graphStats();
    }

    void createFromFMIGraph(const FMIGraph &oldGraph, const vector<bool> &alive, const vector<EdgeType> &newEdges)
    {
        cout << endl
             << "Creation from old graph" << endl;

        // takes old graph, surving nodes list and new edges list; creates new graph
        // keeps both, fmiID as well as osmID
        // cleanup
        nodeList.clear();

        edgeList.clear();
        edgeExtList.clear();

        edgeListOut.clear();
        edgeOffsetOut.clear();
        edgeListIn.clear();
        edgeOffsetIn.clear();

        int nofNodes = 0; // determine new number of nodes and fill node list

        vector<int> old2new;
        for(int i = 0; i < oldGraph.nofNodes(); i++)
            if(alive[i]) {
                old2new.push_back(nofNodes);
                nodeList.push_back(oldGraph.nodeList[i]);
                nofNodes++;
            } else
                old2new.push_back(-1);
        edgeOffsetOut.resize(nofNodes + 1);
        edgeOffsetOut[0] = 0;

        cout << "We have " << nofNodes << " surviving nodes" << endl;
        int nofEdges = 0; // determine new number of edges
        // first count and copy surviving edges
        vector<EdgeType> survivingEdges;

        // count and collect surviving edges
        for(int i = 0; i < oldGraph.nofNodes(); i++)
            for(int j = oldGraph.edgeOffsetOut[i]; j < oldGraph.edgeOffsetOut[i + 1]; j++) {
                EdgeType curEdge = oldGraph.edgeList[oldGraph.edgeListOut[j]];
                int source = curEdge.source, target = curEdge.target;
                assert(source == i);

                if(alive[source] && alive[target]) {
                    assert(old2new[source] != -1);
                    assert(old2new[target] != -1);
                    nofEdges++;
                    EdgeType curFullEdge(old2new[source], old2new[target], curEdge.weight);
                    survivingEdges.push_back(curFullEdge);
                }
            }
        cout << "We have " << nofEdges << " old edges and " << newEdges.size() << " new edges" << endl;


        // now add new edges and sort
        for(int j = 0; j < newEdges.size(); j++) {
            int src = old2new[newEdges[j].source], trg = old2new[newEdges[j].target];
            EdgeCost weight = newEdges[j].weight;
            assert(src != -1);
            assert(trg != -1);
            survivingEdges.push_back(EdgeType(src, trg, weight));
        }

        sort(survivingEdges.begin(), survivingEdges.end());

        cout << "Before unique we have: " << survivingEdges.size() << endl;
        vector<EdgeType> tmpList;
        tmpList.push_back(survivingEdges[0]);
        for(int j = 1; j < survivingEdges.size(); j++) {
            EdgeType curEdge = survivingEdges[j];
            EdgeType lastEdge = survivingEdges[j - 1];
            if((curEdge.source != lastEdge.source) || (curEdge.target != lastEdge.target))
                tmpList.push_back(curEdge);
        }
        survivingEdges = tmpList;

        cout << "After unique we have: " << survivingEdges.size() << endl;
        nofEdges = survivingEdges.size();

        edgeOffsetOut.resize(nofNodes + 1);
        edgeOffsetOut[0] = 0;
        // now deal with surviving edges as if they were read from file

        vector<int> nofIncoming(nofNodes, 0);
        int countZeroWeight = 0;
        int lastSrcID = -1;
        EdgeType curEdge;
        EdgeExtType curExtEdge;
        for(int j = 0; j < nofEdges; j++) {
            curEdge = survivingEdges[j];
            NodeID srcID = curEdge.source;
            if(curEdge.weight == 0) {
                countZeroWeight++;
            }
            assert(srcID >= lastSrcID);
            lastSrcID = srcID;
            edgeList.push_back(curEdge);
            edgeExtList.push_back(curExtEdge);
            edgeListOut.push_back(j);
            edgeOffsetOut[srcID + 1] = j + 1;
            nofIncoming[curEdge.target]++;
        }
        edgeOffsetOut[nodeList.size()] = edgeListOut.size();


        cout << endl
             << "We augmented " << countZeroWeight << " zero-weight edges" << endl;
        cout << "After first Offset creation" << endl;

        // if node v has no outgoing edges, edgeOffsetOut[v+1] is
        // never properly set
        // we need to set edgeOffsetOut[v+1] to edgeOffsetOut[v'] where
        // v'<v+1 and v'-1 had outgoing edges
        for(int i = 0; i < nofNodes; i++)
            if(edgeOffsetOut[i + 1] == 0)
                edgeOffsetOut[i + 1] = edgeOffsetOut[i];

        // we finished construction of outEdge lists and offsets


        // now offsets and stuff for incoming edge list
        edgeOffsetIn.resize(nofNodes + 1, 0);
        edgeListIn.resize(nofEdges);
        edgeOffsetIn[0] = 0;
        for(int i = 0; i < nofNodes; i++)
            edgeOffsetIn[i + 1] = edgeOffsetIn[i] + nofIncoming[i];
        assert(edgeOffsetIn[nofNodes] == nofEdges);


        // now store incoming edges; need to iterate over nodes to get source
        for(int i = 0; i < nofNodes; i++) {
            for(int j = edgeOffsetOut[i]; j < edgeOffsetOut[i + 1]; j++) {
                curEdge = edgeList[edgeListOut[j]];
                assert(curEdge.source == i);
                NodeID src = curEdge.source;
                NodeID trg = curEdge.target;

                assert(nofIncoming[trg] > 0);

                edgeListIn[edgeOffsetIn[trg + 1] - nofIncoming[trg]] = edgeListOut[j];
                nofIncoming[trg] = nofIncoming[trg] - 1;
            }
        }

        graphStats();
    }
    void graphStats()
    {

        double outDegSum = 0, inDegSum = 0;
        int maxOutDegree = 0, maxInDegree = 0;
        int maxLevel = 0;
        cout.precision(15);
        for(int i = 0; i < nofNodes(); i++) {
            //if (edgeOffsetOut[i+1]-edgeOffsetOut[i]>10)
            //      cout<<"Strange thing for node "<<i<<endl;
            maxOutDegree = max(maxOutDegree, edgeOffsetOut[i + 1] - edgeOffsetOut[i]);
            maxInDegree = max(maxInDegree, edgeOffsetIn[i + 1] - edgeOffsetIn[i]);
            outDegSum += edgeOffsetOut[i + 1] - edgeOffsetOut[i];
            inDegSum += edgeOffsetOut[i + 1] - edgeOffsetOut[i];
            if(maxLevel < nodeList[i].level)
                maxLevel = nodeList[i].level;
        }
        cout << "maxOutDegree: " << maxOutDegree << " avgOutDegree: " << outDegSum / nofNodes() << endl;
        cout << "maxInDegree: " << maxInDegree << " avgInDegree: " << inDegSum / nofNodes() << endl;
        cout << "maxLevel:" << maxLevel << endl;


        double edgeSumOut = 0, edgeSumIn = 0;
        for(int i = 0; i < nofNodes(); i++)
            for(int j = edgeOffsetOut[i]; j < edgeOffsetOut[i + 1]; j++)
                edgeSumOut += edgeList[edgeListOut[j]].weight;

        for(int i = 0; i < nofNodes(); i++)
            for(int j = edgeOffsetIn[i]; j < edgeOffsetIn[i + 1]; j++)
                edgeSumIn += edgeList[edgeListIn[j]].weight;
        // cout<<"outEdgeCostSum: "<<edgeSumOut<<endl;
        // cout<<"inEdgeCostSum: "<<edgeSumIn<<endl;
    }


    void unpackEdges(const vector<EdgeID> &toUnpack, vector<EdgeID> &unpackedEdges, double absMaxError, double relMaxError) const
    // unpack edges until below sqrLength
    {
        static vector<bool> checked(edgeList.size(), false);
        vector<EdgeID> toClean, toDo;
        unpackedEdges.clear();
        for(int i = 0; i < toUnpack.size(); i++) {
            toDo.push_back(toUnpack[i]);
        }

        while(!toDo.empty()) {
            EdgeID curEdge = toDo[toDo.size() - 1];
            toDo.pop_back();
            EdgeID shortA = edgeExtList[curEdge].shortA;
            EdgeID shortB = edgeExtList[curEdge].shortB;
            if(shortA == -1) // we are already at bottom -> produce
            {
                checked[curEdge] = true;
                toClean.push_back(curEdge);
                unpackedEdges.push_back(curEdge);
            }
            // otherwise check edge if not already checked
            if(checked[curEdge] == false) {
                assert(shortA != -1);
                assert(shortB != -1);
                checked[curEdge] = true;
                toClean.push_back(curEdge);
                // test squared length of edge
                NodeID src = edgeList[curEdge].source;
                NodeID trg = edgeList[curEdge].target;

                double dLat = nodeList[src].lat - nodeList[trg].lat;
                double dLon = nodeList[src].lon - nodeList[trg].lon;
                double absError = edgeExtList[curEdge].error;
                double relError = absError / sqrt(dLat * dLat + dLon * dLon);
                //if (dLat*dLat+dLon*dLon>maxSqrLength)         // if too long, push shortcuts
                //cout<<edgeExtList[curEdge].error<<" "<<flush;
                if((absError > absMaxError) || (relError > relMaxError)) {
                    if(!checked[shortA])
                        toDo.push_back(shortA);
                    if(!checked[shortB])
                        toDo.push_back(shortB);
                } else // otherwise produce
                    unpackedEdges.push_back(curEdge);
            }
        }
        // no clean up
        for(int j = 0; j < toClean.size(); j++)
            checked[toClean[j]] = false;

    }
    void unpackEdges()
    {
        // IDEA: if an edge e is selected to be drawn, NO higher shortcut bridging e must be drawn
        //      instead, every shortcut bridging e, must be unpacked until e is shown
        // STRATEGY:    - consider nodes from low level to high level
        //              - inspect all adjacent UPWARD shortcut edges;
        //                      if a shortcut edge bridges an edge to draw or a forbidden edge, mark it FORBIDDEN
        //      This should result in all edges bridging a path containing a drawn edge as being marked FORBIDDEN
        //      Then: as long as there are forbidden edges -> unpack them

        //      Note: an edge to be drawn which is not forbidden after inspection will not become forbidden later on! (True?)
    }


    void checkSorting()
    {
        // make sure that edgeListOut, IDs are sorted DECREASINGLY by level
        for(int i = 0; i < nofNodes(); i++) {
            int oldLevel = 99999999;
            for(int j = edgeOffsetOut[i]; j < edgeOffsetOut[i + 1]; j++) {
                EdgeType curEdge = edgeList[edgeListOut[j]];
                NodeID src = curEdge.source;
                NodeID trg = curEdge.target;
                if(src != i)
                    cout << "X";
                int newLevel = nodeList[trg].level;


                if(newLevel > oldLevel)
                    cout << "a";
                else
                    cout << "b";

                oldLevel = newLevel;
            }
        }
        // make sure that edgeListIn, IDs are sorted DECREASINGLY by level
        for(int i = 0; i < nofNodes(); i++) {
            int oldLevel = 99999999;
            for(int j = edgeOffsetIn[i]; j < edgeOffsetIn[i + 1]; j++) {
                EdgeType curEdge = edgeList[edgeListIn[j]];
                NodeID src = curEdge.source;
                NodeID trg = curEdge.target;
                //cout<<i<<" "<<src<<" "<<trg<<"\n";
                if(trg != i)
                    cout << "Y";
                int newLevel = nodeList[src].level;

                if(newLevel > oldLevel)
                    cout << "A";
                else
                    cout << "B";
                oldLevel = newLevel;
            }
        }
    }

    void writeOutSorted()
    {
        typedef pair<int, int> LevelNodeElement;
        vector<int> new2old;
        vector<int> old2new(nofNodes(), NO_NODE_ID);

        priority_queue<LevelNodeElement> myPQ;
        int counter = 0;
        for(int i = 0; i < nofNodes(); i++)
            myPQ.push(LevelNodeElement(nodeList[i].level, i));
        while(!myPQ.empty()) {
            LevelNodeElement curEl = myPQ.top();
            myPQ.pop();
            new2old.push_back(curEl.second);
            old2new[curEl.second] = counter++;
        }
        assert(new2old.size() == old2new.size());
        // now write out
        // nodes first
        ofstream sortedFile("sorted.gaga");
        sortedFile.precision(10);
        for(int i = 0; i < 9; i++)
            sortedFile << "#\n";
        sortedFile << "\n";
        sortedFile << nofNodes() << "\n";
        sortedFile << nofEdges() << "\n";
        for(int i = 0; i < nofNodes(); i++) // nodes first
        {
            NodeType oldNode = nodeList[new2old[i]];
            sortedFile << i << " " << oldNode.osmID << " " << oldNode.lat << " " << oldNode.lon << " " << oldNode.elev << " " << oldNode.level << "\n";
        }

        vector<int> new2oldEdge;
        vector<int> old2newEdge(nofEdges(), NO_EDGE_ID);
        // first need to determine mapping from old to new edgeIDs
        // IDEA: - sort all edges according to (newsrc, newtrg)
        //       - this yields a new order stored in new2oldEdge and old2newEdge

        vector<tuple<int, int, int>> edgesToSort;
        for(int j = 0; j < nofEdges(); j++) {
            NodeID oldSrc = edgeList[j].source;
            NodeID oldTrg = edgeList[j].target;
            NodeID newSrc = old2new[oldSrc];
            NodeID newTrg = old2new[oldTrg];
            edgesToSort.push_back(tuple<int, int, int>(newSrc, newTrg, j));
        }
        sort(edgesToSort.begin(), edgesToSort.end());
        // now set the stuff
        for(int j = 0; j < nofEdges(); j++) {
            EdgeID oldEdgeID = std::get<2>(edgesToSort[j]);
            new2oldEdge.push_back(oldEdgeID);
            old2newEdge[oldEdgeID] = j;
        }
        for(int j = 0; j < nofEdges(); j++) // now edges
        {
            EdgeID curEdgeOldID = std::get<2>(edgesToSort[j]);
            EdgeType curEdgeOld = edgeList[curEdgeOldID];
            EdgeExtType curExtEdge = edgeExtList[curEdgeOldID];
            NodeID newTarget = old2new[curEdgeOld.target];
            NodeID newSource = old2new[curEdgeOld.source];
            sortedFile << newSource << " " << newTarget << " " << curEdgeOld.weight << " " << curExtEdge.type << " " << curExtEdge.speed << " ";
            if(curExtEdge.shortA != -1)
                sortedFile << old2newEdge[curExtEdge.shortA] << " " << old2newEdge[curExtEdge.shortB] << "\n";
            else
                sortedFile << "-1 -1\n";
        }
    }
};


void readBinaryFMICHgraph(char *fname, FMIGraph &myGraph, bool compressed = false)
{
    cout << "Start binary read " << flush;
    ifstream graphFile(fname, ios_base::in | ios_base::binary);

    if(compressed) {
        boost::iostreams::filtering_istream in;
        in.push(boost::iostreams::zlib_decompressor());
        in.push(graphFile);
        boost::archive::binary_iarchive ia(in);
        ia &myGraph;
    } else {
        boost::archive::binary_iarchive ia(graphFile);
        ia &myGraph;
    }
}

void writeBinaryFMICHgraph(char *fname, const FMIGraph &myGraph, bool compressed = false)
{
    cout << "Writing binary form" << endl;
    ofstream graphFile(fname, ios_base::out | ios_base::binary);

    if(compressed) {
        boost::iostreams::filtering_ostream out;
        out.push(boost::iostreams::zlib_compressor());
        out.push(graphFile);
        boost::archive::binary_oarchive oa(out);
        oa &myGraph;
    } else {
        boost::archive::binary_oarchive oa(graphFile);
        oa &myGraph;
    }
}

#endif
