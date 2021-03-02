#pragma once

#include <boost/serialization/utility.hpp>
#include <climits>
#include <functional>
#include <omp.h>
#include <optional>
#include <pathfinding/Distance.hpp>
#include <pathfinding/FMIGraph.hpp>
#include <pathfinding/Path.hpp>
#include <queue>
#include <string_view>
#include <vector>

using namespace std;

typedef pair<NodeID, EdgeCost> HubType;

class HubLabels
{
public:
    vector<vector<HubType>> outLabels;
    vector<vector<HubType>> inLabels;

    vector<EdgeCost> srcDist;
    vector<EdgeCost> trgDist;

    vector<int> lvl2id; // lvl[i] contains ID of last node with level i
    int hybLabCollected; // counts number of collected labels in hybrid approach



    const FMIGraph &hubMG;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &outLabels;
        ar &inLabels;
    }

public:
    HubLabels(const class FMIGraph &G)
        : hubMG(G)
    {
        outLabels.resize(hubMG.nofNodes(), vector<HubType>());
        inLabels.resize(hubMG.nofNodes(), vector<HubType>());

        srcDist.resize(hubMG.nofNodes(), MAX_EDGE_COST);
        trgDist.resize(hubMG.nofNodes(), MAX_EDGE_COST);
        cout << "We have maxLevel " << hubMG.nodeList[0].level << endl;
        lvl2id.resize(hubMG.nodeList[0].level + 1);
        for(int i = 0; i < hubMG.nofNodes(); i++)
            lvl2id[hubMG.nodeList[i].level] = i;
        for(int i = 0; i < 4; i++)
            cout << "StopID for level " << i << " is " << lvl2id[i] << endl;
        for(int i = hubMG.nodeList[0].level - 4; i <= hubMG.nodeList[0].level; i++)
            cout << "StopID for level " << i << " is " << lvl2id[i] << endl;
    }

    void printHLsize()
    {
        int curNode = 0;
        int maxNode = outLabels.size() - 1;
        cout << "MaxNode: " << maxNode << endl;
        int maxLevel = hubMG.nodeList[0].level;
        cout << "MaxLevel: " << maxLevel << endl;
        double nodeCount[maxLevel + 1];
        double sizeCount[maxLevel + 1];
        for(int i = 0; i < maxLevel + 1; i++) {
            sizeCount[i] = 0;
            nodeCount[i] = 0;
        }
        do {
            int curLevel = hubMG.nodeList[curNode].level;
            sizeCount[curLevel] += outLabels[curNode].size();
            sizeCount[curLevel] += inLabels[curNode].size();
            nodeCount[curLevel]++;
            curNode++;
        } while(curNode <= maxNode);

        double cumul = 0;
        for(int i = maxLevel; i >= 0; i--) {
            cumul += sizeCount[i];
            int aggsizeMB = cumul * sizeof(HubType) / (1024 * 1024);
            cout << "Down to level " << i << " " << aggsizeMB << "MB"
                 << " or " << int(cumul * sizeof(HubType) / hubMG.nofNodes()) << "bytes/node \n";
        }
        cout << "Average HubLabel Size: " << cumul / maxNode << endl;
    }


    void pruneLabel(vector<HubType> &curLabel)
    {
        int lastID = -1;
        vector<HubType> result;
        for(auto & i : curLabel)
            if(i.first != lastID) {
                result.push_back(i);
                lastID = i.first;
            }
        curLabel = result;
    }

    void mergeLabels(const vector<vector<HubType>> &oldLabels, vector<HubType> &newLabel)
    // make a k-way merge of all oldLabels
    {
        newLabel.clear();
        int k = oldLabels.size();
        // maintain k positions, always make one scan, determine smallest hub, push to new Label, kill all with same node ID
        int positions[k];
        int sizes[k];
        int non_finished = 0;
        for(int i = 0; i < k; i++) {
            positions[i] = 0;
            sizes[i] = oldLabels[i].size();
            if(sizes[i] > 0)
                non_finished++;
        }
        //		cout<<"now merge with k="<<k<<endl;
        while(non_finished > 0) {
            HubType bestHub = make_pair(numeric_limits<NodeID>::max(), MAX_EDGE_COST);
            for(int i = 0; i < k; i++) // determine smallest hub
            {
                if(positions[i] < sizes[i]) {
                    //					cout<<"Compare with "<<oldLabels[i][positions[i]].first<<" ";
                    if(bestHub > oldLabels[i][positions[i]])
                        bestHub = oldLabels[i][positions[i]];
                }
            }
            // progress on all with same ID
            for(int i = 0; i < k; i++) {
                if(positions[i] < sizes[i]) {
                    if(bestHub.first == oldLabels[i][positions[i]].first)
                        positions[i]++;
                    if(positions[i] >= sizes[i])
                        non_finished--;
                }
            }
            if(bestHub.second == MAX_EDGE_COST)
                exit(0);
            newLabel.push_back(bestHub);
            //			cout<<"Pushed: "<<bestHub.first<<"="<<bestHub.second<<" ";
        }
    }

    EdgeCost distRecovery(const vector<HubType> &srcDists, const vector<HubType> &trgDists, NodeID src, NodeID trg, int stopLevel)
    {
        // sort according to ID,cost
        // go from back in parallel
        // - if ID matches, have tentative distance
        // - if level high enough, add to current labels
        vector<HubType> newSRClabel, newTRGlabel;

        vector<vector<HubType>> oldSRClabels, oldTRGlabels;

        int ind_s = srcDists.size() - 1, ind_t = trgDists.size() - 1;

        EdgeCost bestDist = MAX_EDGE_COST;

        int stopID = lvl2id[stopLevel];

        while((ind_s >= 0) || (ind_t >= 0)) {
            const HubType &srcHub = srcDists[ind_s], &trgHub = trgDists[ind_t];

            int order = srcHub.first - trgHub.first;
            if(ind_t < 0)
                order = 1;
            if(ind_s < 0)
                order = -1;
            if(order == 0) {
                bestDist = std::min(bestDist, srcHub.second + trgHub.second);
                ind_s--;
                ind_t--;
                if(srcHub.first <= stopID) // (hubMG.nodeList[srcHub.first].level>=stopLevel)
                {
                    const vector<HubType> &srcLabel = outLabels[srcHub.first];
                    //oldSRClabels.push_back(srcLabel);
                    newSRClabel.clear();
                    for(int j = 0; j < srcLabel.size(); j++) // better: append vector?
                    {
                        newSRClabel.push_back(make_pair(srcLabel[j].first, srcLabel[j].second + srcHub.second));
                    }
                    oldSRClabels.push_back(newSRClabel);
                }
                if(trgHub.first <= stopID) // (hubMG.nodeList[trgHub.first].level>=stopLevel)
                {
                    const vector<HubType> &trgLabel = inLabels[trgHub.first];
                    //oldTRGlabels.push_back(trgLabel);
                    newTRGlabel.clear();
                    for(int j = 0; j < trgLabel.size(); j++) // better: append vector?
                    {
                        newTRGlabel.push_back(make_pair(trgLabel[j].first, trgLabel[j].second + trgHub.second));
                    }
                    oldTRGlabels.push_back(newTRGlabel);
                }
                //				cout<<"Found direct distance: "<<bestDist<<endl;
            } else if(order > 0) {
                ind_s--;
                if(srcHub.first <= stopID) // (hubMG.nodeList[srcHub.first].level>=stopLevel)
                {
                    const vector<HubType> &srcLabel = outLabels[srcHub.first];
                    //oldSRClabels.push_back(srcLabel);
                    newSRClabel.clear();
                    for(int j = 0; j < srcLabel.size(); j++) // better: append vector?
                    {
                        newSRClabel.push_back(make_pair(srcLabel[j].first, srcLabel[j].second + srcHub.second));
                    }
                    oldSRClabels.push_back(newSRClabel);
                }
            } else {
                ind_t--;
                if(trgHub.first <= stopID) // (hubMG.nodeList[trgHub.first].level>=stopLevel)
                {
                    const vector<HubType> &trgLabel = inLabels[trgHub.first];
                    //oldTRGlabels.push_back(trgLabel);
                    newTRGlabel.clear();
                    for(int j = 0; j < trgLabel.size(); j++) // better: append vector?
                    {
                        newTRGlabel.push_back(make_pair(trgLabel[j].first, trgLabel[j].second + trgHub.second));
                    }
                    oldTRGlabels.push_back(newTRGlabel);
                }
            }
        }
        //		myTimer.stop();
        //		cout<<"Main loop: "<<myTimer.usecs()<<"us"<<endl;
        //		myTimer.start();
        mergeLabels(oldSRClabels, newSRClabel);
        mergeLabels(oldTRGlabels, newTRGlabel);
        //		myTimer.stop();
        // now we have found in bestDist the best distance in the up searches; now also compare with hubLabel distance
        //		sort(newSRClabel.begin(), newSRClabel.end());
        //		sort(newTRGlabel.begin(), newTRGlabel.end());
        //		cout<<"new sizes: "<<newSRClabel.size()<<" "<<newTRGlabel.size()<<endl;
        //		pruneLabel(newSRClabel);
        //		pruneLabel(newTRGlabel);
        //		cout<<"new2 sizes: "<<newSRClabel.size()<<" "<<newTRGlabel.size()<<endl;
        //		cout<<"Sort&Prune: "<<myTimer.usecs()<<"us"<<endl;

        EdgeCost hlDist = distOracle(newSRClabel, newTRGlabel, src, trg);
        if(hlDist < bestDist)
            bestDist = hlDist;

        return bestDist;
    }


    EdgeCost findDistance(int s, int t) const // compute distance between node i and j using hub labels
    {
        return distOracle(s, t);
    }

    EdgeCost distOracle(int s, int t, bool debug = false) const  // compute distance between node i and j using hub labels
    {
        return distOracle(outLabels[s], inLabels[t], s, t, debug);
    }
    EdgeCost distOracle(const vector<HubType> &outLabel, const vector<HubType> &inLabel, int s, int t, bool debug = false) const
    {
        int ind_s = 0, ind_t = 0;
        int sz_s = outLabel.size();
        int sz_t = inLabel.size();


        EdgeCost bestDist = MAX_EDGE_COST;
        NodeID bestNode = NO_NODE_ID;


        while((ind_s < sz_s) && (ind_t < sz_t)) {
            HubType srcHub = outLabel[ind_s], trgHub = inLabel[ind_t];
            int order = srcHub.first - trgHub.first;
            if(order == 0) {
                EdgeCost tmpDist = std::min(bestDist, srcHub.second + trgHub.second);
                if(tmpDist < bestDist) {
                    bestDist = tmpDist;
                    bestNode = srcHub.first;
                    if(debug) {
                        cout << bestNode << ": " << srcHub.second << " + " << trgHub.second << endl;
                    }
                }
                ind_s++;
                ind_t++;
            } else if(order < 0)
                ind_s++;
            else
                ind_t++;
        }

        return bestDist;
    }
    void constructOutLabel(int curNode)
    {
        // iterate over all out neighbors, collect their hubs, add edge cost and check each hub for optimality
        // also add neighbor itself !
        vector<HubType> tmpHubs;
        // add itself
        tmpHubs.push_back(HubType(curNode, 0));
        for(int j = hubMG.edgeOffsetOut[curNode]; j < hubMG.edgeOffsetOut[curNode + 1]; j++) {
            NodeID trgNode = hubMG.edgeList[hubMG.edgeListOut[j]].target;
            EdgeCost dist = hubMG.edgeList[hubMG.edgeListOut[j]].weight;
            if(trgNode < curNode) // only higher nodes
            {
                // add neighbor itself
                tmpHubs.push_back(HubType(trgNode, dist));
                vector<HubType> &trgLabel = outLabels[trgNode];
                for(int i = 0; i < trgLabel.size(); i++)
                    tmpHubs.push_back(HubType(trgLabel[i].first, trgLabel[i].second + dist));
            }
        }
        sort(tmpHubs.begin(), tmpHubs.end());
        // now we have unpruned hub set for curNode; use this to self-prune
        // TODO
        vector<HubType> finalHubs;
        for(int i = 0; i < tmpHubs.size(); i++) {
            int curTrg = tmpHubs[i].first;
            EdgeCost curDist = tmpHubs[i].second;
            EdgeCost realDist;
            if(curTrg == curNode) {
                realDist = 0;
            } else
                realDist = distOracle(tmpHubs, inLabels[curTrg], curNode, curTrg);
            if((curDist <= realDist) || (curTrg == curNode))
                finalHubs.push_back(tmpHubs[i]);
            if(curDist < realDist) {
                cout << "FUCK! with" << curDist << " and " << realDist << endl;
                exit(0);
            }
        }

        auto last = std::unique(finalHubs.begin(), finalHubs.end());
        finalHubs.erase(last, finalHubs.end());
        //cout<<"OUT: final vs tmp: "<<finalHubs.size()<<" "<<tmpHubs.size()<<endl;
        // add sentinel?
        outLabels[curNode] = finalHubs;
    }


    void constructInLabel(int curNode)
    {
        // iterate over all in neighbors, collect their hubs, add edge cost and check each hub for optimality
        vector<HubType> tmpHubs;
        // add itself
        tmpHubs.push_back(HubType(curNode, 0));
        for(int j = hubMG.edgeOffsetIn[curNode]; j < hubMG.edgeOffsetIn[curNode + 1]; j++) {
            NodeID srcNode = hubMG.edgeList[hubMG.edgeListIn[j]].source;
            EdgeCost dist = hubMG.edgeList[hubMG.edgeListIn[j]].weight;
            if(srcNode < curNode) // only higher nodes
            {
                // add neighbor itself
                tmpHubs.push_back(HubType(srcNode, dist));
                vector<HubType> &srcLabel = inLabels[srcNode];
                for(int i = 0; i < srcLabel.size(); i++)
                    tmpHubs.push_back(HubType(srcLabel[i].first, srcLabel[i].second + dist));
            }
        }
        sort(tmpHubs.begin(), tmpHubs.end());
        // now we have unpruned hub set for curNode; use this to self-prune
        // TODO
        vector<HubType> finalHubs;
        for(int i = 0; i < tmpHubs.size(); i++) {
            int curSrc = tmpHubs[i].first;
            EdgeCost curDist = tmpHubs[i].second;
            int realDist = distOracle(outLabels[curSrc], tmpHubs, curSrc, curNode);
            if((curDist <= realDist) || (curSrc == curNode))
                finalHubs.push_back(tmpHubs[i]);
            if(curDist < realDist) {
                cout << "FUCK!" << endl;
                exit(0);
            }
        }
        auto last = std::unique(finalHubs.begin(), finalHubs.end());
        finalHubs.erase(last, finalHubs.end());
        //cout<<"IN: final vs tmp: "<<finalHubs.size()<<" "<<tmpHubs.size()<<endl;
        // add sentinel?
        inLabels[curNode] = finalHubs;
    }



    void constructLabelsParallel(int cutLevel)
    {
        double sizeSum = 0;
        int maxSize = 0;
        int maxLevel = hubMG.nodeList[0].level;
        cout << "MaxLevel is: " << maxLevel << " going down to cutLevel=" << cutLevel << endl;
        int curNode = 0; // always point to the start of next node block (of a level)

        for(int level = maxLevel; level >= cutLevel; level--) {
            cout << "Level: " << level << ": Going from " << curNode << " to " << lvl2id[level] << endl;

#pragma omp parallel for
            for(int i = curNode; i <= lvl2id[level]; i++) {
                constructOutLabel(i);
                constructInLabel(i);
            }
            for(int i = curNode; i <= lvl2id[level]; i++) {
                sizeSum += outLabels[i].size();
                sizeSum += inLabels[i].size();
                maxSize = std::max(maxSize, int(outLabels[i].size()));
                maxSize = std::max(maxSize, int(inLabels[i].size()));
            }

            curNode = lvl2id[level] + 1;
            cout << "End: " << lvl2id[level] << endl;
            // if (curNode%1000==0)
            {
                cout << "Finished Level: " << level << " at node " << curNode - 1 << "\t";
                cout << "(" << sizeof(HubType) * int(sizeSum / (1024 * 1024)) << "MB) " << endl;
            }
        }
        cout << "avg. label size " << sizeSum / (2 * hubMG.nofNodes()) << " max label size: " << maxSize << "\n";
    }
};


inline auto generateHubLabelsFromFmiFile(const std::string &fmi_file)
    -> HubLabels
{
    FMIGraph fmi_graph;
    fmi_graph.readFromFMIFile(fmi_file);
    fmi_graph.writeOutSorted();
    fmi_graph.readFromFMIFile("sorted.gaga");

    HubLabels hl(fmi_graph);
    hl.constructLabelsParallel(0);

    return hl;
}
