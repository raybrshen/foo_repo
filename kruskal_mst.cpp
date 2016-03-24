//
//  Kruskal MST.cpp
//  Algorithm
//
//  Created by Ray Shen on 2015-01-31.
//  Copyright (c) 2015 Ray Shen. All rights reserved.
//
//  Implemented Kruskal's algorithm for finding minimum spanning tree in a weighted undirected graph.
// 

#include <cstdio>
#include <cstring>
#include <algorithm>

typedef struct _node
{
    int id;
    _node* next;
}NODE, *PNODE;

typedef struct edge
{
    int a;
    int b;
    int dis;
}EDGE;

int map[100][100];
int n;
int ret;
int edgenum;
EDGE elist[100*99/2];
PNODE plist[100][2];

bool compare(const EDGE& lhs, const edge& rhs)
{
    return lhs.dis < rhs.dis;
}

void Kruskal()
{
    int i, a, b;
    for(i=0; i<edgenum; i++)
    {
        a = elist[i].a;
        b = elist[i].b;
        if(plist[a][0]==NULL && plist[b][0]==NULL)
        {
            PNODE phead = new NODE;
            PNODE pnode = new NODE;
            phead->id = a;
            phead->next = pnode;
            pnode->id = b;
            pnode->next = NULL;
            plist[a][0] = phead;
            plist[b][0] = phead;
            plist[a][1] = pnode;
            plist[b][1] = pnode;
            ret += elist[i].dis;
        }
        else if(plist[a][0]==NULL && plist[b][0]!=NULL)
        {
            PNODE ptail = plist[b][1];
            PNODE phead = new NODE;
            phead->id = a;
            phead->next = plist[b][0];
            PNODE ptemp = phead;
            while(ptemp != NULL)
            {
                plist[ptemp->id][0] = phead;
                plist[ptemp->id][1] = ptail;
                ptemp = ptemp->next;
            }
            ret += elist[i].dis;
        }
        else if(plist[a][0]!=NULL && plist[b][0]==NULL)
        {
            PNODE ptail = plist[a][1];
            PNODE phead = new NODE;
            phead->id = b;
            phead->next = plist[a][0];
            PNODE ptemp = phead;
            while(ptemp != NULL)
            {
                plist[ptemp->id][0] = phead;
                plist[ptemp->id][1] = ptail;
                ptemp = ptemp->next;
            }
            ret += elist[i].dis;
        }
        else if(plist[a][0]!=NULL && plist[b][0]!=NULL && plist[a][0]!=plist[b][0])
        {
            PNODE phead = plist[a][0];
            PNODE ptail = plist[b][1];
            plist[a][1]->next = plist[b][0];
            PNODE ptemp = phead;
            while(ptemp != NULL)
            {
                plist[ptemp->id][0] = phead;
                plist[ptemp->id][1] = ptail;
                ptemp = ptemp->next;
            }
            ret += elist[i].dis;
        }
        else continue;
    }
    PNODE pn = plist[0][0];
    PNODE pold = NULL;
    while(pn != NULL)
    {
        pold = pn;
        pn = pn->next;
        delete pold;
    }
}

int main(int argc, const char * argv[])
{
    int i, j, k;
    while(scanf("%d", &n) != EOF)
    {
        k=0;
        edgenum = n*(n-1)/2;
        ret = 0;
        memset(plist, 0, sizeof(plist));
        memset(elist, 0, sizeof(elist));
        for(i=0; i<n; i++)
        {
            for(j=0; j<n; j++)
            {
                scanf("%d", &map[i][j]);
                if(j>i)
                {
                    elist[k].a = i;
                    elist[k].b = j;
                    elist[k].dis = map[i][j];
                    k++;
                }
            }
        }
        std::sort(elist, elist+edgenum, compare);
        Kruskal();
        printf("%d\n", ret);
    }
    return 0;
 }
 