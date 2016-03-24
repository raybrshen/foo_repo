//
//  main.cpp
//  Algorithm
//
//  Created by Ray Shen on 2015-01-31.
//
//  Implemented Prim's algorithm for finding minimum spanning tree in a weighted undirected graph.
// 

#include <cstdio>
#include <cstring>

#define INF 1000000;

int ret, n;
bool used[100];
int dis[100];
int map[100][100];


void prim()
{
    int i, j, k, min;
    memset(used, 0, sizeof(used));
    
    for(i=0; i<n; i++)
    {
        dis[i] = map[0][i];
    }
    used[0] = true;
    
    for(i=1; i<n; i++)
    {
        min = INF;
        k = 0;
        for(j=0; j<n; j++)
        {
            if(!used[j] && dis[j]<min)
            {
                min = dis[j];
                k = j;
            }
        }
        ret += min;
        used[k] = true;
        for(j=0; j<n; j++)
        {
            if(!used[j] && dis[j]>map[k][j])
            {
                dis[j] = map[k][j];
            }
        }
    }
}

int main(int argc, const char * argv[]) {
    int i, j;
    while(scanf("%d", &n)!=EOF)
    {
        ret = 0;
        for(i=0; i<n; i++)
        {
            for(j=0; j<n; j++)
            {
                scanf("%d", &map[i][j]);
            }
        }
        prim();
        printf("%d\n", ret);
    }
    
    return 0;
}
