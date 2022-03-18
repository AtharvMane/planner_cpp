#include <bits/stdc++.h>

void solve(int a[1024][1024],int strt_x,int end_x,int strt_y,int end_y,int x,int y){
    int centre_x=(end_x+strt_x+1)/2;
    int centre_y=(end_y+strt_y+1)/2;
    if(strt_x-end_x==1){
        if(x==strt_x){
            a[end_x][strt_y]=1;
            a[end_x][end_y]=1;
            if(y==strt_y){
                a[strt_x][end_y]=0;
            }else{
                a[strt_x][strt_y]=0;

            }
        }else{
            a[strt_x][strt_y]=1;
            a[strt_x][end_y]=1;
            if(y==strt_y){
                a[end_x][end_y]=0;
            }else{
                a[end_x][strt_y]=0;

            }

        }
    }
    if(x<centre_x && y<centre_y){
        solve(a,strt_x,centre_x-1,strt_y,centre_y-1,x,y);
        solve(a,centre_x,end_x,strt_y,centre_y-1,centre_x,centre_y-1);
        solve(a,strt_x,centre_x-1,centre_y,end_y,centre_x-1,centre_y-1);
        solve(a,centre_x,end_x,centre_y,end_y,centre_x,end_y);
    }else if(x>=centre_x && y<centre_y){
        solve(a,strt_x,centre_x-1,strt_y,centre_y-1,centre_x-1,centre_y-1);
        solve(a,centre_x,end_x,strt_y,centre_y-1,x,y);
        solve(a,strt_x,centre_x-1,centre_y,end_y,centre_x-1,centre_y-1);
        solve(a,centre_x,end_x,centre_y,end_y,centre_x,end_y);
    }else if(x>=centre_x && y>=centre_y){
        solve(a,strt_x,centre_x-1,strt_y,centre_y-1,centre_x-1,centre_y-1);
        solve(a,centre_x,end_x,strt_y,centre_y-1,centre_x,centre_y-1);
        solve(a,strt_x,centre_x-1,centre_y,end_y,centre_x-1,centre_y-1);
        solve(a,centre_x,end_x,centre_y,end_y,x,y);
    }else if(x<centre_x && y>=centre_y){
        solve(a,strt_x,centre_x-1,strt_y,centre_y-1,centre_x-1,centre_y-1);
        solve(a,centre_x,end_x,strt_y,centre_y-1,centre_x,centre_y-1);
        solve(a,strt_x,centre_x-1,centre_y,end_y,x,y);
        solve(a,centre_x,end_x,centre_y,end_y,centre_x,end_y);
    }

}

int main() {

    /* Enter your code here. Read input from STDIN. Print output to STDOUT */    
    return 0;
}