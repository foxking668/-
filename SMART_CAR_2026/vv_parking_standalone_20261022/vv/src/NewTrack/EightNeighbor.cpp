#include "EightNeighbor.hpp"
#include <algorithm>
#include <cmath>

namespace NewTrack { namespace EightNeighbor {
namespace {
using Roundabout::Point;
const int dx[8]={1,1,0,-1,-1,-1,0,1};
const int dy[8]={0,1,1,1,0,-1,-1,-1};

std::vector<Point> walk(const unsigned char* p,int w,int h,std::size_t stride,
                        bool mirror,const std::vector<int>& hints) {
    auto white=[&](int x,int y) {
        return x>=0 && x<w && y>=0 && y<h && p[y*stride+(mirror?w-1-x:x)]!=0;
    };
    auto follow=[&](Point seed) {
        std::vector<Point> path;
        Point current=seed,back{seed.x-1,seed.y};
        std::vector<unsigned char> visited(static_cast<std::size_t>(w)*h*8,0);
        for(int steps=0;steps<w*h*8;++steps) {
            int start=4;
            for(int d=0;d<8;++d) if(current.x+dx[d]==back.x && current.y+dy[d]==back.y) {start=d;break;}
            const auto key=(static_cast<std::size_t>(current.y)*w+current.x)*8+start;
            if(visited[key]) break;
            visited[key]=1;path.push_back(current);
            int next=-1;
            for(int k=1;k<=8;++k) {
                const int d=(start+k)%8;
                if(white(current.x+dx[d],current.y+dy[d])) {next=d;break;}
            }
            if(next<0) break;
            const int previous=(next+7)%8;
            back={current.x+dx[previous],current.y+dy[previous]};
            current={current.x+dx[next],current.y+dy[next]};
        }
        if(!path.empty()) {
            const auto top=std::min_element(path.begin(),path.end(),[](Point a,Point b){return a.y<b.y;});
            path.erase(top+1,path.end());
            if(path.front().y-path.back().y<h/6) path.clear();
        }
        return path;
    };
    std::vector<Point> path;
    // Use the outer side of the nearest white run. Hints select a run, never fabricate evidence.
    // The lowest 10% often contains the bumper/wheels. Start above it and leave
    // unobserved lower rows missing for the existing width-based repair.
    for(int y=h-1-std::max(3,h/10);y>=h*2/3;--y) {
        int target=w/4;
        if(static_cast<int>(hints.size())==h && hints[y]>=0 && hints[y]<w)
            target=mirror?w-1-hints[y]:hints[y];
        std::vector<Point> seeds;
        for(int x=0;x<w;) {
            if(!white(x,y)) {++x;continue;}
            const int a=x;while(x<w && white(x,y)) ++x;
            const int b=x-1;if(b-a+1<3) continue;
            seeds.push_back({a,y});
        }
        std::stable_sort(seeds.begin(),seeds.end(),[&](Point a,Point b){return std::abs(a.x-target)<std::abs(b.x-target);});
        for(const auto seed:seeds) {
            auto candidate=follow(seed);
            if(candidate.empty()) continue;
            if(path.empty() || candidate.back().y<path.back().y) path=std::move(candidate);
            if(path.back().y<=h/3) break;
        }
        if(!path.empty() && path.back().y<=h/3) break;
    }
    if(mirror) for(auto& q:path) q.x=w-1-q.x;
    return path;
}
}
Boundary trace(const unsigned char* p,int w,int h,std::size_t stride,
               const std::vector<int>& lh,const std::vector<int>& rh) {
    Boundary out;
    if(w<=0 || h<=0) return out;
    out.left.assign(h,-1);out.right.assign(h,-1);
    if(!p || w<12 || h<12 || stride<static_cast<std::size_t>(w)) return out;
    out.leftPath=walk(p,w,h,stride,false,lh);
    out.rightPath=walk(p,w,h,stride,true,rh);
    for(auto q:out.leftPath) out.left[q.y]=std::max(out.left[q.y],q.x);
    for(auto q:out.rightPath) if(out.right[q.y]<0 || q.x<out.right[q.y]) out.right[q.y]=q.x;
    return out;
}
} }
