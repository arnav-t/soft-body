//Softbody Physics by AV 2015
#include <iostream>
#include <math.h>
#include <SDL2/SDL.h>
#include <vector>
const int scr_w=480;
const int scr_h=800;
using namespace std;
struct Vector
{
    float x;
    float y;
};
double magnitude(Vector a)
{
    double m=sqrt(a.x*a.x+a.y*a.y);
    return m;
}
Vector unit(Vector a)
{
    Vector b;
    double m=magnitude(a);
    if(m==0)
        return {0,0};
    b.x=a.x/m;
    b.y=a.y/m;
    return b;
}
double dot(Vector a,Vector b)
{
    double d=a.x*b.x+a.y*b.y;
    return d;
}
class Display
{
    public:
        Display(string win_name,int w,int h)
        {
            win=SDL_CreateWindow(win_name.c_str(),SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,w,h,SDL_WINDOW_SHOWN);
            ren=SDL_CreateRenderer(win,-1,SDL_RENDERER_ACCELERATED);
        }
        ~Display()
        {
            SDL_DestroyTexture(tex);
            SDL_DestroyRenderer(ren);
            SDL_DestroyWindow(win);
        }
        void initTexture(string path)
        {
            SDL_Surface* surf=NULL;
            surf=SDL_LoadBMP(path.c_str());
            if(surf==NULL)
            {
                cout<<"Error:\n"<<SDL_GetError();
            }
            tex=SDL_CreateTextureFromSurface(ren,surf);
            texSize={surf->w,surf->h};
            SDL_FreeSurface(surf);
        }
        void drawLine(Vector a,Vector b,SDL_Color c)
        {
            SDL_SetRenderDrawColor(ren,c.r,c.g,c.b,c.a);
            SDL_RenderDrawLine(ren,a.x,a.y,b.x,b.y);
        }
        void renderTexture(int x,int y)
        {
            SDL_Rect paste={x-texSize.x/2,y-texSize.y/2,texSize.x,texSize.y};
            SDL_RenderCopy(ren,tex,NULL,&paste);
        }
        void clearScreen(SDL_Color c)
        {
            SDL_SetRenderDrawColor(ren,c.r,c.g,c.b,c.a);
            SDL_RenderFillRect(ren,NULL);
        }
        void updateScreen()
        {
            SDL_RenderPresent(ren);
        }
    private:
        SDL_Window* win=NULL;
        SDL_Renderer* ren=NULL;
        SDL_Texture* tex=NULL;
        SDL_Point texSize;
};
class Softbody
{
    public:
        Softbody(float restitution,float spring_constant,float damping)
        {
            e=restitution;
            k=spring_constant;
            c=damping;
        }
        void appendPoint(int x, int y)
        {
            Vector newVertex={x,y};
            soft_body.push_back(newVertex);
            Vector zero={0,0};
            velocity.push_back(zero);
            force.push_back(zero);
            for(int i=0;i<soft_body.size();++i)
            {
                for(int j=0;j<soft_body.size();++j)
                {
                    if(i!=j)
                    {
                        Vector d={(soft_body[j].x-soft_body[i].x)/100.f,(soft_body[j].y-soft_body[i].y)/100.f};
                        sprlen[i<j?i:j][i<j?j:i]=magnitude(d);
                    }
                }
            }
        }
        void acceleratePoints()
        {
            vector<Vector> soft_body_copy=soft_body;
            vector<Vector> velocity_copy=velocity;
            //Vector centroid_copy=centroid;
            //Vector cv_copy=cv;
            for(int i=0;i<soft_body.size();++i)
            {
                force[i]={0,0};
                for(int j=0;j<soft_body.size();++j)
                {
                    if(j!=i)//j==i-1||j==i+1||(j==soft_body.size()-1&&i==0)||(i==soft_body.size()-1&&j==0))
                    {
                        Vector d={(soft_body[j].x-soft_body[i].x)/100.f,(soft_body[j].y-soft_body[i].y)/100.f};
                        if(magnitude(d)!=0)
                        {
                            float t=atan2(d.y,d.x);
                            float m=magnitude(d);
                            float disp=m-sprlen[i<j?i:j][i<j?j:i];
                            Vector anew={(k*disp*cos(t))/10000.f,(k*disp*sin(t))/10000.f};
                            force[i].x+=anew.x;
                            force[i].y+=anew.y;
                            force[i].x-=c*magnitude(d)*(velocity[i].x-velocity[j].x)/100.f;
                            force[i].y-=c*magnitude(d)*(velocity[i].y-velocity[j].y)/100.f;
                            //cout<<c*unit(d).x*(velocity[i].x-velocity[j].x)/100.f<<endl;
                        }
                    }
                }
                velocity_copy[i].x+=force[i].x;
                velocity_copy[i].y+=force[i].y+0.098;
                soft_body_copy[i].x+=velocity_copy[i].x;
                soft_body_copy[i].y+=velocity_copy[i].y;
            }
            soft_body=soft_body_copy;
            velocity=velocity_copy;
        }
        void checkCollision()
        {
            for(int k=0;k<soft_body.size();++k)
            {
                if(soft_body[k].x>=scr_w-1||soft_body[k].x<=0)
                {
                    velocity[k].x*=-e;
                    soft_body[k].x=soft_body[k].x>scr_w/2?scr_w-1:1;
                }
                if(soft_body[k].y>=scr_h-1||soft_body[k].y<=0)
                {
                    velocity[k].y*=-e;
                    soft_body[k].y=soft_body[k].y>scr_h/2?scr_h-1:1;
                }
            }
            /*if(cv.x>=scr_w-1||cv.x<=0)
            {
                cv.x*=-e;
                    //soft_body[k].x=soft_body[k].x>scr_w/2?scr_w-1:1;
            }
            if(cv.y>=scr_h-1||cv.y<=0)
            {
                cv.y*=-e;
                    //soft_body[k].y=soft_body[k].y>scr_h/2?scr_h-1:1;
            }*/
        }
        void drag(int x, int y)
        {
            Vector mouse={x-soft_body[0].x,y-soft_body[0].y};
            mouse=unit(mouse);
            for(int q=0;q<1;++q)
            {
                Vector d={soft_body[q].x-mouse.x,soft_body[q].y-mouse.y};
                d=unit(d);
                velocity[q].x+=2*mouse.x;//+fabs(d.x/10);
                velocity[q].y+=2*mouse.y;//+fabs(d.y/10);
            }
        }
        void drawBody(Display& display,SDL_Color c,SDL_Color d)
        {
            for(int z=0;z<soft_body.size();++z)
            {
                display.renderTexture(soft_body[z].x,soft_body[z].y);
                display.drawLine(soft_body[z],soft_body[(z+1)%soft_body.size()],c);
            }
            /*display.renderTexture(centroid.x,centroid.y);
            for(int r=0;r<soft_body.size();++r)
            {
                display.drawLine(centroid,soft_body[r],d);
            }*/
        }
        Vector returnDrag()
        {
            return(soft_body[0]);
        }
    private:
        vector<Vector> soft_body;
        vector<Vector> velocity;
        vector<Vector> force;
        float sprlen[200][200];
        //Vector centroid;
        //Vector cv;
        //float natural_length=50;
        float e;
        float k;
        float c;
};
int main(int argc,char *argv[])
{
    Softbody body(0.8,50000,1);
    Display display("Soft Body Physics",scr_w,scr_h);
    display.initTexture("img/point.bmp");
    SDL_Color white={255,255,255,255};
    SDL_Color red={255,0,0,255};
    SDL_Color green={0,255,0,255};
    bool init=true,quit=false,drag=false;
    int x,y;
    Vector mouse;
    SDL_Event e;
    while(!quit)
    {
        while(SDL_PollEvent(&e)!=0)
        {
            if(e.type==SDL_QUIT)
            {
                quit=true;
                break;
            }
            else if(init)
            {
                if(e.type==SDL_MOUSEMOTION||e.type==SDL_MOUSEBUTTONDOWN)
                {
                   // if(e.button.button==SDL_BUTTON_RIGHT)
                        
                    if(e.button.button==SDL_BUTTON_LEFT)
                    {
                        SDL_GetMouseState(&x,&y);
                        if(y<scr_h/10)
                            init=false;
                        else
                            body.appendPoint(x,y);
                    }
                }
            }
            else
            {
                if(e.type==SDL_MOUSEBUTTONDOWN)
                {
                    drag=true;
                    SDL_GetMouseState(&x,&y);
                }
                if(drag)
                {
                    if(e.type==SDL_MOUSEMOTION)
                        SDL_GetMouseState(&x,&y);
                }
                if(e.type==SDL_MOUSEBUTTONUP)
                    drag=false;
            }
        }
        if(!init)
        {
            if(drag)
            {
                body.drag(x,y);
            }
            body.checkCollision();
            body.acceleratePoints();
        }
        display.clearScreen(white);
        body.drawBody(display,red,green);
        if(drag)
            display.drawLine({x,y},body.returnDrag(),green);
        display.updateScreen();
        SDL_Delay(10);
    }
    return 0;
}﻿