#include <iostream>
#include <GL/freeglut.h>
#include <utility>
#include <iostream>

// using std::string;

extern float ballX;
extern float ballSpeed;
extern float cellSize;
extern float z_depth;
extern int windowWidth;
extern int windowHeight;
extern int limitStep;
extern int step;
extern std::pair<float, float> origin; // origen de coordenadas
// std::pair<float, float> origin = std::make_pair(0.0f, 0.0f); // origen de coordenadas


void translate(float x, float y) ;


std::pair<float, float> transform(float x, float y);


void reshape(int w, int h) ;


void updateLogic() ;
    

void idleFunc() ;


int getBitmapStringWidth(void* font, std::string string) ;


void renderBitmapString(float x, float y, std::string string, void* font = GLUT_BITMAP_HELVETICA_18) ;
    

void drawSquare(float x, float y, float size) ;


void drawCircle(float x, float y, float radius, float r, float g, float b, float a) ;


void drawAgent(float x, float y, float radius = cellSize/2, float sightRadius= cellSize);
    

void drawLine(float initX, float initY, float finalX, float finalY, float r, float g, float b, float width = 20.0f);


void drawGrid();


void display() ; 


void prepareRenderer();
