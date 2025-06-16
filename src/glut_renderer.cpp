// #include <iostream>
// #include <GL/freeglut.h>
// #include <utility>
// #include <iostream>

#include "glut_renderer.h"

using std::string;

float ballX = 2.0f;
float ballSpeed = 0.04f;

float cellSize = 0.05f;
float z_depth = -0.0f;
int windowWidth = 1000;
int windowHeight = 1000;

int limitStep = 1000;
int step = -1;

// std::pair<float, float> origin = std::make_pair(0.0f, 0.0f); // origen de coordenadas
std::pair<float, float> origin = std::make_pair(-0.9f, -0.9f); // origen de coordenadas

void translate(float x, float y) {
    glTranslatef(origin.first + x*cellSize, origin.second + y*cellSize, z_depth);
}

std::pair<float, float> transform(float x, float y){
    return std::make_pair((float) (origin.first + x*cellSize), (float) (origin.second + y*cellSize) );
}

void reshape(int w, int h) {
    if (h == 0) h = 1;
    float aspect = (float)w / (float)h;

    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, aspect, 0.1, 100.0);

    glMatrixMode(GL_MODELVIEW);
}

// void updateLogic() {
//     step += 1;
//     if (step>= limitStep){
//         glutLeaveMainLoop();
//     }
//     // Actualiza la posición del círculo
//     ballX += ballSpeed;

//     if (ballX > 3.0f || ballX < 2.0f) {
//         ballSpeed = -ballSpeed;  // rebote simple
//     }
// }

void idleFunc() {
    updateLogic();
    glutPostRedisplay();  // Solicita redibujar la ventana
}

int getBitmapStringWidth(void* font, string string) {
    int width = 0;
    for (char c : string){
        width += glutBitmapWidth(font, c);
    }
    return width;
}

void renderBitmapString(float x, float y, string string, void* font) {
    glPushMatrix();
    translate(x, y);

    // Obtiene el ancho del texto en píxeles
    int textWidthPixels = getBitmapStringWidth(font, string);

    // Ajuste aproximado: asume que cada 100 píxeles es aproximadamente una celda
    // Ajusta 100.0f según la resolución y tamaño de tu ventana y fuente
    float pixelsPerCell = 50.0f;
    float textWidthUnits = (textWidthPixels / pixelsPerCell) * cellSize;

    glColor3f(1.0f, 1.0f, 1.0f); 

    // Centra horizontalmente
    glRasterPos2f(-textWidthUnits / 2.0f, 0.0f);

    for (char c : string){
        glutBitmapCharacter(font, c);
    }

    glPopMatrix();
}

void drawSquare(float x, float y, float size) {
    glPushMatrix();

    translate(x, y);
    glColor3f(1.0f, 0.0f, 0.0f);

    glBegin(GL_QUADS);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(size, 0.0f, 0.0f);
        glVertex3f(size, size, 0.0f);
        glVertex3f(0.0f, size, 0.0f);
    glEnd();

    glPopMatrix();
}

void drawCircle(float x, float y, float radius, float r, float g, float b, float a) {
    glPushMatrix();

    translate(x, y);
    glColor4f(r, g, b, a);  
    glutSolidSphere(radius, 20, 20);

    glPopMatrix();
}

void drawAgent(float x, float y, float radius, float sightRadius){
    int id = -1000;
    string string = std::to_string(id);
    drawCircle(x, y, sightRadius, 0.0f, 0.0f, 0.0f, 0.5f);
    drawCircle(x, y, radius,      1.0f, 0.5f, 0.0f, 1.0f);
    renderBitmapString(x, y, string);
}

void drawLine(float initX, float initY, float finalX, float finalY, float r, float g, float b, float width){
    glPushMatrix();

    std::pair<float, float> init = transform(initX, initY);
    std::pair<float, float> final = transform(finalX, finalY);

    glColor3f(r, g, b); // rojo
    glLineWidth(width); // línea de 5 píxeles de grosor

    glBegin(GL_LINES);
        glVertex2f(init.first, init.second);
        glVertex2f(final.first, final.second);
    glEnd();

    glPopMatrix();
}

void drawGrid(){
    float cellsX = windowWidth / cellSize;
    float cellsY = windowHeight / cellSize;

    drawLine(0.0f, 0.0f, cellsX, 0.0f, 0.5f, 0.5f, 0.5f);
    drawLine(0.0f, 0.0f, 0.0f, cellsY, 0.5f, 0.5f, 0.5f);

    float divisorLine = 0.3f;

    if(windowHeight == windowWidth){
        for (int i=0; i< cellsX; ++i){
            drawLine(i, 0, i, cellsX, 0.5f, 0.5f, 0.5f, 1);
            drawLine(0, i, cellsY, i, 0.5f, 0.5f, 0.5f, 1);

            drawLine(i, divisorLine, i, -divisorLine, 0.5f, 0.5f, 0.5f, 5);
            drawLine(divisorLine, i, -divisorLine, i, 0.5f, 0.5f, 0.5f, 5);

            if (i % 5 == 0){
                if (i!=0){
                    renderBitmapString(-divisorLine, i, std::to_string(i));
                }
                renderBitmapString(i, -divisorLine, std::to_string(i));
            }
        }
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glEnable(GL_BLEND);  // Activar el blending
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // Configurar el blending

    // Mueve la cámara hacia atrás (si quieres)
    // glTranslatef(0.0f, 0.0f, -2.0f);  // opcional, pero redundante si ya colocas la esfera en -2.0

    drawGrid();
    
    drawSquare(0.0f, 0.0f, cellSize);
    drawSquare(1.0f, 0.0f, cellSize);
    drawSquare(2.0f, 0.0f, cellSize);
    drawSquare(3.0f, 0.0f, cellSize);
    drawSquare(4.0f, 0.0f, cellSize);
    drawSquare(5.0f, 0.0f, cellSize);

    drawSquare(0.0f, 4.0f, cellSize);
    drawSquare(1.0f, 4.0f, cellSize);
    drawSquare(2.0f, 4.0f, cellSize);
    drawSquare(3.0f, 4.0f, cellSize);
    drawSquare(4.0f, 4.0f, cellSize);
    drawSquare(5.0f, 4.0f, cellSize);

    drawSquare(0.0f, 1.0f, cellSize);
    drawSquare(0.0f, 2.0f, cellSize);
    drawSquare(0.0f, 3.0f, cellSize);
    drawSquare(0.0f, 4.0f, cellSize);

    drawSquare(5.0f, 1.0f, cellSize);
    drawSquare(5.0f, 2.0f, cellSize);
    drawSquare(5.0f, 3.0f, cellSize);
    drawSquare(5.0f, 4.0f, cellSize);

    drawAgent(ballX, 2.0f);
    drawAgent(ballX, 3.0f);
    
    glDisable(GL_BLEND);  // Desactivar el blending

    glutSwapBuffers(); 
}

void prepareRenderer(){
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    if(true){
        glutCreateWindow("ORCA");
    }else{
        glutCreateWindow("C-NAV");
    }
    
    glClearColor(0.0f, 0.3f, 0.5f, 1.0f);   // Fondo azul oscuro
    // glEnable(GL_DEPTH_TEST);  // habilita el depth test

    glutDisplayFunc(display);
    // glutReshapeFunc(reshape);
    glutIdleFunc(idleFunc);

    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
}

// TO DO: ENABLE DEPTH
// int main(int argc, char** argv) {
//     glutInit(&argc, argv);
//     glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
//     glutInitWindowSize(windowWidth, windowHeight);
//     if(true){
//         glutCreateWindow("ORCA");
//     }else{
//         glutCreateWindow("C-NAV");
//     }
    
//     glClearColor(0.0f, 0.3f, 0.5f, 1.0f);   // Fondo azul oscuro
//     // glEnable(GL_DEPTH_TEST);  // habilita el depth test

//     glutDisplayFunc(display);
//     // glutReshapeFunc(reshape);
//     glutIdleFunc(idleFunc);

//     glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
//     glutMainLoop();

//     return 0;
// }
