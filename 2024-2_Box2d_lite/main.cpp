/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erin Catto makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */
#define _USE_MATH_DEFINES
#define STB_IMAGE_IMPLEMENTATION

#include <string.h>
#include "glut.h"
#include <cmath>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include <iostream>

namespace
{
	Body bodies[200];
	Joint joints[100];

	Body *bomb = NULL;
	const float timeStep = 1.0f / 60.0f; // 1/60초로 고정
	const float oneFrame = timeStep * 1000;
	int iterations = 10;
	Vec2 gravity(0.0f, 0.0f);

	int numBodies = 0;
	int numJoints = 0;
	int demoIndex = 0;

	World world(gravity, iterations);

	Body *selectedBody = nullptr; 
	float mouseX, mouseY;		 
	bool isDragging = false;	

	//GameProperties
	int currentRound;
	bool IsGravityOn = false; // 게임 스타트 여부
	bool isFullScreen = false;
	bool isReady = false;
	enum EGameState{ Stay, Play, Clear, GameOver};
	EGameState gameState = Stay;
	enum ESceneState{ 
		Round1,
		Round2,
		Round3, 
		Round4,
		Round5, 
		ResultScene, 
		GameOverScene
	};
	
	//progress properties
	float xStart = -5; 
	float yStart = 11; 
	float width = 2; 
	float height =2;
	int timer = 5;

	float outerX1 = xStart - 0.1f; 
	float outerY1 = yStart - 0.1f;
	float outerX2 = xStart + (width + 0.1f) * 5; 
	float outerY2 = yStart + height + 0.1; 

	//Texture  //구현 X
	GLuint clearTexture;
	//Window
	int windowWidth = 1280, windowHeight = 720;
	int windowPosX = 100, windowPosY = 100;
	int deathCount;

	//const
	const float Screen_WIDTH = 1920;  // 화면 너비
	const float Screen_HEIGHT = 1080;  // 화면 높이
	const float WORLD_WIDTH = 32.0f;  // world 상수
	const float WORLD_HEIGHT = 18.0f;  // world 상수
	const float WORLD_Y_Half = WORLD_HEIGHT * 0.5f;
	const float WORLD_Y_OFFSET = 3;	// y 오프셋을 상수로 정의
	const float WORLD_ASPECT = WORLD_WIDTH / WORLD_HEIGHT; 
	const int MaxRound = 5;
}



#pragma region gameover font

void drawG(float x, float y, float scale) {

	glBegin(GL_LINE_STRIP);
	glVertex2f(x + scale, y + scale / 2);
	glVertex2f(x + scale, y);
	glVertex2f(x, y);
	glVertex2f(x, y + scale);
	glVertex2f(x + scale, y + scale);  

	glEnd();

	
	glBegin(GL_LINES);
	glVertex2f(x+scale, y + scale / 2);         
	glVertex2f(x + scale / 2, y + scale / 2);
	glEnd();
}

void drawA(float x, float y, float scale) {
	glBegin(GL_LINE_LOOP);
	glVertex2f(x, y);
	glVertex2f(x + scale / 2, y + scale);
	glVertex2f(x + scale, y);
	glEnd();
}

void drawM(float x, float y, float scale) {
	glBegin(GL_LINE_STRIP);
	glVertex2f(x, y);
	glVertex2f(x, y + scale);
	glVertex2f(x + scale / 2, y + scale / 2);
	glVertex2f(x + scale, y + scale);
	glVertex2f(x + scale, y);
	glEnd();
}

void drawE(float x, float y, float scale) {
	glBegin(GL_LINE_STRIP);
	glVertex2f(x + scale, y);
	glVertex2f(x, y);
	glVertex2f(x, y + scale);
	glVertex2f(x + scale, y + scale);
	glEnd();

	glBegin(GL_LINES);
	glVertex2f(x, y + scale / 2);
	glVertex2f(x + scale / 2, y + scale / 2);
	glEnd();
}

void drawO(float x, float y, float scale) {
	glBegin(GL_LINE_LOOP);
	glVertex2f(x, y);
	glVertex2f(x + scale, y);
	glVertex2f(x + scale, y + scale);
	glVertex2f(x, y + scale);
	glEnd();
}

void drawV(float x, float y, float scale) {
	glBegin(GL_LINE_STRIP);
	glVertex2f(x, y + scale);
	glVertex2f(x + scale / 2, y);
	glVertex2f(x + scale, y + scale);
	glEnd();
}

void drawR(float x, float y, float scale) {
	glBegin(GL_LINE_STRIP);
	glVertex2f(x, y);
	glVertex2f(x, y + scale);
	glVertex2f(x + scale / 2, y + scale);
	glVertex2f(x + scale, y + scale / 2);
	glVertex2f(x + scale / 2, y + scale / 2);
	glVertex2f(x + scale, y);
	glEnd();
}

void DrawGameOver(float x, float y, float scale) {
	float gap = scale * 1.5f; 
	drawG(x, y, scale);          
	drawA(x + gap, y, scale);     
	drawM(x + 2 * gap, y, scale); 
	drawE(x + 3 * gap, y, scale); 

	drawO(x, y - gap, scale);         
	drawV(x + gap, y - gap, scale);    
	drawE(x + 2 * gap, y - gap, scale); 
	drawR(x + 3 * gap, y - gap, scale);
}
void DrawGameover() {

	// 좌표계 설정
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, 640.0, 0.0, 480.0); 

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glColor3f(1.0f, 0.0f, 0.0f);


	DrawGameOver(200, 300.0f, 50.0f); 

	glFlush();
}

#pragma endregion

#pragma region Render

void Reshape(int width, int height) //조정
{
	if ((float)width / height != 16.0f / 9.0f) {
		int Scaled_width = height * 16 / 9;
		int Scaled_height = width * 9 / 16;

		if (width > Scaled_width) {
			glutReshapeWindow(Scaled_width, height);
		}
		else {
			
			glutReshapeWindow(width, Scaled_height);
		}
	}
	if (height == 0)
		height = 1;

	float screenAspect = (float)width / (float)height;
	float worldWidth = WORLD_WIDTH;
	float worldHeight = WORLD_HEIGHT;

	// 화면 비율에 따라 월드 크기 조정
	if (screenAspect > WORLD_ASPECT) {

		worldWidth = worldHeight * screenAspect;
	}
	else {

		worldHeight = worldWidth / screenAspect;
	}
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-worldWidth / 2, worldWidth / 2, -worldHeight / 2, worldHeight / 2, -1.0f, 1.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void ToggleFullScreen() { // 전체화면
  
    if (isFullScreen) {
        glutReshapeWindow(windowWidth, windowHeight);
        glutPositionWindow(windowPosX, windowPosY);
        isFullScreen = false;
    } else {
        windowWidth = glutGet(GLUT_WINDOW_WIDTH);
        windowHeight = glutGet(GLUT_WINDOW_HEIGHT);
        windowPosX = glutGet(GLUT_WINDOW_X);
        windowPosY = glutGet(GLUT_WINDOW_Y);

        glutFullScreen();
        isFullScreen = true;
    }

    Reshape(glutGet(GLUT_SCREEN_WIDTH), glutGet(GLUT_SCREEN_HEIGHT));
}

void Frame(int value)
{
		glutPostRedisplay();
		glutTimerFunc(oneFrame, Frame, 0);
}
void StartFrame()
{
		glutTimerFunc(oneFrame, Frame, 0); 
}
void DrawGameLine() {//게임 공간

	if (!isReady) {
		glColor3f(1, 0, 0);
	}
	else {
		glColor3f(0, 1, 0);
	}
	glBegin(GL_LINE_LOOP);
	glVertex2f(12, -3);
	glVertex2f(12, 9);
	glVertex2f(-12, 9);
	glVertex2f(-12, -3);

	glEnd();
}

void DrawBlueLine() { // 도형 공간
	glColor3f(0.5, 0.5, 1);

	glBegin(GL_LINE_LOOP);
	glVertex2f(14, 9.5);
	glVertex2f(-14, 9.5);

	glEnd();
}
void DrawClear() { //  clear시 그려져야 하나 구현 X

	glEnable(GL_TEXTURE_2D); 

	glBindTexture(GL_TEXTURE_2D, clearTexture); 

	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f); glVertex2f(-10.0f, -10.0f); 
	glTexCoord2f(1.0f, 0.0f); glVertex2f(10.0f, -10.0f);  
	glTexCoord2f(1.0f, 1.0f); glVertex2f(10.0f, 10.0f);   
	glTexCoord2f(0.0f, 1.0f); glVertex2f(-10.0f, 10.0f); 
	glEnd();

	glDisable(GL_TEXTURE_2D); 
	glFlush();

}
void DrawText(int x, int y, char *string)
{
	float scaledX = x;
	float scaledY = y ;
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);
	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	
	glColor3f(0.9f, 0.6f, 0.6f);
	glRasterPos2f(scaledX, scaledY);
	
	int len = (int)strlen(string);
	for (int i = 0; i < len; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, string[i]);
	glScalef(5, 5, 5);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);

}


static void DrawBody(Body *body)
{
	Mat22 R(body->rotation);
	Vec2 x = body->position;
	Vec2 h = 0.5f * body->width;
	float r = body->radius;
	Vec2 v1;
	Vec2 v2;
	Vec2 v3;
	Vec2 v4;

	switch (body->shape)
	{
	case 0:
		glColor3f(0.5, 0.5, 1);
		if (body->canDrag == false)
			glColor3f(1,1,1);
		v1 = x + R * Vec2(-h.x, -h.y);
		v2 = x + R * Vec2(h.x, -h.y);
		v3 = x + R * Vec2(h.x, h.y);
		v4 = x + R * Vec2(-h.x, h.y);

		glBegin(GL_QUADS);
		glVertex2f(v1.x, v1.y);
		glVertex2f(v2.x, v2.y);
		glVertex2f(v3.x, v3.y);
		glVertex2f(v4.x, v4.y);
		glEnd();

		glColor3f(0,0,0);
		glBegin(GL_LINE_STRIP);
		glVertex2f(v1.x, v1.y);
		glVertex2f(v2.x, v2.y);
		glVertex2f(v3.x, v3.y);
		glVertex2f(v4.x, v4.y);
		glEnd();
		break;

	case 1:

		glBegin(GL_POLYGON);
		glColor3f(1, 1, 0.5);
		if (body->canDrag == false)
			glColor3f(1, 1, 1);
		for (float i = 0; i < 1; i += 0.02)
		{
			float angle = 2.0f * M_PI * i;	  //  theta
			float v1 = x.x + r * cosf(angle); // x 
			float v2 = x.y + r * sinf(angle); // y 
			glVertex2f(v1, v2);
		}
		glEnd();

		glColor3f(0, 0, 0);
		glBegin(GL_LINE_STRIP);
		for (float i = 0; i < 1; i += 0.02)
		{
			float angle = 2.0f * M_PI * i;	  //  theta
			float v1 = x.x + r * cosf(angle); // x 
			float v2 = x.y + r * sinf(angle); // y 
			glVertex2f(v1, v2);
		}	glEnd();
		break;

	case 2:
		glColor3f(0.5, 1, 1);

		if (body->canDrag == false)
			glColor3f(1, 1, 1);
		v1 = x + R * Vec2(-h.x, -h.y);
		v2 = x + R * Vec2(h.x, -h.y);
		v3 = x + R * Vec2(0, h.y);
		glBegin(GL_POLYGON);
		glVertex2f(v1.x, v1.y);
		glVertex2f(v2.x, v2.y);
		glVertex2f(v3.x, v3.y);
		glEnd();

		glColor3f(0, 0, 0);
		glBegin(GL_LINE_STRIP);
		glVertex2f(v1.x, v1.y);
		glVertex2f(v2.x, v2.y);
		glVertex2f(v3.x, v3.y);
		glEnd();
		break;
	}
}
void DrawJoint(Joint *joint)
{
	Body *b1 = joint->body1;
	Body *b2 = joint->body2;

	Mat22 R1(b1->rotation);
	Mat22 R2(b2->rotation);

	Vec2 x1 = b1->position;
	Vec2 p1 = x1 + R1 * joint->localAnchor1;

	Vec2 x2 = b2->position;
	Vec2 p2 = x2 + R2 * joint->localAnchor2;

	glColor3f(0.5f, 0.5f, 0.8f);
	glBegin(GL_LINES);
	glVertex2f(x1.x, x1.y);
	glVertex2f(p1.x, p1.y);
	glVertex2f(x2.x, x2.y);
	glVertex2f(p2.x, p2.y);
	glEnd();
}

//void LaunchBomb()
//{
//	if (!bomb)
//	{
//		bomb = bodies + numBodies;
//		bomb->TriangleSet(Vec2(1.0f, 1.0f), 50.0f);
//		bomb->friction = 0.2f;
//		world.Add(bomb);
//		++numBodies;
//	}
//
//	bomb->position.Set(Random(-15.0f, 15.0f), 15.0f);
//	bomb->rotation = Random(-1.5f, 1.5f);
//	bomb->velocity = -1.5f * bomb->position;
//	bomb->angularVelocity = Random(-20.0f, 20.0f);
//}
#pragma endregion

#pragma region GameScenes
void Round1Scene(Body *b, Joint *j)
{
	b->BoxSet(Vec2(0.4f, 0.4f), FLT_MAX);
	b->position.Set(0, 0 );
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(5, 0.5f), 100);
	b->position.Set(0.0f, 0.5);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 1), 100);
	b->position.Set(-5, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 1), 100);
	b->position.Set(-2, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 1), 100);
	b->position.Set(2, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 1), 100);
	b->position.Set(5, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->CircleSet(Vec2(1, 1), 100);
	b->position.Set(-7, 12);
	world.Add(b);
	++b;
	++numBodies;


	b->CircleSet(Vec2(1, 1), 100);
	b->position.Set(-0, 12);
	world.Add(b);
	++b;
	++numBodies;

}
void Round2Scene(Body* b, Joint* j)
{
	b->BoxSet(Vec2(0.2, 0.2), FLT_MAX);
	b->position.Set(2, 0);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(0.2, 0.2), FLT_MAX);
	b->position.Set(-2, 0);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->TriangleSet(Vec2(2, 2), 100);
	b->position.Set(-2, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->TriangleSet(Vec2(2, 2), 100);
	b->position.Set(1, 12);
	world.Add(b);
	++b;
	++numBodies;



}
void Round3Scene(Body *b, Joint *j)
{
	b->BoxSet(Vec2(0.2f, 0.2f), FLT_MAX);
	b->position.Set(0, 0);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(5, 0.5f), 100);
	b->position.Set(0.0f, 0.5);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 5), 100);
	b->position.Set(0.0f, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 5), 100);
	b->position.Set(-4, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->CircleSet(Vec2(1, 1), 100);
	b->position.Set(-2, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->CircleSet(Vec2(1, 1), 100);
	b->position.Set(2, 12);
	world.Add(b);
	++b;
	++numBodies;

}
void Round4Scene(Body *b, Joint *j)
{
	b->BoxSet(Vec2(0.2f, 0.2f), FLT_MAX);
	b->position.Set(-2, 0);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(3, 0.5f), 100);
	b->position.Set(-2, 0.5);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(0.2f, 0.2f), FLT_MAX);
	b->position.Set(2, 0);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(3, 0.5f), 100);
	b->position.Set(2, 0.5);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 1), 100);
	b->position.Set(7, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 1), 100);
	b->position.Set(5, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 1), 100);
	b->position.Set(2, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 1), 100);
	b->position.Set(0, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->CircleSet(Vec2(1, 1), 100);
	b->position.Set(-2, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->CircleSet(Vec2(1, 1), 100);
	b->position.Set(-5, 12);
	world.Add(b);
	++b;
	++numBodies;



}
void Round5Scene(Body* b, Joint* j)
{
	b->BoxSet(Vec2(0.2f, 0.2f), FLT_MAX);
	b->position.Set(0, 0);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(10, 0.5f), 100);
	b->position.Set(0.0f, 0.5);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(4, 0.5f), 100);
	b->position.Set(2, 11);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(3, 0.5f), 100);
	b->position.Set(3, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(2, 0.5f), 100);
	b->position.Set(4, 13);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 0.5f), 100);
	b->position.Set(-1, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 3), 100);
	b->position.Set(-3, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->BoxSet(Vec2(1, 3), 100);
	b->position.Set(-5, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->CircleSet(Vec2(1, 3), 100);
	b->position.Set(-7, 12);
	world.Add(b);
	++b;
	++numBodies;

	b->CircleSet(Vec2(2, 2), 100);
	b->position.Set(0, 2);
	b->canDrag = false;
	world.Add(b);
	++b;
	++numBodies;


}
void Clear_Scene(Body *b, Joint *j)
{
	
}
void GameOver_Scene(Body *b, Joint *j)
{
	
}
#pragma endregion

#pragma region GameLogic

void (*demos[])(Body* b, Joint* j) = {
	Round1Scene,
	Round2Scene,
	Round3Scene,
	Round4Scene,
	Round5Scene,
	Clear_Scene,
	GameOver_Scene
};

void InitDemo(int index)
{
	world.Clear();
	numBodies = 0;
	numJoints = 0;
	bomb = NULL;
	
	for (int i = 0; i < 200; ++i)
	{
		bodies[i].canDrag = true; // 드래그 속성 초기화
	}
	demoIndex = index;
	demos[index](bodies, joints);
}

void ChangeGravity() {
	if (IsGravityOn) {
		world.gravity.y = -10.0f;  // 중력 켜기
	}
	else {
		world.gravity.y = 0.0f;    // 중력 끄기
	}
}

void CheckGameReady() {
	isReady = true; 
	for (int i = 0; i < numBodies; ++i) {
		Body& b = bodies[i]; 

		if (b.position.y < -2 || b.position.y > 10 || b.position.x < -12 || b.position.x > 12) {
			isReady = false;
			break; 
		}
	}
}
void CheckGameOver()
{
	for each (Body b in bodies)
	{
		if (b.position.y < -8)
		{
			gameState = GameOver;
			InitDemo(GameOverScene);
			ChangeGravity();
			glutPostRedisplay();
			break;
		}
	}
}

void NextRound() {
	IsGravityOn = !IsGravityOn;
	ChangeGravity();
	currentRound++;
	InitDemo(currentRound);
	if (currentRound == ResultScene) { gameState = Clear; return; }
	gameState = Stay;
}

void RestartRound(int round) {
	gameState = Stay;
	IsGravityOn = false;
	ChangeGravity();
	InitDemo(round);
	Reshape(glutGet(GLUT_SCREEN_WIDTH), glutGet(GLUT_SCREEN_HEIGHT));
	glutPostRedisplay(); 
	isReady = false;
	DrawGameLine();
	
}

void OneTimer(int value) {
	if (gameState != Play) return;
	timer--; 
	glutTimerFunc(1000, OneTimer, 0); 
	
}
void PlayTimeCount() {
	OneTimer(1);
	timer = 0;
}
void ProgressBar() {


	if (timer == 0) { 
		NextRound();  
		return;
	}
	float localXStart = xStart;

	
	glBegin(GL_LINE_LOOP); 
	glVertex2f(outerX1, outerY1); 
	glVertex2f(outerX2, outerY1); 
	glVertex2f(outerX2, outerY2); 
	glVertex2f(outerX1, outerY2); 
	glEnd();

	for (int i = 0; i < timer; i++) {
		glColor3f(0.6, 1, 1); 
		glBegin(GL_QUADS); 
		glVertex2f(localXStart, yStart);                   
		glVertex2f(localXStart + width, yStart);           
		glVertex2f(localXStart + width, yStart + height); 
		glVertex2f(localXStart, yStart + height);         
		glEnd();
		localXStart += width + 0.1f;
	}

	glFlush(); 

}
void UserInterface() {
	char buffer[64];
	switch (gameState)
	{
	case Stay:
	case Play:

		sprintf(buffer, "Round %d", currentRound+1); 
		DrawText(5, 15, buffer);

	
		sprintf(buffer, "(S)tart Round %s", IsGravityOn ? "ON" : "OFF");
		DrawText(5, 50, buffer);

		sprintf(buffer, "(F)ull Screen%s", isFullScreen ? "ON" : "OFF");
		DrawText(5, 80, buffer);

		sprintf(buffer, isReady ? "Ready" : "Stay");
		DrawText(5, 110, buffer);
		break;
	case GameOver:
		sprintf(buffer, "(R)estart Pre Round ");
		DrawText(5, 130, buffer);
		sprintf(buffer, "(G)o To Round 1");
		DrawText(5, 150, buffer);
		sprintf(buffer, "DeathCount %d", deathCount);
		DrawText(5, 170, buffer);
		break;
	case Clear : 
		sprintf(buffer, "!!! Congraturation !!!");
		DrawText(glutGet(GLUT_SCREEN_WIDTH)/2 - 100, glutGet(GLUT_SCREEN_HEIGHT) / 2-100, buffer);
		sprintf(buffer, "DeathCount %d", deathCount);
		DrawText(glutGet(GLUT_SCREEN_WIDTH) / 2 - 70, glutGet(GLUT_SCREEN_HEIGHT) / 2 - 50, buffer);
		sprintf(buffer, "(G)o To Round 1");
		DrawText(glutGet(GLUT_SCREEN_WIDTH) / 2 - 70, glutGet(GLUT_SCREEN_HEIGHT) / 2, buffer);
		break;
	}

}

#pragma endregion

#pragma region GameLoop

void SimulationLoop()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	UserInterface();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0f, -WORLD_Y_Half + WORLD_Y_OFFSET, 0);

	world.Step(timeStep, selectedBody);

	switch (gameState) {
	case Play:
		ProgressBar();
		DrawGameLine();
		CheckGameOver();
		break;
	case Stay:
		DrawBlueLine();
		DrawGameLine();
		break;
	case GameOver:
		DrawGameover();
		break;
	case Clear:
			//DrawClear();
			break;
	}
	
	for (int i = 0; i < numBodies; ++i)
		DrawBody(bodies + i);

	for (int i = 0; i < numJoints; ++i)
		DrawJoint(joints + i);

	glutSwapBuffers();
}

#pragma endregion

#pragma region Input

void Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
		break;
	/*case '8':					//라운드 확인용
		gameState = Clear;
		InitDemo(ResultScene);
		break;*/
	//case '1':
	//case '2':
	//case '3':
	//case '4':
	//case '5':
	//case '6':
	//case '7':
	//	InitDemo(key - '1');
	//	glutPostRedisplay();
	//	break;
	case 's':
		if (isDragging || !isReady || gameState != Stay) return;
		
		gameState = Play;
		IsGravityOn = !IsGravityOn;
			ChangeGravity();
			PlayTimeCount();
			timer = 5;
            break;
	case 'f':
		ToggleFullScreen();
		break;
	case 'r':
		deathCount++;
		RestartRound(currentRound);
		break;
	case 'g':
		if (gameState == GameOver || gameState==Clear)
		{
			RestartRound(Round1);
			deathCount = 0;
			currentRound = 0;
		}
		break;
	}
}

float ScreenToWorldX(int x)
{
    int width = glutGet(GLUT_WINDOW_WIDTH);
	float worldWidth = WORLD_WIDTH;
    return (x / (float)width) * worldWidth - worldWidth/2;
}

float ScreenToWorldY(int y)
{
    int height = glutGet(GLUT_WINDOW_HEIGHT);
    float worldHeight = WORLD_HEIGHT ;  
    return (worldHeight/2 - (y / (float)height) * worldHeight) + WORLD_Y_Half - WORLD_Y_OFFSET;
}


void MouseInput(int button, int state, int x, int y)
{
	if (IsGravityOn) return;
	mouseX = ScreenToWorldX(x);
	mouseY = ScreenToWorldY(y);

	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{

		for (int i = 0; i < numBodies; ++i)
		{
			Body *body = &bodies[i];
			Vec2 halfSize = body->width * 0.5f;

			//마우스 감지
			if (mouseX >= body->position.x - halfSize.x &&
				mouseX <= body->position.x + halfSize.x &&
				mouseY >= body->position.y - halfSize.y &&
				mouseY <= body->position.y + halfSize.y)
			{

				selectedBody = body;
				if(selectedBody->canDrag == false) break; // 고정 물체 제외
				selectedBody->angularVelocity = 0;
				selectedBody->velocity = Vec2(0, 0);
				isDragging = true;
				break;
			}
		}
	}

	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		isDragging = false;
		selectedBody = nullptr;
	}
}
void MouseDrag(int x, int y)
{
	if (IsGravityOn) return;
	if (isDragging && selectedBody)
	{

		mouseX = ScreenToWorldX(x);
		mouseY = ScreenToWorldY(y);

		selectedBody->position.Set(mouseX, mouseY);
		CheckGameReady();
	}
}
#pragma endregion

int main(int argc, char** argv)
{
	InitDemo(Round1);
	IsGravityOn = false;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(Screen_WIDTH, Screen_HEIGHT);
	glutCreateWindow("Balance Challenge Game");
	glutReshapeFunc(Reshape);
	glutDisplayFunc(SimulationLoop);
	glutKeyboardFunc(Keyboard);
	glutMouseFunc(MouseInput);
	glutMotionFunc(MouseDrag);
	StartFrame();
	glutMainLoop();
	
	return 0;
}
