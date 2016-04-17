#include <cstdlib>
#include <cstdio>
#include <iostream>
#include "cloth.h"

#ifdef __APPLE__
#include <glut.h>
#else
#include <GL/glut.h>
#endif
#include <glm/glm.hpp>

const int HEIGHT = 600, WIDTH = 800;
bool start = false;

// current rotation angle
static float h_angle = 0.f;
static float v_angle = 0.f;
static GLint prev_time = 0;
const int timestep = 2;
int time_accumulator = 0;
glm::vec3 position = glm::vec3(0.f, 1.f, 2.f);

Cloth cloth(40, 40, 0.8, 0.8);

static int rendered = 0;
//--------------------------------------------------------------------------------
void on_idle()
{
  rendered += 1;
  if (time_accumulator > timestep) {
    if (start) {
      for (; time_accumulator > 0; time_accumulator -= timestep) {
        cloth.timestep(float(timestep) / 1000.f);
      }
    }
    time_accumulator = 0;
  }
}

//----------------------------------------------------------------------------
void reshape(int width, int height)
{
	const double aspectRatio = (float) width / height, fieldOfView = 45.0;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fieldOfView, aspectRatio,
		1.0, 1000.0);  /* Znear and Zfar */
	glViewport(0, 0, width, height);
}

//----------------------------------------------------------------------------
void do_motion (void)
{
	static GLint prev_fps_time = 0;
	static int frames = 0;

	int time = glutGet(GLUT_ELAPSED_TIME);
  time_accumulator += (time - prev_time);
	prev_time = time;

	frames += 1;
  /* update every seconds */
	if ((time - prev_fps_time) > 1000) {
    int current_fps = frames * 1000 / (time - prev_fps_time);
    printf("%d fps %d\n", current_fps, rendered);
    frames = 0;
    rendered = 0;
    prev_fps_time = time;
  }

	glutPostRedisplay();
}

//----------------------------------------------------------------------------
void draw_grid()
{
  const int GRID_SIZE = 10;
	glBegin(GL_LINES);
	glColor3f(0.5f, 0.5f, 0.5f);
	for(int i = -GRID_SIZE; i <= GRID_SIZE; i++) {
		glVertex3f((float)i, 0, (float)-GRID_SIZE);
		glVertex3f((float)i, 0, (float) GRID_SIZE);

		glVertex3f((float)-GRID_SIZE, 0, (float)i);
		glVertex3f((float)GRID_SIZE, 0, (float)i);
	}
	glEnd();
}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

  //glm::vec3 lookat = position - glm::vec3(
  //    cos(v_angle) * sin(h_angle),
  //    sin(v_angle),
  //    cos(v_angle) * cos(h_angle));

	//gluLookAt(
  //    position.x, position.y, position.z,
  //    lookat.x, lookat.y, lookat.z,
  //    0.f, 1.f, 0.f);
  
  gluLookAt(
      position.x, position.y, position.z,
      0.f, 0.f, 0.f,
      0.f, 1.f, 0.f);

  glRotatef(h_angle, 0.f, 1.f, 0.f);

  draw_grid();
  cloth.draw();

	glutSwapBuffers();

	do_motion();
}

//--------------------------------------------------------------------------------
int mouse_button;
void mouse_func(int button, int state, int x, int y)
{
  if (state == GLUT_DOWN) {
    mouse_button = button;
  }
}

void passive_motion_callback(int x, int y)
{
  int width = glutGet(GLUT_WINDOW_WIDTH);
  int height = glutGet(GLUT_WINDOW_HEIGHT);

  int cx = (width>>1);
  int cy = (height>>1);

  if (cx != x || cy != y) {
    glutWarpPointer(cx, cy);
  }

  if (abs(x-cx) > 10 || abs(y-cy) > 10) return;

  switch (mouse_button) {
    case GLUT_LEFT_BUTTON: 
    {
      h_angle += (x-cx) * .002f;
      v_angle += (y-cy) * .002f;
      break;
    }
    case GLUT_RIGHT_BUTTON:
    {
      break;
    }
    default:
    {
      break;
    }
  }
}

void keyboard_func(unsigned char key, int x, int y)
{
  switch (key) {
    case 'a':
      h_angle += 1;
      break;
    case 'd':
      h_angle -= 1;
      break;
    default:
      start = true;
      break;
  }
  /*
  glm::vec3 direction(
      cos(v_angle) * sin(h_angle),
      sin(v_angle),
      cos(v_angle) * cos(h_angle));

  glm::vec3 right(
      sin(h_angle - 3.14f/2.0f),
      0,
      cos(h_angle - 3.14f/2.0f));

  direction = glm::normalize(direction);
  right = glm::normalize(right);

  switch (key) {
    case 'w':
      position -= 0.05f * direction;
      break;
    case 's':
      position += 0.05f * direction;
      break;
    case 'd':
      position -= 0.05f * right;
      break;
    case 'a':
      position += 0.05f * right;
      break;
    default:
      start = true;
      break;
  }
  */
}

void init()
{
  cloth.add_wind(glm::vec3(-.01f, .001f, .01f));
}

//----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  init();
	glutInitWindowSize(WIDTH, HEIGHT);
	glutInitWindowPosition(100, 100);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInit(&argc, argv);

	glutCreateWindow("sample");
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
  glutIdleFunc(on_idle);

	glutMotionFunc(passive_motion_callback);
	glutMouseFunc(mouse_func);
  glutKeyboardFunc(keyboard_func);

	glClearColor(0.9f, 0.9f, 0.9f, 1.f);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT0);
	GLfloat lightPos0[4] = {0.0, 5.0, -4, 0.0};
	GLfloat lightAmbient[] = {.6, .6, .6, 1.0};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightAmbient);

	glEnable(GL_DEPTH_TEST);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	glEnable(GL_NORMALIZE);

	glEnable(GL_COLOR_MATERIAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  glShadeModel(GL_SMOOTH);

	/* XXX docs say all polygons are emitted CCW, but tests show that some aren't. */
	if(getenv("MODEL_IS_BROKEN"))  
		glFrontFace(GL_CW);

	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

	glutGet(GLUT_ELAPSED_TIME);
	glutMainLoop();

  return 0;
}

