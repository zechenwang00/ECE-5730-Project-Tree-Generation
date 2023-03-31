/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * Barnsley Fern calculation and visualization
 * Uses PIO-assembly VGA driver
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0 and 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */
//#include <u.h>
//#include <libc.h>
#include <math.h>
#include <String.h>
#include "vga_graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

// Fixed point data type
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)(((( signed long long)(a))*(( signed long long)(b)))>>15)) 
#define float2fix15(a) ((fix15)((a)*32768.0f)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0f) 
#define int2fix15(a) ((a)<<15)
#define PI 3.1415926

typedef struct Lsystem Lsystem;
typedef struct Rule Rule;
typedef struct State State;

struct Lsystem
{
	char*	name;
	char*	axiom;
    //pointer to the first rule
	Rule*	rules;
	int	linelen;
	double	initangle;
	double	leftangle;
	double	rightangle;
};

struct Rule
{
	char	pred;
	char*	succ;
	Rule*	next;
};

struct State
{
	int	x;
	int	y;
	double	angle;
	State*	prev;
};

//void* emalloc(ulong size);
//global variables
Lsystem	*ls;
// state at top of stack
State	*state;
int 	x;
int 	y;
double 	angle;
char 	*curgen;

void
pushstate(void)
{
	State *s;
	//s = emalloc(sizeof(State));
	s->x = x;
	s->y = y;
	s->angle = angle;
	s->prev = state;
	state = s;
}

void
popstate(void)
{
	State *s;

	s = state;
	x = s->x;
	y = s->y;
	angle = s->angle;
	state = state->prev;
	free(s);
}

//search in the rule successively for the substitute for char c
char*
production(char c)
{
	Rule *r;
    //start from the pointer to the first rule (ls->rules)
	for(r = ls->rules; r!=NULL; r=r->next)
		if(r->pred == c)
			return r->succ;
	return NULL;
}

//update current generation(the string s) and curgen 
void
nextgen(int iteration)
{
	char *s;
	char *p, *q;
    //s = s_new();
	s = malloc(100);
	//int num = 0;
	p = curgen;
	for (int i = 0;i<=4;i++){
		setTextColor(WHITE);
	    setTextSize(1);
	    setCursor(10,200+20*i);
	    //num++;
	    writeString(p[i]);
	}
    
	for(p = curgen; p!=NULL; p++){
		//p[i]
		q = production(*p);
		if(q)
			strcat(s,q);
		else
			strcat(s,*p);
	}
	//s_terminate(s);
	//free(curgen);
	//update current generation 
	curgen = s;
	free(s);
}

void
forward(void)
{
	int x1, y1;
	x1 = x + ls->linelen*cos(angle * (PI/180.0));
	y1 = y + ls->linelen*sin(angle * (PI/180.0));
	drawLine((short) x, (short) y, (short) x1, (short) y1, GREEN);
	x = x1;
	y = y1;
}

void
rotate(double angle_delta)
{
	angle += angle_delta;
	if(angle >= 360.0)
		angle -= 360.0;
	if(angle <= 0.0)
		angle += 360.0;
}


int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;
    
    //write rules
    Rule *r1;
    r1->pred = 'X';
    r1->succ = 'F+[[X]-X]-F[-FX]+X';
    Rule *r2;
    r2->pred = 'F';
    r2->succ = 'FF';
    r1->next = r2;
    //initialize L system
    ls->axiom = "XFFFFF";
    ls->linelen = 5;
    ls->initangle = -30;
    ls->leftangle = 25;
    ls->rightangle = -25;
    ls->rules = r1;
    //generate command string curgen
    //initialize curgen (axiom X)
    curgen = ls->axiom;
    int iteration = 1000;
    for (int i = 0; i<iteration; i++){
		// setTextColor(WHITE);
		// setTextSize(1);
		// setCursor(10,20*i+20);
		// writeString(curgen);
		nextgen(i);
    }
    //draw the picture 
    char *s;
	x = 100;
	y = 380;
	angle = ls->initangle;
	for(s = curgen; *s; s++){
		switch(*s){
		case 'X':
            break;
        case 'F':
			forward();
			break;
		case '-':
			rotate(ls->leftangle);
			break;
		case '+':
			rotate(ls->rightangle);
			break;
		case '[':
			pushstate();
			break;
		case ']':
			popstate();
			break;
		}
	}
}
