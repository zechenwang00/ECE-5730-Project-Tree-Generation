/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
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
 *  - DMA channels 0, 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
#define sqrtfix(a) (float2fix15(sqrt(fix2float15(a))))
#define PI 3.1415926

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// Stuff for Barnsley fern ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Maximum number of iterations
#define max_count 1000
fix15 point_x[max_count];
fix15 point_y[max_count];
volatile bool sync = false;

// State transition equations
fix15 f1y_coeff_1 = float2fix15(0.16) ; ;
#define F1x(a,b) 0 ;
#define F1y(a,b) ((fix15)(multfix15(a,f1y_coeff_1)))

fix15 f2x_coeff_1 = float2fix15(0.82) ;
fix15 f2x_coeff_2 = float2fix15(0.08) ;
#define F2x(a,b) ((fix15)(multfix15(f2x_coeff_1,a) + multfix15(f2x_coeff_2,b)))
fix15 f2y_coeff_1 = float2fix15(-0.08) ;
fix15 f2y_coeff_2 = float2fix15(0.85) ;
fix15 f2y_coeff_3 = float2fix15(1.6) ;
#define F2y(a,b) ((fix15)(multfix15(f2y_coeff_1,a) + multfix15(f2y_coeff_2,b) + f2y_coeff_3))

fix15 f3x_coeff_1 = float2fix15(0.2) ;
fix15 f3x_coeff_2 = float2fix15(0.26) ;
#define F3x(a,b) ((fix15)(multfix15(f3x_coeff_1,a) - multfix15(f3x_coeff_2,b)))
fix15 f3y_coeff_1 = float2fix15(0.23) ;
fix15 f3y_coeff_2 = float2fix15(0.22) ;
fix15 f3y_coeff_3 = float2fix15(1.6) ;
#define F3y(a,b) ((fix15)(multfix15(f3y_coeff_1,a) + multfix15(f3y_coeff_2,b) + f3y_coeff_3))

fix15 f4x_coeff_1 = float2fix15(-0.15) ;
fix15 f4x_coeff_2 = float2fix15(0.28) ;
#define F4x(a,b) ((fix15)(multfix15(f4x_coeff_1,a) + multfix15(f4x_coeff_2,b)))
fix15 f4y_coeff_1 = float2fix15(0.26) ;
fix15 f4y_coeff_2 = float2fix15(0.24) ;
fix15 f4y_coeff_3 = float2fix15(0.44) ;
#define F4y(a,b) ((fix15)(multfix15(f4y_coeff_1,a) + multfix15(f4y_coeff_2,b) + f4y_coeff_3))

// Probability thresholds (rearrange for faster execution, check lowest range last)
#define F1_THRESH 21474835
#define F2_THRESH 1846835936
#define F3_THRESH 1997159792

fix15 vga_scale = float2fix15(30) ;

//////////////////////////////////////////////////////////////////////////////////
////////////////////////////// L-System //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
typedef struct Lsystem Lsystem;
typedef struct Rule Rule;
typedef struct State State;

struct Lsystem
{
	char* name;
	char* axiom;
	//pointer to the first rule
	Rule* rules;
	int	linelen;
	float	initangle;
	float	leftangle;
	float	rightangle;
};

struct Rule
{
    char  pred;
	char* succ;
	Rule* next;
};

struct State
{
	int	x;
	int	y;
	float	angle;
	State* prev;
}Sta;

//void* emalloc(ulong size);
//global variables
struct Lsystem lsystem;
Lsystem* ls = &lsystem;

// state point to the State at top of stack
State* state = &Sta;

//current angle and position
int 	x_cur;
int 	y_cur;
float 	angle_cur;
char *curgen;

void pushstate(void)
{
	//struct State s;
	State* ptr_s = (State*)malloc(sizeof(State));
	ptr_s->x = x_cur;
	ptr_s->y = y_cur;
	ptr_s->angle = angle_cur;
	ptr_s->prev = state;
	state = ptr_s;
}

void popstate(void)
{
	struct State s;
	State* ptr_s = &s;
	ptr_s = state;
	x_cur = ptr_s->x;
	y_cur = ptr_s->y;
	angle_cur = ptr_s->angle;
	state = state->prev;
	free(ptr_s);
}

//search in the rule successively for the substitute for char c
char* production(char c)
{
	Rule* r;
	//start from the pointer to the first rule (ls->rules)
	for (r = ls->rules; r != NULL; r = r->next)
		if (r->pred == c)
			return r->succ;
	return NULL;
}

//update current generation(the string s) and curgen 
void nextgen()
{
	char* s;
	char* p, * q;
	s = (char*)malloc(10000);
	s[0] = '\0';
	//int num = 0;
	p = curgen;
	for (p = curgen; *p != '\0'; p++) {
		//q is char* or NULL
		q = production(*p);
		if (q)
			strcat(s, q);
		//q is NULL 
		else {
			size_t len = strlen(s);
			s[len] = *p;
			s[len + 1] = '\0';
		}		
	}
	//update current generation 
	strcpy(curgen, s);
	free(s);
}

void forward(char color)
{
	int x1, y1;
	x1 = x_cur + ls->linelen * cos(angle_cur * (PI / 180.0));
	y1 = y_cur + ls->linelen * sin(angle_cur * (PI / 180.0));
	drawLine((short)x_cur, (short)y_cur, (short)x1, (short)y1, color);
	x_cur = x1;
	y_cur = y1;
}

void rotate(float angle_delta)
{
	angle_cur += angle_delta;
	if (angle_cur >= 360.0)
		angle_cur -= 360.0;
	if (angle_cur <= 0.0)
		angle_cur += 360.0;
}




// ==================================================
// === users audio input thread
// ==================================================
//FFT thread
static PT_THREAD (protothread_fft(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // non-blocking write
      while(1) {
       
        
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// thread for L-System trees
static PT_THREAD (protothread_lsys(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

	//initialize rules
	struct Rule r1;
	struct Rule r2;
	Rule *ptr_r2 = &r2;
	Rule *ptr_r1 = &r1;
	// ptr_r1->pred = 'X';
	// ptr_r1->succ = "F + [[X]- X] - F[-FX] + X";
	// ptr_r1->succ = "F-[[X]+X]+F[+FX]-X";
	// ptr_r2->pred = 'F';
	// ptr_r2->succ = "FF";
	// ptr_r2->next = NULL;
	// ptr_r1->next = ptr_r2;
	// //initialize L system
	// //ls is the pointer to struct Lsystem lsystem
	// ls->axiom = "X";
	// ls->linelen = 5;
	// ls->initangle = -90;
	// ls->leftangle = 30;
	// ls->rightangle = -30;
	// ls->rules = ptr_r1;
	ptr_r1->pred = 'F';
	//ptr_r1->succ = "F + [[X]- X] - F[-FX] + X";
	ptr_r1->succ = "F[+F]F[-F]F";
	ptr_r1->next = NULL;
	//initialize L system
	//ls is the pointer to struct Lsystem lsystem
	ls->axiom = "F";
	ls->linelen = 3;
	ls->initangle = -90;
	ls->leftangle = -30;
	ls->rightangle = 30;
	ls->rules = ptr_r1;
    char color_ls = 2;
    int iteration = 4;
    while(1) {
        //initialize position
	    x_cur = 320;
	    y_cur = 480;
	    angle_cur = ls->initangle;
	    //initialize state
	    state->x = x_cur;
	    state->y = y_cur;
	    state->angle = angle_cur;
	    state->prev = NULL;
        //generate command string curgen
        //initialize curgen on heap (axiom X)
        curgen = (char*)malloc(10000);
        strcpy(curgen,ls->axiom);
        for (int i = 0; i < iteration; i++) {
            printf("iteration%d,curgen=%s\n", i, curgen);
            nextgen();
        }
        for (char* s = curgen; *s!='\0'; s++) {
            switch (*s) {
            case 'X':
                break;
            case 'F':
                forward(color_ls);
                sleep_ms(5);
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
        free(curgen);
        //update parameters: color length rules angle 
        //rules:
        int rule_num = rand()%3;
        //tree type 
        //d
        if(rule_num ==0){
            iteration = 6;
            ls->initangle = -90;
            ls->linelen = rand()%2+2;
            ls->axiom = "X";
            ptr_r1->pred = 'X';
	        ptr_r1->succ = "F[+X]F[-X]+X";
	        ptr_r2->pred = 'F';
	        ptr_r2->succ = "FF";
	        ptr_r2->next = NULL;
	        ptr_r1->next = ptr_r2;
        }
        //e
        else if(rule_num ==1){
            iteration = 6;
            ls->initangle = -90;
            ls->linelen = rand()%3+2;
            ls->axiom = "X";
            ptr_r1->pred = 'X';
	        ptr_r1->succ = "F[+X][-X]FX";
	        ptr_r2->pred = 'F';
	        ptr_r2->succ = "FF";
	        ptr_r2->next = NULL;
	        ptr_r1->next = ptr_r2;
        }
        //a
        else{
            iteration = 4;
            ls->initangle = -90;
            ls->linelen = rand()%2+3;
            ls->axiom = "F";
            ptr_r1->pred = 'F';
	        ptr_r1->succ = "F[+F]F[-F]F";
	        ptr_r1->next = NULL;
        }
	    ls->leftangle = -rand()%11-25;
	    ls->rightangle = rand()%11+25;
        color_ls = rand()%7+1;
        if(color_ls==4){
            color_ls = 3;
        }
        sync = true;
        sleep_ms(2000);
        fillRect(0,0,640,480,BLACK);
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// thread for barnsley fern 
static PT_THREAD (protothread_fern(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);
    fix15 x_old ;
    fix15 y_old ;
    fix15 x_new ;
    fix15 y_new ;

    fix15 scaled_x ;
    fix15 scaled_y ;
    
    x_old = 0 ;
    y_old = 0 ;

    uint32_t start_time ;
    uint32_t end_time ;

    start_time = time_us_32() ;
    fix15 scaled_x_left[max_count];
    fix15 scaled_y_left[max_count];
    fix15 scaled_x_right[max_count];
    fix15 scaled_y_right[max_count];

    // start generating trees
    int num_trees = 2;
    int tree_x = 160;
    while(1) {
        for(int i = 0; i < num_trees; i++){
        // randomize F2 function to generate different leaves
        f2x_coeff_1 = float2fix15((float)(rand() % 30 + 60) / 100.0);   //  0.60 - 0.90
        f2x_coeff_2 = float2fix15((float)(rand() % 30 - 15) / 100.0);   // -0.15 - 0.15
        f2y_coeff_1 = float2fix15((float)(rand() % 30 - 15) / 100.0);   // -0.15 - 0.15
        f2y_coeff_3 = float2fix15((float)(rand() % 100 + 100) / 100.0); //  1.00 - 2.00
        int vga_scale_int = rand() % 20 + 20;
        vga_scale = int2fix15(vga_scale_int);

        //generate left leaves and right leaves model
        for (int i=0; i<max_count; i++) {

            int test = rand() ;

            if (test<F1_THRESH) {
                x_new = F1x(x_old, y_old) ;
                y_new = F1y(x_old, y_old) ;
            }
            else if (test<F2_THRESH) {
                x_new = F2x(x_old, y_old) ;
                y_new = F2y(x_old, y_old) ;
            }
            else if (test<F3_THRESH) {
                x_new = F3x(x_old, y_old) ;
                y_new = F3y(x_old, y_old) ;
            }
            else {
                x_new = F4x(x_old, y_old) ;
                y_new = F4y(x_old, y_old) ;
            }

            scaled_x = multfix15(vga_scale, x_new) ;
            scaled_y = multfix15(vga_scale, y_new) ;
            
            //left leaves scaled points
            scaled_x_left[i] = multfix15(vga_scale, F3x(x_new, y_new)) ;
            scaled_y_left[i] = multfix15(vga_scale, F3y(x_new,y_new)) ;
            scaled_x_right[i] = multfix15(vga_scale, F4x(x_new, y_new)) ;
            scaled_y_right[i] = multfix15(vga_scale, F4y(x_new,y_new)) ;
            
            x_old = x_new ;
            y_old = y_new ;
        }

        int max_l = 20;
        float scale_current = 1;
        float scale_factor = 0.8;
        int x_offset = 0;
        int x_offset_increment = 10;
        x_offset_increment = rand() % 20 - 10;
        int y_offset = 460;
        int y_offset_increment = 80;
        y_offset_increment = vga_scale_int * 2;
        fix15 x_shrinked_left;
        fix15 x_shrinked_right;
        fix15 y_shrinked_left;
        fix15 y_shrinked_right;
        //draw leaves on the tree
        for (int leaf = 0; leaf<max_l; leaf++) {
            //draw each pixel on two leaves
            for (int i=0; i<max_count; i++) {
                // left leaf
                x_shrinked_left  = multfix15(scaled_x_left[i], float2fix15(scale_current));
                y_shrinked_left  = multfix15(scaled_y_left[i], float2fix15(scale_current));
                // right leaf
                x_shrinked_right = multfix15(scaled_x_right[i], float2fix15(scale_current));
                y_shrinked_right = multfix15(scaled_y_right[i], float2fix15(scale_current));
                // draw both leaves
                drawPixel((x_shrinked_left >>15) + x_offset + tree_x, y_offset-(y_shrinked_left >>15), GREEN) ;
                drawPixel((x_shrinked_right>>15) + x_offset + tree_x, y_offset-(y_shrinked_right>>15), GREEN) ;
                sleep_us(10);
            }
            sleep_ms(80);
            x_offset += x_offset_increment;
            y_offset -= y_offset_increment;
            y_offset_increment = round(y_offset_increment * scale_factor);
            scale_current *= scale_factor;
        }
        tree_x += 320;
    }
    while (sync == false);
    sleep_ms(2000);
    sync = false;
    // here update all the random parameters

    tree_x = 160;
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
    } // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_fern);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  // initialize stio
  stdio_init_all() ;
  // initialize VGA
  initVGA() ;

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_fft);
  pt_add_thread(protothread_lsys);

  // start scheduler
  pt_schedule_start ;
} 
