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
#include "vga_graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include <time.h>


////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// Stuff for Barnsley fern ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Fixed point data type
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)(((( signed long long)(a))*(( signed long long)(b)))>>15)) 
#define float2fix15(a) ((fix15)((a)*32768.0f)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0f) 
#define int2fix15(a) ((a)<<15)

// Maximum number of iterations
#define max_count 1000
fix15 point_x[max_count];
fix15 point_y[max_count];

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


////////////////////////////////////////////////////////////////////////////////////////////////////


int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // ===================================== Fern =======================================================
    /////////////////////////////////////////////////////////////////////////////////////////////////////

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
    int num_trees = 3;
    int tree_x = 160;
    while(true) {
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
            tree_x += 160;
        }

        sleep_ms(2000);
        fillRect(0,0,640,480,BLACK);
        tree_x = 160;
    }
}
