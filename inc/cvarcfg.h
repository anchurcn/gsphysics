#pragma once
#include<cvardef.h>

extern cvar_t* phys_corpse;
extern cvar_t* phys_corpsestay;
extern cvar_t* phys_debugdraw;
extern cvar_t* phys_drawstatic;
extern cvar_t* phys_simurate;
extern cvar_t* phys_scale;
extern cvar_t* phys_gravity;
extern cvar_t* phys_dtest;
extern cvar_t* phys_jiggle;
// radius
extern cvar_t* phys_explode_r;
// intensity
extern cvar_t* phys_explode_i;

float G2BScale();
float B2GScale();
int cvarcfg_DebugDrawMode();
void cvarcfg_NewMap();