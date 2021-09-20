#include "cvarcfg.h"

cvar_t* phys_corpse;
cvar_t* phys_corpsestay;
cvar_t* phys_debugdraw;
cvar_t* phys_drawstatic;
cvar_t* phys_simurate;
cvar_t* phys_scale;
cvar_t* phys_gravity;
cvar_t* phys_dtest;
cvar_t* phys_jiggle;

cvar_t* phys_explode_r;
cvar_t* phys_explode_i;

float b2gscale = 0;
float g2bscale = 0;
float G2BScale()
{
	return g2bscale;
}

float B2GScale()
{
	return b2gscale;
}

int cvarcfg_DebugDrawMode()
{
	return (int)phys_debugdraw->value;
}

void cvarcfg_NewMap()
{
	g2bscale = phys_scale->value;
	b2gscale = 1 / phys_scale->value;
}
